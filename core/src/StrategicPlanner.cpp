#include <aigd/StrategicPlanner.hpp>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cmath>
#include <cstdio>

namespace aigd {

namespace {
// First-press delays tried (steps from the next one), and delays after a landing.
constexpr int kFirstDelays[] = {0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 14, 17, 20, 24, 28, 33, 38, 44, 50, 58, 66, 76, 88, 100, 115, 130};
constexpr int kAfterLanding[] = {0, 2, 4, 6, 9, 12, 16, 21, 27, 34, 42};
// The controller's own look-ahead: what "judging the first obstacles only" means.
constexpr int kLocalHorizon = 180;

std::string fmtStr(char const* f, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof(buf), f, ap);
    va_end(ap);
    return buf;
}

char const* landingWord(float space) {
    if (space < kBlock) return "<1-block";
    if (space < 2.f * kBlock) return "1-2-block";
    if (space < 4.f * kBlock) return "2-4-block";
    return "open";
}

std::string planText(Plan const& p) {
    if (p.pressAt < 0) return p.orbPressAt >= 0 ? fmtStr("orb press in %d%s", p.orbPressAt, p.orbHoldFor > 0 ? fmtStr(" held %d", p.orbHoldFor).c_str() : "") : "no press";
    std::string s = fmtStr("press in %d", p.pressAt);
    if (p.rejump) s += " (hold through the landing)";
    if (p.secondAt >= 0) s += fmtStr(", again %d after landing", p.secondAt);
    if (p.thirdAt >= 0) s += fmtStr(", again %d after the next landing", p.thirdAt);
    if (p.orbPressAt >= 0) s += p.orbHoldFor > 0 ? fmtStr(", orb press in %d held %d", p.orbPressAt, p.orbHoldFor) : fmtStr(", orb press in %d", p.orbPressAt);
    return s;
}
} // namespace

void StrategicPlanner::reset() {
    m_plan = StrategicPlan{};
    m_pressesSeen = 0;
    m_landingsSeen = 0;
    m_lastLandTick = 0;
    m_wasGrounded = true;
    m_holding = false;
}

void StrategicPlanner::invalidate(std::string reason) {
    if (m_plan.valid) {
        ++invalidations;
        lastInvalidation = std::move(reason);
    }
    m_plan.valid = false;
}

StrategicPlan const* StrategicPlanner::update(Observation const& obs, bool holding, Predictor const& predictor, std::span<Obstacle const> obstacles,
                                              std::span<PhysicsEvent const> events, StrategyLibrary const* library, std::string const& level) {
    bool const jumpMode = obs.mode == GameMode::Cube || obs.mode == GameMode::Robot;
    // execution tracking
    if (m_plan.valid) {
        if (holding && !m_holding) ++m_pressesSeen;
        if (obs.onGround && !m_wasGrounded) {
            ++m_landingsSeen;
            m_lastLandTick = obs.tick;
        }
    }
    m_holding = holding;
    m_wasGrounded = obs.onGround;

    if (!jumpMode || obstacles.empty()) {
        if (m_plan.valid) invalidate(jumpMode ? "nothing ahead" : "left cube/robot mode");
        return nullptr;
    }
    bool replan = !m_plan.valid;
    if (m_plan.valid) {
        StrategicPlan const& P = m_plan;
        int32_t const idx = obs.tick - P.madeTick - 1;
        float const h = situationHalfSize(obs.mini);
        if (obs.mode != P.mode || obs.speedClass() != P.speedClass || obs.upsideDown != P.upsideDown) {
            invalidate("mode, speed or gravity changed");
            replan = true;
        } else if (obs.geometryVersion != P.geometryVersion) {
            invalidate("the level geometry changed (a moving or toggled object) - replanning the sequence");
            replan = true;
        } else if (obs.x - h > P.toX + 2.f) {
            m_plan.valid = false; // sequence passed: the next one is planned
            replan = true;
        } else if (idx < 0 || idx >= static_cast<int32_t>(P.path.points.size())) {
            m_plan.valid = false; // the predicted path ended (or a collision was predicted there)
            replan = true;
        } else {
            PathPoint const& e = P.path.points[static_cast<size_t>(idx)];
            float const drift = predictor.model().driftPerTick(obs.mode, obs.mini);
            float const tol = 2.5f + 2.f * drift * static_cast<float>(idx);
            auto groundAt = [&](int32_t i) {
                return i >= 0 && i < static_cast<int32_t>(P.path.ground.size()) ? P.path.ground[static_cast<size_t>(i)] != 0 : obs.onGround;
            };
            bool const groundOk = groundAt(idx) == obs.onGround || groundAt(idx - 1) == obs.onGround || groundAt(idx + 1) == obs.onGround;
            if (std::fabs(e.x - obs.x) > 3.f || std::fabs(e.y - obs.y) > tol || !groundOk) {
                invalidate(fmtStr("state left the strategic plan (height %+.1f, %s, planned %s) - replanning the sequence from the actual state",
                                  obs.y - e.y, obs.onGround ? "on the ground" : "in the air", groundAt(idx) ? "on the ground" : "in the air"));
                replan = true;
            } else if (obs.tick - P.madeTick >= m_cfg.replanEvery && obs.onGround) {
                m_plan.valid = false; // refresh with what has been learned about the model since
                replan = true;
            }
        }
    }
    // Sequences are planned from the ground (where a jump starts); in the air
    // the current plan is only checked.
    if (replan && !obs.onGround) {
        m_plan.valid = false;
        return nullptr;
    }
    if (replan) {
        m_pressesSeen = 0;
        m_landingsSeen = 0;
        if (!makePlan(obs, holding, predictor, obstacles, events, library, level)) m_plan.valid = false;
    }
    return m_plan.valid ? &m_plan : nullptr;
}

bool StrategicPlanner::makePlan(Observation const& obs, bool holding, Predictor const& predictor, std::span<Obstacle const> obstacles,
                                std::span<PhysicsEvent const> events, StrategyLibrary const* library, std::string const& level) {
    auto const t0 = std::chrono::steady_clock::now();
    StrategicPlan P;
    P.madeTick = obs.tick;
    P.madeX = obs.x;
    P.madeY = obs.y;
    P.madeGrounded = obs.onGround;
    P.mode = obs.mode;
    P.speedClass = obs.speedClass();
    P.upsideDown = obs.upsideDown;
    P.geometryVersion = obs.geometryVersion;
    float const h = situationHalfSize(obs.mini);
    float const vx = predictor.model().vxPerTick(P.speedClass);

    // ---- the sequence: groups with little landing space between them ----
    Observation probe = obs;
    probe.obstacles = obstacles;
    probe.onGround = true;
    float const window = obstacles.empty() ? 0.f : obstacles.back().r - obs.x;
    Situation first;
    float prevEnd = 0.f;
    for (int gi = 0; gi < 6; ++gi) {
        Situation s = describeSituation(probe, std::max(10.f, (window - 2.f * kBlock) / vx));
        if (!s.valid) break;
        if (gi > 0 && (s.startX - prevEnd > m_cfg.joinLanding || s.endX - P.groupStart[0] > m_cfg.maxSequence)) break;
        if (gi == 0) first = s;
        P.groupStart[gi] = s.startX;
        P.groupEnd[gi] = s.endX;
        if (gi > 0) P.description += std::string(" -> ") + landingWord(s.startX - prevEnd) + " landing -> ";
        std::string d = s.describe();
        P.description += d.substr(0, d.find(','));
        P.groups = gi + 1;
        prevEnd = s.endX;
        probe.x = s.endX + 1.f - h;
    }
    if (P.groups == 0) return false; // nothing to plan for
    P.fromX = P.groupStart[0];
    P.toX = P.groupEnd[P.groups - 1];
    if ((P.fromX - (obs.x + h)) / vx > m_cfg.planAhead) return false; // nothing to decide yet
    P.horizon = std::clamp(static_cast<int>((P.toX + 3.f * kBlock - obs.x) / vx) + 20, predictor.horizon(), m_cfg.maxHorizon);

    SimStart st = predictor.startFrom(obs, holding);
    st.horizon = P.horizon;
    st.events = events;
    float const contact = (P.groupStart[0] - (obs.x + h)) / vx;

    PredictedPath tmp;
    int sims = 0;
    auto outOfBudget = [&]() {
        if (sims >= m_cfg.maxSimulations) return true;
        if (m_cfg.maxMicros <= 0.f || (sims & 7) != 0) return false;
        return std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - t0).count() > m_cfg.maxMicros;
    };
    std::vector<SequenceCandidate> all;
    all.reserve(96);

    auto evaluate = [&](Plan const& plan) -> SequenceCandidate {
        predictor.simulate(st, obstacles, plan, tmp);
        ++sims;
        SequenceCandidate c;
        c.plan = plan;
        c.outcome = tmp.outcome;
        c.collisionStep = tmp.outcome == Certainty::Collision ? tmp.collisionStep : -1;
        float const reached = tmp.outcome == Certainty::Collision ? tmp.collisionX : (tmp.points.empty() ? obs.x : tmp.points.back().x);
        for (int g = 0; g < P.groups; ++g) c.groupsCleared += P.groupEnd[g] + h < reached ? 1 : 0;
        // room in front of every landing: the next thing on the walking path
        c.landingRoom = 2.f * kBlock;
        for (int l = 0; l < tmp.landings; ++l) {
            float const lx = tmp.landingX[l];
            size_t const ps = static_cast<size_t>(std::max(0, tmp.landingStep[l]));
            float const ly = ps < tmp.points.size() ? tmp.points[ps].y : obs.y;
            for (auto const& o : obstacles) {
                if (o.kind == Obstacle::Kind::Uncertain || o.r <= lx + h) continue;
                bool const onPath = obs.upsideDown ? (o.t > ly - h + 2.f && o.b < ly + h - 1.f) : (o.b < ly + h - 2.f && o.t > ly - h + 1.f);
                if (!onPath) continue;
                c.landingRoom = std::min(c.landingRoom, std::max(0.f, o.l - (lx + h)));
                break;
            }
        }
        // open end: the collision comes after the landing that follows the last press
        if (c.outcome == Certainty::Collision && plan.pressAt >= 0) {
            int32_t lastPress = -1;
            uint8_t prevHold = st.holding ? 1 : 0;
            for (size_t i = 0; i < tmp.holds.size(); ++i) {
                if (tmp.holds[i] && !prevHold) lastPress = static_cast<int32_t>(i);
                prevHold = tmp.holds[i];
            }
            int32_t landing = -1;
            for (int l = 0; l < tmp.landings && landing < 0; ++l)
                if (lastPress >= 0 && tmp.landingStep[l] > lastPress) landing = tmp.landingStep[l];
            if (landing >= 0 && c.collisionStep > landing + 3) {
                c.openEnd = true;
                c.safeUntil = landing;
            }
        }
        if (c.openEnd) {
            c.score = 900.f + 40.f * static_cast<float>(c.groupsCleared) - 4.f * static_cast<float>(tmp.presses) +
                      std::min(c.landingRoom, 2.f * kBlock) / kBlock;
        } else if (c.outcome == Certainty::Collision) {
            c.score = -1000.f + static_cast<float>(c.collisionStep) + 40.f * static_cast<float>(c.groupsCleared);
        } else {
            float base = c.outcome == Certainty::Safe ? 1000.f : 500.f;
            c.score = base + std::min(tmp.clearance, 10.f) - 4.f * static_cast<float>(tmp.presses) + std::min(c.landingRoom, 2.f * kBlock) / kBlock;
        }
        return c;
    };
    // a press more after its last landing may handle what it runs into
    auto expandable = [&](SequenceCandidate const& c, int) { return c.openEnd; };

    // ---- depth 1: when to press first (or not at all) ----
    std::vector<int> firstDelays;
    firstDelays.push_back(-1);
    for (int k : kFirstDelays)
        if (k < P.horizon && static_cast<float>(k) <= std::max(contact, 0.f) + 8.f) firstDelays.push_back(k);
    int libraryDelay = -1;
    if (library && first.valid) {
        StrategyChoice c = library->choose(first, level);
        if (c.usable && c.primitive != Primitive::Wait && c.leadSteps >= 0.f) {
            libraryDelay = std::max(0, static_cast<int>(std::lround(contact - c.leadSteps)));
            if (std::find(firstDelays.begin(), firstDelays.end(), libraryDelay) == firstDelays.end()) firstDelays.push_back(libraryDelay);
        }
    }
    struct Partial {
        SequenceCandidate c;
        int depth;
    };
    std::vector<Partial> frontier;
    std::vector<SequenceCandidate> level1;
    for (int k : firstDelays) {
        if (outOfBudget()) break;
        Plan p{k, 0, 0, false};
        SequenceCandidate c = evaluate(p);
        bool const exp = expandable(c, 1);
        all.push_back(c);
        level1.push_back(c);
        if (exp && k >= 0) frontier.push_back({c, 1});
    }
    // ---- refinement: single steps around the most promising first presses ----
    {
        std::vector<SequenceCandidate> top = level1;
        std::sort(top.begin(), top.end(), [](SequenceCandidate const& a, SequenceCandidate const& b) { return a.score > b.score; });
        if (top.size() > 3) top.resize(3);
        for (auto const& t : top) {
            if (t.plan.pressAt < 0) continue;
            for (int d : {-2, -1, 1, 2}) {
                int const k = t.plan.pressAt + d;
                if (k < 0 || k >= P.horizon || outOfBudget()) continue;
                bool tried = false;
                for (auto const& c : level1) tried = tried || c.plan.pressAt == k;
                if (tried) continue;
                SequenceCandidate c = evaluate(Plan{k, 0, 0, false});
                bool const exp = expandable(c, 1);
                all.push_back(c);
                level1.push_back(c);
                if (exp) frontier.push_back({c, 1});
            }
        }
    }

    // ---- orbs: for the most promising first presses (and not pressing), an
    // orb their path passes through is activated at the start, middle or end
    // of the overlap (dash orbs: held for several durations). The result is a
    // candidate like any other, and is expanded further below. ----
    if (!st.orbs.empty()) {
        std::vector<SequenceCandidate> top = level1;
        std::sort(top.begin(), top.end(), [](SequenceCandidate const& a, SequenceCandidate const& b) { return a.score > b.score; });
        if (top.size() > 4) top.resize(4);
        static constexpr int32_t kDashHolds[] = {10, 22, 40, 64, 100, 140};
        PredictedPath base;
        for (auto const& t : top) {
            if (outOfBudget()) break;
            predictor.simulate(st, obstacles, t.plan, base);
            ++sims;
            for (auto const& ob : st.orbs) {
                if (ob.used || !orbSimulated(ob.type) || base.activates(ob.id)) continue;
                int32_t w0 = -1, w1 = -1;
                for (size_t i = 0; i < base.points.size(); ++i) {
                    PathPoint const& pt = base.points[i];
                    bool const over = ob.l <= pt.x + h && ob.r >= pt.x - h && ob.b <= pt.y + h && ob.t >= pt.y - h;
                    if (over && w0 < 0) w0 = static_cast<int32_t>(i);
                    if (over) w1 = static_cast<int32_t>(i);
                    if (!over && w0 >= 0) break;
                }
                if (w0 < 0) continue;
                int32_t const ks[3] = {w0, (w0 + w1) / 2, std::max(w0, w1 - 1)};
                for (int ki = 0; ki < 3; ++ki) {
                    if (ki > 0 && ks[ki] == ks[ki - 1]) continue;
                    int const nh = isDashOrb(ob.type) ? 6 : 1;
                    for (int hi = 0; hi < nh && !outOfBudget(); ++hi) {
                        Plan p = t.plan;
                        p.orbPressAt = ks[ki];
                        p.orbHoldFor = isDashOrb(ob.type) ? kDashHolds[hi] : 0;
                        SequenceCandidate c = evaluate(p);
                        bool const exp = expandable(c, 1);
                        all.push_back(c);
                        if (exp && p.pressAt >= 0) frontier.push_back({c, 1});
                    }
                }
                break; // the first orb on the path
            }
        }
    }

    // ---- depth 2 and 3: the next press, timed from the landing ----
    // (only while no safe sequence with fewer presses has been found: one more
    // press only costs)
    auto safeFound = [&]() {
        for (auto const& c : all)
            if (c.outcome == Certainty::Safe) return true;
        return false;
    };
    for (int depth = 1; depth <= 2 && !frontier.empty() && !safeFound(); ++depth) {
        std::sort(frontier.begin(), frontier.end(), [](Partial const& a, Partial const& b) {
            return a.c.groupsCleared != b.c.groupsCleared ? a.c.groupsCleared > b.c.groupsCleared : a.c.collisionStep > b.c.collisionStep;
        });
        if (static_cast<int>(frontier.size()) > m_cfg.beam) frontier.resize(static_cast<size_t>(m_cfg.beam));
        std::vector<Partial> next;
        for (auto const& f : frontier) {
            for (int d : kAfterLanding) {
                if (outOfBudget()) break;
                Plan p = f.c.plan;
                (depth == 1 ? p.secondAt : p.thirdAt) = d;
                SequenceCandidate c = evaluate(p);
                bool const exp = expandable(c, depth + 1);
                all.push_back(c);
                if (exp && depth == 1) next.push_back({c, 2});
            }
        }
        frontier = std::move(next);
    }

    // ---- robustness: a timing whose neighbours also work is preferred ----
    for (auto& c : all) {
        if (c.outcome == Certainty::Collision) continue;
        int neighbours = 0;
        for (auto const& o : all) {
            if (&o == &c || o.outcome == Certainty::Collision) continue;
            bool const sameShape = (o.plan.secondAt < 0) == (c.plan.secondAt < 0) && (o.plan.thirdAt < 0) == (c.plan.thirdAt < 0) &&
                                   (o.plan.pressAt < 0) == (c.plan.pressAt < 0);
            if (sameShape && std::abs(o.plan.pressAt - c.plan.pressAt) <= 2 && std::abs(o.plan.secondAt - c.plan.secondAt) <= 3) ++neighbours;
        }
        c.score += 0.5f * static_cast<float>(std::min(neighbours, 2));
    }
    auto best = std::max_element(all.begin(), all.end(), [](SequenceCandidate const& a, SequenceCandidate const& b) { return a.score < b.score; });
    if (best == all.end()) return false;
    P.chosen = *best;
    P.fromLibrary = libraryDelay >= 0 && P.chosen.plan.pressAt == libraryDelay;

    // ---- what judging only the next few obstacles would have chosen ----
    {
        SequenceCandidate local;
        float localScore = -1e9f;
        for (auto const& c : level1) {
            // within the controller's own look-ahead a later collision is invisible
            bool const safeLocally = c.outcome != Certainty::Collision || c.collisionStep >= kLocalHorizon;
            float const s = safeLocally ? 1000.f - 4.f * (c.plan.pressAt >= 0 ? 1.f : 0.f) - 0.01f * static_cast<float>(std::max(0, c.plan.pressAt))
                                        : -1000.f + static_cast<float>(c.collisionStep);
            if (s > localScore) {
                localScore = s;
                local = c;
            }
        }
        P.localBest = local;
        P.overridesLocal = local.plan.pressAt != P.chosen.plan.pressAt && std::abs(local.plan.pressAt - P.chosen.plan.pressAt) > 2 &&
                           local.score < P.chosen.score && local.outcome == Certainty::Collision;
        if (P.overridesLocal) {
            P.reason = fmtStr("%s clears the first obstacle but collides %d steps ahead (%d of %d groups handled); %s handles %d of %d",
                              planText(local.plan).c_str(), local.collisionStep + 1, local.groupsCleared, P.groups,
                              planText(P.chosen.plan).c_str(), P.chosen.outcome == Certainty::Collision ? P.chosen.groupsCleared : P.groups,
                              P.groups);
        } else {
            P.reason = fmtStr("%s (%s over %d group%s)", planText(P.chosen.plan).c_str(), certaintyName(P.chosen.outcome), P.groups,
                              P.groups == 1 ? "" : "s");
        }
    }

    // ---- alternatives: the best other first-press timings ----
    std::vector<SequenceCandidate> sorted = all;
    std::sort(sorted.begin(), sorted.end(), [](SequenceCandidate const& a, SequenceCandidate const& b) { return a.score > b.score; });
    for (auto const& c : sorted) {
        if (P.alternatives.size() >= 3) break;
        if (std::abs(c.plan.pressAt - P.chosen.plan.pressAt) <= 2) continue;
        bool dup = false;
        for (auto const& a : P.alternatives) dup = dup || std::abs(a.plan.pressAt - c.plan.pressAt) <= 2;
        if (!dup) P.alternatives.push_back(c);
    }

    // ---- the chosen path, for following / checking / drawing ----
    predictor.simulate(st, obstacles, P.chosen.plan, P.path);
    ++sims;
    uint8_t prev = st.holding ? 1 : 0;
    for (size_t i = 0; i < P.path.holds.size() && i < P.path.points.size(); ++i) {
        if (P.path.holds[i] && !prev) P.pressX.push_back(P.path.points[i].x);
        prev = P.path.holds[i];
    }
    for (int l = 0; l < P.path.landings; ++l) P.landingX.push_back(P.path.landingX[l]);

    P.effectiveHorizon = P.chosen.openEnd ? std::min(P.chosen.collisionStep - 1, P.chosen.safeUntil + 20) : P.horizon;
    P.simulations = sims;
    P.micros = std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - t0).count();
    P.valid = true;
    ++plansMade;
    if (P.overridesLocal) ++overrides;
    totalMicros += P.micros;
    maxMicros = std::max(maxMicros, P.micros);
    m_plan = std::move(P);
    return true;
}

void StrategicPlanner::adoptController(Observation const& obs, PredictedPath const& path) {
    if (!m_plan.valid || path.points.empty()) return;
    StrategicPlan& P = m_plan;
    P.madeTick = obs.tick;
    P.madeX = obs.x;
    P.madeY = obs.y;
    P.madeGrounded = obs.onGround;
    P.horizon = std::max(P.horizon - 0, path.steps);
    P.effectiveHorizon = path.steps;
    P.chosen.openEnd = false;
    P.path = path;
    P.chosen.plan = path.plan;
    P.chosen.outcome = path.outcome;
    P.chosen.collisionStep = path.outcome == Certainty::Collision ? path.collisionStep : -1;
    P.pressX.clear();
    P.landingX.clear();
    uint8_t prev = m_holding ? 1 : 0;
    for (size_t i = 0; i < path.holds.size() && i < path.points.size(); ++i) {
        if (path.holds[i] && !prev) P.pressX.push_back(path.points[i].x);
        prev = path.holds[i];
    }
    for (int l = 0; l < path.landings; ++l) P.landingX.push_back(path.landingX[l]);
    P.reason = "refined by the controller: " + planText(path.plan) + " (" + certaintyName(path.outcome) + ")";
    m_pressesSeen = 0;
    m_landingsSeen = 0;
    ++adopted;
}

bool StrategicPlanner::seed(Observation const& obs, Plan& out, int& horizon) const {
    if (!m_plan.valid) return false;
    StrategicPlan const& P = m_plan;
    int32_t const elapsed = obs.tick - P.madeTick; // plan step of the next physics step
    Plan const& c = P.chosen.plan;
    horizon = std::max(0, P.effectiveHorizon - elapsed);
    // press steps and the landings after them, from the predicted path
    int32_t pressSteps[3] = {-1, -1, -1};
    int np = 0;
    uint8_t prev = 0;
    for (size_t i = 0; i < P.path.holds.size() && np < 3; ++i) {
        if (P.path.holds[i] && !prev) pressSteps[np++] = static_cast<int32_t>(i);
        prev = P.path.holds[i];
    }
    // nothing to propose: no press planned, or no sequence that is safe at
    // least up to its last landing (the controller's own search does better
    // without a doomed candidate)
    if (!(P.chosen.outcome == Certainty::Safe || P.chosen.openEnd)) return false;
    // an orb press is timed from when the plan was made (it acts where the orb is)
    int32_t const orbRel = c.orbPressAt >= 0 && c.orbPressAt - elapsed >= 0 ? c.orbPressAt - elapsed : -1;
    if (c.pressAt < 0) {
        if (orbRel < 0) return false;
        out = Plan{-1, 0, 0, false};
        out.orbPressAt = orbRel;
        out.orbHoldFor = c.orbHoldFor;
        return true;
    }
    if (m_pressesSeen == 0) {
        int32_t const rel = c.pressAt - elapsed;
        if (rel < 0) return false;
        out = c;
        out.pressAt = rel;
        out.orbPressAt = orbRel;
        return true;
    }
    // later presses: timed from the landing after the previous press
    int const nextIdx = m_pressesSeen; // 1 -> second press, 2 -> third press
    int32_t const after = nextIdx == 1 ? c.secondAt : nextIdx == 2 ? c.thirdAt : -1;
    if (after < 0) return false;
    int32_t landTick;
    if (m_landingsSeen >= nextIdx) {
        landTick = m_lastLandTick;
    } else {
        // not landed yet: the predicted landing after that press
        int32_t landStep = -1;
        for (int l = 0; l < P.path.landings; ++l)
            if (pressSteps[nextIdx - 1] >= 0 && P.path.landingStep[l] > pressSteps[nextIdx - 1]) {
                landStep = P.path.landingStep[l];
                break;
            }
        if (landStep < 0) return false;
        landTick = P.madeTick + 1 + landStep;
    }
    int32_t const rel = landTick + after - obs.tick;
    if (rel < 0) return false;
    out = Plan{rel, 0, 0, false};
    out.secondAt = nextIdx == 1 ? c.thirdAt : -1;
    out.orbPressAt = orbRel;
    out.orbHoldFor = c.orbHoldFor;
    return true;
}

} // namespace aigd
