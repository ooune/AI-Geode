#include <aigd/Predictor.hpp>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>

namespace aigd {

namespace {
constexpr float kNear = 40.f;            // obstacles further than this from the player are ignored per step
constexpr float kLandTolerance = 2.f;    // how far into a surface a player may sink and still be put on it
constexpr float kEstimatedBoundZone = 10.f; // touching an unconfirmed floor/ceiling makes a path uncertain
constexpr int kCoarseDelays[] = {1, 2, 4, 6, 9, 12, 16, 20, 25, 31, 38, 46, 55, 65, 76, 88, 100, 115, 130, 150, 170};

float overlap(float a0, float a1, float b0, float b1) { return std::min(a1, b1) - std::max(a0, b0); }

bool continuousControl(GameMode m) { return m == GameMode::Ship || m == GameMode::Wave; }
bool tapControl(GameMode m) { return m == GameMode::Ufo || m == GameMode::Swing; }
// Modes where the button only acts while touching the ground, so in the air
// nothing changes until shortly before landing.
bool groundOnly(GameMode m) { return m == GameMode::Cube || m == GameMode::Ball || m == GameMode::Spider; }
} // namespace

char const* certaintyName(Certainty c) {
    switch (c) {
        case Certainty::Safe: return "safe";
        case Certainty::Uncertain: return "uncertain";
        case Certainty::Collision: return "collision";
        default: return "?";
    }
}

Predictor::Predictor(PhysicsModel& model) : m_model(model) {
    for (auto* p : {&m_press, &m_release, &m_tmp, &m_resim}) {
        p->points.reserve(512);
        p->holds.reserve(512);
    }
    m_holdBuf.reserve(kHistory);
}

SimStart Predictor::startFrom(Observation const& obs, bool holding) const {
    SimStart s;
    s.x = obs.x;
    s.y = obs.y;
    s.vy = obs.vy;
    s.mode = obs.mode;
    s.onGround = obs.onGround;
    s.upsideDown = obs.upsideDown;
    s.mini = obs.mini;
    s.holding = holding;
    s.speedClass = obs.speedClass();
    s.tick = obs.tick;
    s.bounds = m_bounds;
    if (obs.mode == GameMode::Robot && m_robotAlive)
        s.robotBoostLeft = std::max(0, m_model.params(obs.mode, obs.mini).boostSteps - m_robotSince);
    return s;
}

bool Predictor::trusted(GameMode mode, bool mini) const {
    return m_model.calibrated(mode, mini) && m_model.driftPerTick(mode) < 0.15f && m_caution < 3.f;
}

float Predictor::margin(int step, GameMode mode, bool mini) const {
    float base = trusted(mode, mini) ? 0.5f : 3.f;
    return std::min(12.f, (base + m_model.driftPerTick(mode) * static_cast<float>(step)) * m_caution);
}

void Predictor::simulate(SimStart const& s, std::span<Obstacle const> obstacles, Plan const& plan, PredictedPath& out,
                         std::span<uint8_t const> holds) const {
    ModePhysics const& P = m_model.params(s.mode, s.mini);
    float const vx = m_model.vxPerTick(s.speedClass);
    float const h = P.halfSize;
    float const in = P.innerHalf;
    int const steps = holds.empty() ? m_horizon : static_cast<int>(holds.size());
    Bounds const& B = s.bounds;
    float const floorC = B.floor + h;  // player centre when resting on the floor
    float const ceilC = B.ceil - h;    // ... against the ceiling

    out.points.clear();
    out.holds.clear();
    out.plan = plan;
    out.outcome = Certainty::Safe;
    out.collisionStep = -1;
    out.obstacle = -1;
    out.uncertainFrom = -1;
    out.landed = false;
    out.clearance = 1e9f;
    out.steps = 0;

    float maxW = 0.f;
    for (auto const& o : obstacles) maxW = std::max(maxW, o.r - o.l);

    float x = s.x;
    float y = s.y;
    float vy = s.vy;
    float g = s.upsideDown ? -1.f : 1.f; // gravity direction: +1 pulls down
    bool onGround = s.onGround;
    bool prevHold = s.holding;

    bool pressActive = false;
    int32_t pressStart = 0;
    int32_t nextPress = plan.pressAt;
    int effects = 0;
    int const target = plan.rejump ? 2 : 1;
    int32_t sinceTakeoff = 1 << 20;
    bool boostAlive = false; // robot: letting go ends the boost for this jump
    if (s.robotBoostLeft > 0) {
        boostAlive = true;
        sinceTakeoff = P.boostSteps - s.robotBoostLeft;
    }
    // edge-triggered modes: a plain press is a short tap
    bool const tapRelease = P.press == PressAction::ImpulseOnEdge || P.press == PressAction::FlipOnEdge ||
                            P.press == PressAction::TeleportOnGround;

    bool potential = false;
    bool definite = false;
    float const m0 = (trusted(s.mode, s.mini) ? 0.5f : 3.f) * m_caution;
    float const dm = m_model.driftPerTick(s.mode) * m_caution;
    size_t first = 0;
    size_t const nObs = obstacles.size();

    auto recordHit = [&](int step, int idx, bool isDefinite) {
        if (isDefinite) definite = true;
        else if (potential) return;
        else potential = true;
        out.collisionStep = step;
        out.collisionX = x;
        out.collisionY = y;
        out.obstacle = idx;
        out.hit = obstacles[static_cast<size_t>(idx)];
    };
    auto boundContact = [&](int step, bool estimated) {
        if (estimated && out.uncertainFrom < 0) {
            potential = true;
            out.uncertainFrom = step;
        }
    };

    for (int step = 0; step < steps; ++step) {
        bool hold;
        if (!holds.empty()) {
            hold = holds[static_cast<size_t>(step)] != 0;
        } else {
            if (nextPress >= 0 && step == nextPress) {
                pressActive = true;
                pressStart = step;
                effects = 0;
                nextPress = plan.repeatEvery > 0 ? step + plan.repeatEvery : -1;
            }
            if (pressActive) {
                int32_t heldFor = step - pressStart;
                if (plan.holdFor > 0 && heldFor >= plan.holdFor) pressActive = false;
                else if (plan.holdFor == 0 && tapRelease && heldFor >= 2) pressActive = false;
                else if (plan.holdFor == 0 && !tapRelease && effects >= target && heldFor >= 2) pressActive = false;
            }
            hold = pressActive;
        }
        bool const edge = hold && !prevHold;
        prevHold = hold;
        out.holds.push_back(hold ? 1 : 0);

        float const prevBottom = y - h;
        float const prevTop = y + h;
        float const px = x;
        float const py = y;
        bool const wasAir = !onGround;
        float vyN = vy * g;
        bool effect = false;
        bool teleported = false;
        bool stop = false;

        switch (P.press) {
            case PressAction::JumpOnGround:
                if (onGround && hold) {
                    vyN = P.impulse;
                    onGround = false;
                    effect = true;
                    sinceTakeoff = 0;
                    boostAlive = true;
                } else if (!onGround) {
                    if (!hold) boostAlive = false;
                    bool boost = s.mode == GameMode::Robot && hold && boostAlive && sinceTakeoff < P.boostSteps;
                    vyN += boost ? P.accHold : P.accRelease;
                }
                break;
            case PressAction::ImpulseOnEdge:
                if (edge) {
                    vyN = P.impulse;
                    onGround = false;
                    effect = true;
                } else if (!onGround) {
                    vyN += P.accRelease;
                }
                break;
            case PressAction::FlipOnGround:
            case PressAction::FlipOnEdge: {
                bool trigger = P.press == PressAction::FlipOnEdge ? edge : (onGround && hold);
                if (trigger) {
                    g = -g;
                    vyN = P.flipVy;
                    onGround = false;
                    effect = true;
                } else if (!onGround) {
                    vyN += P.accRelease;
                }
                break;
            }
            case PressAction::TeleportOnGround:
                if (onGround && edge) {
                    // the nearest surface on the other side (blocks or the playing-area bound)
                    bool found = false;
                    bool estimated = false;
                    float dest = 0.f;
                    for (size_t j = 0; j < nObs; ++j) {
                        Obstacle const& o = obstacles[j];
                        if (o.kind != Obstacle::Kind::Solid || o.l >= x + h || o.r <= x - h) continue;
                        if (g > 0.f && o.b >= y + h - 1.f) {
                            float c = o.b - h;
                            if (!found || c < dest) dest = c, found = true;
                        } else if (g < 0.f && o.t <= y - h + 1.f) {
                            float c = o.t + h;
                            if (!found || c > dest) dest = c, found = true;
                        }
                    }
                    if (g > 0.f && B.hasCeil && (!found || ceilC < dest)) dest = ceilC, found = true, estimated = B.ceilEstimated;
                    if (g < 0.f && B.hasFloor && (!found || floorC > dest)) dest = floorC, found = true, estimated = B.floorEstimated;
                    if (!found) {
                        out.uncertainFrom = step;
                        stop = true;
                        break;
                    }
                    y = dest;
                    g = -g;
                    vyN = 0.f;
                    onGround = true;
                    effect = true;
                    teleported = true;
                    boundContact(step, estimated);
                } else if (!onGround) {
                    vyN += P.accRelease;
                }
                break;
            case PressAction::None:
                vyN += hold ? P.accHold : P.accRelease;
                if (vyN > 0.f) onGround = false;
                break;
            case PressAction::SetVelocity:
                vyN = (hold ? 1.f : -1.f) * P.waveVy * vx / P.yScale;
                if (vyN > 0.f) onGround = false;
                break;
        }
        if (stop) {
            out.steps = step;
            break;
        }
        if (effect) ++effects;
        if (!effect) ++sinceTakeoff;
        if (P.press != PressAction::SetVelocity) vyN = std::clamp(vyN, -P.maxDown, P.maxUp);
        vy = vyN * g;
        if (!teleported) y += vy * P.yScale;
        x += vx;

        bool supported = false;
        // playing-area bounds (GD's ground / ceiling)
        if (B.hasFloor && y <= floorC) {
            y = floorC;
            if (vy < 0.f) vy = 0.f;
            if (g > 0.f) {
                onGround = true;
                supported = true;
            }
            boundContact(step, B.floorEstimated);
        }
        if (B.hasCeil && y >= ceilC) {
            y = ceilC;
            if (vy > 0.f) vy = 0.f;
            if (g < 0.f) {
                onGround = true;
                supported = true;
            }
            boundContact(step, B.ceilEstimated);
        }

        float const m = std::min(12.f, m0 + dm * static_cast<float>(step));
        float const sweep = 0.5f * std::max(std::fabs(x - px), std::fabs(y - py));
        float const lo = x - h - kNear - maxW;
        while (first < nObs && obstacles[first].l < lo) ++first;
        for (size_t idxu = first; idxu < nObs && !stop; ++idxu) {
            Obstacle const& o = obstacles[idxu];
            if (o.l > x + h + kNear) break;
            if (o.r < x - h - kNear) continue;
            int const idx = static_cast<int>(idxu);
            float const ox = overlap(x - h, x + h, o.l, o.r);
            float const oy = overlap(y - h, y + h, o.b, o.t);

            switch (o.kind) {
                case Obstacle::Kind::Uncertain:
                    if (ox > 0.f && oy > 0.f) {
                        out.uncertainFrom = step;
                        stop = true;
                    }
                    break;
                case Obstacle::Kind::Hazard: {
                    float sep = (ox > 0.f && oy > 0.f) ? -std::min(ox, oy) : std::max(-ox, -oy);
                    out.clearance = std::min(out.clearance, sep);
                    if (ox > m && oy > m) {
                        recordHit(step, idx, true);
                        stop = true;
                    } else if (ox > -(m + sweep) && oy > -(m + sweep)) {
                        recordHit(step, idx, false);
                    }
                    break;
                }
                case Obstacle::Kind::Solid: {
                    if (ox <= 0.f) break;
                    float const ix = overlap(x - in, x + in, o.l, o.r);
                    float const iy = overlap(y - in, y + in, o.b, o.t);
                    // how close the inner hitbox came to the block (a side-on graze is risky)
                    out.clearance = std::min(out.clearance, std::max(-ix, -iy));
                    // Reaching a top or bottom surface: stand on it / slide along it,
                    // unless the small inner hitbox is already inside the block.
                    bool fromAbove = vy <= 0.f && prevBottom >= o.t - kLandTolerance && y - h < o.t;
                    bool fromBelow = vy >= 0.f && prevTop <= o.b + kLandTolerance && y + h > o.b;
                    if (fromAbove || fromBelow) {
                        if (ix > m && iy > m) {
                            recordHit(step, idx, true);
                            stop = true;
                            break;
                        }
                        y = fromAbove ? o.t + h : o.b - h;
                        vy = 0.f;
                        if ((fromAbove && g > 0.f) || (fromBelow && g < 0.f)) {
                            onGround = true;
                            supported = true;
                        }
                        break;
                    }
                    if (onGround && ((g > 0.f && std::fabs((y - h) - o.t) < 0.5f) || (g < 0.f && std::fabs((y + h) - o.b) < 0.5f)))
                        supported = true;
                    if (ix > m && iy > m) {
                        recordHit(step, idx, true);
                        stop = true;
                    } else if (oy > 0.f && ix > -(m + sweep) && iy > -(m + sweep)) {
                        recordHit(step, idx, false);
                    }
                    break;
                }
            }
        }

        out.points.push_back({x, y});
        out.steps = step + 1;
        if (definite || stop) break;

        // Walking off an edge: nothing under us (in the gravity direction) any more.
        if (onGround && !supported) {
            bool sup = (g > 0.f && B.hasFloor && std::fabs(y - floorC) < 0.5f) || (g < 0.f && B.hasCeil && std::fabs(y - ceilC) < 0.5f);
            for (size_t j = first; j < nObs && !sup; ++j) {
                Obstacle const& o = obstacles[j];
                if (o.l > x + h) break;
                if (o.kind != Obstacle::Kind::Solid || o.r <= x - h) continue;
                if (g > 0.f ? std::fabs((y - h) - o.t) < 0.5f : std::fabs((y + h) - o.b) < 0.5f) sup = true;
            }
            if (!sup) onGround = false;
        }
        if (wasAir && onGround && !out.landed) {
            out.landed = true;
            out.landX = x;
            out.landY = y;
        }
    }

    if (definite) out.outcome = Certainty::Collision;
    else if (potential || out.uncertainFrom >= 0) out.outcome = Certainty::Uncertain;
    else out.outcome = Certainty::Safe;
}

float Predictor::score(PredictedPath const& p) const {
    if (p.outcome == Certainty::Collision) return -1000.f + static_cast<float>(p.collisionStep);
    float base = p.outcome == Certainty::Safe ? 1000.f : 500.f;
    if (p.outcome == Certainty::Uncertain) {
        int32_t when = p.collisionStep >= 0 ? p.collisionStep : p.uncertainFrom;
        if (when >= 0) base += static_cast<float>(when) * 0.5f;
    }
    return base + std::min(p.clearance, 30.f);
}

bool Predictor::sameState(Observation const& obs, bool holding) const {
    return obs.tick == m_adviceTick && holding == m_adviceHolding && obs.x == m_stateX && obs.y == m_stateY && obs.vy == m_stateVy &&
           obs.mode == m_computedMode && obs.onGround == m_stateGround && obs.upsideDown == m_stateFlipped;
}

bool Predictor::offPredictedPath(Observation const& obs) const {
    // Where the executed branch said the player would be now.
    PredictedPath const& p = executedPath();
    int32_t i = obs.tick - m_computedTick - 1;
    if (!m_executedValid || i < 0 || i >= static_cast<int32_t>(p.points.size())) return true;
    return std::fabs(p.points[static_cast<size_t>(i)].x - obs.x) > 1.f || std::fabs(p.points[static_cast<size_t>(i)].y - obs.y) > 0.5f;
}

void Predictor::evaluate(SimStart const& s, std::span<Obstacle const> obs, Plan const& plan, PredictedPath& tmp,
                         PredictedPath& best, bool& haveBest, int branch) {
    simulate(s, obs, plan, tmp);
    ++m_advice.simulations;
    ++m_totalSims;
    if (tmp.outcome != Certainty::Collision) {
        // How long this plan keeps the branch's first button state.
        int32_t keep = static_cast<int32_t>(tmp.holds.size());
        for (size_t i = 1; i < tmp.holds.size(); ++i) {
            if (tmp.holds[i] != tmp.holds[0]) {
                keep = static_cast<int32_t>(i);
                break;
            }
        }
        int32_t& k = branch ? m_advice.pressKeepFor : m_advice.releaseKeepFor;
        k = std::max(k, keep);
        if (tmp.outcome == Certainty::Safe) {
            int32_t& ks = branch ? m_advice.pressKeepForSafe : m_advice.releaseKeepForSafe;
            ks = std::max(ks, keep);
        }
    }
    if (!haveBest || score(tmp) > score(best)) {
        std::swap(tmp, best);
        haveBest = true;
    }
}

Predictor::Advice const& Predictor::advise(Observation const& obs, bool holding, bool allowReuse) {
    // The same state again in the same tick: nothing can have changed.
    if (sameState(obs, holding) && (allowReuse || m_computedTick == obs.tick)) return m_advice;
    // Reuse the last plan where re-planning cannot change anything (or every
    // other step for continuously controlled modes) - but never while a
    // collision is near, and never once the player is not where the executed
    // plan said it would be.
    bool reuse = false;
    bool const imminent = !m_advice.bestPresses && m_advice.bestPressAt >= 0 && m_advice.bestPressAt <= 2 + (obs.tick - m_computedTick);
    int const ex = m_executedPress ? 1 : 0;
    // (16 steps: several times the time an input needs to take effect)
    bool const danger = m_advice.outcome(ex) != Certainty::Safe || m_advice.keepForSafe(ex) < 16;
    if (allowReuse && m_advice.valid && obs.tick > m_adviceTick && holding == m_adviceHolding && obs.mode == m_computedMode && !imminent &&
        !danger && holding == m_executedPress && !offPredictedPath(obs)) {
        if (groundOnly(obs.mode) && !holding && !obs.onGround && obs.tick <= m_reuseUntilTick && !m_advice.bestPresses) reuse = true;
        if ((continuousControl(obs.mode) || tapControl(obs.mode)) && obs.tick - m_computedTick < 2) reuse = true;
    }
    if (reuse) {
        m_adviceTick = obs.tick;
        m_stateX = obs.x;
        m_stateY = obs.y;
        m_stateVy = obs.vy;
        m_stateGround = obs.onGround;
        m_stateFlipped = obs.upsideDown;
        m_advice.reused = true;
        m_advice.simulations = 0;
        m_advice.micros = 0.f;
        ++m_reused;
        return m_advice;
    }
    m_reuseUntilTick = INT32_MIN;
    auto const t0 = std::chrono::steady_clock::now();
    m_advice = Advice{};
    m_adviceTick = obs.tick;
    m_computedTick = obs.tick;
    m_computedMode = obs.mode;
    m_adviceHolding = holding;
    m_advice.computedTick = obs.tick;
    m_stateX = obs.x;
    m_stateY = obs.y;
    m_stateVy = obs.vy;
    m_stateGround = obs.onGround;
    m_stateFlipped = obs.upsideDown;
    m_executedValid = false;
    m_keep = PredictedPath{};
    m_press = PredictedPath{};
    m_release = PredictedPath{};
    if (!supports(obs.mode) || obs.obstacles.empty()) return m_advice;

    SimStart s = startFrom(obs, holding);
    int const savedHorizon = m_horizon;
    // flying modes: a shorter look-ahead (they re-plan every couple of steps anyway)
    if (continuousControl(s.mode) || tapControl(s.mode)) m_horizon = std::min(m_horizon, 120);

    float const reach = s.x + static_cast<float>(m_horizon) * m_model.vxPerTick(s.speedClass) + kNear;
    bool relevant = false;
    for (auto const& o : obs.obstacles) {
        if (o.kind == Obstacle::Kind::Uncertain) continue;
        if (o.l < reach && o.r > s.x - 20.f) {
            relevant = true;
            break;
        }
    }
    if (!relevant) {
        m_horizon = savedHorizon;
        return m_advice;
    }
    m_advice.valid = true;
    m_advice.trusted = trusted(s.mode, s.mini);

    auto const& span = obs.obstacles;
    bool havePress = false, haveRelease = false;
    auto press = [&](Plan p) { evaluate(s, span, p, m_tmp, m_press, havePress, 1); };

    // ---- branch 1: button down this step ----
    switch (s.mode) {
        case GameMode::Cube:
            press({0, 0, 0, false});
            press({0, 0, 0, true});
            break;
        case GameMode::Robot:
            for (int d : {0, 12, 30, 60}) press({0, d, 0, false});
            break;
        case GameMode::Ship:
        case GameMode::Wave:
            for (int d : {2, 6, 12, 24, 48, -1}) press({0, d, 0, false});
            for (auto [hh, r] : {std::pair{4, 10}, {6, 12}, {10, 20}}) press({0, hh, r, false});
            break;
        case GameMode::Ufo:
            for (int r : {0, 18, 30, 45}) press({0, 0, r, false});
            break;
        case GameMode::Swing:
            for (int r : {0, 25, 45}) press({0, 0, r, false});
            break;
        default: // ball, spider
            press({0, 0, 0, false});
            break;
    }

    // ---- branch 2: button up this step; never press, or press k steps later ----
    evaluate(s, span, Plan{-1, 0, 0, false}, m_tmp, m_release, haveRelease, 0);
    // Pressing at step k behaves exactly like never pressing up to step k: if
    // "never" already collides by then, so does every later k (no need to simulate).
    int32_t const neverHit = m_release.outcome == Certainty::Collision ? m_release.collisionStep : INT32_MAX;

    // continuation shapes tried for each delay
    std::array<Plan, 3> shapes{};
    size_t nShapes = 0;
    switch (s.mode) {
        case GameMode::Robot: shapes[nShapes++] = {0, 0, 0, false}; shapes[nShapes++] = {0, 30, 0, false}; break;
        case GameMode::Ship:
        case GameMode::Wave: shapes[nShapes++] = {0, -1, 0, false}; shapes[nShapes++] = {0, 8, 0, false}; break;
        case GameMode::Ufo: shapes[nShapes++] = {0, 0, 0, false}; shapes[nShapes++] = {0, 0, 24, false}; break;
        case GameMode::Swing: shapes[nShapes++] = {0, 0, 0, false}; shapes[nShapes++] = {0, 0, 30, false}; break;
        default: shapes[nShapes++] = {0, 0, 0, false}; break;
    }

    struct Tried {
        int k;
        size_t shape;
        Certainty outcome;
    };
    std::array<Tried, 64> tried{};
    size_t nTried = 0;
    for (int k : kCoarseDelays) {
        if (k >= m_horizon) break;
        for (size_t si = 0; si < nShapes; ++si) {
            // second shapes only for the nearer delays (where they matter most)
            if (si > 0 && k > 40) continue;
            Plan p = shapes[si];
            p.pressAt = k;
            Certainty out;
            if (k > neverHit) {
                out = Certainty::Collision;
            } else {
                evaluate(s, span, p, m_tmp, m_release, haveRelease, 0);
                bool kept = m_release.plan.pressAt == k && m_release.plan.holdFor == p.holdFor && m_release.plan.repeatEvery == p.repeatEvery;
                out = kept ? m_release.outcome : m_tmp.outcome;
            }
            if (nTried < tried.size()) tried[nTried++] = {k, si, out};
        }
    }
    // Refine: single-step resolution around the best delay and wherever the
    // outcome changes between neighbouring coarse delays.
    std::array<std::pair<int, size_t>, 16> refine{};
    size_t nRefine = 0;
    auto addRefine = [&](int k, size_t si) {
        if (k < 1 || k >= m_horizon || k > neverHit || nRefine >= refine.size()) return;
        for (size_t i = 0; i < nTried; ++i) if (tried[i].k == k && tried[i].shape == si) return;
        for (size_t i = 0; i < nRefine; ++i) if (refine[i].first == k && refine[i].second == si) return;
        refine[nRefine++] = {k, si};
    };
    if (m_release.plan.pressAt > 0) {
        size_t si = 0;
        for (size_t i = 0; i < nShapes; ++i)
            if (shapes[i].holdFor == m_release.plan.holdFor && shapes[i].repeatEvery == m_release.plan.repeatEvery) si = i;
        for (int d : {-2, -1, 1, 2}) addRefine(m_release.plan.pressAt + d, si);
    }
    for (size_t i = 1; i < nTried; ++i) {
        for (size_t j = i; j-- > 0;) {
            if (tried[j].shape != tried[i].shape) continue;
            if (tried[j].outcome != tried[i].outcome) addRefine((tried[i].k + tried[j].k) / 2, tried[i].shape);
            break;
        }
    }
    for (size_t i = 0; i < nRefine; ++i) {
        Plan p = shapes[refine[i].second];
        p.pressAt = refine[i].first;
        evaluate(s, span, p, m_tmp, m_release, haveRelease, 0);
    }
    if ((s.mode == GameMode::Cube || s.mode == GameMode::Robot) && m_release.plan.pressAt > 0) {
        Plan p = m_release.plan;
        p.rejump = true;
        evaluate(s, span, p, m_tmp, m_release, haveRelease, 0);
    }

    m_advice.press = m_press.outcome;
    m_advice.release = m_release.outcome;
    m_advice.pressTTC = m_press.outcome == Certainty::Collision ? m_press.collisionStep : -1;
    m_advice.releaseTTC = m_release.outcome == Certainty::Collision ? m_release.collisionStep : -1;
    m_advice.pressRisk = m_press.outcome != Certainty::Safe ? m_press.collisionStep : -1;
    m_advice.releaseRisk = m_release.outcome != Certainty::Safe ? m_release.collisionStep : -1;

    // What happens if nobody touches the button: the current state kept.
    simulate(s, span, holding ? Plan{0, -1, 0, false} : Plan{-1, 0, 0, false}, m_keep);
    ++m_advice.simulations;
    ++m_totalSims;
    m_advice.keep = m_keep.outcome;
    m_advice.keepTTC = m_keep.outcome == Certainty::Collision ? m_keep.collisionStep : -1;
    // Prefer not pressing unless pressing now is clearly better: waiting keeps
    // options open. But if the best "wait" plan presses within two steps anyway,
    // waiting gains nothing and risks being too late: press unless waiting is
    // clearly better.
    bool const reallyWaits = m_release.plan.pressAt < 0 || m_release.plan.pressAt >= 3;
    m_bestPresses = reallyWaits ? score(m_press) > score(m_release) + 1.f : score(m_press) + 1.f >= score(m_release);
    m_advice.bestPresses = m_bestPresses;
    m_advice.bestPressAt = m_bestPresses ? 0 : m_release.plan.pressAt;

    if (groundOnly(s.mode) && !s.onGround && !holding && !m_bestPresses && m_release.landed) {
        int32_t landStep = 0;
        for (auto const& pt : m_release.points) {
            ++landStep;
            if (pt.x >= m_release.landX) break;
        }
        if (landStep > 24) m_reuseUntilTick = obs.tick + landStep - 20;
    }
    m_horizon = savedHorizon;
    m_advice.micros = std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - t0).count();
    return m_advice;
}

void Predictor::noteExecuted(bool pressed) {
    m_executedPress = pressed;
    m_executedValid = m_advice.valid;
    if (!m_advice.valid) {
        m_expectValid = false;
        return;
    }
    PredictedPath const& p = pressed ? m_press : m_release;
    m_expectValid = true;
    m_expectOutcome = p.outcome;
    m_expectTick = m_adviceTick;
    m_expectSteps = p.steps;
    m_expectCollisionStep = p.collisionStep;
    m_expectCX = p.collisionX;
    m_expectCY = p.collisionY;
}

void Predictor::onDeath(int32_t tick, float x, float y) {
    char buf[200];
    if (!m_expectValid) {
        m_deathComparison = "no prediction was active";
        return;
    }
    int32_t dt = tick - m_expectTick;
    if (m_expectOutcome == Certainty::Safe && dt <= 3) {
        // A safe continuation was predicted a moment before dying: the model is wrong here.
        ++m_mispredictions;
        m_caution = std::min(4.f, m_caution * 1.3f);
        std::snprintf(buf, sizeof(buf), "MISPREDICTION: predicted safe %d steps before dying at (%.1f, %.1f); caution now %.2f", dt, x, y,
                      m_caution);
    } else if (m_expectCollisionStep >= 0) {
        std::snprintf(buf, sizeof(buf), "predicted %s at (%.1f, %.1f) in %d steps; died at (%.1f, %.1f) after %d steps",
                      certaintyName(m_expectOutcome), m_expectCX, m_expectCY, m_expectCollisionStep + 1, x, y, dt);
    } else {
        std::snprintf(buf, sizeof(buf), "last prediction (%s) was %d steps before dying at (%.1f, %.1f)", certaintyName(m_expectOutcome), dt,
                      x, y);
    }
    m_deathComparison = buf;
    m_expectValid = false;
}

void Predictor::updateBounds(Observation const& obs, bool held) {
    ModePhysics const& P = m_model.params(obs.mode, obs.mini);
    float const h = P.halfSize;
    bool const hintChanged = obs.hasBoundsHint &&
                             (std::fabs(obs.boundsHintBottom - m_boundsHintBottom) > 1.f || std::fabs(obs.boundsHintTop - m_boundsHintTop) > 1.f);
    if (obs.mode != m_boundsMode || hintChanged) {
        // New mode / corridor: start from the hint (unconfirmed) or the plain level floor.
        bool first = m_boundsMode == GameMode::Count;
        m_boundsMode = obs.mode;
        if (first) {
            m_bounds = Bounds{};
            m_bounds.floor = m_model.cubeFloorY(false) - 15.f;
        }
        // Without a hint the previous floor/ceiling stay, but must be re-confirmed.
        if (obs.mode != GameMode::Cube) {
            m_bounds.floorEstimated = true;
            m_bounds.ceilEstimated = true;
        } else {
            m_bounds.hasCeil = false;
        }
        if (obs.hasBoundsHint && obs.mode != GameMode::Cube) {
            m_bounds.floor = obs.boundsHintBottom;
            m_bounds.hasCeil = obs.boundsHintTop > obs.boundsHintBottom + 2.f * h;
            m_bounds.ceil = obs.boundsHintTop;
            m_bounds.ceilEstimated = true;
        }
        m_boundsHintBottom = obs.boundsHintBottom;
        m_boundsHintTop = obs.boundsHintTop;
    }

    // Confirm a bound by touching it where no block explains the contact.
    auto blockAt = [&](bool below) {
        for (auto const& o : obs.obstacles) {
            if (o.kind != Obstacle::Kind::Solid || o.l >= obs.x + h || o.r <= obs.x - h) continue;
            if (below && std::fabs(o.t - (obs.y - h)) < 1.f) return true;
            if (!below && std::fabs(o.b - (obs.y + h)) < 1.f) return true;
        }
        return false;
    };
    bool const resting = std::fabs(obs.vy) < 0.01f;
    bool bottom = false, top = false;
    if (obs.onGround) {
        (obs.upsideDown ? top : bottom) = true;
    } else if (obs.mode == GameMode::Ship || obs.mode == GameMode::Wave) {
        // flying pressed against a bound: no movement for several steps (a single
        // zero-velocity step also happens when the ship turns around mid-air)
        bool pinned = resting && std::fabs(obs.y - m_lastY) < 0.01f;
        m_pressedSteps = pinned ? m_pressedSteps + 1 : 0;
        if (m_pressedSteps >= 3) (held != obs.upsideDown ? top : bottom) = true;
    }
    m_lastY = obs.y;
    if (bottom || top) {
        bool atBottom = bottom;
        if (atBottom && !blockAt(true)) {
            float surf = obs.y - h;
            if (m_bounds.floorEstimated || surf < m_bounds.floor) {
                m_bounds.floor = surf;
                m_bounds.floorEstimated = false;
                m_bounds.hasFloor = true;
            }
        } else if (!atBottom && !blockAt(false)) {
            float surf = obs.y + h;
            if (!m_bounds.hasCeil || m_bounds.ceilEstimated || surf > m_bounds.ceil) {
                m_bounds.ceil = surf;
                m_bounds.ceilEstimated = false;
                m_bounds.hasCeil = true;
            }
        }
    }
}

void Predictor::observe(Observation const& obs, bool heldDuringStep) {
    // robot boost bookkeeping (needed to simulate from the middle of a jump)
    if (obs.mode == GameMode::Robot) {
        Rec const* last = m_histCount ? &m_hist[(m_histHead + kHistory - 1) % kHistory] : nullptr;
        if (last && last->tick + 1 == obs.tick && last->onGround && !obs.onGround && heldDuringStep) {
            m_robotSince = 0;
            m_robotAlive = true;
        } else if (!obs.onGround) {
            ++m_robotSince;
            if (!heldDuringStep) m_robotAlive = false;
        } else {
            m_robotAlive = false;
        }
    } else {
        m_robotAlive = false;
    }
    int32_t boostLeft = obs.mode == GameMode::Robot && m_robotAlive
                            ? std::max(0, m_model.params(obs.mode, obs.mini).boostSteps - m_robotSince) : 0;
    Rec r{obs.x, obs.y, obs.vy, obs.onGround, obs.upsideDown, obs.mini, heldDuringStep, obs.dead, obs.mode, obs.speedClass(), obs.tick,
          obs.playerSpeed, boostLeft};
    if (!obs.dead) updateBounds(obs, heldDuringStep);

    if (m_histCount > 0) {
        Rec const& p = m_hist[(m_histHead + kHistory - 1) % kHistory];
        if (p.tick + 1 == r.tick) {
            Observation prev;
            prev.x = p.x;
            prev.y = p.y;
            prev.vy = p.vy;
            prev.onGround = p.onGround;
            prev.upsideDown = p.upsideDown;
            prev.mini = p.mini;
            prev.dead = p.dead;
            prev.mode = p.mode;
            prev.tick = p.tick;
            prev.playerSpeed = p.playerSpeed;
            m_model.observeStep(prev, obs, heldDuringStep, p.held);

            // one-step prediction error (diagnostics)
            if (p.mode == obs.mode && p.mini == obs.mini && !p.dead && !obs.dead) {
                SimStart s{p.x, p.y, p.vy, p.mode, p.onGround, p.upsideDown, p.mini, p.held, p.speedClass, p.tick, m_bounds,
                           p.robotBoostLeft};
                uint8_t hold = heldDuringStep ? 1 : 0;
                simulate(s, obs.obstacles, Plan{}, m_resim, std::span<uint8_t const>(&hold, 1));
                if (!m_resim.points.empty()) {
                    m_predY = m_resim.points.back().y;
                    m_actY = obs.y;
                    m_stepErr = std::fabs(m_predY - m_actY);
                }
            }
        } else {
            m_histCount = 0; // gap (reset, respawn): start a new history
        }
    }
    m_hist[m_histHead] = r;
    m_histHead = (m_histHead + 1) % kHistory;
    m_histCount = std::min(m_histCount + 1, kHistory);

    // Every 16 steps, re-simulate the last 32 real steps with the inputs that
    // were actually used and compare the end point with reality.
    if (++m_sinceResim >= 16 && m_histCount > static_cast<size_t>(kResimSteps)) {
        m_sinceResim = 0;
        auto at = [&](size_t back) -> Rec const& { return m_hist[(m_histHead + kHistory - 1 - back) % kHistory]; };
        Rec const& s0 = at(kResimSteps);
        bool ok = supports(s0.mode);
        for (int i = 0; i <= kResimSteps && ok; ++i) {
            Rec const& q = at(static_cast<size_t>(i));
            ok = q.mode == s0.mode && q.mini == s0.mini && !q.dead && q.speedClass == s0.speedClass;
        }
        if (ok) {
            m_holdBuf.clear();
            for (int i = kResimSteps - 1; i >= 0; --i) m_holdBuf.push_back(at(static_cast<size_t>(i)).held ? 1 : 0);
            SimStart s{s0.x, s0.y, s0.vy, s0.mode, s0.onGround, s0.upsideDown, s0.mini, s0.held, s0.speedClass, s0.tick, m_bounds,
                       s0.robotBoostLeft};
            simulate(s, obs.obstacles, Plan{}, m_resim, m_holdBuf);
            Rec const& end = at(0);
            float err;
            if (m_resim.uncertainFrom >= 0) {
                err = -1.f; // unmodelled object involved: not the model's fault
            } else if (m_resim.outcome == Certainty::Collision || static_cast<int>(m_resim.points.size()) < kResimSteps) {
                err = 20.f; // the model thought we would die but we did not
            } else {
                auto const& e = m_resim.points.back();
                err = std::hypot(e.x - end.x, e.y - end.y);
            }
            if (err >= 0.f) m_model.recordAccuracy(s0.mode, err, kResimSteps);
        }
    }

    if (m_expectValid && obs.tick > m_expectTick + m_expectSteps) {
        if (m_expectOutcome == Certainty::Safe) m_caution = std::max(1.f, m_caution * 0.98f);
        m_expectValid = false;
    }
}

} // namespace aigd
