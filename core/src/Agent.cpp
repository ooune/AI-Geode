#include <aigd/Agent.hpp>
#include <aigd/StateKey.hpp>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

namespace aigd {

namespace {
constexpr float kTieEpsilon = 1e-6f;
constexpr float kCommitMargin = 0.05f;
constexpr int32_t kMaxShiftCells = 24;
constexpr int32_t kMaxFlyingShiftCells = 60;
constexpr size_t kTraceSamples = 240;

std::string fmt(char const* f, ...) {
    char buf[256];
    va_list args;
    va_start(args, f);
    std::vsnprintf(buf, sizeof(buf), f, args);
    va_end(args);
    return buf;
}

int desire(ExperimentType type, FailureKind kind) {
    switch (type) {
        case ExperimentType::ShiftBlock:
        case ExperimentType::InsertPress:
            switch (kind) {
                case FailureKind::TooEarly: return +1;
                case FailureKind::TooLate: return -1;
                case FailureKind::NoJump: return -1;
                case FailureKind::ReleasedTooLong: return -1;
                default: return 0;
            }
        case ExperimentType::ShiftEnd:
            switch (kind) {
                case FailureKind::HeldTooLong: return -1;
                case FailureKind::ReleasedTooLong: return +1;
                default: return 0;
            }
        case ExperimentType::ShiftStart:
        case ExperimentType::HoldRange:
            switch (kind) {
                case FailureKind::ReleasedTooLong: return -1;
                case FailureKind::HeldTooLong: return +1;
                default: return 0;
            }
        case ExperimentType::ReleaseRange:
            switch (kind) {
                case FailureKind::HeldTooLong: return -1;
                case FailureKind::ReleasedTooLong: return +1;
                default: return 0;
            }
        default: return 0;
    }
}
} // namespace

Agent::Agent(LearnerConfig cfg) : m_cfg(cfg), m_rng(cfg.seed) {}

void Agent::attach(LevelMemory* level, PatternMemory* patterns) {
    m_level = level;
    m_patterns = patterns;
    resetTransient();
    refreshStatus();
}

void Agent::detach() {
    m_level = nullptr;
    m_patterns = nullptr;
    resetTransient();
    m_status = AgentStatus{};
}

void Agent::resetTransient() {
    m_active = false;
    m_steps.clear();
    m_inputs.clear();
    m_leads.clear();
    m_activeExp = {};
    m_pendingExp = {};
    m_incumbent = {};
    m_searches.clear();
    m_constraints.clear();
    m_lastSearchHotspot = INT32_MIN;
    m_focusFrom = 0.f;
    m_focusTo = -1.f;
    m_prevEndX = -1.f;
    m_protectLimitX = 1e30f;
    m_regressions.clear();
    m_nextChange.clear();
    m_lastReport = {};
    m_curAction = 0;
    m_curKey = ~uint64_t(0);
}

float Agent::progressReward(float dx) const {
    return std::max(dx, 0.f) / kBlock * m_cfg.progressRewardPerBlock;
}

uint32_t Agent::stuckLevel() const {
    if (!m_level || m_cfg.stuckAttempts <= 0) return 0;
    return std::min<uint32_t>(3, m_level->attemptsSinceBest / static_cast<uint32_t>(m_cfg.stuckAttempts));
}

float Agent::protectedUpTo() const {
    if (!m_level || m_level->bestPath.empty()) return -1e30f;
    float limit;
    if (m_level->completions > 0 && m_level->levelLength > 1.f && m_level->bestPathX >= m_level->levelLength - kBlock) {
        limit = 1e30f; // the whole level is solved
    } else {
        // Leave the last stretch before the frontier free for learning: that is
        // where the death that is being worked on is usually caused.
        float scale = unitsPerSecond(m_lastObs.speedClass()) / unitsPerSecond(1);
        limit = m_level->bestPathX - 8.f * kBlock * std::max(1.f, scale);
    }
    return std::min(limit, m_protectLimitX);
}

void Agent::commitBestPath() {
    if (!m_level || !m_cfg.learningEnabled) return;
    float upTo = protectedUpTo();
    for (auto const& p : m_level->bestPath) {
        if (p.x >= upTo) break;
        QEntry& e = m_level->q.getOrCreate(p.key);
        int a = p.action;
        if (e.q[a] <= e.q[1 - a] + kCommitMargin) e.q[a] = e.q[1 - a] + kCommitMargin;
    }
}

float Agent::epsilonAt(float x) const {
    if (x < protectedUpTo()) return 0.f;
    if (x < m_focusFrom || x > m_focusTo) return 0.f;
    float eps = m_cfg.focusEpsilon * (1.f + 2.f * static_cast<float>(stuckLevel()));
    if (!m_activeExp.active) eps *= 2.f; // no targeted idea: explore more broadly
    return std::min(eps, 0.5f);
}

QEntry* Agent::lookup(uint64_t key, uint64_t pkey, QEntry& scratch) {
    auto seed = [&](QEntry& e) {
        if (!m_patterns) return;
        if (auto const* pe = m_patterns->q.find(pkey)) {
            for (int a = 0; a < kActions; ++a) {
                if (pe->visited(a)) e.q[a] = pe->q[a] * m_cfg.priorWeight;
            }
        }
    };
    if (m_cfg.learningEnabled) {
        bool created = false;
        QEntry& e = m_level->q.getOrCreate(key, &created);
        if (created) seed(e);
        return &e;
    }
    if (auto* e = m_level->q.find(key)) return e;
    scratch = QEntry{};
    seed(scratch);
    return &scratch;
}

int Agent::greedyAction(QEntry const& e, uint64_t) const {
    if (e.q[1] > e.q[0] + kTieEpsilon) return 1;
    return 0;
}

void Agent::beginAttempt(Observation const& first, bool fromCheckpoint) {
    if (!m_level) return;
    if (m_active) abortAttempt(nullptr);

    m_active = true;
    m_fromCheckpoint = fromCheckpoint;
    m_steps.clear();
    m_steps.reserve(8192);
    m_inputs.clear();
    m_leads.clear();
    m_curKey = ~uint64_t(0);
    m_curAction = 0;
    m_lastDecisionTick = first.tick;
    m_lastToggleTick = -1000000;
    m_startTick = first.tick;
    m_startX = first.x;
    m_maxX = first.x;
    m_lastObs = first;

    m_activeExp = {};
    if (m_cfg.learningEnabled && m_pendingExp.active && m_pendingExp.toCell >= xCell(first.x, m_cfg.cellSize)) {
        m_activeExp = std::move(m_pendingExp);
    }
    m_pendingExp = {};

    m_rng.seed(m_cfg.seed ^ ((m_level->totalAttempts + 1) * 2654435761u));
    m_status.phase = AgentPhase::Playing;
    m_status.attempt = m_level->totalAttempts + 1;
    m_status.shieldsThisAttempt = 0;
    m_status.plannedThisAttempt = 0;
    m_status.lastChange = m_nextChange;
    m_status.progress = first.progress();
    refreshStatus();
}

bool Agent::act(Observation const& obs) {
    if (!m_active || !m_level) return false;
    m_lastObs = obs;
    m_maxX = std::max(m_maxX, obs.x);
    m_status.progress = obs.progress();

    // Timing statistics: a press that is followed by safely passing the obstacle
    // that was ahead at the time is recorded as a successful lead distance.
    for (auto it = m_leads.begin(); it != m_leads.end();) {
        if (obs.x > it->obstacleX + kBlock) {
            if (m_cfg.learningEnabled && m_patterns) m_patterns->leads[it->key].add(it->lead);
            it = m_leads.erase(it);
        } else {
            ++it;
        }
    }

    bool holdingNow = m_curAction == 1;
    uint64_t key = encodeLevelKey(obs, holdingNow, m_cfg.cellSize);
    if (key == m_curKey && obs.tick - m_lastDecisionTick < m_cfg.maxStickTicks) return holdingNow;

    if (!m_steps.empty()) m_steps.back().reward += progressReward(obs.x - m_steps.back().x);

    uint64_t pkey = encodePatternKey(obs, holdingNow);
    QEntry scratch;
    QEntry* e = lookup(key, pkey, scratch);
    int a = greedyAction(*e, pkey);
    bool forced = false;
    bool explored = false;

    int32_t cell = xCell(obs.x, m_cfg.cellSize);
    if (m_activeExp.covers(cell)) {
        a = m_activeExp.actionAt(cell);
        forced = true;
    } else if (m_cfg.learningEnabled) {
        float eps = epsilonAt(obs.x);
        if (eps > 0.f && std::uniform_real_distribution<float>(0.f, 1.f)(m_rng) < eps) {
            a = 1 - a;
            explored = true;
        }
    }

    // Trajectory prediction: simulate pressing vs. releasing before committing.
    bool planned = false;
    bool shielded = false;
    bool advised = false;
    if (m_predictor && m_cfg.usePrediction && !obs.obstacles.empty()) {
        auto const& adv = m_predictor->advise(obs, holdingNow);
        advised = true;
        m_status.predictionValid = adv.valid;
        m_status.predictionTrusted = adv.trusted;
        if (adv.valid) {
            m_status.pressOutcome = adv.press;
            m_status.releaseOutcome = adv.release;
            m_status.pressTTC = adv.pressTTC;
            m_status.releaseTTC = adv.releaseTTC;
        }
        if (adv.valid && adv.trusted) {
            // No learned preference here yet: follow the planner's best plan.
            bool opinion = (e->visited(0) || e->visited(1)) && std::fabs(e->q[0] - e->q[1]) > 1e-3f;
            if (!forced && !explored && !opinion) {
                a = adv.bestPresses ? 1 : 0;
                planned = true;
            }
            // Never execute an action whose every continuation is predicted to
            // collide while the other action still has a way through. The
            // rejected action is learned as an imagined death right away, so
            // the timing does not have to be discovered by dying.
            Certainty mine = a ? adv.press : adv.release;
            Certainty other = a ? adv.release : adv.press;
            if (mine == Certainty::Collision && other != Certainty::Collision) {
                if (m_cfg.learningEnabled) {
                    int32_t ttc = std::max(0, a ? adv.pressTTC : adv.releaseTTC);
                    float target = -m_cfg.deathPenalty * std::pow(m_cfg.gammaPerTick, static_cast<float>(ttc));
                    e->q[a] += m_cfg.alpha() * m_cfg.imaginedScale * (target - e->q[a]);
                }
                a = 1 - a;
                shielded = true;
                forced = false;
                explored = false;
            }
        }
    }

    // Never toggle faster than the configured minimum (prevents input spam).
    if (a != m_curAction && obs.tick - m_lastToggleTick < m_cfg.minToggleTicks) {
        a = m_curAction;
        forced = false;
        explored = false;
        shielded = false;
    }
    if (advised) m_predictor->noteExecuted(a == 1);
    if (shielded) {
        ++m_status.shieldsThisAttempt;
        ++m_status.shieldsTotal;
    }
    if (planned) ++m_status.plannedThisAttempt;

    if (a != m_curAction) {
        m_lastToggleTick = obs.tick;
        m_inputs.emplace_back(obs.x, static_cast<uint8_t>(a));
        if (a == 1 && obs.hazard.present && obs.hazard.dx >= 0.f && obs.hazard.dx < 6.f * kBlock && m_leads.size() < 8) {
            m_leads.push_back({obs.x + obs.hazard.dx, obs.hazard.dx, timingKey(obs.mode, obs.speedClass(), obs.mini)});
        }
    }

    Step s;
    s.key = key;
    s.pkey = pkey;
    s.x = obs.x;
    s.y = obs.y;
    s.vy = obs.vy;
    s.tick = obs.tick;
    s.mode = obs.mode;
    s.onGround = obs.onGround;
    s.action = static_cast<uint8_t>(a);
    s.forced = forced;
    s.explored = explored;
    s.planned = planned;
    s.shielded = shielded;
    m_steps.push_back(s);

    m_curKey = key;
    m_curAction = static_cast<uint8_t>(a);
    m_lastDecisionTick = obs.tick;

    m_status.q[0] = e->q[0];
    m_status.q[1] = e->q[1];
    m_status.action = static_cast<uint8_t>(a);
    m_status.forced = forced;
    m_status.explored = explored;
    m_status.planned = planned;
    m_status.shielded = shielded;
    m_status.epsilon = epsilonAt(obs.x);
    m_status.decisions = m_steps.size();
    return a == 1;
}

void Agent::backwardUpdate(float terminalBonus, bool terminal, float bootstrap, int32_t endTick) {
    if (m_steps.empty() || !m_level) return;
    m_steps.back().reward += terminalBonus;

    float const alpha = m_cfg.alpha();
    float const pAlpha = m_cfg.patternAlpha();
    float const logG = std::log(std::clamp(m_cfg.gammaPerTick, 0.5f, 0.99999f));
    float const logL = std::log(std::clamp(m_cfg.lambdaPerTick, 0.01f, 0.99999f));

    float G = terminal ? 0.f : bootstrap;   // lambda-return of the following step
    float nextMax = G;                      // max Q of the following state
    int32_t nextTick = endTick;
    size_t const n = m_steps.size();
    for (size_t i = n; i-- > 0;) {
        Step const& s = m_steps[i];
        int32_t dt = std::max(1, nextTick - s.tick);
        float g = std::exp(logG * static_cast<float>(dt));
        float boot;
        if (i == n - 1) {
            boot = terminal ? 0.f : bootstrap;
        } else {
            float l = std::exp(logL * static_cast<float>(dt));
            boot = (1.f - l) * nextMax + l * G;
        }
        float target = s.reward + g * boot;

        QEntry& e = m_level->q.getOrCreate(s.key);
        e.q[s.action] += alpha * (target - e.q[s.action]);
        if (e.n[s.action] < 0xFFFF) ++e.n[s.action];

        if (m_patterns) {
            QEntry& pe = m_patterns->q.getOrCreate(s.pkey);
            pe.q[s.action] += pAlpha * (target - pe.q[s.action]);
            if (pe.n[s.action] < 0xFFFF) ++pe.n[s.action];
        }

        G = target;
        nextMax = e.best();
        nextTick = s.tick;
    }
}

void Agent::commitExperiment() {
    // The experiment worked: make sure the greedy policy now prefers what the
    // experiment did, so the improvement is kept on the next attempts.
    for (auto const& s : m_steps) {
        if (!s.forced) continue;
        if (auto* e = m_level->q.find(s.key)) {
            int a = s.action;
            if (e->q[a] <= e->q[1 - a] + kCommitMargin) e->q[a] = e->q[1 - a] + kCommitMargin;
        }
    }
}

Agent::ActionTrace Agent::buildTrace(float endX) const {
    ActionTrace t;
    if (m_steps.empty()) return t;
    float cs = m_cfg.cellSize;
    t.valid = true;
    t.startCell = xCell(m_steps.front().x, cs);
    int32_t endCell = std::max(t.startCell, xCell(endX, cs));
    t.actions.assign(static_cast<size_t>(endCell - t.startCell + 1), 0);
    t.ground.assign(t.actions.size(), 1);
    t.endX = endX;
    // The action recorded for a cell is the one active at the cell's centre.
    size_t j = 0;
    for (int32_t c = t.startCell; c <= endCell; ++c) {
        float centre = (static_cast<float>(c) + 0.5f) * cs;
        while (j + 1 < m_steps.size() && m_steps[j + 1].x <= centre) ++j;
        bool covered = m_steps[j].x <= centre;
        auto i = static_cast<size_t>(c - t.startCell);
        t.actions[i] = covered ? m_steps[j].action : 0;
        t.ground[i] = covered ? (m_steps[j].onGround ? 1 : 0) : 1;
    }
    return t;
}

void Agent::finishAttempt(float endX, int32_t endTick, bool died, bool completed, FailureKind kind) {
    auto& L = *m_level;
    ++L.totalAttempts;
    if (died) ++L.deaths;
    if (completed) ++L.completions;
    if (L.levelLength < m_lastObs.levelLength) L.levelLength = m_lastObs.levelLength;

    float progress = completed ? 1.f : (L.levelLength > 1.f ? std::clamp(endX / L.levelLength, 0.f, 1.f) : 0.f);

    AttemptRecord rec;
    rec.number = L.totalAttempts;
    rec.progress = progress;
    rec.endX = endX;
    rec.kind = kind;
    rec.ticks = endTick - m_startTick;
    rec.completed = completed;
    rec.fromCheckpoint = m_fromCheckpoint;
    rec.improved = m_prevEndX >= 0.f && (completed || endX > m_prevEndX + kBlock * 0.5f);
    rec.change = m_status.lastChange;
    m_status.lastImproved = m_prevEndX >= 0.f ? (rec.improved ? 1 : 0) : -1;
    m_prevEndX = endX;

    int32_t expHotspot = INT32_MIN;
    if (m_activeExp.active) {
        expHotspot = m_activeExp.hotspot;
        float hotspotEnd = static_cast<float>(m_activeExp.hotspot + 1) * kBlock;
        bool success = completed || endX > hotspotEnd + kBlock * 0.5f;
        auto& h = L.hotspots[m_activeExp.hotspot];
        h.bucket = m_activeExp.hotspot;
        if (success) {
            ++h.passes;
            h.experiments = 0;
            h.escalation = 0;
            for (auto s = m_searches.begin(); s != m_searches.end();) {
                s = std::abs(s->first - m_activeExp.hotspot) <= 1 ? m_searches.erase(s) : std::next(s);
            }
            if (m_activeExp.type == ExperimentType::ShiftBlock || m_activeExp.type == ExperimentType::InsertPress) {
                // Remember the timing window that made this work, so that fixing a
                // later obstacle never silently undoes it.
                int32_t newStart = m_activeExp.blockStart + m_activeExp.shift;
                Constraint c;
                if (m_activeExp.lo != INT32_MIN) c.lo = m_activeExp.blockStart + m_activeExp.lo;
                if (m_activeExp.hi != INT32_MAX) c.hi = m_activeExp.blockStart + m_activeExp.hi;
                for (int32_t k = m_activeExp.blockStart - 1; k <= m_activeExp.blockStart + 1; ++k) {
                    auto old = m_constraints.find(k);
                    if (old == m_constraints.end()) continue;
                    c.lo = std::max(c.lo, old->second.lo);
                    c.hi = std::min(c.hi, old->second.hi);
                    m_constraints.erase(old);
                }
                if (c.lo != INT32_MIN || c.hi != INT32_MAX) m_constraints[newStart] = c;
            }
            if (m_cfg.learningEnabled) commitExperiment();
            rec.change += " [kept: it worked]";
        } else {
            ++h.experiments;
            rec.change += " [did not pass]";
        }
    }
    m_lastSearchHotspot = expHotspot;

    for (auto& [bucket, h] : L.hotspots) {
        if (bucket == expHotspot) continue;
        float bx = static_cast<float>(bucket) * kBlock;
        if (bx >= m_startX && bx + kBlock * 1.5f < endX) ++h.passes;
    }

    if (!m_fromCheckpoint && (endX > L.bestPathX + 1e-3f || (completed && L.bestPathX < L.levelLength))) {
        L.bestPathX = completed ? std::max(endX, L.levelLength) : endX;
        L.bestPath.clear();
        L.bestPath.reserve(m_steps.size());
        for (auto const& s : m_steps) L.bestPath.push_back({s.x, s.key, s.action});
        m_protectLimitX = 1e30f;
        m_regressions.clear();
    }
    commitBestPath();

    if (progress > L.bestProgress + 1e-6f) {
        L.bestProgress = progress;
        L.bestX = endX;
        if (!m_fromCheckpoint) L.bestRunInputs = m_inputs;
        L.attemptsSinceBest = 0;
    } else {
        ++L.attemptsSinceBest;
    }

    if (died) {
        L.lastFailureTrace.clear();
        size_t from = m_steps.size() > kTraceSamples ? m_steps.size() - kTraceSamples : 0;
        for (size_t i = from; i < m_steps.size(); ++i) {
            auto const& s = m_steps[i];
            L.lastFailureTrace.push_back({s.x, s.y, s.vy, static_cast<uint8_t>(s.mode), s.onGround, s.action});
        }
    }

    L.pushHistory(std::move(rec));
    m_activeExp = {};
    refreshStatus();
}

void Agent::onDeath(Observation const& last, DeathInfo const& death) {
    if (!m_active || !m_level) return;
    m_active = false;
    m_lastObs = last;
    float endX = std::max({death.x, last.x, m_maxX});
    if (!m_steps.empty()) m_steps.back().reward += progressReward(endX - m_steps.back().x);

    int32_t bucket = static_cast<int32_t>(std::floor(endX / kBlock));
    auto& h = m_level->hotspots[bucket];
    h.bucket = bucket;
    ++h.deaths;

    FailureReport rep = analyzeFailure(m_steps, last, death, h.escalation);
    if (m_predictor) {
        m_predictor->onDeath(last.tick, death.x, death.y);
        m_status.predictionNote = m_predictor->lastDeathComparison();
    }
    h.lastKind = rep.kind;
    m_lastReport = rep;

    if (m_cfg.learningEnabled) backwardUpdate(-m_cfg.deathPenalty, true, 0.f, last.tick);
    finishAttempt(endX, last.tick, true, false, rep.kind);

    m_focusFrom = rep.windowStartX;
    m_focusTo = rep.deathX + kBlock;
    // An experiment that made things worse is judged by planNext, not here.
    bool regression = !m_fromCheckpoint && endX < protectedUpTo() && m_lastSearchHotspot == INT32_MIN;
    if (regression && m_cfg.learningEnabled) {
        // Died in a section the best run already got through. The best path
        // has just been re-committed, so replay it; if that keeps failing here
        // (the game is not perfectly repeatable), hand the spot back to learning.
        if (++m_regressions[bucket] >= 3) {
            m_protectLimitX = std::min(m_protectLimitX, static_cast<float>(bucket - 1) * kBlock);
            m_regressions.erase(bucket);
            regression = false;
        } else {
            m_pendingExp = {};
            m_nextChange = fmt("Fell behind the best run at %.1f%%: restoring the known-good path", rep.deathProgress * 100.f);
        }
    }
    if (regression) {
        // handled above
    } else if (m_cfg.learningEnabled) {
        planNext(rep);
    } else {
        m_pendingExp = {};
        m_nextChange = "Learning is off: replaying the current policy unchanged";
    }

    m_status.lastFailure = rep.text;
    m_status.lastFailureProgress = rep.deathProgress;
    m_status.phase = AgentPhase::Learning;
    refreshStatus();
}

void Agent::onComplete(Observation const& last) {
    if (!m_active || !m_level) return;
    m_active = false;
    m_lastObs = last;
    float endX = std::max(last.x, m_maxX);
    if (!m_steps.empty()) m_steps.back().reward += progressReward(endX - m_steps.back().x);
    if (m_cfg.learningEnabled) backwardUpdate(m_cfg.completionReward, true, 0.f, last.tick);
    finishAttempt(endX, last.tick, false, true, FailureKind::None);
    m_pendingExp = {};
    m_incumbent = buildTrace(endX);
    m_nextChange = "Level completed: keeping the learned policy";
    m_status.phase = AgentPhase::Completed;
    refreshStatus();
}

void Agent::abortAttempt(Observation const*) {
    if (!m_active) return;
    m_active = false;
    // An interrupted attempt carries no outcome, so nothing is learned from it.
    // A running experiment is retried on the next attempt instead of being judged.
    if (m_activeExp.active) m_pendingExp = std::move(m_activeExp);
    m_activeExp = {};
    m_steps.clear();
    m_inputs.clear();
    m_leads.clear();
    m_curAction = 0;
    m_curKey = ~uint64_t(0);
}

void Agent::planNext(FailureReport const& rep) {
    m_pendingExp = {};
    auto qOnly = [&](char const* why) {
        m_nextChange = fmt(
            "%sUpdated values for %zu decisions; exploring around %.1f%%",
            why, m_steps.size(), rep.deathProgress * 100.f
        );
    };
    if (!m_cfg.useExperiments) {
        qOnly("");
        return;
    }

    ActionTrace latest = buildTrace(rep.deathX);
    latest.report = rep;

    bool haveInc = m_incumbent.valid;
    bool atLeastAsGood = !haveInc || latest.endX >= m_incumbent.endX - kBlock * 0.5f;
    bool diedBeforeWindow = haveInc && latest.endX < m_incumbent.report.windowStartX;
    bool useLatest = atLeastAsGood || diedBeforeWindow;
    if (atLeastAsGood) m_incumbent = latest;
    ActionTrace const& base = useLatest ? latest : m_incumbent;
    FailureReport const& report = base.report;

    // Find an existing search for this hotspot (one block tolerance).
    auto it = m_searches.end();
    for (auto s = m_searches.begin(); s != m_searches.end(); ++s) {
        if (std::abs(s->first - report.hotspot) <= 1) {
            it = s;
            break;
        }
    }
    bool experimentWasHere = m_lastSearchHotspot != INT32_MIN && std::abs(m_lastSearchHotspot - report.hotspot) <= 1;

    auto& hot = m_level->hotspots[report.hotspot];
    hot.bucket = report.hotspot;

    if (it == m_searches.end() && hot.deaths < static_cast<uint32_t>(std::max(1, m_cfg.experimentAfterDeaths))) {
        // New trouble spot: Q-value backtracking usually fixes simple cases on
        // its own; targeted experiments start once deaths keep repeating here.
        qOnly("");
        return;
    }

    Search* search = nullptr;
    if (it != m_searches.end()) {
        search = &it->second;
        if (experimentWasHere) feedback(*search, rep.kind, !useLatest, rep.deathX, rep.deathY);
    }

    auto spent = [&](Search const* s) {
        return !s || s->exhausted || s->tries >= static_cast<uint32_t>(m_cfg.maxExperimentsPerHotspot);
    };
#ifdef AIGD_TRACE
    std::fprintf(stderr, "[plan] death=(%.2f,%.2f) kind=%d hot=%d useLatest=%d search=%d expHere=%d", rep.deathX, rep.deathY, (int)rep.kind, report.hotspot, (int)useLatest, search != nullptr, (int)experimentWasHere);
    if (search) std::fprintf(stderr, " shift=%d lo=%d hi=%d tries=%u exh=%d noEff=%u", search->shift, search->lo, search->hi, search->tries, (int)search->exhausted, search->noEffect);
    std::fprintf(stderr, " esc=%u\n", hot.escalation);
#endif
    for (int attempt = 0; spent(search); ++attempt) {
        if (search) {
            // Every timing variation of the suspected input failed (or there was
            // no room to vary it): the cause is probably an earlier input, so
            // widen the window and look further back.
            ++hot.escalation;
            for (auto s = m_searches.begin(); s != m_searches.end();) {
                s = std::abs(s->first - report.hotspot) <= 1 ? m_searches.erase(s) : std::next(s);
            }
            search = nullptr;
        }
        if (hot.escalation > 3 || attempt > 4) {
            hot.escalation = 0; // start over next time; rely on Q-values + exploration now
            qOnly("All timing ideas tried here. ");
            return;
        }
        Search s;
        if (!initSearch(s, report, base, &m_lastObs)) {
            if (hot.escalation >= 3) {
                qOnly("");
                return;
            }
            s.exhausted = true; // nothing to vary at this level of suspicion: escalate
        }
        s.base = base;
        search = &(m_searches[report.hotspot] = std::move(s));
    }
    buildExperiment(*search, search->base, search->base.report);
}

bool Agent::initSearch(Search& s, FailureReport const& report, ActionTrace const& base, Observation const* last) {
    auto const& hot = m_level->hotspots[report.hotspot];
    int speed = last ? last->speedClass() : 1;
    float window = analysisWindow(speed, hot.escalation);
    float cs = m_cfg.cellSize;

    std::vector<int> cand;
    for (int i = 0; i < static_cast<int>(report.blocks.size()); ++i) {
        auto const& b = report.blocks[i];
        if (b.effective && b.startX >= report.deathX - window && b.startX <= report.deathX) cand.push_back(i);
    }
    int esc = static_cast<int>(std::min<uint32_t>(hot.escalation, 3));
    int block = -1;
    if (!cand.empty()) {
        int idx = static_cast<int>(cand.size()) - 1 - esc;
        if (idx < 0) return false; // no earlier input left to suspect
        block = cand[static_cast<size_t>(idx)];
    }

    int32_t d0 = m_cfg.initialShiftCells();
    s = Search{};
    s.step = d0;
    s.lastEndX = report.deathX;
    s.lastEndY = report.deathY;
    GameMode mode = report.mode;

    auto useBlock = [&](ExperimentType type, int32_t shift) {
        auto const& b = report.blocks[static_cast<size_t>(block)];
        s.type = type;
        s.blockStart = xCell(b.startX, cs);
        s.blockEnd = std::max(s.blockStart, xCell(b.endX, cs) - 1);
        s.shift = shift;
    };
    auto useInsert = [&](float atX, int32_t len) {
        s.type = ExperimentType::InsertPress;
        s.blockStart = xCell(atX, cs);
        s.insertLen = len;
        s.shift = 0;
        if (needsGroundToAct(mode) && !base.groundAt(s.blockStart)) {
            // The player was in the air there last time, where a tap does nothing.
            // Hold through the landing instead (holding re-jumps on landing).
            int32_t c = s.blockStart;
            int32_t limit = std::min(base.endCell(), s.blockStart + 40);
            while (c <= limit && !base.groundAt(c)) ++c;
            if (c <= limit) s.insertLen = std::max(len, c - s.blockStart + 2);
        }
        s.blockEnd = s.blockStart + s.insertLen - 1;
    };

    float ups = unitsPerSecond(speed);
    float lead = (mode == GameMode::Cube || mode == GameMode::Robot) ? 0.2f * ups : 0.12f * ups;
    if (m_patterns && last) {
        auto lt = m_patterns->leads.find(timingKey(mode, speed, last->mini));
        if (lt != m_patterns->leads.end() && lt->second.count >= 3) lead = static_cast<float>(lt->second.mean());
    }
    int32_t tapLen = isFlyingMode(mode) ? 6 : 2;

    if (isFlyingMode(mode) && (report.kind == FailureKind::ReleasedTooLong || report.kind == FailureKind::HeldTooLong ||
                               report.kind == FailureKind::HitWall)) {
        int32_t deathCell = xCell(report.deathX, cs);
        if (report.kind == FailureKind::HitWall) {
            // Over or under is unknown: alternate as the search escalates.
            s.type = (esc % 2 == 0) ? ExperimentType::HoldRange : ExperimentType::ReleaseRange;
        } else {
            s.type = report.kind == FailureKind::HeldTooLong ? ExperimentType::ReleaseRange : ExperimentType::HoldRange;
        }
        s.blockEnd = deathCell + 1;
        s.blockStart = deathCell - 2 * d0 - esc * 4 * d0;
        s.shift = 0;
        return true;
    }

    switch (report.kind) {
        case FailureKind::NoJump:
            if (esc == 0 || block < 0) useInsert(report.deathX + kBlock * 0.5f - lead, tapLen);
            else useBlock(ExperimentType::ShiftBlock, -d0);
            break;
        case FailureKind::TooEarly:
            if (block < 0) return false;
            useBlock(ExperimentType::ShiftBlock, +d0);
            break;
        case FailureKind::TooLate:
            if (block < 0) return false;
            useBlock(ExperimentType::ShiftBlock, -d0);
            break;
        case FailureKind::HeldTooLong:
            if (block < 0) return false;
            useBlock(ExperimentType::ShiftEnd, -d0);
            if (needsGroundToAct(mode) && report.blocks[static_cast<size_t>(block)].groundContacts > 1) {
                // A long hold re-jumped on landing: try a short tap instead.
                s.shift = -std::max(1, s.blockEnd - s.blockStart - 1);
            }
            break;
        case FailureKind::ReleasedTooLong:
            if (block >= 0 && report.deathX - report.blocks[static_cast<size_t>(block)].endX < window * 0.5f)
                useBlock(ExperimentType::ShiftEnd, +d0);
            else
                useInsert(report.deathX - static_cast<float>(d0 + 4) * cs, 4);
            break;
        case FailureKind::HitWall:
        case FailureKind::Unknown:
            if (block >= 0 && isFlyingMode(mode)) useBlock(ExperimentType::ShiftEnd, +d0);
            else if (block >= 0) useBlock(ExperimentType::ShiftBlock, +d0);
            else useInsert(report.deathX + kBlock * 0.5f - lead, tapLen);
            break;
        default:
            return false;
    }

    if (s.type == ExperimentType::ShiftBlock) {
        // Respect timing windows learned from earlier successes with this press.
        for (int32_t k = s.blockStart - 1; k <= s.blockStart + 1; ++k) {
            auto c = m_constraints.find(k);
            if (c == m_constraints.end()) continue;
            if (c->second.lo != INT32_MIN) s.lo = std::max(s.lo, c->second.lo - s.blockStart);
            if (c->second.hi != INT32_MAX) s.hi = std::min(s.hi, c->second.hi - s.blockStart);
        }
        if (s.shift <= s.lo || s.shift >= s.hi) {
            int32_t cand = s.shift > 0 ? std::min(s.shift, s.hi - 1) : std::max(s.shift, s.lo + 1);
            bool sameDirection = (s.shift > 0) == (cand > 0);
            if (cand == 0 || !sameDirection || cand <= s.lo || cand >= s.hi) return false;
            s.shift = cand;
        }
    }
    return true;
}

void Agent::feedback(Search& s, FailureKind kind, bool worse, float endX, float endY) {
    ++s.tries;
    // Two tries in a row that died at exactly the same spot: the input being
    // varied has no influence on this death, so stop varying it.
    if (s.lastEndX >= 0.f && std::fabs(endX - s.lastEndX) < 0.5f && std::fabs(endY - s.lastEndY) < 0.5f) {
        if (++s.noEffect >= 2) {
            s.exhausted = true;
            return;
        }
    } else {
        s.noEffect = 0;
    }
    s.lastEndX = endX;
    s.lastEndY = endY;
    bool flyingType = s.type == ExperimentType::ShiftEnd || s.type == ExperimentType::ShiftStart ||
                      s.type == ExperimentType::HoldRange || s.type == ExperimentType::ReleaseRange;
    int32_t limit = flyingType ? kMaxFlyingShiftCells : kMaxShiftCells;
    // Direction suggested by how the experiment attempt died. This is used even
    // when the result was worse than before, because "worse, and now clearly too
    // late" still says which way to go.
    int d = desire(s.type, kind);
    if (d > 0) {
        s.lo = std::max(s.lo, s.shift);
    } else if (d < 0) {
        s.hi = std::min(s.hi, s.shift);
    } else if (worse) {
        // No directional information and the change made things worse: treat it
        // as an overshoot and go back the other way.
        if (s.shift > 0) s.hi = std::min(s.hi, s.shift);
        else if (s.shift < 0) s.lo = std::max(s.lo, s.shift);
        d = s.shift > 0 ? -1 : (s.shift < 0 ? +1 : 0);
    }

    int32_t next = s.shift;
    bool hasLo = s.lo != INT32_MIN;
    bool hasHi = s.hi != INT32_MAX;
    if (d == 0) {
        // No directional information: alternate +a, -a, +2a, -2a ...
        if (s.shift > 0) {
            next = -s.shift;
        } else {
            next = -s.shift + s.step;
            s.step = s.step + (s.step + 1) / 2;
        }
        if (next == 0) next = s.step;
    } else if (hasLo && hasHi) {
        if (s.hi - s.lo <= 1) {
            s.exhausted = true;
            return;
        }
        next = s.lo + (s.hi - s.lo) / 2;
    } else if (hasLo) {
        next = s.lo + s.step;
        s.step = s.step + (s.step + 1) / 2;
    } else if (hasHi) {
        next = s.hi - s.step;
        s.step = s.step + (s.step + 1) / 2;
    }

    if (std::abs(next) > limit || (hasLo && next <= s.lo) || (hasHi && next >= s.hi)) {
        s.exhausted = true;
        return;
    }
    s.shift = next;
}

void Agent::buildExperiment(Search const& s, ActionTrace const& base, FailureReport const& report) {
    float cs = m_cfg.cellSize;
    auto const& hot = m_level->hotspots[report.hotspot];
    int speed = m_lastObs.speedClass();
    float window = analysisWindow(speed, hot.escalation);

    Experiment ex;
    ex.active = true;
    ex.type = s.type;
    ex.hotspot = report.hotspot;
    ex.shift = s.shift;
    ex.blockStart = s.blockStart;
    ex.lo = s.lo;
    ex.hi = s.hi;
    int32_t deathCell = xCell(report.deathX, cs);
    // The experiment replays the best known actions over a margin before the
    // change as well, so that value updates from the last deaths (which reach a
    // few blocks back) cannot alter the approach and spoil the comparison.
    float margin = 3.f * kBlock * unitsPerSecond(speed) / unitsPerSecond(1);
    ex.fromCell = std::min(xCell(report.deathX - window - margin, cs), std::min(s.blockStart, s.blockStart + s.shift) - 1);
    ex.fromCell = std::max(ex.fromCell, base.startCell);
    ex.toCell = std::max(deathCell + 3, s.blockEnd + std::max(0, s.shift) + 1);
    ex.actions.resize(static_cast<size_t>(ex.toCell - ex.fromCell + 1));
    for (int32_t c = ex.fromCell; c <= ex.toCell; ++c) ex.actions[static_cast<size_t>(c - ex.fromCell)] = base.at(c);

    auto set = [&](int32_t from, int32_t to, uint8_t v) { // inclusive range
        for (int32_t c = std::max(from, ex.fromCell); c <= std::min(to, ex.toCell); ++c)
            ex.actions[static_cast<size_t>(c - ex.fromCell)] = v;
    };

    float units = std::fabs(static_cast<float>(s.shift)) * cs;
    char const* dir = s.shift > 0 ? "later" : "earlier";
    float at = m_lastObs.levelLength > 1.f ? static_cast<float>(s.blockStart) * cs / m_lastObs.levelLength * 100.f : 0.f;
    switch (s.type) {
        case ExperimentType::ShiftBlock:
            set(s.blockStart, s.blockEnd, 0);
            set(s.blockStart + s.shift, s.blockEnd + s.shift, 1);
            ex.description = fmt("Press at %.1f%% moved %.0f units %s", at, units, dir);
            break;
        case ExperimentType::ShiftStart:
            if (s.shift < 0) set(s.blockStart + s.shift, s.blockStart - 1, 1);
            else set(s.blockStart, s.blockStart + s.shift - 1, 0);
            ex.description = fmt("Start holding at %.1f%% %.0f units %s", at, units, dir);
            break;
        case ExperimentType::ShiftEnd:
            if (s.shift < 0) set(s.blockEnd + s.shift + 1, s.blockEnd, 0);
            else set(s.blockEnd + 1, s.blockEnd + s.shift, 1);
            ex.description = fmt("Release at %.1f%% %.0f units %s", at, units, s.shift > 0 ? "later (hold longer)" : "earlier");
            break;
        case ExperimentType::HoldRange:
        case ExperimentType::ReleaseRange: {
            uint8_t v = s.type == ExperimentType::HoldRange ? 1 : 0;
            set(s.blockStart + s.shift, s.blockEnd, v);
            float before = (report.deathX - static_cast<float>(s.blockStart + s.shift) * cs);
            ex.description = fmt("%s continuously from %.0f units before the obstacle at %.1f%%", v ? "Hold" : "Release", before,
                                 report.deathProgress * 100.f);
            break;
        }
        case ExperimentType::InsertPress: {
            set(s.blockStart + s.shift, s.blockStart + s.shift + s.insertLen - 1, 1);
            float before = (report.deathX - static_cast<float>(s.blockStart + s.shift) * cs) / kBlock;
            ex.description = fmt("Added a press %.1f blocks before the obstacle at %.1f%%", before, report.deathProgress * 100.f);
            break;
        }
        default: break;
    }
    // The stretch under investigation is deliberately opened up for change
    // (until the next new best), even if it lies behind the frontier.
    m_protectLimitX = std::min(m_protectLimitX, static_cast<float>(ex.fromCell) * cs);
    m_nextChange = ex.description;
    m_pendingExp = std::move(ex);
}

void Agent::refreshStatus() {
    if (!m_level) return;
    m_status.deaths = m_level->deaths;
    m_status.completions = m_level->completions;
    m_status.bestProgress = m_level->bestProgress;
    m_status.completionRate = m_level->recentCompletionRate();
    m_status.tableSize = m_level->q.size();
    m_status.patternSize = m_patterns ? m_patterns->q.size() : 0;
    m_status.stuck = stuckLevel() > 0;
}

// ---------------------------------------------------------------------------

float LevelMemory::recentCompletionRate(size_t window) const {
    if (history.empty()) return 0.f;
    size_t n = std::min(window, history.size());
    size_t done = 0;
    for (size_t i = history.size() - n; i < history.size(); ++i) done += history[i].completed ? 1 : 0;
    return static_cast<float>(done) / static_cast<float>(n);
}

float LevelMemory::averageProgress(size_t window) const {
    if (history.empty()) return 0.f;
    size_t n = std::min(window, history.size());
    float sum = 0.f;
    for (size_t i = history.size() - n; i < history.size(); ++i) sum += history[i].progress;
    return sum / static_cast<float>(n);
}

} // namespace aigd
