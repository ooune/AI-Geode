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
    char buf[512];
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

// What an action means in words, for the decision diagnostics.
char const* failedPhrase(GameMode mode, bool onGround, bool holding, int a) {
    if (needsGroundToAct(mode)) {
        if (a == 1) return holding ? "holding on" : (onGround ? "an immediate jump" : "pressing in the air");
        return onGround ? "waiting" : "staying released";
    }
    if (a == 1) return holding ? "holding on" : "pressing now";
    return holding ? "releasing now" : "staying released";
}

char const* alternativePhrase(GameMode mode, bool onGround, bool holding, int b) {
    if (needsGroundToAct(mode)) {
        if (b == 1) return onGround ? "jumping here (earlier)" : "holding into the landing";
        return holding ? "releasing here" : "a delayed jump";
    }
    if (b == 1) return holding ? "keeping it held" : "pressing here (earlier)";
    return holding ? "releasing here (earlier)" : "a delayed press";
}

constexpr size_t kNone = static_cast<size_t>(-1);
constexpr size_t kSafetyLogSize = 256;

// Could the player have passed under what killed it by not pressing at all,
// i.e. staying at the height where the press started? Unknown geometry: yes
// (worth one test).
bool walkingUnderClears(FailureReport const& r, PressBlock const& b, bool mini) {
    if (!r.hasKillerExtent) return true;
    float const half = mini ? 9.f : 15.f;
    float const startY = r.upsideDown ? -b.startY : b.startY;
    return r.killerBottom > startY + half + 0.5f;
}

char const* buttonName(GameMode mode, int a) {
    if (needsGroundToAct(mode)) return a ? "jump/hold" : "release";
    return a ? "hold" : "release";
}
} // namespace

char const* safetyVerdictName(SafetyVerdict v) {
    switch (v) {
        case SafetyVerdict::NoPrediction: return "no prediction";
        case SafetyVerdict::Clear: return "safe";
        case SafetyVerdict::Kept: return "kept (danger near)";
        case SafetyVerdict::Overridden: return "SAFETY OVERRIDE";
        case SafetyVerdict::NoSafeOption: return "NO SAFE OPTION";
        case SafetyVerdict::NoInputNeeded: return "no jump needed";
        default: return "?";
    }
}

Agent::Agent(LearnerConfig cfg) : m_cfg(cfg), m_rng(cfg.seed) {}

void Agent::attach(LevelMemory* level, PatternMemory* patterns, StrategyLibrary* strategies) {
    m_level = level;
    m_patterns = patterns;
    m_strategies = strategies;
    resetTransient();
    m_status.transferTotal = {};
    m_status.transferLog.clear();
    m_status.transferEvent.clear();
    if (m_strategies && m_level && m_level->totalAttempts == 0 && m_strategies->size() > 0) {
        int other = 0;
        for (auto const& l : m_strategies->levels) other += l != m_level->key ? 1 : 0;
        noteTransfer(fmt("New level detected - %zu known situations from %d other level%s available as search guidance",
                         m_strategies->size(), other, other == 1 ? "" : "s"));
    }
    refreshStatus();
}

void Agent::detach() {
    m_level = nullptr;
    m_patterns = nullptr;
    m_strategies = nullptr;
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
    m_parked.clear();
    m_constraints.clear();
    m_lastSearchHotspot = INT32_MIN;
    m_focusFrom = 0.f;
    m_focusTo = -1.f;
    m_prevEndX = -1.f;
    m_protectLimitX = 1e30f;
    m_pivot = kNone;
    m_cutAtPivot = false;
    m_avoidedThisAttempt.clear();
    m_removeTried.clear();
    m_plan.clear();
    m_groups.clear();
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

float Agent::epsilonAt(float x) const {
    if (x < protectedUpTo()) return 0.f;
    if (x < m_focusFrom || x > m_focusTo) return 0.f;
    // Broader when stuck, narrower right after progress (refine what works).
    float eps = m_cfg.focusEpsilon * (1.f + 2.f * static_cast<float>(stuckLevel()));
    if (!m_activeExp.active) eps *= 2.f; // no targeted idea: explore more broadly
    if (m_status.lastImproved == 1) eps *= 0.5f;
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
    m_avoidedThisAttempt.clear();
    m_pivot = kNone;
    m_cutAtPivot = false;
    m_danger = false;
    m_safetyLog.clear();
    m_plan.clear();
    m_expChecked = false;
    m_status.planEvent.clear();
    m_status.plansInvalidatedThisAttempt = 0;
    m_status.actionsCancelledThisAttempt = 0;
    m_status.decisionsFollowingPlan = 0;
    m_status.overridesThisAttempt = 0;
    m_status.noSafeOptionThisAttempt = 0;
    m_status.pressesAvoidedThisAttempt = 0;
    m_status.pressesThisAttempt = 0;
    m_status.decisionsTrusted = 0;
    m_status.decisionsUntrusted = 0;
    m_status.safety = SafetyRecord{};
    m_status.transfer = {};
    m_status.transferLog.clear();
    m_groups.clear();
    m_strategic.reset();
    m_strategic.plansMade = 0;
    m_strategic.invalidations = 0;
    m_strategic.overrides = 0;
    m_strategic.totalMicros = 0.0;
    m_strategic.maxMicros = 0.f;
    m_status.strategic.clear();
    m_status.strategicEvent.clear();
    m_status.strategicSeeded = 0;
    m_status.strategicMatched = 0;
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
            if (m_cfg.learningEnabled && m_patterns) {
                m_patterns->leads[it->key].add(it->lead);
                m_patterns->situationLeads[it->situation].add(it->lead);
            }
            it = m_leads.erase(it);
        } else {
            ++it;
        }
    }

    // Obstacle groups ahead (cross-level strategies): resolved when passed,
    // and a new one is described and looked up when it comes into view.
    trackGroups(obs);

    // Strategic plan for the obstacle sequence ahead: checked against the
    // real state on every observation and re-made when the state leaves it.
    // Only with a trusted model (its paths are only as good as the model).
    if (m_predictor && m_cfg.usePrediction && m_cfg.useStrategicPlanning && m_predictor->trusted(obs.mode, obs.mini)) {
        m_strategic.update(obs, m_curAction == 1, *m_predictor, obs.obstacles, obs.events, m_strategies, m_level->key);
    } else if (m_strategic.active()) {
        m_strategic.invalidate("no trusted prediction");
    }
    m_status.strategicPlans = m_strategic.plansMade;
    m_status.strategicInvalidated = m_strategic.invalidations;
    m_status.strategicOverrides = m_strategic.overrides;
    m_status.strategicMicros = m_strategic.totalMicros;
    m_status.strategicMaxMicros = m_strategic.maxMicros;
    if (!m_strategic.lastInvalidation.empty()) m_status.strategicEvent = m_strategic.lastInvalidation;
    if (m_strategic.active()) {
        auto const& sp = m_strategic.plan();
        m_status.strategic = sp.description + ": " + sp.reason;
    }

    bool holdingNow = m_curAction == 1;
    uint64_t key = encodeLevelKey(obs, holdingNow, m_cfg.cellSize);
    // The same discretised state as at the last decision: keep the input -
    // unless a collision may be near, or the player has left the path that
    // was predicted for the input being executed. Then every step is checked.
    bool const watched = m_predictor && m_cfg.usePrediction && !obs.obstacles.empty();
    bool const offPath = watched && m_predictor->hasExecuted() && m_predictor->offPredictedPath(obs);

    // ---- Live plan: does the real state still match what it was made for? ----
    // Checked on every observation, against the state the plan expected for
    // exactly this physics step (position, height, grounded or airborne,
    // mode, speed, gravity) - not merely the x position. A material
    // difference cancels the plan's pending inputs (its learning record is
    // untouched) and forces a new decision from the actual state now.
    bool planInvalidated = false;
    if (m_plan.active()) {
        float const drift = m_predictor ? m_predictor->model().driftPerTick(obs.mode) : 0.f;
        // (2.5 units: the game snaps the player onto surfaces when landing)
        float const tol = 2.5f + 2.f * drift * static_cast<float>(std::max(0, m_plan.age(obs.tick)));
        PlanCheck const chk = m_plan.validate(obs, tol);
        if (chk.hadPlan && !chk.valid && !chk.finished) {
            planInvalidated = true;
            ++m_status.plansInvalidatedThisAttempt;
            m_status.actionsCancelledThisAttempt += static_cast<uint32_t>(chk.cancelled);
            m_status.planEvent = fmt("%s - cancelling %d pending action%s; replanning from the actual state", chk.reason.c_str(),
                                     chk.cancelled, chk.cancelled == 1 ? "" : "s");
        }
    }
    int32_t const planCell = xCell(obs.x, m_cfg.cellSize);
    int const plannedHold = m_plan.holdFor(obs.tick);
    // A still-valid, safe plan made by a trusted model is followed for the
    // step it planned for (also between cells, where the state key does not
    // change), without a new search. It is re-made every planFollowSteps
    // steps, near danger, when the plan runs short, or during an experiment.
    bool const following = watched && !planInvalidated && !m_danger && plannedHold >= 0 && m_plan.plan().followable &&
                           m_plan.age(obs.tick) >= 1 && m_predictor->trusted(obs.mode, obs.mini) &&
                           m_plan.age(obs.tick) < m_cfg.planFollowSteps && m_plan.remaining(obs.tick) > m_cfg.planMinLeft &&
                           m_plan.plan().outcome == Certainty::Safe && !m_activeExp.covers(planCell);
    if (!following && !planInvalidated && key == m_curKey && obs.tick - m_lastDecisionTick < m_cfg.maxStickTicks &&
        !(watched && (m_danger || offPath || m_plan.active())))
        return holdingNow;

    if (!m_steps.empty()) m_steps.back().reward += progressReward(obs.x - m_steps.back().x);

    uint64_t pkey = encodePatternKey(obs, holdingNow);
    QEntry scratch;
    QEntry* e = lookup(key, pkey, scratch);
    QEntry const* pe = m_patterns ? m_patterns->q.find(pkey) : nullptr;
    auto& M = m_status.metrics;

    // Values used for the decision: own experience in this exact state where
    // there is any, otherwise what was learned in similar situations. The
    // prior is read live, so later knowledge about a situation still reaches
    // states that were created before it was learned. It is never mixed with
    // own experience: a state with a record of its own decides by that alone,
    // so learning elsewhere cannot silently change a decision that works here.
    float v[kActions] = {e->q[0], e->q[1]};
    bool fromPrior = false;
    if (!e->visited(0) && !e->visited(1) && pe && (pe->visited(0) || pe->visited(1))) {
        for (int k = 0; k < kActions; ++k) {
            if (pe->visited(k)) v[k] = pe->q[k] * m_cfg.priorWeight;
        }
        fromPrior = true;
    }
    int a = v[1] > v[0] + (fromPrior ? m_cfg.priorPressMargin : kTieEpsilon) ? 1 : 0;
    DecisionSource src = fromPrior ? DecisionSource::Prior : DecisionSource::Policy;
    bool forced = false;
    bool explored = false;
    bool replayed = false;
    bool decided = false;
    std::string reason; // only for notable decisions

    // Trajectory prediction for both actions, used to vet alternatives and to
    // find the deciding decision afterwards.
    bool adoptStrategic = false; // the controller refined the strategic plan (see below)
    Predictor::Advice const* adv = nullptr;
    if (following) {
        a = plannedHold;
        decided = true;
        src = DecisionSource::Plan;
        ++m_status.decisionsFollowingPlan;
    } else if (m_predictor && m_cfg.usePrediction && !obs.obstacles.empty()) {
        // A press timing learned for this kind of obstacle is tried first by
        // the search (and only used if it simulates well).
        if (needsGroundToAct(obs.mode) && obs.onGround && obs.hazard.present && obs.hazard.dx > 0.f && m_patterns) {
            float lead = -1.f;
            auto sit = m_patterns->situationLeads.find(obstacleKey(obs));
            if (sit != m_patterns->situationLeads.end() && sit->second.count >= 2) lead = static_cast<float>(sit->second.mean());
            else {
                auto lt = m_patterns->leads.find(timingKey(obs.mode, obs.speedClass(), obs.mini));
                if (lt != m_patterns->leads.end() && lt->second.count >= 3) lead = static_cast<float>(lt->second.mean());
            }
            float const vx = unitsPerSecond(obs.speedClass()) / static_cast<float>(kTicksPerSecond);
            if (lead > 0.f) m_predictor->setHint(static_cast<int32_t>(std::lround((obs.hazard.dx - lead) / vx)));
        }
        // A strategy from the library for the group ahead goes to the search
        // as a whole plan (simulated first, chosen only if it scores best).
        PendingGroup* const g = m_strategies ? currentGroup(obs) : nullptr;
        if (g && g->choice.usable && !g->pressed && obs.onGround && g->choice.primitive != Primitive::Wait && g->choice.leadSteps >= 0.f) {
            float const vx = unitsPerSecond(obs.speedClass()) / static_cast<float>(kTicksPerSecond);
            float const contact = (g->sit.startX - (obs.x + situationHalfSize(obs.mini))) / vx;
            Plan p;
            p.pressAt = std::max(0, static_cast<int32_t>(std::lround(contact - g->choice.leadSteps)));
            if (g->choice.primitive == Primitive::RejumpChain) p.rejump = true;
            if (g->choice.primitive == Primitive::DoubleJump) p.secondAt = std::max(0, static_cast<int32_t>(std::lround(g->choice.secondSteps)));
            if (g->choice.primitive == Primitive::LongJump) p.holdFor = std::max(12, static_cast<int32_t>(std::lround(g->choice.secondSteps)));
            m_predictor->addSeed(p);
            g->seedContact = contact;
        }
        // The strategic plan for the whole sequence is one more candidate for
        // the search, which also looks as far ahead as the sequence goes, so
        // a first press is judged by what it does to the later obstacles.
        Plan strategicSeed;
        int strategicHorizon = 0;
        adoptStrategic = false;
        bool const strategicSeeded = m_strategic.useful() && m_strategic.seed(obs, strategicSeed, strategicHorizon);
        if (m_strategic.useful()) {
            if (strategicSeeded) m_predictor->addLateSeed(strategicSeed);
            int const remaining = std::max(strategicHorizon, m_strategic.plan().effectiveHorizon - (obs.tick - m_strategic.plan().madeTick));
            m_predictor->extendHorizonOnce(remaining);
        }
        adv = &m_predictor->advise(obs, holdingNow, !m_danger && !offPath);
        if (strategicSeeded && adv->computedTick == obs.tick && !adv->reused) {
            ++m_status.strategicSeeded;
            if (adv->fromLateSeed) ++m_status.strategicMatched;
            // The controller's own search (same look-ahead) found a better
            // input sequence: the strategic plan follows what is executed
            // (below) instead of being invalidated as soon as the two diverge.
            else if (adv->trusted && adv->bestScore >= m_predictor->scoreOf(m_strategic.plan().path)) adoptStrategic = true;
        }
        if (g && adv->strategySeeded && adv->computedTick == obs.tick && !adv->reused) {
            g->seeded = true;
            g->simChose = adv->fromStrategy;
            g->seedOutcome = adv->seedOutcome;
            g->seedCollision = adv->seedCollisionStep;
            g->seedScore = adv->seedScore;
            g->bestScore = adv->bestScore;
        }
        m_status.predictionValid = adv->valid;
        m_status.predictionTrusted = adv->trusted;
        if (adv->valid) {
            m_status.pressOutcome = adv->press;
            m_status.releaseOutcome = adv->release;
            m_status.pressTTC = adv->pressTTC;
            m_status.releaseTTC = adv->releaseTTC;
        }
    }
    bool const trusted = adv && adv->valid && adv->trusted;
    auto predicted = [&](int k) { return k ? adv->press : adv->release; };

    // No trusted prediction and no experience of its own in this state: a
    // strategy that worked for this kind of situation on other levels is
    // played (instead of the generic pattern prior). Still subject to the
    // failure memory and to the safety check where the model can check it.
    if (!following && !trusted && m_strategies && !e->visited(0) && !e->visited(1)) {
        if (PendingGroup* g = currentGroup(obs)) {
            int const ta = transferAction(obs, *g);
            if (ta >= 0) {
                a = ta;
                src = DecisionSource::Transfer;
                fromPrior = false;
                g->played = true;
            }
        }
    }

    int32_t cell = xCell(obs.x, m_cfg.cellSize);
    if (m_activeExp.tests(cell) && cell == m_activeExp.testFrom && !m_expChecked) {
        // A learned test sequence is only meaningful in the state it was
        // planned for: at its first changed cell the player must be exactly
        // where the reference attempt was. Otherwise it is cancelled (and
        // retried on a later attempt), instead of being played into a
        // situation it was not made for.
        m_expChecked = true;
        // Materially different: another game mode, or - for jump-type modes,
        // where a press only acts on the ground - grounded vs. airborne or
        // another height. (Flying-mode tests hold or release over a range of
        // cells, which does not depend on the exact altitude.)
        size_t const ei = static_cast<size_t>(cell - m_activeExp.fromCell);
        uint64_t const ref = m_activeExp.baseKeyAt(cell);
        bool const groundMode = needsGroundToAct(obs.mode);
        bool const differs = ref != 0 && ei < m_activeExp.baseYs.size() &&
                             (keyMode(ref) != keyMode(key) ||
                              (groundMode && ((m_activeExp.baseGround[ei] != 0) != obs.onGround ||
                                              std::fabs(m_activeExp.baseYs[ei] - obs.y) > 4.f)));
        if (differs) {
            ++m_status.experimentsCancelled;
            m_status.planEvent = fmt("Experiment cancelled: the state at its first changed cell differs from the reference attempt (%s)",
                                     m_activeExp.description.c_str());
            // Its search was built on a reference attempt the AI no longer
            // plays like: start that search again from current play (the
            // next death there) instead of re-testing the outdated sequence.
            for (auto sr = m_searches.begin(); sr != m_searches.end();)
                sr = std::abs(sr->first - m_activeExp.hotspot) <= 1 ? m_searches.erase(sr) : std::next(sr);
            for (auto sr = m_parked.begin(); sr != m_parked.end();)
                sr = std::abs(sr->first - m_activeExp.hotspot) <= 1 ? m_parked.erase(sr) : std::next(sr);
            m_activeExp = {};
        }
    }
    if (!following && m_activeExp.tests(cell)) {
        // The one change the experiment is testing.
        a = m_activeExp.actionAt(cell);
        forced = true;
        decided = true;
        src = DecisionSource::Experiment;
    } else if (!following && m_activeExp.covers(cell) && m_activeExp.baseKeyAt(cell) == key) {
        // Same state as in the reference attempt: do the same, so the
        // experiment's result is a fair comparison. In any other state the
        // decision is made normally.
        a = m_activeExp.actionAt(cell);
        replayed = true;
        decided = true;
        src = DecisionSource::Reference;
    }

    if (!following) {
        // Contextual failure memory: has this decision, in exactly this state,
        // been identified as the cause of deaths, again and again? This
        // applies to every source, including experiments and reference
        // replays: nothing repeats a known mistake in the same situation.
        int const b = 1 - a;
        int const thr = std::max(1, m_cfg.repeatFailureThreshold);
        if (e->streak[a] >= thr && e->streak[b] < e->streak[a]) {
            bool first = std::find(m_avoidedThisAttempt.begin(), m_avoidedThisAttempt.end(), key) == m_avoidedThisAttempt.end();
            if (first) {
                m_avoidedThisAttempt.push_back(key);
                ++M.repeatedFailuresDetected;
            }
            char const* failed = failedPhrase(obs.mode, obs.onGround, holdingNow, a);
            char const* alt = alternativePhrase(obs.mode, obs.onGround, holdingNow, b);
            if (trusted && predicted(b) == Certainty::Collision && predicted(a) != Certainty::Collision) {
                // The alternative is predicted to die as well: switching cannot
                // help here, the cause must be earlier (credit assignment
                // will look there).
                if (first) ++M.alternativesVetoed;
                reason = fmt("Repeated failure detected: %s failed %u times here, but %s is predicted to collide; the cause is probably earlier.",
                             failed, static_cast<unsigned>(e->streak[a]), alt);
            } else {
                if (first) ++M.alternativesTried;
                reason = fmt("Repeated failure detected: %s failed %u times here. Testing %s%s%s.", failed,
                             static_cast<unsigned>(e->streak[a]), alt, trusted ? ", predicted " : "",
                             trusted ? certaintyName(predicted(b)) : "");
                a = b;
                decided = true;
                forced = false;
                replayed = false;
                src = DecisionSource::AvoidRepeat;
            }
        } else if (!decided && e->streak[b] == 0 && e->passes[b] >= static_cast<uint16_t>(std::max(1, m_cfg.knownGoodPasses)) &&
                   (e->passes[a] == 0 || e->streak[a] > 0)) {
            // The other action reliably got through from exactly this state and
            // has not failed since; the learned choice has no such record.
            // (Values here may have been disturbed by deaths further on.)
            // Exploration may still try something else when stuck.
            reason = fmt("Known good here: %s got through %u times; %s has no such record.",
                         alternativePhrase(obs.mode, obs.onGround, holdingNow, b), static_cast<unsigned>(e->passes[b]),
                         failedPhrase(obs.mode, obs.onGround, holdingNow, a));
            a = b;
            src = DecisionSource::KnownGood;
            ++M.knownGoodKept;
        }
    }

    if (!decided && m_cfg.learningEnabled) {
        float eps = epsilonAt(obs.x);
        if (eps > 0.f && std::uniform_real_distribution<float>(0.f, 1.f)(m_rng) < eps) {
            a = 1 - a;
            explored = true;
            src = DecisionSource::Explore;
        }
    }

    // Trajectory prediction: no learned preference here yet -> follow the
    // planner's best plan.
    bool planned = false;
    bool shielded = false;
    if (trusted) {
        bool opinion = (e->visited(0) || e->visited(1)) && std::fabs(e->q[0] - e->q[1]) > 1e-3f;
        if (!decided && !explored && !opinion) {
            a = adv->bestPresses ? 1 : 0;
            planned = true;
            src = DecisionSource::Planner;
        }
    }

    // Never toggle faster than the configured minimum (prevents input spam).
    // Applied before the safety check, so that the check judges the action
    // that will really be executed; the check itself may override it.
    if (!following && a != m_curAction && obs.tick - m_lastToggleTick < m_cfg.minToggleTicks) {
        a = m_curAction;
        forced = false;
        explored = false;
        replayed = false;
        planned = false;
        src = DecisionSource::Debounce;
    }

    // ---- Safety check on the action about to be executed ----
    // The prediction of an action is its best continuation ("press now, then
    // release in 2 steps"). The controller does not promise to follow that
    // continuation, so an action is only accepted while there is enough time
    // left to change course: when keeping the current input would require a
    // change within the slack, the change is made now. An action predicted
    // to collide is replaced whenever the other one is not.
    SafetyRecord rec;
    rec.tick = obs.tick;
    rec.x = obs.x;
    rec.y = obs.y;
    rec.vy = obs.vy;
    rec.mode = obs.mode;
    rec.onGround = obs.onGround;
    rec.holdingBefore = holdingNow;
    rec.proposed = static_cast<uint8_t>(a);
    rec.proposedSource = src;
    rec.hazardAhead = obs.hazard.present;
    rec.hazardDx = obs.hazard.dx;
    rec.hazardDy = obs.hazard.dy;
    rec.predictionValid = adv && adv->valid;
    rec.trusted = trusted;
    if (rec.predictionValid) {
        rec.never = adv->never;
        rec.neverTTC = adv->neverTTC;
        if (trusted) ++m_status.decisionsTrusted;
        else ++m_status.decisionsUntrusted;
    }
    bool overridden = false;
    if (trusted) {
        rec.predictionCurrent = adv->computedTick == obs.tick && !adv->reused;
        rec.predictionAge = obs.tick - adv->computedTick;
        for (int k = 0; k < kActions; ++k) {
            rec.outcome[k] = adv->outcome(k);
            rec.ttc[k] = adv->ttc(k);
            rec.keepFor[k] = adv->keepFor(k);
        }
        rec.keep = adv->keep;
        rec.keepTTC = adv->keepTTC;
        // Rank both candidates by safety:
        //   0 predicted collision
        //   1 possible hit (a near miss within the error margin) in the horizon
        //   2 no hit predicted, but the input would have to change within the slack
        //   3 no hit predicted, but an unmodelled object / unconfirmed surface is ahead
        //   4 safe with time to spare
        // "Must change within the slack" only applies to keeping the current
        // input, or to any input in continuously steered modes (ship, wave),
        // where whatever is chosen now is held until the next change. It does
        // not apply to the planner's own choice: the planner follows its plan
        // and re-plans every step while danger is near, so it makes that
        // change itself, at the planned moment.
        int32_t const slack = std::max(1, m_cfg.safetySlackSteps);
        bool const steered = obs.mode == GameMode::Ship || obs.mode == GameMode::Wave;
        bool const planCommitted = src == DecisionSource::Planner;
        int const proposedAction = a;
        auto margin = [&](int k) {
            int32_t m = adv->keepForSafe(k);
            return m >= 0 ? m : adv->keepFor(k);
        };
        auto rank = [&](int k) {
            Certainty o = adv->outcome(k);
            if (o == Certainty::Collision) return 0;
            if (o == Certainty::Uncertain && adv->risk(k) >= 0) return 1;
            bool continuing = (k == 1) == holdingNow;
            if ((continuing || steered) && margin(k) < slack && !(planCommitted && k == proposedAction)) return 2;
            return o == Certainty::Safe ? 4 : 3;
        };
        auto describe = [&](int k) {
            switch (rank(k)) {
                case 0: return fmt("%s collides in %d steps", buttonName(obs.mode, k), adv->ttc(k) + 1);
                case 1: return fmt("%s risks a hit in %d steps", buttonName(obs.mode, k), adv->risk(k) + 1);
                case 2: return fmt("keeping %s leaves %d step(s) to change course", buttonName(obs.mode, k), std::max(0, margin(k)));
                case 3: return fmt("%s has no predicted hit, but passes something the model does not simulate", buttonName(obs.mode, k));
                default:
                    if (adv->outcome(k) == Certainty::Safe) return fmt("%s is safe (input can stay for %d steps)", buttonName(obs.mode, k), margin(k));
                    return fmt("%s has no definite hit, but is uncertain further ahead", buttonName(obs.mode, k));
            }
        };
        int const b = 1 - a;
        int const ra = rank(a);
        int const rb = rank(b);
        rec.alternativeAvailable = rb > 0;
        char const* fresh = rec.predictionCurrent ? "current prediction" : "prediction from an earlier step";
        if (ra == 4) {
            rec.verdict = margin(a) < m_cfg.dangerWindowSteps || adv->outcome(a) != Certainty::Safe ? SafetyVerdict::Kept : SafetyVerdict::Clear;
            if (rec.verdict == SafetyVerdict::Kept) rec.reason = fmt("%s kept: %s (%s)", buttonName(obs.mode, a), describe(a).c_str(), fresh);
        } else if (rb > ra) {
            // The other action is safer: execute it. A rejected definite
            // collision is also learned as an imagined death right away.
            if (ra == 0 && m_cfg.learningEnabled) {
                float target = -m_cfg.deathPenalty * std::pow(m_cfg.gammaPerTick, static_cast<float>(std::max(0, adv->ttc(a))));
                e->q[a] += m_cfg.alpha() * m_cfg.imaginedScale * (target - e->q[a]);
                if (e->n[a] < 0xFFFF) ++e->n[a]; // it is evidence about this action
            }
            rec.verdict = SafetyVerdict::Overridden;
            rec.reason = fmt("Safety override: %s; executing %s instead: %s (%s)", describe(a).c_str(), buttonName(obs.mode, b),
                             describe(b).c_str(), fresh);
            a = b;
            overridden = true;
        } else if (ra == 0) {
            // Every candidate collides: take the one that collides later (more
            // time for the next step to find a way), and say so.
            int pick = adv->ttc(b) > adv->ttc(a) ? b : a;
            if (std::min(adv->ttc(0), adv->ttc(1)) >= m_cfg.farCollisionSteps) {
                // Both collide only far ahead: one input cannot clear
                // everything in view yet, later decisions still can. Same
                // choice, but this is not "no way out".
                rec.verdict = pick != a ? SafetyVerdict::Overridden : SafetyVerdict::Kept;
                rec.reason = fmt("Both options collide only far ahead (release in %d, press in %d steps); executing %s (collides later), "
                                 "later decisions can still act",
                                 adv->ttc(0) + 1, adv->ttc(1) + 1, buttonName(obs.mode, pick));
            } else {
                rec.verdict = SafetyVerdict::NoSafeOption;
                rec.reason = fmt("No safe option: release collides in %d, press in %d steps; executing %s (collides later)",
                                 adv->ttc(0) + 1, adv->ttc(1) + 1, buttonName(obs.mode, pick));
            }
            overridden = pick != a;
            a = pick;
        } else {
            // Not comfortable, but the other action is no better: keep it and
            // check again next step.
            rec.verdict = SafetyVerdict::Kept;
            rec.reason = fmt("%s kept: %s; the other is no better (%s) (%s)", buttonName(obs.mode, a), describe(a).c_str(),
                             describe(b).c_str(), fresh);
        }
        if (overridden) {
            shielded = true;
            forced = false;
            explored = false;
            replayed = false;
            planned = false;
            src = DecisionSource::Shield;
            reason = rec.reason;
        }
        m_danger = rec.verdict != SafetyVerdict::Clear;

        // ---- No input where none is needed ----
        // A press (a jump, or holding into a landing re-jump) is only made
        // when it is needed now: when not pressing is fully safe for now
        // (rank 4) and the planner's best-timed plan does not press now
        // either, the press is dropped, whoever proposed it. If a jump is
        // needed later, the planner times it (best clearance, fewest inputs).
        // Not for steered modes (ship, wave: the button sets the altitude),
        // not for an experiment's deliberate test, and not where not pressing
        // already failed in exactly this state (the prediction would be wrong
        // here: reality wins).
        bool const pressInput = a == 1 && (!holdingNow || needsGroundToAct(obs.mode));
        bool const canChange = m_curAction == 0 || obs.tick - m_lastToggleTick >= m_cfg.minToggleTicks;
        if (!steered && pressInput && canChange && src != DecisionSource::Experiment && !adv->bestPresses && rank(0) == 4 &&
            e->streak[0] == 0) {
            rec.verdict = SafetyVerdict::NoInputNeeded;
            if (adv->never == Certainty::Safe)
                rec.reason = fmt("No jump needed: not pressing is predicted clear for %d steps (pressing now: %s); %s proposed it",
                                 m_predictor->neverPath().steps, describe(1).c_str(), decisionSourceName(rec.proposedSource));
            else
                rec.reason = fmt("No jump yet: waiting is safe, the best plan presses in %d steps (pressing now: %s); %s proposed it",
                                 adv->bestPressAt, describe(1).c_str(), decisionSourceName(rec.proposedSource));
            a = 0;
            src = DecisionSource::Economy;
            shielded = false;
            forced = false;
            explored = false;
            replayed = false;
            planned = false;
            reason = rec.reason;
            ++m_status.pressesAvoidedThisAttempt;
            m_danger = margin(0) < m_cfg.dangerWindowSteps;
        }
        m_status.trust.clear();
    } else if (!following && m_predictor && m_cfg.usePrediction && (!adv || !adv->valid) && m_predictor->trusted(obs.mode, obs.mini) && a == 1 &&
               !holdingNow && obs.onGround &&
               needsGroundToAct(obs.mode) && src != DecisionSource::Experiment && e->streak[0] == 0 &&
               (m_curAction == 0 || obs.tick - m_lastToggleTick >= m_cfg.minToggleTicks)) {
        // Nothing to jump over within the whole prediction horizon.
        m_danger = false;
        rec.verdict = SafetyVerdict::NoInputNeeded;
        rec.trusted = true;
        rec.reason = fmt("No jump needed: nothing ahead within the prediction horizon; %s proposed a jump",
                         decisionSourceName(rec.proposedSource));
        a = 0;
        src = DecisionSource::Economy;
        forced = explored = replayed = planned = shielded = false;
        reason = rec.reason;
        ++m_status.pressesAvoidedThisAttempt;
        m_status.trust.clear();
    } else {
        m_danger = false;
        if (rec.predictionValid) {
            // Simulated, but not trusted: the decision is the policy's alone,
            // and the record says so (and why).
            m_status.trust = m_predictor->trustReason(obs.mode, obs.mini);
            rec.reason = "Prediction not used: " + m_status.trust;
        }
    }
    if (following) {
        rec.verdict = SafetyVerdict::Clear;
        rec.trusted = true;
        rec.predictionValid = true;
        rec.reason = fmt("Following the plan made %d steps ago (state as expected: height %+.1f, %s)", m_plan.age(obs.tick),
                         m_plan.lastCheck.dy, obs.onGround ? "on the ground" : "in the air");
    }
    rec.executed = static_cast<uint8_t>(a);
    if (a == 1 && m_curAction == 0) ++m_status.pressesThisAttempt;
    // The executed action's predicted path becomes the live plan (made by a
    // trusted model from exactly this state); without one there is no plan.
    if (!following && m_predictor) {
        // the path of the action executed now (its first planned input must be exactly that action)
        PredictedPath const& mine = a ? m_predictor->pressPath() : m_predictor->releasePath();
        size_t const first = static_cast<size_t>(m_predictor->inputLatency());
        bool const matches = mine.holds.size() > first && mine.holds[first] == static_cast<uint8_t>(a);
        if (adv && trusted && adv->computedTick == obs.tick && matches) {
            if (adoptStrategic) m_strategic.adoptController(obs, mine);
            bool const byPrediction = src == DecisionSource::Planner || src == DecisionSource::Shield || src == DecisionSource::Economy;
            m_plan.adopt(mine, obs, obs.tick, adv->fromSeed ? PlanSource::Learned : PlanSource::Search, *adv, byPrediction);
        } else if (m_plan.active()) {
            m_plan.release(obs.tick, trusted ? "replaced" : "no trusted prediction: the learned policy decides");
        }
    }
    if (rec.verdict == SafetyVerdict::Overridden) ++m_status.overridesThisAttempt;
    if (rec.verdict == SafetyVerdict::NoSafeOption) ++m_status.noSafeOptionThisAttempt;
    if (rec.verdict != SafetyVerdict::Clear && rec.verdict != SafetyVerdict::NoPrediction && rec.verdict != SafetyVerdict::NoInputNeeded) {
        if (m_safetyLog.size() >= kSafetyLogSize) m_safetyLog.erase(m_safetyLog.begin(), m_safetyLog.begin() + kSafetyLogSize / 2);
        m_safetyLog.push_back(rec);
    }
    m_status.safety = std::move(rec);

    if (adv) m_predictor->noteExecuted(a == 1);
    if (shielded) {
        ++m_status.shieldsThisAttempt;
        ++m_status.shieldsTotal;
    }
    if (planned) ++m_status.plannedThisAttempt;

    if (a != m_curAction) {
        m_lastToggleTick = obs.tick;
        m_inputs.emplace_back(obs.x, static_cast<uint8_t>(a));
        if (a == 1 && obs.onGround) {
            PendingGroup* g = currentGroup(obs);
            // A first press only counts for the group if that group is what is
            // in front of the player (not an obstacle that was never tracked).
            // Later presses (on top of the group) are its second jump.
            if (g && !g->pressed && !(m_sit.valid && std::fabs(m_sit.startX - g->sit.startX) < 1.f)) g = nullptr;
            if (g) {
                float const vx = unitsPerSecond(obs.speedClass()) / static_cast<float>(kTicksPerSecond);
                if (!g->pressed) {
                    g->pressed = true;
                    g->primitive = Primitive::Jump;
                    g->lead = std::max(0.f, (g->sit.startX - (obs.x + situationHalfSize(obs.mini))) / vx);
                    g->pressTick = obs.tick;
                    // the verdict on the library's suggestion is known now
                    if (!g->reported) reportGroup(*g, trusted);
                } else if (g->landed && g->primitive == Primitive::Jump) {
                    g->primitive = Primitive::DoubleJump;
                    g->second = static_cast<float>(obs.tick - g->landTick);
                }
            }
        }
        if (a == 1 && obs.hazard.present && obs.hazard.dx >= 0.f && obs.hazard.dx < 6.f * kBlock && m_leads.size() < 8) {
            m_leads.push_back({obs.x + obs.hazard.dx, obs.hazard.dx, timingKey(obs.mode, obs.speedClass(), obs.mini), obstacleKey(obs)});
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
    s.replayed = replayed;
    s.source = src;
    if (trusted && adv->computedTick == obs.tick) {
        s.pred[0] = static_cast<int8_t>(adv->release);
        s.pred[1] = static_cast<int8_t>(adv->press);
    }
    m_steps.push_back(s);
    if (replayed) ++M.referenceSteps;
    else ++M.independentSteps;

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
    m_status.source = src;
    if (!reason.empty()) m_status.reason = std::move(reason);
    for (int k = 0; k < kActions; ++k) {
        m_status.fails[k] = e->fails[k];
        m_status.passes[k] = e->passes[k];
        m_status.streak[k] = e->streak[k];
    }
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
        if (m_cutAtPivot && i > m_pivot && s.pred[0] == static_cast<int8_t>(Certainty::Collision) &&
            s.pred[1] == static_cast<int8_t>(Certainty::Collision)) {
            // Past the deciding decision both actions were predicted to die,
            // so the death says as much about the other action: learn it too,
            // instead of making the untried action look better than it is.
            int o = 1 - s.action;
            e.q[o] += alpha * m_cfg.imaginedScale * (target - e.q[o]);
            if (e.n[o] < 0xFFFF) ++e.n[o];
        }

        if (m_patterns) {
            QEntry& pe = m_patterns->q.getOrCreate(s.pkey);
            pe.q[s.action] += pAlpha * (target - pe.q[s.action]);
            if (pe.n[s.action] < 0xFFFF) ++pe.n[s.action];
        }

        G = target;
        nextMax = e.best();
        nextTick = s.tick;
        // The deciding decision had a way through (its alternative was
        // predicted to survive): decisions before it are judged by the best
        // value available at that point, not by the death that followed a
        // later mistake (Watkins-style trace cut).
        if (m_cutAtPivot && i == m_pivot) G = nextMax;
    }
}

void Agent::applyNewBestBonus(float endX) {
    if (!m_level || m_fromCheckpoint || m_steps.empty() || m_cfg.newBestReward <= 0.f) return;
    if (m_level->totalAttempts == 0) return; // the first attempt has no record to beat
    float prev = m_level->bestX;
    if (endX < prev + kBlock) return;
    // Rewarded once, on the decision that carried the player past the old
    // record. The record then moves, so the same achievement is never paid twice.
    for (size_t i = 0; i < m_steps.size(); ++i) {
        if (m_steps[i].x > prev) {
            m_steps[i ? i - 1 : 0].reward += m_cfg.newBestReward;
            break;
        }
    }
    ++m_status.metrics.newBests;
}

void Agent::assignCredit(int32_t endTick, FailureReport const* report, bool completed) {
    m_pivot = kNone;
    m_cutAtPivot = false;
    if (m_steps.empty() || !m_level) return;
    auto& M = m_status.metrics;
    size_t const n = m_steps.size();
    int const thr = std::max(1, m_cfg.repeatFailureThreshold);
    auto const collision = static_cast<int8_t>(Certainty::Collision);
    auto const safe = static_cast<int8_t>(Certainty::Safe);
    auto recentlyFailed = [&](Step const& s, int a, int atLeast) {
        auto const* e = m_level->q.find(s.key);
        return e && e->streak[a] >= atLeast;
    };

    size_t pivot = kNone;
    enum { None, Prediction, Analysis } how = None;
    bool mispredicted = false;
    bool walkedPredictions = false;
    if (!completed && report) {
        // 1. Did the model see this death coming? A decision right before the
        // death whose own path was predicted safe means the physics model was
        // wrong: that is corrected by physics learning, and the predictions
        // are not used to blame a decision.
        for (size_t i = n; i-- > 0;) {
            Step const& s = m_steps[i];
            if (endTick - s.tick > 12) break;
            if (s.pred[0] < 0) continue;
            mispredicted = s.pred[s.action] == safe;
            break;
        }
        if (mispredicted) ++M.mispredictedDeaths;

        // 2. With trusted predictions: the deciding decision is the last one
        // whose alternative still had a way through. Later decisions could not
        // have saved the attempt (both actions predicted to collide), so they
        // are not blamed. A predicted way through that already failed
        // repeatedly in reality does not count (prediction vs. reality).
        if (!mispredicted) {
            for (size_t i = n; i-- > 0;) {
                Step const& s = m_steps[i];
                if (endTick - s.tick > m_cfg.blameTicks) break;
                if (s.pred[0] < 0) break; // no prediction: no evidence from here on back
                walkedPredictions = true;
                int alt = 1 - s.action;
                if (s.pred[alt] != collision && !recentlyFailed(s, alt, thr)) {
                    pivot = i;
                    how = Prediction;
                    break;
                }
                ++M.doomedSkipped;
            }
        }

        // 3. Otherwise: the failure analysis' suspect (from what really happened).
        if (pivot == kNone && !walkedPredictions) {
            FailureReport const& R = *report;
            if (R.responsibleBlock >= 0 && R.responsibleBlock < static_cast<int>(R.blocks.size())) {
                auto const& b = R.blocks[static_cast<size_t>(R.responsibleBlock)];
                switch (R.kind) {
                    case FailureKind::TooLate: pivot = b.firstStep > 0 ? b.firstStep - 1 : kNone; break;
                    case FailureKind::HeldTooLong: pivot = b.lastStep; break;
                    case FailureKind::ReleasedTooLong: pivot = b.lastStep + 1 < n ? b.lastStep + 1 : kNone; break;
                    default: pivot = b.firstStep; break;
                }
            } else {
                // No press to blame: the last chance to press before the death.
                for (size_t i = n; i-- > 0;) {
                    Step const& s = m_steps[i];
                    if (endTick - s.tick > m_cfg.blameTicks) break;
                    if (s.action == 0 && (!needsGroundToAct(s.mode) || s.onGround)) {
                        pivot = i;
                        break;
                    }
                }
            }
            // Where both actions have already failed in this state, neither is
            // the cause: the trouble starts earlier.
            while (pivot != kNone && pivot < n && recentlyFailed(m_steps[pivot], 0, 1) && recentlyFailed(m_steps[pivot], 1, 1)) {
                ++M.doomedSkipped;
                size_t j = pivot;
                pivot = kNone;
                while (j-- > 0) {
                    Step const& s = m_steps[j];
                    if (endTick - s.tick > m_cfg.blameTicks) break;
                    if (s.key != m_steps[j + 1].key && (!needsGroundToAct(s.mode) || s.onGround || s.action == 1)) {
                        pivot = j;
                        break;
                    }
                }
            }
            if (pivot >= n) pivot = kNone;
            if (pivot != kNone) how = Analysis;
        }
    }

    float const len = m_level->levelLength > 1.f ? m_level->levelLength : 1.f;
    if (pivot != kNone) {
        Step const& s = m_steps[pivot];
        QEntry const* known = m_level->q.find(s.key);
        uint8_t streak = known ? known->streak[s.action] : 0;
        if (streak >= thr) ++M.sameMistakeRepeated;
        if (m_cfg.learningEnabled) {
            QEntry& e = m_level->q.getOrCreate(s.key);
            e.noteFail(s.action);
            streak = e.streak[s.action];
            if (m_patterns) m_patterns->q.getOrCreate(s.pkey).noteFail(s.action);
        }
        bool holdingBefore = pivot > 0 && m_steps[pivot - 1].action == 1;
        m_pivot = pivot;
        m_cutAtPivot = how == Prediction;
        if (how == Prediction) ++M.pivotsByPrediction;
        else ++M.pivotsByAnalysis;
        m_status.lastCredit = fmt("Blamed %s at %.1f%% (%s; failed %u times here)",
                                  failedPhrase(s.mode, s.onGround, holdingBefore, s.action), s.x / len * 100.f,
                                  how == Prediction ? "prediction: the other action still had a way through"
                                                    : "failure analysis",
                                  static_cast<unsigned>(streak));
    } else if (!completed && report) {
        ++M.pivotsUnknown;
        m_status.lastCredit = walkedPredictions
                                  ? "No decision blamed: every decision in reach was already doomed (cause is further back)"
                                  : "No decision blamed: no evidence which decision caused it";
    }
    if (mispredicted) m_status.lastCredit += "; the path had been predicted safe, so the physics model was corrected and the prediction not used for blame";

    // Passes: decisions the attempt got safely past (well before the death,
    // and before the deciding decision).
    if (m_cfg.learningEnabled) {
        uint64_t lastKey = ~uint64_t(0);
        uint8_t lastA = 0xFF;
        for (size_t i = 0; i < n; ++i) {
            Step const& s = m_steps[i];
            if (!completed && endTick - s.tick <= m_cfg.passTicks) break;
            if (m_pivot != kNone && i >= m_pivot) break;
            if (s.key == lastKey && s.action == lastA) continue;
            lastKey = s.key;
            lastA = s.action;
            m_level->q.getOrCreate(s.key).notePass(s.action);
            if (m_patterns) m_patterns->q.getOrCreate(s.pkey).notePass(s.action);
            if (s.source == DecisionSource::AvoidRepeat) ++M.alternativesThatHelped;
        }
    }
}

void Agent::commitExperiment() {
    // The experiment worked: make sure the greedy policy now prefers what the
    // experiment did, so the improvement is kept on the next attempts. That is
    // the tested change plus the decisions leading into it that were taken in
    // exactly the reference attempt's states (never anything outside them).
    for (auto const& s : m_steps) {
        if (!s.forced && !s.replayed) continue;
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
    t.keys.assign(t.actions.size(), 0);
    t.ys.assign(t.actions.size(), 0.f);
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
        t.keys[i] = covered ? m_steps[j].key : 0;
        t.ys[i] = covered ? m_steps[j].y : 0.f;
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
            for (auto s = m_parked.begin(); s != m_parked.end();) {
                s = std::abs(s->first - m_activeExp.hotspot) <= 1 ? m_parked.erase(s) : std::next(s);
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
    }

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
    // Bounded memory: forget the least experienced states first.
    if (L.q.size() > LevelMemory::kMaxStates) L.q.prune(LevelMemory::kMaxStates * 9 / 10);
    if (m_patterns && m_patterns->q.size() > PatternMemory::kMaxStates) m_patterns->q.prune(PatternMemory::kMaxStates * 9 / 10);
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
    resolveGroups(endX, true, false);

    assignCredit(last.tick, &rep, false);
    if (m_cfg.learningEnabled) {
        applyNewBestBonus(endX);
        backwardUpdate(-m_cfg.deathPenalty, true, 0.f, last.tick);
    }
    finishAttempt(endX, last.tick, true, false, rep.kind);

    m_focusFrom = rep.windowStartX;
    m_focusTo = rep.deathX + kBlock;
    if (m_pivot != kNone) {
        // Explore from the deciding decision on, even when it lies before the
        // analysis window (the trouble started earlier than the death).
        // (Still never before the solved part: see protectedUpTo.)
        m_focusFrom = std::min(m_focusFrom, m_steps[m_pivot].x - kBlock);
    }
    if (m_cfg.learningEnabled) {
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
    resolveGroups(endX, false, true);
    assignCredit(last.tick, nullptr, true);
    if (m_cfg.learningEnabled) {
        applyNewBestBonus(endX);
        backwardUpdate(m_cfg.completionReward, true, 0.f, last.tick);
    }
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
    m_avoidedThisAttempt.clear();
    m_pivot = kNone;
    m_cutAtPivot = false;
    m_steps.clear();
    m_inputs.clear();
    m_leads.clear();
    resolveGroups(0.f, false, false);
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
        int32_t const key = it->first;
        bool const removeTried = std::find(m_removeTried.begin(), m_removeTried.end(), key) != m_removeTried.end();
        // the press being varied: the latest one starting at the search's cell
        PressBlock const* varied = nullptr;
        for (auto const& b : search->base.report.blocks)
            if (xCell(b.startX, m_cfg.cellSize) == search->blockStart) varied = &b;
        if (search->type == ExperimentType::ShiftBlock && needsGroundToAct(report.mode) && search->tries >= 2 && !search->exhausted &&
            !removeTried && varied && walkingUnderClears(rep, *varied, m_lastObs.mini)) {
            // Two timing variations of this jump failed: before trying more
            // timings, test not jumping there at all (the jump itself may be
            // the mistake). The timing search is paused, not lost.
            m_removeTried.push_back(key);
            Search r;
            r.type = ExperimentType::RemovePress;
            r.blockStart = search->blockStart;
            r.blockEnd = search->blockEnd;
            r.base = search->base;
            r.lastEndX = search->lastEndX;
            r.lastEndY = search->lastEndY;
            m_parked[key] = std::move(*search);
            *search = std::move(r);
        } else if (search->type == ExperimentType::RemovePress && search->exhausted) {
            // Not pressing did not work either: the jump is needed. Continue
            // its timing search where it was paused.
            auto p = m_parked.find(key);
            if (p != m_parked.end()) {
                *search = std::move(p->second);
                m_parked.erase(p);
            }
        }
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
            // widen the window and look further back. (Not after "remove the
            // press" failed: then the press is needed, and its timing is next.)
            if (search->type != ExperimentType::RemovePress) ++hot.escalation;
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

    // The jump carried the player into something that was not below it
    // (above, or level with it): the jump itself may be the mistake. Test not
    // pressing at all, once per trouble spot, before varying its timing.
    if (needsGroundToAct(mode) && block >= 0 && report.needDir <= 0 &&
        walkingUnderClears(report, report.blocks[static_cast<size_t>(block)], last && last->mini) &&
        (report.kind == FailureKind::HeldTooLong || report.kind == FailureKind::TooEarly || report.kind == FailureKind::TooLate) &&
        std::find(m_removeTried.begin(), m_removeTried.end(), report.hotspot) == m_removeTried.end()) {
        m_removeTried.push_back(report.hotspot);
        useBlock(ExperimentType::RemovePress, 0);
        return true;
    }

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
    if (s.type == ExperimentType::RemovePress) {
        s.exhausted = true; // a single test: not pressing either works or it does not
        return;
    }
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
    ex.baseKeys.resize(ex.actions.size());
    ex.baseYs.resize(ex.actions.size(), 0.f);
    ex.baseGround.resize(ex.actions.size(), 1);
    for (int32_t c = ex.fromCell; c <= ex.toCell; ++c) {
        size_t const i = static_cast<size_t>(c - ex.fromCell);
        ex.actions[i] = base.at(c);
        ex.baseKeys[i] = base.keyAt(c);
        size_t const bi = static_cast<size_t>(c - base.startCell);
        if (c >= base.startCell && bi < base.ys.size()) {
            ex.baseYs[i] = base.ys[bi];
            ex.baseGround[i] = base.ground[bi];
        }
    }
    // The cells that make up the change being tested.
    switch (s.type) {
        case ExperimentType::ShiftBlock:
            ex.testFrom = std::min(s.blockStart, s.blockStart + s.shift);
            ex.testTo = std::max(s.blockEnd, s.blockEnd + s.shift);
            break;
        case ExperimentType::ShiftStart:
            ex.testFrom = std::min(s.blockStart, s.blockStart + s.shift);
            ex.testTo = s.blockEnd;
            break;
        case ExperimentType::ShiftEnd:
            ex.testFrom = s.blockStart;
            ex.testTo = std::max(s.blockEnd, s.blockEnd + s.shift);
            break;
        case ExperimentType::RemovePress:
            ex.testFrom = s.blockStart;
            ex.testTo = std::max(s.blockEnd, deathCell + 1);
            break;
        case ExperimentType::HoldRange:
        case ExperimentType::ReleaseRange:
            ex.testFrom = s.blockStart + s.shift;
            ex.testTo = s.blockEnd;
            break;
        case ExperimentType::InsertPress:
            ex.testFrom = s.blockStart + s.shift;
            ex.testTo = s.blockStart + s.shift + s.insertLen - 1;
            break;
        default: break;
    }

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
        case ExperimentType::RemovePress:
            // no press from the removed one until past the place of death
            set(s.blockStart, std::max(s.blockEnd, deathCell + 1), 0);
            ex.description = fmt("Removed the press at %.1f%%: testing not jumping there", at);
            break;
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

void QTable::prune(size_t keep) {
    if (m_map.size() <= keep) return;
    std::vector<std::pair<uint32_t, uint64_t>> score;
    score.reserve(m_map.size());
    for (auto const& [k, e] : m_map) {
        uint32_t sc = static_cast<uint32_t>(e.n[0]) + e.n[1] + e.passes[0] + e.passes[1] + 2u * (e.fails[0] + e.fails[1]);
        score.emplace_back(sc, k);
    }
    size_t drop = m_map.size() - keep;
    std::nth_element(score.begin(), score.begin() + static_cast<std::ptrdiff_t>(drop), score.end());
    for (size_t i = 0; i < drop; ++i) m_map.erase(score[i].second);
}

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

// ---------------------------------------------------------------------------
// Cross-level transfer (strategy library)

void Agent::noteTransfer(std::string msg) {
    m_status.transferEvent = msg;
    m_status.transferLog.push_back(std::move(msg));
    if (m_status.transferLog.size() > 6) m_status.transferLog.erase(m_status.transferLog.begin());
}

Agent::PendingGroup* Agent::currentGroup(Observation const& obs) {
    // The nearest group not yet passed.
    float const h = situationHalfSize(obs.mini);
    PendingGroup* best = nullptr;
    for (auto& g : m_groups) {
        if (g.sit.endX < obs.x - h) continue;
        if (!best || g.sit.startX < best->sit.startX) best = &g;
    }
    return best;
}

void Agent::trackGroups(Observation const& obs) {
    m_sit = Situation{};
    if (!m_strategies || !m_level) return;
    if (obs.mode != GameMode::Cube && obs.mode != GameMode::Robot) {
        m_groups.clear(); // another mode: what was being tracked no longer applies
        return;
    }
    float const h = situationHalfSize(obs.mini);
    auto& T = m_status.transfer;
    // Passed groups: success.
    for (auto it = m_groups.begin(); it != m_groups.end();) {
        if (obs.x - h > it->sit.endX + 2.f) {
            if (!it->reported) reportGroup(*it, false);
            // Only what was really done for this group is recorded: passing it
            // in the air without a press of its own (an earlier jump cleared
            // it) says nothing about how to handle it.
            if (it->pressed || !it->airborne) {
                if (m_cfg.learningEnabled) m_strategies->record(it->sit, it->primitive, it->lead, it->second, true, m_level->key);
                ++T.successes;
            } else {
                ++T.unattributed;
            }
            it = m_groups.erase(it);
        } else {
            ++it;
        }
    }
    for (auto& g : m_groups) {
        if (!g.pressed && !obs.onGround) g.airborne = true;
        if (g.pressed && !g.landed && obs.onGround && obs.tick > g.pressTick + 2) {
            g.landed = true;
            g.landTick = obs.tick;
            // still holding when it lands: a cube jumps again right away
            if (m_curAction == 1 && g.primitive == Primitive::Jump && obs.mode == GameMode::Cube) g.primitive = Primitive::RejumpChain;
        }
        if (g.pressed && obs.mode == GameMode::Robot && g.primitive == Primitive::Jump && m_curAction == 1 && obs.tick - g.pressTick >= 12)
            g.primitive = Primitive::LongJump;
        if (g.primitive == Primitive::LongJump && m_curAction == 1) g.second = static_cast<float>(obs.tick - g.pressTick);
    }

    if (obs.obstacles.empty() || !obs.onGround) return;
    m_sit = describeSituation(obs);
    m_status.situation = m_sit.valid ? m_sit.describe() : std::string();
    if (!m_sit.valid || m_sit.contactSteps < 0.f) return;
    for (auto const& g : m_groups)
        if (std::fabs(g.sit.startX - m_sit.startX) < 1.f) return; // already tracked
    if (m_groups.size() >= 8) return;

    PendingGroup g;
    g.sit = m_sit;
    g.choice = m_strategies->choose(m_sit, m_level->key);
    ++T.situations;
    auto const& c = g.choice;
    if (c.rejectedHere) {
        ++T.rejectedHere;
        noteTransfer(fmt("Previous strategy rejected - it failed %u times on this level (%s); learning this level's own timing",
                         c.localFailures, m_sit.describe().c_str()));
    } else if (!c.usable) {
        ++T.unknown;
        noteTransfer(fmt("New obstacle pattern - gathering experience (%s)", m_sit.describe().c_str()));
    } else {
        if (c.match == StrategyChoice::Match::Exact) ++T.known;
        else ++T.similar;
        if (c.speedAdapted) ++T.speedAdapted;
    }
    m_groups.push_back(std::move(g));
}

int Agent::transferAction(Observation const& obs, PendingGroup const& g) const {
    auto const& c = g.choice;
    if (!c.usable) return -1;
    if (!g.pressed) {
        if (!obs.onGround) return -1;
        if (c.primitive == Primitive::Wait || c.leadSteps < 0.f) return 0;
        float const vx = unitsPerSecond(obs.speedClass()) / static_cast<float>(kTicksPerSecond);
        float const contact = (g.sit.startX - (obs.x + situationHalfSize(obs.mini))) / vx;
        return contact <= c.leadSteps + 0.5f ? 1 : 0;
    }
    if (!g.landed) {
        switch (c.primitive) {
            case Primitive::RejumpChain: return 1;
            case Primitive::LongJump: return obs.tick - g.pressTick < std::lround(std::max(c.secondSteps, 12.f)) ? 1 : 0;
            default: return 0; // a single jump: release in the air (holding would jump again on landing)
        }
    }
    if (c.primitive == Primitive::DoubleJump && g.primitive != Primitive::DoubleJump && obs.onGround)
        return obs.tick - g.landTick >= std::lround(std::max(c.secondSteps, 0.f)) ? 1 : 0;
    return 0;
}

void Agent::reportGroup(PendingGroup& g, bool trustedNow) {
    (void)trustedNow;
    g.reported = true;
    auto const& c = g.choice;
    if (!c.usable) return;
    auto& T = m_status.transfer;
    bool const jumpChoice = c.primitive != Primitive::Wait;
    bool const matches =
        jumpChoice ? (g.pressed && g.primitive == c.primitive && std::fabs(g.lead - c.leadSteps) <= 4.f) : !g.pressed;
    char const* name = primitiveName(c.primitive);
    std::string what = jumpChoice ? fmt("%s, press %.0f steps before contact", name, c.leadSteps) : std::string(name);
    std::string evidence = fmt("worked %u times, on %d other level%s", c.successes, c.otherLevels, c.otherLevels == 1 ? "" : "s");
    std::string speed;
    if (c.speedAdapted && c.learnedSpeed >= 0)
        speed = fmt("Known strategy adapted for current speed (learned at speed class %d, now %d: same steps, %s distance). ",
                    c.learnedSpeed, g.sit.speedClass, g.sit.speedClass > c.learnedSpeed ? "longer" : "shorter");
    bool const newLevel = !c.learnedHere && m_level && m_level->completions == 0;
    char const* lead = newLevel ? "New level detected - using" : "Using";
    if (g.played) {
        ++T.played;
        noteTransfer(speed + fmt("%s previously learned %s strategy (%s; %s) - no trusted prediction yet, played directly", lead,
                                 g.sit.category(), what.c_str(), evidence.c_str()));
        return;
    }
    if (!g.seeded) return; // the search never saw it (e.g. a plan was being followed): no verdict
    ++T.seeded;
    if (matches) {
        ++T.confirmed;
        noteTransfer(speed + fmt("%s previously learned %s strategy (%s; %s) - confirmed by simulation", lead, g.sit.category(),
                                 what.c_str(), evidence.c_str()));
    } else {
        ++T.rejectedBySim;
        // A collision is only the strategy's fault if it happens at this group
        // (not at something further ahead that no single plan clears yet).
        float const groupSteps = (g.sit.endX - g.sit.startX) / std::max(0.1f, unitsPerSecond(g.sit.speedClass) / kTicksPerSecond);
        bool const hitsHere = g.seedOutcome == Certainty::Collision && g.seedCollision >= 0 &&
                              static_cast<float>(g.seedCollision) <= g.seedContact + groupSteps + 20.f;
        std::string why = hitsHere ? fmt("simulation predicts a collision at this obstacle group, %d steps after the decision", g.seedCollision + 1)
                                   : fmt("the search found a better plan (score %.1f vs %.1f)", g.bestScore, g.seedScore);
        if (c.landingDiffers) why += "; landing space differs from where it was learned";
        else if (c.match == StrategyChoice::Match::Similar) why += "; the layout is only similar to where it was learned";
        noteTransfer(fmt("Previous strategy rejected - %s (%s for %s)", why.c_str(), what.c_str(), g.sit.describe().c_str()));
    }
}

void Agent::resolveGroups(float deathX, bool died, bool completed) {
    if (m_strategies && m_level && m_cfg.learningEnabled) {
        // Sorted by position: only the group the player died at is a failure.
        std::sort(m_groups.begin(), m_groups.end(), [](PendingGroup const& a, PendingGroup const& b) { return a.sit.startX < b.sit.startX; });
        for (auto& g : m_groups) {
            bool const attributable = g.pressed || !g.airborne;
            if (completed) {
                if (!g.reported) reportGroup(g, false);
                if (attributable) {
                    m_strategies->record(g.sit, g.primitive, g.lead, g.second, true, m_level->key);
                    ++m_status.transfer.successes;
                } else {
                    ++m_status.transfer.unattributed;
                }
                continue;
            }
            if (died && deathX >= g.sit.startX - 2.f * kBlock && deathX <= g.sit.endX + kBlock) {
                if (!g.reported) reportGroup(g, false);
                if (attributable) {
                    m_strategies->record(g.sit, g.primitive, g.lead, g.second, false, m_level->key);
                    ++m_status.transfer.failures;
                } else {
                    ++m_status.transfer.unattributed;
                }
                break;
            }
        }
    }
    if (m_strategies) m_status.transferTotal.add(m_status.transfer);
    m_groups.clear();
}

} // namespace aigd
