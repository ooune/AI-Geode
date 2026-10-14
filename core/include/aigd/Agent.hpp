#pragma once

#include <aigd/Config.hpp>
#include <aigd/FailureAnalyzer.hpp>
#include <aigd/Memory.hpp>
#include <aigd/PlanManager.hpp>
#include <aigd/Predictor.hpp>
#include <aigd/Situation.hpp>
#include <aigd/StrategicPlanner.hpp>
#include <aigd/Strategy.hpp>
#include <aigd/Trajectory.hpp>
#include <aigd/Types.hpp>

#include <cstdint>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace aigd {

// HoldRange / ReleaseRange are for flying modes, where altitude is the sum of
// many inputs: "hold continuously from N cells before the obstacle" lets the
// search move N instead of nudging one short tap.
// RemovePress: the jump itself was the mistake (it carried the player into
// something above): try not pressing at all.
enum class ExperimentType : uint8_t { None = 0, ShiftBlock, ShiftStart, ShiftEnd, InsertPress, HoldRange, ReleaseRange, RemovePress };

// A controlled change to the best known way of playing a short section: e.g.
// "press 2 cells later than last time". Experiments are proposed by the failure
// analysis, evaluated by actually playing, and either kept (committed into the
// Q-table) or rejected.
struct Experiment {
    bool active = false;
    ExperimentType type = ExperimentType::None;
    int32_t hotspot = 0;
    int32_t fromCell = 0;   // inclusive
    int32_t toCell = -1;    // inclusive
    int32_t shift = 0;
    std::vector<uint8_t> actions; // per cell in [fromCell, toCell]
    // The cells the experiment actually changes (its hypothesis). Only these
    // are played as written. Elsewhere in [fromCell, toCell] the reference
    // attempt's action is reproduced only while the player is in exactly the
    // reference attempt's state there (same key), so a fair comparison is made
    // without ever replaying actions in a situation they were not chosen for.
    int32_t testFrom = 0;
    int32_t testTo = -1;
    std::vector<uint64_t> baseKeys; // per cell, the reference attempt's state key (0 = unknown)
    std::vector<float> baseYs;      // per cell, the reference attempt's height
    std::vector<uint8_t> baseGround;// per cell, whether the reference attempt was on the ground
    int cancellations = 0;          // times it was cancelled because the state did not match the reference
    std::string description;
    // The search that produced this experiment (used to remember what worked).
    int32_t blockStart = 0;
    int32_t lo = INT32_MIN;
    int32_t hi = INT32_MAX;

    bool covers(int32_t cell) const { return active && cell >= fromCell && cell <= toCell; }
    uint8_t actionAt(int32_t cell) const { return actions[static_cast<size_t>(cell - fromCell)]; }
    bool tests(int32_t cell) const { return covers(cell) && cell >= testFrom && cell <= testTo; }
    uint64_t baseKeyAt(int32_t cell) const {
        size_t i = static_cast<size_t>(cell - fromCell);
        return covers(cell) && i < baseKeys.size() ? baseKeys[i] : 0;
    }
};

// Counters that show whether learning is working (diagnostics + tests).
struct LearningMetrics {
    uint32_t repeatedFailuresDetected = 0; // a decision about to be repeated had failed repeatedly in that state
    uint32_t alternativesTried = 0;        // ... and the alternative was played instead
    uint32_t alternativesVetoed = 0;       // ... or not, because the alternative was predicted to collide
    uint32_t alternativesThatHelped = 0;   // a tried alternative got safely past that point
    uint32_t sameMistakeRepeated = 0;      // deaths blamed on a decision already known to fail there
    uint32_t knownGoodKept = 0;            // the learned choice was overruled by a reliable record of the other action
    uint32_t pivotsByPrediction = 0;       // deaths whose cause was located with the recorded predictions
    uint32_t pivotsByAnalysis = 0;         // ... with the failure analysis
    uint32_t pivotsUnknown = 0;            // ... not located (no decision blamed)
    uint32_t doomedSkipped = 0;            // decisions not blamed because neither action could have survived
    uint32_t mispredictedDeaths = 0;       // deaths on a path that was predicted to be safe (model error)
    uint32_t newBests = 0;
    uint32_t referenceSteps = 0;           // decisions reproduced from an experiment's reference attempt
    uint32_t independentSteps = 0;         // all other decisions
};

// What the cross-level strategy library contributed (diagnostics + tests).
struct TransferMetrics {
    uint32_t situations = 0;     // obstacle groups met (approached on the ground)
    uint32_t known = 0;          // ... with a usable strategy for exactly this layout
    uint32_t similar = 0;        // ... with one for a similar layout only
    uint32_t unknown = 0;        // ... with none: a new obstacle pattern
    uint32_t rejectedHere = 0;   // ... whose strategy failed repeatedly on this level (not used here)
    uint32_t speedAdapted = 0;   // ... whose strategy was learned at another speed
    uint32_t seeded = 0;         // groups whose strategy was handed to the search (simulated)
    uint32_t confirmed = 0;      // ... and what was executed matched it
    uint32_t rejectedBySim = 0;  // ... and something else was executed (the simulation preferred it)
    uint32_t played = 0;         // groups where the strategy was played directly (no trusted prediction, no own experience)
    uint32_t successes = 0;      // outcomes recorded into the library: got past the group
    uint32_t failures = 0;       // ... died at it
    uint32_t unattributed = 0;   // passed or died at without a press of its own, but not on the ground
                                 // all the way (cleared by an earlier jump): not recorded

    void add(TransferMetrics const& o) {
        situations += o.situations; known += o.known; similar += o.similar; unknown += o.unknown;
        rejectedHere += o.rejectedHere; speedAdapted += o.speedAdapted; seeded += o.seeded;
        confirmed += o.confirmed; rejectedBySim += o.rejectedBySim; played += o.played;
        successes += o.successes; failures += o.failures; unattributed += o.unattributed;
    }
};

// Outcome of the safety check that runs on the action about to be executed.
enum class SafetyVerdict : uint8_t {
    NoPrediction = 0, // no trusted, relevant prediction: nothing was checked
    Clear,            // the executed action is safe and keeps enough time to change course
    Kept,             // danger near, but the executed action is still the best available
    Overridden,       // the proposed action was unsafe and was replaced by a safer one
    NoSafeOption,     // every candidate is predicted to collide; the least bad one was executed
    NoInputNeeded,    // a press was proposed, but not pressing is predicted clear: no press
};
char const* safetyVerdictName(SafetyVerdict v);

// One safety-critical decision, as it really happened.
struct SafetyRecord {
    int32_t tick = 0;
    float x = 0.f, y = 0.f, vy = 0.f;
    GameMode mode = GameMode::Cube;
    bool onGround = false;
    bool holdingBefore = false;
    bool predictionCurrent = false;  // simulated from exactly this state in this step
    int32_t predictionAge = 0;       // steps since it was simulated (0 = current)
    Certainty outcome[2] = {Certainty::Safe, Certainty::Safe}; // [release, press]
    int32_t ttc[2] = {-1, -1};       // steps to a definite collision (-1 none)
    int32_t keepFor[2] = {-1, -1};   // steps that action can be kept before it must change (-1 never safe)
    Certainty keep = Certainty::Safe; // keeping the current button state unchanged
    int32_t keepTTC = -1;
    uint8_t proposed = 0;            // what the policy/planner wanted
    DecisionSource proposedSource = DecisionSource::Policy;
    uint8_t executed = 0;            // what was actually returned to the game
    SafetyVerdict verdict = SafetyVerdict::NoPrediction;
    bool alternativeAvailable = false;
    std::string reason;
    // decision explanation
    bool predictionValid = false;    // something relevant ahead was simulated
    bool trusted = false;            // ... and the model was trusted to decide
    Certainty never = Certainty::Safe; // not pressing at all within the horizon
    int32_t neverTTC = -1;
    bool hazardAhead = false;        // nearest hazard ahead (relative to the player)
    float hazardDx = 0.f, hazardDy = 0.f;
};

struct AgentStatus {
    AgentPhase phase = AgentPhase::Idle;
    uint32_t attempt = 0;          // number of the attempt being played
    uint32_t deaths = 0;
    uint32_t completions = 0;
    float progress = 0.f;
    float bestProgress = 0.f;
    float completionRate = 0.f;    // over recent attempts
    float lastFailureProgress = -1.f;
    std::string lastFailure;       // why the last attempt failed
    std::string lastChange;        // what the AI changed going into this attempt
    int lastImproved = -1;         // -1 unknown, 0 no, 1 yes
    bool stuck = false;

    // debug data for the overlay
    float q[2] = {0.f, 0.f};
    uint8_t action = 0;
    bool forced = false;
    bool explored = false;
    float epsilon = 0.f;
    size_t tableSize = 0;
    size_t patternSize = 0;
    size_t decisions = 0;

    // trajectory prediction
    bool predictionValid = false;
    bool predictionTrusted = false;
    Certainty pressOutcome = Certainty::Safe;
    Certainty releaseOutcome = Certainty::Safe;
    int32_t pressTTC = -1;
    int32_t releaseTTC = -1;
    bool planned = false;
    bool shielded = false;
    uint32_t shieldsThisAttempt = 0;
    uint32_t plannedThisAttempt = 0;
    uint64_t shieldsTotal = 0;
    std::string predictionNote;    // predicted vs. actual at the last death

    // decision diagnostics
    DecisionSource source = DecisionSource::Policy;
    std::string reason;            // why the last notable decision was taken
    uint16_t fails[2] = {0, 0};    // evidence for the current state
    uint16_t passes[2] = {0, 0};
    uint8_t streak[2] = {0, 0};
    std::string lastCredit;        // which decision was blamed for the last death, and why
    LearningMetrics metrics;

    // safety layer
    SafetyRecord safety;           // the last decision's safety check
    uint32_t overridesThisAttempt = 0;
    uint32_t noSafeOptionThisAttempt = 0;
    uint32_t pressesAvoidedThisAttempt = 0;   // proposed presses not executed because not pressing was clear
    uint32_t pressesThisAttempt = 0;          // presses actually executed
    uint32_t decisionsTrusted = 0;            // decisions with a trusted prediction / with an untrusted one
    uint32_t decisionsUntrusted = 0;
    std::string trust;                        // why predictions are (not) used, at the last decision

    // live plan
    std::string planEvent;                    // the last invalidation / cancellation, in words
    uint32_t plansInvalidatedThisAttempt = 0;
    uint32_t actionsCancelledThisAttempt = 0; // pending input changes cancelled with invalid plans
    uint32_t decisionsFollowingPlan = 0;      // decisions that followed a still-valid plan (no new search)
    uint32_t experimentsCancelled = 0;        // learned test sequences cancelled because the state differed

    // cross-level transfer
    TransferMetrics transfer;                 // this attempt
    TransferMetrics transferTotal;            // since the level was attached
    std::string transferEvent;                // the last transfer message, made from a real decision
    std::vector<std::string> transferLog;     // this attempt's transfer messages (the last few, most recent last)
    std::string situation;                    // the obstacle group ahead, described

    // strategic planning (v0.5.0)
    std::string strategic;                    // the current sequence plan and why
    std::string strategicEvent;               // the last strategic invalidation
    uint32_t strategicPlans = 0;              // sequence plans made this attempt
    uint32_t strategicInvalidated = 0;        // ... invalidated because the real state left them
    uint32_t strategicOverrides = 0;          // ... whose first press differs from the locally best one
    uint32_t strategicSeeded = 0;             // controller decisions that had the plan as a candidate
    uint32_t strategicMatched = 0;            // ... where the controller's best plan was the strategic one
    float strategicMaxMicros = 0.f;           // most expensive sequence plan this attempt
    double strategicMicros = 0.0;             // total this attempt
};

class Agent {
public:
    explicit Agent(LearnerConfig cfg = {});

    void setConfig(LearnerConfig const& cfg) { m_cfg = cfg; }
    LearnerConfig const& config() const { return m_cfg; }

    // Memory is owned by the caller and must outlive the attachment.
    // `strategies` (layer B, cross-level) is optional; without it nothing is
    // transferred between levels except the pattern prior and physics.
    void attach(LevelMemory* level, PatternMemory* patterns, StrategyLibrary* strategies = nullptr);
    StrategyLibrary* strategies() const { return m_strategies; }
    void detach();
    // Optional trajectory predictor (not owned). Without it the agent learns
    // purely from experience, as before.
    void setPredictor(Predictor* predictor) { m_predictor = predictor; }
    Predictor* predictor() const { return m_predictor; }
    bool attached() const { return m_level != nullptr; }
    LevelMemory* level() const { return m_level; }

    void beginAttempt(Observation const& first, bool fromCheckpoint);
    // Called once per physics step. Returns whether the jump button should be held.
    bool act(Observation const& obs);
    void onDeath(Observation const& last, DeathInfo const& death);
    void onComplete(Observation const& last);
    // Attempt ended without a death or completion (restart, quit, AI switched off).
    void abortAttempt(Observation const* last);

    bool attemptActive() const { return m_active; }
    bool holding() const { return m_curAction == 1; }

    AgentStatus const& status() const { return m_status; }
    void setPhase(AgentPhase phase) { m_status.phase = phase; }

    // Inspection (tests, debug overlay)
    std::vector<Step> const& steps() const { return m_steps; }
    Experiment const& activeExperiment() const { return m_activeExp; }
    Experiment const& pendingExperiment() const { return m_pendingExp; }
    FailureReport const& lastReport() const { return m_lastReport; }
    // Safety-critical decisions of the current/last attempt (most recent last).
    std::vector<SafetyRecord> const& safetyLog() const { return m_safetyLog; }
    // The live execution plan (separate from all learned memory).
    PlanManager const& plans() const { return m_plan; }
    // The strategic (sequence) planner.
    StrategicPlanner const& strategic() const { return m_strategic; }

    // Forget transient search state (used after memory is reset/imported).
    void resetTransient();

private:
    struct ActionTrace {
        bool valid = false;
        int32_t startCell = 0;
        std::vector<uint8_t> actions;
        std::vector<uint8_t> ground;  // player on the ground at the cell centre
        std::vector<uint64_t> keys;   // state key of the decision active at the cell centre
        std::vector<float> ys;        // player height at the cell centre
        float endX = 0.f;
        FailureReport report;
        uint8_t at(int32_t cell) const {
            if (!valid || cell < startCell) return 0;
            size_t i = static_cast<size_t>(cell - startCell);
            return i < actions.size() ? actions[i] : 0;
        }
        bool groundAt(int32_t cell) const {
            if (!valid || cell < startCell) return true;
            size_t i = static_cast<size_t>(cell - startCell);
            return i < ground.size() ? ground[i] != 0 : true;
        }
        int32_t endCell() const { return startCell + static_cast<int32_t>(actions.size()) - 1; }
        uint64_t keyAt(int32_t cell) const {
            if (!valid || cell < startCell) return 0;
            size_t i = static_cast<size_t>(cell - startCell);
            return i < keys.size() ? keys[i] : 0;
        }
    };

    // Timing constraint learned from a successful experiment: the press that now
    // starts at this cell must keep its start strictly between lo and hi
    // (absolute cells) or the earlier obstacle is no longer cleared.
    struct Constraint {
        int32_t lo = INT32_MIN;
        int32_t hi = INT32_MAX;
    };

    struct Search {
        ExperimentType type = ExperimentType::None;
        int32_t blockStart = 0;   // cells
        int32_t blockEnd = 0;     // inclusive
        int32_t insertLen = 0;
        int32_t lo = INT32_MIN;   // admissible shifts lie strictly between lo and hi
        int32_t hi = INT32_MAX;
        int32_t shift = 0;
        int32_t step = 1;
        uint32_t tries = 0;
        bool exhausted = false;
        float lastEndX = -1.f;     // where the previous try of this search died
        float lastEndY = 0.f;
        uint32_t noEffect = 0;     // tries that changed nothing at all
        // The attempt this search varies. Every try is built from it, so a
        // try never inherits the previous try's modification.
        ActionTrace base;
    };

    // An obstacle group being approached or passed, and what is done about it.
    // Resolved into the strategy library when it is passed (success) or the
    // player dies at it (failure).
    struct PendingGroup {
        Situation sit;            // as described on the ground before it (relative geometry only)
        StrategyChoice choice;    // what the library suggested then
        Primitive primitive = Primitive::Wait; // what is actually being executed
        float lead = -1.f;        // steps before contact of the first press
        float second = -1.f;
        bool pressed = false;
        bool airborne = false;    // left the ground before its own first press (an earlier jump)
        bool landed = false;
        int32_t pressTick = 0;
        int32_t landTick = 0;
        bool seeded = false;      // its strategy was simulated by the search
        bool simChose = false;    // ... and the last search chose it
        Certainty seedOutcome = Certainty::Safe;
        int32_t seedCollision = -1;
        float seedScore = 0.f, bestScore = 0.f;
        float seedContact = 0.f;  // steps to the group when the strategy was last simulated
        bool played = false;      // played directly (Transfer)
        bool reported = false;    // its verdict message was produced
    };

    struct LeadCandidate {
        float obstacleX;
        float lead;
        uint32_t key;
        uint32_t situation;
    };

    LearnerConfig m_cfg;
    LevelMemory* m_level = nullptr;
    PatternMemory* m_patterns = nullptr;
    StrategyLibrary* m_strategies = nullptr;
    std::vector<PendingGroup> m_groups;
    Situation m_sit;                             // the group ahead at the current step
    Predictor* m_predictor = nullptr;
    std::mt19937 m_rng;

    // per attempt
    bool m_active = false;
    bool m_fromCheckpoint = false;
    std::vector<Step> m_steps;
    std::vector<std::pair<float, uint8_t>> m_inputs;
    std::vector<LeadCandidate> m_leads;
    uint64_t m_curKey = ~uint64_t(0);
    uint8_t m_curAction = 0;
    int32_t m_lastDecisionTick = 0;
    int32_t m_lastToggleTick = -1000000;
    int32_t m_startTick = 0;
    float m_startX = 0.f;
    float m_maxX = 0.f;

    // learning state that spans attempts (not persisted)
    Experiment m_activeExp;
    Experiment m_pendingExp;
    ActionTrace m_incumbent;
    std::map<int32_t, Search> m_searches;
    std::map<int32_t, Search> m_parked;          // timing searches paused while "remove the press" is tested
    std::map<int32_t, Constraint> m_constraints;
    int32_t m_lastSearchHotspot = INT32_MIN;
    float m_focusFrom = 0.f;
    float m_focusTo = -1.f;
    float m_prevEndX = -1.f;
    float m_protectLimitX = 1e30f;               // no random exploration before this x (lowered by experiments)
    std::string m_nextChange;
    // Credit assignment of the attempt being finished: the decision identified
    // as the cause of the death, and whether that came from the predictions
    // (then earlier decisions are judged by the best value at that point
    // instead of by the death).
    size_t m_pivot = static_cast<size_t>(-1);
    bool m_cutAtPivot = false;
    std::vector<uint64_t> m_avoidedThisAttempt;  // states where the alternative to a repeated failure was played
    // Danger near: the executed input may have to change soon, so it is
    // re-checked every step instead of being kept while the state key is unchanged.
    bool m_danger = false;
    std::vector<SafetyRecord> m_safetyLog;
    std::vector<int32_t> m_removeTried;          // hotspots where "remove the press" was already tried
    PlanManager m_plan;                          // live plan: pending inputs + the states they expect
    StrategicPlanner m_strategic;                // whole obstacle sequences ahead
    bool m_expChecked = false;                   // the active experiment's state was checked at its first changed cell
    FailureReport m_lastReport;
    AgentStatus m_status;

    QEntry* lookup(uint64_t key, uint64_t pkey, QEntry& scratch);
    float progressReward(float dx) const;
    float epsilonAt(float x) const;
    uint32_t stuckLevel() const;
    // No random exploration before this x: the level is solved up to there.
    float protectedUpTo() const;

    void assignCredit(int32_t endTick, FailureReport const* report, bool completed);
    void applyNewBestBonus(float endX);
    void backwardUpdate(float terminalBonus, bool terminal, float bootstrap, int32_t endTick);
    void commitExperiment();
    void finishAttempt(float endX, int32_t endTick, bool died, bool completed, FailureKind kind);
    ActionTrace buildTrace(float endX) const;
    void planNext(FailureReport const& report);
    void feedback(Search& s, FailureKind kind, bool worse, float endX, float endY);
    bool initSearch(Search& s, FailureReport const& report, ActionTrace const& base, Observation const* last);
    void buildExperiment(Search const& s, ActionTrace const& base, FailureReport const& report);
    void refreshStatus();
    // cross-level transfer
    void trackGroups(Observation const& obs);
    PendingGroup* currentGroup(Observation const& obs);
    int transferAction(Observation const& obs, PendingGroup const& g) const;
    void reportGroup(PendingGroup& g, bool trustedNow);
    void resolveGroups(float deathX, bool died, bool completed);
    void noteTransfer(std::string msg);

    Observation m_lastObs;
};

} // namespace aigd
