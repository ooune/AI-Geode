#pragma once

#include <aigd/Config.hpp>
#include <aigd/FailureAnalyzer.hpp>
#include <aigd/Memory.hpp>
#include <aigd/Predictor.hpp>
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
enum class ExperimentType : uint8_t { None = 0, ShiftBlock, ShiftStart, ShiftEnd, InsertPress, HoldRange, ReleaseRange };

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
};

class Agent {
public:
    explicit Agent(LearnerConfig cfg = {});

    void setConfig(LearnerConfig const& cfg) { m_cfg = cfg; }
    LearnerConfig const& config() const { return m_cfg; }

    // Memory is owned by the caller and must outlive the attachment.
    void attach(LevelMemory* level, PatternMemory* patterns);
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

    // Forget transient search state (used after memory is reset/imported).
    void resetTransient();

private:
    struct ActionTrace {
        bool valid = false;
        int32_t startCell = 0;
        std::vector<uint8_t> actions;
        std::vector<uint8_t> ground;  // player on the ground at the cell centre
        std::vector<uint64_t> keys;   // state key of the decision active at the cell centre
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

    struct LeadCandidate {
        float obstacleX;
        float lead;
        uint32_t key;
    };

    LearnerConfig m_cfg;
    LevelMemory* m_level = nullptr;
    PatternMemory* m_patterns = nullptr;
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

    Observation m_lastObs;
};

} // namespace aigd
