#pragma once

#include <aigd/Config.hpp>
#include <aigd/FailureAnalyzer.hpp>
#include <aigd/Memory.hpp>
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
    std::string description;
    // The search that produced this experiment (used to remember what worked).
    int32_t blockStart = 0;
    int32_t lo = INT32_MIN;
    int32_t hi = INT32_MAX;

    bool covers(int32_t cell) const { return active && cell >= fromCell && cell <= toCell; }
    uint8_t actionAt(int32_t cell) const { return actions[static_cast<size_t>(cell - fromCell)]; }
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
};

class Agent {
public:
    explicit Agent(LearnerConfig cfg = {});

    void setConfig(LearnerConfig const& cfg) { m_cfg = cfg; }
    LearnerConfig const& config() const { return m_cfg; }

    // Memory is owned by the caller and must outlive the attachment.
    void attach(LevelMemory* level, PatternMemory* patterns);
    void detach();
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
    float m_protectLimitX = 1e30f;               // lowered if the best path stops working
    std::map<int32_t, uint32_t> m_regressions;   // deaths inside the protected region per hotspot
    std::string m_nextChange;
    FailureReport m_lastReport;
    AgentStatus m_status;

    QEntry* lookup(uint64_t key, uint64_t pkey, QEntry& scratch);
    int greedyAction(QEntry const& e, uint64_t pkey) const;
    float progressReward(float dx) const;
    float epsilonAt(float x) const;
    uint32_t stuckLevel() const;
    // Everything before this x is "known good": the best path is kept there.
    float protectedUpTo() const;
    void commitBestPath();

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
