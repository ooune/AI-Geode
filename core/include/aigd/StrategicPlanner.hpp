#pragma once

#include <aigd/Predictor.hpp>
#include <aigd/Situation.hpp>
#include <aigd/Strategy.hpp>
#include <aigd/Types.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace aigd {

struct StrategicConfig {
    int maxSimulations = 160;        // per plan (bounded work)
    int maxHorizon = 420;            // steps
    float joinLanding = 4.f * kBlock; // groups with less landing space than this between them are one sequence
    float maxSequence = 18.f * kBlock; // a sequence is at most this long
    int replanEvery = 60;            // steps (a plan is also re-made when the state leaves it)
    int beam = 4;                    // partial sequences expanded per depth
    float planAhead = 160.f;         // steps: a sequence is planned once its first group is this close
};

// One evaluated input sequence (presses relative to the step the plan was made in).
struct SequenceCandidate {
    Plan plan;
    Certainty outcome = Certainty::Collision;
    int32_t collisionStep = -1;
    int groupsCleared = 0;           // obstacle groups of the sequence passed before any collision
    float landingRoom = 0.f;         // smallest free path in front of a landing (units)
    float score = -1e9f;
    // Handled up to the landing after its last press; only a later obstacle
    // (that the next decision will take care of) is hit.
    bool openEnd = false;
    int32_t safeUntil = -1;          // that landing's step
};

// What the strategic planner decided for the obstacle sequence ahead.
struct StrategicPlan {
    bool valid = false;
    int32_t madeTick = 0;            // made from the observation of this step; plan steps count from the next one
    float madeX = 0.f, madeY = 0.f;
    bool madeGrounded = false;
    GameMode mode = GameMode::Cube;
    int speedClass = 1;
    bool upsideDown = false;
    uint64_t geometryVersion = 0;    // the level geometry the plan was made with

    // the sequence
    float fromX = 0.f, toX = 0.f;    // world x of the first group's start and the last group's end
    int groups = 0;
    float groupStart[6] = {}, groupEnd[6] = {};
    std::string description;         // e.g. "single spike -> 2-block landing -> double spike"

    // the chosen input sequence and its predicted path
    SequenceCandidate chosen;
    PredictedPath path;
    int horizon = 0;
    int effectiveHorizon = 0;        // how far the plan is valid (open-ended plans: up to their last landing)
    std::vector<float> pressX;       // where the presses are predicted to happen
    std::vector<float> landingX;     // where it is predicted to land
    std::vector<SequenceCandidate> alternatives; // other first-press timings (best first)

    // why
    bool overridesLocal = false;     // a locally better-looking first press was rejected for its consequences
    SequenceCandidate localBest;     // what judging only the first obstacle(s) would have chosen
    std::string reason;
    bool fromLibrary = false;        // the chosen timing came from the strategy library

    // cost
    int simulations = 0;
    float micros = 0.f;
};

// Plans whole obstacle sequences ahead (cube and robot): it finds the groups
// that have to be handled together, searches input sequences of up to three
// presses over all of them with the trajectory simulation (bounded), scores
// them by safety, groups handled, landing room, inputs and robustness, and
// keeps the best as the strategic plan. The real-time controller only gets
// that plan as a candidate for its own search (it still decides, checks
// safety and cancels); the plan is re-made whenever the real state leaves it.
class StrategicPlanner {
public:
    explicit StrategicPlanner(StrategicConfig cfg = {}) : m_cfg(cfg) {}
    void setConfig(StrategicConfig const& cfg) { m_cfg = cfg; }
    StrategicConfig const& config() const { return m_cfg; }

    void reset();
    // Checks the plan against the actual state and re-makes it when needed.
    // `obstacles`/`events` must cover the sequence (the caller collects a
    // wide enough window). Returns the current plan or nullptr.
    StrategicPlan const* update(Observation const& obs, bool holding, Predictor const& predictor, std::span<Obstacle const> obstacles,
                                std::span<PhysicsEvent const> events, StrategyLibrary const* library = nullptr,
                                std::string const& level = {});
    void invalidate(std::string reason);
    // The controller's search (over the same look-ahead) found a better input
    // sequence than the plan: the plan takes it over, so the plan keeps
    // describing what is actually being executed.
    void adoptController(Observation const& obs, PredictedPath const& path);

    // The plan as a candidate for the controller's search at this step:
    // press times relative to now. False when there is nothing to propose
    // (no plan, or its presses are already done).
    bool seed(Observation const& obs, Plan& out, int& horizon) const;

    StrategicPlan const& plan() const { return m_plan; }
    bool active() const { return m_plan.valid; }
    // A plan the controller should take into account: a fully safe sequence
    // was found. (When none was, the controller's own receding-horizon search
    // decides alone: judging every candidate by a far-away collision would
    // only make the next input worse.)
    bool useful() const { return m_plan.valid && (m_plan.chosen.outcome == Certainty::Safe || m_plan.chosen.openEnd); }

    // statistics (diagnostics, tests)
    uint32_t plansMade = 0;
    uint32_t invalidations = 0;
    uint32_t overrides = 0;          // plans whose first press differs from the locally best one
    uint32_t adopted = 0;            // plans refined by the controller's own search
    double totalMicros = 0.0;
    float maxMicros = 0.f;
    std::string lastInvalidation;

private:
    StrategicConfig m_cfg;
    StrategicPlan m_plan;
    // execution tracking: how far along the chosen sequence the player is
    int m_pressesSeen = 0;           // presses of the plan that happened
    int m_landingsSeen = 0;
    int32_t m_lastLandTick = 0;
    bool m_wasGrounded = true;
    bool m_holding = false;

    bool makePlan(Observation const& obs, bool holding, Predictor const& predictor, std::span<Obstacle const> obstacles,
                  std::span<PhysicsEvent const> events, StrategyLibrary const* library, std::string const& level);
};

} // namespace aigd
