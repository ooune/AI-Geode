#pragma once

#include <aigd/Physics.hpp>
#include <aigd/Types.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace aigd {

// How sure the predictor is about a simulated path.
enum class Certainty : uint8_t {
    Safe = 0,      // no contact within the horizon, even with error margins
    Uncertain = 1, // near miss within the error margin, unmodelled object, or model not calibrated
    Collision = 2, // hitboxes overlap by more than the error margin
};
char const* certaintyName(Certainty c);

struct PathPoint {
    float x = 0.f, y = 0.f;
};

// An input plan, relative to the next physics step.
struct Plan {
    int32_t pressAt = -1; // step at which the button goes down (-1 = never)
    bool rejump = false;  // keep holding through the first landing (jump again)
};

struct PredictedPath {
    std::vector<PathPoint> points; // one per simulated step, world coordinates
    Plan plan;
    Certainty outcome = Certainty::Safe;
    int32_t collisionStep = -1;    // first step with a (definite or potential) collision
    float collisionX = 0.f, collisionY = 0.f;
    int obstacle = -1;             // index into the obstacle span
    int32_t uncertainFrom = -1;    // step where an unmodelled object was touched
    bool landed = false;
    float landX = 0.f, landY = 0.f;
    float clearance = 1e9f;        // smallest gap to a hazard along the path
    int32_t steps = 0;
};

// Player state the simulation starts from.
struct SimStart {
    float x = 0.f, y = 0.f, vy = 0.f;
    bool onGround = false, upsideDown = false, mini = false, holding = false;
    int speedClass = 1;
    int32_t tick = 0;
};

// Forward simulation of the cube with hitbox collision prediction, plus a small
// receding-horizon planner: before each decision it simulates "press now",
// "release now and press k steps later" (coarse-to-fine k), "never press" and
// "keep holding through the landing", and reports which branches are safe.
class Predictor {
public:
    struct Advice {
        bool valid = false;            // false: mode unsupported or nothing relevant ahead
        bool trusted = false;          // model accurate enough to act on
        Certainty press = Certainty::Safe;   // best outcome if the button is held this step
        Certainty release = Certainty::Safe; // best outcome if it is released this step
        int32_t pressTTC = -1;         // steps to collision of the best press plan (-1 none)
        int32_t releaseTTC = -1;
        bool bestPresses = false;      // first action of the overall best plan
        int32_t bestPressAt = -1;      // when the overall best plan presses
        int simulations = 0;
        float micros = 0.f;
    };

    explicit Predictor(PhysicsModel& model);

    static bool supports(GameMode mode) { return mode == GameMode::Cube; }
    static SimStart startFrom(Observation const& obs, bool holding);

    void setHorizon(int steps) { m_horizon = std::max(30, std::min(steps, 480)); }
    int horizon() const { return m_horizon; }
    bool trusted(bool mini) const;
    float margin(int step, bool mini) const;

    // Simulates a plan. If `holds` is given it overrides the plan with an
    // explicit per-step button state (used to re-simulate recorded play).
    void simulate(SimStart const& start, std::span<Obstacle const> obstacles, Plan const& plan, PredictedPath& out,
                  std::span<uint8_t const> holds = {}) const;

    // Receding-horizon evaluation for the current step (cached per tick).
    Advice const& advise(Observation const& obs, bool holding);
    // Which branch was actually executed (for misprediction tracking).
    void noteExecuted(bool pressed);

    // Calibration + accuracy: call once per physics step with the new state and
    // whether the button was held during the step that led to it.
    void observe(Observation const& obs, bool heldDuringStep);
    // Compare the last executed prediction with an actual death.
    void onDeath(int32_t tick, float x, float y);

    PredictedPath const& bestPath() const { return m_bestPresses ? m_press : m_release; }
    PredictedPath const& alternativePath() const { return m_bestPresses ? m_release : m_press; }
    PredictedPath const& pressPath() const { return m_press; }
    PredictedPath const& releasePath() const { return m_release; }
    Advice const& lastAdvice() const { return m_advice; }

    uint32_t mispredictions() const { return m_mispredictions; }
    float caution() const { return m_caution; }
    uint64_t totalSimulations() const { return m_totalSims; }
    uint64_t reusedAdvice() const { return m_reused; }
    std::string const& lastDeathComparison() const { return m_deathComparison; }
    PhysicsModel const& model() const { return m_model; }

    // Last one-step prediction vs. what actually happened (diagnostics).
    float lastPredictedY() const { return m_predY; }
    float lastActualY() const { return m_actY; }
    float lastStepError() const { return m_stepErr; }

private:
    struct Rec {
        float x, y, vy;
        bool onGround, upsideDown, mini, held, dead;
        GameMode mode;
        int speedClass;
        int32_t tick;
    };

    float score(PredictedPath const& p) const;
    void evaluate(SimStart const& s, std::span<Obstacle const> obs, Plan const& plan, PredictedPath& tmp, PredictedPath& best,
                  bool& haveBest);

    PhysicsModel& m_model;
    int m_horizon = 180;
    float m_caution = 1.f;
    uint32_t m_mispredictions = 0;
    uint64_t m_totalSims = 0;

    Advice m_advice;
    int32_t m_adviceTick = INT32_MIN;
    bool m_adviceHolding = false;
    int32_t m_reuseUntilTick = INT32_MIN; // airborne with nothing to decide: reuse the last plan
    uint64_t m_reused = 0;
    PredictedPath m_press, m_release, m_tmp;
    bool m_bestPresses = false;

    // what the agent actually did, to judge the model later
    bool m_expectValid = false;
    Certainty m_expectOutcome = Certainty::Safe;
    int32_t m_expectTick = 0;
    int32_t m_expectSteps = 0;
    int32_t m_expectCollisionStep = -1;
    float m_expectCX = 0.f, m_expectCY = 0.f;
    std::string m_deathComparison;

    // history for calibration / re-simulation
    static constexpr size_t kHistory = 64;
    static constexpr int kResimSteps = 32;
    std::array<Rec, kHistory> m_hist{};
    size_t m_histCount = 0;
    size_t m_histHead = 0;
    int m_sinceResim = 0;
    std::vector<uint8_t> m_holdBuf;
    PredictedPath m_resim;
    float m_predY = 0.f, m_actY = 0.f, m_stepErr = 0.f;
};

} // namespace aigd
