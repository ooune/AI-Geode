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
    Uncertain = 1, // near miss within the error margin, unmodelled object, unconfirmed bound
    Collision = 2, // hitboxes overlap by more than the error margin
};
char const* certaintyName(Certainty c);

struct PathPoint {
    float x = 0.f, y = 0.f;
};

// An input plan, relative to the next physics step.
struct Plan {
    int32_t pressAt = -1;     // step at which the button goes down (-1 = never)
    int32_t holdFor = 0;      // steps held; 0 = until the press takes effect (jump, flap, flip,
                              // teleport; at least 2 steps), -1 = keep holding to the end
    int32_t repeatEvery = 0;  // press again this many steps after each press started (0 = once)
    bool rejump = false;      // cube/robot: keep holding through the first landing (jump again)
};

struct PredictedPath {
    std::vector<PathPoint> points; // one per simulated step, world coordinates
    std::vector<uint8_t> holds;    // button state used at each simulated step
    Plan plan;
    Certainty outcome = Certainty::Safe;
    int32_t collisionStep = -1;    // first step with a (definite or potential) collision
    float collisionX = 0.f, collisionY = 0.f;
    int obstacle = -1;             // index into the obstacle span used for the simulation
    Obstacle hit;                  // copy of that obstacle (the span may be refilled later)
    int32_t uncertainFrom = -1;    // step where an unmodelled object / unknown surface was reached
    bool landed = false;
    float landX = 0.f, landY = 0.f;
    float clearance = 1e9f;        // smallest gap to a hazard along the path
    int32_t steps = 0;
};

// Floor / ceiling of the playing area, as surfaces (not player centres).
struct Bounds {
    bool hasFloor = true;
    float floor = 90.f;            // GD's ground surface
    bool floorEstimated = false;   // from a hint, not confirmed by contact yet
    bool hasCeil = false;
    float ceil = 0.f;
    bool ceilEstimated = true;
};

// Player state the simulation starts from.
struct SimStart {
    float x = 0.f, y = 0.f, vy = 0.f;
    GameMode mode = GameMode::Cube;
    bool onGround = false, upsideDown = false, mini = false;
    bool holding = false;          // button state before the first simulated step
    int speedClass = 1;
    int32_t tick = 0;
    Bounds bounds;
    int32_t robotBoostLeft = 0;    // robot: boost steps still available while held
};

// Forward simulation of the player for every game mode, with hitbox collision
// prediction, plus a small receding-horizon planner: before each decision it
// simulates the button being down vs. up now, each followed by a set of
// mode-appropriate continuations (press after k steps, hold for d steps,
// repeated taps, hovering), and reports which branches can avoid a collision.
class Predictor {
public:
    struct Advice {
        bool valid = false;            // false: nothing relevant ahead
        bool trusted = false;          // model for this mode accurate enough to act on
        Certainty press = Certainty::Safe;   // best outcome if the button is down this step
        Certainty release = Certainty::Safe; // best outcome if it is up this step
        int32_t pressTTC = -1;         // steps to collision of the best press plan (-1 none)
        int32_t releaseTTC = -1;
        bool bestPresses = false;      // first action of the overall best plan
        int32_t bestPressAt = -1;      // when the overall best plan presses
        int simulations = 0;
        float micros = 0.f;
    };

    explicit Predictor(PhysicsModel& model);

    static bool supports(GameMode mode) { return mode < GameMode::Count; }
    // Start state for a simulation from an observation, with the floor/ceiling
    // the predictor has learned for the current mode.
    SimStart startFrom(Observation const& obs, bool holding) const;

    void setHorizon(int steps) { m_horizon = std::max(30, std::min(steps, 480)); }
    int horizon() const { return m_horizon; }
    bool trusted(GameMode mode, bool mini) const;
    bool trusted(bool mini) const { return trusted(GameMode::Cube, mini); }
    float margin(int step, GameMode mode, bool mini) const;
    float margin(int step, bool mini) const { return margin(step, GameMode::Cube, mini); }

    // Simulates a plan. If `holds` is given it overrides the plan with an
    // explicit per-step button state (used to re-simulate recorded play).
    void simulate(SimStart const& start, std::span<Obstacle const> obstacles, Plan const& plan, PredictedPath& out,
                  std::span<uint8_t const> holds = {}) const;

    // Receding-horizon evaluation for the current step (cached per tick; while
    // nothing can change, e.g. a cube in the air, the previous plan is reused
    // unless allowReuse is false).
    Advice const& advise(Observation const& obs, bool holding, bool allowReuse = true);
    void noteExecuted(bool pressed);

    // Calibration + accuracy + floor/ceiling tracking: call once per physics
    // step with the new state and whether the button was held during the step
    // that led to it.
    void observe(Observation const& obs, bool heldDuringStep);
    void onDeath(int32_t tick, float x, float y);

    Bounds const& bounds() const { return m_bounds; }

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
        float playerSpeed;
        int32_t robotBoostLeft;
    };

    float score(PredictedPath const& p) const;
    void evaluate(SimStart const& s, std::span<Obstacle const> obs, Plan const& plan, PredictedPath& tmp, PredictedPath& best,
                  bool& haveBest);
    void updateBounds(Observation const& obs, bool held);

    PhysicsModel& m_model;
    int m_horizon = 180;
    float m_caution = 1.f;
    uint32_t m_mispredictions = 0;
    uint64_t m_totalSims = 0;

    Bounds m_bounds;
    int32_t m_robotSince = 1 << 20;
    int m_pressedSteps = 0;      // flying: consecutive steps pinned against a bound
    float m_lastY = 0.f;
    bool m_robotAlive = false;
    GameMode m_boundsMode = GameMode::Count;
    float m_boundsHintBottom = 0.f, m_boundsHintTop = 0.f;

    Advice m_advice;
    int32_t m_adviceTick = INT32_MIN;
    bool m_adviceHolding = false;
    int32_t m_reuseUntilTick = INT32_MIN;
    int32_t m_computedTick = INT32_MIN;   // when the current plan was actually computed
    GameMode m_computedMode = GameMode::Count;
    uint64_t m_reused = 0;
    PredictedPath m_press, m_release, m_tmp;
    bool m_bestPresses = false;

    bool m_expectValid = false;
    Certainty m_expectOutcome = Certainty::Safe;
    int32_t m_expectTick = 0;
    int32_t m_expectSteps = 0;
    int32_t m_expectCollisionStep = -1;
    float m_expectCX = 0.f, m_expectCY = 0.f;
    std::string m_deathComparison;

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
