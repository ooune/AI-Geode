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
    int32_t secondAt = -1;    // cube/robot: a second press this many steps after the first landing (-1 none)
    int32_t thirdAt = -1;     // cube/robot: a third press this many steps after the second landing (-1 none)
    // An orb press: the button goes down at this plan step (whatever else the
    // plan does) and is held for orbHoldFor steps (0 = a 2-step tap). Pressed
    // while the hitbox overlaps an orb, it activates the orb; for a dash orb
    // the hold is how long the dash lasts.
    int32_t orbPressAt = -1;
    int32_t orbHoldFor = 0;
};

// How far the timing search got for one decision.
enum class SearchStage : uint8_t { None = 0, Coarse, Refined, TwoJump };
char const* searchStageName(SearchStage s);

struct PredictedPath {
    std::vector<PathPoint> points; // one per simulated step, world coordinates
    std::vector<uint8_t> holds;    // button state used at each simulated step
    std::vector<uint8_t> ground;   // on the ground (or a block) after each simulated step
    Plan plan;
    Certainty outcome = Certainty::Safe;
    int32_t collisionStep = -1;    // first step with a (definite or potential) collision
    float collisionX = 0.f, collisionY = 0.f;
    int obstacle = -1;             // index into the obstacle span used for the simulation
    Obstacle hit;                  // copy of that obstacle (the span may be refilled later)
    int32_t uncertainFrom = -1;    // step where an unmodelled object / unknown surface was reached
    bool landed = false;
    float landX = 0.f, landY = 0.f;
    int32_t landStep = -1;         // simulated step of the first landing
    float clearance = 1e9f;        // smallest gap to a hazard (or the side of a block) along the path
    int32_t steps = 0;
    int presses = 0;               // new presses the path makes (inputs it costs)
    // Every landing (up to 4): where, and in which simulated step.
    static constexpr int kMaxLandings = 4;
    int landings = 0;
    float landingX[kMaxLandings] = {0.f, 0.f, 0.f, 0.f};
    int32_t landingStep[kMaxLandings] = {0, 0, 0, 0};
    int32_t eventStep = -1;        // first simulated speed/gravity change (-1 none)
    bool unverifiedPhysics = false; // went through a change whose model is not verified
    // orbs activated along the path (up to 2)
    int orbActivations = 0;
    int32_t orbStep[2] = {-1, -1};
    float orbX[2] = {0.f, 0.f}, orbY[2] = {0.f, 0.f};
    OrbType orbType[2] = {OrbType::Yellow, OrbType::Yellow};
    int32_t orbId[2] = {0, 0};
    int32_t dashEndStep = -1;      // a dash along the path ended here
    bool activates(int32_t id) const { return (orbActivations > 0 && orbId[0] == id) || (orbActivations > 1 && orbId[1] == id); }
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
    // Input latency: button states already sent that take effect during the
    // first `lag` simulated steps (the plan starts after them).
    int lag = 0;
    uint8_t pending[4] = {0, 0, 0, 0};
    // Accuracy measurement: only the movement matters, hazards never end the
    // path and blocks never kill (no margins involved).
    bool measureOnly = false;
    // Speed / gravity changes ahead (portals), sorted by left edge.
    std::span<PhysicsEvent const> events;
    // Simulated steps (0 = the predictor's horizon).
    int horizon = 0;
    // Orbs ahead (sorted by left edge) and a dash already in progress.
    std::span<Orb const> orbs;
    bool dashing = false;
    OrbType dashType = OrbType::Dash;
    float dashRotation = 0.f;
    int32_t dashLeft = 1 << 20;    // steps until the dash ends by itself
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
        // First step with a definite or potential collision of each branch's
        // best plan (-1 none); unlike pressTTC also set for "uncertain".
        int32_t pressRisk = -1;
        int32_t releaseRisk = -1;
        // How many steps the button can stay in that state (down / up) and
        // still have a plan that does not collide: the latest moment the
        // input must change. -1 = no such plan; the horizon = no change needed.
        int32_t pressKeepFor = -1;
        int32_t releaseKeepFor = -1;
        // The same, counting only plans predicted Safe (no near miss either).
        int32_t pressKeepForSafe = -1;
        int32_t releaseKeepForSafe = -1;
        // Keeping the current button state unchanged for the whole horizon.
        Certainty keep = Certainty::Safe;
        int32_t keepTTC = -1;
        // Releasing now and never pressing within the horizon ("no jump").
        Certainty never = Certainty::Safe;
        int32_t neverTTC = -1;
        int32_t neverRisk = -1;
        int32_t computedTick = INT32_MIN; // the tick whose state this was simulated from
        // The search (diagnostics, tests)
        SearchStage stage = SearchStage::None;
        Plan bestPlan;                   // best plan overall (its first action is bestPresses)
        float bestScore = 0.f;
        bool bestLanded = false;
        float bestLandX = 0.f;
        int firstSafeSim = -1;           // simulations needed until the first fully safe candidate
        bool seeded = false;             // a learned timing was tried first
        bool fromSeed = false;           // ... and the chosen timing is (within the resolution of) it
        bool strategySeeded = false;     // a plan from the strategy library was simulated
        bool fromStrategy = false;       // ... and the chosen plan is (within the resolution of) it
        // The nearest orb ahead: activating it vs. not (diagnostics, overlay).
        struct OrbChoice {
            bool present = false;
            OrbType type = OrbType::Yellow;
            int32_t id = 0;
            float l = 0.f, b = 0.f, r = 0.f, t = 0.f;
            bool simulated = false;      // its effect can be simulated (orb type supported)
            bool known = false;          // ... and has been calibrated from the game
            int32_t windowFrom = -1, windowTo = -1; // steps in which the paths tried overlap it
            bool overlappingNow = false; // activation possible this step
            int candidates = 0;          // activation timings simulated
            Certainty activate = Certainty::Collision;
            int32_t activateTTC = -1;
            bool activateHitsAbove = false; // the best activation collides with something above the player
            float activateScore = -1e9f;
            Plan activatePlan;
            Certainty skip = Certainty::Collision;
            int32_t skipTTC = -1;
            float skipScore = -1e9f;
            bool chosenActivates = false;
        };
        OrbChoice orb;
        bool lateSeeded = false;         // the strategic sequence plan was evaluated
        bool fromLateSeed = false;       // ... and it is the chosen plan
        Certainty seedOutcome = Certainty::Safe; // the best strategy plan's simulated outcome
        int32_t seedCollisionStep = -1;
        float seedScore = 0.f;
        bool budgetHit = false;          // the computation budget ended the search early
        int resolution = 1;              // smallest timing step the search refined to
        bool reused = false;             // carried over from an earlier tick (not recomputed)
        int simulations = 0;
        float micros = 0.f;

        Certainty outcome(int action) const { return action ? press : release; }
        int32_t ttc(int action) const { return action ? pressTTC : releaseTTC; }
        int32_t risk(int action) const { return action ? pressRisk : releaseRisk; }
        int32_t keepFor(int action) const { return action ? pressKeepFor : releaseKeepFor; }
        int32_t keepForSafe(int action) const { return action ? pressKeepForSafe : releaseKeepForSafe; }
    };

    explicit Predictor(PhysicsModel& model);

    static bool supports(GameMode mode) { return mode < GameMode::Count; }
    // Start state for a simulation from an observation, with the floor/ceiling
    // the predictor has learned for the current mode.
    SimStart startFrom(Observation const& obs, bool holding) const;

    void setHorizon(int steps) { m_horizon = std::max(30, std::min(steps, 480)); }
    // A longer look-ahead for the next advise() only (cube/robot), e.g. to
    // cover a whole obstacle sequence the strategic planner is working on.
    void extendHorizonOnce(int steps) { m_extendOnce = std::max(0, std::min(steps, 480)); }
    // Most simulations per decision (bounded work per game update). Near an
    // imminent hazard a smaller share is used, to find a viable action fast.
    void setBudget(int sims) { m_budget = std::max(16, sims); }
    int budget() const { return m_budget; }
    // A press delay (steps from now) that worked before in a comparable
    // situation. It is evaluated first, like any other candidate; it is never
    // executed without being simulated. Applies to the next advise() only.
    void setHint(int32_t pressAt) { m_hint = pressAt; }
    // Whole plans from the strategy library (another level's experience in
    // a comparable situation): also evaluated first, also never executed
    // without being simulated. Apply to the next advise() only.
    static constexpr int kMaxSeeds = 4;
    void addSeed(Plan const& p) {
        if (m_nSeeds < kMaxSeeds) m_seeds[static_cast<size_t>(m_nSeeds++)] = p;
    }
    // A candidate evaluated after the search's own (so it only wins when it
    // is strictly better, never on a tie): the strategic sequence plan.
    void addLateSeed(Plan const& p) {
        m_lateSeed = p;
        m_hasLateSeed = true;
    }
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
    // Advice is only ever returned for exactly the given state (position,
    // velocity, mode, button): a cached result for a different state is
    // never handed out. It is not reused across ticks while a collision is
    // near, or once the player has left the predicted path.
    Advice const& advise(Observation const& obs, bool holding, bool allowReuse = true);
    // The button state the controller actually executes after the last
    // advice. The overlay draws that branch, never a plan that is not being
    // followed.
    void noteExecuted(bool pressed);

    // Calibration + accuracy + floor/ceiling tracking: call once per physics
    // step with the new state and whether the button was held during the step
    // that led to it.
    // `obs.tick` must count real physics steps (see StepClock): consecutive
    // observations may be several steps apart, with the button held as given
    // for all of them.
    void observe(Observation const& obs, bool heldDuringStep);
    // Average number of physics steps between observations (1 = every step).
    float decisionInterval() const { return m_interval; }
    void onDeath(int32_t tick, float x, float y);

    Bounds const& bounds() const { return m_bounds; }

    PredictedPath const& bestPath() const { return m_bestPresses ? m_press : m_release; }
    PredictedPath const& alternativePath() const { return m_bestPresses ? m_release : m_press; }
    // The branch of the action that was executed (see noteExecuted), the
    // other one, and "keep the current button state unchanged".
    PredictedPath const& executedPath() const { return m_executedPress ? m_press : m_release; }
    PredictedPath const& otherPath() const { return m_executedPress ? m_release : m_press; }
    PredictedPath const& keepPath() const { return m_keep; }
    PredictedPath const& neverPath() const { return m_never; }
    // The search's objective for a simulated path (safety, capped clearance, input cost).
    float scoreOf(PredictedPath const& p) const { return score(p); }

    // Accuracy of the model against the real game (measured continuously by
    // re-simulating recent real steps with the inputs actually used).
    int inputLatency() const { return m_latency; }            // steps between sending the button and it acting
    float latencyError(int lag) const { return lag >= 0 && lag < kLags ? m_lagErr[static_cast<size_t>(lag)] : 0.f; }
    uint32_t latencySamples() const { return m_lagSamples; }
    // Why predictions are or are not used for decisions in this mode.
    std::string trustReason(GameMode mode, bool mini) const;
    bool executedPress() const { return m_executedPress; }
    bool hasExecuted() const { return m_executedValid; }
    PredictedPath const& pressPath() const { return m_press; }
    // The nearest orb ahead: best path that activates it / best that does not.
    PredictedPath const& orbActivatedPath() const { return m_orbOn; }
    PredictedPath const& orbSkippedPath() const { return m_orbOff; }
    // Orb learning (diagnostics): the last activation the game confirmed and
    // how far the model was off; dash steps compared with the model.
    struct OrbCheck {
        int32_t tick = -1;
        OrbType type = OrbType::Yellow;
        float vyError = 0.f;          // actual - predicted velocity right after (normalised)
        bool wasKnown = false;        // the model had been calibrated before
        bool dash = false;
        float dashError = 0.f;        // position error of the last dash step (units)
        uint32_t activations = 0;     // confirmed by the game since load
    };
    OrbCheck const& lastOrbCheck() const { return m_orbCheck; }
    bool dashActive() const { return m_dashActive; }
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
                  bool& haveBest, int branch);
    bool sameState(Observation const& obs, bool holding) const;

public:
    // True when the player is not where the executed branch predicted it
    // would be now (or there is no such prediction).
    bool offPredictedPath(Observation const& obs) const;

private:
    void updateBounds(Observation const& obs, bool held);

    PhysicsModel& m_model;
    int m_horizon = 180;
    int m_budget = 200;
    int m_budgetNow = 200;
    int32_t m_hint = -1;
    int m_extendOnce = 0;
    Plan m_lateSeed;
    bool m_hasLateSeed = false;
    std::array<Plan, kMaxSeeds> m_seeds{};
    int m_nSeeds = 0;
    float m_caution = 1.f;
    uint32_t m_mispredictions = 0;
    uint64_t m_totalSims = 0;

    Bounds m_bounds;
    int32_t m_robotSince = 1 << 20;
    int m_pressedSteps = 0;      // flying: consecutive steps pinned against a bound
    float m_lastY = 0.f;
    bool m_robotAlive = false;
    // dash in progress (from the game's dash start event)
    bool m_dashActive = false;
    OrbType m_dashType = OrbType::Dash;
    float m_dashRotation = 0.f;
    int32_t m_dashStartTick = 0;
    int32_t m_dashMaxSteps = 1 << 20;
    int32_t m_lastOrbTick = -1000;
    OrbCheck m_orbCheck;
    std::vector<int32_t> m_overOrbs;     // orbs the player overlapped at the last observation
    PredictedPath m_orbOn, m_orbOff, m_orbTmp;
    GameMode m_boundsMode = GameMode::Count;
    float m_boundsHintBottom = 0.f, m_boundsHintTop = 0.f;

    Advice m_advice;
    int32_t m_adviceTick = INT32_MIN;
    bool m_adviceHolding = false;
    int32_t m_reuseUntilTick = INT32_MIN;
    int32_t m_computedTick = INT32_MIN;   // when the current plan was actually computed
    GameMode m_computedMode = GameMode::Count;
    uint64_t m_reused = 0;
    PredictedPath m_press, m_release, m_tmp, m_keep, m_never;
    static constexpr int kLags = 3;
    int m_latency = 0;
    std::array<float, kLags> m_lagErr{};   // average re-simulation error per assumed latency
    uint32_t m_lagSamples = 0;             // windows with an input change (where latency matters)
    bool m_bestPresses = false;
    bool m_executedPress = false;
    bool m_executedValid = false;
    // state the current advice was computed for (exact match required)
    float m_stateX = 0.f, m_stateY = 0.f, m_stateVy = 0.f;
    bool m_stateGround = false, m_stateFlipped = false;

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
    float m_interval = 1.f;
    // Button state that acted during the physics step ending at `step` (as
    // sent, before any latency), from the history; `current` covers steps
    // after the newest record (the step being observed now).
    bool sentAt(int32_t step, int currentFrom = INT32_MAX, bool current = false) const;
    std::vector<uint8_t> m_holdBuf;
    PredictedPath m_resim;
    float m_predY = 0.f, m_actY = 0.f, m_stepErr = 0.f;
};

} // namespace aigd
