#pragma once

#include <aigd/Predictor.hpp>
#include <aigd/Types.hpp>

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace aigd {

// Where the live plan's timing came from.
enum class PlanSource : uint8_t { None = 0, Search, Learned, Fallback };
char const* planSourceName(PlanSource s);

// One physics step of the live plan: the button state to use, and the state
// the player is expected to be in after that step.
struct PlannedStep {
    int32_t tick = 0;
    uint8_t hold = 0;
    float x = 0.f, y = 0.f;
    bool grounded = false;
};

// The live execution plan: the predicted path of the action that is being
// executed and its planned continuation (pending inputs). It is the short-term
// "what I am doing now", kept apart from everything learned: cancelling it
// never touches the learning memory.
struct LivePlan {
    bool active = false;
    int32_t createdTick = 0;
    GameMode mode = GameMode::Cube;
    int speedClass = 1;
    bool upsideDown = false;
    bool mini = false;
    std::vector<PlannedStep> steps; // steps createdTick+1, createdTick+2, ...
    Plan params;                    // the candidate's parameters (wait, hold, second press)
    Certainty outcome = Certainty::Safe;
    float score = 0.f;
    int candidates = 0;             // candidates simulated to choose it
    SearchStage stage = SearchStage::None;
    PlanSource source = PlanSource::None;
    bool landed = false;
    float landX = 0.f, landY = 0.f;
    float startX = 0.f, startY = 0.f; // the state it was made from
    bool startGrounded = false;
    // Made by the prediction itself (planner, safety check, "no input
    // needed"): its continuation is what the AI intends, so it can be
    // followed. A plan behind a learned-policy decision is only checked
    // against reality; the policy keeps deciding.
    bool followable = false;

    // Index of the step for `tick` (-1 before the plan, size() after it).
    int indexOf(int32_t tick) const;
    // Input changes still ahead after `tick` (the pending actions).
    int pendingChanges(int32_t tick) const;
    int32_t endTick() const { return createdTick + static_cast<int32_t>(steps.size()); }
};

// Result of checking the live plan against the real state.
struct PlanCheck {
    bool hadPlan = false;
    bool valid = false;
    bool finished = false;      // the plan ran out (not a failure)
    std::string reason;         // why it was invalidated, in words
    int cancelled = 0;          // pending input changes that were cancelled
    float dx = 0.f, dy = 0.f;   // actual minus expected
    bool expectedGrounded = false;
    bool actualGrounded = false;
};

// A finished, cancelled or replaced plan (diagnostic history; the learning
// memory keeps its own records).
struct PlanRecord {
    int32_t createdTick = 0;
    int32_t endedTick = 0;
    Plan params;
    PlanSource source = PlanSource::None;
    bool cancelled = false;
    int cancelledActions = 0;
    std::string reason;
};

class PlanManager {
public:
    // Adopts the predicted path of the action just executed (path step i =
    // physics step tick+1+i) as the live plan, replacing the previous one.
    void adopt(PredictedPath const& path, Observation const& from, int32_t tick, PlanSource source, Predictor::Advice const& adv,
               bool followable);
    // Compares the real state with what the plan expected for this tick and
    // cancels the plan when they differ materially. `tolerance` is the
    // vertical deviation (units) still considered "as planned"; it should
    // grow with the model's measured error.
    PlanCheck validate(Observation const& obs, float tolerance);
    // Drops the live plan (e.g. the AI stopped following it); not counted as
    // an invalidation.
    void release(int32_t tick, char const* why);
    void clear();

    LivePlan const& plan() const { return m_plan; }
    bool active() const { return m_plan.active; }
    // Button state the plan wants for the step starting at `tick` (the step
    // tick+1), or -1 without a plan entry there.
    int holdFor(int32_t tick) const;
    int32_t age(int32_t tick) const { return tick - m_plan.createdTick; }
    int32_t remaining(int32_t tick) const { return m_plan.endTick() - tick; }

    // counters (per attempt; reset with clear())
    uint32_t adopted = 0;
    uint32_t invalidated = 0;
    uint32_t cancelledActions = 0;
    PlanCheck lastCheck;            // the most recent validation
    PlanCheck lastInvalidation;     // the most recent one that cancelled the plan
    std::deque<PlanRecord> history; // most recent last

private:
    void archive(int32_t tick, bool cancelled, int cancelledActions, std::string reason);
    LivePlan m_plan;
};

} // namespace aigd
