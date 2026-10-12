#pragma once

#include <aigd/Agent.hpp>
#include <aigd/Physics.hpp>
#include <aigd/Predictor.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace aigd {

// What the in-game trajectory overlay draws, in world coordinates. Built here
// (not in the Geode code) so tests can check that the drawing matches the
// simulation exactly.
struct OverlayGeometry {
    enum class Style : uint8_t {
        PathSafe,
        PathUncertain,
        PathDanger,
        Alternative,
        Landing,
        Collision,
        Hazard,
        HazardHit,
        Keep,     // what keeping the current button state would do
        Stale,    // a prediction that was not computed for the current step
    };
    struct Segment {
        float x0, y0, x1, y1;
        Style style;
    };
    struct Dot {
        float x, y;
        Style style;
    };
    struct Box {
        float l, b, r, t;
        Style style;
    };

    std::vector<Segment> segments;
    std::vector<Dot> dots;
    std::vector<Box> boxes;
    bool hasCollision = false;
    float collisionX = 0.f, collisionY = 0.f;
    int32_t stepsToCollision = -1;
    Certainty outcome = Certainty::Safe;
    std::string label; // e.g. "collision in 37 steps" / "safe"
    // decision overlay: what the main path is
    bool selectedPress = false;   // the main path is the executed action's branch with this button state
    bool stale = false;           // prediction not computed for the current step
    int32_t predictionAge = 0;
    SafetyVerdict verdict = SafetyVerdict::NoPrediction;

    void clear();
};

// Builds the overlay for a main path and an optional alternative. `stride`
// keeps every n-th simulated point (the first and last are always kept), so
// a 180-step prediction becomes ~45 segments.
void buildOverlay(PredictedPath const& main, PredictedPath const* alternative, std::span<Obstacle const> obstacles,
                  OverlayGeometry& out, int stride = 4, size_t maxHazards = 24);

// The overlay of the decision that was actually executed: the main path is
// the executed action's branch (never a plan the controller is not
// following), plus "keep the current input" when that differs, the other
// action if wanted, and a label with the action, its safety, the time to a
// collision and whether the safety layer overrode the policy. Returns false
// when there is nothing current to show (the label then says why).
bool buildDecisionOverlay(Predictor const& predictor, SafetyRecord const& decision, int32_t nowTick, bool showAlternative,
                          std::span<Obstacle const> obstacles, OverlayGeometry& out);

} // namespace aigd
