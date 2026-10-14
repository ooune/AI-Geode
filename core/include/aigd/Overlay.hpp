#pragma once

#include <aigd/Agent.hpp>
#include <aigd/LevelAnalysis.hpp>
#include <aigd/LevelMap.hpp>
#include <aigd/StrategicPlanner.hpp>
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
        OrbBox,       // an orb's activation area
        OrbActivated, // the path if the orb is activated (not the selected action)
        OrbSkipped,   // the path if it is not (not the selected action)
        OrbPoint,     // where the activation happens
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
    bool hasEndpoint = false;
    float endpointX = 0.f, endpointY = 0.f;
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
    // orb layer: the nearest orb, both outcomes, and which one is selected
    bool hasOrb = false;
    bool orbSelected = false;     // the selected (executed) path activates it
    std::string orbLabel;
    float orbLabelX = 0.f, orbLabelY = 0.f;

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
// While the AI follows its live plan, `plan` is drawn from the current step
// on instead (the path that is actually being followed).
bool buildDecisionOverlay(Predictor const& predictor, SafetyRecord const& decision, int32_t nowTick, bool showAlternative,
                          std::span<Obstacle const> obstacles, OverlayGeometry& out, LivePlan const* plan = nullptr, bool showOrbs = true);

// The orb layer alone (added by buildDecisionOverlay when showOrbs): the
// orb's area; the selected path is the executed one (drawn by the decision
// overlay, with the orb's effect only if the executed input activates it);
// the other outcome is drawn in its own style (activated / skipped), plus
// the activation point, the landing and collision of the activation.
void addOrbLayer(Predictor const& predictor, OverlayGeometry& out, int stride = 4);

// ---------------------------------------------------------------------------
// "What the AI believes is in the level": the level-understanding debug view.
// Styles keep three things apart: objects actually extracted from the level
// (filled), geometry the AI inferred from them (outlined), and predicted
// movement (lines).

struct UnderstandingLayers {
    bool overview = true;      // compact strip of the whole level
    bool objects = true;       // extracted objects around the player
    bool groups = true;        // inferred obstacle groups / the planned sequence
    bool landing = true;       // inferred landing zones and the strategic target
    bool plan = true;          // the strategic plan: presses and its predicted path
    bool alternatives = false; // other first-press timings that were evaluated
    bool uncertain = true;     // objects that are not simulated, moved by triggers, or approximate
};

struct UnderstandingOverlay {
    enum class Style : uint8_t {
        Hazard, Solid, Portal, NotSimulated, Dynamic, Approximate, // extracted
        Group, TightGroup, Sequence, LandingZone, Target,          // inferred
        PlannedPath, AlternativePath, Press,                       // predicted
        Player,
    };
    struct Box {
        float l, b, r, t;
        Style style;
        bool filled;
    };
    struct Segment {
        float x0, y0, x1, y1;
        Style style;
    };
    struct Dot {
        float x, y;
        Style style;
    };
    // overview strip: x as a fraction of the level (0..1), one row per kind
    struct Mark {
        float f0, f1;
        Style style;
        int row; // 0 objects, 1 groups, 2 physics changes
    };
    std::vector<Box> boxes;
    std::vector<Segment> segments;
    std::vector<Dot> dots;
    std::vector<Mark> overview;
    float playerFraction = 0.f;
    std::vector<std::string> warnings;
    void clear();
};

// The overview strip of the whole level (built once per level; cheap to keep).
void buildLevelOverview(LevelMap const& map, LevelAnalysis const& analysis, std::vector<UnderstandingOverlay::Mark>& out, size_t maxMarks = 1500);

// World-space layers for [x0, x1] around the player. Any pointer may be null.
void buildUnderstandingOverlay(LevelMap const* map, LevelAnalysis const* analysis, StrategicPlan const* plan, float playerX, float playerY,
                               float x0, float x1, UnderstandingLayers const& layers, UnderstandingOverlay& out, int stride = 4);

} // namespace aigd
