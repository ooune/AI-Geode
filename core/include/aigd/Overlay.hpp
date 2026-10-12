#pragma once

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

    void clear();
};

// Builds the overlay for a main path and an optional alternative. `stride`
// keeps every n-th simulated point (the first and last are always kept), so
// a 180-step prediction becomes ~45 segments.
void buildOverlay(PredictedPath const& main, PredictedPath const* alternative, std::span<Obstacle const> obstacles,
                  OverlayGeometry& out, int stride = 4, size_t maxHazards = 24);

} // namespace aigd
