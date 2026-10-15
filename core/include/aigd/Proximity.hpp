#pragma once

#include <aigd/Types.hpp>

#include <span>

namespace aigd {

// How close a player hitbox is to dying, by the collision rules the
// predictor uses for GD:
//  - hazards kill on contact with the player's outer hitbox;
//  - blocks kill only when the small inner hitbox enters them from the side
//    the player runs into (their left face). Landing on a block, sliding
//    along its underside or walking off it is not a danger.
// Floor/ceiling bounds and objects whose effect is not simulated (orbs,
// pads, portals) are not dangers. Distances are between rectangles (the
// game's collision rectangles, not the artwork), in world units; <= 0 means
// the hitboxes touch or overlap.
struct ProximityConfig {
    float safe = 18.f;            // at or above this clearance a path is "safe" (green)
    float danger = 4.f;           // at or below this it is "dangerously close" (red)
    float range = 40.f;           // obstacles further than this are not measured (clearance is capped here)
    float scoreWeight = 10.f;     // score bonus at `safe` clearance (linear from 0)
    float nearMissPenalty = 6.f;  // score penalty at 0 clearance (linear from `danger`)
    float learnPenalty = 0.3f;    // learning: reward taken from a real step this close to dying (at 0 clearance)
    // Scoring may add this many units per step ahead to the clearance (a close
    // call far ahead being a later decision's problem). Measured: 0.1 made the
    // AI fly closer (it ignored the back corner of the spike it was jumping),
    // so the default is 0: the whole path counts.
    float discountPerStep = 0.f;
    bool legacyScore = false;     // score with the pre-v0.7.0 clearance term instead (setting / comparisons)
};

// Clearance of one obstacle to a player hitbox centred at (x, y) with outer
// half size h and inner half size in. Returns `cap` when the obstacle cannot
// kill (unmodelled) or is further away.
float obstacleClearance(float x, float y, float h, float in, Obstacle const& o, float cap);

// The smallest clearance over a set of obstacles (sorted by l), and which one.
struct ClearanceResult {
    float clearance = 0.f;
    int obstacle = -1;
};
ClearanceResult hitboxClearance(float x, float y, float h, float in, std::span<Obstacle const> obstacles, float cap);

// 0 = safe (green) .. 0.5 = approaching danger (orange) .. 1 = dangerously close (red); smooth in between.
float riskOf(float clearance, ProximityConfig const& cfg);

struct RiskColor {
    float r, g, b;
};
// Green -> orange -> red, interpolated continuously.
RiskColor riskColor(float risk);

} // namespace aigd
