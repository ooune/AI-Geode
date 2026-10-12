#pragma once

#include <aigd/Types.hpp>

#include <cstdint>
#include <vector>

namespace aigd {

// One decision made by the agent during an attempt. A decision is taken every
// time the discretised state changes (and at least every maxStickTicks), so a
// step covers one or more physics ticks during which the action is held.
struct Step {
    uint64_t key = 0;      // level state key
    uint64_t pkey = 0;     // pattern (cross-level) key
    float x = 0.f;
    float y = 0.f;
    float vy = 0.f;
    int32_t tick = 0;
    float reward = 0.f;    // reward collected until the next decision
    GameMode mode = GameMode::Cube;
    bool onGround = false;
    uint8_t action = 0;    // 0 = released, 1 = held
    bool forced = false;   // chosen by a timing experiment
    bool explored = false; // chosen by random exploration
    bool planned = false;  // chosen by the trajectory planner (no learned preference yet)
    bool shielded = false; // the predicted collision of the other action was avoided
};

// A contiguous stretch of steps with the button held.
struct PressBlock {
    size_t firstStep = 0;
    size_t lastStep = 0;   // inclusive
    float startX = 0.f;
    float endX = 0.f;      // x where the button was released (or the attempt ended)
    int32_t startTick = 0;
    int32_t endTick = 0;
    bool openAtEnd = false; // still held when the attempt ended
    // In jump-like modes a press only does something if the player touches the
    // ground while it is held; a mid-air tap is a no-op and must not be blamed.
    bool effective = true;
    // Number of separate ground contacts while held. In jump modes holding the
    // button re-jumps on landing, so >1 means one long hold made several jumps.
    int groundContacts = 0;
};

// Modes where the button only acts when the player is on the ground.
inline bool needsGroundToAct(GameMode m) {
    return m == GameMode::Cube || m == GameMode::Robot || m == GameMode::Ball || m == GameMode::Spider;
}

std::vector<PressBlock> findPressBlocks(std::vector<Step> const& steps, float endX, int32_t endTick);

} // namespace aigd
