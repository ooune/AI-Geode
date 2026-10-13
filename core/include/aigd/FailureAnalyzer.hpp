#pragma once

#include <aigd/Trajectory.hpp>
#include <aigd/Types.hpp>

#include <string>
#include <vector>

namespace aigd {

struct FailureReport {
    FailureKind kind = FailureKind::None;
    float deathX = 0.f;
    float deathY = 0.f;
    float deathProgress = 0.f;
    int32_t hotspot = 0;          // one-block bucket of the death position
    GameMode mode = GameMode::Cube;
    // Index into the press blocks of the attempt that is most likely responsible
    // (-1 when the problem is a missing press).
    int responsibleBlock = -1;
    // Which way the player should have been: +1 higher, -1 lower (the killer
    // was above), 0 unknown.
    int needDir = 0;
    // Bottom of what killed the player (gravity-normalised world height).
    bool hasKillerExtent = false;
    float killerBottom = 0.f;
    bool upsideDown = false;
    std::vector<PressBlock> blocks;
    float windowStartX = 0.f;     // how far back the analysis looked
    std::string text;             // human readable explanation
};

// Window (in world units) behind a death that is searched for the cause.
float analysisWindow(int speedClass, uint32_t escalation);

// Classifies why an attempt died, using the agent's own trajectory plus what the
// game reported about the collision. This never assumes that the last action was
// wrong: it looks back over the recent press blocks and the physics state at the
// time of death (rising vs. falling, obstacle above vs. below vs. ahead).
FailureReport analyzeFailure(
    std::vector<Step> const& steps,
    Observation const& last,
    DeathInfo const& death,
    uint32_t escalation
);

} // namespace aigd
