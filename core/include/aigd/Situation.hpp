#pragma once

#include <aigd/Types.hpp>

#include <cstdint>
#include <string>

namespace aigd {

// Transferable description of the obstacle group ahead of the player: only
// relative geometry (heights relative to the player's feet, widths and gaps),
// the player's mode, gravity, size and ground state - never a level position.
// Two levels that put the same obstacles in front of the player in the same
// state produce the same description, wherever they are in the level.
struct SituationElement {
    enum Kind : uint8_t {
        Hazard = 0,   // hazard on the walking path (must be jumped over)
        Solid,        // block on the walking path (wall, step or platform)
        HazardAbove,  // hazard above the walking path (limits the jump)
        SolidAbove,   // block above the walking path (a ceiling)
    };
    uint8_t kind = Hazard;
    int8_t start = 0;   // half-blocks from the group's first element
    int8_t width = 0;   // half-blocks
    int8_t bottom = 0;  // bottom edge above the player's feet, half-blocks (gravity-normalised)
    int8_t top = 0;     // top edge above the player's feet, half-blocks
};

struct Situation {
    static constexpr int kMaxElements = 4;

    bool valid = false;        // an obstacle group is ahead within the look-ahead
    GameMode mode = GameMode::Cube;
    bool mini = false;
    bool upsideDown = false;
    bool grounded = false;
    int speedClass = 1;
    bool unmodelled = false;   // an orb/pad/portal lies in or right after the group
    int count = 0;             // elements in the group (on the path and above it)
    bool truncated = false;    // the group had more elements than are described
    SituationElement el[kMaxElements];
    int landingBand = 3;       // free path after the group: 0 <1 block, 1 <2, 2 <4, 3 >=4 blocks

    // Runtime only (never stored): where the group is, and when it is reached.
    float startX = 0.f, endX = 0.f;   // world x of the group's first left edge and last right edge
    float contactSteps = 0.f;         // physics steps until the player's front edge reaches the group
    float landingSpace = 0.f;         // units of free path after the group

    uint64_t fine = 0;         // the whole layout + landing space (exact match)
    uint64_t coarse = 0;       // the first element + number of elements (similar situation)

    // e.g. "double spike, landing space 3 blocks"
    std::string describe() const;
    // e.g. "spike-clearing", "block-climbing": for the transfer messages.
    char const* category() const;
};

// Steps the look-ahead covers (at the current speed) for the start of a group.
inline constexpr float kSituationLookAheadSteps = 110.f;

// Describes the obstacle group ahead from the observation's collision shapes.
// Only for modes that act from the ground (cube, robot); invalid otherwise or
// when nothing blocks the path within the look-ahead.
Situation describeSituation(Observation const& obs, float lookAheadSteps = kSituationLookAheadSteps);

// The fine key the same layout would have with another landing space (used
// to tell "the landing space differs" apart from "never seen").
uint64_t situationKeyWithLanding(Situation const& s, int landingBand);

// The player's outer hitbox half extent used for situations (and nothing
// else): 15 units, 9 when mini.
inline float situationHalfSize(bool mini) { return mini ? 9.f : 15.f; }

} // namespace aigd
