#pragma once

#include <cstdint>
#include <span>
#include <string>

namespace aigd {

// One block in Geometry Dash is 30 world units.
inline constexpr float kBlock = 30.f;
// GD 2.2 runs its physics at a fixed 240 steps per second.
inline constexpr int kTicksPerSecond = 240;

enum class GameMode : uint8_t { Cube = 0, Ship, Ball, Ufo, Wave, Robot, Spider, Swing, Count };

char const* gameModeName(GameMode mode);

// Modes where the button changes vertical acceleration continuously rather than
// triggering a discrete jump/flip.
inline bool isFlyingMode(GameMode m) {
    return m == GameMode::Ship || m == GameMode::Wave || m == GameMode::Swing;
}

// GD speed portals set PlayerObject::m_playerSpeed to one of these values.
// Index: 0 = 0.5x, 1 = 1x, 2 = 2x, 3 = 3x, 4 = 4x.
int speedClassFromPlayerSpeed(float playerSpeed);
// Horizontal velocity in world units per second for a speed class.
float unitsPerSecond(int speedClass);

// A nearby object relative to the player. dy is gravity-normalised: positive
// means "towards the ceiling from the player's point of view".
struct Feature {
    bool present = false;
    float dx = 0.f;
    float dy = 0.f;
};

// A collision shape near the player, in world units (same space as player x/y).
struct Obstacle {
    enum class Kind : uint8_t {
        Hazard,    // kills on contact with the player's outer hitbox
        Solid,     // can be landed on; kills if the player's inner hitbox enters it
        Uncertain, // orbs, pads, portals, slopes...: effect not modelled
    };
    float l = 0.f, b = 0.f, r = 0.f, t = 0.f;
    Kind kind = Kind::Hazard;
};

// Everything the agent may know about the current physics step. Filled by the
// Geode observer from real game state (or by the simulator in tests).
struct Observation {
    int32_t tick = 0;          // physics step index since attempt start
    float x = 0.f;             // player position (world units)
    float y = 0.f;
    float vy = 0.f;            // raw vertical velocity (GD units)
    GameMode mode = GameMode::Cube;
    bool onGround = false;
    bool upsideDown = false;   // gravity flipped
    bool mini = false;
    bool dead = false;
    float playerSpeed = 0.9f;  // raw PlayerObject::m_playerSpeed
    float levelLength = 1.f;   // x position of the end of the level
    Feature hazard;            // nearest dangerous object ahead
    Feature solid;             // nearest solid block ahead
    Feature orb;               // nearest clickable orb/ring ahead
    // Collision shapes around the player, sorted by left edge. Points into a
    // buffer owned by whoever filled the observation; valid for this step only.
    std::span<Obstacle const> obstacles;
    // Playing-area floor/ceiling surfaces reported by the game for corridor
    // modes (ship, UFO, ...). Only a hint: confirmed by actual contact.
    bool hasBoundsHint = false;
    float boundsHintBottom = 0.f;
    float boundsHintTop = 0.f;

    float progress() const;    // 0..1
    float vyNorm() const { return upsideDown ? -vy : vy; }
    int speedClass() const { return speedClassFromPlayerSpeed(playerSpeed); }
};

enum class KillerKind : uint8_t { Unknown = 0, Hazard, Solid };

struct DeathInfo {
    float x = 0.f;
    float y = 0.f;
    bool hasKiller = false;
    KillerKind killer = KillerKind::Unknown;
    float killerDx = 0.f;      // killer centre relative to player, gravity-normalised dy
    float killerDy = 0.f;
    // Vertical extent of the killer relative to the player centre (gravity-
    // normalised). A tall pillar from the floor has its centre above a low ship
    // but must be flown over, so the edges matter more than the centre.
    bool hasExtent = false;
    float killerTop = 0.f;
    float killerBottom = 0.f;
};

enum class FailureKind : uint8_t {
    None = 0,
    NoJump,          // ran into an obstacle without pressing
    TooEarly,        // pressed too early (came down on/before the obstacle)
    TooLate,         // pressed too late (hit the obstacle on the way up)
    HeldTooLong,     // flew/jumped into something above
    ReleasedTooLong, // fell into something below
    HitWall,         // straight into a wall, direction unclear
    Unknown,
    Count
};

char const* failureKindName(FailureKind kind);

enum class AgentPhase : uint8_t { Idle = 0, Observing, Playing, Learning, Retrying, Completed, Disabled };

char const* agentPhaseName(AgentPhase phase);

// Where a decision came from. Shown in the diagnostics and recorded per step so
// that learning can tell learned choices from tests and safety overrides.
enum class DecisionSource : uint8_t {
    Policy = 0,   // learned values of this exact state
    Prior,        // no own experience yet: values of similar situations (other levels/places)
    Planner,      // no learned preference: the trajectory planner's best plan
    Explore,      // random exploration near the current trouble spot
    Experiment,   // the one change a timing experiment is testing
    Reference,    // reproducing the reference attempt (same state) so an experiment is a fair comparison
    KnownGood,    // the other action reliably got through here and the learned choice has no such record
    AvoidRepeat,  // the learned choice was the cause of repeated deaths here: testing the alternative
    Shield,       // the chosen action was predicted to collide; the other was not
    Debounce,     // kept the button state (minimum time between changes)
    Economy,      // a press was proposed, but not pressing is predicted clear: no input
    Count
};

char const* decisionSourceName(DecisionSource source);

} // namespace aigd
