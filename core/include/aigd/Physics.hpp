#pragma once

#include <aigd/Types.hpp>

#include <array>
#include <cstdint>
#include <string>

namespace aigd {

// What the jump button does in a game mode.
enum class PressAction : uint8_t {
    None,              // ship: holding changes acceleration only
    JumpOnGround,      // cube, robot: jump while touching the ground (holding re-jumps)
    ImpulseOnEdge,     // UFO: every new press gives an upward kick, also in the air
    FlipOnGround,      // ball: flip gravity while touching the ground
    FlipOnEdge,        // swing: every new press flips gravity, also in the air
    TeleportOnGround,  // spider: a new press on the ground jumps instantly to the opposite surface
    SetVelocity,       // wave: held = move diagonally up, released = down
};

// Per-step vertical physics of one game mode (and size). All vertical values
// are gravity-normalised: positive = away from the surface gravity pulls to.
struct ModePhysics {
    PressAction press = PressAction::JumpOnGround;
    float accHold = -0.215595f;    // velocity change per airborne step while held
    float accRelease = -0.215595f; // ... while released
    float maxUp = 30.f;            // velocity caps
    float maxDown = 15.f;
    float yScale = 0.225f;         // y += vy * yScale per step
    float impulse = 10.9644f;      // velocity after a jump/flap step
    float flipVy = 0.f;            // velocity (new gravity frame) right after a flip
    float waveVy = 5.77f;          // wave: |vy| while moving
    int boostSteps = 0;            // robot: accHold applies only this long after take-off
    float halfSize = 15.f;         // outer hitbox half extent (assumption, not read from the game)
    float innerHalf = 4.5f;        // inner hitbox half extent (assumption)
};

struct RunningStat {
    double sum = 0.0;
    uint32_t n = 0;
    // Transient (not saved): an unbroken run of rejected samples that agree
    // with each other. If it grows long enough the physics really changed (or
    // the first estimate was wrong) and the statistic restarts from that run.
    double altSum = 0.0;
    uint32_t altN = 0;
    uint32_t regimeChanges = 0;
    void add(double v) {
        sum += v;
        ++n;
    }
    double mean(double fallback) const { return n ? sum / n : fallback; }
};

// Running sums used to calibrate one mode/size; plain data so it can be saved.
struct ModeCalibration {
    RunningStat accHold, accRelease, impulse, flipVy, waveVy;
    double kYV = 0.0, kVV = 0.0;
    uint32_t kN = 0;
    float maxUp = 0.f, maxDown = 0.f;
    uint32_t capUpN = 0, capDownN = 0;
    uint32_t flips = 0, teleports = 0;
    float floorY = 0.f;   // cube only: the level floor (GD's ground)
    uint32_t floorN = 0;
    uint32_t boostMax = 0; // robot: longest boost seen after a take-off (steps)
};

// What activating an orb does, gravity-normalised (+ = away from the surface
// gravity pulls to, in the gravity frame *after* the activation).
struct OrbPhysics {
    bool known = false;        // calibrated from activations observed in the game
    bool flips = false;        // the activation flips gravity
    float vy = 0.f;            // vertical velocity right after the activation
    // dash orbs: a straight line while the button is held
    bool dash = false;
    bool dashKnown = false;    // the dash movement itself has been observed
    float dashDx = 1.f;        // x advance per step during the dash, relative to the normal x speed
    float dashSlope = 1.f;     // (dy/dx during the dash) / tan(the direction from the orb rotation)
    float dashEndVy = 0.f;     // vertical velocity right after the dash ends
};

// Running sums for one orb type in one mode/size (plain data, saved).
struct OrbCalibration {
    RunningStat vy;            // normalised vy after activation
    uint32_t n = 0, flips = 0; // activations, and how many flipped gravity
    RunningStat dashDx, dashSlope, dashEndVy;
    uint32_t dashEndsReleased = 0, dashEndsOther = 0;
};

struct PhysicsCalibration {
    static constexpr size_t kSlots = static_cast<size_t>(GameMode::Count) * 2;
    static constexpr size_t kOrbSlots = static_cast<size_t>(OrbType::Count) * kSlots;
    std::array<ModeCalibration, kSlots> modes{};
    std::array<double, 5> vxSum{};
    std::array<uint32_t, 5> vxN{};
    std::array<OrbCalibration, kOrbSlots> orbs{};
    // Holding the button from before reaching an orb: did the orb activate?
    uint32_t heldEntryActivated = 0, heldEntryIgnored = 0;

    static size_t slot(GameMode m, bool mini) { return static_cast<size_t>(m) * 2 + (mini ? 1 : 0); }
    static size_t orbSlot(OrbType t, GameMode m, bool mini) { return static_cast<size_t>(t) * kSlots + slot(m, mini); }
};

// Physics model of the player for every game mode, fitted to what the game
// actually does. Defaults are GD 2.2081 values where measured (cube) and
// rough guesses elsewhere; a mode is only "calibrated" (trusted) once enough
// of its own events have been observed. Single odd frames (orbs, pads,
// teleport portals) are rejected as outliers.
class PhysicsModel {
public:
    PhysicsModel();

    ModePhysics const& params(GameMode mode, bool mini) const { return m_params[PhysicsCalibration::slot(mode, mini)]; }
    ModePhysics const& cube(bool mini) const { return params(GameMode::Cube, mini); }
    float cubeFloorY(bool mini) const;
    float vxPerTick(int speedClass) const;
    bool calibrated(GameMode mode, bool mini) const;
    bool calibrated(bool mini) const { return calibrated(GameMode::Cube, mini); }
    static ModePhysics defaults(GameMode mode, bool mini);

    // Two consecutive observations (one physics step apart) and the button
    // state during that step / the step before (to detect new presses).
    void observeStep(Observation const& prev, Observation const& cur, bool heldDuringStep, bool heldBefore);

    // ---- orbs ----
    // The effect of an orb type in a mode (calibrated values once observed,
    // otherwise guesses with known = false).
    OrbPhysics orb(OrbType type, GameMode mode, bool mini) const;
    // An activation reported by the game: learns the velocity and gravity
    // after it. Returns actual - predicted velocity (0 for dash orbs).
    float observeOrb(OrbEvent const& e);
    // One step of a dash (positions one physics step apart, gravity-normalised dy).
    void observeDashStep(OrbType type, GameMode mode, bool mini, float dx, float dy, float rotation, int speedClass);
    // The end of a dash: velocity right after it, and whether the button had been released.
    void observeDashEnd(OrbType type, GameMode mode, bool mini, float vyNormalised, bool released);
    // Whether holding the button from before reaching an orb activates it
    // (-1 not known yet, 0 no, 1 yes), learned from what the game did.
    int heldEntryActivates() const;
    void observeHeldEntry(bool activated);
    std::string describeOrbs() const;

    void recordAccuracy(GameMode mode, float error, int steps);
    float driftPerTick(GameMode mode) const;
    float driftPerTick() const { return driftPerTick(GameMode::Cube); }
    float lastResimError() const { return m_lastResimError; }
    uint32_t accuracySamples() const { return m_accuracyN; }
    // How often an estimate was replaced because the game consistently did
    // something else (physics changed, or the first estimate was wrong).
    uint32_t regimeChanges() const {
        uint32_t n = m_vxReplaced;
        for (auto const& c : m_cal.modes) {
            for (auto const* st : {&c.accHold, &c.accRelease, &c.impulse, &c.flipVy, &c.waveVy}) n += st->regimeChanges;
        }
        return n;
    }

    PhysicsCalibration const& calibration() const { return m_cal; }
    void loadCalibration(PhysicsCalibration const& cal);
    void reset();

    std::string describe(GameMode mode) const;
    std::string describe() const { return describe(GameMode::Cube); }

private:
    void refit(size_t slot);

    PhysicsCalibration m_cal;
    std::array<ModePhysics, PhysicsCalibration::kSlots> m_params;
    std::array<float, static_cast<size_t>(GameMode::Count)> m_drift{};
    std::array<uint32_t, static_cast<size_t>(GameMode::Count)> m_driftN{};
    float m_lastResimError = 0.f;
    uint32_t m_accuracyN = 0;
    int32_t m_robotSinceTakeoff = 1 << 20;
    std::array<double, 5> m_vxAltSum{};   // x speed: run of samples disagreeing with the estimate
    std::array<uint32_t, 5> m_vxAltN{};
    uint32_t m_vxReplaced = 0;
};

} // namespace aigd
