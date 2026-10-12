#pragma once

#include <aigd/Types.hpp>

#include <array>
#include <cstdint>
#include <string>

namespace aigd {

// Cube physics per 240 Hz step. Defaults were measured from Geometry Dash
// 2.2081 (normal-size cube): vy drops by 0.2156 per step, y changes by
// vy * 0.225, and the first airborne step after a jump has vy = 10.964
// (11.180 jump impulse minus one step of gravity).
struct CubePhysics {
    float takeoffVy = 10.9644f;  // vy after the take-off step
    float gravity = 0.215595f;   // vy lost per airborne step
    float yScale = 0.225f;       // y += vy * yScale per step
    float maxFall = 15.f;        // terminal falling speed (|vy|)
    float halfSize = 15.f;       // outer hitbox half extent
    float innerHalf = 4.5f;      // inner hitbox half extent (solids kill only here)
    float floorY = 105.f;        // player centre when standing on the level floor
};

// Running sums used to calibrate the model; plain data so it can be saved.
struct PhysicsCalibration {
    struct Size {
        double gSum = 0, gSq = 0;
        uint32_t gN = 0;
        double kYV = 0, kVV = 0;
        uint32_t kN = 0;
        double v0Sum = 0;
        uint32_t v0N = 0;
        float fallCap = 0.f;
        uint32_t fallCapN = 0;
        float floorY = 0.f;
        uint32_t floorN = 0;
    };
    std::array<Size, 2> size; // [0] normal, [1] mini
    std::array<double, 5> vxSum{};
    std::array<uint32_t, 5> vxN{};
};

// Physics model of the player that is fitted to what the game actually does.
// Parameters only move once enough consistent samples exist, and single odd
// frames (orbs, pads, teleports) are rejected as outliers.
class PhysicsModel {
public:
    PhysicsModel();

    CubePhysics const& cube(bool mini) const { return m_params[mini ? 1 : 0]; }
    float vxPerTick(int speedClass) const;
    bool calibrated(bool mini) const;

    // Feed two consecutive observations (one physics step apart) and whether
    // the jump button was held during that step.
    void observeStep(Observation const& prev, Observation const& cur, bool heldDuringStep);

    // Accuracy of multi-step re-simulation (position error after n steps).
    void recordAccuracy(float error, int steps);
    float driftPerTick() const { return m_drift; }
    float lastResimError() const { return m_lastResimError; }
    uint32_t accuracySamples() const { return m_accuracyN; }

    PhysicsCalibration const& calibration() const { return m_cal; }
    void loadCalibration(PhysicsCalibration const& cal);
    void reset();

    std::string describe() const;

private:
    void refit(int i);

    PhysicsCalibration m_cal;
    std::array<CubePhysics, 2> m_params;
    float m_drift = 0.25f;   // pessimistic until measured
    float m_lastResimError = 0.f;
    uint32_t m_accuracyN = 0;
};

} // namespace aigd
