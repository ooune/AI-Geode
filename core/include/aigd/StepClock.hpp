#pragma once

#include <cstdint>

namespace aigd {

// Counts the real physics steps between two observations of the player.
//
// In game the AI is not called on every physics step: traces recorded in
// Geometry Dash show 1, 2 or 3 steps between consecutive calls, and the
// level-time based tick did not reflect that. Everything that reasons in
// steps (physics calibration, accuracy checks, plans, timings) needs the true
// count. The player moves horizontally by exactly one speed unit per step,
// so the count is read from the horizontal movement.
class StepClock {
public:
    void reset() { m_has = false; }

    // Feeds the player's x. Returns the number of physics steps since the
    // previous observation: 0 = still the same step (nothing to decide), >0 =
    // that many steps, and kDiscontinuity when the movement cannot be
    // explained by steps (respawn, teleport): the tick then jumps so that
    // nothing treats the two observations as connected.
    int advance(float x, int speedClass);

    int32_t tick() const { return m_tick; }
    // Average steps per observation (1 = every step observed) and the largest
    // gap seen recently.
    float averageInterval() const { return m_avg; }
    int recentMaxInterval() const { return m_recentMax; }

    static constexpr int kDiscontinuity = -1;
    static constexpr int kMaxGap = 8;

private:
    bool m_has = false;
    float m_lastX = 0.f;
    int32_t m_tick = 0;
    float m_avg = 1.f;
    int m_recentMax = 1;
    int m_sinceMax = 0;
};

} // namespace aigd
