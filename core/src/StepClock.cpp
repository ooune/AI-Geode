#include <aigd/StepClock.hpp>
#include <aigd/Types.hpp>

#include <algorithm>
#include <cmath>

namespace aigd {

int StepClock::advance(float x, int speedClass) {
    float const vx = unitsPerSecond(speedClass) / static_cast<float>(kTicksPerSecond);
    if (!m_has) {
        m_has = true;
        m_lastX = x;
        m_tick += kMaxGap * 2; // a new history: not connected to anything before
        return kDiscontinuity;
    }
    float const dx = x - m_lastX;
    float const steps = dx / vx;
    if (steps < -0.25f || steps > static_cast<float>(kMaxGap) + 0.5f) {
        m_lastX = x;
        m_tick += kMaxGap * 2;
        return kDiscontinuity;
    }
    int const n = static_cast<int>(std::lround(steps));
    if (n <= 0) return 0;
    m_lastX = x;
    m_tick += n;
    m_avg = m_avg * 0.95f + static_cast<float>(n) * 0.05f;
    if (n >= m_recentMax) {
        m_recentMax = n;
        m_sinceMax = 0;
    } else if (++m_sinceMax > 240) {
        m_recentMax = std::max(1, m_recentMax - 1); // forget an old outlier slowly
        m_sinceMax = 0;
    }
    return n;
}

} // namespace aigd
