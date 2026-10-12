#include <aigd/Physics.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace aigd {

namespace {
CubePhysics defaults(bool mini) {
    CubePhysics p;
    if (mini) {
        // Not measured yet: scaled guesses, the calibrator replaces them quickly.
        p.takeoffVy = 9.18f;
        p.halfSize = 9.f;
        p.innerHalf = 2.7f;
        p.floorY = 99.f;
    }
    return p;
}
} // namespace

PhysicsModel::PhysicsModel() {
    reset();
}

void PhysicsModel::reset() {
    m_cal = PhysicsCalibration{};
    m_params = {defaults(false), defaults(true)};
    m_drift = 0.25f;
    m_lastResimError = 0.f;
    m_accuracyN = 0;
}

void PhysicsModel::loadCalibration(PhysicsCalibration const& cal) {
    m_cal = cal;
    refit(0);
    refit(1);
}

float PhysicsModel::vxPerTick(int speedClass) const {
    int i = std::clamp(speedClass, 0, 4);
    if (m_cal.vxN[i] >= 10) return static_cast<float>(m_cal.vxSum[i] / m_cal.vxN[i]);
    return unitsPerSecond(i) / kTicksPerSecond;
}

bool PhysicsModel::calibrated(bool mini) const {
    auto const& s = m_cal.size[mini ? 1 : 0];
    return s.v0N >= 2 && s.gN >= 20 && s.kN >= 20;
}

void PhysicsModel::observeStep(Observation const& prev, Observation const& cur, bool held) {
    if (prev.mode != GameMode::Cube || cur.mode != GameMode::Cube) return;
    if (prev.mini != cur.mini || prev.upsideDown != cur.upsideDown) return;
    if (cur.tick - prev.tick != 1 || prev.dead || cur.dead) return;

    int i = cur.mini ? 1 : 0;
    auto& s = m_cal.size[i];
    auto const& p = m_params[i];

    // horizontal speed
    float dx = cur.x - prev.x;
    if (prev.speedClass() == cur.speedClass() && dx > 0.f && dx < 4.f) {
        int sc = cur.speedClass();
        double mean = m_cal.vxN[sc] ? m_cal.vxSum[sc] / m_cal.vxN[sc] : dx;
        if (m_cal.vxN[sc] < 10 || std::fabs(dx - mean) < 0.05 * mean) {
            m_cal.vxSum[sc] += dx;
            ++m_cal.vxN[sc];
        }
    }

    float sign = cur.upsideDown ? -1.f : 1.f;
    float vp = prev.vy * sign;
    float vc = cur.vy * sign;
    float dy = (cur.y - prev.y) * sign;
    bool bothAir = !prev.onGround && !cur.onGround;

    // gravity: velocity lost per airborne step (not at the fall cap)
    if (bothAir && vc < vp && std::fabs(vc) < p.maxFall * 0.98f) {
        float d = vp - vc;
        double mean = s.gN ? s.gSum / s.gN : p.gravity;
        bool ok = d > 0.02f && d < 1.f && (s.gN < 20 || std::fabs(d - mean) < std::max(0.02, 0.1 * mean));
        if (ok) {
            s.gSum += d;
            s.gSq += static_cast<double>(d) * d;
            ++s.gN;
        }
    }

    // vertical scale: y change per unit of velocity
    if (bothAir && std::fabs(vc) > 0.5f) {
        float ratio = dy / vc;
        double k = s.kVV > 0 ? s.kYV / s.kVV : p.yScale;
        bool ok = ratio > 0.05f && ratio < 1.f && (s.kN < 20 || std::fabs(ratio - k) < 0.1 * k);
        if (ok) {
            s.kYV += static_cast<double>(dy) * vc;
            s.kVV += static_cast<double>(vc) * vc;
            ++s.kN;
        }
    }

    // jump impulse: velocity on the first airborne step after pressing on the ground
    if (prev.onGround && !cur.onGround && held && vc > 1.f) {
        double mean = s.v0N ? s.v0Sum / s.v0N : vc;
        if (s.v0N < 2 || std::fabs(vc - mean) < 0.15 * mean) {
            s.v0Sum += vc;
            ++s.v0N;
        }
    }

    // terminal velocity: falling without accelerating any more
    if (bothAir && vc < -3.f && std::fabs(vp - vc) < 0.005f) {
        s.fallCap = s.fallCapN ? std::max(s.fallCap, -vc) : -vc;
        ++s.fallCapN;
    }

    // level floor: the lowest height the player stands at
    if (!cur.upsideDown && cur.onGround && prev.onGround && std::fabs(cur.vy) < 0.01f) {
        if (s.floorN == 0 || cur.y < s.floorY) s.floorY = cur.y;
        ++s.floorN;
    }

    refit(i);
}

void PhysicsModel::refit(int i) {
    auto const& s = m_cal.size[i];
    CubePhysics p = defaults(i == 1);
    if (s.gN >= 10) p.gravity = static_cast<float>(s.gSum / s.gN);
    if (s.kN >= 10 && s.kVV > 0) p.yScale = static_cast<float>(s.kYV / s.kVV);
    if (s.v0N >= 1) p.takeoffVy = static_cast<float>(s.v0Sum / s.v0N);
    if (s.fallCapN >= 3) p.maxFall = s.fallCap;
    if (s.floorN >= 1) p.floorY = std::min(p.floorY, s.floorY);
    m_params[i] = p;
}

void PhysicsModel::recordAccuracy(float error, int steps) {
    if (steps <= 0) return;
    float perTick = error / static_cast<float>(steps);
    // Slow exponential average: one odd window cannot swing the trust level.
    m_drift = m_accuracyN == 0 ? perTick : m_drift * 0.9f + perTick * 0.1f;
    m_lastResimError = error;
    ++m_accuracyN;
}

std::string PhysicsModel::describe() const {
    auto const& p = m_params[0];
    auto const& s = m_cal.size[0];
    char buf[200];
    std::snprintf(buf, sizeof(buf), "g=%.4f v0=%.3f k=%.4f cap=%.1f floor=%.1f (n %u/%u/%u)%s", p.gravity, p.takeoffVy, p.yScale,
                  p.maxFall, p.floorY, s.gN, s.kN, s.v0N, calibrated(false) ? "" : " [calibrating]");
    return buf;
}

} // namespace aigd
