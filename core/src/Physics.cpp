#include <aigd/Physics.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace aigd {

namespace {
// Modes where holding the button does not change the falling acceleration, so
// held and released samples measure the same thing.
bool pooledGravity(GameMode m) {
    return m != GameMode::Ship && m != GameMode::Robot && m != GameMode::Wave;
}

// Samples needed in a row, all disagreeing with the current estimate but
// agreeing with each other, before the estimate is replaced. Odd single frames
// (orbs, pads, portals) never come close; a real change produces this many
// within a fraction of a second.
constexpr uint32_t kRegimeRun = 40;

// Adds v to s unless it is an outlier of an established estimate. Returns
// whether it was added.
bool offerAround(RunningStat& s, double v, double mean, bool warm, double tol) {
    if (!warm || std::fabs(v - mean) < tol) {
        s.add(v);
        s.altSum = 0.0;
        s.altN = 0;
        return true;
    }
    if (s.altN && std::fabs(v - s.altSum / s.altN) >= tol) {
        s.altSum = 0.0;
        s.altN = 0;
    }
    s.altSum += v;
    ++s.altN;
    if (s.altN >= kRegimeRun) {
        s.sum = s.altSum;
        s.n = s.altN;
        s.altSum = 0.0;
        s.altN = 0;
        ++s.regimeChanges;
        return true;
    }
    return false;
}

bool offerSample(RunningStat& s, double v, double fallback, double relTol, double absTol) {
    double mean = s.mean(fallback);
    return offerAround(s, v, mean, s.n >= 20, std::max(absTol, relTol * std::fabs(mean)));
}
} // namespace

ModePhysics PhysicsModel::defaults(GameMode mode, bool mini) {
    // Cube values were measured from GD 2.2081; everything else is a starting
    // guess that the calibrator replaces from real observations.
    ModePhysics p;
    float scale = mini ? 0.6f : 1.f;
    p.halfSize = 15.f * scale;
    p.innerHalf = 4.5f * scale;
    switch (mode) {
        case GameMode::Cube:
            p.press = PressAction::JumpOnGround;
            if (mini) p.impulse = 9.18f;
            break;
        case GameMode::Robot:
            p.press = PressAction::JumpOnGround;
            p.impulse = 7.f;
            p.accHold = -0.08f;
            p.boostSteps = 60;
            break;
        case GameMode::Ship:
            p.press = PressAction::None;
            p.accHold = 0.18f;
            p.accRelease = -0.18f;
            p.maxUp = 8.f;
            p.maxDown = 8.f;
            p.halfSize = 12.f * scale;
            break;
        case GameMode::Ufo:
            p.press = PressAction::ImpulseOnEdge;
            p.impulse = 7.f;
            p.accHold = p.accRelease = -0.13f;
            p.maxDown = 10.f;
            break;
        case GameMode::Ball:
            p.press = PressAction::FlipOnGround;
            p.accHold = p.accRelease = -0.15f;
            p.flipVy = -2.f;
            p.maxDown = 12.f;
            break;
        case GameMode::Wave:
            p.press = PressAction::SetVelocity;
            p.waveVy = mini ? 2.f : 1.f; // slope: |dy| per unit of dx
            p.halfSize = 6.f * scale;
            p.innerHalf = 3.f * scale;
            break;
        case GameMode::Spider:
            p.press = PressAction::TeleportOnGround;
            p.maxDown = 15.f;
            break;
        case GameMode::Swing:
            p.press = PressAction::FlipOnEdge;
            p.accHold = p.accRelease = -0.13f;
            p.flipVy = -1.f;
            p.maxDown = 9.f;
            break;
        default:
            break;
    }
    return p;
}

PhysicsModel::PhysicsModel() {
    reset();
}

void PhysicsModel::reset() {
    m_cal = PhysicsCalibration{};
    for (size_t i = 0; i < PhysicsCalibration::kSlots; ++i) {
        m_params[i] = defaults(static_cast<GameMode>(i / 2), (i % 2) == 1);
    }
    m_drift.fill(0.25f);
    m_driftN.fill(0);
    m_lastResimError = 0.f;
    m_accuracyN = 0;
}

void PhysicsModel::loadCalibration(PhysicsCalibration const& cal) {
    m_cal = cal;
    // GD's horizontal speeds are exact constants. A saved estimate far from
    // them can only come from the old tick counting (before v0.3.0, which
    // took 2-step moves for 1 step and learned ~1.2x the speed): drop it.
    for (int sc = 0; sc < 5; ++sc) {
        if (m_cal.vxN[static_cast<size_t>(sc)] == 0) continue;
        double const mean = m_cal.vxSum[static_cast<size_t>(sc)] / m_cal.vxN[static_cast<size_t>(sc)];
        double const nominal = unitsPerSecond(sc) / static_cast<double>(kTicksPerSecond);
        if (std::fabs(mean - nominal) > 0.1 * nominal) {
            m_cal.vxSum[static_cast<size_t>(sc)] = 0.0;
            m_cal.vxN[static_cast<size_t>(sc)] = 0;
            ++m_vxReplaced;
        }
    }
    for (size_t i = 0; i < PhysicsCalibration::kSlots; ++i) refit(i);
}

float PhysicsModel::cubeFloorY(bool mini) const {
    auto const& c = m_cal.modes[PhysicsCalibration::slot(GameMode::Cube, mini)];
    float def = mini ? 99.f : 105.f;
    return c.floorN ? std::min(def, c.floorY) : def;
}

float PhysicsModel::vxPerTick(int speedClass) const {
    int i = std::clamp(speedClass, 0, 4);
    if (m_cal.vxN[i] >= 10) return static_cast<float>(m_cal.vxSum[i] / m_cal.vxN[i]);
    return unitsPerSecond(i) / kTicksPerSecond;
}

bool PhysicsModel::calibrated(GameMode mode, bool mini) const {
    auto const& c = m_cal.modes[PhysicsCalibration::slot(mode, mini)];
    uint32_t pooled = c.accHold.n + c.accRelease.n;
    switch (mode) {
        case GameMode::Cube: return c.impulse.n >= 2 && pooled >= 20 && c.kN >= 20;
        case GameMode::Robot: return c.impulse.n >= 2 && c.accRelease.n >= 20 && c.accHold.n >= 10 && c.kN >= 20;
        case GameMode::Ship: return c.accHold.n >= 20 && c.accRelease.n >= 20 && c.kN >= 20;
        case GameMode::Ufo: return c.impulse.n >= 2 && pooled >= 20 && c.kN >= 20;
        case GameMode::Ball:
        case GameMode::Swing: return c.flips >= 2 && pooled >= 20 && c.kN >= 20;
        case GameMode::Spider: {
            // a spider rarely free-falls; its gravity can come from the cube's
            auto const& cube = m_cal.modes[PhysicsCalibration::slot(GameMode::Cube, mini)];
            return c.teleports >= 2 && ((pooled >= 20 && c.kN >= 20) || (cube.accHold.n + cube.accRelease.n >= 20 && cube.kN >= 20));
        }
        case GameMode::Wave: return c.waveVy.n >= 10;
        default: return false;
    }
}

void PhysicsModel::observeStep(Observation const& prev, Observation const& cur, bool held, bool heldBefore) {
    if (prev.mode != cur.mode || prev.mini != cur.mini) return;
    if (cur.tick - prev.tick != 1 || prev.dead || cur.dead) return;
    size_t const slot = PhysicsCalibration::slot(cur.mode, cur.mini);
    auto& c = m_cal.modes[slot];
    ModePhysics const& p = m_params[slot];

    // horizontal speed (all modes move right at the speed-portal speed)
    float dx = cur.x - prev.x;
    if (prev.speedClass() == cur.speedClass() && dx > 0.f && dx < 4.f) {
        int sc = cur.speedClass();
        double mean = m_cal.vxN[sc] ? m_cal.vxSum[sc] / m_cal.vxN[sc] : dx;
        if (m_cal.vxN[sc] < 10 || std::fabs(dx - mean) < 0.05 * mean) {
            m_cal.vxSum[sc] += dx;
            ++m_cal.vxN[sc];
            m_vxAltN[static_cast<size_t>(sc)] = 0;
        } else {
            // A long unbroken run of samples that agree with each other but not
            // with the estimate: the estimate is wrong (or the game changed).
            auto& an = m_vxAltN[static_cast<size_t>(sc)];
            auto& as = m_vxAltSum[static_cast<size_t>(sc)];
            if (an && std::fabs(dx - as / an) >= 0.05 * (as / an)) an = 0;
            if (an == 0) as = 0.0;
            as += dx;
            if (++an >= 40) {
                m_cal.vxSum[sc] = as;
                m_cal.vxN[sc] = an;
                an = 0;
                ++m_vxReplaced;
            }
        }
    }

    float const gp = prev.upsideDown ? -1.f : 1.f;
    float const gc = cur.upsideDown ? -1.f : 1.f;
    float const vp = prev.vy * gp;
    float const vc = cur.vy * gc;
    float const dyN = (cur.y - prev.y) * gp;
    bool const edge = held && !heldBefore;
    bool const bothAir = !prev.onGround && !cur.onGround;

    if (prev.upsideDown != cur.upsideDown) {
        // Gravity flipped during this step: a ball/swing flip or a spider teleport.
        if (p.press == PressAction::TeleportOnGround && std::fabs(cur.y - prev.y) > 20.f) {
            ++c.teleports;
        } else if (p.press == PressAction::FlipOnGround || p.press == PressAction::FlipOnEdge) {
            if (offerSample(c.flipVy, vc, p.flipVy, 0.25, 0.3)) ++c.flips;
        }
        refit(slot);
        return;
    }

    if (p.press == PressAction::SetVelocity) {
        // wave: straight diagonal lines; slope = |dy| / dx
        if (bothAir && dx > 0.5f && std::fabs(dyN) > 0.05f) {
            double slope = std::fabs(dyN) / dx;
            offerSample(c.waveVy, slope, p.waveVy, 0.05, 0.02);
        }
        refit(slot);
        return;
    }

    // Press events
    if (p.press == PressAction::JumpOnGround && prev.onGround && !cur.onGround && held && vc > 1.f) {
        offerSample(c.impulse, vc, p.impulse, 0.15, 0.2);
        m_robotSinceTakeoff = 0;
        refit(slot);
        return;
    }
    if (cur.mode == GameMode::Robot && bothAir) ++m_robotSinceTakeoff;
    if (p.press == PressAction::ImpulseOnEdge && edge && vc - vp > 1.f) {
        offerSample(c.impulse, vc, p.impulse, 0.15, 0.2);
        refit(slot);
        return;
    }

    if (bothAir) {
        float d = vc - vp;
        // velocity caps: no more acceleration although airborne
        if (std::fabs(d) < 0.005f && std::fabs(vc) > 1.f) {
            if (vc > 0.f) {
                c.maxUp = c.capUpN ? std::max(c.maxUp, vc) : vc;
                ++c.capUpN;
            } else {
                c.maxDown = c.capDownN ? std::max(c.maxDown, -vc) : -vc;
                ++c.capDownN;
            }
        } else if (std::fabs(d) < 1.f) {
            if (cur.mode == GameMode::Robot) {
                // robot: holding only boosts briefly after take-off; keep the two apart
                if (held && d > p.accRelease + 0.03f) {
                    offerSample(c.accHold, d, p.accHold, 0.2, 0.02);
                    if (m_robotSinceTakeoff < 1000) c.boostMax = std::max<uint32_t>(c.boostMax, static_cast<uint32_t>(m_robotSinceTakeoff));
                } else if (!held) {
                    offerSample(c.accRelease, d, p.accRelease, 0.1, 0.02);
                }
            } else if (pooledGravity(cur.mode)) {
                // the same gravity whatever the button does: pool both
                RunningStat& s = c.accRelease;
                double mean = (c.accHold.sum + c.accRelease.sum) / std::max<uint32_t>(1, c.accHold.n + c.accRelease.n);
                uint32_t changes = s.regimeChanges;
                offerAround(s, d, mean, c.accHold.n + c.accRelease.n >= 20, std::max(0.02, 0.1 * std::fabs(mean)));
                if (s.regimeChanges != changes) c.accHold = {}; // the pooled estimate restarts as a whole
            } else {
                RunningStat& s = held ? c.accHold : c.accRelease;
                offerSample(s, d, held ? p.accHold : p.accRelease, 0.1, 0.02);
            }
        }
        if (std::fabs(vc) > 0.5f) {
            float ratio = dyN / vc;
            double k = c.kVV > 0 ? c.kYV / c.kVV : p.yScale;
            if (ratio > 0.05f && ratio < 1.f && (c.kN < 20 || std::fabs(ratio - k) < 0.1 * k)) {
                c.kYV += static_cast<double>(dyN) * vc;
                c.kVV += static_cast<double>(vc) * vc;
                ++c.kN;
            }
        }
    }

    if (cur.mode == GameMode::Cube && !cur.upsideDown && cur.onGround && prev.onGround && std::fabs(cur.vy) < 0.01f) {
        if (c.floorN == 0 || cur.y < c.floorY) c.floorY = cur.y;
        ++c.floorN;
    }
    refit(slot);
}

void PhysicsModel::refit(size_t slot) {
    auto const mode = static_cast<GameMode>(slot / 2);
    bool const mini = (slot % 2) == 1;
    auto const& c = m_cal.modes[slot];
    ModePhysics p = defaults(mode, mini);
    if (pooledGravity(mode)) {
        uint32_t n = c.accHold.n + c.accRelease.n;
        auto const& cube = m_cal.modes[PhysicsCalibration::slot(GameMode::Cube, mini)];
        uint32_t cn = cube.accHold.n + cube.accRelease.n;
        if (n >= 10) p.accHold = p.accRelease = static_cast<float>((c.accHold.sum + c.accRelease.sum) / n);
        else if (mode == GameMode::Spider && cn >= 10) {
            p.accHold = p.accRelease = static_cast<float>((cube.accHold.sum + cube.accRelease.sum) / cn);
            if (cube.kN >= 10 && cube.kVV > 0) p.yScale = static_cast<float>(cube.kYV / cube.kVV);
        }
    } else {
        if (c.accHold.n >= 10) p.accHold = static_cast<float>(c.accHold.mean(p.accHold));
        if (c.accRelease.n >= 10) p.accRelease = static_cast<float>(c.accRelease.mean(p.accRelease));
    }
    if (c.kN >= 10 && c.kVV > 0) p.yScale = static_cast<float>(c.kYV / c.kVV);
    if (c.impulse.n >= 1) p.impulse = static_cast<float>(c.impulse.mean(p.impulse));
    if (c.flipVy.n >= 1) p.flipVy = static_cast<float>(c.flipVy.mean(p.flipVy));
    if (c.waveVy.n >= 5) p.waveVy = static_cast<float>(c.waveVy.mean(p.waveVy));
    if (c.capUpN >= 3) p.maxUp = c.maxUp;
    if (c.capDownN >= 3) p.maxDown = c.maxDown;
    if (mode == GameMode::Robot && c.boostMax > 0) p.boostSteps = static_cast<int>(c.boostMax);
    m_params[slot] = p;
}

void PhysicsModel::recordAccuracy(GameMode mode, float error, int steps, bool mini) {
    if (steps <= 0) return;
    auto i = PhysicsCalibration::slot(mode, mini);
    float perTick = error / static_cast<float>(steps);
    // slow average: one odd window cannot swing the trust level
    m_drift[i] = m_driftN[i] == 0 ? perTick : m_drift[i] * 0.9f + perTick * 0.1f;
    ++m_driftN[i];
    m_lastResimError = error;
    ++m_accuracyN;
}

float PhysicsModel::driftPerTick(GameMode mode, bool mini) const {
    return m_drift[PhysicsCalibration::slot(mode, mini)];
}

std::string PhysicsModel::describe(GameMode mode) const {
    auto const& p = params(mode, false);
    auto const& c = m_cal.modes[PhysicsCalibration::slot(mode, false)];
    char buf[240];
    char const* tag = calibrated(mode, false) ? "" : " [calibrating]";
    switch (p.press) {
        case PressAction::SetVelocity:
            std::snprintf(buf, sizeof(buf), "%s slope=%.3f (n %u)%s", gameModeName(mode), p.waveVy, c.waveVy.n, tag);
            break;
        case PressAction::None:
            std::snprintf(buf, sizeof(buf), "%s up=%.3f down=%.3f caps +%.1f/-%.1f k=%.4f (n %u/%u)%s", gameModeName(mode), p.accHold,
                          p.accRelease, p.maxUp, p.maxDown, p.yScale, c.accHold.n, c.accRelease.n, tag);
            break;
        default:
            std::snprintf(buf, sizeof(buf), "%s g=%.4f hold=%.4f impulse=%.3f flip=%.2f k=%.4f cap=%.1f floor=%.1f (n %u/%u, %u flips, %u tp)%s",
                          gameModeName(mode), -p.accRelease, p.accHold, p.impulse, p.flipVy, p.yScale, p.maxDown, cubeFloorY(false),
                          c.accRelease.n + c.accHold.n, c.impulse.n, c.flips, c.teleports, tag);
            break;
    }
    return buf;
}

} // namespace aigd
