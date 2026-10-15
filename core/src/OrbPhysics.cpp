#include <aigd/Physics.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace aigd {

namespace {
// Guesses before anything was observed. They are never trusted: with
// known = false every path through an activation stays uncertain.
OrbPhysics guess(OrbType t, ModePhysics const& m) {
    OrbPhysics o;
    switch (t) {
        case OrbType::Yellow: o.vy = m.impulse; break;
        case OrbType::Pink: o.vy = 0.7f * m.impulse; break;
        case OrbType::Red: o.vy = 1.3f * m.impulse; break;
        case OrbType::Blue: o.flips = true; o.vy = -0.4f * m.impulse; break;
        case OrbType::Green: o.flips = true; o.vy = m.impulse; break;
        case OrbType::Black: o.vy = -1.5f * m.impulse; break;
        case OrbType::Dash: o.dash = true; break;
        case OrbType::GravityDash: o.dash = true; o.flips = true; break;
        default: break;
    }
    return o;
}
} // namespace

OrbPhysics PhysicsModel::orb(OrbType type, GameMode mode, bool mini) const {
    OrbPhysics o = guess(type, params(mode, mini));
    if (!orbSimulated(type)) return o;
    auto const& c = m_cal.orbs[PhysicsCalibration::orbSlot(type, mode, mini)];
    if (c.n > 0) {
        o.known = true;
        o.flips = c.flips * 2 > c.n;
        if (!o.dash) o.vy = static_cast<float>(c.vy.mean(o.vy));
    }
    if (o.dash) {
        o.dashDx = static_cast<float>(c.dashDx.mean(1.0));
        o.dashSlope = static_cast<float>(c.dashSlope.mean(1.0));
        o.dashEndVy = static_cast<float>(c.dashEndVy.mean(0.0));
        // the movement and its end must both have been seen
        o.dashKnown = c.dashDx.n >= 3 && c.dashEndVy.n >= 1;
        o.known = o.known && o.dashKnown;
    }
    return o;
}

float PhysicsModel::observeOrb(OrbEvent const& e) {
    if (!orbSimulated(e.type) || e.kind != OrbEvent::Kind::Activated) return 0.f;
    OrbPhysics const before = orb(e.type, e.mode, e.mini);
    auto& c = m_cal.orbs[PhysicsCalibration::orbSlot(e.type, e.mode, e.mini)];
    bool const flipped = e.flippedAfter != e.flippedBefore;
    float const vyN = e.flippedAfter ? -e.vyAfter : e.vyAfter;
    ++c.n;
    if (flipped) ++c.flips;
    if (isDashOrb(e.type)) return 0.f;
    c.vy.add(vyN);
    return vyN - before.vy;
}

void PhysicsModel::observeDashStep(OrbType type, GameMode mode, bool mini, float dx, float dy, float rotation, int speedClass) {
    if (!isDashOrb(type)) return;
    auto& c = m_cal.orbs[PhysicsCalibration::orbSlot(type, mode, mini)];
    float const vx = vxPerTick(speedClass);
    if (vx <= 0.f || dx <= 0.f) return;
    c.dashDx.add(dx / vx);
    // the direction from the orb rotation (cocos rotates clockwise)
    float const dir = std::tan(-rotation * 3.14159265f / 180.f);
    if (std::fabs(dir) > 0.05f) c.dashSlope.add((dy / dx) / dir);
}

void PhysicsModel::observeDashEnd(OrbType type, GameMode mode, bool mini, float vyNormalised, bool released) {
    if (!isDashOrb(type)) return;
    auto& c = m_cal.orbs[PhysicsCalibration::orbSlot(type, mode, mini)];
    c.dashEndVy.add(vyNormalised);
    if (released) ++c.dashEndsReleased;
    else ++c.dashEndsOther;
}

int PhysicsModel::heldEntryActivates() const {
    uint32_t const yes = m_cal.heldEntryActivated, no = m_cal.heldEntryIgnored;
    if (yes + no < 2) return -1;
    if (yes >= 2 * no + 2) return 1;
    if (no >= 2 * yes + 2) return 0;
    return -1;
}

void PhysicsModel::observeHeldEntry(bool activated) {
    if (activated) ++m_cal.heldEntryActivated;
    else ++m_cal.heldEntryIgnored;
}

namespace {
OrbPhysics padGuess(PadType t, ModePhysics const& m) {
    OrbPhysics o;
    switch (t) {
        case PadType::Yellow: o.vy = 1.45f * m.impulse; break;
        case PadType::Pink: o.vy = 0.95f * m.impulse; break;
        case PadType::Red: o.vy = 1.8f * m.impulse; break;
        case PadType::Blue: o.flips = true; o.vy = -0.5f * m.impulse; break;
        default: break;
    }
    return o;
}
} // namespace

OrbPhysics PhysicsModel::pad(PadType type, GameMode mode, bool mini) const {
    OrbPhysics o = padGuess(type, params(mode, mini));
    if (!padSimulated(type)) return o;
    auto const& c = m_cal.pads[PhysicsCalibration::padSlot(type, mode, mini)];
    if (c.n > 0) {
        o.known = true;
        o.flips = c.flips * 2 > c.n;
        o.vy = static_cast<float>(c.vy.mean(o.vy));
    }
    return o;
}

float PhysicsModel::observePad(PadEvent const& e) {
    if (!padSimulated(e.type)) return 0.f;
    OrbPhysics const before = pad(e.type, e.mode, e.mini);
    auto& c = m_cal.pads[PhysicsCalibration::padSlot(e.type, e.mode, e.mini)];
    bool const flipped = e.flippedAfter != e.flippedBefore;
    float const vyN = e.flippedAfter ? -e.vyAfter : e.vyAfter;
    ++c.n;
    if (flipped) ++c.flips;
    c.vy.add(vyN);
    return vyN - before.vy;
}

std::string PhysicsModel::describePads() const {
    std::string s;
    char buf[160];
    for (int t = 0; t < static_cast<int>(PadType::Count); ++t) {
        for (int m = 0; m < static_cast<int>(GameMode::Count); ++m) {
            for (int mini = 0; mini < 2; ++mini) {
                auto const type = static_cast<PadType>(t);
                auto const mode = static_cast<GameMode>(m);
                auto const& c = m_cal.pads[PhysicsCalibration::padSlot(type, mode, mini != 0)];
                if (c.n == 0) continue;
                OrbPhysics const o = pad(type, mode, mini != 0);
                std::snprintf(buf, sizeof(buf), "%s%s/%s%s: vy %.2f%s (%u)", s.empty() ? "" : "; ", padTypeName(type), gameModeName(mode),
                              mini ? " mini" : "", o.vy, o.flips ? ", flips gravity" : "", c.n);
                s += buf;
            }
        }
    }
    return s.empty() ? "no pad activations seen yet" : s;
}

std::string PhysicsModel::describeOrbs() const {
    std::string s;
    char buf[200];
    for (int t = 0; t < static_cast<int>(OrbType::Count); ++t) {
        for (int m = 0; m < static_cast<int>(GameMode::Count); ++m) {
            auto const type = static_cast<OrbType>(t);
            auto const mode = static_cast<GameMode>(m);
            auto const& c = m_cal.orbs[PhysicsCalibration::orbSlot(type, mode, false)];
            if (c.n == 0) continue;
            OrbPhysics const o = orb(type, mode, false);
            if (o.dash)
                std::snprintf(buf, sizeof(buf), "%s%s/%s: dash x%.2f, slope x%.2f, end vy %.2f (%u activations, %u dash steps)", s.empty() ? "" : "; ",
                              orbTypeName(type), gameModeName(mode), o.dashDx, o.dashSlope, o.dashEndVy, c.n, c.dashDx.n);
            else
                std::snprintf(buf, sizeof(buf), "%s%s/%s: vy %.2f%s (%u)", s.empty() ? "" : "; ", orbTypeName(type), gameModeName(mode), o.vy,
                              o.flips ? ", flips gravity" : "", c.n);
            s += buf;
        }
    }
    int const held = heldEntryActivates();
    std::snprintf(buf, sizeof(buf), "%sholding into an orb: %s (%u activated, %u not)", s.empty() ? "" : "; ",
                  held < 0 ? "not known yet" : held ? "activates it" : "does not activate it", m_cal.heldEntryActivated, m_cal.heldEntryIgnored);
    return s + buf;
}

} // namespace aigd
