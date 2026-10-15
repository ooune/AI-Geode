#include <aigd/Proximity.hpp>

#include <algorithm>
#include <cmath>

namespace aigd {

namespace {
// Distance between two axis-aligned rectangles; negative = depth of the overlap.
float rectDistance(float al, float ab, float ar, float at, float bl, float bb, float br, float bt) {
    float const gx = std::max(bl - ar, al - br);
    float const gy = std::max(bb - at, ab - bt);
    if (gx < 0.f && gy < 0.f) return std::max(gx, gy); // overlap: the shallower axis
    return std::hypot(std::max(gx, 0.f), std::max(gy, 0.f));
}
} // namespace

float obstacleClearance(float x, float y, float h, float in, Obstacle const& o, float cap) {
    switch (o.kind) {
        case Obstacle::Kind::Hazard: {
            float const d = rectDistance(x - h, y - h, x + h, y + h, o.l, o.b, o.r, o.t);
            return std::min(d, cap);
        }
        case Obstacle::Kind::Solid: {
            // The left face: the side the player runs into. It can only kill
            // while the inner hitbox is level with it (above it the player
            // lands on the block, below it slides along the underside), and
            // not once the player is past it.
            if (x - in > o.l) return cap;
            if (y + in <= o.b || y - in >= o.t) return cap;
            return std::min(o.l - (x + in), cap);
        }
        default: return cap;
    }
}

ClearanceResult hitboxClearance(float x, float y, float h, float in, std::span<Obstacle const> obstacles, float cap) {
    ClearanceResult r{cap, -1};
    for (size_t i = 0; i < obstacles.size(); ++i) {
        Obstacle const& o = obstacles[i];
        if (o.l > x + h + cap) break;
        if (o.r < x - h - cap) continue;
        float const c = obstacleClearance(x, y, h, in, o, cap);
        if (c < r.clearance) {
            r.clearance = c;
            r.obstacle = static_cast<int>(i);
        }
    }
    return r;
}

float riskOf(float clearance, ProximityConfig const& cfg) {
    float const span = std::max(0.01f, cfg.safe - cfg.danger);
    float const t = std::clamp((cfg.safe - clearance) / span, 0.f, 1.f);
    // smoothstep: no visible jumps between neighbouring segments
    return t * t * (3.f - 2.f * t);
}

RiskColor riskColor(float risk) {
    risk = std::clamp(risk, 0.f, 1.f);
    RiskColor const green{0.25f, 1.f, 0.35f}, orange{1.f, 0.6f, 0.1f}, red{1.f, 0.15f, 0.15f};
    auto mix = [](RiskColor a, RiskColor b, float t) { return RiskColor{a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; };
    return risk < 0.5f ? mix(green, orange, risk * 2.f) : mix(orange, red, (risk - 0.5f) * 2.f);
}

} // namespace aigd
