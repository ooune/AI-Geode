#include <aigd/Overlay.hpp>

#include <algorithm>
#include <cstdio>

namespace aigd {

void OverlayGeometry::clear() {
    segments.clear();
    dots.clear();
    boxes.clear();
    hasCollision = false;
    stepsToCollision = -1;
    outcome = Certainty::Safe;
    label.clear();
}

namespace {
void addPath(PredictedPath const& p, OverlayGeometry::Style style, int stride, OverlayGeometry& out) {
    auto const& pts = p.points;
    if (pts.size() < 2) return;
    size_t const step = static_cast<size_t>(std::max(1, stride));
    size_t prev = 0;
    for (size_t i = step; i < pts.size(); i += step) {
        out.segments.push_back({pts[prev].x, pts[prev].y, pts[i].x, pts[i].y, style});
        prev = i;
    }
    if (prev != pts.size() - 1) out.segments.push_back({pts[prev].x, pts[prev].y, pts.back().x, pts.back().y, style});
}
} // namespace

void buildOverlay(PredictedPath const& main, PredictedPath const* alternative, std::span<Obstacle const> obstacles,
                  OverlayGeometry& out, int stride, size_t maxHazards) {
    out.clear();
    out.outcome = main.outcome;
    auto style = main.outcome == Certainty::Safe ? OverlayGeometry::Style::PathSafe
               : main.outcome == Certainty::Uncertain ? OverlayGeometry::Style::PathUncertain
                                                       : OverlayGeometry::Style::PathDanger;
    if (alternative) addPath(*alternative, OverlayGeometry::Style::Alternative, stride, out);
    addPath(main, style, stride, out);

    if (main.landed) out.dots.push_back({main.landX, main.landY, OverlayGeometry::Style::Landing});
    if (main.collisionStep >= 0) {
        out.hasCollision = true;
        out.collisionX = main.collisionX;
        out.collisionY = main.collisionY;
        out.stepsToCollision = main.collisionStep + 1;
        out.dots.push_back({main.collisionX, main.collisionY, OverlayGeometry::Style::Collision});
    }

    // Hazards the path passes over or near.
    if (!main.points.empty()) {
        float x0 = main.points.front().x - 30.f;
        float x1 = main.points.back().x + 30.f;
        auto const& h = main.hit;
        bool hasHit = main.collisionStep >= 0;
        for (size_t i = 0; i < obstacles.size() && out.boxes.size() < maxHazards; ++i) {
            auto const& o = obstacles[i];
            if (o.kind != Obstacle::Kind::Hazard || o.r < x0 || o.l > x1) continue;
            bool hit = hasHit && o.l == h.l && o.r == h.r && o.b == h.b && o.t == h.t;
            if (!hit) out.boxes.push_back({o.l, o.b, o.r, o.t, OverlayGeometry::Style::Hazard});
        }
        // the obstacle the path runs into (hazard or block), always shown
        if (hasHit) out.boxes.push_back({h.l, h.b, h.r, h.t, OverlayGeometry::Style::HazardHit});
    }

    char buf[96];
    if (main.outcome == Certainty::Collision) {
        std::snprintf(buf, sizeof(buf), "collision in %d steps (%.2f s)", out.stepsToCollision, out.stepsToCollision / 240.f);
    } else if (main.outcome == Certainty::Uncertain) {
        if (main.collisionStep >= 0) std::snprintf(buf, sizeof(buf), "possible hit in %d steps", out.stepsToCollision);
        else std::snprintf(buf, sizeof(buf), "uncertain after %d steps", main.uncertainFrom + 1);
    } else {
        std::snprintf(buf, sizeof(buf), "safe for %d steps", main.steps);
    }
    out.label = buf;
}

} // namespace aigd
