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

bool buildDecisionOverlay(Predictor const& predictor, SafetyRecord const& decision, int32_t nowTick, bool showAlternative,
                          std::span<Obstacle const> obstacles, OverlayGeometry& out, LivePlan const* plan) {
    out.clear();
    if (plan && plan->active && decision.proposedSource == DecisionSource::Plan && decision.tick == nowTick) {
        // Following the live plan: draw what is left of it, from now on.
        int const from = std::max(0, plan->indexOf(nowTick));
        auto const style = plan->outcome == Certainty::Safe ? OverlayGeometry::Style::PathSafe
                         : plan->outcome == Certainty::Uncertain ? OverlayGeometry::Style::PathUncertain
                                                                 : OverlayGeometry::Style::PathDanger;
        size_t prev = static_cast<size_t>(from);
        for (size_t i = prev + 4; i < plan->steps.size(); i += 4) {
            out.segments.push_back({plan->steps[prev].x, plan->steps[prev].y, plan->steps[i].x, plan->steps[i].y, style});
            prev = i;
        }
        if (plan->landed) out.dots.push_back({plan->landX, 0.f, OverlayGeometry::Style::Landing});
        out.outcome = plan->outcome;
        out.verdict = decision.verdict;
        out.selectedPress = decision.executed != 0;
        out.predictionAge = nowTick - plan->createdTick;
        bool const steered = decision.mode == GameMode::Ship || decision.mode == GameMode::Wave || decision.mode == GameMode::Ufo ||
                             decision.mode == GameMode::Swing;
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s - %s - following plan (made %d step(s) ago, still matching)",
                      steered ? (out.selectedPress ? "HOLD" : "RELEASE") : (out.selectedPress ? "PRESS" : "RELEASE"),
                      plan->outcome == Certainty::Safe ? "safe" : "risky", out.predictionAge);
        out.label = buf;
        return true;
    }
    auto const& adv = predictor.lastAdvice();
    out.verdict = decision.verdict;
    if (!adv.valid || !predictor.hasExecuted()) {
        out.label = "no prediction (nothing ahead)";
        return false;
    }
    PredictedPath const& main = predictor.executedPath();
    buildOverlay(main, showAlternative ? &predictor.otherPath() : nullptr, obstacles, out);
    out.verdict = decision.verdict;
    out.selectedPress = predictor.executedPress();
    out.predictionAge = nowTick - adv.computedTick;
    out.stale = out.predictionAge > 0 || adv.reused;
    // What would happen without touching the button, when that is not what is
    // being executed (e.g. the safety layer just changed the input).
    if (predictor.keepPath().points.size() >= 2 && (decision.holdingBefore != predictor.executedPress())) {
        auto const& kp = predictor.keepPath().points;
        size_t prev = 0;
        for (size_t i = 4; i < kp.size(); i += 4) {
            out.segments.push_back({kp[prev].x, kp[prev].y, kp[i].x, kp[i].y, OverlayGeometry::Style::Keep});
            prev = i;
        }
    }
    if (out.stale) {
        for (auto& sgm : out.segments)
            if (sgm.style != OverlayGeometry::Style::Alternative && sgm.style != OverlayGeometry::Style::Keep) sgm.style = OverlayGeometry::Style::Stale;
    }

    char const* button = decision.mode == GameMode::Ship || decision.mode == GameMode::Wave || decision.mode == GameMode::Ufo ||
                                 decision.mode == GameMode::Swing
                             ? (out.selectedPress ? "HOLD" : "RELEASE")
                             : (out.selectedPress ? "PRESS" : "RELEASE");
    char const* safety = main.outcome == Certainty::Safe ? "safe" : main.outcome == Certainty::Uncertain ? "risky" : "UNSAFE";
    char buf[192];
    int n = std::snprintf(buf, sizeof(buf), "%s - %s", button, safety);
    if (out.hasCollision) n += std::snprintf(buf + n, sizeof(buf) - n, ", hit in %d", out.stepsToCollision);
    if (!adv.trusted) n += std::snprintf(buf + n, sizeof(buf) - n, " - prediction not trusted, not used");
    if (decision.verdict == SafetyVerdict::Overridden) n += std::snprintf(buf + n, sizeof(buf) - n, " - SAFETY OVERRIDE");
    if (decision.verdict == SafetyVerdict::NoSafeOption) n += std::snprintf(buf + n, sizeof(buf) - n, " - NO SAFE OPTION");
    if (out.stale) std::snprintf(buf + n, sizeof(buf) - n, " - prediction %d step(s) old", out.predictionAge);
    out.label = buf;
    return true;
}

} // namespace aigd
