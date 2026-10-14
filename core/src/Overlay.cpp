#include <aigd/Overlay.hpp>

#include <algorithm>
#include <cctype>
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
    hasOrb = false;
    orbSelected = false;
    orbLabel.clear();
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

void addOrbLayer(Predictor const& predictor, OverlayGeometry& out, int stride) {
    auto const& O = predictor.lastAdvice().orb;
    if (!O.present) return;
    out.hasOrb = true;
    out.boxes.push_back({O.l, O.b, O.r, O.t, OverlayGeometry::Style::OrbBox});
    PredictedPath const& executed = predictor.executedPath();
    bool const selected = predictor.hasExecuted() && executed.activates(O.id);
    out.orbSelected = selected;
    PredictedPath const& on = selected ? executed : predictor.orbActivatedPath();
    PredictedPath const& off = selected ? predictor.orbSkippedPath() : executed;
    // the outcome that is not selected, in its own style
    if (selected) addPath(off, OverlayGeometry::Style::OrbSkipped, stride, out);
    else if (on.points.size() >= 2) addPath(on, OverlayGeometry::Style::OrbActivated, stride, out);
    if (on.activates(O.id)) {
        int const i = on.orbActivations > 1 && on.orbId[1] == O.id ? 1 : 0;
        out.dots.push_back({on.orbX[i], on.orbY[i], OverlayGeometry::Style::OrbPoint});
        if (!selected) {
            if (on.collisionStep >= 0) out.dots.push_back({on.collisionX, on.collisionY, OverlayGeometry::Style::Collision});
            else if (on.landed) out.dots.push_back({on.landX, on.landY, OverlayGeometry::Style::Landing});
        }
    }
    auto outcome = [](Certainty c, int32_t ttc) {
        char buf[48];
        if (c == Certainty::Collision) std::snprintf(buf, sizeof(buf), "collision in %d", ttc + 1);
        else std::snprintf(buf, sizeof(buf), "%s", c == Certainty::Safe ? "safe" : "uncertain");
        return std::string(buf);
    };
    std::string name = orbTypeName(O.type);
    for (auto& ch : name) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    char buf[220];
    if (!O.simulated) std::snprintf(buf, sizeof(buf), "%s: effect not simulated - %s", name.c_str(), selected ? "ACTIVATING" : "not activating");
    else
        std::snprintf(buf, sizeof(buf), "%s: activate -> %s%s | skip -> %s | selected: %s", name.c_str(),
                      O.candidates ? outcome(O.activate, O.activateTTC).c_str() : "not reachable",
                      O.known ? "" : " (not calibrated)", outcome(O.skip, O.skipTTC).c_str(), selected ? "ACTIVATE" : "SKIP");
    out.orbLabel = buf;
    out.orbLabelX = 0.5f * (O.l + O.r);
    out.orbLabelY = O.t + 30.f;
}

bool buildDecisionOverlay(Predictor const& predictor, SafetyRecord const& decision, int32_t nowTick, bool showAlternative,
                          std::span<Obstacle const> obstacles, OverlayGeometry& out, LivePlan const* plan, bool showOrbs) {
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
        if (showOrbs) addOrbLayer(predictor, out);
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
    if (showOrbs) addOrbLayer(predictor, out);
    return true;
}

// ---------------------------------------------------------------------------
// Level understanding

void UnderstandingOverlay::clear() {
    boxes.clear();
    segments.clear();
    dots.clear();
    warnings.clear();
}

void buildLevelOverview(LevelMap const& map, LevelAnalysis const& analysis, std::vector<UnderstandingOverlay::Mark>& out, size_t maxMarks) {
    using S = UnderstandingOverlay::Style;
    out.clear();
    float const len = std::max(1.f, map.length());
    // objects: merged into runs so that a long level stays a few hundred marks
    float runL = -1.f, runR = -1.f;
    S runStyle = S::Hazard;
    auto flush = [&]() {
        if (runL >= 0.f && out.size() < maxMarks) out.push_back({runL / len, runR / len, runStyle, 0});
        runL = runR = -1.f;
    };
    float const join = len / 600.f; // ~600 pixels of resolution
    for (auto const& o : map.objects()) {
        S st;
        switch (o.kind) {
            case MapKind::Hazard: st = S::Hazard; break;
            case MapKind::Solid: st = S::Solid; break;
            default: continue; // portals go to their own row
        }
        if (o.dynamic) st = S::Dynamic;
        if (runL >= 0.f && st == runStyle && o.l <= runR + join) {
            runR = std::max(runR, o.r);
            continue;
        }
        flush();
        runL = o.l;
        runR = o.r;
        runStyle = st;
    }
    flush();
    for (auto const& g : analysis.groups) {
        if (out.size() >= maxMarks) break;
        bool const tight = g.landingSpace < 2.f * kBlock;
        out.push_back({g.startX / len, g.endX / len, tight ? S::TightGroup : S::Group, 1});
    }
    for (auto const& c : analysis.changes) {
        if (out.size() >= maxMarks) break;
        out.push_back({c.x / len, c.x / len, S::Portal, 2});
    }
}

void buildUnderstandingOverlay(LevelMap const* map, LevelAnalysis const* analysis, StrategicPlan const* plan, float playerX, float playerY,
                               float x0, float x1, UnderstandingLayers const& layers, UnderstandingOverlay& out, int stride) {
    using S = UnderstandingOverlay::Style;
    out.clear();
    if (map && map->length() > 1.f) out.playerFraction = std::clamp(playerX / map->length(), 0.f, 1.f);
    out.dots.push_back({playerX, playerY, S::Player});
    if (map && (layers.objects || layers.uncertain)) {
        thread_local std::vector<uint32_t> hits;
        map->query(x0, x1, hits);
        int notSimulated = 0, moving = 0, approximate = 0;
        for (uint32_t i : hits) {
            MapObject const& o = map->at(i);
            S st;
            bool uncertain = false;
            switch (o.kind) {
                case MapKind::Hazard: st = S::Hazard; break;
                case MapKind::Solid: st = S::Solid; break;
                case MapKind::SpeedPortal:
                case MapKind::GravityPortal: st = S::Portal; break;
                default: st = S::NotSimulated; uncertain = true; ++notSimulated; break;
            }
            if (o.dynamic) {
                st = S::Dynamic;
                uncertain = true;
                ++moving;
            } else if (o.approximate) {
                st = S::Approximate;
                uncertain = true;
                ++approximate;
            }
            if (uncertain ? layers.uncertain : layers.objects) out.boxes.push_back({o.l, o.b, o.r, o.t, st, !uncertain});
        }
        if (layers.uncertain) {
            char buf[160];
            if (notSimulated) {
                std::snprintf(buf, sizeof(buf), "%d object(s) ahead are not simulated (orbs, pads, mode portals...)", notSimulated);
                out.warnings.push_back(buf);
            }
            if (moving) {
                std::snprintf(buf, sizeof(buf), "%d object(s) ahead move or are toggled by triggers (refreshed while near)", moving);
                out.warnings.push_back(buf);
            }
            if (approximate) {
                std::snprintf(buf, sizeof(buf), "%d rotated hitbox(es) ahead are approximated by their bounding box", approximate);
                out.warnings.push_back(buf);
            }
        }
    }
    if (analysis && layers.groups) {
        for (size_t i = analysis->groupAt(x0); i < analysis->groups.size(); ++i) {
            ObstacleGroup const& g = analysis->groups[i];
            if (g.startX > x1) break;
            float top = g.surface + 2.f * kBlock;
            bool const tight = g.landingSpace < 2.f * kBlock;
            out.boxes.push_back({g.startX, g.surface, g.endX, top, tight ? S::TightGroup : S::Group, false});
            if (layers.landing && g.landingSpace > 0.f)
                out.segments.push_back({g.endX, g.surface, g.endX + std::min(g.landingSpace, 8.f * kBlock), g.surface, S::LandingZone});
        }
    }
    if (plan && plan->valid) {
        if (layers.groups) {
            for (int g = 0; g < plan->groups; ++g)
                out.boxes.push_back({plan->groupStart[g], plan->madeY - 20.f, plan->groupEnd[g], plan->madeY + 70.f, S::Sequence, false});
        }
        if (layers.plan) {
            auto const& pts = plan->path.points;
            size_t const st = static_cast<size_t>(std::max(1, stride));
            if (pts.size() >= 2) {
                size_t prev = 0;
                for (size_t i = st; i < pts.size(); i += st) {
                    out.segments.push_back({pts[prev].x, pts[prev].y, pts[i].x, pts[i].y, S::PlannedPath});
                    prev = i;
                }
                if (prev != pts.size() - 1) out.segments.push_back({pts[prev].x, pts[prev].y, pts.back().x, pts.back().y, S::PlannedPath});
            }
            for (size_t i = 0, k = 0; i < plan->path.holds.size() && i < pts.size(); ++i) {
                bool const edge = plan->path.holds[i] && (i == 0 || !plan->path.holds[i - 1]);
                if (edge && k++ < 6) out.dots.push_back({pts[i].x, pts[i].y, S::Press});
            }
        }
        if (layers.landing) {
            for (float lx : plan->landingX) out.dots.push_back({lx, plan->madeY, S::Target});
        }
        if (layers.alternatives) {
            // where the other evaluated first presses would start (not their paths)
            float const vx = unitsPerSecond(plan->speedClass) / static_cast<float>(kTicksPerSecond);
            for (auto const& a : plan->alternatives)
                if (a.plan.pressAt >= 0)
                    out.dots.push_back({plan->madeX + static_cast<float>(a.plan.pressAt + 1) * vx, plan->madeY, S::AlternativePath});
        }
    }
}
} // namespace aigd
