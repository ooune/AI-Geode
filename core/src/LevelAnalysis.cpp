#include <aigd/LevelAnalysis.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>

namespace aigd {

namespace {
constexpr float kPlayerSpeeds[] = {0.7f, 0.9f, 1.1f, 1.3f, 1.6f};

bool isPortal(MapKind k) {
    return k == MapKind::SpeedPortal || k == MapKind::GravityPortal || k == MapKind::ModePortal || k == MapKind::SizePortal ||
           k == MapKind::OtherPortal;
}
bool notSimulated(MapKind k) {
    return k == MapKind::Platform || k == MapKind::Slope || k == MapKind::Orb || k == MapKind::Pad || k == MapKind::ModePortal ||
           k == MapKind::SizePortal || k == MapKind::OtherPortal;
}
} // namespace

size_t LevelAnalysis::groupAt(float x) const {
    auto it = std::lower_bound(groups.begin(), groups.end(), x, [](ObstacleGroup const& g, float v) { return g.endX < v; });
    return static_cast<size_t>(it - groups.begin());
}

std::string LevelAnalysis::summary() const {
    if (!valid) return "no level geometry available";
    char buf[512];
    int speed = 0, gravity = 0, mode = 0, size = 0;
    for (auto const& c : changes) {
        speed += c.kind == MapKind::SpeedPortal;
        gravity += c.kind == MapKind::GravityPortal;
        mode += c.kind == MapKind::ModePortal;
        size += c.kind == MapKind::SizePortal;
    }
    int groupSections = 0, flying = 0, narrow = 0;
    for (auto const& s : sections) {
        groupSections += s.analysedGroups ? 1 : 0;
        flying += s.analysedGroups ? 0 : 1;
        narrow += s.narrowPassages;
    }
    std::snprintf(buf, sizeof(buf),
                  "%zu objects (%zu hazards, %zu blocks, %zu portals, %zu not simulated, %zu moved by triggers, %zu rotated boxes); "
                  "x %.0f to %.0f; %zu sections (%d with obstacle groups, %d checked for vertical room only, %d narrow passages); "
                  "%zu obstacle groups (%zu with under 2 blocks of landing space; library: %d known, %d similar); "
                  "physics changes: %d speed, %d gravity, %d mode, %d size; %.2f ms",
                  objects, hazards, solids, portals, unmodelled, dynamicObjects, approximate, startX, finishX, sections.size(), groupSections,
                  flying, narrow, groups.size(), tight.size(), knownGroups, similarGroups, speed, gravity, mode, size, millis);
    return buf;
}

LevelAnalysis analyzeLevel(LevelMap const& map, AnalysisStart const& start, StrategyLibrary const* library, std::string const& level) {
    auto const t0 = std::chrono::steady_clock::now();
    LevelAnalysis A;
    A.valid = !map.empty();
    A.startX = start.startX;
    A.finishX = map.length();
    A.objects = map.size();
    A.hazards = map.count(MapKind::Hazard);
    A.solids = map.count(MapKind::Solid);
    for (auto const& o : map.objects()) {
        A.portals += isPortal(o.kind) ? 1 : 0;
        A.unmodelled += notSimulated(o.kind) ? 1 : 0;
        A.approximate += o.approximate ? 1 : 0;
    }
    A.dynamicObjects = map.dynamicObjects().size();
    if (!A.valid) return A;

    // ---- physics changes and sections ----
    for (auto const& o : map.objects()) {
        if (o.kind == MapKind::SpeedPortal || o.kind == MapKind::GravityPortal || o.kind == MapKind::ModePortal || o.kind == MapKind::SizePortal)
            A.changes.push_back({0.5f * (o.l + o.r), o.kind, o.value});
    }
    std::sort(A.changes.begin(), A.changes.end(), [](PhysicsChange const& a, PhysicsChange const& b) { return a.x < b.x; });
    MapSection cur;
    cur.startX = start.startX;
    cur.mode = static_cast<int>(start.mode);
    cur.speedClass = start.speedClass;
    cur.gravity = start.upsideDown ? 1 : 0;
    cur.mini = start.mini;
    for (auto const& c : A.changes) {
        if (c.x <= start.startX) continue;
        MapSection next = cur;
        switch (c.kind) {
            case MapKind::SpeedPortal: next.speedClass = c.value; break;
            case MapKind::GravityPortal: next.gravity = c.value == 2 ? 1 - cur.gravity : c.value; break;
            case MapKind::ModePortal: next.mode = c.value; break;
            case MapKind::SizePortal: next.mini = c.value != 0; break;
            default: break;
        }
        if (next.speedClass == cur.speedClass && next.gravity == cur.gravity && next.mode == cur.mode && next.mini == cur.mini) continue;
        cur.endX = c.x;
        A.sections.push_back(cur);
        next.startX = c.x;
        cur = next;
    }
    cur.endX = std::max(cur.startX, A.finishX);
    A.sections.push_back(cur);

    // ---- obstacle groups (cube / robot, normal gravity) and vertical room (the rest) ----
    std::vector<Obstacle> obs;
    std::vector<PhysicsEvent> ev;
    std::vector<uint32_t> hits;
    float const floor = map.floorSurface();
    for (size_t si = 0; si < A.sections.size(); ++si) {
        MapSection& sec = A.sections[si];
        sec.firstGroup = static_cast<int>(A.groups.size());
        GameMode const mode = static_cast<GameMode>(sec.mode);
        float const h = situationHalfSize(sec.mini);
        if ((mode == GameMode::Cube || mode == GameMode::Robot) && sec.gravity == 0) {
            sec.analysedGroups = true;
            float const vx = unitsPerSecond(sec.speedClass) / static_cast<float>(kTicksPerSecond);
            float constexpr kWindow = 600.f;
            float surface = floor;
            float x = sec.startX;
            float prevEnd = sec.startX;
            int guard = 0;
            while (x < sec.endX && guard++ < 100000) {
                map.collect(x - 60.f, x + kWindow, obs, ev, true);
                Observation o;
                o.x = x;
                o.y = surface + h;
                o.mode = mode;
                o.onGround = true;
                o.mini = sec.mini;
                o.playerSpeed = kPlayerSpeeds[std::clamp(sec.speedClass, 0, 4)];
                o.obstacles = obs;
                Situation s = describeSituation(o, (kWindow - 2.f * kBlock) / vx);
                if (!s.valid || s.startX >= sec.endX) {
                    x += kWindow - 4.f * kBlock;
                    continue;
                }
                ObstacleGroup g;
                g.startX = s.startX;
                g.endX = std::min(s.endX, std::max(s.startX, sec.endX));
                g.surface = surface;
                g.sit = s;
                for (int i = 0; i < s.count; ++i) {
                    switch (s.el[i].kind) {
                        case SituationElement::Hazard: ++g.hazards; break;
                        case SituationElement::Solid: ++g.solids; break;
                        default: ++g.above; break;
                    }
                }
                g.gapBefore = s.startX - prevEnd;
                g.landingSpace = s.landingSpace;
                g.unmodelled = s.unmodelled;
                map.query(g.startX - 1.f, g.endX + 1.f, hits);
                for (uint32_t i : hits) {
                    MapObject const& m = map.at(i);
                    g.dynamic = g.dynamic || m.dynamic;
                    g.approximate = g.approximate || m.approximate;
                    if ((m.kind == MapKind::Hazard || m.kind == MapKind::Solid) && m.b >= surface + 2.f * h - 2.f && m.b < surface + 8.f * kBlock)
                        g.headroom = std::min(g.headroom, m.b - surface);
                }
                g.section = static_cast<int>(si);
                g.difficulty = 0.5f * static_cast<float>(s.count) + (g.landingSpace < kBlock ? 2.f : g.landingSpace < 2.f * kBlock ? 1.f : 0.f) +
                               (g.headroom < 2.5f * kBlock ? 1.5f : 0.f) + (g.gapBefore < 2.f * kBlock ? 1.f : 0.f) + (g.unmodelled ? 1.f : 0.f) +
                               (g.dynamic ? 1.f : 0.f);
                if (library) {
                    StrategyChoice c = library->choose(s, level);
                    if (c.usable) g.strategy = c.match;
                }
                A.groups.push_back(g);
                prevEnd = g.endX;
                // The walking surface behind the group: the highest block top
                // a jump from the current surface could reach, else the floor.
                float const nx = g.endX + 1.f;
                float best = floor;
                map.query(nx, nx + 2.f * h, hits);
                for (uint32_t i : hits) {
                    MapObject const& m = map.at(i);
                    if (m.kind != MapKind::Solid) continue;
                    if (m.t <= surface + 2.5f * kBlock && m.t > best) best = m.t;
                }
                surface = best;
                float const nextX = g.endX + 1.f - h;
                x = nextX > x ? nextX : x + kBlock;
            }
        } else {
            // Corridor modes and flipped gravity: the largest vertical gap
            // between obstacles in every 15-unit column.
            bool narrowRun = false;
            for (float x = sec.startX; x < sec.endX; x += 15.f) {
                map.query(x, x + 15.f, hits);
                std::pair<float, float> gaps[64];
                int n = 0;
                for (uint32_t i : hits) {
                    MapObject const& m = map.at(i);
                    if (m.kind != MapKind::Hazard && m.kind != MapKind::Solid) continue;
                    if (n < 64) {
                        gaps[n] = {m.b, m.t};
                        ++n;
                    }
                }
                std::sort(gaps, gaps + n);
                float top = floor;
                float bestGap = 0.f;
                float const limit = floor + 12.f * kBlock;
                for (int i = 0; i < n; ++i) {
                    if (gaps[i].second <= top) continue;
                    bestGap = std::max(bestGap, std::min(gaps[i].first, limit) - top);
                    top = std::max(top, gaps[i].second);
                }
                if (n == 0) continue; // nothing in this column
                bestGap = std::max(bestGap, limit - top);
                sec.minVerticalRoom = std::min(sec.minVerticalRoom, bestGap);
                bool const narrow = bestGap < 2.5f * kBlock;
                if (narrow && !narrowRun) ++sec.narrowPassages;
                narrowRun = narrow;
            }
        }
        sec.groups = static_cast<int>(A.groups.size()) - sec.firstGroup;
    }

    for (size_t i = 0; i < A.groups.size(); ++i) {
        ObstacleGroup const& g = A.groups[i];
        if (g.landingSpace < 2.f * kBlock) A.tight.push_back(static_cast<int>(i));
        if (g.strategy == StrategyChoice::Match::Exact) ++A.knownGroups;
        else if (g.strategy == StrategyChoice::Match::Similar) ++A.similarGroups;
    }
    std::vector<int> order(A.groups.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return A.groups[static_cast<size_t>(a)].difficulty > A.groups[static_cast<size_t>(b)].difficulty; });
    order.resize(std::min<size_t>(order.size(), 10));
    A.difficult = std::move(order);
    A.millis = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return A;
}

} // namespace aigd
