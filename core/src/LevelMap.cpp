#include <aigd/LevelMap.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace aigd {

char const* mapKindName(MapKind k) {
    switch (k) {
        case MapKind::Hazard: return "hazard";
        case MapKind::Solid: return "block";
        case MapKind::Platform: return "one-way platform";
        case MapKind::Slope: return "slope";
        case MapKind::Orb: return "orb";
        case MapKind::Pad: return "pad";
        case MapKind::SpeedPortal: return "speed portal";
        case MapKind::GravityPortal: return "gravity portal";
        case MapKind::ModePortal: return "mode portal";
        case MapKind::SizePortal: return "size portal";
        case MapKind::OtherPortal: return "other portal";
        default: return "?";
    }
}

void LevelMap::clear() {
    m_objs.clear();
    m_staticCount = 0;
    m_columns.clear();
    m_dynamic.clear();
    m_isDynamic.clear();
    for (auto& c : m_counts) c = 0;
    m_length = 0.f;
    m_version = 0;
    m_promoted = 0;
}

void LevelMap::build(std::vector<MapObject> objects, float levelLength, float floorSurface) {
    auto const t0 = std::chrono::steady_clock::now();
    clear();
    // static objects first, sorted by left edge; dynamic ones after them
    std::stable_partition(objects.begin(), objects.end(), [](MapObject const& o) { return !o.dynamic; });
    size_t const nStatic = static_cast<size_t>(
        std::find_if(objects.begin(), objects.end(), [](MapObject const& o) { return o.dynamic; }) - objects.begin());
    std::sort(objects.begin(), objects.begin() + static_cast<std::ptrdiff_t>(nStatic),
              [](MapObject const& a, MapObject const& b) { return a.l < b.l; });
    m_objs = std::move(objects);
    m_staticCount = nStatic;
    m_length = levelLength;
    m_floor = floorSurface;
    m_isDynamic.assign(m_objs.size(), 0);
    float maxR = 0.f;
    for (auto const& o : m_objs) maxR = std::max(maxR, o.r);
    size_t const nCols = static_cast<size_t>(std::max(0.f, std::max(maxR, levelLength)) / kColumn) + 2;
    m_columns.assign(nCols, {});
    for (uint32_t i = 0; i < m_objs.size(); ++i) {
        MapObject const& o = m_objs[i];
        ++m_counts[static_cast<size_t>(o.kind)];
        if (i >= nStatic) {
            m_dynamic.push_back(i);
            m_isDynamic[i] = 1;
            continue;
        }
        int c0 = std::max(0, static_cast<int>(std::floor(o.l / kColumn)));
        int c1 = std::max(0, static_cast<int>(std::floor(o.r / kColumn)));
        for (int c = c0; c <= c1 && static_cast<size_t>(c) < nCols; ++c) m_columns[static_cast<size_t>(c)].push_back(i);
    }
    m_version = 1;
    m_buildMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void LevelMap::updateObject(uint32_t index, float l, float b, float r, float t, bool enabled) {
    if (index >= m_objs.size()) return;
    MapObject& o = m_objs[index];
    bool const moved = std::fabs(o.l - l) > 0.01f || std::fabs(o.b - b) > 0.01f || std::fabs(o.r - r) > 0.01f || std::fabs(o.t - t) > 0.01f;
    if (!moved && o.enabled == enabled) return;
    if (!m_isDynamic[index]) {
        // A static object that moves: from now on it is treated as dynamic.
        // Its old entries in the static index are filtered out by the flag.
        m_isDynamic[index] = 1;
        o.dynamic = true;
        m_dynamic.push_back(index);
        ++m_promoted;
    }
    o.l = l;
    o.b = b;
    o.r = r;
    o.t = t;
    o.enabled = enabled;
    ++m_version;
}

void LevelMap::query(float x0, float x1, std::vector<uint32_t>& out) const {
    out.clear();
    if (m_objs.empty() || x1 < x0) return;
    int const nCols = static_cast<int>(m_columns.size());
    int const c0 = std::clamp(static_cast<int>(std::floor(x0 / kColumn)), 0, nCols - 1);
    int const c1 = std::clamp(static_cast<int>(std::floor(x1 / kColumn)), 0, nCols - 1);
    for (int c = c0; c <= c1; ++c) {
        for (uint32_t i : m_columns[static_cast<size_t>(c)]) {
            MapObject const& o = m_objs[i];
            // each object once: in the first queried column it overlaps
            int const first = std::max(c0, static_cast<int>(std::floor(o.l / kColumn)));
            if (first != c || m_isDynamic[i] || !o.enabled) continue;
            if (o.r < x0 || o.l > x1) continue;
            out.push_back(i);
        }
    }
    std::sort(out.begin(), out.end()); // static indices are in left-edge order
    size_t const nStaticHits = out.size();
    for (uint32_t i : m_dynamic) {
        MapObject const& o = m_objs[i];
        if (!o.enabled || o.r < x0 || o.l > x1) continue;
        out.push_back(i);
    }
    if (out.size() > nStaticHits)
        std::sort(out.begin(), out.end(), [this](uint32_t a, uint32_t b) { return m_objs[a].l < m_objs[b].l; });
}

void LevelMap::collect(float x0, float x1, std::vector<Obstacle>& obstacles, std::vector<PhysicsEvent>& events,
                       bool gravityVerified, std::vector<Orb>* orbs, std::vector<Pad>* pads) const {
    thread_local std::vector<uint32_t> hits;
    query(x0, x1, hits);
    obstacles.clear();
    events.clear();
    if (orbs) orbs->clear();
    if (pads) pads->clear();
    for (uint32_t i : hits) {
        MapObject const& o = m_objs[i];
        if (o.r - o.l <= 0.f || o.t - o.b <= 0.f) continue;
        switch (o.kind) {
            case MapKind::Hazard: obstacles.push_back({o.l, o.b, o.r, o.t, Obstacle::Kind::Hazard}); break;
            case MapKind::Solid: obstacles.push_back({o.l, o.b, o.r, o.t, Obstacle::Kind::Solid}); break;
            case MapKind::SpeedPortal: {
                PhysicsEvent e{o.l, o.b, o.r, o.t, PhysicsEvent::Kind::Speed, o.value, true};
                events.push_back(e);
                break;
            }
            case MapKind::GravityPortal: {
                PhysicsEvent::Kind k = o.value == 0   ? PhysicsEvent::Kind::GravityNormal
                                       : o.value == 1 ? PhysicsEvent::Kind::GravityFlipped
                                                      : PhysicsEvent::Kind::GravityToggle;
                events.push_back({o.l, o.b, o.r, o.t, k, 0, gravityVerified});
                break;
            }
            case MapKind::ModePortal:
                events.push_back({o.l, o.b, o.r, o.t, PhysicsEvent::Kind::Mode, o.value, false});
                break;
            case MapKind::SizePortal:
                events.push_back({o.l, o.b, o.r, o.t, PhysicsEvent::Kind::Size, o.value, false});
                break;
            case MapKind::Orb:
                if (orbs) {
                    Orb b;
                    b.l = o.l;
                    b.b = o.b;
                    b.r = o.r;
                    b.t = o.t;
                    b.type = static_cast<OrbType>(std::min<int>(o.value, static_cast<int>(OrbType::Count) - 1));
                    b.rotation = o.rotation;
                    b.dashSpeed = o.dashSpeed;
                    b.maxDuration = o.maxDuration;
                    b.id = o.uniqueId;
                    orbs->push_back(b);
                    break;
                }
                obstacles.push_back({o.l, o.b, o.r, o.t, Obstacle::Kind::Uncertain});
                break;
            case MapKind::Pad:
                if (pads) {
                    Pad p;
                    p.l = o.l;
                    p.b = o.b;
                    p.r = o.r;
                    p.t = o.t;
                    p.type = static_cast<PadType>(std::min<int>(o.value, static_cast<int>(PadType::Count) - 1));
                    p.rotation = o.rotation;
                    p.id = o.uniqueId;
                    pads->push_back(p);
                    break;
                }
                obstacles.push_back({o.l, o.b, o.r, o.t, Obstacle::Kind::Uncertain});
                break;
            default: obstacles.push_back({o.l, o.b, o.r, o.t, Obstacle::Kind::Uncertain}); break;
        }
    }
}

} // namespace aigd
