#include "Observer.hpp"

#include <Geode/Geode.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <unordered_set>

using namespace geode::prelude;

namespace {
constexpr float kLookBehind = 15.f;
constexpr float kLookAhead = 240.f;     // 8 blocks
constexpr float kMaxDy = 90.f;          // 3 blocks above/below
constexpr float kIndexPadding = 120.f;  // tolerance for objects moved by triggers
constexpr int kMaxScan = 512;           // hard cap on objects inspected per tick

bool isSpeedPortal(int id) { return id == 200 || id == 201 || id == 202 || id == 203 || id == 1334; }

// Objects that change the player's movement in ways the physics model does not
// simulate: a predicted path that touches one becomes "uncertain".
bool isUnmodelled(GameObject* obj) {
    switch (obj->m_objectType) {
        case GameObjectType::Slope:
        case GameObjectType::YellowJumpPad:
        case GameObjectType::PinkJumpPad:
        case GameObjectType::GravityPad:
        case GameObjectType::RedJumpPad:
        case GameObjectType::SpiderPad:
        case GameObjectType::InverseGravityPortal:
        case GameObjectType::NormalGravityPortal:
        case GameObjectType::GravityTogglePortal:
        case GameObjectType::ShipPortal:
        case GameObjectType::CubePortal:
        case GameObjectType::BallPortal:
        case GameObjectType::UfoPortal:
        case GameObjectType::WavePortal:
        case GameObjectType::RobotPortal:
        case GameObjectType::SpiderPortal:
        case GameObjectType::SwingPortal:
        case GameObjectType::RegularSizePortal:
        case GameObjectType::MiniSizePortal:
        case GameObjectType::DualPortal:
        case GameObjectType::SoloPortal:
        case GameObjectType::TeleportPortal:
            return true;
        case GameObjectType::Solid:
        case GameObjectType::Breakable:
            return obj->m_isPassable; // one-way platforms
        default:
            return isSpeedPortal(obj->m_objectID);
    }
}

std::optional<uint8_t> classify(GameObjectType type) {
    switch (type) {
        case GameObjectType::Hazard:
        case GameObjectType::AnimatedHazard:
            return 0;
        case GameObjectType::Solid:
        case GameObjectType::Breakable:
        case GameObjectType::Slope:
            return 1;
        case GameObjectType::YellowJumpRing:
        case GameObjectType::PinkJumpRing:
        case GameObjectType::GravityRing:
        case GameObjectType::GreenRing:
        case GameObjectType::DropRing:
        case GameObjectType::RedJumpRing:
        case GameObjectType::CustomRing:
        case GameObjectType::DashRing:
        case GameObjectType::GravityDashRing:
        case GameObjectType::SpiderOrb:
        case GameObjectType::TeleportOrb:
            return 2;
        default:
            return std::nullopt;
    }
}
} // namespace

namespace {
// What an object is for the level map: kind and value (speed class, mode, ...).
std::optional<std::pair<aigd::MapKind, uint8_t>> classifyForMap(GameObject* obj) {
    using K = aigd::MapKind;
    using M = aigd::GameMode;
    switch (obj->m_objectID) {
        case 200: return std::pair{K::SpeedPortal, uint8_t(0)}; // 0.5x
        case 201: return std::pair{K::SpeedPortal, uint8_t(1)}; // 1x
        case 202: return std::pair{K::SpeedPortal, uint8_t(2)}; // 2x
        case 203: return std::pair{K::SpeedPortal, uint8_t(3)}; // 3x
        case 1334: return std::pair{K::SpeedPortal, uint8_t(4)}; // 4x
        default: break;
    }
    auto mode = [](M m) { return std::pair{K::ModePortal, static_cast<uint8_t>(m)}; };
    switch (obj->m_objectType) {
        case GameObjectType::Hazard:
        case GameObjectType::AnimatedHazard: return std::pair{K::Hazard, uint8_t(0)};
        case GameObjectType::Solid:
        case GameObjectType::Breakable: return std::pair{obj->m_isPassable ? K::Platform : K::Solid, uint8_t(0)};
        case GameObjectType::Slope: return std::pair{K::Slope, uint8_t(0)};
        case GameObjectType::YellowJumpPad:
        case GameObjectType::PinkJumpPad:
        case GameObjectType::GravityPad:
        case GameObjectType::RedJumpPad:
        case GameObjectType::SpiderPad: return std::pair{K::Pad, uint8_t(0)};
        case GameObjectType::YellowJumpRing: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Yellow)};
        case GameObjectType::PinkJumpRing: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Pink)};
        case GameObjectType::RedJumpRing: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Red)};
        case GameObjectType::GravityRing: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Blue)};
        case GameObjectType::GreenRing: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Green)};
        case GameObjectType::DropRing: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Black)};
        case GameObjectType::DashRing: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Dash)};
        case GameObjectType::GravityDashRing: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::GravityDash)};
        case GameObjectType::SpiderOrb: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Spider)};
        case GameObjectType::TeleportOrb: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Teleport)};
        case GameObjectType::CustomRing: return std::pair{K::Orb, static_cast<uint8_t>(aigd::OrbType::Custom)};
        case GameObjectType::NormalGravityPortal: return std::pair{K::GravityPortal, uint8_t(0)};
        case GameObjectType::InverseGravityPortal: return std::pair{K::GravityPortal, uint8_t(1)};
        case GameObjectType::GravityTogglePortal: return std::pair{K::GravityPortal, uint8_t(2)};
        case GameObjectType::CubePortal: return mode(M::Cube);
        case GameObjectType::ShipPortal: return mode(M::Ship);
        case GameObjectType::BallPortal: return mode(M::Ball);
        case GameObjectType::UfoPortal: return mode(M::Ufo);
        case GameObjectType::WavePortal: return mode(M::Wave);
        case GameObjectType::RobotPortal: return mode(M::Robot);
        case GameObjectType::SpiderPortal: return mode(M::Spider);
        case GameObjectType::SwingPortal: return mode(M::Swing);
        case GameObjectType::RegularSizePortal: return std::pair{K::SizePortal, uint8_t(0)};
        case GameObjectType::MiniSizePortal: return std::pair{K::SizePortal, uint8_t(1)};
        case GameObjectType::DualPortal:
        case GameObjectType::SoloPortal:
        case GameObjectType::TeleportPortal: return std::pair{K::OtherPortal, uint8_t(0)};
        default: return std::nullopt;
    }
}

// Triggers that move, rotate, toggle or animate the objects of a group.
bool movesGroups(int id) {
    switch (id) {
        case 901:  // move
        case 1346: // rotate
        case 1049: // toggle
        case 1347: // follow
        case 1814: // follow player y
        case 1585: // animate
        case 3006: case 3007: case 3008: case 3009: case 3010: // area move / rotate / scale / fade / tint
        case 3011: case 3012: case 3013: case 3014: case 3015: // edit area triggers
        case 3016: // advanced follow
        case 3660: case 3661: // edit / re-target advanced follow
        case 3033: // keyframe animation
            return true;
        default: return false;
    }
}
} // namespace

void Observer::reset() {
    m_mapSources.clear();
    m_items.clear();
    m_items.shrink_to_fit();
    m_cursor = 0;
    m_lastX = 0.f;
    m_indexedCount = ~0u;
    m_layer = nullptr;
}

bool Observer::ensureIndex(PlayLayer* layer) {
    if (!layer || !layer->m_objects) return false;
    unsigned count = layer->m_objects->count();
    if (layer == m_layer && count == m_indexedCount) return false;

    m_items.clear();
    m_layer = layer;
    m_indexedCount = count;
    m_items.reserve(count / 4);
    for (auto* obj : CCArrayExt<GameObject*>(layer->m_objects)) {
        if (!obj) continue;
        auto kind = classify(obj->m_objectType);
        bool unmodelled = isUnmodelled(obj);
        if (!kind && !unmodelled) continue;
        m_items.push_back({obj, obj->getPositionX(), kind ? static_cast<Kind>(*kind) : Kind::Other, unmodelled});
    }
    std::sort(m_items.begin(), m_items.end(), [](Item const& a, Item const& b) { return a.x < b.x; });
    m_cursor = 0;
    m_lastX = 0.f;
    log::info("Observer indexed {} relevant objects (of {})", m_items.size(), count);
    return true;
}

void Observer::buildMap(PlayLayer* layer, aigd::LevelMap& map, float floorSurface, MapStats& stats) {
    stats = MapStats{};
    m_mapSources.clear();
    map.clear();
    if (!layer || !layer->m_objects) return;
    stats.levelObjects = layer->m_objects->count();
    // groups that triggers move or toggle
    std::unordered_set<int> moved;
    for (auto* obj : CCArrayExt<GameObject*>(layer->m_objects)) {
        if (!obj || !obj->m_isTrigger || !movesGroups(obj->m_objectID)) continue;
        if (auto* e = typeinfo_cast<EffectGameObject*>(obj)) {
            ++stats.triggers;
            if (e->m_targetGroupID > 0) moved.insert(e->m_targetGroupID);
        }
    }
    stats.movedGroups = moved.size();
    std::vector<aigd::MapObject> objs;
    objs.reserve(stats.levelObjects / 3);
    for (auto* obj : CCArrayExt<GameObject*>(layer->m_objects)) {
        if (!obj || obj->m_isNoTouch) continue;
        auto k = classifyForMap(obj);
        if (!k) continue;
        auto const& rect = obj->getObjectRect();
        if (rect.size.width <= 0.f || rect.size.height <= 0.f) {
            ++stats.skipped;
            continue;
        }
        aigd::MapObject m;
        m.l = rect.getMinX();
        m.b = rect.getMinY();
        m.r = rect.getMaxX();
        m.t = rect.getMaxY();
        m.kind = k->first;
        m.value = k->second;
        m.enabled = !obj->m_isGroupDisabled;
        float const rot = std::fmod(std::fabs(obj->getRotation()), 90.f);
        m.approximate = rot > 1.f && rot < 89.f; // the box is the bounding box of a rotated hitbox
        if (obj->m_groups && !moved.empty()) {
            for (int g = 0; g < obj->m_groupCount && g < 10; ++g)
                if (moved.count((*obj->m_groups)[static_cast<size_t>(g)])) m.dynamic = true;
        }
        if (m.kind == aigd::MapKind::Orb) {
            m.rotation = obj->getRotation();
            m.uniqueId = obj->m_uniqueID;
            if (auto* dash = typeinfo_cast<DashRingObject*>(obj)) {
                m.dashSpeed = dash->m_dashSpeed;
                m.maxDuration = dash->m_maxDuration;
            }
        }
        m.source = static_cast<uint32_t>(m_mapSources.size());
        m_mapSources.push_back(obj);
        objs.push_back(m);
    }
    map.build(std::move(objs), std::max(1.f, layer->m_levelLength), floorSurface);
}

void Observer::refreshMap(aigd::LevelMap& map, float x, int32_t tick) {
    if (map.empty() || m_mapSources.empty()) return;
    bool const all = (tick & 7) == 0;
    auto const& dyn = map.dynamicObjects();
    for (size_t k = 0; k < dyn.size(); ++k) {
        uint32_t const i = dyn[k];
        auto const& m = map.at(i);
        if (!all && (m.r < x - 200.f || m.l > x + 1400.f)) continue;
        if (m.source >= m_mapSources.size()) continue;
        GameObject* obj = m_mapSources[m.source];
        auto const& r = obj->getObjectRect();
        map.updateObject(i, r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY(), !obj->m_isGroupDisabled);
    }
    if (tick % 30 == 0) {
        // static objects ahead: check that they really do not move
        thread_local std::vector<uint32_t> hits;
        map.query(x, x + 600.f, hits);
        int checked = 0;
        for (uint32_t i : hits) {
            if (checked++ >= 64) break;
            auto const& m = map.at(i);
            if (m.dynamic || m.source >= m_mapSources.size()) continue;
            GameObject* obj = m_mapSources[m.source];
            auto const& r = obj->getObjectRect();
            bool const enabled = !obj->m_isGroupDisabled;
            if (std::fabs(r.getMinX() - m.l) > 1.f || std::fabs(r.getMinY() - m.b) > 1.f || enabled != m.enabled)
                map.updateObject(i, r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY(), enabled);
        }
    }
}

aigd::GameMode Observer::modeOf(PlayerObject* p) {
    using aigd::GameMode;
    if (p->m_isShip) return GameMode::Ship;
    if (p->m_isBird) return GameMode::Ufo;
    if (p->m_isBall) return GameMode::Ball;
    if (p->m_isDart) return GameMode::Wave;
    if (p->m_isRobot) return GameMode::Robot;
    if (p->m_isSpider) return GameMode::Spider;
    if (p->m_isSwing) return GameMode::Swing;
    return GameMode::Cube;
}

aigd::Observation Observer::observe(PlayLayer* layer, int32_t tick) {
    aigd::Observation o;
    auto* p = layer->m_player1;
    if (!p) return o;

    o.tick = tick;
    o.x = p->m_position.x;
    o.y = p->m_position.y;
    o.vy = static_cast<float>(p->m_yVelocity);
    o.mode = modeOf(p);
    o.onGround = p->m_isOnGround;
    o.upsideDown = p->m_isUpsideDown;
    o.mini = p->m_vehicleSize < 0.9f;
    o.dead = p->m_isDead;
    o.playerSpeed = p->m_playerSpeed;
    o.levelLength = std::max(1.f, layer->m_levelLength);
    o.dashing = p->m_isDashing;

    if (m_items.empty()) return o;

    // Move the cursor to the first object that could be in range. Positions
    // can jump backwards (respawn at a checkpoint), so re-seek in that case.
    float from = o.x - kLookBehind - kIndexPadding;
    if (o.x < m_lastX - 1.f || m_cursor > m_items.size()) {
        m_cursor = static_cast<size_t>(std::lower_bound(m_items.begin(), m_items.end(), from,
            [](Item const& it, float v) { return it.x < v; }) - m_items.begin());
    }
    while (m_cursor < m_items.size() && m_items[m_cursor].x < from) ++m_cursor;
    m_lastX = o.x;

    float sign = o.upsideDown ? -1.f : 1.f;
    float best[3] = {1e9f, 1e9f, 1e9f};
    aigd::Feature* out[3] = {&o.hazard, &o.solid, &o.orb};
    int scanned = 0;
    for (size_t i = m_cursor; i < m_items.size() && scanned < kMaxScan; ++i, ++scanned) {
        auto const& it = m_items[i];
        if (it.x > o.x + kLookAhead + kIndexPadding) break;
        auto* obj = it.object;
        if (obj->m_isGroupDisabled || it.kind == Kind::Other) continue;
        auto const& rect = obj->getObjectRect();
        float cx = rect.getMidX();
        float cy = rect.getMidY();
        float dx = cx - o.x;
        float dy = (cy - o.y) * sign;
        if (dx < -kLookBehind || dx > kLookAhead || std::fabs(dy) > kMaxDy) continue;
        auto k = static_cast<size_t>(it.kind);
        if (dx < best[k]) {
            best[k] = dx;
            out[k]->present = true;
            out[k]->dx = dx;
            out[k]->dy = dy;
        }
    }
    return o;
}

void Observer::collectObstacles(float x0, float x1, std::vector<aigd::Obstacle>& out) const {
    out.clear();
    if (m_items.empty()) return;
    auto it = std::lower_bound(m_items.begin(), m_items.end(), x0 - kIndexPadding,
                               [](Item const& item, float v) { return item.x < v; });
    for (; it != m_items.end() && out.size() < kMaxObstacles; ++it) {
        if (it->x > x1 + kIndexPadding) break;
        auto* obj = it->object;
        if (obj->m_isGroupDisabled || obj->m_isNoTouch) continue;
        auto const& rect = obj->getObjectRect();
        if (rect.getMaxX() < x0 || rect.getMinX() > x1 || rect.size.width <= 0.f || rect.size.height <= 0.f) continue;
        aigd::Obstacle o;
        o.l = rect.getMinX();
        o.b = rect.getMinY();
        o.r = rect.getMaxX();
        o.t = rect.getMaxY();
        if (it->unmodelled) o.kind = aigd::Obstacle::Kind::Uncertain;
        else if (it->kind == Kind::Hazard) o.kind = aigd::Obstacle::Kind::Hazard;
        else if (it->kind == Kind::Solid) o.kind = aigd::Obstacle::Kind::Solid;
        else o.kind = aigd::Obstacle::Kind::Uncertain; // orbs: act only if pressed inside them
        out.push_back(o);
    }
    std::sort(out.begin(), out.end(), [](aigd::Obstacle const& a, aigd::Obstacle const& b) { return a.l < b.l; });
}

aigd::DeathInfo Observer::deathInfo(PlayLayer* layer, PlayerObject* player, GameObject* killer) const {
    aigd::DeathInfo d;
    if (!player) return d;
    d.x = player->m_position.x;
    d.y = player->m_position.y;
    if (killer && killer != layer->m_anticheatSpike) {
        auto const& rect = killer->getObjectRect();
        float sign = player->m_isUpsideDown ? -1.f : 1.f;
        d.hasKiller = true;
        d.killerDx = rect.getMidX() - d.x;
        d.killerDy = (rect.getMidY() - d.y) * sign;
        float top = (rect.getMaxY() - d.y) * sign;
        float bottom = (rect.getMinY() - d.y) * sign;
        d.hasExtent = true;
        d.killerTop = std::max(top, bottom);
        d.killerBottom = std::min(top, bottom);
        auto kind = classify(killer->m_objectType);
        d.killer = !kind ? aigd::KillerKind::Unknown : (*kind == 0 ? aigd::KillerKind::Hazard : aigd::KillerKind::Solid);
    }
    return d;
}
