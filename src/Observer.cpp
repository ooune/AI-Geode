#include "Observer.hpp"

#include <Geode/Geode.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

using namespace geode::prelude;

namespace {
constexpr float kLookBehind = 15.f;
constexpr float kLookAhead = 240.f;     // 8 blocks
constexpr float kMaxDy = 90.f;          // 3 blocks above/below
constexpr float kIndexPadding = 120.f;  // tolerance for objects moved by triggers
constexpr int kMaxScan = 512;           // hard cap on objects inspected per tick

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

void Observer::reset() {
    m_items.clear();
    m_items.shrink_to_fit();
    m_cursor = 0;
    m_lastX = 0.f;
    m_indexedCount = ~0u;
    m_layer = nullptr;
}

void Observer::ensureIndex(PlayLayer* layer) {
    if (!layer || !layer->m_objects) return;
    unsigned count = layer->m_objects->count();
    if (layer == m_layer && count == m_indexedCount) return;

    m_items.clear();
    m_layer = layer;
    m_indexedCount = count;
    m_items.reserve(count / 4);
    for (auto* obj : CCArrayExt<GameObject*>(layer->m_objects)) {
        if (!obj) continue;
        auto kind = classify(obj->m_objectType);
        if (!kind) continue;
        m_items.push_back({obj, obj->getPositionX(), static_cast<Kind>(*kind)});
    }
    std::sort(m_items.begin(), m_items.end(), [](Item const& a, Item const& b) { return a.x < b.x; });
    m_cursor = 0;
    m_lastX = 0.f;
    log::info("Observer indexed {} relevant objects (of {})", m_items.size(), count);
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
        if (obj->m_isGroupDisabled) continue;
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
