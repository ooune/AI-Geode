#include "HitboxTrace.hpp"

#include "Observer.hpp"

#include <Geode/Geode.hpp>

#include <algorithm>
#include <cmath>

using namespace geode::prelude;

namespace {
constexpr size_t kMaxRows = 60000;

bool rotatedOffAxis(GameObject* o) {
    float const r = std::fmod(std::fabs(o->getRotation()), 90.f);
    return r > 0.01f && r < 89.99f;
}

float rectGap(CCRect const& a, CCRect const& b) {
    float const gx = std::max(a.getMinX() - b.getMaxX(), b.getMinX() - a.getMaxX());
    float const gy = std::max(a.getMinY() - b.getMaxY(), b.getMinY() - a.getMaxY());
    return std::max(gx, gy);
}
} // namespace

void HitboxTrace::begin(std::string const& path) {
    end();
    m_file = std::fopen(path.c_str(), "a");
    m_rows = 0;
    if (m_file) std::fprintf(m_file, "# level start\n");
}

void HitboxTrace::end() {
    if (m_file) std::fclose(m_file);
    m_file = nullptr;
}

void HitboxTrace::player(char kind, PlayerObject* p, int32_t tick) {
    auto const outer = p->getObjectRect();
    auto const inner = p->getObjectRect(0.3f, 0.3f);
    std::fprintf(m_file, "%c,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", kind, tick,
                 static_cast<int>(Observer::modeOf(p)), p->m_vehicleSize < 0.9f ? 1 : 0, p->m_isUpsideDown ? 1 : 0, p->m_vehicleSize,
                 p->m_position.x, p->m_position.y, outer.getMinX(), outer.getMinY(), outer.getMaxX(), outer.getMaxY(), inner.getMinX(),
                 inner.getMinY(), inner.getMaxX(), inner.getMaxY(), p->getRotation(), p->getObjectRotation(),
                 static_cast<float>(p->m_yVelocity));
    OBB2D* box = p->getOrientedBox();
    if (box)
        for (auto const& c : box->m_corners) std::fprintf(m_file, ",%.3f,%.3f", c.x, c.y);
    else
        std::fprintf(m_file, ",,,,,,,,");
}

void HitboxTrace::object(GameObject* o) {
    auto const rect = o->getObjectRect();
    std::fprintf(m_file, ",%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d", o->m_objectID,
                 static_cast<int>(o->m_objectType), static_cast<float>(o->m_positionX), static_cast<float>(o->m_positionY),
                 o->getRotation(), o->getScaleX(), o->getScaleY(), o->m_objectRadius, rect.getMinX(), rect.getMinY(), rect.getMaxX(),
                 rect.getMaxY(), o->m_isFlipX ? 1 : 0, o->m_isFlipY ? 1 : 0);
    OBB2D* box = rotatedOffAxis(o) ? o->getOrientedBox() : nullptr;
    if (box)
        for (auto const& c : box->m_corners) std::fprintf(m_file, ",%.3f,%.3f", c.x, c.y);
    std::fprintf(m_file, "\n");
}

void HitboxTrace::step(PlayLayer*, PlayerObject* p, int32_t tick, std::vector<GameObject*> const& nearby) {
    if (!m_file || !p || m_rows >= kMaxRows) return;
    if (tick % 30 == 0) {
        player('P', p, tick);
        std::fprintf(m_file, "\n");
        ++m_rows;
    }
    auto const outer = p->getObjectRect();
    for (auto* o : nearby) {
        bool const hazard = o->m_objectType == GameObjectType::Hazard || o->m_objectType == GameObjectType::AnimatedHazard;
        if (!hazard || (o->m_objectRadius <= 0.f && !rotatedOffAxis(o))) continue;
        if (rectGap(outer, o->getObjectRect()) > 8.f) continue;
        player('N', p, tick);
        object(o);
        ++m_rows;
    }
}

void HitboxTrace::death(PlayLayer*, PlayerObject* p, GameObject* killer, int32_t tick) {
    if (!m_file || !p || !killer) return;
    player('D', p, tick);
    object(killer);
    ++m_rows;
    std::fflush(m_file);
}
