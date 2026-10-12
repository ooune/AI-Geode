#include "TrajectoryOverlay.hpp"

#include <Geode/Geode.hpp>

using namespace geode::prelude;
using Style = aigd::OverlayGeometry::Style;

namespace {
ccColor4F color(Style s) {
    switch (s) {
        case Style::PathSafe: return {0.25f, 1.f, 0.35f, 0.95f};
        case Style::PathUncertain: return {1.f, 0.85f, 0.2f, 0.95f};
        case Style::PathDanger: return {1.f, 0.2f, 0.2f, 0.95f};
        case Style::Alternative: return {0.45f, 0.75f, 1.f, 0.55f};
        case Style::Landing: return {1.f, 1.f, 1.f, 0.95f};
        case Style::Collision: return {1.f, 0.1f, 0.1f, 1.f};
        case Style::Hazard: return {1.f, 0.6f, 0.1f, 0.8f};
        case Style::HazardHit: return {1.f, 0.1f, 0.1f, 1.f};
        case Style::Keep: return {0.85f, 0.45f, 1.f, 0.6f};
        case Style::Stale: return {0.6f, 0.6f, 0.6f, 0.7f};
        default: return {1.f, 1.f, 1.f, 1.f};
    }
}
} // namespace

TrajectoryOverlay* TrajectoryOverlay::create() {
    auto* ret = new TrajectoryOverlay();
    if (ret->init()) {
        ret->autorelease();
        return ret;
    }
    delete ret;
    return nullptr;
}

bool TrajectoryOverlay::init() {
    if (!CCNode::init()) return false;
    m_draw = CCDrawNode::create();
    // GD's CCDrawNode ignores its blend func unless asked; keep alpha working.
    m_draw->setBlendFunc({GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA});
    this->addChild(m_draw);
    m_label = CCLabelBMFont::create("", "bigFont.fnt");
    m_label->setScale(0.35f);
    m_label->setAnchorPoint({0.5f, 0.f});
    this->addChild(m_label, 1);
    return true;
}

void TrajectoryOverlay::clearAll() {
    m_draw->clear();
    m_label->setVisible(false);
}

void TrajectoryOverlay::render(aigd::OverlayGeometry const& g) {
    m_draw->clear();
    ccColor4F const none{0.f, 0.f, 0.f, 0.f};
    for (auto const& b : g.boxes) {
        float w = b.style == Style::HazardHit ? 1.5f : 0.75f;
        m_draw->drawRect(CCRect(b.l, b.b, b.r - b.l, b.t - b.b), none, w, color(b.style));
    }
    for (auto const& s : g.segments) {
        float r = s.style == Style::Alternative || s.style == Style::Keep ? 1.f : 1.6f;
        m_draw->drawSegment({s.x0, s.y0}, {s.x1, s.y1}, r, color(s.style));
    }
    for (auto const& d : g.dots) {
        float r = d.style == Style::Collision ? 5.f : 3.5f;
        m_draw->drawDot({d.x, d.y}, r, color(d.style));
    }

    if (g.label.empty()) {
        m_label->setVisible(false);
        return;
    }
    m_label->setVisible(true);
    m_label->setString(g.label.c_str());
    auto c = color(g.outcome == aigd::Certainty::Safe ? Style::PathSafe
                   : g.outcome == aigd::Certainty::Uncertain ? Style::PathUncertain : Style::PathDanger);
    m_label->setColor({static_cast<GLubyte>(c.r * 255), static_cast<GLubyte>(c.g * 255), static_cast<GLubyte>(c.b * 255)});
    if (g.hasCollision) {
        m_label->setPosition({g.collisionX, g.collisionY + 22.f});
    } else if (!g.segments.empty()) {
        auto const& last = g.segments.back();
        m_label->setPosition({last.x1, last.y1 + 22.f});
    }
}
