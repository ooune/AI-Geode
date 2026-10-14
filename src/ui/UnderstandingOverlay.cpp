#include "UnderstandingOverlay.hpp"

#include <Geode/Geode.hpp>

using namespace geode::prelude;
using Style = aigd::UnderstandingOverlay::Style;

namespace {
ccColor4F color(Style s, float alpha = 1.f) {
    ccColor4F c;
    switch (s) {
        // extracted from the level
        case Style::Hazard: c = {1.f, 0.25f, 0.25f, 1.f}; break;
        case Style::Solid: c = {0.75f, 0.75f, 0.8f, 1.f}; break;
        case Style::Portal: c = {0.2f, 0.9f, 1.f, 1.f}; break;
        case Style::NotSimulated: c = {1.f, 0.9f, 0.2f, 1.f}; break;
        case Style::Dynamic: c = {1.f, 0.3f, 1.f, 1.f}; break;
        case Style::Approximate: c = {1.f, 0.6f, 0.15f, 1.f}; break;
        // inferred by the AI
        case Style::Group: c = {1.f, 1.f, 1.f, 1.f}; break;
        case Style::TightGroup: c = {1.f, 0.55f, 0.1f, 1.f}; break;
        case Style::Sequence: c = {0.35f, 0.6f, 1.f, 1.f}; break;
        case Style::LandingZone: c = {0.3f, 1.f, 0.4f, 1.f}; break;
        case Style::Target: c = {0.3f, 1.f, 0.4f, 1.f}; break;
        // predicted movement
        case Style::PlannedPath: c = {0.6f, 1.f, 0.6f, 1.f}; break;
        case Style::AlternativePath: c = {0.45f, 0.75f, 1.f, 1.f}; break;
        case Style::Press: c = {1.f, 1.f, 1.f, 1.f}; break;
        case Style::Player: c = {1.f, 1.f, 1.f, 1.f}; break;
        default: c = {1.f, 1.f, 1.f, 1.f}; break;
    }
    c.a = alpha;
    return c;
}
} // namespace

UnderstandingWorld* UnderstandingWorld::create() {
    auto* ret = new UnderstandingWorld();
    if (ret->init()) {
        ret->autorelease();
        return ret;
    }
    delete ret;
    return nullptr;
}

bool UnderstandingWorld::init() {
    if (!CCNode::init()) return false;
    m_draw = CCDrawNode::create();
    m_draw->setBlendFunc({GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA});
    this->addChild(m_draw);
    return true;
}

void UnderstandingWorld::clearAll() { m_draw->clear(); }

void UnderstandingWorld::render(aigd::UnderstandingOverlay const& g) {
    m_draw->clear();
    ccColor4F const none{0.f, 0.f, 0.f, 0.f};
    for (auto const& b : g.boxes) {
        // extracted objects: filled; inferred / uncertain: outline only
        ccColor4F const fill = b.filled ? color(b.style, 0.22f) : none;
        float const w = b.style == Style::Sequence ? 1.2f : 0.6f;
        m_draw->drawRect(CCRect(b.l, b.b, b.r - b.l, b.t - b.b), fill, w, color(b.style, b.filled ? 0.8f : 0.9f));
    }
    for (auto const& s : g.segments) {
        float const r = s.style == Style::LandingZone ? 1.5f : 1.2f;
        m_draw->drawSegment({s.x0, s.y0}, {s.x1, s.y1}, r, color(s.style, s.style == Style::LandingZone ? 0.6f : 0.85f));
    }
    for (auto const& d : g.dots) {
        float const r = d.style == Style::Player ? 2.5f : d.style == Style::Press ? 4.f : 3.f;
        m_draw->drawDot({d.x, d.y}, r, color(d.style, 0.95f));
    }
}

LevelStrip* LevelStrip::create() {
    auto* ret = new LevelStrip();
    if (ret->init()) {
        ret->autorelease();
        return ret;
    }
    delete ret;
    return nullptr;
}

bool LevelStrip::init() {
    if (!CCNode::init()) return false;
    m_static = CCDrawNode::create();
    m_static->setBlendFunc({GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA});
    m_marker = CCDrawNode::create();
    m_marker->setBlendFunc({GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA});
    this->addChild(m_static);
    this->addChild(m_marker, 1);
    m_warnings = CCLabelBMFont::create("", "chatFont.fnt");
    m_warnings->setScale(0.45f);
    m_warnings->setAnchorPoint({0.f, 0.f});
    this->addChild(m_warnings, 2);
    return true;
}

void LevelStrip::hideAll() {
    m_static->setVisible(false);
    m_marker->setVisible(false);
    m_warnings->setVisible(false);
}

void LevelStrip::render(std::vector<aigd::UnderstandingOverlay::Mark> const& marks, uint64_t version, float playerFraction,
                        std::vector<std::string> const& warnings, bool showStrip) {
    auto const win = CCDirector::sharedDirector()->getWinSize();
    float const x0 = 20.f, width = win.width - 40.f, y0 = 6.f, rowH = 5.f;
    m_static->setVisible(showStrip);
    m_marker->setVisible(showStrip);
    if (showStrip && version != m_version) {
        m_version = version;
        m_static->clear();
        m_static->drawRect(CCRect(x0 - 2.f, y0 - 2.f, width + 4.f, 3.f * rowH + 6.f), {0.f, 0.f, 0.f, 0.45f}, 0.5f, {1.f, 1.f, 1.f, 0.4f});
        for (auto const& m : marks) {
            float const a = x0 + m.f0 * width;
            float const b = x0 + std::max(m.f1 * width, m.f0 * width + 1.f);
            float const y = y0 + static_cast<float>(m.row) * (rowH + 1.f);
            m_static->drawRect(CCRect(a, y, b - a, rowH), color(m.style, 0.85f), 0.f, {0.f, 0.f, 0.f, 0.f});
        }
    }
    if (showStrip) {
        m_marker->clear();
        float const px = x0 + playerFraction * width;
        m_marker->drawSegment({px, y0 - 3.f}, {px, y0 + 3.f * rowH + 5.f}, 1.f, {1.f, 1.f, 1.f, 1.f});
    }
    if (warnings.empty()) {
        m_warnings->setVisible(false);
    } else {
        std::string text;
        for (size_t i = 0; i < warnings.size() && i < 3; ++i) text += (i ? "\n" : "") + warnings[i];
        m_warnings->setString(text.c_str());
        m_warnings->limitLabelWidth(width, 0.45f, 0.25f);
        m_warnings->setPosition({x0, y0 + 3.f * rowH + 10.f});
        m_warnings->setVisible(true);
    }
}
