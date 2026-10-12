#include "StatusPanel.hpp"

#include "AIController.hpp"

#include <Geode/Geode.hpp>

using namespace geode::prelude;

namespace {
constexpr float kScale = 0.42f;
constexpr float kPad = 4.f;

std::string percent(float p) {
    return fmt::format("{:.1f}%", p * 100.f);
}
} // namespace

StatusPanel* StatusPanel::create() {
    auto* ret = new StatusPanel();
    if (ret->init()) {
        ret->autorelease();
        return ret;
    }
    delete ret;
    return nullptr;
}

bool StatusPanel::init() {
    if (!CCNode::init()) return false;
    auto win = CCDirector::get()->getWinSize();
    this->setAnchorPoint({0.f, 1.f});
    this->setPosition({6.f, win.height - 24.f});

    m_background = CCLayerColor::create({0, 0, 0, 130}, 10.f, 10.f);
    m_background->setAnchorPoint({0.f, 1.f});
    this->addChild(m_background);

    m_label = CCLabelBMFont::create("", "chatFont.fnt");
    m_label->setAnchorPoint({0.f, 1.f});
    m_label->setScale(kScale);
    m_label->setAlignment(kCCTextAlignmentLeft);
    this->addChild(m_label, 1);
    return true;
}

void StatusPanel::refresh() {
    auto& ai = AIController::get();
    auto const& set = ai.settings();
    this->setVisible(set.showPanel);
    if (!set.showPanel) return;

    std::string text;
    if (!ai.supported()) {
        text = "AI: unavailable here\n" + ai.unsupportedReason();
    } else if (!set.aiEnabled) {
        text = "AI: off  (toggle with the keybind, default F8)";
        if (auto const* mem = ai.levelMemory(); mem && mem->totalAttempts > 0) {
            text += fmt::format("\nRemembered: {} attempts, best {}, {} completions", mem->totalAttempts, percent(mem->bestProgress), mem->completions);
        }
    } else {
        auto const& st = ai.status();
        auto const* mem = ai.levelMemory();
        text = fmt::format(
            "AI: {}{}{}  |  Attempt {}  |  Deaths {}\n"
            "Progress {}  Best {}  Completions {} ({:.0f}% of recent)",
            aigd::agentPhaseName(st.phase), set.learning ? "" : " (learning off)", st.stuck ? " - stuck, widening search" : "",
            st.attempt, st.deaths,
            percent(st.progress), percent(st.bestProgress), st.completions, st.completionRate * 100.f
        );
        if (st.lastFailureProgress >= 0.f) text += "\nLast failure: " + st.lastFailure;
        if (!st.lastChange.empty()) text += "\nChange: " + st.lastChange;
        if (st.lastImproved >= 0) text += st.lastImproved ? "\nResult: went further than the previous attempt" : "\nResult: did not get further";
        if (!ai.notice().empty()) text += "\n" + ai.notice();

        if (set.showDebug) {
            auto const& o = ai.lastObservation();
            auto feat = [](char const* name, aigd::Feature const& f) {
                return f.present ? fmt::format(" {} dx={:.0f} dy={:.0f}", name, f.dx, f.dy) : std::string();
            };
            text += fmt::format(
                "\n[obs] {} {}{}{} x={:.1f} y={:.1f} vy={:.2f} speed={:.1f}{}{}{}",
                aigd::gameModeName(o.mode), o.onGround ? "ground" : "air", o.upsideDown ? " flipped" : "", o.mini ? " mini" : "",
                o.x, o.y, o.vy, o.playerSpeed, feat("hazard", o.hazard), feat("solid", o.solid), feat("orb", o.orb)
            );
            text += fmt::format(
                "\n[decide] Q(release)={:.2f} Q(hold)={:.2f} -> {}{}{}  eps={:.2f}",
                st.q[0], st.q[1], st.action ? "HOLD" : "release", st.forced ? " (experiment)" : "", st.explored ? " (explore)" : "", st.epsilon
            );
            text += fmt::format(
                "\n[memory] {} level states, {} pattern states, {} hotspots, {} objects indexed",
                st.tableSize, st.patternSize, mem ? mem->hotspots.size() : 0, ai.indexedObjects()
            );
        }
    }

    m_label->setString(text.c_str());
    auto size = m_label->getScaledContentSize();
    m_background->setContentSize({size.width + kPad * 2.f, size.height + kPad * 2.f});
    m_background->setPosition({-kPad, -size.height - kPad});
    m_label->setPosition({0.f, 0.f});
}
