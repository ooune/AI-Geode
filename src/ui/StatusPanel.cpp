#include "StatusPanel.hpp"

#include "AIController.hpp"

#include <Geode/Geode.hpp>
#include <algorithm>

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

    auto const& st = ai.status();
    std::string text;
    if (!ai.supported()) text = "AI unavailable\n" + ai.unsupportedReason();
    else if (!set.aiEnabled) text = "AI OFF  /  F8 to enable";
    else {
        text = fmt::format("AI {}  /  Attempt {}\nProgress {}   Best {}\n{}  /  {}",
            set.learning ? "LEARNING" : "PLAYING", st.attempt,
            percent(st.progress), percent(st.bestProgress),
            aigd::gameModeName(ai.lastObservation().mode), aigd::decisionSourceName(st.source));
        if (set.prediction && st.predictionValid)
            text += fmt::format("\n{}  /  {}", st.safety.executed ? "PRESS" : "RELEASE",
                aigd::safetyVerdictName(st.safety.verdict));
        if (set.showDebug) {
            auto const& adv = ai.predictor().lastAdvice();
            text += fmt::format("\nPlans {}   Overrides {}\nSearch {} sims / {:.0f} us\nPrediction error {:.2f} units",
                ai.plans().adopted, st.overridesThisAttempt, adv.simulations, adv.micros,
                ai.predictor().lastStepError());
            auto const& analysis = ai.analysis();
            if (analysis.valid)
                text += fmt::format("\nMap: {} groups / {} changes", analysis.groups.size(), analysis.changes.size());
        }
    }
    if (!ai.notice().empty()) {
        auto notice = ai.notice();
        if (notice.size() > 70) notice = notice.substr(0, 67) + "...";
        text += "\n" + notice;
    }
    if (text != m_text) {
        m_text = text;
        m_label->setString(text.c_str());
    }
    auto win = CCDirector::get()->getWinSize();
    this->setPosition({10.f, win.height - 30.f});
    float width = std::min(240.f, win.width * 0.42f);
    m_label->limitLabelWidth(width, kScale, 0.25f);
    auto size = m_label->getScaledContentSize();
    float maxHeight = win.height * 0.42f;
    if (size.height > maxHeight) {
        m_label->setScale(m_label->getScale() * maxHeight / size.height);
        size = m_label->getScaledContentSize();
    }
    m_label->setColor(set.aiEnabled ? ccColor3B{225, 245, 255} : ccColor3B{185, 195, 210});
    m_background->setContentSize({size.width + kPad * 2.f, size.height + kPad * 2.f});
    m_background->setPosition({-kPad, -size.height - kPad});
    m_label->setPosition({0.f, 0.f});
}
