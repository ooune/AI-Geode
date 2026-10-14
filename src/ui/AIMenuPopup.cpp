#include "AIMenuPopup.hpp"

#include "AIController.hpp"

#include <Geode/Geode.hpp>
#include <Geode/ui/GeodeUI.hpp>

#include <algorithm>
#include <cmath>

using namespace geode::prelude;

namespace {
enum ToggleTag { kAI = 0, kLearning, kAutoRetry, kPractice, kDebug, kPanel, kPrediction, kTrajectory };
constexpr float kWidth = 360.f;
constexpr float kHeight = 330.f;

char const* settingKey(int tag) {
    switch (tag) {
        case kLearning: return "learning-enabled";
        case kAutoRetry: return "auto-retry";
        case kPractice: return "practice-mode";
        case kDebug: return "show-debug";
        case kPanel: return "show-panel";
        case kPrediction: return "prediction-enabled";
        case kTrajectory: return "show-trajectory";
        default: return "ai-enabled";
    }
}
} // namespace

AIMenuPopup* AIMenuPopup::create() {
    auto* ret = new AIMenuPopup();
    if (ret->init()) {
        ret->autorelease();
        return ret;
    }
    delete ret;
    return nullptr;
}

bool AIMenuPopup::init() {
    if (!Popup::init(kWidth, kHeight)) return false;
    this->setTitle("AI Player");

    auto const& s = AIController::get().settings();
    auto* menu = CCMenu::create();
    menu->setPosition({0.f, 0.f});
    m_mainLayer->addChild(menu);

    float left = 30.f;
    float right = kWidth / 2.f + 22.f;
    float top = kHeight - 55.f;
    addToggle(menu, "AI Player", kAI, s.aiEnabled, {left, top});
    addToggle(menu, "Learning", kLearning, s.learning, {left, top - 26.f});
    addToggle(menu, "Auto Retry", kAutoRetry, s.autoRetry, {left, top - 52.f});
    addToggle(menu, "Practice Mode", kPractice, s.practice, {right, top});
    addToggle(menu, "Show Panel", kPanel, s.showPanel, {right, top - 26.f});
    addToggle(menu, "Show Debug Data", kDebug, s.showDebug, {right, top - 52.f});
    addToggle(menu, "Prediction", kPrediction, s.prediction, {left, top - 78.f});
    addToggle(menu, "Show Trajectory", kTrajectory, s.showTrajectory, {right, top - 78.f});

    // Learning speed: - [value] +
    float speedY = top - 108.f;
    auto* speedTitle = CCLabelBMFont::create("Learning Speed", "bigFont.fnt");
    speedTitle->setScale(0.4f);
    speedTitle->setAnchorPoint({0.f, 0.5f});
    speedTitle->setPosition({left - 10.f, speedY});
    m_mainLayer->addChild(speedTitle);
    auto* minus = CCMenuItemSpriteExtra::create(CCSprite::createWithSpriteFrameName("GJ_arrow_01_001.png"), this, menu_selector(AIMenuPopup::onSpeed));
    minus->setScale(0.5f);
    minus->m_baseScale = 0.5f;
    minus->setTag(-1);
    minus->setPosition({left + 140.f, speedY});
    menu->addChild(minus);
    auto* plusSprite = CCSprite::createWithSpriteFrameName("GJ_arrow_01_001.png");
    plusSprite->setFlipX(true);
    auto* plus = CCMenuItemSpriteExtra::create(plusSprite, this, menu_selector(AIMenuPopup::onSpeed));
    plus->setScale(0.5f);
    plus->m_baseScale = 0.5f;
    plus->setTag(1);
    plus->setPosition({left + 210.f, speedY});
    menu->addChild(plus);
    m_speedLabel = CCLabelBMFont::create("", "bigFont.fnt");
    m_speedLabel->setScale(0.45f);
    m_speedLabel->setPosition({left + 175.f, speedY});
    m_mainLayer->addChild(m_speedLabel);

    // Memory statistics for the current level
    m_stats = CCLabelBMFont::create("", "chatFont.fnt");
    m_stats->setScale(0.5f);
    m_stats->setAnchorPoint({0.5f, 1.f});
    m_stats->setAlignment(kCCTextAlignmentCenter);
    m_stats->setPosition({kWidth / 2.f, speedY - 16.f});
    m_mainLayer->addChild(m_stats);

    float row0 = 80.f;
    float row1 = 52.f;
    float row2 = 24.f;
    addButton(menu, "Reset Level", menu_selector(AIMenuPopup::onReset), {kWidth * 0.2f, row0});
    addButton(menu, "Export Level", menu_selector(AIMenuPopup::onExport), {kWidth * 0.5f, row0});
    addButton(menu, "Import Level", menu_selector(AIMenuPopup::onImport), {kWidth * 0.8f, row0});
    addButton(menu, "Reset Shared", menu_selector(AIMenuPopup::onResetShared), {kWidth * 0.2f, row1});
    addButton(menu, "Export Shared", menu_selector(AIMenuPopup::onExportShared), {kWidth * 0.5f, row1});
    addButton(menu, "Import Shared", menu_selector(AIMenuPopup::onImportShared), {kWidth * 0.8f, row1});
    addButton(menu, "Reset All", menu_selector(AIMenuPopup::onResetAll), {kWidth * 0.2f, row2});
    addButton(menu, "Open Folder", menu_selector(AIMenuPopup::onFolder), {kWidth * 0.5f, row2});
    addButton(menu, "All Settings", menu_selector(AIMenuPopup::onSettings), {kWidth * 0.8f, row2});

    refresh();
    return true;
}

void AIMenuPopup::addToggle(CCMenu* menu, char const* label, int tag, bool value, CCPoint pos) {
    auto* toggler = CCMenuItemToggler::createWithStandardSprites(this, menu_selector(AIMenuPopup::onToggle), 0.6f);
    toggler->toggle(value);
    toggler->setTag(tag);
    toggler->setPosition(pos);
    menu->addChild(toggler);
    auto* text = CCLabelBMFont::create(label, "bigFont.fnt");
    text->setScale(0.4f);
    text->setAnchorPoint({0.f, 0.5f});
    text->setPosition({pos.x + 16.f, pos.y});
    m_mainLayer->addChild(text);
}

void AIMenuPopup::addButton(CCMenu* menu, char const* label, SEL_MenuHandler handler, CCPoint pos, float scale) {
    auto* sprite = ButtonSprite::create(label, "goldFont.fnt", "GJ_button_04.png", 0.8f);
    sprite->setScale(scale);
    auto* button = CCMenuItemSpriteExtra::create(sprite, this, handler);
    button->setPosition(pos);
    menu->addChild(button);
}

void AIMenuPopup::refresh() {
    auto& ai = AIController::get();
    m_speedLabel->setString(fmt::format("{:.2f}", ai.settings().learningSpeed).c_str());

    std::string stats;
    if (auto const* mem = ai.levelMemory()) {
        stats = fmt::format(
            "{}\nAttempts {}  |  Deaths {}  |  Completions {}\nBest {:.1f}%  |  Avg (last 20) {:.1f}%  |  Recent completion {:.0f}%\n"
            "{} learned states  |  {} trouble spots  |  {} shared patterns",
            mem->name.empty() ? mem->key : mem->name, mem->totalAttempts, mem->deaths, mem->completions,
            mem->bestProgress * 100.f, mem->averageProgress() * 100.f, mem->recentCompletionRate() * 100.f,
            mem->q.size(), mem->hotspots.size(), ai.patterns().q.size()
        );
        auto const& st = ai.status();
        if (!st.lastChange.empty()) stats += "\nLast change: " + st.lastChange;
    } else {
        stats = fmt::format("No level loaded\n{} shared patterns learned", ai.patterns().q.size());
    }
    auto const& lib = ai.strategies();
    stats += fmt::format("\nCross-level strategies: {} situations, {} strategies, learned on {} levels", lib.size(), lib.records(),
                         lib.levels.size());
    m_stats->setString(stats.c_str());
    m_stats->limitLabelWidth(kWidth - 30.f, 0.5f, 0.2f);
}

void AIMenuPopup::onToggle(CCObject* sender) {
    auto* toggler = static_cast<CCMenuItemToggler*>(sender);
    // The toggler reports its state *before* the click is applied.
    bool value = !toggler->isToggled();
    int tag = toggler->getTag();
    if (tag == kAI) AIController::get().setAIEnabled(value);
    else Mod::get()->setSettingValue<bool>(settingKey(tag), value);
    refresh();
}

void AIMenuPopup::onSpeed(CCObject* sender) {
    float delta = static_cast<float>(sender->getTag()) * 0.05f;
    double value = std::clamp(static_cast<double>(AIController::get().settings().learningSpeed + delta), 0.0, 1.0);
    value = std::round(value * 20.0) / 20.0;
    Mod::get()->setSettingValue<double>("learning-speed", value);
    refresh();
}

void AIMenuPopup::onReset(CCObject*) {
    createQuickPopup(
        "Reset Memory", "Forget everything the AI learned about <cy>this level</c>?", "Cancel", "Reset",
        [this](auto, bool confirmed) {
            if (!confirmed) return;
            AIController::get().resetLevelMemory();
            refresh();
        }
    );
}

void AIMenuPopup::onResetAll(CCObject*) {
    createQuickPopup(
        "Reset All", "Delete the AI memory for <cr>every level</c> and all shared patterns? This cannot be undone.", "Cancel", "Delete",
        [this](auto, bool confirmed) {
            if (!confirmed) return;
            AIController::get().resetAllMemory();
            refresh();
        }
    );
}

void AIMenuPopup::onResetShared(CCObject*) {
    createQuickPopup(
        "Reset Shared", "Forget the <cy>cross-level knowledge</c> (physics, timings, strategies)? Every level's own memory is kept.",
        "Cancel", "Reset",
        [this](auto, bool confirmed) {
            if (!confirmed) return;
            AIController::get().resetCrossLevelKnowledge();
            refresh();
        }
    );
}
void AIMenuPopup::onExportShared(CCObject*) { AIController::get().exportKnowledge(); }
void AIMenuPopup::onImportShared(CCObject*) { AIController::get().importKnowledge(); }
void AIMenuPopup::onExport(CCObject*) { AIController::get().exportLevelMemory(); }
void AIMenuPopup::onImport(CCObject*) { AIController::get().importLevelMemory(); }
void AIMenuPopup::onFolder(CCObject*) { AIController::get().openMemoryFolder(); }
void AIMenuPopup::onSettings(CCObject*) { openSettingsPopup(Mod::get()); }
