#include "AIMenuPopup.hpp"

#include "AIController.hpp"

#include <Geode/Geode.hpp>
#include <Geode/ui/GeodeUI.hpp>

#include <algorithm>
#include <cmath>

using namespace geode::prelude;

namespace {
enum ToggleTag { kAI = 0, kLearning, kAutoRetry, kPractice, kDebug, kPanel, kPrediction, kTrajectory, kMap, kOrbs, kAlternative, kStrategy };
constexpr float kWidth = 420.f;
constexpr float kHeight = 280.f;

char const* settingKey(int tag) {
    switch (tag) {
        case kLearning: return "learning-enabled";
        case kAutoRetry: return "auto-retry";
        case kPractice: return "practice-mode";
        case kDebug: return "show-debug";
        case kPanel: return "show-panel";
        case kPrediction: return "prediction-enabled";
        case kTrajectory: return "show-trajectory";
        case kMap: return "show-understanding";
        case kOrbs: return "show-orb-trajectories";
        case kAlternative: return "show-alternative";
        case kStrategy: return "strategic-planning";
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

    auto heading = [](CCNode* parent, char const* text, float x, float y) {
        auto* label = CCLabelBMFont::create(text, "goldFont.fnt");
        label->setScale(0.5f);
        label->setPosition({x, y});
        parent->addChild(label);
    };
    auto caption = [](CCNode* parent, char const* text, float y) {
        auto* label = CCLabelBMFont::create(text, "chatFont.fnt");
        label->limitLabelWidth(kWidth - 56.f, 0.55f, 0.4f);
        label->setPosition({kWidth / 2.f, y});
        label->setColor({190, 220, 240});
        parent->addChild(label);
    };
    char const* names[] = {"Play", "Display", "Memory"};
    for (int i = 0; i < 3; ++i) {
        m_pages[i] = CCMenu::create();
        m_pages[i]->setPosition({0.f, 0.f});
        m_mainLayer->addChild(m_pages[i]);
        auto* sprite = ButtonSprite::create(names[i], "goldFont.fnt", "GJ_button_04.png", 0.8f);
        sprite->setScale(0.65f);
        m_tabs[i] = CCMenuItemSpriteExtra::create(sprite, this, menu_selector(AIMenuPopup::onTab));
        m_tabs[i]->setTag(i);
        m_tabs[i]->setPosition({kWidth / 2.f + (i - 1) * 112.f, kHeight - 48.f});
        menu->addChild(m_tabs[i]);
    }
    auto* play = m_pages[0];
    heading(play, "CONTROLS", 108.f, 202.f);
    heading(play, "LEARNING", 310.f, 202.f);
    addToggle(play, "AI Player", kAI, s.aiEnabled, {32.f, 174.f});
    addToggle(play, "Prediction", kPrediction, s.prediction, {32.f, 143.f});
    addToggle(play, "Plan Ahead", kStrategy, s.strategicPlanning, {32.f, 112.f});
    addToggle(play, "Learning", kLearning, s.learning, {234.f, 174.f});
    addToggle(play, "Auto Retry", kAutoRetry, s.autoRetry, {234.f, 143.f});
    addToggle(play, "Practice Mode", kPractice, s.practice, {234.f, 112.f});
    heading(play, "Learning Speed", 107.f, 73.f);
    for (int direction : {-1, 1}) {
        auto* arrow = CCSprite::createWithSpriteFrameName("GJ_arrow_01_001.png");
        arrow->setFlipX(direction > 0);
        auto* button = CCMenuItemSpriteExtra::create(arrow, this, menu_selector(AIMenuPopup::onSpeed));
        button->setScale(0.45f);
        button->m_baseScale = 0.45f;
        button->setTag(direction);
        button->setPosition({300.f + direction * 43.f, 73.f});
        play->addChild(button);
    }
    m_speedLabel = CCLabelBMFont::create("", "bigFont.fnt");
    m_speedLabel->setScale(0.45f);
    m_speedLabel->setPosition({300.f, 73.f});
    play->addChild(m_speedLabel);
    caption(play, "Toggle the AI any time with your keybind (default F8).", 43.f);

    auto* display = m_pages[1];
    heading(display, "ON SCREEN", 108.f, 202.f);
    heading(display, "PATHS", 310.f, 202.f);
    addToggle(display, "Status Panel", kPanel, s.showPanel, {32.f, 174.f});
    addToggle(display, "Extra Details", kDebug, s.showDebug, {32.f, 140.f});
    addToggle(display, "Level Map", kMap, s.showUnderstanding, {32.f, 106.f});
    addToggle(display, "Trajectory", kTrajectory, s.showTrajectory, {234.f, 174.f});
    addToggle(display, "Orb Paths", kOrbs, s.showOrbs, {234.f, 140.f});
    addToggle(display, "Other Path", kAlternative, Mod::get()->getSettingValue<bool>("show-alternative"), {234.f, 106.f});
    caption(display, "Path: green clear / orange close / red very close / yellow uncertain", 65.f);
    caption(display, "More map layers are available in All Settings.", 43.f);

    auto* memory = m_pages[2];
    m_stats = CCLabelBMFont::create("", "chatFont.fnt");
    m_stats->setAnchorPoint({0.5f, 1.f});
    m_stats->setAlignment(kCCTextAlignmentCenter);
    m_stats->setPosition({kWidth / 2.f, 205.f});
    memory->addChild(m_stats);
    heading(memory, "THIS LEVEL", 110.f, 136.f);
    heading(memory, "SHARED KNOWLEDGE", 310.f, 136.f);
    addButton(memory, "Export", menu_selector(AIMenuPopup::onExport), {65.f, 108.f});
    addButton(memory, "Import", menu_selector(AIMenuPopup::onImport), {155.f, 108.f});
    addButton(memory, "Export", menu_selector(AIMenuPopup::onExportShared), {265.f, 108.f});
    addButton(memory, "Import", menu_selector(AIMenuPopup::onImportShared), {355.f, 108.f});
    addButton(memory, "Reset Level", menu_selector(AIMenuPopup::onReset), {110.f, 77.f});
    addButton(memory, "Reset Shared", menu_selector(AIMenuPopup::onResetShared), {310.f, 77.f});
    addButton(memory, "Reset All", menu_selector(AIMenuPopup::onResetAll), {kWidth / 2.f, 44.f}, 0.45f);
    addButton(menu, "Open Folder", menu_selector(AIMenuPopup::onFolder), {90.f, 18.f}, 0.45f);
    addButton(menu, "All Settings", menu_selector(AIMenuPopup::onSettings), {330.f, 18.f}, 0.45f);
    auto win = CCDirector::get()->getWinSize();
    m_mainLayer->setScale(std::min({1.f, (win.width - 24.f) / kWidth, (win.height - 16.f) / kHeight}));
    onTab(m_tabs[0]);
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
    text->limitLabelWidth(150.f, 0.4f, 0.3f);
    text->setAnchorPoint({0.f, 0.5f});
    text->setPosition({pos.x + 16.f, pos.y});
    menu->addChild(text);
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
        auto name = mem->name.empty() ? mem->key : mem->name;
        if (name.size() > 36) name = name.substr(0, 33) + "...";
        stats = fmt::format("{}\nBest {:.1f}%   /   {} attempts   /   {} completions", name,
                            mem->bestProgress * 100.f, mem->totalAttempts, mem->completions);
    } else stats = "No level loaded";
    stats += fmt::format("\n{} reusable strategies across {} levels", ai.strategies().records(), ai.strategies().levels.size());
    m_stats->setString(stats.c_str());
    m_stats->limitLabelWidth(kWidth - 48.f, 0.6f, 0.4f);
}

void AIMenuPopup::onTab(CCObject* sender) {
    int page = std::clamp(sender->getTag(), 0, 2);
    for (int i = 0; i < 3; ++i) {
        m_pages[i]->setVisible(i == page);
        m_pages[i]->setEnabled(i == page);
        m_tabs[i]->setOpacity(i == page ? 255 : 150);
    }

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
