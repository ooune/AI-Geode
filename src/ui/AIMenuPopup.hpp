#pragma once

#include <Geode/ui/Popup.hpp>

class CCMenuItemToggler;

// The AI control panel, opened from the pause menu: toggles, learning speed,
// memory statistics and memory management (reset / export / import).
class AIMenuPopup : public geode::Popup {
public:
    static AIMenuPopup* create();

protected:
    bool init();

    void addToggle(cocos2d::CCMenu* menu, char const* label, int tag, bool value, cocos2d::CCPoint pos);
    void addButton(cocos2d::CCMenu* menu, char const* label, cocos2d::SEL_MenuHandler handler, cocos2d::CCPoint pos, float scale = 0.55f);
    void refresh();

    void onToggle(cocos2d::CCObject* sender);
    void onSpeed(cocos2d::CCObject* sender);
    void onReset(cocos2d::CCObject* sender);
    void onResetAll(cocos2d::CCObject* sender);
    void onExport(cocos2d::CCObject* sender);
    void onImport(cocos2d::CCObject* sender);
    void onFolder(cocos2d::CCObject* sender);
    void onSettings(cocos2d::CCObject* sender);

    cocos2d::CCLabelBMFont* m_speedLabel = nullptr;
    cocos2d::CCLabelBMFont* m_stats = nullptr;
};
