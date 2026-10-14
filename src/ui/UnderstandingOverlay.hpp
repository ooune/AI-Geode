#pragma once

#include <aigd/Overlay.hpp>

#include <Geode/cocos/include/cocos2d.h>

#include <cstdint>
#include <vector>

// World-space layers of the level-understanding view (lives in the object
// layer, so it scrolls with the level). One draw node, redrawn per frame.
class UnderstandingWorld : public cocos2d::CCNode {
public:
    static UnderstandingWorld* create();
    void render(aigd::UnderstandingOverlay const& g);
    void clearAll();

protected:
    bool init() override;
    cocos2d::CCDrawNode* m_draw = nullptr;
};

// The strip with the whole level at the bottom of the screen. The level part
// is drawn once per level (it only changes when the map is rebuilt); the
// player marker and the warnings are updated per frame.
class LevelStrip : public cocos2d::CCNode {
public:
    static LevelStrip* create();
    void render(std::vector<aigd::UnderstandingOverlay::Mark> const& marks, uint64_t version, float playerFraction,
                std::vector<std::string> const& warnings, bool showStrip);
    void hideAll();

protected:
    bool init() override;
    cocos2d::CCDrawNode* m_static = nullptr;
    cocos2d::CCDrawNode* m_marker = nullptr;
    cocos2d::CCLabelBMFont* m_warnings = nullptr;
    uint64_t m_version = ~uint64_t(0);
};
