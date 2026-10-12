#pragma once

#include <aigd/Overlay.hpp>

#include <Geode/cocos/base_nodes/CCNode.h>
#include <Geode/cocos/draw_nodes/CCDrawNode.h>
#include <Geode/cocos/label_nodes/CCLabelBMFont.h>

// Draws the AI's predicted trajectory inside the level (it is added to the
// level's object layer, so it uses world coordinates and scrolls with the
// camera). One CCDrawNode is cleared and refilled per frame from geometry built
// by the learning core; nothing is allocated per segment.
class TrajectoryOverlay : public cocos2d::CCNode {
public:
    static TrajectoryOverlay* create();
    void render(aigd::OverlayGeometry const& geometry);
    void clearAll();

protected:
    bool init() override;

    cocos2d::CCDrawNode* m_draw = nullptr;
    cocos2d::CCLabelBMFont* m_label = nullptr;
};
