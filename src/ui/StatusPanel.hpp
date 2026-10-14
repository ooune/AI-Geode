#pragma once
#include <string>

#include <Geode/cocos/base_nodes/CCNode.h>
#include <Geode/cocos/label_nodes/CCLabelBMFont.h>
#include <Geode/cocos/layers_scenes_transitions_nodes/CCLayer.h>

// Compact in-level overlay showing what the AI is doing and what it learned.
class StatusPanel : public cocos2d::CCNode {
public:
    static StatusPanel* create();
    void refresh();

protected:
    bool init() override;

    std::string m_text;
    cocos2d::CCLayerColor* m_background = nullptr;
    cocos2d::CCLabelBMFont* m_label = nullptr;
};
