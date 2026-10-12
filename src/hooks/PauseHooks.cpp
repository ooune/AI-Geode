#include "ui/AIMenuPopup.hpp"

#include <Geode/Geode.hpp>
#include <Geode/modify/PauseLayer.hpp>

using namespace geode::prelude;

class $modify(AIPauseLayer, PauseLayer) {
    void customSetup() {
        PauseLayer::customSetup();

        auto* label = CCLabelBMFont::create("AI", "bigFont.fnt");
        auto* sprite = CircleButtonSprite::create(label, CircleBaseColor::Green, CircleBaseSize::Small);
        auto* button = CCMenuItemSpriteExtra::create(sprite, this, menu_selector(AIPauseLayer::onAIMenu));
        button->setID("ai-menu-button"_spr);

        if (auto* menu = this->getChildByID("right-button-menu")) {
            menu->addChild(button);
            menu->updateLayout();
        } else {
            // Fallback if node IDs are unavailable: our own menu on the right edge.
            auto win = CCDirector::get()->getWinSize();
            auto* own = CCMenu::create();
            own->setPosition({win.width - 30.f, win.height / 2.f});
            own->addChild(button);
            this->addChild(own, 10);
        }
    }

    void onAIMenu(CCObject*) {
        if (auto* popup = AIMenuPopup::create()) popup->show();
    }
};
