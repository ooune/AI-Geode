// Geometry Dash hooks. They only forward events to AIController; no learning
// logic lives here.

#include "AIController.hpp"
#include "ui/StatusPanel.hpp"

#include <Geode/Geode.hpp>
#include <Geode/modify/AppDelegate.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>

using namespace geode::prelude;

class $modify(AIBaseGameLayer, GJBaseGameLayer) {
    // Runs once per physics sub-step. The AI reads the state after the game has
    // processed its own commands and decides the button for the next step,
    // which is the same place established Geode bots inject their inputs.
    void processCommands(float dt, bool isHalfTick, bool isLastTick) {
        GJBaseGameLayer::processCommands(dt, isHalfTick, isLastTick);
        auto* playLayer = PlayLayer::get();
        if (playLayer && static_cast<GJBaseGameLayer*>(playLayer) == this) {
            AIController::get().onPhysicsStep(playLayer);
        }
    }

    // While the AI plays, the jump button belongs to it: user presses are
    // ignored so the two never fight over the input.
    void handleButton(bool down, int button, bool isPlayer1) {
        auto& ai = AIController::get();
        if (button == 1 && !ai.injecting() && ai.controlsInput(this)) return;
        GJBaseGameLayer::handleButton(down, button, isPlayer1);
    }
};

class $modify(AIPlayLayer, PlayLayer) {
    struct Fields {
        PlayLayer* self = nullptr;
        StatusPanel* panel = nullptr;
        float uiTimer = 0.f;

        ~Fields() {
            // Destroyed together with the PlayLayer: make sure nothing keeps a
            // pointer to it (only used for identity, never dereferenced here).
            if (self) AIController::get().onLayerDestroyed(self);
        }
    };

    bool init(GJGameLevel* level, bool useReplay, bool dontCreateObjects) {
        if (!PlayLayer::init(level, useReplay, dontCreateObjects)) return false;
        m_fields->self = this;
        AIController::get().onLevelInit(this);

        auto* panel = StatusPanel::create();
        panel->setID("status-panel"_spr);
        this->addChild(panel, 1000);
        m_fields->panel = panel;
        panel->refresh();
        return true;
    }

    void resetLevel() {
        AIController::get().beforeReset(this);
        PlayLayer::resetLevel();
        AIController::get().afterReset(this);
    }

    void destroyPlayer(PlayerObject* player, GameObject* object) {
        bool wasDead = player && player->m_isDead;
        PlayLayer::destroyPlayer(player, object);
        // GD calls this with an "anticheat spike" at level start; ignore that
        // and anything that did not actually kill the player (e.g. noclip).
        if (player && !wasDead && player->m_isDead && object != m_anticheatSpike) {
            AIController::get().onPlayerDestroyed(this, player, object);
        }
    }

    void levelComplete() {
        AIController::get().onLevelComplete(this);
        PlayLayer::levelComplete();
    }

    void delayedResetLevel() {
        if (AIController::get().shouldSkipDelayedReset(this)) return;
        PlayLayer::delayedResetLevel();
    }

    void onQuit() {
        AIController::get().onLevelQuit(this);
        m_fields->self = nullptr;
        PlayLayer::onQuit();
    }

    void postUpdate(float dt) {
        PlayLayer::postUpdate(dt);
        AIController::get().onPostUpdate(this, dt);
        auto& f = m_fields;
        f->uiTimer += dt;
        if (f->panel && f->uiTimer >= 0.1f) {
            f->uiTimer = 0.f;
            f->panel->refresh();
        }
    }
};

class $modify(AIAppDelegate, AppDelegate) {
    // Called when the game saves (including on exit): flush AI memory too.
    void trySaveGame(bool force) {
        AIController::get().onGameExit();
        AppDelegate::trySaveGame(force);
    }
};
