#include "AIController.hpp"

#include <Geode/Geode.hpp>
#include <Geode/loader/SettingV3.hpp>

using namespace geode::prelude;

$on_mod(Loaded) {
    for (auto const* key : {"ai-enabled", "learning-enabled", "auto-retry", "practice-mode", "show-panel", "show-debug",
                            "prediction-enabled", "show-trajectory", "show-alternative"}) {
        listenForSettingChanges<bool>(key, [](bool) { AIController::get().loadSettings(); });
    }
    for (auto const* key : {"learning-speed", "respawn-delay", "training-speed", "prediction-horizon"}) {
        listenForSettingChanges<double>(key, [](double) { AIController::get().loadSettings(); });
    }
    // Emergency stop / start: works at any time, releases the button immediately.
    listenForKeybindSettingPresses("toggle-ai", [](Keybind const&, bool down, bool repeat, double) {
        if (!down || repeat) return false;
        AIController::get().toggleAI();
        return true;
    });
    AIController::get().loadSettings();
    log::info("AI Geode loaded (AI player {})", AIController::get().settings().aiEnabled ? "on" : "off");
}
