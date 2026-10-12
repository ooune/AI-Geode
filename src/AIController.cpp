#include "AIController.hpp"

#include <aigd/StateKey.hpp>
#include <aigd/Storage.hpp>

#include <Geode/Geode.hpp>
#include <Geode/ui/Notification.hpp>
#include <Geode/utils/async.hpp>
#include <Geode/utils/file.hpp>

#include <algorithm>
#include <cmath>

using namespace geode::prelude;

namespace {
constexpr uint32_t kSaveEveryAttempts = 10;
constexpr int32_t kStallTicks = aigd::kTicksPerSecond * 5;
constexpr uint32_t kCheckpointGiveUpDeaths = 15;

uint64_t fnv1a(char const* data, size_t size, uint64_t h = 1469598103934665603ull) {
    for (size_t i = 0; i < size; ++i) {
        h ^= static_cast<unsigned char>(data[i]);
        h *= 1099511628211ull;
    }
    return h;
}

void notify(std::string const& text, NotificationIcon icon = NotificationIcon::Info) {
    Notification::create(text, icon)->show();
}
} // namespace

AIController& AIController::get() {
    static AIController instance;
    return instance;
}

AIController::AIController() {
    loadSettings();
}

void AIController::loadSettings() {
    auto* mod = Mod::get();
    m_settings.aiEnabled = mod->getSettingValue<bool>("ai-enabled");
    m_settings.learning = mod->getSettingValue<bool>("learning-enabled");
    m_settings.autoRetry = mod->getSettingValue<bool>("auto-retry");
    m_settings.practice = mod->getSettingValue<bool>("practice-mode");
    m_settings.showPanel = mod->getSettingValue<bool>("show-panel");
    m_settings.showDebug = mod->getSettingValue<bool>("show-debug");
    m_settings.learningSpeed = static_cast<float>(mod->getSettingValue<double>("learning-speed"));
    m_settings.respawnDelay = static_cast<float>(mod->getSettingValue<double>("respawn-delay"));
    m_settings.trainingSpeed = static_cast<float>(mod->getSettingValue<double>("training-speed"));
    applyAgentConfig();
}

void AIController::applyAgentConfig() {
    auto cfg = m_agent.config();
    cfg.learningSpeed = std::clamp(m_settings.learningSpeed, 0.f, 1.f);
    cfg.learningEnabled = m_settings.learning;
    m_agent.setConfig(cfg);
}

// ---------------------------------------------------------------------------
// Paths and identity

std::filesystem::path AIController::levelsDir() const {
    return Mod::get()->getSaveDir() / "levels";
}

std::filesystem::path AIController::patternsPath() const {
    return Mod::get()->getSaveDir() / "patterns.json";
}

std::filesystem::path AIController::levelPath(std::string const& key) const {
    return levelsDir() / (aigd::storage::sanitizeFileName(key) + ".json");
}

std::string AIController::levelKey(PlayLayer* layer) {
    auto* level = layer->m_level;
    int id = level->m_levelID.value();
    if (level->m_levelType == GJLevelType::Main) return fmt::format("main-{}", id);
    if (level->m_levelType != GJLevelType::Editor && id > 0) return fmt::format("online-{}", id);
    // Local/editor levels have no stable id: identify them by name + content,
    // so editing the layout starts a fresh memory instead of reusing stale timings.
    std::string name = level->m_levelName;
    std::string data = level->m_levelString;
    uint64_t h = fnv1a(name.data(), name.size());
    h = fnv1a(data.data(), data.size(), h);
    return fmt::format("local-{:016x}", h);
}

void AIController::ensurePatternsLoaded() {
    if (m_patternsLoaded) return;
    m_patternsLoaded = true;
    std::string err;
    if (aigd::storage::loadPatternFile(patternsPath(), m_patterns, &err)) {
        log::info("Loaded pattern memory: {} situations, {} timing stats", m_patterns.q.size(), m_patterns.leads.size());
    } else {
        log::info("No pattern memory yet ({})", err);
    }
}

int32_t AIController::currentTick(PlayLayer* layer) const {
    return static_cast<int32_t>(layer->m_gameState.m_levelTime * aigd::kTicksPerSecond) + 1;
}

// ---------------------------------------------------------------------------
// Level lifecycle

void AIController::onLevelInit(PlayLayer* layer) {
    ensurePatternsLoaded();
    if (m_layer && m_layer != layer) onLayerDestroyed(m_layer);

    m_layer = layer;
    m_lastTick = -1;
    m_holding = false;
    m_deathPending = false;
    m_observer.reset();
    m_checkpointXs.clear();
    m_notice.clear();

    m_supported = true;
    m_unsupportedReason.clear();
    if (layer->m_levelSettings && layer->m_levelSettings->m_platformerMode) {
        m_supported = false;
        m_unsupportedReason = "Platformer levels need left/right control, which this AI does not learn.";
    } else if (layer->m_levelSettings && layer->m_levelSettings->m_twoPlayerMode) {
        m_notice = "2-player level: the AI only controls player 1.";
    }

    std::string key = levelKey(layer);
    if (!m_level || m_level->key != key) {
        if (m_level) saveNow();
        m_saver.flush();
        auto mem = std::make_unique<aigd::LevelMemory>();
        std::string err;
        if (aigd::storage::loadLevelFile(levelPath(key), *mem, &err)) {
            log::info("Loaded memory for {}: {} attempts, {} completions, {} states", key, mem->totalAttempts, mem->completions, mem->q.size());
        } else {
            *mem = aigd::LevelMemory{};
            mem->key = key;
            log::info("Starting new memory for {} ({})", key, err);
        }
        m_level = std::move(mem);
    }
    m_level->name = std::string(layer->m_level->m_levelName);
    m_agent.attach(m_level.get(), &m_patterns);
    applyAgentConfig();
    m_agent.setPhase(m_settings.aiEnabled && m_supported ? aigd::AgentPhase::Observing : aigd::AgentPhase::Disabled);

    if (m_settings.aiEnabled && m_settings.practice && m_supported) {
        queueInMainThread([this, layer] {
            if (PlayLayer::get() != layer || m_layer != layer) return;
            if (!layer->m_isPracticeMode) layer->togglePracticeMode(true);
        });
    }
}

void AIController::onLevelQuit(PlayLayer* layer) {
    if (layer != m_layer) return;
    endAttemptWithoutOutcome(layer);
    saveNow();
    applyTimeScale(false);
    m_agent.detach();
    m_observer.reset();
    m_layer = nullptr;
}

void AIController::onLayerDestroyed(PlayLayer* layer) {
    if (layer != m_layer) return;
    // The layer is going away without onQuit (e.g. scene replaced): never touch it again.
    if (m_agent.attemptActive()) m_agent.abortAttempt(nullptr);
    saveAsync();
    applyTimeScale(false);
    m_agent.detach();
    m_observer.reset();
    m_layer = nullptr;
    m_holding = false;
}

void AIController::onGameExit() {
    if (m_level) saveNow();
}

void AIController::beforeReset(PlayLayer* layer) {
    if (layer != m_layer) return;
    if (m_holding) press(layer, false);
}

void AIController::afterReset(PlayLayer* layer) {
    if (layer != m_layer) return;
    // A restart in the middle of an attempt (user pressed restart) carries no
    // outcome to learn from.
    if (m_agent.attemptActive()) m_agent.abortAttempt(nullptr);
    m_deathPending = false;
    ++m_resetToken;
    m_lastTick = -1;
    m_holding = false;
    if (!layer->m_isPracticeMode) m_checkpointXs.clear();
    if (layer->m_checkpointArray) {
        size_t n = layer->m_checkpointArray->count();
        if (m_checkpointXs.size() > n) m_checkpointXs.resize(n);
    }
    if (m_settings.aiEnabled && m_supported) m_agent.setPhase(aigd::AgentPhase::Observing);
}

void AIController::press(PlayLayer* layer, bool down) {
    m_injecting = true;
    layer->handleButton(down, 1, true);
    m_injecting = false;
    m_holding = down;
}

void AIController::endAttemptWithoutOutcome(PlayLayer* layer) {
    if (m_agent.attemptActive()) m_agent.abortAttempt(nullptr);
    if (m_holding && layer) press(layer, false);
    m_holding = false;
}

void AIController::onPhysicsStep(PlayLayer* layer) {
    if (layer != m_layer || !m_supported || !m_level) return;

    if (!m_settings.aiEnabled) {
        if (m_agent.attemptActive() || m_holding) endAttemptWithoutOutcome(layer);
        m_agent.setPhase(aigd::AgentPhase::Disabled);
        applyTimeScale(false);
        return;
    }

    int32_t tick = currentTick(layer);
    if (tick == m_lastTick) return; // several sub-steps can share one physics tick
    m_lastTick = tick;

    auto* player = layer->m_player1;
    if (!player || player->m_isDead || layer->m_levelEndAnimationStarted || !layer->m_started) return;

    applyTimeScale(true);
    m_observer.ensureIndex(layer);
    auto obs = m_observer.observe(layer, tick);
    if (m_level->levelLength < obs.levelLength) m_level->levelLength = obs.levelLength;

    if (!m_agent.attemptActive()) {
        bool fromCheckpoint = layer->m_isPracticeMode && layer->m_checkpointArray && layer->m_checkpointArray->count() > 0;
        if (m_holding) press(layer, false);
        m_agent.beginAttempt(obs, fromCheckpoint);
        m_stallX = obs.x;
        m_stallTick = tick;
    }

    bool hold = m_agent.act(obs);
    if (hold != m_holding) press(layer, hold);
    m_lastObs = obs;

    // Watchdog: classic levels always move right; if nothing happens for a
    // while (stuck object, odd trigger setup) restart instead of hanging.
    if (obs.x > m_stallX + 1.f) {
        m_stallX = obs.x;
        m_stallTick = tick;
    } else if (tick - m_stallTick > kStallTicks) {
        log::warn("No horizontal progress for 5 seconds at x={:.1f}; restarting", obs.x);
        endAttemptWithoutOutcome(layer);
        m_stallTick = tick;
        queueInMainThread([this, layer] {
            if (PlayLayer::get() == layer && m_layer == layer) layer->resetLevel();
        });
        return;
    }

    if (m_settings.practice) manageCheckpoints(layer, obs);
}

void AIController::manageCheckpoints(PlayLayer* layer, aigd::Observation const& obs) {
    if (!layer->m_isPracticeMode || !layer->m_uiLayer) return;
    float speedScale = aigd::unitsPerSecond(obs.speedClass()) / aigd::unitsPerSecond(1);
    float spacing = 6.f * aigd::kBlock * speedScale;
    float last = m_checkpointXs.empty() ? 0.f : m_checkpointXs.back();
    bool calm = obs.onGround || aigd::isFlyingMode(obs.mode);
    bool clear = !(obs.hazard.present && obs.hazard.dx < 2.f * aigd::kBlock);
    // Only checkpoint ground the AI has already survived past at least once.
    bool known = obs.x + aigd::kBlock < m_level->bestX;
    if (obs.x > last + spacing && calm && clear && known) {
        layer->m_uiLayer->onCheck(nullptr);
        m_checkpointXs.push_back(obs.x);
        m_deathsSinceCheckpointProgress = 0;
        m_bestSinceCheckpoint = obs.x;
        log::debug("Placed checkpoint at x={:.1f}", obs.x);
    }
}

void AIController::onPlayerDestroyed(PlayLayer* layer, PlayerObject* player, GameObject* killer) {
    if (layer != m_layer || !m_agent.attemptActive()) return;
    auto obs = m_observer.observe(layer, currentTick(layer));
    obs.dead = true;
    auto death = m_observer.deathInfo(layer, player, killer);
    if (m_holding) press(layer, false);
    m_agent.onDeath(obs, death);
    m_deathPending = true;
    m_deathTimer = 0.f;
    // lastChange is still the change that was tested in this attempt; the plan
    // for the next attempt becomes visible when the next attempt starts.
    auto const& rec = m_level->history.back();
    log::info("Attempt {}: {} | tested: {}", rec.number, m_agent.status().lastFailure, rec.change.empty() ? "-" : rec.change);

    if (layer->m_isPracticeMode && !m_checkpointXs.empty()) {
        if (death.x > m_bestSinceCheckpoint + aigd::kBlock) {
            m_bestSinceCheckpoint = death.x;
            m_deathsSinceCheckpointProgress = 0;
        } else if (++m_deathsSinceCheckpointProgress >= kCheckpointGiveUpDeaths && layer->m_uiLayer) {
            // The section after this checkpoint looks unwinnable from where the
            // checkpoint put us: go back to the previous one.
            layer->m_uiLayer->onDeleteCheck(nullptr);
            m_checkpointXs.pop_back();
            m_deathsSinceCheckpointProgress = 0;
            m_bestSinceCheckpoint = m_checkpointXs.empty() ? 0.f : m_checkpointXs.back();
            log::info("Removed a checkpoint after {} deaths without progress", kCheckpointGiveUpDeaths);
        }
    }

    if (++m_attemptsSinceSave >= kSaveEveryAttempts) saveAsync();
}

void AIController::onLevelComplete(PlayLayer* layer) {
    if (layer != m_layer || !m_agent.attemptActive()) return;
    auto obs = m_observer.observe(layer, currentTick(layer));
    obs.x = std::max(obs.x, layer->m_levelLength);
    if (m_holding) press(layer, false);
    m_agent.onComplete(obs);
    m_deathPending = false;
    log::info("Level completed on attempt {} ({} completions so far)", m_agent.status().attempt, m_level->completions);
    saveAsync();
}

void AIController::onPostUpdate(PlayLayer* layer, float dt) {
    if (layer != m_layer || !m_deathPending) return;
    if (!m_settings.aiEnabled || !m_settings.autoRetry) return;
    m_deathTimer += dt;
    if (m_deathTimer < m_settings.respawnDelay) return;
    m_deathPending = false;
    m_agent.setPhase(aigd::AgentPhase::Retrying);
    uint32_t token = m_resetToken;
    // Reset outside of the game's update loop.
    queueInMainThread([this, layer, token] {
        if (PlayLayer::get() != layer || m_layer != layer || token != m_resetToken) return;
        if (!layer->m_player1 || !layer->m_player1->m_isDead) return; // the game already respawned
        layer->resetLevel();
    });
}

bool AIController::controlsInput(GJBaseGameLayer* layer) const {
    return m_settings.aiEnabled && m_supported && m_layer && static_cast<GJBaseGameLayer*>(m_layer) == layer;
}

bool AIController::shouldSkipDelayedReset(PlayLayer* layer) const {
    // The AI already restarted after the death; the game's own delayed reset
    // would otherwise cut the next attempt short.
    return layer == m_layer && m_settings.aiEnabled && m_settings.autoRetry && layer->m_player1 && !layer->m_player1->m_isDead;
}

void AIController::applyTimeScale(bool active) {
    auto* scheduler = CCDirector::get()->getScheduler();
    if (active && m_settings.trainingSpeed > 1.001f) {
        if (!m_timeScaleApplied) {
            m_prevTimeScale = scheduler->getTimeScale();
            m_timeScaleApplied = true;
        }
        if (std::fabs(scheduler->getTimeScale() - m_settings.trainingSpeed) > 1e-4f) scheduler->setTimeScale(m_settings.trainingSpeed);
    } else if (m_timeScaleApplied) {
        scheduler->setTimeScale(m_prevTimeScale);
        m_timeScaleApplied = false;
    }
}

// ---------------------------------------------------------------------------
// Persistence

void AIController::saveAsync() {
    m_attemptsSinceSave = 0;
    if (m_level && !m_level->key.empty()) {
        auto snapshot = std::make_shared<aigd::LevelMemory>(*m_level);
        m_saver.enqueue(levelPath(snapshot->key), [snapshot] { return aigd::storage::serializeLevel(*snapshot); });
    }
    if (m_patternsLoaded) {
        auto snapshot = std::make_shared<aigd::PatternMemory>(m_patterns);
        m_saver.enqueue(patternsPath(), [snapshot] { return aigd::storage::serializePatterns(*snapshot); });
    }
}

void AIController::saveNow() {
    saveAsync();
    m_saver.flush();
}

// ---------------------------------------------------------------------------
// User actions

void AIController::setAIEnabled(bool enabled) {
    Mod::get()->setSettingValue<bool>("ai-enabled", enabled);
    m_settings.aiEnabled = enabled;
    if (!enabled && m_layer) {
        endAttemptWithoutOutcome(m_layer);
        applyTimeScale(false);
        m_agent.setPhase(aigd::AgentPhase::Disabled);
    } else if (enabled && m_layer && m_supported) {
        m_agent.setPhase(aigd::AgentPhase::Observing);
    }
}

void AIController::toggleAI() {
    setAIEnabled(!m_settings.aiEnabled);
    notify(m_settings.aiEnabled ? "AI player ON" : "AI player OFF", m_settings.aiEnabled ? NotificationIcon::Success : NotificationIcon::Info);
}

void AIController::resetLevelMemory() {
    if (!m_level) {
        notify("Open a level first to reset its memory.", NotificationIcon::Warning);
        return;
    }
    if (m_layer) endAttemptWithoutOutcome(m_layer);
    aigd::LevelMemory fresh;
    fresh.key = m_level->key;
    fresh.name = m_level->name;
    fresh.levelLength = m_level->levelLength;
    *m_level = std::move(fresh);
    m_agent.resetTransient();
    m_agent.attach(m_level.get(), &m_patterns);
    saveNow();
    notify("AI memory for this level was reset.", NotificationIcon::Success);
}

void AIController::resetAllMemory() {
    if (m_layer) endAttemptWithoutOutcome(m_layer);
    m_saver.flush();
    std::error_code ec;
    std::filesystem::remove_all(levelsDir(), ec);
    std::filesystem::remove(patternsPath(), ec);
    std::filesystem::remove(patternsPath().string() + ".bak", ec);
    m_patterns = aigd::PatternMemory{};
    if (m_level) {
        aigd::LevelMemory fresh;
        fresh.key = m_level->key;
        fresh.name = m_level->name;
        fresh.levelLength = m_level->levelLength;
        *m_level = std::move(fresh);
        m_agent.resetTransient();
        m_agent.attach(m_level.get(), &m_patterns);
    }
    notify("All AI memory was deleted.", NotificationIcon::Success);
}

void AIController::exportLevelMemory() {
    if (!m_level) {
        notify("Open a level first to export its memory.", NotificationIcon::Warning);
        return;
    }
    auto snapshot = std::make_shared<aigd::LevelMemory>(*m_level);
    file::FilePickOptions options;
    options.defaultPath = Mod::get()->getConfigDir() / (aigd::storage::sanitizeFileName(snapshot->key) + ".aigeode.json");
    options.filters.push_back({"AI Geode memory", {"*.json"}});
    async::spawn(file::pick(file::PickMode::SaveFile, std::move(options)), [snapshot](file::PickResult result) {
        if (result.isErr()) {
            notify("Export failed: " + result.unwrapErr(), NotificationIcon::Error);
            return;
        }
        auto path = result.unwrap();
        if (!path) return; // cancelled
        std::string err;
        if (aigd::storage::writeFileAtomic(*path, aigd::storage::serializeLevel(*snapshot), &err)) {
            notify("Exported AI memory to " + path->filename().string(), NotificationIcon::Success);
        } else {
            notify("Export failed: " + err, NotificationIcon::Error);
        }
    });
}

void AIController::importLevelMemory() {
    file::FilePickOptions options;
    options.filters.push_back({"AI Geode memory", {"*.json"}});
    async::spawn(file::pick(file::PickMode::OpenFile, std::move(options)), [this](file::PickResult result) {
        if (result.isErr()) {
            notify("Import failed: " + result.unwrapErr(), NotificationIcon::Error);
            return;
        }
        auto path = result.unwrap();
        if (!path) return;
        std::string err;
        auto text = aigd::storage::readFile(*path, &err);
        aigd::LevelMemory imported;
        if (!text || !aigd::storage::deserializeLevel(*text, imported, &err)) {
            notify("Import failed: " + err, NotificationIcon::Error);
            return;
        }
        m_saver.flush();
        if (!aigd::storage::writeFileAtomic(levelPath(imported.key), *text, &err)) {
            notify("Import failed: " + err, NotificationIcon::Error);
            return;
        }
        if (m_level && m_level->key == imported.key) {
            if (m_layer) endAttemptWithoutOutcome(m_layer);
            *m_level = std::move(imported);
            m_agent.resetTransient();
            m_agent.attach(m_level.get(), &m_patterns);
            notify("Imported memory for this level.", NotificationIcon::Success);
        } else {
            notify("Imported memory for level " + (imported.name.empty() ? imported.key : imported.name) + ".", NotificationIcon::Success);
        }
    });
}

void AIController::openMemoryFolder() const {
    std::error_code ec;
    std::filesystem::create_directories(levelsDir(), ec);
    file::openFolder(levelsDir());
}
