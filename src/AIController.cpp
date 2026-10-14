#include "AIController.hpp"

#include <aigd/StateKey.hpp>
#include <aigd/Storage.hpp>

#include <Geode/Geode.hpp>
#include <Geode/ui/Notification.hpp>
#include <Geode/utils/async.hpp>
#include <Geode/utils/file.hpp>

#include <algorithm>
#include <chrono>
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
    m_agent.setPredictor(&m_predictor);
    m_obstacles.reserve(256);
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
    m_settings.prediction = mod->getSettingValue<bool>("prediction-enabled");
    m_settings.showTrajectory = mod->getSettingValue<bool>("show-trajectory");
    m_settings.showAlternative = mod->getSettingValue<bool>("show-alternative");
    m_settings.predictionHorizon = static_cast<float>(mod->getSettingValue<double>("prediction-horizon"));
    m_settings.strategicPlanning = mod->getSettingValue<bool>("strategic-planning");
    m_settings.showUnderstanding = mod->getSettingValue<bool>("show-understanding");
    m_settings.layers.overview = mod->getSettingValue<bool>("understanding-overview");
    m_settings.layers.objects = mod->getSettingValue<bool>("understanding-objects");
    m_settings.layers.groups = mod->getSettingValue<bool>("understanding-groups");
    m_settings.layers.landing = mod->getSettingValue<bool>("understanding-landing");
    m_settings.layers.plan = mod->getSettingValue<bool>("understanding-plan");
    m_settings.layers.alternatives = mod->getSettingValue<bool>("understanding-alternatives");
    m_settings.layers.uncertain = mod->getSettingValue<bool>("understanding-uncertain");
    applyAgentConfig();
}

void AIController::applyAgentConfig() {
    auto cfg = m_agent.config();
    cfg.learningSpeed = std::clamp(m_settings.learningSpeed, 0.f, 1.f);
    cfg.learningEnabled = m_settings.learning;
    cfg.usePrediction = m_settings.prediction;
    cfg.useStrategicPlanning = m_settings.strategicPlanning;
    cfg.strategicMaxMicros = 2000.f; // a sequence plan never takes much more than 2 ms of a frame
    m_agent.setConfig(cfg);
    m_predictor.setHorizon(static_cast<int>(m_settings.predictionHorizon * aigd::kTicksPerSecond));
}

// ---------------------------------------------------------------------------
// Paths and identity

std::filesystem::path AIController::levelsDir() const {
    return Mod::get()->getSaveDir() / "levels";
}

std::filesystem::path AIController::patternsPath() const {
    return Mod::get()->getSaveDir() / "patterns.json";
}

std::filesystem::path AIController::strategiesPath() const {
    return Mod::get()->getSaveDir() / "strategies.json";
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
        m_physics.loadCalibration(m_patterns.physics);
        log::info("Loaded pattern memory: {} situations, {} timing stats; physics {}", m_patterns.q.size(),
                  m_patterns.leads.size(), m_physics.describe());
    } else {
        log::info("No pattern memory yet ({})", err);
    }
    std::string serr;
    if (aigd::storage::loadStrategyFile(strategiesPath(), m_strategies, &serr)) {
        log::info("Loaded strategy library: {} situations, {} strategies, learned on {} levels", m_strategies.size(),
                  m_strategies.records(), m_strategies.levels.size());
    } else {
        log::info("No strategy library yet ({})", serr);
    }
}

// ---------------------------------------------------------------------------
// Level lifecycle

void AIController::onLevelInit(PlayLayer* layer) {
    ensurePatternsLoaded();
    if (m_layer && m_layer != layer) onLayerDestroyed(m_layer);

    m_layer = layer;
    m_lastTick = -1;
    m_clock.reset();
    m_map.clear();
    m_analysis = aigd::LevelAnalysis{};
    m_overview.clear();
    ++m_overviewVersion;
    m_mapReady = false;
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
    m_agent.attach(m_level.get(), &m_patterns, &m_strategies);
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
    m_clock.reset();
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

    auto* player = layer->m_player1;
    if (!player || player->m_isDead || layer->m_levelEndAnimationStarted || !layer->m_started) return;

    applyTimeScale(true);
    bool const reindexed = m_observer.ensureIndex(layer);
    auto obs = m_observer.observe(layer, 0);
    if (reindexed || !m_mapReady) rebuildMap(layer, obs);
    // Count the real physics steps since the last call from the horizontal
    // movement: in game this hook does not run on every physics step (1-3
    // steps per call were measured), and the level-time tick did not show
    // that. 0 steps: still the same step, nothing new to decide.
    if (m_clock.advance(obs.x, obs.speedClass()) == 0) return;
    int32_t const tick = m_clock.tick();
    obs.tick = tick;
    m_lastTick = tick;
    // Hitboxes around the player for the trajectory predictor: a little behind
    // (for re-simulating the last steps) up to the end of the prediction horizon.
    float reach = m_settings.predictionHorizon * aigd::unitsPerSecond(obs.speedClass()) * 1.1f;
    if (m_mapReady && !m_map.empty()) {
        // From the level map: wide enough for the strategic planner's
        // sequences (up to 420 steps), moving objects refreshed first.
        m_observer.refreshMap(m_map, obs.x, tick);
        float const strategicReach = 420.f * aigd::unitsPerSecond(obs.speedClass()) / aigd::kTicksPerSecond + 120.f;
        reach = std::max(reach, m_settings.strategicPlanning ? strategicReach : 0.f);
        // (gravity portals: the model of their effect has not been checked in game)
        m_map.collect(obs.x - 90.f, obs.x + reach + 60.f, m_obstacles, m_events, false);
        obs.events = m_events;
        obs.geometryVersion = m_map.version();
    } else {
        m_observer.collectObstacles(obs.x - 90.f, obs.x + reach + 60.f, m_obstacles);
    }
    obs.obstacles = m_obstacles;
    m_predictor.observe(obs, m_holding);
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
    // The input sent to the game is exactly the decision that was recorded.
    if (m_agent.status().safety.tick == tick && (m_agent.status().safety.executed != 0) != m_holding) {
        log::error("Input mismatch at tick {}: recorded {} but the button is {}", tick, m_agent.status().safety.executed ? "down" : "up",
                   m_holding ? "down" : "up");
    }
    m_lastObs = obs;
    m_lastObs.obstacles = {}; // the span is only valid for this step
    m_lastObs.events = {};

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

void AIController::rebuildMap(PlayLayer* layer, aigd::Observation const& obs) {
    auto const t0 = std::chrono::steady_clock::now();
    float const floor = m_physics.cubeFloorY(false) - 15.f; // GD's ground surface
    m_observer.buildMap(layer, m_map, floor, m_mapStats);
    aigd::AnalysisStart start;
    start.startX = obs.x;
    start.mode = obs.mode;
    start.speedClass = obs.speedClass();
    start.upsideDown = obs.upsideDown;
    start.mini = obs.mini;
    m_analysis = aigd::analyzeLevel(m_map, start, &m_strategies, m_level ? m_level->key : std::string());
    aigd::buildLevelOverview(m_map, m_analysis, m_overview);
    ++m_overviewVersion;
    m_mapReady = true;
    m_mapMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    log::info("Level map: {} of {} level objects matter for movement or survival ({} triggers move/toggle {} groups; {} without a usable "
              "hitbox); index {:.2f} ms, total {:.2f} ms",
              m_map.size(), m_mapStats.levelObjects, m_mapStats.triggers, m_mapStats.movedGroups, m_mapStats.skipped, m_map.buildMillis(), m_mapMs);
    log::info("Level analysis: {}", m_analysis.summary());
    float const len = std::max(1.f, m_analysis.finishX);
    for (size_t i = 0; i < m_analysis.difficult.size() && i < 5; ++i) {
        auto const& g = m_analysis.groups[static_cast<size_t>(m_analysis.difficult[i])];
        log::info("  Difficult: {:.1f}% {} (landing space {:.1f} blocks, headroom {})", 100.f * g.startX / len, g.sit.describe(),
                  g.landingSpace / aigd::kBlock, g.headroom > 1e8f ? std::string("open") : fmt::format("{:.1f} blocks", g.headroom / aigd::kBlock));
    }
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
    auto obs = m_observer.observe(layer, 0);
    m_clock.advance(obs.x, obs.speedClass()); // real physics steps (see onPhysicsStep)
    obs.tick = m_clock.tick();
    obs.dead = true;
    auto death = m_observer.deathInfo(layer, player, killer);
    if (m_holding) press(layer, false);
    // The safety-critical decisions right before the death, as executed.
    auto const& safetyLog = m_agent.safetyLog();
    for (size_t i = safetyLog.size() > 8 ? safetyLog.size() - 8 : 0; i < safetyLog.size(); ++i) {
        auto const& r = safetyLog[i];
        log::info("Safety t={} x={:.1f} y={:.1f} vy={:.2f} held={} | release {} (hit {}, keep {}) | press {} (hit {}, keep {}) | "
                  "proposed {} by {} -> executed {} | {}{} | {}",
                  r.tick, r.x, r.y, r.vy, r.holdingBefore, aigd::certaintyName(r.outcome[0]), r.ttc[0], r.keepFor[0],
                  aigd::certaintyName(r.outcome[1]), r.ttc[1], r.keepFor[1], r.proposed, aigd::decisionSourceName(r.proposedSource),
                  r.executed, aigd::safetyVerdictName(r.verdict), r.predictionCurrent ? "" : " (stale prediction)", r.reason);
    }
    logAttemptQuality(obs.mode, obs.mini);
    m_agent.onDeath(obs, death);
    if (!m_agent.status().predictionNote.empty()) log::info("Prediction check: {}", m_agent.status().predictionNote);
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

// One line per attempt with what the decisions were based on: how often the
// prediction was trusted (and if not, why), what the safety logic did, and
// how well the physics model matches the game. This is the evidence for
// whether predictions control the AI in game.
void AIController::logAttemptQuality(aigd::GameMode mode, bool mini) const {
    auto const& st = m_agent.status();
    auto const& model = m_predictor.model();
    log::info("Decisions: {} with a trusted prediction, {} untrusted{}{} | presses {}, presses skipped as unnecessary {}, safety overrides {}, "
              "no safe option {} | model ({}): drift {:.3f}/step, input latency {} step(s) (re-simulation error {:.2f}/{:.2f}/{:.2f} for 0/1/2, "
              "{} windows), caution {:.2f}, one-step error {:.2f}, {} estimate(s) replaced",
              st.decisionsTrusted, st.decisionsUntrusted, st.trust.empty() ? "" : " - ", st.trust, st.pressesThisAttempt,
              st.pressesAvoidedThisAttempt, st.overridesThisAttempt, st.noSafeOptionThisAttempt, aigd::gameModeName(mode),
              model.driftPerTick(mode), m_predictor.inputLatency(), m_predictor.latencyError(0), m_predictor.latencyError(1),
              m_predictor.latencyError(2), m_predictor.latencySamples(), m_predictor.caution(), m_predictor.lastStepError(),
              model.regimeChanges());
    auto const& plans = m_agent.plans();
    log::info("Plans: {:.2f} physics steps per AI call (max recently {}), x speed {:.4f}/step | {} plans made, {} invalidated, {} pending "
              "actions cancelled, {} decisions followed a plan, {} experiments cancelled{}{}",
              m_clock.averageInterval(), m_clock.recentMaxInterval(), model.vxPerTick(aigd::speedClassFromPlayerSpeed(m_lastObs.playerSpeed)),
              plans.adopted, st.plansInvalidatedThisAttempt, st.actionsCancelledThisAttempt, st.decisionsFollowingPlan, st.experimentsCancelled,
              st.planEvent.empty() ? "" : " | last: ", st.planEvent);
    auto const& sp = m_agent.strategic();
    log::info("Planning: {} sequence plans ({} chose another first press than the locally best one, {} refined by the controller), {} invalidated; "
              "controller decisions with the plan as candidate {}, executed the plan {} | sequence planning {:.0f} us per plan, {:.0f} us at most | "
              "level map: {} moving objects, {} found moving at runtime, version {}{}{}",
              st.strategicPlans, st.strategicOverrides, sp.adopted, st.strategicInvalidated, st.strategicSeeded, st.strategicMatched,
              st.strategicPlans ? st.strategicMicros / st.strategicPlans : 0.0, st.strategicMaxMicros, m_map.dynamicObjects().size(), m_map.promoted(),
              m_map.version(), st.strategicEvent.empty() ? "" : " | last: ", st.strategicEvent);
    auto const& T = st.transfer;
    log::info("Transfer: {} obstacle groups ({} known, {} similar, {} new, {} rejected after failing here, {} speed-adapted) | strategies "
              "simulated {}: executed as learned {}, replaced by the search {}; played directly {} | recorded {} passes, {} failures, {} not attributable | "
              "library {} situations from {} levels",
              T.situations, T.known, T.similar, T.unknown, T.rejectedHere, T.speedAdapted, T.seeded, T.confirmed, T.rejectedBySim, T.played,
              T.successes, T.failures, T.unattributed, m_strategies.size(), m_strategies.levels.size());
    for (auto const& msg : st.transferLog) log::info("  Transfer message: {}", msg);
    (void)mini;
}

void AIController::onLevelComplete(PlayLayer* layer) {
    if (layer != m_layer || !m_agent.attemptActive()) return;
    auto obs = m_observer.observe(layer, 0);
    m_clock.advance(obs.x, obs.speedClass()); // real physics steps (see onPhysicsStep)
    obs.tick = m_clock.tick();
    obs.x = std::max(obs.x, layer->m_levelLength);
    if (m_holding) press(layer, false);
    logAttemptQuality(obs.mode, obs.mini);
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
        m_patterns.physics = m_physics.calibration();
        auto snapshot = std::make_shared<aigd::PatternMemory>(m_patterns);
        m_saver.enqueue(patternsPath(), [snapshot] { return aigd::storage::serializePatterns(*snapshot); });
        auto lib = std::make_shared<aigd::StrategyLibrary>(m_strategies);
        m_saver.enqueue(strategiesPath(), [lib] { return aigd::storage::serializeStrategies(*lib); });
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
    m_agent.attach(m_level.get(), &m_patterns, &m_strategies);
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
    std::filesystem::remove(strategiesPath(), ec);
    std::filesystem::remove(strategiesPath().string() + ".bak", ec);
    m_patterns = aigd::PatternMemory{};
    m_strategies.clear();
    m_physics.reset();
    if (m_level) {
        aigd::LevelMemory fresh;
        fresh.key = m_level->key;
        fresh.name = m_level->name;
        fresh.levelLength = m_level->levelLength;
        *m_level = std::move(fresh);
        m_agent.resetTransient();
        m_agent.attach(m_level.get(), &m_patterns, &m_strategies);
    }
    notify("All AI memory was deleted.", NotificationIcon::Success);
}

void AIController::resetCrossLevelKnowledge() {
    if (m_layer) endAttemptWithoutOutcome(m_layer);
    m_saver.flush();
    std::error_code ec;
    for (auto const& p : {patternsPath(), strategiesPath()}) {
        std::filesystem::remove(p, ec);
        std::filesystem::remove(p.string() + ".bak", ec);
    }
    m_patterns = aigd::PatternMemory{};
    m_strategies.clear();
    m_physics.reset();
    if (m_level) {
        m_agent.resetTransient();
        m_agent.attach(m_level.get(), &m_patterns, &m_strategies);
    }
    notify("Cross-level knowledge was reset (every level's own memory is kept).", NotificationIcon::Success);
}

void AIController::exportKnowledge() {
    m_patterns.physics = m_physics.calibration();
    auto patterns = std::make_shared<aigd::PatternMemory>(m_patterns);
    auto lib = std::make_shared<aigd::StrategyLibrary>(m_strategies);
    file::FilePickOptions options;
    options.defaultPath = Mod::get()->getConfigDir() / "knowledge.aigeode.json";
    options.filters.push_back({"AI Geode knowledge", {"*.json"}});
    async::spawn(file::pick(file::PickMode::SaveFile, std::move(options)), [patterns, lib](file::PickResult result) {
        if (result.isErr()) {
            notify("Export failed: " + result.unwrapErr(), NotificationIcon::Error);
            return;
        }
        auto path = result.unwrap();
        if (!path) return;
        std::string err;
        if (aigd::storage::writeFileAtomic(*path, aigd::storage::serializeKnowledge(*patterns, *lib), &err)) {
            notify("Exported cross-level knowledge to " + path->filename().string(), NotificationIcon::Success);
        } else {
            notify("Export failed: " + err, NotificationIcon::Error);
        }
    });
}

void AIController::importKnowledge() {
    file::FilePickOptions options;
    options.filters.push_back({"AI Geode knowledge", {"*.json"}});
    async::spawn(file::pick(file::PickMode::OpenFile, std::move(options)), [this](file::PickResult result) {
        if (result.isErr()) {
            notify("Import failed: " + result.unwrapErr(), NotificationIcon::Error);
            return;
        }
        auto path = result.unwrap();
        if (!path) return;
        std::string err;
        auto text = aigd::storage::readFile(*path, &err);
        aigd::PatternMemory patterns;
        aigd::StrategyLibrary lib;
        if (!text || !aigd::storage::deserializeKnowledge(*text, patterns, lib, &err)) {
            notify("Import failed: " + err, NotificationIcon::Error);
            return;
        }
        if (m_layer) endAttemptWithoutOutcome(m_layer);
        m_saver.flush();
        m_patterns = std::move(patterns);
        m_strategies = std::move(lib);
        m_physics.loadCalibration(m_patterns.physics);
        if (m_level) {
            m_agent.resetTransient();
            m_agent.attach(m_level.get(), &m_patterns, &m_strategies);
        }
        saveNow();
        notify(fmt::format("Imported cross-level knowledge: {} strategy situations from {} levels", m_strategies.size(),
                           m_strategies.levels.size()),
               NotificationIcon::Success);
    });
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
            m_agent.attach(m_level.get(), &m_patterns, &m_strategies);
            notify("Imported memory for this level.", NotificationIcon::Success);
        } else {
            notify("Imported memory for level " + (imported.name.empty() ? imported.key : imported.name) + ".", NotificationIcon::Success);
        }
    });
}

bool AIController::understandingGeometry(aigd::UnderstandingOverlay& out) const {
    if (!m_layer || !m_supported || !m_settings.showUnderstanding || !m_mapReady) return false;
    auto const* plan = m_agent.strategic().active() ? &m_agent.strategic().plan() : nullptr;
    aigd::buildUnderstandingOverlay(&m_map, &m_analysis, plan, m_lastObs.x, m_lastObs.y, m_lastObs.x - 150.f, m_lastObs.x + 700.f,
                                    m_settings.layers, out);
    return true;
}

bool AIController::overlayGeometry(aigd::OverlayGeometry& out) const {
    if (!m_layer || !m_supported || !m_settings.aiEnabled || !m_settings.prediction || !m_settings.showTrajectory) return false;
    if (!m_agent.attemptActive()) return false;
    // The executed decision's path, labelled with what is really being done.
    return aigd::buildDecisionOverlay(m_predictor, m_agent.status().safety, m_lastTick, m_settings.showAlternative, m_obstacles, out,
                                      &m_agent.plans().plan());
}

void AIController::openMemoryFolder() const {
    std::error_code ec;
    std::filesystem::create_directories(levelsDir(), ec);
    file::openFolder(levelsDir());
}
