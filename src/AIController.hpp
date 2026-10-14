#pragma once

#include "Observer.hpp"
#include "SaveWorker.hpp"

#include <aigd/Agent.hpp>
#include <aigd/Memory.hpp>
#include <aigd/Overlay.hpp>
#include <aigd/Physics.hpp>
#include <aigd/Predictor.hpp>
#include <aigd/StepClock.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

class PlayLayer;
class PlayerObject;
class GameObject;
class GJBaseGameLayer;

// Glue between Geometry Dash (hooks, settings, UI) and the Geode-independent
// learning core. All methods must be called on the main thread.
class AIController {
public:
    static AIController& get();

    struct Settings {
        bool aiEnabled = false;
        bool learning = true;
        bool autoRetry = true;
        bool practice = false;
        bool showPanel = true;
        bool showDebug = false;
        float learningSpeed = 0.5f;
        float respawnDelay = 0.5f;
        float trainingSpeed = 1.f;
        bool prediction = true;
        bool showTrajectory = true;
        bool showAlternative = false;
        float predictionHorizon = 0.75f; // seconds
        bool strategicPlanning = true;
        bool showUnderstanding = false;
        aigd::UnderstandingLayers layers;
    };

    void loadSettings();
    Settings const& settings() const { return m_settings; }

    // --- Level lifecycle (called from hooks) ---
    void onLevelInit(PlayLayer* layer);
    void onLevelQuit(PlayLayer* layer);
    void onLayerDestroyed(PlayLayer* layer);
    void beforeReset(PlayLayer* layer);
    void afterReset(PlayLayer* layer);
    void onPhysicsStep(PlayLayer* layer);
    void onPlayerDestroyed(PlayLayer* layer, PlayerObject* player, GameObject* killer);
    void onLevelComplete(PlayLayer* layer);
    void onPostUpdate(PlayLayer* layer, float dt);
    void onGameExit();

    // --- Input arbitration ---
    // True while the AI owns the jump button in this layer (user input is ignored).
    bool controlsInput(GJBaseGameLayer* layer) const;
    bool injecting() const { return m_injecting; }
    bool shouldSkipDelayedReset(PlayLayer* layer) const;

    // --- User actions ---
    void setAIEnabled(bool enabled);
    void toggleAI();
    void resetLevelMemory();
    void resetAllMemory();
    void exportLevelMemory();
    void importLevelMemory();
    // Cross-level knowledge (general knowledge + strategy library), kept
    // apart from every level's own memory.
    void resetCrossLevelKnowledge();
    void exportKnowledge();
    void importKnowledge();
    void openMemoryFolder() const;

    // --- Status for UI ---
    aigd::AgentStatus const& status() const { return m_agent.status(); }
    aigd::LevelMemory const* levelMemory() const { return m_level.get(); }
    aigd::PatternMemory const& patterns() const { return m_patterns; }
    aigd::StrategyLibrary const& strategies() const { return m_strategies; }
    aigd::Observation const& lastObservation() const { return m_lastObs; }
    bool inLevel() const { return m_layer != nullptr; }
    bool supported() const { return m_supported; }
    std::string const& unsupportedReason() const { return m_unsupportedReason; }
    std::string const& notice() const { return m_notice; }
    size_t indexedObjects() const { return m_observer.indexedObjects(); }
    aigd::Predictor const& predictor() const { return m_predictor; }
    aigd::PlanManager const& plans() const { return m_agent.plans(); }
    int32_t lastTick() const { return m_lastTick; }
    size_t obstacleCount() const { return m_obstacles.size(); }
    // Fills the trajectory overlay for the current plan; false = draw nothing.
    bool overlayGeometry(aigd::OverlayGeometry& out) const;
    // The level-understanding view (world layers around the player); false = draw nothing.
    bool understandingGeometry(aigd::UnderstandingOverlay& out) const;
    std::vector<aigd::UnderstandingOverlay::Mark> const& levelOverview() const { return m_overview; }
    uint64_t overviewVersion() const { return m_overviewVersion; }
    aigd::LevelAnalysis const& analysis() const { return m_analysis; }
    aigd::LevelMap const& levelMap() const { return m_map; }
    Observer::MapStats const& mapStats() const { return m_mapStats; }

private:
    AIController();

    std::filesystem::path levelsDir() const;
    std::filesystem::path patternsPath() const;
    std::filesystem::path strategiesPath() const;
    std::filesystem::path levelPath(std::string const& key) const;
    static std::string levelKey(PlayLayer* layer);

    void ensurePatternsLoaded();
    void applyAgentConfig();
    void applyTimeScale(bool active);
    void saveAsync();
    void saveNow();
    void press(PlayLayer* layer, bool down);
    void logAttemptQuality(aigd::GameMode mode, bool mini) const;
    void endAttemptWithoutOutcome(PlayLayer* layer);
    void manageCheckpoints(PlayLayer* layer, aigd::Observation const& obs);
    void rebuildMap(PlayLayer* layer, aigd::Observation const& obs);

    Settings m_settings;
    aigd::Agent m_agent;
    std::unique_ptr<aigd::LevelMemory> m_level;
    aigd::PatternMemory m_patterns;      // layer A: physics, timing statistics, pattern prior
    aigd::StrategyLibrary m_strategies;  // layer B: strategies per kind of situation
    bool m_patternsLoaded = false;
    Observer m_observer;
    // whole-level understanding (v0.5.0)
    aigd::LevelMap m_map;
    aigd::LevelAnalysis m_analysis;
    Observer::MapStats m_mapStats;
    std::vector<aigd::PhysicsEvent> m_events;
    std::vector<aigd::UnderstandingOverlay::Mark> m_overview;
    uint64_t m_overviewVersion = 0;
    bool m_mapReady = false;
    double m_mapMs = 0.0;
    SaveWorker m_saver;

    // trajectory prediction
    aigd::PhysicsModel m_physics;
    aigd::Predictor m_predictor{m_physics};
    std::vector<aigd::Obstacle> m_obstacles;

    PlayLayer* m_layer = nullptr;   // identity only, cleared when the layer goes away
    bool m_supported = false;
    std::string m_unsupportedReason;
    std::string m_notice;

    bool m_holding = false;
    bool m_injecting = false;
    int32_t m_lastTick = -1;
    aigd::StepClock m_clock; // real physics steps (the game does not call us on every step)
    aigd::Observation m_lastObs;

    bool m_deathPending = false;
    float m_deathTimer = 0.f;
    uint32_t m_resetToken = 0;
    uint32_t m_attemptsSinceSave = 0;

    // stall watchdog
    float m_stallX = 0.f;
    int32_t m_stallTick = 0;

    // practice-mode checkpoints placed by the AI
    std::vector<float> m_checkpointXs;
    uint32_t m_deathsSinceCheckpointProgress = 0;
    float m_bestSinceCheckpoint = 0.f;

    float m_prevTimeScale = 1.f;
    bool m_timeScaleApplied = false;
};
