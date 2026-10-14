#pragma once

#include <aigd/LevelMap.hpp>
#include <aigd/Types.hpp>

#include <cstdint>
#include <vector>

class PlayLayer;
class PlayerObject;
class GameObject;

// Reads the game state the agent needs directly from Geometry Dash objects
// (PlayerObject fields and the level's GameObjects) - no screen capture.
//
// Nearby obstacles come from an index of hazard/solid/orb objects sorted by
// their x position at level load. Objects moved later by move triggers are
// still found but their sort position may be stale; the lookup window is padded
// to tolerate small movements (documented limitation).
class Observer {
public:
    void reset();
    // Rebuilds the object index if the level's object count changed (returns
    // true when it did).
    bool ensureIndex(PlayLayer* layer);
    aigd::Observation observe(PlayLayer* layer, int32_t tick);
    aigd::DeathInfo deathInfo(PlayLayer* layer, PlayerObject* player, GameObject* killer) const;
    // Hitboxes of hazards, solids and unmodelled objects (orbs, pads, portals,
    // speed changes, slopes) between x0 and x1, sorted by left edge. Uses the
    // game's own collision rectangles (GameObject::getObjectRect), so spike
    // hitboxes are the real, smaller-than-the-artwork ones.
    void collectObstacles(float x0, float x1, std::vector<aigd::Obstacle>& out) const;
    size_t indexedObjects() const { return m_items.size(); }

    // ---- Whole-level map (v0.5.0) ----
    // Every loaded object that matters for movement or survival, with the
    // game's collision rectangle (GameObject::getObjectRect, not the artwork),
    // what it does, and whether a trigger moves or toggles it (its groups are
    // targeted by a move / rotate / toggle / follow / animate / area trigger).
    struct MapStats {
        size_t levelObjects = 0;   // all objects of the level
        size_t triggers = 0;       // moving/toggling triggers found
        size_t movedGroups = 0;    // groups they target
        size_t skipped = 0;        // relevant objects without a usable rectangle
    };
    void buildMap(PlayLayer* layer, aigd::LevelMap& map, float floorSurface, MapStats& stats);
    // Moving/toggled objects near x are re-read every step (all of them every
    // 8 elapsed steps); every 30 elapsed steps up to 64 nearby static objects
    // and 64 objects in a whole-level sweep are checked for unexpected changes.
    void refreshMap(aigd::LevelMap& map, float x, int32_t tick);

    static aigd::GameMode modeOf(PlayerObject* player);

private:
    static constexpr size_t kMaxObstacles = 256;
    enum class Kind : uint8_t { Hazard, Solid, Orb, Other };
    struct Item {
        GameObject* object;
        float x;
        Kind kind;
        bool unmodelled;
    };

    std::vector<Item> m_items;
    std::vector<GameObject*> m_mapSources; // MapObject::source -> game object
    size_t m_mapAuditCursor = 0;
    int32_t m_lastMapRefresh = -1;
    int32_t m_lastMapAudit = -1;
    size_t m_cursor = 0;
    float m_lastX = 0.f;
    unsigned m_indexedCount = ~0u;
    PlayLayer* m_layer = nullptr;
};
