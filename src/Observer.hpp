#pragma once

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
    // Rebuilds the object index if the level's object count changed.
    void ensureIndex(PlayLayer* layer);
    aigd::Observation observe(PlayLayer* layer, int32_t tick);
    aigd::DeathInfo deathInfo(PlayLayer* layer, PlayerObject* player, GameObject* killer) const;
    size_t indexedObjects() const { return m_items.size(); }

    static aigd::GameMode modeOf(PlayerObject* player);

private:
    enum class Kind : uint8_t { Hazard, Solid, Orb };
    struct Item {
        GameObject* object;
        float x;
        Kind kind;
    };

    std::vector<Item> m_items;
    size_t m_cursor = 0;
    float m_lastX = 0.f;
    unsigned m_indexedCount = ~0u;
    PlayLayer* m_layer = nullptr;
};
