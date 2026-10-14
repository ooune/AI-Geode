#pragma once

#include <aigd/Types.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace aigd {

// What a level object is, as far as movement and survival are concerned.
enum class MapKind : uint8_t {
    Hazard = 0,     // kills on contact
    Solid,          // can be stood on; kills from the side
    Platform,       // one-way (passable) solid: not simulated
    Slope,          // not simulated
    Orb,            // acts only when pressed inside it: not simulated
    Pad,            // launches the player: not simulated
    SpeedPortal,    // value = speed class
    GravityPortal,  // value: 0 normal, 1 flipped, 2 toggle
    ModePortal,     // value = GameMode
    SizePortal,     // value: 0 regular, 1 mini
    OtherPortal,    // dual, teleport, ...: not simulated
    Count
};
char const* mapKindName(MapKind k);

// One object of the loaded level: the game's collision rectangle (not the
// artwork), what it does, and whether it can move.
struct MapObject {
    float l = 0.f, b = 0.f, r = 0.f, t = 0.f;
    MapKind kind = MapKind::Hazard;
    uint8_t value = 0;
    bool dynamic = false;   // moved/toggled by triggers (or seen moving): its rectangle is refreshed while near
    bool enabled = true;    // toggled off objects are kept but ignored
    bool approximate = false; // rotated by a non-right angle: the box is the bounding box of the real hitbox
    uint32_t source = 0;    // index in the game's object list (to refresh dynamic objects)
};

// A spatial representation of the whole loaded level.
//  - Static objects are indexed once (sorted by left edge, plus a grid of
//    120-unit columns), so a window query never scans the level.
//  - Dynamic objects are kept apart; their rectangles are updated through
//    updateObject() (the game side refreshes those near the player) and each
//    update bumps version(), so cached results can tell they are stale.
class LevelMap {
public:
    static constexpr float kColumn = 120.f;

    void build(std::vector<MapObject> objects, float levelLength, float floorSurface);
    void clear();
    bool empty() const { return m_objs.empty(); }

    // Moves / toggles a dynamic object (by index into objects()). A static
    // object that turns out to move is promoted to dynamic.
    void updateObject(uint32_t index, float l, float b, float r, float t, bool enabled);

    // Indices of enabled objects whose rectangle overlaps [x0, x1] (any
    // height), sorted by left edge. `out` is reused (no allocation once grown).
    void query(float x0, float x1, std::vector<uint32_t>& out) const;
    // The obstacles and physics changes in [x0, x1], in the form the
    // predictor uses. Speed and gravity portals become events; everything
    // else that is not simulated becomes an Uncertain obstacle.
    // `gravityVerified`: whether the gravity-portal model has been checked
    // against the game (see PhysicsEvent::verified).
    void collect(float x0, float x1, std::vector<Obstacle>& obstacles, std::vector<PhysicsEvent>& events,
                 bool gravityVerified) const;

    std::vector<MapObject> const& objects() const { return m_objs; }
    std::vector<uint32_t> const& dynamicObjects() const { return m_dynamic; }
    MapObject const& at(uint32_t i) const { return m_objs[i]; }
    size_t size() const { return m_objs.size(); }
    size_t count(MapKind k) const { return m_counts[static_cast<size_t>(k)]; }
    float length() const { return m_length; }
    float floorSurface() const { return m_floor; }
    uint64_t version() const { return m_version; }
    uint32_t promoted() const { return m_promoted; }   // static objects found moving at runtime
    double buildMillis() const { return m_buildMs; }

private:
    std::vector<MapObject> m_objs;      // static objects first (sorted by l), dynamic ones after
    size_t m_staticCount = 0;
    std::vector<std::vector<uint32_t>> m_columns; // static objects overlapping each column
    std::vector<uint32_t> m_dynamic;
    std::vector<uint8_t> m_isDynamic;   // per object (promotion moves nothing in memory)
    size_t m_counts[static_cast<size_t>(MapKind::Count)] = {};
    float m_length = 0.f;
    float m_floor = 0.f;
    uint64_t m_version = 0;
    uint32_t m_promoted = 0;
    double m_buildMs = 0.0;
};

} // namespace aigd
