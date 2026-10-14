#pragma once

#include <aigd/LevelMap.hpp>
#include <aigd/Situation.hpp>
#include <aigd/Strategy.hpp>
#include <aigd/Types.hpp>

#include <string>
#include <vector>

namespace aigd {

// One obstacle group found in the level geometry (before playing it).
struct ObstacleGroup {
    float startX = 0.f, endX = 0.f;  // world x
    float surface = 0.f;             // walking surface assumed in front of it (floor or a block top)
    Situation sit;                   // relative description (the strategy library's keys)
    int hazards = 0, solids = 0, above = 0;
    float gapBefore = 0.f;           // free path since the previous group (units)
    float landingSpace = 0.f;        // free path after it (units)
    float headroom = 1e9f;           // room above the walking surface over the group (units; 1e9 = open)
    bool unmodelled = false;         // an orb, pad or untracked portal in or right after it
    bool dynamic = false;            // contains objects moved/toggled by triggers
    bool approximate = false;        // contains rotated hitboxes (bounding boxes)
    int section = 0;
    float difficulty = 0.f;          // relative: more elements, less landing space, low headroom, unknowns
    StrategyChoice::Match strategy = StrategyChoice::Match::None; // what the library has for it
};

// A portal: where the physics change.
struct PhysicsChange {
    float x = 0.f;
    MapKind kind = MapKind::SpeedPortal;
    int value = 0;
};

// A stretch of the level with one mode, speed and gravity (as far as the
// portals tell; -1 = not known from the geometry).
struct MapSection {
    float startX = 0.f, endX = 0.f;
    int mode = -1;
    int speedClass = -1;
    int gravity = -1;          // 0 normal, 1 flipped
    bool mini = false;
    int firstGroup = 0, groups = 0;
    // Flying modes: the smallest vertical gap found between obstacles (units),
    // and how many narrow passages (< 2.5 blocks) there are.
    float minVerticalRoom = 1e9f;
    int narrowPassages = 0;
    bool analysedGroups = false; // obstacle groups are only described for cube / robot with normal gravity
};

// What the AI knows about the whole level before (and while) playing it.
struct LevelAnalysis {
    bool valid = false;
    float startX = 0.f, finishX = 0.f;
    size_t objects = 0, hazards = 0, solids = 0, portals = 0, unmodelled = 0, dynamicObjects = 0, approximate = 0;
    std::vector<MapSection> sections;
    std::vector<PhysicsChange> changes;
    std::vector<ObstacleGroup> groups;
    std::vector<int> tight;      // groups with less than 2 blocks of landing space
    std::vector<int> difficult;  // the hardest groups (up to 10, hardest first)
    int knownGroups = 0, similarGroups = 0;
    double millis = 0.0;

    // First group whose end is at or after x (groups.size() if none).
    size_t groupAt(float x) const;
    std::string summary() const;
};

struct AnalysisStart {
    GameMode mode = GameMode::Cube;
    int speedClass = 1;
    bool upsideDown = false;
    bool mini = false;
    float startX = 0.f;
};

// Analyses the geometry of the whole map. Nothing is simulated here: groups,
// landing space and headroom are geometric (walking surface assumed from the
// blocks a jump could reach). Movement is checked by the predictor while
// playing.
LevelAnalysis analyzeLevel(LevelMap const& map, AnalysisStart const& start, StrategyLibrary const* library = nullptr,
                           std::string const& level = {});

} // namespace aigd
