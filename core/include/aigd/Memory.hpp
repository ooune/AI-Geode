#pragma once

#include <aigd/Physics.hpp>
#include <aigd/Types.hpp>

#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace aigd {

// Action 0 = button released, 1 = button held.
inline constexpr int kActions = 2;

struct QEntry {
    float q[kActions] = {0.f, 0.f};
    uint16_t n[kActions] = {0, 0};
    // Outcome evidence per action in exactly this situation. A decision is
    // only counted as a failure when it was identified as the cause of a death
    // (not merely because it happened shortly before one), and as a pass when
    // the attempt got safely past it. `streak` counts failures since the last
    // pass, so newer evidence always overrides older evidence.
    uint16_t fails[kActions] = {0, 0};
    uint16_t passes[kActions] = {0, 0};
    uint8_t streak[kActions] = {0, 0};

    bool visited(int a) const { return n[a] > 0; }
    float best() const { return q[0] > q[1] ? q[0] : q[1]; }
    bool hasEvidence() const { return fails[0] | fails[1] | passes[0] | passes[1]; }
    void notePass(int a) {
        if (passes[a] < 0xFFFF) ++passes[a];
        streak[a] = 0;
    }
    void noteFail(int a) {
        if (fails[a] < 0xFFFF) ++fails[a];
        if (streak[a] < 0xFF) ++streak[a];
    }
};

class QTable {
public:
    QEntry const* find(uint64_t key) const {
        auto it = m_map.find(key);
        return it == m_map.end() ? nullptr : &it->second;
    }
    QEntry* find(uint64_t key) {
        auto it = m_map.find(key);
        return it == m_map.end() ? nullptr : &it->second;
    }
    QEntry& getOrCreate(uint64_t key, bool* created = nullptr) {
        auto [it, inserted] = m_map.try_emplace(key);
        if (created) *created = inserted;
        return it->second;
    }
    size_t size() const { return m_map.size(); }
    // Keeps memory bounded: drops the entries with the least experience until
    // at most `keep` remain.
    void prune(size_t keep);
    void clear() { m_map.clear(); }
    void reserve(size_t n) { m_map.reserve(n); }

    auto begin() const { return m_map.begin(); }
    auto end() const { return m_map.end(); }

private:
    std::unordered_map<uint64_t, QEntry> m_map;
};

struct AttemptRecord {
    uint32_t number = 0;
    float progress = 0.f;      // 0..1 reached
    float endX = 0.f;
    FailureKind kind = FailureKind::None;
    int32_t ticks = 0;
    bool completed = false;
    bool improved = false;     // went further than the previous attempt
    bool fromCheckpoint = false;
    std::string change;        // what the AI changed going into this attempt
};

// A place where the AI keeps dying. Buckets are one block wide.
struct Hotspot {
    int32_t bucket = 0;
    uint32_t deaths = 0;
    uint32_t passes = 0;
    FailureKind lastKind = FailureKind::None;
    int32_t shift = 0;          // current experiment shift in cells (signed)
    uint32_t experiments = 0;   // experiments tried since the last success
    uint32_t escalation = 0;    // how far back the search window has widened
};

struct PathStep {
    float x = 0.f;
    uint64_t key = 0;
    uint8_t action = 0;
};

struct TraceSample {
    float x = 0.f, y = 0.f, vy = 0.f;
    uint8_t mode = 0;
    bool onGround = false;
    uint8_t action = 0;
};

// Running statistics of a successful timing ("lead distance" between pressing
// and the obstacle that was then cleared), per game mode and speed.
struct TimingStat {
    double sum = 0.0;
    double sumSq = 0.0;
    uint32_t count = 0;

    void add(double v) {
        sum += v;
        sumSq += v * v;
        ++count;
    }
    double mean() const { return count ? sum / count : 0.0; }
    double stddev() const {
        if (count < 2) return 0.0;
        double m = mean();
        double var = sumSq / count - m * m;
        return var > 0 ? std::sqrt(var) : 0.0;
    }
};

inline uint32_t timingKey(GameMode mode, int speedClass, bool mini) {
    return static_cast<uint32_t>(mode) * 16u + static_cast<uint32_t>(speedClass) * 2u + (mini ? 1u : 0u);
}

// Everything remembered about one level.
struct LevelMemory {
    static constexpr size_t kMaxHistory = 300;
    static constexpr size_t kMaxStates = 400000;
    // 2: per-state outcome evidence (fails/passes/streak) in the q table.
    static constexpr int kFormatVersion = 2;

    std::string key;   // stable identifier, e.g. "online-128" or "main-1"
    std::string name;
    float levelLength = 0.f;

    uint32_t totalAttempts = 0;
    uint32_t completions = 0;
    uint32_t deaths = 0;
    float bestProgress = 0.f;
    float bestX = 0.f;
    uint32_t attemptsSinceBest = 0;

    std::deque<AttemptRecord> history;
    std::map<int32_t, Hotspot> hotspots;
    QTable q;

    // Inputs (x position, new button state) of the furthest attempt so far.
    // Kept for inspection/export only; the policy is always the Q-table.
    std::vector<std::pair<float, uint8_t>> bestRunInputs;
    // Decisions (state key + action) of the furthest full attempt. Kept for
    // inspection/export and to know how far the level has been solved; it is
    // never replayed or written back into the policy. What the best run did
    // well reaches later attempts only through the per-state evidence and
    // values it produced.
    std::vector<PathStep> bestPath;
    float bestPathX = 0.f;
    // State/action history leading up to the most recent death.
    std::vector<TraceSample> lastFailureTrace;

    void pushHistory(AttemptRecord rec) {
        history.push_back(std::move(rec));
        while (history.size() > kMaxHistory) history.pop_front();
    }
    float recentCompletionRate(size_t window = 20) const;
    float averageProgress(size_t window = 20) const;
};

// Knowledge that transfers between levels.
struct PatternMemory {
    static constexpr size_t kMaxStates = 200000;
    static constexpr int kFormatVersion = 2;
    QTable q;
    std::map<uint32_t, TimingStat> leads;
    PhysicsCalibration physics; // what the trajectory predictor learned about the game's physics
};

} // namespace aigd
