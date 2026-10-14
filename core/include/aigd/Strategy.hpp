#pragma once

#include <aigd/Memory.hpp>
#include <aigd/Situation.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace aigd {

// Movement primitives: the reusable kinds of input a strategy is made of.
// Each is executed through an ordinary predicted plan (so it is interruptible
// and always simulated first), never as a fixed input sequence.
enum class Primitive : uint8_t {
    Wait = 0,     // no input: the group is passed without jumping
    Jump,         // one jump, pressed `lead` steps before contact (a delayed jump has a small lead)
    RejumpChain,  // jump and keep holding through the landing (jumps again on landing)
    DoubleJump,   // jump, land, and jump again `second` steps after landing
    LongJump,     // robot: jump and hold the boost for `second` steps
    Count
};
char const* primitiveName(Primitive p);

// Where a strategy's evidence came from: one entry per level (the level's key,
// nothing about positions in it).
struct Provenance {
    std::string level;
    uint32_t successes = 0;
    uint32_t failures = 0;
};

// One way of handling one kind of situation, with its evidence. A jump
// strategy is one timing window (press lead in steps, in buckets of
// kLeadBucket): a failure counts against the timing that failed, not against
// jumping in general.
struct StrategyRecord {
    static constexpr size_t kMaxLevels = 8;
    static constexpr int kLeadBucket = 4;

    Primitive primitive = Primitive::Wait;
    int16_t leadBucket = -1;  // round(lead / kLeadBucket); -1 for Wait
    uint32_t successes = 0;
    uint32_t failures = 0;
    // Press time before contact in physics steps (successful uses). Steps,
    // not distance: the jump takes the same time at every speed, so a timing
    // learned at one speed carries over to another.
    TimingStat lead;
    TimingStat second;      // DoubleJump: steps after landing; LongJump: steps held
    uint8_t speedMask = 0;  // speed classes it worked at
    uint32_t lastUse = 0;   // library clock at the last update
    std::vector<Provenance> levels;

    double successRate() const { return (successes + 1.0) / (successes + failures + 2.0); }
    Provenance const* on(std::string const& level) const;
    int levelsWithSuccess() const;
    int levelsWithSuccessExcept(std::string const& level) const;
};

// What the library suggests for a situation on a given level.
struct StrategyChoice {
    enum class Match : uint8_t { None = 0, Exact, Similar };
    Match match = Match::None;
    bool sequence = false;        // matched with what follows the group too (the most specific key)
    bool usable = false;          // evidence is good enough to try it (always simulated / safety-checked)
    Primitive primitive = Primitive::Wait;
    float leadSteps = -1.f;       // press this many steps before contact
    float secondSteps = -1.f;
    bool speedAdapted = false;    // learned at other speeds only (the steps carry over, the distance changes)
    int learnedSpeed = -1;        // a speed it was learned at (for the message)
    bool rejectedHere = false;    // the strategy failed repeatedly on this level: not used here
    uint32_t localFailures = 0;
    uint32_t successes = 0;
    uint32_t failures = 0;
    int otherLevels = 0;          // other levels where it worked
    bool learnedHere = false;     // it has worked on this level
    bool landingDiffers = false;  // only known with another landing space
    double relevance = 0.0;
};

// Layer B: reusable strategies, keyed by situation (fine and coarse), with
// success/failure statistics and provenance. Separate from the per-level
// memory (layer C) and the physics/timing knowledge (layer A).
struct StrategyLibrary {
    static constexpr int kFormatVersion = 1;
    static constexpr size_t kMaxSituations = 40000;
    // Below this relevance a strategy is not proposed.
    static constexpr double kMinRelevance = 0.55;

    std::unordered_map<uint64_t, std::vector<StrategyRecord>> situations;
    uint32_t clock = 0;
    // Levels that contributed (provenance summary).
    std::vector<std::string> levels;

    void record(Situation const& s, Primitive p, float leadSteps, float secondSteps, bool success, std::string const& level);
    StrategyChoice choose(Situation const& s, std::string const& level) const;
    size_t size() const { return situations.size(); }
    size_t records() const;
    void clear() {
        situations.clear();
        levels.clear();
        clock = 0;
    }
    // Keeps memory bounded: drops the situations with the least evidence.
    void prune(size_t keep);

private:
    void recordInto(uint64_t key, int speed, Primitive p, float leadSteps, float secondSteps, bool success, std::string const& level);
    bool best(uint64_t key, int speed, std::string const& level, StrategyChoice& out) const;
};

// How well a strategy fits on a level: its overall success rate, reduced for
// every failure on this level beyond its successes here (negative transfer:
// a strategy from other levels that keeps failing here stops being used here,
// without losing what it learned elsewhere).
double strategyRelevance(StrategyRecord const& r, std::string const& level, bool* rejectedHere = nullptr);

// The primitive a plan corresponds to (for diagnostics and learning).
Primitive primitiveOfPlan(GameMode mode, int32_t pressAt, int32_t holdFor, bool rejump, int32_t secondAt);

} // namespace aigd
