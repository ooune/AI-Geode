#define _CRT_SECURE_NO_WARNINGS
// Unit and behaviour tests for the AI-Geode learning core.
//
//   aigd_tests            run all tests
//   aigd_tests --bench    also run the learning benchmark (several seeds/configs)
//   aigd_tests <filter>   run tests whose name contains <filter>

#include "Sim.hpp"

#include <aigd/Agent.hpp>
#include <aigd/FailureAnalyzer.hpp>
#include <aigd/Overlay.hpp>
#include <aigd/Physics.hpp>
#include <aigd/Predictor.hpp>
#include <aigd/Json.hpp>
#include <aigd/StateKey.hpp>
#include <aigd/StepClock.hpp>
#include <aigd/Storage.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <vector>

using namespace aigd;

// ---------------------------------------------------------------------------
// Tiny test harness

namespace {
struct TestCase {
    char const* name;
    std::function<void()> fn;
};
std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}
struct Registrar {
    Registrar(char const* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};
int g_failures = 0;
int g_checks = 0;
std::string g_current;

#define TEST(name)                                         \
    static void test_##name();                             \
    static Registrar reg_##name(#name, test_##name);       \
    static void test_##name()

#define CHECK(cond)                                                                       \
    do {                                                                                  \
        ++g_checks;                                                                       \
        if (!(cond)) {                                                                    \
            ++g_failures;                                                                 \
            std::printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);               \
        }                                                                                 \
    } while (0)

#define CHECK_MSG(cond, ...)                                                              \
    do {                                                                                  \
        ++g_checks;                                                                       \
        if (!(cond)) {                                                                    \
            ++g_failures;                                                                 \
            std::printf("    FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond);             \
            std::printf(__VA_ARGS__);                                                     \
            std::printf("\n");                                                            \
        }                                                                                 \
    } while (0)

// ---------------------------------------------------------------------------
// Training helpers

struct AttemptResult {
    float progress;
    float endX;
    bool completed;
};

// Trajectory prediction attached to a training run, fed exactly like the mod
// feeds it in game: obstacles around the player + one observation per step.
struct PredictionRig {
    PhysicsModel model;
    Predictor predictor{model};
    std::vector<Obstacle> obstacles;
    bool held = false;

    Observation observe(sim::World const& world) {
        Observation o = world.observe();
        world.collectObstacles(o.x - 90.f, o.x + 480.f, obstacles);
        o.obstacles = obstacles;
        return o;
    }
};

AttemptResult runAttempt(Agent& agent, sim::Level const& level, PredictionRig* rig = nullptr, int speedClass = 1,
                         int maxTicks = 100000) {
    sim::World world(level, speedClass);
    agent.beginAttempt(rig ? rig->observe(world) : world.observe(), false);
    if (rig) rig->held = false;
    for (int t = 0; t < maxTicks; ++t) {
        Observation obs = rig ? rig->observe(world) : world.observe();
        if (rig) rig->predictor.observe(obs, rig->held);
        bool hold = agent.act(obs);
        if (rig) rig->held = hold;
        world.step(hold);
        if (world.dead()) {
            Observation last = world.observe();
            agent.onDeath(last, world.death());
            return {last.progress(), world.x(), false};
        }
        if (world.complete()) {
            Observation last = world.observe();
            agent.onComplete(last);
            return {1.f, world.x(), true};
        }
    }
    agent.abortAttempt(nullptr);
    return {world.observe().progress(), world.x(), false};
}

struct TrainResult {
    int attemptsToComplete = -1; // -1 = never
    std::vector<AttemptResult> attempts;
};

TrainResult train(Agent& agent, sim::Level const& level, int maxAttempts, PredictionRig* rig = nullptr) {
    TrainResult r;
    for (int i = 0; i < maxAttempts; ++i) {
        auto a = runAttempt(agent, level, rig);
        r.attempts.push_back(a);
        if (a.completed) {
            r.attemptsToComplete = i + 1;
            break;
        }
    }
    return r;
}

LevelMemory freshLevel(sim::Level const& level) {
    LevelMemory m;
    m.key = "sim-" + level.name;
    m.name = level.name;
    m.levelLength = level.length;
    return m;
}

// Breadth-first search over "on the ground at tick t" states: proves that a cube
// level can be completed at all (so learning tests are meaningful).
bool cubeLevelSolvable(sim::Level const& level) {
    std::queue<sim::World> open;
    std::set<std::pair<int, int>> seen;
    open.emplace(level);
    while (!open.empty()) {
        sim::World w = open.front();
        open.pop();
        auto o = w.observe();
        auto key = std::make_pair(o.tick, static_cast<int>(o.y));
        if (!seen.insert(key).second) continue;
        // option 1: wait a tick
        {
            sim::World n = w;
            n.step(false);
            if (n.complete()) return true;
            if (!n.dead()) open.push(n);
        }
        // option 2: tap jump, then release until landing
        if (o.onGround) {
            sim::World n = w;
            n.step(true);
            for (int i = 0; i < 400 && !n.dead() && !n.complete() && !n.observe().onGround; ++i) n.step(false);
            if (n.complete()) return true;
            if (!n.dead()) open.push(n);
        }
    }
    return false;
}

std::filesystem::path tempDir() {
    auto p = std::filesystem::temp_directory_path() / "aigd_tests";
    std::filesystem::create_directories(p);
    return p;
}
} // namespace

// ---------------------------------------------------------------------------
// JSON

TEST(json_roundtrip) {
    auto v = json::parse(R"({"a":[1,2.5,-3e2,true,false,null],"s":"he\"llo\né","big":9007199254740991})");
    CHECK(v.has_value());
    CHECK(v->get("a").size() == 6);
    CHECK(v->get("a").at(1).asNumber() == 2.5);
    CHECK(v->get("a").at(2).asNumber() == -300.0);
    CHECK(v->get("a").at(3).asBool() == true);
    CHECK(v->get("s").asString() == "he\"llo\n\xc3\xa9");
    CHECK(v->get("big").asInt() == 9007199254740991LL);
    auto again = json::parse(json::write(*v));
    CHECK(again.has_value());
    CHECK(json::write(*again) == json::write(*v));
}

TEST(json_rejects_garbage) {
    std::string err;
    CHECK(!json::parse("{\"a\":", &err).has_value());
    CHECK(!err.empty());
    CHECK(!json::parse("[1,2,]").has_value());
    CHECK(!json::parse("{} trailing").has_value());
    CHECK(!json::parse("").has_value());
    std::string deep(200, '[');
    CHECK(!json::parse(deep).has_value());
}

// ---------------------------------------------------------------------------
// State keys

TEST(state_keys) {
    Observation o;
    o.x = 300.f;
    o.y = 15.f;
    o.onGround = true;
    uint64_t k1 = encodeLevelKey(o, false, 3.f);
    CHECK(k1 == encodeLevelKey(o, false, 3.f));
    CHECK(k1 <= kKeyMask);
    CHECK(keyCell(k1) == 100);
    CHECK(keyMode(k1) == GameMode::Cube);
    CHECK(k1 != encodeLevelKey(o, true, 3.f));

    Observation o2 = o;
    o2.x = 303.f;
    CHECK(encodeLevelKey(o2, false, 3.f) != k1);
    // pattern keys ignore absolute position
    CHECK(encodePatternKey(o, false) == encodePatternKey(o2, false));
    o2.hazard = {true, 45.f, 0.f};
    CHECK(encodePatternKey(o, false) != encodePatternKey(o2, false));
    // mode and speed separate knowledge
    Observation o3 = o;
    o3.mode = GameMode::Ship;
    CHECK(encodePatternKey(o3, false) != encodePatternKey(o, false));
    o3 = o;
    o3.playerSpeed = 1.1f;
    CHECK(encodePatternKey(o3, false) != encodePatternKey(o, false));
    // huge coordinates stay inside the 53-bit range
    o3.x = 1e9f;
    o3.y = 1e9f;
    CHECK(encodeLevelKey(o3, true, 3.f) <= kKeyMask);
}

// ---------------------------------------------------------------------------
// Failure analysis

namespace {
std::vector<Step> makeSteps(std::initializer_list<std::tuple<float, int32_t, uint8_t>> rows) {
    std::vector<Step> v;
    for (auto const& [x, tick, a] : rows) {
        Step s;
        s.x = x;
        s.tick = tick;
        s.action = a;
        s.onGround = true; // presses happen from the ground (i.e. they are effective)
        v.push_back(s);
    }
    return v;
}
} // namespace

TEST(failure_analysis) {
    Observation last;
    last.levelLength = 3000.f;
    last.x = 600.f;
    last.tick = 460;
    DeathInfo d;
    d.x = 600.f;
    d.hasKiller = true;
    d.killer = KillerKind::Hazard;
    d.killerDx = 12.f;
    d.killerDy = -5.f;

    // Never pressed -> missing jump
    {
        auto steps = makeSteps({{300.f, 230, 0}, {450.f, 345, 0}, {590.f, 452, 0}});
        last.onGround = true;
        last.vy = 0.f;
        auto r = analyzeFailure(steps, last, d, 0);
        CHECK(r.kind == FailureKind::NoJump);
        CHECK(r.hotspot == 20);
    }
    // Pressed recently and still rising -> too late
    {
        auto steps = makeSteps({{300.f, 230, 0}, {570.f, 437, 1}, {574.f, 440, 0}});
        last.onGround = false;
        last.vy = 6.f;
        auto r = analyzeFailure(steps, last, d, 0);
        CHECK(r.kind == FailureKind::TooLate);
        CHECK(r.responsibleBlock == 0);
    }
    // Pressed, now falling onto the hazard -> too early
    {
        auto steps = makeSteps({{300.f, 230, 0}, {500.f, 384, 1}, {504.f, 387, 0}});
        last.onGround = false;
        last.vy = -6.f;
        auto r = analyzeFailure(steps, last, d, 0);
        CHECK(r.kind == FailureKind::TooEarly);
    }
    // Upside down: "rising" is measured relative to gravity
    {
        auto steps = makeSteps({{300.f, 230, 0}, {570.f, 437, 1}, {574.f, 440, 0}});
        last.onGround = false;
        last.upsideDown = true;
        last.vy = -6.f;
        auto r = analyzeFailure(steps, last, d, 0);
        CHECK(r.kind == FailureKind::TooLate);
        last.upsideDown = false;
    }
    // Ship: obstacle above -> held too long; below -> released too long
    {
        auto steps = makeSteps({{300.f, 230, 1}, {590.f, 452, 1}});
        last.mode = GameMode::Ship;
        d.killerDy = 25.f;
        CHECK(analyzeFailure(steps, last, d, 0).kind == FailureKind::HeldTooLong);
        d.killerDy = -25.f;
        CHECK(analyzeFailure(steps, last, d, 0).kind == FailureKind::ReleasedTooLong);
        d.killerDy = 0.f;
        CHECK(analyzeFailure(steps, last, d, 0).kind == FailureKind::HitWall);
        // Edges beat centres: a floor pillar whose centre is above a low ship
        // is not evidence of holding too long.
        d.hasExtent = true;
        d.killerDy = 25.f;
        d.killerTop = 60.f;
        d.killerBottom = -15.f;
        CHECK(analyzeFailure(steps, last, d, 0).kind == FailureKind::HitWall);
        d.killerTop = 90.f;
        d.killerBottom = 10.f;
        CHECK(analyzeFailure(steps, last, d, 0).kind == FailureKind::HeldTooLong);
        d.killerTop = -10.f;
        d.killerBottom = -40.f;
        CHECK(analyzeFailure(steps, last, d, 0).kind == FailureKind::ReleasedTooLong);
        d.hasExtent = false;
        last.mode = GameMode::Cube;
    }
    // Escalation points at an earlier press, not the last one
    {
        auto steps = makeSteps({{420.f, 322, 1}, {424.f, 325, 0}, {560.f, 430, 1}, {564.f, 433, 0}});
        last.onGround = false;
        last.vy = -6.f;
        d.killerDy = -5.f;
        auto r0 = analyzeFailure(steps, last, d, 0);
        auto r1 = analyzeFailure(steps, last, d, 1);
        CHECK(r0.responsibleBlock == 1);
        CHECK(r1.responsibleBlock == 0);
    }
}

// ---------------------------------------------------------------------------
// Q-learning update

TEST(q_update_penalises_death_and_respects_learning_toggle) {
    sim::Level level = sim::levelBasics();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg;
    cfg.useExperiments = false;
    Agent agent(cfg);
    agent.attach(&mem, &pat);

    auto r = runAttempt(agent, level);
    CHECK(!r.completed);
    CHECK(mem.deaths == 1);
    CHECK(mem.q.size() > 0);
    // The decision right before death must now look bad for the action taken.
    auto const& steps = agent.steps();
    CHECK(!steps.empty());
    auto const* e = mem.q.find(steps.back().key);
    CHECK(e != nullptr);
    if (e) CHECK(e->q[steps.back().action] < 0.f);
    // Decisions far from the death keep a positive value (progress reward).
    auto const* first = mem.q.find(steps.front().key);
    CHECK(first != nullptr);
    if (first) CHECK(first->q[steps.front().action] > 0.f);

    // With learning disabled nothing in memory changes.
    size_t before = mem.q.size();
    std::string snapshot = storage::serializeLevel(mem);
    cfg.learningEnabled = false;
    agent.setConfig(cfg);
    runAttempt(agent, level);
    CHECK(mem.q.size() == before);
    LevelMemory copy;
    CHECK(storage::deserializeLevel(snapshot, copy));
    // Only the attempt statistics move; the policy is untouched.
    for (auto const& [k, ent] : copy.q) {
        auto const* now = mem.q.find(k);
        CHECK(now != nullptr);
        if (now) CHECK(now->q[0] == ent.q[0] && now->q[1] == ent.q[1]);
    }
}

// ---------------------------------------------------------------------------
// Storage

TEST(storage_roundtrip_and_recovery) {
    LevelMemory m;
    m.key = "online-12345";
    m.name = "Test \"Level\"";
    m.levelLength = 9000.f;
    m.totalAttempts = 42;
    m.completions = 2;
    m.deaths = 40;
    m.bestProgress = 0.75f;
    m.bestX = 6750.f;
    m.attemptsSinceBest = 3;
    AttemptRecord rec;
    rec.number = 42;
    rec.progress = 0.5f;
    rec.endX = 4500.f;
    rec.kind = FailureKind::TooEarly;
    rec.ticks = 1234;
    rec.improved = true;
    rec.change = "Press moved 6 units later";
    m.pushHistory(rec);
    Hotspot h;
    h.bucket = 150;
    h.deaths = 7;
    h.passes = 1;
    h.lastKind = FailureKind::TooLate;
    h.shift = -3;
    h.experiments = 2;
    h.escalation = 1;
    m.hotspots[150] = h;
    m.q.getOrCreate(123456789012345ULL) = QEntry{{1.5f, -2.25f}, {3, 4}};
    m.q.getOrCreate(kKeyMask) = QEntry{{0.1f, 0.2f}, {65535, 0}};
    m.bestRunInputs = {{100.f, 1}, {104.f, 0}};
    m.lastFailureTrace = {{1.f, 2.f, 3.f, 1, true, 1}};

    std::string text = storage::serializeLevel(m);
    LevelMemory r;
    std::string err;
    CHECK_MSG(storage::deserializeLevel(text, r, &err), "%s", err.c_str());
    CHECK(r.key == m.key);
    CHECK(r.name == m.name);
    CHECK(r.totalAttempts == 42 && r.completions == 2 && r.deaths == 40);
    CHECK(std::fabs(r.bestProgress - 0.75f) < 1e-6f);
    CHECK(r.history.size() == 1 && r.history[0].change == rec.change && r.history[0].kind == FailureKind::TooEarly);
    CHECK(r.hotspots.size() == 1 && r.hotspots[150].shift == -3 && r.hotspots[150].escalation == 1);
    CHECK(r.q.size() == 2);
    auto const* e = r.q.find(123456789012345ULL);
    CHECK(e && e->q[0] == 1.5f && e->q[1] == -2.25f && e->n[0] == 3 && e->n[1] == 4);
    auto const* e2 = r.q.find(kKeyMask);
    CHECK(e2 && e2->n[0] == 65535);
    CHECK(r.bestRunInputs.size() == 2);
    CHECK(r.lastFailureTrace.size() == 1 && r.lastFailureTrace[0].onGround);

    // files: atomic write + .bak recovery
    auto dir = tempDir();
    auto path = dir / "level.json";
    std::filesystem::remove(path);
    std::filesystem::remove(path.string() + ".bak");
    CHECK(storage::writeFileAtomic(path, text, &err));
    m.totalAttempts = 43;
    CHECK(storage::writeFileAtomic(path, storage::serializeLevel(m), &err));
    // corrupt the main file: loader must fall back to the previous version
    CHECK(storage::writeFileAtomic(path, "{ not json", &err));
    // writeFileAtomic copied the valid 43-attempt file into .bak before corrupting
    LevelMemory loaded;
    CHECK(storage::loadLevelFile(path, loaded, &err));
    CHECK(loaded.totalAttempts == 43);
    // garbage everywhere -> failure, output untouched
    CHECK(storage::writeFileAtomic(path.string() + ".bak", "garbage", &err));
    LevelMemory untouched;
    untouched.key = "keep";
    CHECK(!storage::loadLevelFile(path, untouched, &err));
    CHECK(untouched.key == "keep");
    // wrong document type is rejected
    CHECK(!storage::deserializeLevel(R"({"format":"something-else"})", untouched, &err));

    PatternMemory p;
    p.q.getOrCreate(77) = QEntry{{0.5f, 0.25f}, {1, 2}};
    p.leads[timingKey(GameMode::Cube, 1, false)].add(60.0);
    p.leads[timingKey(GameMode::Cube, 1, false)].add(64.0);
    PatternMemory p2;
    CHECK(storage::deserializePatterns(storage::serializePatterns(p), p2, &err));
    CHECK(p2.q.size() == 1 && p2.leads.size() == 1);
    CHECK(std::fabs(p2.leads.begin()->second.mean() - 62.0) < 1e-9);

    CHECK(storage::sanitizeFileName("online-1/..\\x") == "online-1____x");
}

// ---------------------------------------------------------------------------
// Simulator sanity

TEST(sim_cube_levels_are_solvable) {
    CHECK(cubeLevelSolvable(sim::levelBasics()));
    CHECK(cubeLevelSolvable(sim::levelSpikes()));
    CHECK(cubeLevelSolvable(sim::levelBlocks()));
    // and the agent is not given a free win: doing nothing dies
    sim::Level l = sim::levelBasics();
    sim::World w(l);
    while (!w.dead() && !w.complete()) w.step(false);
    CHECK(w.dead());
}

// ---------------------------------------------------------------------------
// Learning behaviour

TEST(agent_changes_strategy_after_failure) {
    sim::Level level = sim::levelBasics();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg;
    cfg.experimentAfterDeaths = 1; // exercise the failure-analysis path immediately
    Agent agent(cfg);
    agent.attach(&mem, &pat);

    runAttempt(agent, level);
    std::vector<uint8_t> first;
    for (auto const& s : agent.steps()) first.push_back(s.action);
    CHECK(!agent.status().lastFailure.empty());
    CHECK(agent.pendingExperiment().active);
    CHECK(!agent.pendingExperiment().description.empty());

    runAttempt(agent, level);
    std::vector<uint8_t> second;
    for (auto const& s : agent.steps()) second.push_back(s.action);
    CHECK(first != second);
    CHECK(!agent.status().lastChange.empty());
}

namespace {
void expectLearns(sim::Level const& level, int budget, LearnerConfig cfg = {}) {
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(cfg);
    agent.attach(&mem, &pat);
    auto r = train(agent, level, budget);
    std::printf("    %-8s completed after %d attempts (best %.1f%%)\n", level.name.c_str(), r.attemptsToComplete, mem.bestProgress * 100.f);
    CHECK_MSG(r.attemptsToComplete > 0, "level %s not completed within %d attempts", level.name.c_str(), budget);
    // best progress never decreases and ends at 100 %
    CHECK(mem.bestProgress == 1.f);
    // measurable improvement: later attempts get further than early ones
    if (r.attempts.size() >= 20) {
        auto avg = [&](size_t from, size_t to) {
            float s = 0.f;
            for (size_t i = from; i < to; ++i) s += r.attempts[i].progress;
            return s / static_cast<float>(to - from);
        };
        size_t n = r.attempts.size();
        float early = avg(0, 10);
        float late = avg(n - 10, n);
        CHECK_MSG(late > early, "early %.3f late %.3f", early, late);
    }
}
} // namespace

TEST(agent_learns_basics) { expectLearns(sim::levelBasics(), 400); }
TEST(agent_learns_spikes) { expectLearns(sim::levelSpikes(), 800); }
TEST(agent_learns_blocks) { expectLearns(sim::levelBlocks(), 1200); }
TEST(agent_learns_ship) { expectLearns(sim::levelShip(), 1500); }

// Once something works it must keep working: after the first completion, keep
// training (learning on, exploration on) and require the AI to stay consistent,
// and never to fall far behind its best in the meantime.
TEST(agent_does_not_regress) {
    for (auto const& level : sim::allLevels()) {
        LevelMemory mem = freshLevel(level);
        PatternMemory pat;
        Agent agent;
        agent.attach(&mem, &pat);
        auto r = train(agent, level, 1500);
        CHECK(r.attemptsToComplete > 0);
        // Regressions while learning: attempts ending more than 3 blocks
        // before the best already reached at that time.
        int regressions = 0;
        float best = 0.f;
        for (auto const& a : r.attempts) {
            if (a.endX < best - 3.f * kBlock) ++regressions;
            best = std::max(best, a.endX);
        }
        int completed = 0;
        for (int i = 0; i < 20; ++i) completed += runAttempt(agent, level).completed ? 1 : 0;
        std::printf("    %-8s %d regressions in %zu attempts; %d/20 completions after first success\n", level.name.c_str(),
                    regressions, r.attempts.size(), completed);
        CHECK_MSG(completed >= 18, "%s only completed %d/20 after learning it", level.name.c_str(), completed);
        CHECK_MSG(regressions * 4 <= static_cast<int>(r.attempts.size()), "%s regressed in %d of %zu attempts", level.name.c_str(),
                  regressions, r.attempts.size());
    }
}

TEST(learned_policy_survives_save_and_reload) {
    sim::Level level = sim::levelSpikes();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    {
        Agent agent;
        agent.attach(&mem, &pat);
        auto r = train(agent, level, 800);
        CHECK(r.attemptsToComplete > 0);
    }
    auto dir = tempDir();
    std::string err;
    CHECK(storage::writeFileAtomic(dir / "spikes.json", storage::serializeLevel(mem), &err));
    CHECK(storage::writeFileAtomic(dir / "patterns.json", storage::serializePatterns(pat), &err));

    // "Restart the game": brand new agent and memory loaded from disk.
    LevelMemory loaded;
    PatternMemory loadedPat;
    CHECK(storage::loadLevelFile(dir / "spikes.json", loaded, &err));
    CHECK(storage::loadPatternFile(dir / "patterns.json", loadedPat, &err));
    CHECK(loaded.q.size() == mem.q.size());

    LearnerConfig cfg;
    cfg.learningEnabled = false; // pure replay of the learned policy
    Agent fresh(cfg);
    fresh.attach(&loaded, &loadedPat);
    auto a = runAttempt(fresh, level);
    CHECK_MSG(a.completed, "reloaded policy reached %.1f%%", a.progress * 100.f);

    // An untrained agent does not complete it.
    LevelMemory empty = freshLevel(level);
    PatternMemory emptyPat;
    Agent untrained(cfg);
    untrained.attach(&empty, &emptyPat);
    CHECK(!runAttempt(untrained, level).completed);
}

TEST(learning_disabled_repeats_identically) {
    sim::Level level = sim::levelSpikes();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg;
    cfg.learningEnabled = false;
    Agent agent(cfg);
    agent.attach(&mem, &pat);
    auto a = runAttempt(agent, level);
    auto b = runAttempt(agent, level);
    CHECK(a.endX == b.endX);
    CHECK(mem.q.size() == 0);
}

TEST(pattern_knowledge_transfers_between_levels) {
    // Train on one level, then learn a different level with the shared pattern
    // memory vs. from scratch. Reported, and must not be dramatically worse.
    PatternMemory shared;
    {
        sim::Level a = sim::levelSpikes();
        LevelMemory mem = freshLevel(a);
        Agent agent;
        agent.attach(&mem, &shared);
        train(agent, a, 800);
    }
    sim::Level b = sim::levelBasics();
    LevelMemory m1 = freshLevel(b), m2 = freshLevel(b);
    PatternMemory none;
    Agent withPrior, withoutPrior;
    withPrior.attach(&m1, &shared);
    withoutPrior.attach(&m2, &none);
    auto r1 = train(withPrior, b, 400);
    auto r2 = train(withoutPrior, b, 400);
    std::printf("    basics: %d attempts with transferred patterns, %d from scratch\n", r1.attemptsToComplete, r2.attemptsToComplete);
    CHECK(r1.attemptsToComplete > 0);
    CHECK(r2.attemptsToComplete > 0);
    CHECK(r1.attemptsToComplete <= r2.attemptsToComplete * 2 + 5);
}

TEST(per_tick_cost_is_small) {
    sim::Level level = sim::levelSpikes();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent;
    agent.attach(&mem, &pat);
    train(agent, level, 50);

    sim::World world(level);
    agent.beginAttempt(world.observe(), false);
    auto t0 = std::chrono::steady_clock::now();
    int calls = 0;
    while (!world.dead() && !world.complete() && calls < 100000) {
        world.step(agent.act(world.observe()));
        ++calls;
    }
    auto t1 = std::chrono::steady_clock::now();
    double usPerTick = std::chrono::duration<double, std::micro>(t1 - t0).count() / std::max(calls, 1);

    auto t2 = std::chrono::steady_clock::now();
    if (world.dead()) agent.onDeath(world.observe(), world.death());
    else agent.onComplete(world.observe());
    auto t3 = std::chrono::steady_clock::now();
    double msUpdate = std::chrono::duration<double, std::milli>(t3 - t2).count();

    auto t4 = std::chrono::steady_clock::now();
    std::string blob = storage::serializeLevel(mem);
    auto t5 = std::chrono::steady_clock::now();
    std::printf(
        "    %.2f us per tick (observe+decide, sim included), %.3f ms end-of-attempt update over %zu decisions, "
        "%.2f ms to serialise %zu Q entries (%zu bytes)\n",
        usPerTick, msUpdate, agent.steps().size(),
        std::chrono::duration<double, std::milli>(t5 - t4).count(), mem.q.size(), blob.size()
    );
    CHECK(usPerTick < 50.0);
    CHECK(msUpdate < 20.0);
}

// ---------------------------------------------------------------------------
// Trajectory prediction

namespace {
// Calibrates the physics model the way the first seconds of a real attempt
// would: flat ground with a jump every 150 steps.
void warmUp(PredictionRig& rig, int speedClass = 1) {
    sim::Level flat;
    flat.name = "flat";
    flat.finish(400);
    sim::World w(flat, speedClass);
    rig.held = false;
    for (int t = 0; t < 2400 && !w.complete(); ++t) {
        Observation o = rig.observe(w);
        rig.predictor.observe(o, rig.held);
        bool hold = (t % 150) < 2;
        rig.held = hold;
        w.step(hold);
    }
}

sim::Level spikeLevel(char const* name, std::initializer_list<float> spikes, float length = 60.f) {
    sim::Level l;
    l.name = name;
    for (float b : spikes) l.spike(b);
    l.finish(length);
    return l;
}

struct Truth {
    bool died = false;
    int32_t tick = 0;
    float x = 0.f, y = 0.f;
    bool landed = false;
    float landX = 0.f, landY = 0.f;
};

// Ground truth from the simulator: press at step `pressAt` and hold until take-off.
Truth groundTruth(sim::Level const& level, int pressAt, float passX, int speedClass = 1) {
    sim::World w(level, speedClass);
    Truth r;
    int takeoffs = 0;
    bool airborne = false;
    for (int t = 0; t < 4000; ++t) {
        bool onGround = w.observe().onGround;
        bool hold = pressAt >= 0 && t >= pressAt && takeoffs < 1;
        w.step(hold);
        auto o = w.observe();
        if (hold && onGround && !o.onGround) ++takeoffs;
        if (!o.onGround) airborne = true;
        if (airborne && o.onGround && !r.landed) {
            r.landed = true;
            r.landX = o.x;
            r.landY = o.y;
        }
        if (w.dead()) {
            r.died = true;
            r.tick = o.tick;
            r.x = w.death().x;
            r.y = w.death().y;
            return r;
        }
        if (w.x() > passX || w.complete()) {
            r.tick = o.tick;
            return r;
        }
    }
    return r;
}

PredictedPath predict(PredictionRig& rig, sim::Level const& level, Plan plan, int speedClass = 1) {
    sim::World w(level, speedClass);
    Observation o = rig.observe(w);
    PredictedPath p;
    rig.predictor.simulate(rig.predictor.startFrom(o, false), o.obstacles, plan, p);
    return p;
}

struct Sweep {
    int agree = 0, falseSafe = 0, falseCollision = 0, uncertain = 0, total = 0;
};

Sweep sweepTimings(PredictionRig& rig, sim::Level const& level, int from, int to, float passX) {
    Sweep s;
    for (int k = from; k <= to; ++k) {
        Truth t = groundTruth(level, k, passX);
        PredictedPath p = predict(rig, level, Plan{k, false});
        // only judge the prediction up to the point the ground truth covered
        bool predHit = p.outcome == Certainty::Collision && p.collisionStep + 1 <= t.tick + 2;
        bool predSafe = p.outcome == Certainty::Safe || (p.outcome == Certainty::Collision && !predHit);
        ++s.total;
        if (p.outcome == Certainty::Uncertain) ++s.uncertain;
        else if (t.died && predHit) ++s.agree;
        else if (!t.died && predSafe) ++s.agree;
        else if (t.died) ++s.falseSafe;
        else ++s.falseCollision;
    }
    return s;
}
} // namespace

TEST(physics_calibration_learns_the_game) {
    PredictionRig rig;
    rig.predictor.setHorizon(400);
    // Defaults are Geometry Dash's measured values; the simulator deliberately
    // uses slightly different physics, so these must be learned.
    CHECK(std::fabs(-rig.model.cube(false).accRelease - sim::World::kGravity) > 0.004f);
    CHECK(!rig.model.calibrated(false));
    warmUp(rig);
    auto const& p = rig.model.cube(false);
    std::printf("    learned: %s, vx=%.4f, drift %.4f/step\n", rig.model.describe().c_str(), rig.model.vxPerTick(1), rig.model.driftPerTick());
    CHECK(rig.model.calibrated(false));
    CHECK(std::fabs(-p.accRelease - sim::World::kGravity) < 0.002f);
    CHECK(std::fabs(p.yScale - 1.f / sim::World::kVyScale) < 0.002f);
    CHECK(std::fabs(p.impulse - (sim::World::kJumpVy - sim::World::kGravity)) < 0.05f);
    CHECK(std::fabs(rig.model.cubeFloorY(false) - sim::World::kHalf) < 0.5f);
    CHECK(std::fabs(rig.model.vxPerTick(1) - aigd::unitsPerSecond(1) / 240.f) < 0.01f);
    CHECK(rig.model.accuracySamples() > 10);
    CHECK(rig.model.driftPerTick() < 0.02f);
    CHECK(rig.predictor.trusted(false));
}

TEST(prediction_standing_still_before_spike) {
    PredictionRig rig;
    warmUp(rig);
    rig.predictor.setHorizon(400);
    auto level = spikeLevel("one", {6.f});
    PredictedPath p = predict(rig, level, Plan{-1, false});
    Truth t = groundTruth(level, -1, 400.f);
    CHECK(t.died);
    CHECK(p.outcome == Certainty::Collision);
    std::printf("    predicted collision at step %d (x=%.1f), simulator died at step %d (x=%.1f)\n", p.collisionStep + 1, p.collisionX, t.tick, t.x);
    CHECK(std::abs((p.collisionStep + 1) - t.tick) <= 1);
    CHECK(std::fabs(p.collisionX - t.x) < 2.f);
}

TEST(prediction_jump_timings_single_spike) {
    PredictionRig rig;
    warmUp(rig);
    rig.predictor.setHorizon(400);
    auto level = spikeLevel("one", {6.f});
    Sweep s = sweepTimings(rig, level, 20, 130, 320.f);
    std::printf("    %d timings: %d agree, %d uncertain, %d false-safe, %d false-collision\n", s.total, s.agree, s.uncertain, s.falseSafe, s.falseCollision);
    CHECK(s.falseSafe == 0);
    CHECK(s.agree >= s.total * 85 / 100);
    // both "too early" and "too late" timings exist in this range and are predicted
    CHECK(groundTruth(level, 20, 320.f).died);   // far too early: lands on the spike
    CHECK(groundTruth(level, 130, 320.f).died);  // too late: runs into it
}

TEST(prediction_jump_timings_consecutive_spikes) {
    PredictionRig rig;
    warmUp(rig);
    rig.predictor.setHorizon(400);
    auto level = spikeLevel("triple", {6.f, 7.f, 8.f});
    Sweep s = sweepTimings(rig, level, 40, 130, 380.f);
    int safeTimings = 0;
    for (int k = 40; k <= 130; ++k) safeTimings += groundTruth(level, k, 380.f).died ? 0 : 1;
    std::printf("    %d timings (%d survivable): %d agree, %d uncertain, %d false-safe, %d false-collision\n", s.total, safeTimings, s.agree,
                s.uncertain, s.falseSafe, s.falseCollision);
    CHECK(safeTimings > 0);
    CHECK(s.falseSafe == 0);
    CHECK(s.agree >= s.total * 85 / 100);
}

TEST(prediction_landing_positions) {
    PredictionRig rig;
    warmUp(rig);
    rig.predictor.setHorizon(400);
    // flat ground
    {
        auto level = spikeLevel("flat", {});
        Truth t = groundTruth(level, 10, 600.f);
        PredictedPath p = predict(rig, level, Plan{10, false});
        CHECK(t.landed && p.landed);
        std::printf("    flat: predicted landing x=%.2f y=%.2f, simulator x=%.2f y=%.2f\n", p.landX, p.landY, t.landX, t.landY);
        CHECK(std::fabs(p.landX - t.landX) < 1.5f);
        CHECK(std::fabs(p.landY - t.landY) < 0.5f);
    }
    // landing on a block before the next obstacle
    {
        sim::Level level;
        level.name = "step";
        level.block(8, 0, 4, 1);
        level.spike(14);
        level.finish(40);
        int found = -1;
        Truth t;
        for (int k = 60; k < 200 && found < 0; ++k) {
            t = groundTruth(level, k, 400.f);
            if (!t.died && t.landed && std::fabs(t.landY - 45.f) < 0.5f) found = k;
        }
        CHECK(found >= 0);
        if (found >= 0) {
            PredictedPath p = predict(rig, level, Plan{found, false});
            std::printf("    block top: predicted landing x=%.2f y=%.2f, simulator x=%.2f y=%.2f (press at %d)\n", p.landX, p.landY, t.landX, t.landY, found);
            CHECK(p.landed);
            CHECK(std::fabs(p.landY - 45.f) < 0.5f);
            CHECK(std::fabs(p.landX - t.landX) < 1.5f);
        }
    }
}

TEST(prediction_different_speeds) {
    for (int sc : {0, 2, 4}) {
        PredictionRig rig;
        warmUp(rig, sc);
        rig.predictor.setHorizon(400);
        auto level = spikeLevel("one", {10.f});
        PredictedPath p = predict(rig, level, Plan{-1, false}, sc);
        Truth t = groundTruth(level, -1, 600.f, sc);
        std::printf("    speed %d: vx=%.3f, predicted hit step %d, simulator %d\n", sc, rig.model.vxPerTick(sc), p.collisionStep + 1, t.tick);
        CHECK(p.outcome == Certainty::Collision);
        CHECK(std::abs((p.collisionStep + 1) - t.tick) <= 1);
        Sweep s = sweepTimings(rig, level, 0, 400 / (sc + 1), 600.f);
        CHECK(s.falseSafe == 0);
    }
}

TEST(prediction_gravity_changes) {
    PredictionRig rig;
    warmUp(rig);
    rig.predictor.setHorizon(400);
    // A gravity portal ahead is not simulated: the path must not be called safe.
    {
        sim::Level level;
        level.name = "portal";
        level.gravityPortal(6);
        level.spike(12);
        level.finish(40);
        PredictedPath p = predict(rig, level, Plan{-1, false});
        CHECK(p.outcome == Certainty::Uncertain);
        CHECK(p.uncertainFrom > 0);
    }
    // Upside down (after a portal): "falls" up onto the underside of a ceiling.
    {
        sim::Level level;
        level.name = "ceiling";
        level.block(0, 4, 40, 1); // y 120..150
        level.finish(40);
        sim::World w(level);
        w.place(60.f, 60.f, 0.f, false, true);
        Observation o = rig.observe(w);
        PredictedPath p;
        rig.predictor.simulate(rig.predictor.startFrom(o, false), o.obstacles, Plan{-1, false}, p);
        float landX = 0.f, landY = 0.f;
        for (int t = 0; t < 200; ++t) {
            w.step(false);
            if (w.observe().onGround) {
                landX = w.observe().x;
                landY = w.observe().y;
                break;
            }
        }
        std::printf("    upside down: predicted landing x=%.2f y=%.2f, simulator x=%.2f y=%.2f\n", p.landX, p.landY, landX, landY);
        CHECK(p.landed);
        CHECK(std::fabs(p.landY - 105.f) < 0.5f && std::fabs(landY - 105.f) < 0.5f);
        CHECK(std::fabs(p.landX - landX) < 1.5f);
    }
}

TEST(prediction_overlay_matches_simulation) {
    PredictionRig rig;
    warmUp(rig);
    rig.predictor.setHorizon(400);
    auto level = spikeLevel("one", {6.f});
    sim::World w(level);
    Observation o = rig.observe(w);
    PredictedPath main, alt;
    rig.predictor.simulate(rig.predictor.startFrom(o, false), o.obstacles, Plan{-1, false}, main);
    rig.predictor.simulate(rig.predictor.startFrom(o, false), o.obstacles, Plan{95, false}, alt);
    OverlayGeometry g;
    buildOverlay(main, &alt, o.obstacles, g, 4);
    CHECK(g.outcome == Certainty::Collision);
    CHECK(g.hasCollision);
    CHECK(g.collisionX == main.collisionX && g.collisionY == main.collisionY);
    CHECK(g.stepsToCollision == main.collisionStep + 1);
    CHECK(g.label.find("collision in") != std::string::npos);
    // every vertex of the main path drawing is a simulated point, in order, ending at the last one
    std::vector<OverlayGeometry::Segment> mainSegs;
    for (auto const& s : g.segments) if (s.style == OverlayGeometry::Style::PathDanger) mainSegs.push_back(s);
    CHECK(!mainSegs.empty());
    CHECK(mainSegs.front().x0 == main.points.front().x && mainSegs.front().y0 == main.points.front().y);
    CHECK(mainSegs.back().x1 == main.points.back().x && mainSegs.back().y1 == main.points.back().y);
    bool allOnPath = true;
    for (auto const& s : mainSegs) {
        bool found = false;
        for (auto const& pt : main.points) found |= (pt.x == s.x1 && pt.y == s.y1);
        allOnPath &= found;
    }
    CHECK(allOnPath);
    // the hazard that is hit is marked, at its real hitbox
    bool hitMarked = false;
    for (auto const& b : g.boxes) {
        if (b.style == OverlayGeometry::Style::HazardHit) {
            auto const& h = o.obstacles[static_cast<size_t>(main.obstacle)];
            hitMarked = b.l == h.l && b.r == h.r && b.b == h.b && b.t == h.t;
        }
    }
    CHECK(hitMarked);
    // alternative path drawn separately, safe path style when not colliding
    size_t altSegs = 0;
    for (auto const& s : g.segments) altSegs += s.style == OverlayGeometry::Style::Alternative ? 1 : 0;
    CHECK(altSegs > 0);
    buildOverlay(alt, nullptr, o.obstacles, g, 4);
    CHECK(alt.outcome == Certainty::Safe);
    CHECK(g.outcome == Certainty::Safe && !g.hasCollision);
}

TEST(planner_alone_clears_cube_levels) {
    // Drive the simulator with nothing but the receding-horizon planner (no
    // learning): re-plan every step, execute only the first action.
    for (auto const& level : {sim::levelBasics(), sim::levelSpikes(), sim::levelBlocks()}) {
        PredictionRig rig;
        warmUp(rig);
        sim::World w(level);
        int firstPress = -1;
        double micros = 0;
        int advises = 0, sims = 0, ticks = 0;
        for (int t = 0; t < 100000 && !w.dead() && !w.complete(); ++t) {
            Observation o = rig.observe(w);
            rig.predictor.observe(o, rig.held);
            auto const& adv = rig.predictor.advise(o, rig.held);
            bool hold = adv.valid && adv.bestPresses;
            if (adv.valid) {
                micros += adv.micros;
                sims += adv.simulations;
                ++advises;
            }
            if (hold && firstPress < 0) firstPress = t;
            rig.predictor.noteExecuted(hold);
            rig.held = hold;
            w.step(hold);
            ++ticks;
        }
        std::printf("    %-7s planner only: %s at %.1f%%, first press at step %d; %.1f us per physics step on average "
                    "(%.1f simulations per step, %d evaluated steps)\n",
                    level.name.c_str(), w.complete() ? "COMPLETED" : "died", w.observe().progress() * 100.f, firstPress,
                    ticks ? micros / ticks : 0.0, ticks ? double(sims) / ticks : 0.0, advises);
        CHECK(w.complete());
        // it waits for the right moment instead of jumping straight away
        CHECK(firstPress > 100);
    }
}

TEST(prediction_makes_learning_faster) {
    for (auto const& level : {sim::levelBasics(), sim::levelSpikes(), sim::levelBlocks(), sim::levelShip()}) {
        LevelMemory m1 = freshLevel(level), m2 = freshLevel(level);
        PatternMemory p1, p2;
        Agent with, without;
        PredictionRig rig;
        with.attach(&m1, &p1);
        with.setPredictor(&rig.predictor);
        without.attach(&m2, &p2);
        auto a = train(with, level, 1500, &rig);
        auto b = train(without, level, 1500);
        std::printf("    %-7s attempts to complete: %d with prediction (%llu collisions avoided), %d without\n", level.name.c_str(),
                    a.attemptsToComplete, static_cast<unsigned long long>(with.status().shieldsTotal), b.attemptsToComplete);
        CHECK(a.attemptsToComplete > 0);
        if (level.name != "ship") CHECK(a.attemptsToComplete <= std::max(3, b.attemptsToComplete / 2));
    }
}

TEST(prediction_learning_survives_reload_and_stays_consistent) {
    auto level = sim::levelSpikes();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    PredictionRig rig;
    Agent agent;
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = train(agent, level, 200, &rig);
    CHECK(r.attemptsToComplete > 0);
    int completed = 0;
    for (int i = 0; i < 20; ++i) completed += runAttempt(agent, level, &rig).completed ? 1 : 0;
    CHECK(completed >= 19);
    // the calibration is plain data and round-trips through the pattern file
    pat.physics = rig.model.calibration();
    PatternMemory loaded;
    std::string err;
    CHECK(storage::deserializePatterns(storage::serializePatterns(pat), loaded, &err));
    PhysicsModel m2;
    m2.loadCalibration(loaded.physics);
    CHECK(m2.calibrated(false));
    CHECK(std::fabs(m2.cube(false).accRelease - rig.model.cube(false).accRelease) < 1e-5f);
}

TEST(misprediction_makes_predictor_cautious) {
    PredictionRig rig;
    warmUp(rig);
    auto level = spikeLevel("one", {6.f});
    sim::World w(level);
    Observation o = rig.observe(w);
    auto const& adv = rig.predictor.advise(o, false);
    CHECK(adv.valid && adv.release == Certainty::Safe);
    rig.predictor.noteExecuted(false);
    float before = rig.predictor.caution();
    // pretend the player died right after a "safe" prediction
    rig.predictor.onDeath(o.tick + 1, o.x + 1.f, o.y);
    CHECK(rig.predictor.mispredictions() == 1);
    CHECK(rig.predictor.caution() > before);
    CHECK(rig.predictor.lastDeathComparison().find("MISPREDICTION") != std::string::npos);
    CHECK(rig.predictor.margin(10, false) > 0.5f);
}

TEST(prediction_is_not_trusted_before_calibration) {
    PredictionRig rig; // GD defaults, no observations of this "game"
    auto level = spikeLevel("one", {6.f});
    sim::World w(level);
    Observation o = rig.observe(w);
    auto const& adv = rig.predictor.advise(o, false);
    CHECK(adv.valid);
    CHECK(!adv.trusted);
    // an untrusted agent hook must not override anything
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent;
    agent.attach(&mem, &pat);
    PredictionRig fresh;
    agent.setPredictor(&fresh.predictor);
    agent.beginAttempt(o, false);
    agent.act(o);
    CHECK(!agent.status().shielded && !agent.status().planned);
}

// ---------------------------------------------------------------------------
// Trajectory prediction for every game mode

namespace {
constexpr GameMode kAllModes[] = {GameMode::Cube, GameMode::Ship, GameMode::Ball, GameMode::Ufo,
                                  GameMode::Wave, GameMode::Robot, GameMode::Spider, GameMode::Swing};

// A mode-sensible random input pattern (what a player of that mode does).
struct InputPattern {
    GameMode mode;
    uint32_t state;
    int left = 0;
    bool hold = false;

    uint32_t next() {
        state = state * 1664525u + 1013904223u;
        return state >> 8;
    }
    int range(int lo, int hi) { return lo + static_cast<int>(next() % static_cast<uint32_t>(hi - lo + 1)); }

    bool step() {
        if (--left > 0) return hold;
        switch (mode) {
            case GameMode::Ship:
            case GameMode::Wave:
                hold = !hold;
                left = range(8, 40);
                break;
            case GameMode::Ufo:
                hold = !hold;
                left = hold ? 2 : range(20, 45);
                break;
            case GameMode::Ball:
            case GameMode::Swing:
            case GameMode::Spider:
                hold = !hold;
                left = hold ? 2 : range(40, 90);
                break;
            case GameMode::Robot:
                hold = !hold;
                left = hold ? range(5, 60) : range(30, 80);
                break;
            default: // cube
                hold = !hold;
                left = hold ? 2 : range(60, 140);
                break;
        }
        return hold;
    }
};

// Flat corridor in one mode (floor + a ceiling of blocks), long enough to
// calibrate on.
sim::Level corridor(GameMode m, float length = 600.f) {
    sim::Level l;
    l.name = std::string("corridor-") + gameModeName(m);
    l.modePortal(1, m);
    if (m != GameMode::Cube && m != GameMode::Robot) l.block(3, m == GameMode::Ball || m == GameMode::Swing || m == GameMode::Spider ? 5.f : 7.f, length, 1);
    l.finish(length);
    return l;
}

void warmUp(PredictionRig& rig, int speedClass);

void warmUpMode(PredictionRig& rig, GameMode m, int steps = 6000, uint32_t seed = 7) {
    if (m == GameMode::Spider) warmUp(rig, 1); // spiders borrow the cube's gravity (they rarely free-fall)
    sim::Level lvl = corridor(m);
    sim::World w(lvl);
    InputPattern in{m, seed};
    rig.held = false;
    for (int t = 0; t < steps && !w.dead() && !w.complete(); ++t) {
        Observation o = rig.observe(w);
        rig.predictor.observe(o, rig.held);
        bool hold = w.mode() == m ? in.step() : false;
        rig.held = hold;
        w.step(hold);
    }
}

struct Accuracy {
    double meanErr = 0, maxErr = 0;
    int samples = 0;
};

// Start from many states of a random run and predict 32 steps ahead with the
// inputs that are then actually used; compare with where the simulator ends up.
Accuracy measureAccuracy(PredictionRig& rig, GameMode m, uint32_t seed = 99) {
    sim::Level lvl = corridor(m);
    sim::World w(lvl);
    InputPattern in{m, seed};
    Accuracy a;
    std::vector<uint8_t> holds;
    bool held = false;
    for (int t = 0; t < 4000 && !w.dead() && !w.complete(); ++t) {
        Observation o = rig.observe(w);
        rig.predictor.observe(o, held);
        if (t > 200 && t % 37 == 0 && w.mode() == m) {
            sim::World future = w;
            InputPattern fin = in;
            holds.clear();
            for (int k = 0; k < 32; ++k) {
                bool h = fin.step();
                holds.push_back(h ? 1 : 0);
                future.step(h);
            }
            if (!future.dead()) {
                PredictedPath p;
                rig.predictor.simulate(rig.predictor.startFrom(o, held), o.obstacles, Plan{}, p, holds);
                if (static_cast<int>(p.points.size()) == 32) {
                    auto f = future.observe();
                    double err = std::hypot(p.points.back().x - f.x, p.points.back().y - f.y);
                    if (std::getenv("AIGD_ACC_DEBUG") && err > 3.0) {
                        std::printf("      start x=%.1f y=%.2f vy=%.2f ground=%d held=%d | pred y=%.2f real y=%.2f | holds ", o.x, o.y, o.vy,
                                    (int)o.onGround, (int)held, p.points.back().y, f.y);
                        for (auto hh : holds) std::printf("%d", (int)hh);
                        std::printf(" | floor %.1f%s ceil %d %.1f\n", rig.predictor.bounds().floor, rig.predictor.bounds().floorEstimated ? "?" : "",
                                    (int)rig.predictor.bounds().hasCeil, rig.predictor.bounds().ceil);
                    }
                    a.meanErr += err;
                    a.maxErr = std::max(a.maxErr, err);
                    ++a.samples;
                }
            }
        }
        bool hold = w.mode() == m ? in.step() : false;
        held = hold;
        w.step(hold);
    }
    if (a.samples) a.meanErr /= a.samples;
    return a;
}
} // namespace

TEST(all_modes_calibrate_to_the_game) {
    for (GameMode m : kAllModes) {
        PredictionRig rig;
        CHECK(!rig.model.calibrated(m, false));
        if (m == GameMode::Spider) warmUp(rig); // spiders borrow the cube's gravity (they rarely free-fall)
        warmUpMode(rig, m);
        auto const& p = rig.model.params(m, false);
        auto S = sim::World::spec(m);
        std::printf("    %-6s %s\n", gameModeName(m), rig.model.describe(m).c_str());
        CHECK_MSG(rig.model.calibrated(m, false), "%s not calibrated", gameModeName(m));
        switch (m) {
            case GameMode::Ship:
                CHECK(std::fabs(p.accHold - S.accHold) < 0.01f);
                CHECK(std::fabs(p.accRelease - S.accRelease) < 0.01f);
                break;
            case GameMode::Ufo:
                CHECK(std::fabs(p.impulse - S.impulse) < 0.05f);
                CHECK(std::fabs(p.accRelease - S.accRelease) < 0.01f);
                break;
            case GameMode::Ball:
            case GameMode::Swing:
                CHECK(std::fabs(p.accRelease - S.accRelease) < 0.01f);
                CHECK(std::fabs(p.flipVy - S.flipVy) < 0.1f);
                break;
            case GameMode::Wave:
                CHECK(std::fabs(p.waveVy - S.slope) < 0.02f);
                break;
            case GameMode::Robot:
                CHECK(std::fabs(p.impulse - (S.impulse + S.accHold)) < 0.05f);
                CHECK(std::fabs(p.accHold - S.accHold) < 0.01f);
                CHECK(std::fabs(p.accRelease - S.accRelease) < 0.01f);
                CHECK(std::abs(p.boostSteps - (S.boostSteps - 1)) <= 2);
                break;
            case GameMode::Spider:
                CHECK(std::fabs(p.accRelease - S.accRelease) < 0.01f);
                break;
            default:
                CHECK(std::fabs(p.accRelease - S.accRelease) < 0.01f);
                break;
        }
        if (m != GameMode::Wave && m != GameMode::Spider) CHECK(std::fabs(p.yScale - 1.f / sim::World::kVyScale) < 0.003f);
    }
}

TEST(all_modes_predict_32_steps_ahead) {
    for (GameMode m : kAllModes) {
        PredictionRig rig;
        if (m == GameMode::Spider) warmUp(rig);
        warmUpMode(rig, m);
        Accuracy a = measureAccuracy(rig, m);
        std::printf("    %-6s %d samples: mean error %.2f, max %.2f units after 32 steps (resim drift %.4f/step)\n", gameModeName(m),
                    a.samples, a.meanErr, a.maxErr, rig.model.driftPerTick(m));
        CHECK(a.samples >= 20);
        CHECK_MSG(a.meanErr < 1.0, "%s mean error %.2f", gameModeName(m), a.meanErr);
        CHECK_MSG(a.maxErr < 5.0, "%s max error %.2f", gameModeName(m), a.maxErr);
        CHECK(rig.predictor.trusted(m, false));
    }
}

TEST(all_modes_planner_alone_clears_course) {
    // No learning: re-plan every step with the receding-horizon planner only.
    for (GameMode m : kAllModes) {
        PredictionRig rig;
        warmUpMode(rig, m);
        sim::Level level = sim::modeCourse(m);
        sim::World w(level);
        double micros = 0;
        int ticks = 0, sims = 0;
        for (int t = 0; t < 100000 && !w.dead() && !w.complete(); ++t) {
            Observation o = rig.observe(w);
            rig.predictor.observe(o, rig.held);
            auto const& adv = rig.predictor.advise(o, rig.held);
            bool hold = adv.valid ? adv.bestPresses : (w.mode() == GameMode::Ship ? o.y < 90.f : false);
            if (adv.valid) {
                micros += adv.micros;
                sims += adv.simulations;
            }
            rig.predictor.noteExecuted(hold);
            rig.held = hold;
            w.step(hold);
            ++ticks;
            if (w.dead() && std::getenv("AIGD_PLAN_DEBUG")) {
                auto const& pp = rig.predictor.pressPath();
                auto const& rp = rig.predictor.releasePath();
                std::printf("      died at x=%.1f y=%.2f (obs y=%.2f vy=%.2f ground=%d) valid=%d trusted=%d press=%s(at %d hold %d rep %d, ttc %d) "
                            "release=%s(at %d hold %d rep %d, ttc %d) best=%s sims=%d bounds floor %.1f%s ceil %d %.1f%s\n",
                            w.death().x, w.death().y, o.y, o.vy, (int)o.onGround, (int)adv.valid, (int)adv.trusted,
                            certaintyName(adv.press), pp.plan.pressAt, pp.plan.holdFor, pp.plan.repeatEvery, pp.collisionStep,
                            certaintyName(adv.release), rp.plan.pressAt, rp.plan.holdFor, rp.plan.repeatEvery, rp.collisionStep,
                            adv.bestPresses ? "press" : "release", adv.simulations, rig.predictor.bounds().floor,
                            rig.predictor.bounds().floorEstimated ? "?" : "", (int)rig.predictor.bounds().hasCeil, rig.predictor.bounds().ceil,
                            rig.predictor.bounds().ceilEstimated ? "?" : "");
            }
        }
        std::printf("    %-6s planner only: %s at %.1f%%, %.1f us and %.1f simulations per step\n", gameModeName(m),
                    w.complete() ? "COMPLETED" : "died", w.observe().progress() * 100.f, ticks ? micros / ticks : 0.0,
                    ticks ? double(sims) / ticks : 0.0);
        CHECK_MSG(w.complete(), "%s course not completed by the planner", gameModeName(m));
    }
}

TEST(all_modes_collision_predictions_are_never_falsely_safe) {
    // From many states on each course, the outcome of the planner's own best
    // plan is compared with what really happens when that plan is executed
    // open-loop: a path predicted "safe" must not die within its horizon.
    for (GameMode m : kAllModes) {
        PredictionRig rig;
        warmUpMode(rig, m);
        sim::Level level = sim::modeCourse(m);
        sim::World w(level);
        InputPattern in{m, 4242};
        int checked = 0, falseSafe = 0, predictedHits = 0, confirmedHits = 0;
        for (int t = 0; t < 6000 && !w.dead() && !w.complete(); ++t) {
            Observation o = rig.observe(w);
            rig.predictor.observe(o, rig.held);
            if (t % 23 == 0 && w.mode() == m) {
                auto const& adv = rig.predictor.advise(o, rig.held, false); // fresh plans from this exact state
                if (adv.valid && adv.trusted) {
                    for (PredictedPath const* p : {&rig.predictor.pressPath(), &rig.predictor.releasePath()}) {
                        // replay exactly the simulated button states in a copy of the world
                        sim::World f = w;
                        for (size_t k = 0; k < p->holds.size() && !f.dead(); ++k) f.step(p->holds[k] != 0);
                        ++checked;
                        if (p->outcome == Certainty::Safe && f.dead()) {
                            ++falseSafe;
                            if (std::getenv("AIGD_FS_DEBUG")) {
                                size_t k = static_cast<size_t>(f.observe().tick - o.tick) - 1;
                                std::printf("      false safe: start x=%.1f y=%.2f vy=%.2f ground=%d plan(at %d hold %d rep %d rejump %d) died step %zu at (%.1f,%.2f) killer dx %.1f dy %.1f; predicted (%.1f,%.2f)\n",
                                            o.x, o.y, o.vy, (int)o.onGround, p->plan.pressAt, p->plan.holdFor, p->plan.repeatEvery, (int)p->plan.rejump, k,
                                            f.death().x, f.death().y, f.death().killerDx, f.death().killerDy,
                                            k < p->points.size() ? p->points[k].x : -1.f, k < p->points.size() ? p->points[k].y : -1.f);
                            }
                        }
                        if (p->outcome == Certainty::Collision) {
                            ++predictedHits;
                            if (f.dead()) ++confirmedHits;
                        }
                    }
                }
            }
            bool hold = w.mode() == m ? in.step() : false;
            rig.held = hold;
            w.step(hold);
        }
        std::printf("    %-6s %d plans checked: %d predicted collisions (%d confirmed), %d falsely safe\n", gameModeName(m), checked,
                    predictedHits, confirmedHits, falseSafe);
        CHECK(checked > 0);
        CHECK_MSG(falseSafe == 0, "%s: %d paths predicted safe died", gameModeName(m), falseSafe);
    }
}

TEST(all_modes_overlay_and_agent_hook_work) {
    // The agent uses prediction in every mode once calibrated.
    for (GameMode m : kAllModes) {
        PredictionRig rig;
        warmUpMode(rig, m);
        sim::Level level = sim::modeCourse(m);
        LevelMemory mem = freshLevel(level);
        PatternMemory pat;
        Agent agent;
        agent.attach(&mem, &pat);
        agent.setPredictor(&rig.predictor);
        auto r = train(agent, level, 300, &rig);
        std::printf("    %-6s learning with prediction: completed after %d attempts (%llu collisions avoided)\n", gameModeName(m),
                    r.attemptsToComplete, static_cast<unsigned long long>(agent.status().shieldsTotal));
        CHECK(r.attemptsToComplete > 0);
        // overlay geometry is produced for the current plan
        OverlayGeometry g;
        buildOverlay(rig.predictor.bestPath(), &rig.predictor.alternativePath(), {}, g);
        CHECK(!g.label.empty());
    }
}

// ---------------------------------------------------------------------------
// Learning architecture (2.0): contextual failure memory, credit assignment,
// no copying of successful runs, generalisation, physics correction

namespace {
// Agent with only the mechanism under test: no timing experiments, no random
// exploration.
LearnerConfig isolatedConfig() {
    LearnerConfig cfg;
    cfg.useExperiments = false;
    cfg.focusEpsilon = 0.f;
    return cfg;
}

struct GravityScale {
    explicit GravityScale(float s) { sim::World::s_gravityScale = s; }
    ~GravityScale() { sim::World::s_gravityScale = 1.f; }
};

size_t countIf(QTable const& t, std::function<bool(QEntry const&)> const& f) {
    size_t n = 0;
    for (auto const& [k, e] : t) n += f(e) ? 1 : 0;
    return n;
}
} // namespace

TEST(repeated_failure_is_detected_and_the_alternative_tested) {
    sim::Level level = sim::levelBasics();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg = isolatedConfig();
    Agent agent(cfg);
    agent.attach(&mem, &pat);
    sim::World w(level);
    Observation o = w.observe();
    QEntry& e = mem.q.getOrCreate(encodeLevelKey(o, false, cfg.cellSize));
    // The values still favour waiting here, but waiting was identified as the
    // cause of the last two deaths in exactly this state.
    e.q[0] = 1.f;
    e.q[1] = 0.f;
    e.n[0] = 5;
    e.n[1] = 1;
    e.fails[0] = 2;
    e.streak[0] = 2;

    agent.beginAttempt(o, false);
    bool hold = agent.act(o);
    auto const& st = agent.status();
    std::printf("    decision: %s | %s\n", decisionSourceName(st.source), st.reason.c_str());
    CHECK(hold);
    CHECK(st.source == DecisionSource::AvoidRepeat);
    CHECK(st.reason.find("Repeated failure detected") != std::string::npos);
    CHECK(st.metrics.repeatedFailuresDetected == 1);
    CHECK(st.metrics.alternativesTried == 1);
    agent.abortAttempt(nullptr);

    // One failure is not yet a pattern: the values decide.
    e.streak[0] = 1;
    agent.beginAttempt(o, false);
    CHECK(!agent.act(o));
    CHECK(agent.status().source == DecisionSource::Policy);
    agent.abortAttempt(nullptr);

    // A pass since then (newer evidence) clears the streak.
    e.streak[0] = 2;
    e.notePass(0);
    agent.beginAttempt(o, false);
    CHECK(!agent.act(o));
    agent.abortAttempt(nullptr);
}

TEST(repeated_failure_alternative_is_vetted_by_prediction) {
    // Spike 4 blocks ahead: jumping now lands on it, waiting (and jumping
    // later) clears it. Waiting "failed twice" here, but the only alternative
    // is predicted to collide, so switching is rejected.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level;
    level.name = "veto";
    level.spike(4.4f);
    level.finish(20);
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg = isolatedConfig();
    Agent agent(cfg);
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    sim::World w(level);
    Observation o = rig.observe(w);
    rig.predictor.observe(o, false);
    auto const& adv = rig.predictor.advise(o, false, false);
    std::printf("    press now: %s, wait: %s\n", certaintyName(adv.press), certaintyName(adv.release));
    CHECK(adv.valid && adv.trusted);
    CHECK(adv.press == Certainty::Collision);
    CHECK(adv.release != Certainty::Collision);

    QEntry& e = mem.q.getOrCreate(encodeLevelKey(o, false, cfg.cellSize));
    e.q[0] = 1.f;
    e.n[0] = 5;
    e.fails[0] = 2;
    e.streak[0] = 2;
    agent.beginAttempt(o, false);
    bool hold = agent.act(o);
    std::printf("    decision: %s | %s\n", decisionSourceName(agent.status().source), agent.status().reason.c_str());
    CHECK(!hold);
    CHECK(agent.status().metrics.alternativesVetoed == 1);
    CHECK(agent.status().reason.find("predicted to collide") != std::string::npos);
    agent.abortAttempt(nullptr);
}

TEST(failure_memory_is_contextual) {
    sim::Level level = sim::levelBasics();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg = isolatedConfig();
    Agent agent(cfg);
    agent.attach(&mem, &pat);
    sim::World w(level);
    Observation a = w.observe();
    for (int t = 0; t < 40; ++t) w.step(false);
    Observation b = w.observe(); // further along, otherwise the same situation
    Observation c = a;           // same place, different state (airborne, rising)
    c.y += 40.f;
    c.vy = 6.f;
    c.onGround = false;
    uint64_t ka = encodeLevelKey(a, false, cfg.cellSize);
    uint64_t kb = encodeLevelKey(b, false, cfg.cellSize);
    uint64_t kc = encodeLevelKey(c, false, cfg.cellSize);
    CHECK(ka != kb && ka != kc && kb != kc);
    for (uint64_t k : {ka, kb, kc}) {
        QEntry& e = mem.q.getOrCreate(k);
        e.q[1] = 1.f; // pressing preferred everywhere
        e.n[1] = 3;
    }
    QEntry& bad = mem.q.getOrCreate(ka);
    bad.fails[1] = 3;
    bad.streak[1] = 3; // ...but pressing failed repeatedly in state a only

    auto decide = [&](Observation const& o) {
        agent.beginAttempt(o, false);
        bool h = agent.act(o);
        DecisionSource s = agent.status().source;
        agent.abortAttempt(nullptr);
        return std::make_pair(h, s);
    };
    auto da = decide(a), db = decide(b), dc = decide(c);
    CHECK(!da.first && da.second == DecisionSource::AvoidRepeat);
    // Not a global "do not press" rule: other places and other states are unaffected.
    CHECK(db.first && db.second == DecisionSource::Policy);
    CHECK(dc.first && dc.second == DecisionSource::Policy);
}

TEST(failure_memory_breaks_a_mistake_values_alone_repeat) {
    // Without a death penalty a death costs the values nothing, so plain
    // Q-learning keeps repeating the fatal decision before the first spike.
    // The contextual failure memory has to recognise and break the loop.
    struct Outcome {
        float bestX;
        LearningMetrics m;
        int sameEnd;
    };
    auto run = [](int threshold) {
        sim::Level level = sim::levelBasics();
        LevelMemory mem = freshLevel(level);
        PatternMemory pat;
        LearnerConfig cfg = isolatedConfig();
        cfg.deathPenalty = 0.f;
        cfg.repeatFailureThreshold = threshold;
        Agent agent(cfg);
        agent.attach(&mem, &pat);
        int sameEnd = 0;
        float lastEnd = -1.f;
        for (int i = 0; i < 80; ++i) {
            auto r = runAttempt(agent, level);
            if (std::fabs(r.endX - lastEnd) < 0.01f) ++sameEnd;
            lastEnd = r.endX;
            if (r.completed) break;
        }
        return Outcome{mem.bestX, agent.status().metrics, sameEnd};
    };
    Outcome with = run(2), without = run(100000);
    float spike1 = 12.f * kBlock;
    std::printf("    without failure memory: best x %.0f, %d of 79 attempts died at exactly the same spot\n", without.bestX, without.sameEnd);
    std::printf("    with failure memory:    best x %.0f; %u repeated failures detected, %u alternatives tried, %u helped, "
                "%u deaths on an already known mistake\n",
                with.bestX, with.m.repeatedFailuresDetected, with.m.alternativesTried, with.m.alternativesThatHelped,
                with.m.sameMistakeRepeated);
    CHECK(without.bestX < spike1 + kBlock);       // values alone: stuck at the first spike
    CHECK(with.bestX > spike1 + 2.f * kBlock);    // failure memory: gets past it
    CHECK(with.m.repeatedFailuresDetected > 0);
    CHECK(with.m.alternativesThatHelped > 0);
    CHECK(with.m.sameMistakeRepeated == 0);
    CHECK(without.sameEnd >= 75);
}

TEST(credit_assignment_blames_one_decision_and_keeps_uncertainty) {
    sim::Level level = sim::levelBasics();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg = isolatedConfig();
    Agent agent(cfg);
    agent.attach(&mem, &pat);
    auto r = runAttempt(agent, level); // never jumps: dies at the first spike
    CHECK(!r.completed);
    size_t blamed = countIf(mem.q, [](QEntry const& e) { return e.fails[0] + e.fails[1] > 0; });
    size_t passed = countIf(mem.q, [](QEntry const& e) { return e.passes[0] + e.passes[1] > 0; });
    size_t neither = countIf(mem.q, [](QEntry const& e) { return !e.hasEvidence(); });
    std::printf("    %zu states: %zu blamed, %zu passed, %zu without evidence | %s\n", mem.q.size(), blamed, passed, neither,
                agent.status().lastCredit.c_str());
    CHECK(blamed == 1); // one decision, not every decision before the death
    CHECK(passed > 0);
    CHECK(neither > 0); // the decisions right before the death: no claim either way
    // The blamed decision is "waiting", shortly before the spike.
    float maxPassedX = 0.f;
    for (auto const& s : agent.steps()) {
        auto const* e = mem.q.find(s.key);
        if (e && e->fails[0] > 0) {
            CHECK(s.action == 0);
            CHECK(s.x > r.endX - 3.f * kBlock);
            CHECK(e->q[0] < 0.f); // and its value dropped
        }
        if (e && e->passes[s.action] > 0) maxPassedX = std::max(maxPassedX, s.x);
    }
    CHECK(maxPassedX < r.endX - kBlock); // passes only well before the death
    CHECK(agent.status().metrics.pivotsByAnalysis == 1);

    // A completed attempt passes every decision and blames none.
    LevelMemory mem2 = freshLevel(level);
    PatternMemory pat2;
    Agent trained;
    trained.attach(&mem2, &pat2);
    auto t = train(trained, level, 400);
    CHECK(t.attemptsToComplete > 0);
    for (auto const& s : trained.steps()) CHECK(mem2.q.find(s.key)->passes[s.action] > 0);
}

TEST(failure_memory_survives_save_and_reload) {
    sim::Level level = sim::levelBasics();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    {
        Agent agent;
        agent.attach(&mem, &pat);
        train(agent, level, 400);
    }
    size_t withEvidence = countIf(mem.q, [](QEntry const& e) { return e.hasEvidence(); });
    CHECK(withEvidence > 0);

    std::string err;
    LevelMemory loaded;
    PatternMemory loadedPat;
    CHECK(storage::deserializeLevel(storage::serializeLevel(mem), loaded, &err));
    CHECK(storage::deserializePatterns(storage::serializePatterns(pat), loadedPat, &err));
    bool same = loaded.q.size() == mem.q.size();
    for (auto const& [k, e] : mem.q) {
        auto const* l = loaded.q.find(k);
        same = same && l && l->fails[0] == e.fails[0] && l->fails[1] == e.fails[1] && l->passes[0] == e.passes[0] &&
               l->passes[1] == e.passes[1] && l->streak[0] == e.streak[0] && l->streak[1] == e.streak[1];
    }
    CHECK(same);
    CHECK(countIf(loadedPat.q, [](QEntry const& e) { return e.hasEvidence(); }) > 0);

    // Files from version 1 (no evidence) still load; files from the future do not.
    LevelMemory old = freshLevel(level);
    QEntry& oe = old.q.getOrCreate(42);
    oe.q[1] = 2.f;
    oe.n[1] = 1;
    std::string v1 = storage::serializeLevel(old);
    auto pos = v1.find("\"version\":2");
    CHECK(pos != std::string::npos);
    std::string v3 = v1;
    v1.replace(pos, 11, "\"version\":1");
    v3.replace(pos, 11, "\"version\":3");
    LevelMemory fromV1, fromV3;
    CHECK(storage::deserializeLevel(v1, fromV1, &err));
    CHECK(fromV1.q.find(42) && fromV1.q.find(42)->q[1] == 2.f && !fromV1.q.find(42)->hasEvidence());
    CHECK(!storage::deserializeLevel(v3, fromV3, &err));

    // The reloaded memory produces the same decision from the same evidence.
    sim::World w(level);
    Observation o = w.observe();
    LearnerConfig cfg = isolatedConfig();
    QEntry& e = mem.q.getOrCreate(encodeLevelKey(o, false, cfg.cellSize));
    e.streak[e.q[1] > e.q[0] ? 1 : 0] = 5;
    LevelMemory again;
    CHECK(storage::deserializeLevel(storage::serializeLevel(mem), again, &err));
    PatternMemory noPat;
    Agent fresh(cfg);
    fresh.attach(&again, &noPat);
    fresh.beginAttempt(o, false);
    fresh.act(o);
    CHECK(fresh.status().source == DecisionSource::AvoidRepeat);
    fresh.abortAttempt(nullptr);
}

TEST(memory_is_bounded) {
    QTable t;
    for (uint64_t k = 0; k < 1000; ++k) {
        QEntry& e = t.getOrCreate(k);
        e.n[0] = static_cast<uint16_t>(k % 7 == 0 ? 50 : 0);
    }
    t.prune(200);
    CHECK(t.size() == 200);
    size_t kept = countIf(t, [](QEntry const& e) { return e.n[0] == 50; });
    CHECK(kept == 143); // every experienced state (1000/7 rounded up) survives
}

TEST(prior_reaches_unseen_states_but_never_overrides_own_experience) {
    sim::Level level = sim::levelBasics();
    sim::World w(level);
    Observation o = w.observe();
    LearnerConfig cfg = isolatedConfig();
    PatternMemory pat;
    QEntry& pe = pat.q.getOrCreate(encodePatternKey(o, false));
    pe.q[1] = 4.f; // in similar situations elsewhere, pressing worked
    pe.n[1] = 10;
    pe.q[0] = -2.f;
    pe.n[0] = 10;

    auto decide = [&](LevelMemory& mem) {
        Agent agent(cfg);
        agent.attach(&mem, &pat);
        agent.beginAttempt(o, false);
        bool h = agent.act(o);
        DecisionSource s = agent.status().source;
        agent.abortAttempt(nullptr);
        return std::make_pair(h, s);
    };
    // Unseen state: the prior decides.
    LevelMemory fresh = freshLevel(level);
    auto d1 = decide(fresh);
    CHECK(d1.first && d1.second == DecisionSource::Prior);
    // A state created before the prior was learned still gets it (live prior).
    LevelMemory created = freshLevel(level);
    created.q.getOrCreate(encodeLevelKey(o, false, cfg.cellSize));
    auto d2 = decide(created);
    CHECK(d2.first && d2.second == DecisionSource::Prior);
    // Own experience in this exact state wins over the prior.
    LevelMemory own = freshLevel(level);
    QEntry& e = own.q.getOrCreate(encodeLevelKey(o, false, cfg.cellSize));
    e.q[0] = 1.f;
    e.n[0] = 2;
    auto d3 = decide(own);
    CHECK(!d3.first && d3.second == DecisionSource::Policy);
}

TEST(physics_change_is_detected_and_corrected) {
    PredictionRig rig;
    warmUpMode(rig, GameMode::Cube);
    float g1 = rig.model.params(GameMode::Cube, false).accRelease;
    {
        GravityScale heavier(1.3f);
        float truth = sim::World::spec(GameMode::Cube).accRelease;
        warmUpMode(rig, GameMode::Cube, 3000, 11);
        float g2 = rig.model.params(GameMode::Cube, false).accRelease;
        Accuracy a = measureAccuracy(rig, GameMode::Cube);
        std::printf("    gravity before %.4f, after the change %.4f (truth %.4f); %u estimate(s) replaced; 32-step error %.2f\n",
                    g1, g2, truth, rig.model.regimeChanges(), a.meanErr);
        CHECK(rig.model.regimeChanges() > 0);
        CHECK(std::fabs(g2 - truth) < 0.01f);
        CHECK(a.samples > 10 && a.meanErr < 1.0);

        // The learning agent with prediction still completes a level under
        // the changed physics (single spikes: with 30% more gravity a jump
        // no longer clears three in a row). A death on a path that was
        // predicted safe is counted as a model error, not blamed via prediction.
        sim::Level level = sim::levelBasics();
        LevelMemory mem = freshLevel(level);
        PatternMemory pat;
        Agent agent;
        agent.attach(&mem, &pat);
        agent.setPredictor(&rig.predictor);
        auto r = train(agent, level, 300, &rig);
        std::printf("    basics under changed physics: completed after %d attempts (%u mispredicted deaths)\n", r.attemptsToComplete,
                    agent.status().metrics.mispredictedDeaths);
        CHECK(r.attemptsToComplete > 0);
    }
    // and back: the original physics are recovered too
    warmUpMode(rig, GameMode::Cube, 3000, 13);
    CHECK(std::fabs(rig.model.params(GameMode::Cube, false).accRelease - g1) < 0.01f);
}

TEST(experiments_replay_only_in_the_same_state) {
    // An experiment changes one thing. Around it, the reference attempt's
    // actions are reproduced only while the player is in exactly the same
    // state as then; once the change has made the situation different, the
    // decisions are made normally instead of being replayed by position.
    sim::Level level = sim::levelBasics();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent;
    agent.attach(&mem, &pat);
    runAttempt(agent, level);
    Experiment ex = agent.pendingExperiment();
    CHECK(ex.active);
    CHECK(ex.testFrom >= ex.fromCell && ex.testTo <= ex.toCell && ex.testFrom <= ex.testTo);
    runAttempt(agent, level);
    int reference = 0, tested = 0, independentInSpan = 0, violations = 0;
    for (auto const& s : agent.steps()) {
        int32_t cell = xCell(s.x, agent.config().cellSize);
        if (!ex.covers(cell)) continue;
        if (s.source == DecisionSource::Reference) {
            ++reference;
            if (s.key != ex.baseKeyAt(cell) || ex.tests(cell)) ++violations;
        } else if (s.source == DecisionSource::Experiment) {
            ++tested;
            if (!ex.tests(cell)) ++violations;
        } else if (!ex.tests(cell)) {
            ++independentInSpan;
        }
    }
    std::printf("    experiment span: %d reference decisions (same state), %d tested, %d decided normally\n", reference, tested,
                independentInSpan);
    CHECK(violations == 0);
    CHECK(tested > 0);
    CHECK(independentInSpan > 0); // after the change the states differ: no blind replay
}

TEST(newer_evidence_overrides_the_best_run) {
    // The best run is kept for inspection only. It is never written back into
    // the policy, so when later evidence says one of its decisions now fails,
    // that evidence wins.
    sim::Level level = sim::levelSpikes();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent;
    agent.attach(&mem, &pat);
    auto r = train(agent, level, 800);
    CHECK(r.attemptsToComplete > 0);
    CHECK(!mem.bestPath.empty());

    // (1) Values of states an attempt does not visit are left alone: an
    // early state of the best run is changed, then an attempt is played from
    // a checkpoint after it.
    PathStep target{};
    for (auto const& p : mem.bestPath) {
        if (p.x > 5.f * kBlock && p.action == 0) {
            target = p;
            break;
        }
    }
    QEntry& e = mem.q.getOrCreate(target.key);
    e.q[1] = e.q[0] + 7.f; // new preference, against the best run
    float before = e.q[1];
    {
        sim::World w(level);
        w.reset(20.f * kBlock);
        agent.beginAttempt(w.observe(), true);
        for (int t = 0; t < 100000 && !w.dead() && !w.complete(); ++t) w.step(agent.act(w.observe()));
        if (w.dead()) agent.onDeath(w.observe(), w.death());
        else if (w.complete()) agent.onComplete(w.observe());
    }
    CHECK(mem.q.find(target.key)->q[1] == before);
    mem.q.find(target.key)->q[1] = mem.q.find(target.key)->q[0] - 7.f; // undo, so the next check follows the best run

    // (2) A best-run decision that newer evidence says now fails is not repeated.
    PathStep press{};
    for (auto const& p : mem.bestPath) {
        if (p.action == 1 && p.x > 10.f * kBlock) {
            press = p;
            break;
        }
    }
    QEntry& pe = mem.q.getOrCreate(press.key);
    std::printf("    best-run press at x=%.0f: q %.2f/%.2f, passes %u/%u, fails %u/%u, streak %u/%u\n", press.x, pe.q[0], pe.q[1],
                pe.passes[0], pe.passes[1], pe.fails[0], pe.fails[1], pe.streak[0], pe.streak[1]);
    pe.streak[1] = static_cast<uint8_t>(std::max(2, pe.streak[0] + 1));
    pe.fails[1] += pe.streak[1];
    LearnerConfig replayCfg;
    replayCfg.learningEnabled = false;
    Agent check(replayCfg);
    check.attach(&mem, &pat);
    runAttempt(check, level);
    bool pressedThere = false, avoided = false;
    for (auto const& s : check.steps()) {
        if (s.key != press.key) continue;
        pressedThere = pressedThere || s.action == 1;
        avoided = avoided || s.source == DecisionSource::AvoidRepeat;
    }
    std::printf("    with new evidence against it: %s\n", pressedThere ? "pressed anyway" : "not pressed (alternative tested)");
    CHECK(!pressedThere);
    CHECK(avoided);
}

TEST(learning_metrics_regression) {
    // Attempts to first completion plus the learning-quality counters, for
    // every simulator level and three seeds, without and with prediction.
    std::printf("    %-7s %-5s %8s %6s %7s %9s %8s %7s %6s %6s %7s\n", "level", "pred", "attempts", "regr", "repeat", "detected",
                "tried", "helped", "vetoed", "known", "blamed");
    int failures = 0;
    for (auto const& level : sim::allLevels()) {
        for (int pred = 0; pred < 2; ++pred) {
            for (uint32_t seed = 1; seed <= 3; ++seed) {
                LevelMemory mem = freshLevel(level);
                PatternMemory pat;
                LearnerConfig cfg;
                cfg.seed = seed * 7919u;
                Agent agent(cfg);
                agent.attach(&mem, &pat);
                PredictionRig rig;
                if (pred) {
                    warmUp(rig);
                    agent.setPredictor(&rig.predictor);
                }
                auto r = train(agent, level, 1500, pred ? &rig : nullptr);
                int regressions = 0;
                float best = 0.f;
                for (auto const& a : r.attempts) {
                    if (a.endX < best - 3.f * kBlock) ++regressions;
                    best = std::max(best, a.endX);
                }
                auto const& m = agent.status().metrics;
                int deaths = static_cast<int>(r.attempts.size()) - (r.attemptsToComplete > 0 ? 1 : 0);
                std::printf("    %-7s %-5s %8d %6d %7u %9u %8u %7u %6u %6u %4u/%-3d\n", level.name.c_str(), pred ? "yes" : "no",
                            r.attemptsToComplete, regressions, m.sameMistakeRepeated, m.repeatedFailuresDetected,
                            m.alternativesTried, m.alternativesThatHelped, m.alternativesVetoed, m.knownGoodKept > 0 ? 1u : 0u,
                            m.pivotsByPrediction + m.pivotsByAnalysis, deaths);
                if (r.attemptsToComplete <= 0) ++failures;
                CHECK_MSG(r.attemptsToComplete > 0, "%s seed %u pred %d not completed", level.name.c_str(), seed, pred);
                CHECK_MSG(regressions * 4 <= static_cast<int>(r.attempts.size()), "%s seed %u: %d regressions in %zu",
                          level.name.c_str(), seed, regressions, r.attempts.size());
                // Deaths on a decision already known to fail there stay rare.
                CHECK_MSG(m.sameMistakeRepeated * 5 <= static_cast<uint32_t>(std::max(deaths, 5)), "%s seed %u: %u of %d deaths repeated a known mistake",
                          level.name.c_str(), seed, m.sameMistakeRepeated, deaths);
            }
        }
    }
    std::printf("    (repeat = deaths on a decision already known to fail there; known = known-good overrides used)\n");
    CHECK(failures == 0);
}

// ---------------------------------------------------------------------------
// Prediction must control the executed input (regression tests for "the
// overlay predicted the crash but the AI kept its input and died")

namespace {
// A learned policy that insists on one action: before every decision the
// values of the current state are set to prefer `want`, with experience on
// both actions (so the policy has an opinion and the planner is not asked).
// Every step also checks that the overlay shows the executed action's path
// and that the safety record matches the input really applied.
struct StubbornRun {
    bool died = false;
    bool completed = false;
    float endX = 0.f;
    int overridden = 0;          // ticks where the executed input differed from what the policy wanted
    int firstOverrideTick = -1;
    int overriddenAfterCalm = 0; // ... after `calmAfterX`, where nothing is in the way any more
    int deathTick = -1;
    int overlayMismatches = 0;   // overlay main path is not the executed action's branch
    int recordMismatches = 0;    // safety record disagrees with the applied input / its own verdict
    int overrideRecords = 0;
    int presses = 0;             // presses actually applied to the game
    int pressesSkipped = 0;      // decisions where a proposed press was dropped as unnecessary
    std::vector<uint8_t> applied; // button state actually applied to the game, per tick
    std::vector<uint8_t> wanted;
};

StubbornRun runStubborn(Agent& agent, LevelMemory& mem, sim::Level const& level, PredictionRig& rig,
                        std::function<int(Observation const&, bool)> const& want, float calmAfterX = 1e9f, int maxTicks = 20000) {
    StubbornRun r;
    sim::World w(level);
    rig.held = false;
    agent.beginAttempt(rig.observe(w), false);
    for (int t = 0; t < maxTicks; ++t) {
        Observation o = rig.observe(w);
        rig.predictor.observe(o, rig.held);
        int a = want(o, agent.holding()); // -1: no opinion, the AI decides
        if (a >= 0) {
            QEntry& e = mem.q.getOrCreate(encodeLevelKey(o, agent.holding(), agent.config().cellSize));
            e.q[a] = 5.f;
            e.q[1 - a] = 0.f;
            e.n[0] = e.n[1] = 50;
        }
        bool hold = agent.act(o);
        auto const& rec = agent.status().safety;
        if (rec.tick == o.tick) {
            if ((rec.executed != 0) != hold) ++r.recordMismatches;
            if (rec.verdict == SafetyVerdict::Overridden && rec.executed == rec.proposed) ++r.recordMismatches;
            if ((rec.verdict == SafetyVerdict::Clear || rec.verdict == SafetyVerdict::Kept) && rec.executed != rec.proposed) ++r.recordMismatches;
            if (rec.verdict == SafetyVerdict::NoInputNeeded && (rec.executed != 0 || rec.proposed != 1)) ++r.recordMismatches;
            if (rec.verdict == SafetyVerdict::NoInputNeeded) ++r.pressesSkipped;
            if (rec.verdict == SafetyVerdict::Overridden) ++r.overrideRecords;
            OverlayGeometry g;
            if (buildDecisionOverlay(rig.predictor, rec, o.tick, false, o.obstacles, g, &agent.plans().plan())) {
                bool const mainIsExecuted = rec.proposedSource == DecisionSource::Plan
                                                ? agent.plans().holdFor(o.tick) == (hold ? 1 : 0)
                                                : (!rig.predictor.executedPath().holds.empty() &&
                                                   (rig.predictor.executedPath().holds.front() != 0) == hold);
                if (g.selectedPress != hold || !mainIsExecuted) ++r.overlayMismatches;
                bool saysOverride = g.label.find("SAFETY OVERRIDE") != std::string::npos;
                if (saysOverride != (rec.verdict == SafetyVerdict::Overridden)) ++r.overlayMismatches;
            }
        }
        r.wanted.push_back(static_cast<uint8_t>(a < 0 ? (hold ? 1 : 0) : a));
        if (hold && (r.applied.empty() || r.applied.back() == 0)) {
            ++r.presses;
            if (std::getenv("AIGD_PRESS_DEBUG")) {
                auto const& pl = agent.plans().plan();
                std::printf("      press t=%d x=%.1f %s src=%s plan(k=%d hold=%d second=%d stage=%s age=%d) | %s\n", o.tick, o.x,
                            o.onGround ? "ground" : "air", decisionSourceName(agent.status().safety.proposedSource), pl.params.pressAt,
                            pl.params.holdFor, pl.params.secondAt, searchStageName(pl.stage), agent.plans().age(o.tick),
                            agent.status().safety.reason.c_str());
            }
        }
        r.applied.push_back(hold ? 1 : 0);
        if (a >= 0 && (hold ? 1 : 0) != a) {
            ++r.overridden;
            if (r.firstOverrideTick < 0) r.firstOverrideTick = t;
            if (o.x > calmAfterX) ++r.overriddenAfterCalm;
        }
        rig.held = hold;
        w.step(hold);
        if (w.dead()) {
            r.died = true;
            r.deathTick = t;
            r.endX = w.x();
            if (std::getenv("AIGD_SAFETY_DEBUG")) {
                auto const& log = agent.safetyLog();
                for (size_t i = log.size() > 30 ? log.size() - 30 : 0; i < log.size(); ++i) {
                    auto const& s = log[i];
                    std::printf("      t=%d x=%.1f y=%.1f vy=%.2f held=%d R:%s/%d/k%d P:%s/%d/k%d keep:%s/%d prop=%d exe=%d %s cur=%d | %s\n", s.tick,
                                s.x, s.y, s.vy, s.holdingBefore, certaintyName(s.outcome[0]), s.ttc[0], s.keepFor[0],
                                certaintyName(s.outcome[1]), s.ttc[1], s.keepFor[1], certaintyName(s.keep), s.keepTTC, s.proposed,
                                s.executed, safetyVerdictName(s.verdict), s.predictionCurrent, s.reason.c_str());
                }
            }
            agent.onDeath(w.observe(), w.death());
            return r;
        }
        if (w.complete()) {
            r.completed = true;
            r.endX = w.x();
            agent.onComplete(w.observe());
            return r;
        }
    }
    r.endX = w.x();
    agent.abortAttempt(nullptr);
    return r;
}

sim::Level shipCorridor(char const* name) {
    sim::Level l;
    l.name = name;
    l.modePortal(1, GameMode::Ship);
    l.block(3, 7.f, 60.f, 1); // ceiling at 7 blocks
    return l;
}

void reportRun(char const* what, StubbornRun const& r) {
    std::printf("    %s: %s at x=%.0f; input overridden on %d ticks (first at tick %d, %d after the danger), %d overlay / %d record mismatches\n",
                what, r.completed ? "completed" : "DIED", r.endX, r.overridden, r.firstOverrideTick, r.overriddenAfterCalm,
                r.overlayMismatches, r.recordMismatches);
}

void checkConsistent(StubbornRun const& r) {
    CHECK(r.overlayMismatches == 0); // test G: overlay shows what is executed
    CHECK(r.recordMismatches == 0);  // diagnostics never claim what did not happen
}
} // namespace

TEST(control_A_cube_waiting_into_a_spike_jumps_instead) {
    // The policy insists on not jumping; waiting will hit the spike, jumping
    // (in time) clears it. The controller must jump.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = spikeLevel("control-a", {14.f}, 24.f);
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runStubborn(agent, mem, level, rig, [](Observation const&, bool) { return 0; }, 15.5f * kBlock);
    reportRun("cube, policy 'never jump'", r);
    CHECK(r.completed);
    CHECK(r.overridden > 0);
    CHECK(r.overriddenAfterCalm == 0); // not latched: the policy is back in control after the spike
    checkConsistent(r);
}

TEST(control_A2_cube_jumping_into_a_spike_waits_instead) {
    // The policy insists on jumping whenever on the ground; a jump right now
    // lands on the spike, a later one clears it.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = spikeLevel("control-a2", {4.4f}, 14.f);
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runStubborn(agent, mem, level, rig, [](Observation const& o, bool) { return o.onGround ? 1 : 0; });
    reportRun("cube, policy 'always jump'", r);
    CHECK(r.completed);
    CHECK(r.overridden > 0);
    checkConsistent(r);
}

TEST(control_B_ship_holding_into_ceiling_spikes_releases_in_time) {
    // Holding is safe at first; as the ship approaches spikes under the
    // ceiling, keeping the button held becomes fatal. The controller must
    // release before that becomes unavoidable.
    PredictionRig rig;
    warmUpMode(rig, GameMode::Ship);
    sim::Level level = shipCorridor("control-b");
    for (float b = 22.f; b < 30.f; b += 1.f) level.spike(b, 6.f);
    level.finish(40.f);
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runStubborn(agent, mem, level, rig, [](Observation const& o, bool) { return o.mode == GameMode::Ship ? 1 : 0; },
                         31.f * kBlock);
    reportRun("ship, policy 'hold' toward ceiling spikes", r);
    CHECK(r.completed);
    // Test F: the held button is really released, against the policy.
    bool releasedAfterHolding = false;
    for (size_t t = 1; t < r.applied.size(); ++t)
        if (r.applied[t - 1] == 1 && r.applied[t] == 0 && r.wanted[t] == 1) releasedAfterHolding = true;
    CHECK(releasedAfterHolding);
    CHECK(r.overriddenAfterCalm == 0);
    checkConsistent(r);
}

TEST(control_C_ship_flying_low_into_a_wall_climbs) {
    // The policy keeps the ship down; a wall rises from the floor ahead.
    PredictionRig rig;
    warmUpMode(rig, GameMode::Ship);
    sim::Level level = shipCorridor("control-c");
    level.block(24.f, 0.f, 1.f, 3.f);
    level.finish(40.f);
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runStubborn(agent, mem, level, rig, [](Observation const&, bool) { return 0; });
    reportRun("ship, policy 'release' toward a wall", r);
    CHECK(r.completed);
    bool pressed = false;
    for (size_t t = 0; t < r.applied.size(); ++t)
        if (r.applied[t] == 1 && r.wanted[t] == 0) pressed = true;
    CHECK(pressed); // the actual input changed, against the policy
    checkConsistent(r);
}

TEST(control_C2_ship_through_a_gap) {
    // Wall from the floor and a block from the ceiling: a gap in the middle.
    // Policy alternates stubbornly between "always hold" and "always release"
    // every 120 ticks; neither is a way through.
    PredictionRig rig;
    warmUpMode(rig, GameMode::Ship);
    sim::Level level = shipCorridor("control-c2");
    level.block(24.f, 0.f, 1.f, 2.f);
    level.block(24.f, 5.f, 1.f, 2.f);
    level.block(34.f, 0.f, 1.f, 3.f);
    level.finish(45.f);
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runStubborn(agent, mem, level, rig, [](Observation const& o, bool) { return (o.tick / 120) % 2; });
    reportRun("ship, alternating policy through a gap", r);
    CHECK(r.completed);
    checkConsistent(r);
}

TEST(control_D_stale_prediction_cannot_authorise_the_old_action) {
    // Advice is computed for one state; then, in the same tick, the state
    // is different. The answer must be for the new state, and so must the
    // decision built on it.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = spikeLevel("control-d", {8.f}, 14.f);
    sim::World w(level);
    Observation far = rig.observe(w);  // spike 7.5 blocks ahead: waiting is fine for a long time
    rig.predictor.observe(far, false);
    auto const adv1 = rig.predictor.advise(far, false);
    Observation near = far;            // same tick, right before the spike
    near.x = 8.f * kBlock - 40.f;
    near.hazard.dx = 40.f;
    auto const adv2 = rig.predictor.advise(near, false);
    float firstX = rig.predictor.releasePath().points.empty() ? 0.f : rig.predictor.releasePath().points.front().x;
    std::printf("    far: wait possible for %d steps | near, same tick: %d steps; path starts at x=%.1f (state x=%.1f)\n",
                adv1.releaseKeepForSafe, adv2.releaseKeepForSafe, firstX, near.x);
    CHECK(adv2.computedTick == near.tick && !adv2.reused);
    CHECK(std::fabs(firstX - near.x) < 3.f);
    CHECK(adv2.releaseKeepForSafe < adv1.releaseKeepForSafe);

    // Through the agent: decide at `far`, then at `near` in the same tick.
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    agent.beginAttempt(far, false);
    agent.act(far);
    agent.act(near);
    auto const& rec = agent.status().safety;
    CHECK(rec.x == near.x && rec.predictionCurrent);
    CHECK(rec.keepFor[0] == adv2.releaseKeepFor);
    agent.abortAttempt(nullptr);
}

TEST(control_E_no_viable_alternative_is_reported_honestly) {
    // Directly before a spike, too close to jump over it: every candidate
    // collides. The controller must say so and not claim a safe path.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = spikeLevel("control-e", {8.f}, 14.f);
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    sim::World w(level);
    w.reset(8.f * kBlock - 28.f);
    Observation o = rig.observe(w);
    rig.predictor.observe(o, false);
    agent.beginAttempt(o, false);
    agent.act(o);
    auto const& rec = agent.status().safety;
    std::printf("    press %s (hit %d), release %s (hit %d) | %s | %s\n", certaintyName(rec.outcome[1]), rec.ttc[1],
                certaintyName(rec.outcome[0]), rec.ttc[0], safetyVerdictName(rec.verdict), rec.reason.c_str());
    CHECK(rec.outcome[0] == Certainty::Collision && rec.outcome[1] == Certainty::Collision);
    CHECK(rec.verdict == SafetyVerdict::NoSafeOption);
    CHECK(!rec.alternativeAvailable);
    CHECK(rec.reason.find("No safe option") != std::string::npos);
    OverlayGeometry g;
    buildDecisionOverlay(rig.predictor, rec, o.tick, false, o.obstacles, g);
    std::printf("    overlay: %s\n", g.label.c_str());
    CHECK(g.label.find("NO SAFE OPTION") != std::string::npos);
    CHECK(g.label.find("safe -") == std::string::npos);
    CHECK(g.outcome == Certainty::Collision);
    agent.abortAttempt(nullptr);
}

TEST(control_cost_per_step_with_safety_layer) {
    // Wall-clock cost of a whole decision (observe, predict, decide, safety
    // check) per physics step, for the learning agent with prediction.
    for (GameMode m : {GameMode::Cube, GameMode::Ship, GameMode::Wave, GameMode::Spider}) {
        PredictionRig rig;
        warmUpMode(rig, m);
        sim::Level level = sim::modeCourse(m);
        LevelMemory mem = freshLevel(level);
        PatternMemory pat;
        Agent agent;
        agent.attach(&mem, &pat);
        agent.setPredictor(&rig.predictor);
        sim::World w(level);
        rig.held = false;
        agent.beginAttempt(rig.observe(w), false);
        auto t0 = std::chrono::steady_clock::now();
        int ticks = 0;
        uint64_t sims0 = rig.predictor.totalSimulations();
        for (; ticks < 100000 && !w.dead() && !w.complete(); ++ticks) {
            Observation o = rig.observe(w);
            rig.predictor.observe(o, rig.held);
            bool hold = agent.act(o);
            rig.held = hold;
            w.step(hold);
        }
        double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / std::max(1, ticks);
        std::printf("    %-6s %s: %.0f us and %.1f simulations per physics step (%d steps; %.1f%% of a core at 1x, %.0f%% at 4x)\n",
                    gameModeName(m), w.complete() ? "completed" : "died", us,
                    double(rig.predictor.totalSimulations() - sims0) / std::max(1, ticks), ticks, us * 240.0 / 1e4, us * 960.0 / 1e4);
        if (w.dead()) agent.onDeath(w.observe(), w.death());
        else if (w.complete()) agent.onComplete(w.observe());
        CHECK(us < 2000.0);
    }
}

TEST(control_H_learning_still_works_with_the_safety_layer) {
    // The safety layer changes what is executed, not how learning works: the
    // agent with prediction still learns the levels, records outcome
    // evidence per state, and keeps it through save/reload.
    for (auto const& level : {sim::levelShip(), sim::levelBlocks()}) {
        PredictionRig rig;
        warmUp(rig);
        LevelMemory mem = freshLevel(level);
        PatternMemory pat;
        Agent agent;
        agent.attach(&mem, &pat);
        agent.setPredictor(&rig.predictor);
        auto r = train(agent, level, 300, &rig);
        size_t evidence = countIf(mem.q, [](QEntry const& e) { return e.hasEvidence(); });
        std::printf("    %-7s with prediction: completed after %d attempts, %zu states with evidence\n", level.name.c_str(),
                    r.attemptsToComplete, evidence);
        CHECK(r.attemptsToComplete > 0);
        CHECK(evidence > 0);
        LevelMemory loaded;
        std::string err;
        CHECK(storage::deserializeLevel(storage::serializeLevel(mem), loaded, &err));
        CHECK(countIf(loaded.q, [](QEntry const& e) { return e.hasEvidence(); }) == evidence);
    }
}

// ---------------------------------------------------------------------------
// Decision pipeline (v0.2.2): no unnecessary jumps, prediction quality checked
// against reality before it is trusted, learning can remove a jump

namespace {
struct SimHooks {
    SimHooks(int delay, float noise, float inflate) {
        sim::World::s_inputDelay = delay;
        sim::World::s_noise = noise;
        sim::World::s_hazardInflate = inflate;
    }
    ~SimHooks() {
        sim::World::s_inputDelay = 0;
        sim::World::s_noise = 0.f;
        sim::World::s_hazardInflate = 0.f;
    }
};

// Ground made of blocks flush with the floor, with hazards far above the path:
// nothing on it requires a jump.
sim::Level flatBlocks() {
    sim::Level l;
    l.name = "flat-blocks";
    for (float b = 4.f; b < 60.f; b += 1.f) l.block(b, -1.f, 1.f, 1.f);
    for (float b = 20.f; b < 50.f; b += 6.f) l.spike(b, 4.f);
    l.finish(64.f);
    return l;
}

// Spikes hanging just above the cube's head: walking under them is safe, a
// jump hits them.
sim::Level hangingSpikes() {
    sim::Level l;
    l.name = "hanging";
    for (float b = 14.f; b < 20.f; b += 1.f) l.spike(b, 1.1f);
    l.finish(30.f);
    return l;
}
} // namespace

TEST(decision_A_no_unnecessary_jump_by_the_planner) {
    // Nothing needs a jump. With no learned opinion the trajectory planner
    // decides: it must not jump just because a jump is possible (before, the
    // planner scored a jump's extra distance from blocks and hazards higher).
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = flatBlocks();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runStubborn(agent, mem, level, rig, [](Observation const&, bool) { return -1; });
    std::printf("    planner over flat blocks: %s, %d presses\n", r.completed ? "completed" : "DIED", r.presses);
    CHECK(r.completed);
    CHECK(r.presses == 0);
    checkConsistent(r);
}

TEST(decision_B_policy_jump_dropped_when_not_needed) {
    // The learned policy wants to jump at every chance. Not jumping is
    // predicted clear the whole way, so no jump is executed, and each dropped
    // press is recorded with the reason from the candidate evaluation.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = flatBlocks();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runStubborn(agent, mem, level, rig, [](Observation const& o, bool) { return o.onGround ? 1 : 0; });
    std::printf("    policy 'always jump' over flat blocks: %s, %d presses executed, %d proposed presses dropped | %s\n",
                r.completed ? "completed" : "DIED", r.presses, r.pressesSkipped, agent.status().safety.reason.c_str());
    CHECK(r.completed);
    CHECK(r.presses == 0);
    CHECK(r.pressesSkipped > 0);
    checkConsistent(r);
    // the explanation is generated from the evaluation, not a fixed text
    bool found = false;
    for (auto const& s : agent.steps()) found = found || s.source == DecisionSource::Economy;
    CHECK(found);
}

TEST(decision_C_jumps_when_needed_and_only_then) {
    // Same 'always jump' policy on a level with spikes on the ground: the
    // needed jumps happen (the level is completed), the others are dropped.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = sim::levelSpikes();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runStubborn(agent, mem, level, rig, [](Observation const& o, bool) { return o.onGround ? 1 : 0; });
    // jumps needed: one per spike group
    std::printf("    policy 'always jump' on spikes: %s, %d presses executed (9 spike groups), %d proposed presses dropped\n",
                r.completed ? "completed" : "DIED", r.presses, r.pressesSkipped);
    CHECK(r.completed);
    CHECK(r.presses >= 9 && r.presses <= 12);
    CHECK(r.pressesSkipped > 0);
    checkConsistent(r);

    // And the planner alone on the same level: also only the needed jumps.
    PredictionRig rig2;
    warmUp(rig2);
    LevelMemory mem2 = freshLevel(level);
    Agent planner(isolatedConfig());
    planner.attach(&mem2, &pat);
    planner.setPredictor(&rig2.predictor);
    auto p = runStubborn(planner, mem2, level, rig2, [](Observation const&, bool) { return -1; });
    std::printf("    planner on spikes: %s, %d presses\n", p.completed ? "completed" : "DIED", p.presses);
    CHECK(p.completed);
    CHECK(p.presses >= 9 && p.presses <= 12);
}

TEST(decision_D_learning_removes_a_harmful_jump) {
    // Without prediction: the policy has learned to jump before low hanging
    // spikes (e.g. from an earlier version of the level). Walking under them
    // is safe; the jump kills. Learning must find out that the jump itself is
    // the mistake and stop making it.
    sim::Level level = hangingSpikes();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg;
    Agent agent(cfg);
    agent.attach(&mem, &pat);
    {
        // The old habit: prefer pressing in every state of the walk before the spikes.
        sim::World w(level);
        while (!w.dead() && !w.complete() && w.x() < 13.5f * kBlock) {
            Observation o = w.observe();
            if (o.x > 11.f * kBlock) {
                QEntry& e = mem.q.getOrCreate(encodeLevelKey(o, false, cfg.cellSize));
                e.q[1] = 2.f;
                e.q[0] = 0.f;
                e.n[0] = e.n[1] = 3;
            }
            w.step(false);
        }
    }
    auto r = train(agent, level, 60);
    bool removedTried = false;
    for (auto const& h : mem.history) {
        removedTried = removedTried || h.change.find("Removed the press") != std::string::npos;
        if (std::getenv("AIGD_DIAG_D"))
            std::printf("      #%u end %.0f %s | %s\n", h.number, h.endX, failureKindName(h.kind), h.change.c_str());
    }
    std::printf("    hanging spikes: completed after %d attempts; 'remove the press' experiment %s\n", r.attemptsToComplete,
                removedTried ? "used" : "not used");
    CHECK(r.attemptsToComplete > 0);
    CHECK(r.attemptsToComplete <= 10);
    // the learned policy no longer jumps there: replay with learning off
    LearnerConfig replay;
    replay.learningEnabled = false;
    Agent check(replay);
    check.attach(&mem, &pat);
    CHECK(runAttempt(check, level).completed);
    // ...and that survives a save/reload
    LevelMemory loaded;
    std::string err;
    CHECK(storage::deserializeLevel(storage::serializeLevel(mem), loaded, &err));
    Agent fresh(replay);
    fresh.attach(&loaded, &pat);
    CHECK(runAttempt(fresh, level).completed);
}

TEST(decision_E_unreliable_model_is_detected_and_not_obeyed) {
    // The game moves in a way the model cannot predict (random jitter in the
    // air). The measured drift must exceed the limit, the prediction must not
    // be trusted, and the AI must say so instead of acting on it.
    SimHooks hooks(0, 4.f, 0.f);
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = sim::levelBasics();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runStubborn(agent, mem, level, rig, [](Observation const&, bool) { return 0; }, 1e9f, 900);
    auto const& st = agent.status();
    std::printf("    jittery game: drift %.3f/step, trusted %s; %u decisions trusted, %u untrusted | %s\n", rig.model.driftPerTick(GameMode::Cube),
                rig.predictor.trusted(GameMode::Cube, false) ? "yes" : "no", st.decisionsTrusted, st.decisionsUntrusted,
                st.safety.reason.c_str());
    CHECK(!rig.predictor.trusted(GameMode::Cube, false));
    CHECK(st.decisionsUntrusted > 0);
    // Overrides only ever came from trusted predictions (early in the run,
    // before the drift measurement had caught up), never from untrusted ones.
    int untrustedOverrides = 0;
    for (auto const& rec : agent.safetyLog())
        if (rec.verdict == SafetyVerdict::Overridden && !rec.trusted) ++untrustedOverrides;
    CHECK(untrustedOverrides == 0);
    CHECK(st.safety.verdict == SafetyVerdict::NoPrediction);
    (void)r;
    CHECK(rig.predictor.trustReason(GameMode::Cube, false).find("not trusted") != std::string::npos);
    OverlayGeometry g;
    buildDecisionOverlay(rig.predictor, st.safety, st.safety.tick, false, {}, g);
    std::printf("    overlay: %s\n", g.label.c_str());
}

TEST(decision_F_hitbox_mismatch_does_not_destroy_trust) {
    // The observed hazards are 2 units bigger than what really kills (as
    // suspected in game). Walking under spikes with 1 unit of real headroom
    // survives; the model sees an overlap. The accuracy measurement compares
    // movement only, so that does not count as a prediction error (before, it
    // counted as a 20-unit error each time and destroyed trust).
    SimHooks hooks(0, 0.f, 2.f);
    PredictionRig rig;
    warmUp(rig);
    float const before = rig.model.driftPerTick(GameMode::Cube);
    sim::Level level;
    level.name = "graze";
    for (float b = 6.f; b < 60.f; b += 1.f) level.spike(b, 31.f / kBlock - 0.1f); // hitbox bottom at 31: 1 unit above the head
    level.finish(64.f);
    sim::World w(level);
    rig.held = false;
    for (int t = 0; t < 1500 && !w.dead() && !w.complete(); ++t) {
        Observation o = rig.observe(w);
        rig.predictor.observe(o, false);
        w.step(false);
    }
    std::printf("    walking 1 unit under spikes the model sees 2 units bigger: %s, drift %.3f -> %.3f/step, trusted %s\n",
                w.dead() ? "DIED" : "alive", before, rig.model.driftPerTick(GameMode::Cube),
                rig.predictor.trusted(GameMode::Cube, false) ? "yes" : "no");
    CHECK(!w.dead());
    CHECK(rig.model.driftPerTick(GameMode::Cube) < 0.05f);
    CHECK(rig.predictor.trusted(GameMode::Cube, false));
}

TEST(decision_G_input_latency_is_measured_and_corrected) {
    // The game applies the button one step later than the model assumes. The
    // predictor must find that out from real play and plan with it.
    SimHooks hooks(1, 0.f, 0.f);
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = sim::levelSpikes();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent;
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = train(agent, level, 100, &rig);
    std::printf("    1-step input delay: estimated latency %d (re-simulation error %.2f / %.2f / %.2f for 0/1/2, %u windows); "
                "spikes completed after %d attempts\n",
                rig.predictor.inputLatency(), rig.predictor.latencyError(0), rig.predictor.latencyError(1), rig.predictor.latencyError(2),
                rig.predictor.latencySamples(), r.attemptsToComplete);
    CHECK(rig.predictor.inputLatency() == 1);
    CHECK(r.attemptsToComplete > 0);

    // Without a delay the estimate stays at 0.
    SimHooks none(0, 0.f, 0.f);
    PredictionRig rig0;
    warmUp(rig0);
    LevelMemory mem0 = freshLevel(level);
    Agent agent0;
    agent0.attach(&mem0, &pat);
    agent0.setPredictor(&rig0.predictor);
    train(agent0, level, 20, &rig0);
    CHECK(rig0.predictor.inputLatency() == 0);
}

// ---------------------------------------------------------------------------
// Game-like observation timing: in Geometry Dash the AI is called once per
// 1-3 physics steps (measured from real traces), not on every step.

namespace {
struct GapPattern {
    uint32_t state;
    int next() {
        state = state * 1664525u + 1013904223u;
        uint32_t r = (state >> 16) % 10;
        return r < 5 ? 1 : (r < 9 ? 2 : 3); // about the in-game mix: mostly 1-2, sometimes 3
    }
};

// Observes like the mod: ticks from a StepClock (true physics steps), or -
// as before the fix - one tick per call.
struct GameLikeObserver {
    PredictionRig& rig;
    bool naiveTicks = false;
    StepClock clock;
    int32_t naive = 0;
    Observation observe(sim::World const& w) {
        Observation o = rig.observe(w);
        clock.advance(o.x, o.speedClass());
        o.tick = naiveTicks ? ++naive : clock.tick();
        return o;
    }
};

void warmUpGaps(PredictionRig& rig, bool naiveTicks, uint32_t seed = 3) {
    sim::Level flat;
    flat.name = "flat";
    flat.finish(400);
    sim::World w(flat);
    GameLikeObserver ob{rig, naiveTicks};
    GapPattern gaps{seed};
    rig.held = false;
    for (int i = 0; i < 1600 && !w.complete(); ++i) {
        Observation o = ob.observe(w);
        rig.predictor.observe(o, rig.held);
        bool hold = (i % 90) == 0; // a tap every ~90 calls
        rig.held = hold;
        for (int k = gaps.next(); k > 0; --k) w.step(hold);
    }
}

AttemptResult runAttemptGaps(Agent& agent, sim::Level const& level, PredictionRig& rig, bool naiveTicks, uint32_t seed) {
    sim::World w(level);
    GameLikeObserver ob{rig, naiveTicks};
    GapPattern gaps{seed};
    rig.held = false;
    agent.beginAttempt(ob.observe(w), false);
    for (int i = 0; i < 60000; ++i) {
        Observation o = ob.observe(w);
        rig.predictor.observe(o, rig.held);
        bool hold = agent.act(o);
        rig.held = hold;
        for (int k = gaps.next(); k > 0 && !w.dead() && !w.complete(); --k) w.step(hold);
        if (w.dead()) {
            Observation last = ob.observe(w);
            agent.onDeath(last, w.death());
            return {last.progress(), w.x(), false};
        }
        if (w.complete()) {
            agent.onComplete(ob.observe(w));
            return {1.f, w.x(), true};
        }
    }
    agent.abortAttempt(nullptr);
    return {w.observe().progress(), w.x(), false};
}
} // namespace

TEST(step_clock_counts_physics_steps) {
    StepClock c;
    float const vx = unitsPerSecond(1) / 240.f;
    CHECK(c.advance(100.f, 1) == StepClock::kDiscontinuity);
    int32_t t0 = c.tick();
    CHECK(c.advance(100.f + vx, 1) == 1);
    CHECK(c.advance(100.f + 3.f * vx, 1) == 2);
    CHECK(c.advance(100.f + 3.f * vx, 1) == 0);  // same step again: nothing to do
    CHECK(c.advance(100.f + 6.f * vx, 1) == 3);
    CHECK(c.tick() == t0 + 6);
    CHECK(c.advance(50.f, 1) == StepClock::kDiscontinuity); // respawn behind
    CHECK(c.tick() > t0 + 6 + 8);
}

TEST(game_like_frame_gaps_keep_predictions_trusted) {
    // Before the fix (one tick per call), the model learned a horizontal
    // speed that is too high and the accuracy check compared 32 "steps" with
    // more real steps: drift above the limit, predictions never trusted -
    // what the in-game log showed (drift 0.25, 0 trusted decisions).
    PredictionRig naive;
    warmUpGaps(naive, true);
    float const nominal = unitsPerSecond(1) / 240.f;
    std::printf("    one tick per call (old): x speed %.4f (true %.4f), drift %.3f/step, trusted %s\n", naive.model.vxPerTick(1), nominal,
                naive.model.driftPerTick(GameMode::Cube), naive.predictor.trusted(GameMode::Cube, false) ? "yes" : "no");
    CHECK(naive.model.vxPerTick(1) > nominal * 1.1f);
    CHECK(!naive.predictor.trusted(GameMode::Cube, false));

    // With real physics steps counted, the same play calibrates correctly and
    // the predictions are trusted.
    PredictionRig rig;
    warmUpGaps(rig, false);
    std::printf("    real steps (StepClock):  x speed %.4f (true %.4f), drift %.3f/step, trusted %s, %.2f steps per call\n",
                rig.model.vxPerTick(1), nominal, rig.model.driftPerTick(GameMode::Cube),
                rig.predictor.trusted(GameMode::Cube, false) ? "yes" : "no", rig.predictor.decisionInterval());
    CHECK(std::fabs(rig.model.vxPerTick(1) - nominal) < nominal * 0.01f);
    CHECK(rig.model.driftPerTick(GameMode::Cube) < 0.05f);
    CHECK(rig.predictor.trusted(GameMode::Cube, false));
    CHECK(rig.predictor.decisionInterval() > 1.3f);

    // And the AI with prediction plays levels at that observation rate.
    for (auto const& level : {sim::levelBasics(), sim::levelSpikes(), sim::levelBlocks()}) {
        LevelMemory mem = freshLevel(level);
        PatternMemory pat;
        Agent agent;
        agent.attach(&mem, &pat);
        agent.setPredictor(&rig.predictor);
        int done = -1;
        uint32_t trusted = 0, untrusted = 0;
        for (int i = 0; i < 60 && done < 0; ++i) {
            if (runAttemptGaps(agent, level, rig, false, 11 + static_cast<uint32_t>(i)).completed) done = i + 1;
            trusted += agent.status().decisionsTrusted;
            untrusted += agent.status().decisionsUntrusted;
        }
        std::printf("    %-7s at 1-3 steps per call: completed after %d attempts (%u decisions with trusted prediction, %u untrusted)\n",
                    level.name.c_str(), done, trusted, untrusted);
        CHECK(done > 0);
        CHECK(trusted > untrusted);
    }
}

// ---------------------------------------------------------------------------
// Live plan, cancellation and timing optimisation (v0.3)

namespace {
// One planner-driven attempt (no learned opinion), recording what the live
// plan did on every step.
struct PlanRun {
    bool completed = false, died = false;
    int presses = 0;
    int cancelTick = -1;                 // first plan invalidation
    std::string cancelReason;
    int cancelledActions = 0;
    bool replannedSameTick = false;      // a full decision at the cancellation tick
    bool newPlanFromActualState = false; // the next plan starts from the real (landed) state
    int stalePlanInputs = 0;             // inputs from a plan made before the cancellation, executed after it
    int contradictions = 0;              // executed input disagreeing with the record / the plan followed
    int followed = 0;
    int maxSims = 0;
};

PlanRun runPlanned(Agent& agent, sim::Level const& level, PredictionRig& rig, int maxTicks = 20000) {
    PlanRun r;
    sim::World w(level);
    rig.held = false;
    agent.beginAttempt(rig.observe(w), false);
    bool prevHold = false;
    for (int t = 0; t < maxTicks; ++t) {
        Observation o = rig.observe(w);
        rig.predictor.observe(o, rig.held);
        uint32_t const before = agent.plans().invalidated;
        bool hold = agent.act(o);
        auto const& rec = agent.status().safety;
        auto const& plans = agent.plans();
        if (plans.invalidated > before && r.cancelTick < 0) {
            r.cancelTick = o.tick;
            r.cancelReason = plans.lastInvalidation.reason;
            r.cancelledActions = plans.lastInvalidation.cancelled;
            r.replannedSameTick = rec.tick == o.tick && rec.proposedSource != DecisionSource::Plan;
            r.newPlanFromActualState = plans.active() && plans.plan().createdTick == o.tick &&
                                       std::fabs(plans.plan().startY - o.y) < 0.01f && plans.plan().startGrounded == o.onGround;
        }
        if (rec.tick == o.tick) {
            if ((rec.executed != 0) != hold) ++r.contradictions;
            if (rec.proposedSource == DecisionSource::Plan) {
                ++r.followed;
                if (plans.holdFor(o.tick) != (hold ? 1 : 0)) ++r.contradictions;
                if (r.cancelTick >= 0 && plans.plan().createdTick < r.cancelTick && hold != prevHold) ++r.stalePlanInputs;
            }
        }
        r.maxSims = std::max(r.maxSims, rig.predictor.lastAdvice().simulations);
        if (hold && !prevHold) ++r.presses;
        prevHold = hold;
        rig.held = hold;
        w.step(hold);
        if (w.dead()) {
            r.died = true;
            agent.onDeath(w.observe(), w.death());
            return r;
        }
        if (w.complete()) {
            r.completed = true;
            agent.onComplete(w.observe());
            return r;
        }
    }
    agent.abortAttempt(nullptr);
    return r;
}
} // namespace

TEST(plan_early_landing_cancels_the_obsolete_plan) {
    // Scenarios 1-3 and 7. The jump over the first spike is planned to land
    // on the floor, with a second jump after landing for the next spike. A
    // block the AI cannot see makes it land earlier. The landing is detected
    // from the real state, the pending second press of the old plan is
    // cancelled (never executed), and the next decision is made from the
    // actual landed state at once. Learning records are kept.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level;
    level.name = "early-landing";
    level.spike(10.f);
    level.hiddenBlock(11.6f, 0.f, 1.4f, 1.f); // the unexpected landing spot
    level.spike(15.f);
    level.finish(26.f);
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runPlanned(agent, level, rig);
    auto const& plans = agent.plans();
    std::printf("    %s; first invalidation at step %d: \"%s\", %d pending action(s) cancelled; replanned in the same step: %s, "
                "new plan from the landed state: %s; old-plan inputs executed afterwards: %d; %d decisions followed a plan\n",
                r.completed ? "completed" : "DIED", r.cancelTick, r.cancelReason.c_str(), r.cancelledActions, r.replannedSameTick ? "yes" : "no",
                r.newPlanFromActualState ? "yes" : "no", r.stalePlanInputs, r.followed);
    std::printf("    panel: %s\n", agent.status().planEvent.c_str());
    CHECK(r.cancelTick >= 0);
    CHECK(r.cancelReason.find("Landing detected earlier than expected") != std::string::npos);
    CHECK(r.cancelledActions >= 1);
    CHECK(r.replannedSameTick);
    CHECK(r.newPlanFromActualState);
    CHECK(r.stalePlanInputs == 0);
    CHECK(r.contradictions == 0);
    CHECK(r.completed);
    // the cancelled plan is in the plan history with its reason ...
    bool archived = false;
    for (auto const& h : plans.history) archived = archived || (h.cancelled && h.reason.find("Landing detected") != std::string::npos);
    CHECK(archived);
    // ... and cancelling a plan forgot nothing that was learned
    CHECK(mem.totalAttempts == 1);
    CHECK(mem.q.size() > 0);
    CHECK(countIf(mem.q, [](QEntry const& e) { return e.hasEvidence(); }) > 0);
}

TEST(plan_valid_plan_is_followed_without_new_searches) {
    // When nothing unexpected happens the plan stays valid and is followed:
    // fewer searches, no cancellations, same result.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = sim::levelSpikes();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    uint64_t sims0 = rig.predictor.totalSimulations();
    auto r = runPlanned(agent, level, rig);
    std::printf("    spikes: %s, %d decisions followed a valid plan, %u plans made, %u invalidated, %.1f simulations per step\n",
                r.completed ? "completed" : "DIED", r.followed, agent.plans().adopted, agent.plans().invalidated,
                double(rig.predictor.totalSimulations() - sims0) / 2100.0);
    CHECK(r.completed);
    CHECK(r.followed > 0);
    CHECK(agent.plans().invalidated == 0);
    CHECK(r.contradictions == 0);
    CHECK(r.presses == 9);
}

TEST(plan_gravity_change_invalidates_pending_actions) {
    // Scenario 8: a gravity portal changes the physics every plan made before
    // it assumed: the plan is invalidated at once.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level;
    level.name = "gravity";
    level.block(3.f, 5.f, 40.f, 1.f); // ceiling
    level.gravityPortal(12.f);
    level.finish(30.f);
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runPlanned(agent, level, rig);
    bool gravity = r.cancelReason.find("gravity change") != std::string::npos;
    if (std::getenv("AIGD_PLAN_DEBUG")) std::printf("      first invalidation at %d: %s\n", r.cancelTick, r.cancelReason.c_str());
    for (auto const& h : agent.plans().history) {
        gravity = gravity || h.reason.find("gravity change") != std::string::npos;
        if (std::getenv("AIGD_PLAN_DEBUG"))
            std::printf("      plan %d..%d %s: %s\n", h.createdTick, h.endedTick, h.cancelled ? "cancelled" : "ended", h.reason.c_str());
    }
    std::printf("    gravity portal: %s; plan invalidated by the gravity change: %s\n", r.completed ? "completed" : "DIED", gravity ? "yes" : "no");
    CHECK(gravity);
    CHECK(r.contradictions == 0);
}

TEST(optimiser_prefers_the_jump_that_sets_up_the_next_one) {
    // Scenario 4: two spikes in a row. Among the first-jump timings that
    // clear the first spike, the earliest is not the best: the search scores
    // each first jump together with the second jump it allows, and picks a
    // (near-)optimal pair - not the first timing that works.
    PredictionRig rig;
    warmUp(rig);
    for (float second : {13.f, 13.5f, 14.f, 14.5f}) {
        sim::Level level;
        level.name = "pair";
        level.spike(10.f);
        level.spike(second);
        level.finish(30.f);
        sim::World w(level);
        for (int i = 0; i < 150; ++i) w.step(false); // the first spike ~3 blocks ahead
        Observation o = rig.observe(w);
        rig.predictor.observe(o, false);
        SimStart s = rig.predictor.startFrom(o, false);
        // brute force over first-press delay and second-press timing
        int earliest = -1, bestK = -1;
        float best = -1e9f;
        PredictedPath p;
        for (int k = 1; k < 70; ++k) {
            float bestHere = -1e9f;
            for (int k2 = -1; k2 < 70; ++k2) {
                Plan plan{k, 0, 0, false, k2};
                rig.predictor.simulate(s, o.obstacles, plan, p);
                bestHere = std::max(bestHere, rig.predictor.scoreOf(p));
            }
            if (bestHere > 900.f && earliest < 0) earliest = k;
            if (bestHere > best) {
                best = bestHere;
                bestK = k;
            }
        }
        auto const& adv = rig.predictor.advise(o, false, false);
        std::printf("    spikes at 10 and %.1f: earliest safe first jump in %d steps, best in %d (score %.1f); search chose %s in %d, "
                    "second press %d after landing, score %.1f, stage %s, %d simulations\n",
                    second, earliest, bestK, best, adv.bestPresses ? "press" : "wait", adv.bestPlan.pressAt, adv.bestPlan.secondAt,
                    adv.bestScore, searchStageName(adv.stage), adv.simulations);
        CHECK(adv.bestScore > 900.f);            // a safe pair was found
        CHECK(adv.bestScore >= best - 4.f);      // and it is (near) the best possible
        if (bestK > earliest + 2) CHECK(adv.bestPlan.pressAt > earliest); // not just the earliest that works
    }
}

TEST(optimiser_uses_learned_timing_without_forcing_it) {
    // Scenarios 6 and 11: a learned press timing is tried first. Where it fits
    // the current state it is chosen and a safe plan is found with fewer
    // simulations; where the state differs it is simulated, found worse, and
    // not used.
    PredictionRig rig;
    warmUp(rig);
    sim::Level level = spikeLevel("hint", {10.f}, 20.f);
    sim::World w(level);
    for (int i = 0; i < 150; ++i) w.step(false); // the spike ~3 blocks ahead
    Observation o = rig.observe(w);
    rig.predictor.observe(o, false);
    auto const plain = rig.predictor.advise(o, false, false);
    rig.predictor.setHint(plain.bestPlan.pressAt);
    auto const good = rig.predictor.advise(o, false, false);
    std::printf("    no hint: first safe candidate after %d simulations; good hint: after %d, chosen from the hint: %s\n", plain.firstSafeSim,
                good.firstSafeSim, good.fromSeed ? "yes" : "no");
    CHECK(good.seeded && good.fromSeed);
    CHECK(good.firstSafeSim < plain.firstSafeSim);
    CHECK(good.bestPlan.pressAt == plain.bestPlan.pressAt);

    // A timing from a different situation (pressing far too early) is not reused.
    rig.predictor.setHint(3);
    auto const bad = rig.predictor.advise(o, false, false);
    std::printf("    wrong hint (press in 3): seeded %s, chosen from the hint %s, chosen press in %d (%s)\n", bad.seeded ? "yes" : "no",
                bad.fromSeed ? "yes" : "no", bad.bestPlan.pressAt, certaintyName(bad.release));
    CHECK(bad.seeded && !bad.fromSeed);
    CHECK(bad.bestPlan.pressAt == plain.bestPlan.pressAt);
    CHECK(bad.release == Certainty::Safe);
}

TEST(optimiser_stays_within_its_budget) {
    // Scenario 9: with a small budget the search still finds viable actions
    // (the course is completed) and never exceeds the budget (+2 fixed
    // reference simulations: keep the input / never press).
    PredictionRig rig;
    warmUp(rig);
    rig.predictor.setBudget(40);
    sim::Level level = sim::levelBlocks();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    Agent agent(isolatedConfig());
    agent.attach(&mem, &pat);
    agent.setPredictor(&rig.predictor);
    auto r = runPlanned(agent, level, rig);
    std::printf("    budget 40: blocks %s, at most %d simulations in one decision\n", r.completed ? "completed" : "DIED", r.maxSims);
    CHECK(r.completed);
    CHECK(r.maxSims <= 42);
}

TEST(optimiser_resolution_follows_model_and_decision_rate) {
    // With observations every ~2 steps (as in game) refining to single steps
    // is pointless: the search refines at the rate the AI can act.
    PredictionRig rig;
    warmUpGaps(rig, false);
    sim::Level level = spikeLevel("res", {10.f}, 20.f);
    sim::World w(level);
    for (int i = 0; i < 40; ++i) w.step(false);
    Observation o = rig.observe(w);
    o.tick = 100000; // a new history
    rig.predictor.observe(o, false);
    auto const& adv = rig.predictor.advise(o, false, false);
    std::printf("    %.2f steps per decision -> timing resolution %d step(s)\n", rig.predictor.decisionInterval(), adv.resolution);
    CHECK(adv.resolution >= 2);
    PredictionRig every;
    warmUp(every);
    every.predictor.observe(o, false);
    CHECK(every.predictor.advise(o, false, false).resolution == 1);
}

TEST(learned_sequence_is_not_replayed_in_a_different_state) {
    // Scenario 6 for learned test sequences: an experiment planned for a
    // grounded approach is cancelled - not played - when the player arrives
    // at its first changed cell in the air (an earlier decision changed).
    sim::Level level = sim::levelBasics();
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg;
    Agent agent(cfg);
    agent.attach(&mem, &pat);
    runAttempt(agent, level); // dies without jumping; an experiment is planned
    Experiment ex = agent.pendingExperiment();
    CHECK(ex.active);
    // Second attempt: the same approach (the real walk), except that the
    // player arrives at the experiment's first changed cell in the air.
    sim::World w(level);
    agent.beginAttempt(w.observe(), false);
    float const firstX = static_cast<float>(ex.testFrom) * cfg.cellSize;
    bool pressedAtTest = false;
    while (!w.dead()) {
        Observation o = w.observe();
        if (o.x >= firstX) {
            o.y += 40.f;
            o.vy = 6.f;
            o.onGround = false;
            pressedAtTest = agent.act(o);
            break;
        }
        agent.act(o);
        w.step(false);
    }
    int forcedSteps = 0;
    for (auto const& s : agent.steps()) forcedSteps += s.source == DecisionSource::Experiment ? 1 : 0;
    agent.abortAttempt(nullptr);
    std::printf("    experiment \"%s\": %u cancelled (%s), %d forced steps played\n", ex.description.c_str(),
                agent.status().experimentsCancelled, agent.status().planEvent.c_str(), forcedSteps);
    CHECK(agent.status().experimentsCancelled == 1);
    CHECK(forcedSteps == 0);
    CHECK(!pressedAtTest);
    CHECK(agent.status().planEvent.find("Experiment cancelled") != std::string::npos);
}

TEST(wrong_saved_x_speed_is_corrected) {
    // The calibration saved by earlier versions in game: x speed 1.5579 at
    // 1x from 10 samples (the real speed is 1.2983). Loading it must not keep
    // the wrong value, and a wrong estimate built up during play must be
    // replaced once correct samples keep arriving.
    float const nominal = unitsPerSecond(1) / 240.f;
    PhysicsCalibration saved;
    saved.vxSum[1] = 15.5790043;
    saved.vxN[1] = 10;
    PhysicsModel model;
    model.loadCalibration(saved);
    std::printf("    saved 1.5579/step -> after loading: %.4f (true %.4f)\n", model.vxPerTick(1), nominal);
    CHECK(std::fabs(model.vxPerTick(1) - nominal) < 0.01f);

    // Wrong during play (old tick counting), then real steps: corrected.
    PredictionRig rig;
    warmUpGaps(rig, true);
    float const wrong = rig.model.vxPerTick(1);
    warmUpGaps(rig, false, 9);
    std::printf("    learned with one tick per call: %.4f; after real steps: %.4f (true %.4f)\n", wrong, rig.model.vxPerTick(1), nominal);
    CHECK(wrong > nominal * 1.1f);
    CHECK(std::fabs(rig.model.vxPerTick(1) - nominal) < nominal * 0.01f);
}

// ---------------------------------------------------------------------------
// Benchmark (not run by default)

static void runBench() {
    std::printf("\n=== Learning benchmark: attempts to first completion (5 seeds) ===\n");
    struct Variant {
        char const* name;
        LearnerConfig cfg;
        bool prediction = false;
    };
    std::vector<Variant> variants;
    {
        LearnerConfig c;
        c.useExperiments = false;
        variants.push_back({"Q-learning only", c});
    }
    for (int k : {1, 3, 6}) {
        LearnerConfig c;
        c.experimentAfterDeaths = k;
        static std::string names[8];
        names[k] = "Q + analysis after " + std::to_string(k) + " deaths";
        variants.push_back({names[k].c_str(), c});
    }
    for (float sp : {0.1f, 1.0f}) {
        LearnerConfig c;
        c.learningSpeed = sp;
        variants.push_back({sp < 0.5f ? "default, speed 0.1" : "default, speed 1.0", c});
    }
    // Same default learner plus trajectory prediction (fresh, uncalibrated
    // physics model per run: calibration happens during the first attempts).
    variants.push_back({"default + trajectory prediction", LearnerConfig{}, true});

    for (auto const& level : sim::allLevels()) {
        std::printf("%s\n", level.name.c_str());
        for (auto const& v : variants) {
            std::vector<int> results;
            for (uint32_t seed = 1; seed <= 5; ++seed) {
                LearnerConfig cfg = v.cfg;
                cfg.seed = seed * 7919u;
                LevelMemory mem = freshLevel(level);
                PatternMemory pat;
                Agent agent(cfg);
                agent.attach(&mem, &pat);
                PredictionRig rig;
                if (v.prediction) agent.setPredictor(&rig.predictor);
                auto r = train(agent, level, 3000, v.prediction ? &rig : nullptr);
                results.push_back(r.attemptsToComplete < 0 ? 99999 : r.attemptsToComplete);
            }
            std::sort(results.begin(), results.end());
            int fails = static_cast<int>(std::count(results.begin(), results.end(), 99999));
            double mean = 0;
            int n = 0;
            for (int x : results) {
                if (x != 99999) {
                    mean += x;
                    ++n;
                }
            }
            if (n) mean /= n;
            std::printf("    %-34s median %5d  mean %5.0f%s\n", v.name, results[2] == 99999 ? -1 : results[2], mean,
                        fails ? (" (" + std::to_string(fails) + "/5 failed)").c_str() : "");
        }
    }
}

// ---------------------------------------------------------------------------
// Cross-level transfer (v0.4.0)
//
// Training and evaluation levels are generated from disjoint seeds: an
// evaluation level is never seen during training, so anything it gains can
// only come from knowledge that transfers, not from memorised positions.

namespace {
constexpr uint32_t kTrainSeeds[] = {1, 2, 3, 4, 5, 6};
constexpr uint32_t kEvalSeeds[] = {101, 102, 103, 104, 105, 106};

sim::Level solvableGenerated(uint32_t seed, uint32_t allowed = ~0u) {
    for (uint32_t k = 0; k < 50; ++k) {
        sim::Level L = sim::generatedLevel(seed + k * 7919u, 9, allowed);
        if (cubeLevelSolvable(L)) {
            L.name = "gen-" + std::to_string(seed);
            return L;
        }
    }
    return sim::levelBasics();
}

struct Knowledge {
    PatternMemory patterns;
    StrategyLibrary strategies;
};

// Plays a level until it is completed (or maxAttempts), sharing `k`.
TrainResult playLevel(Knowledge& k, sim::Level const& level, int maxAttempts, bool prediction, bool useGeneral, bool useStrategies,
                      uint32_t seed, TransferMetrics* totalOut = nullptr, LevelMemory* memOut = nullptr,
                      std::vector<std::string>* messages = nullptr) {
    LevelMemory mem = freshLevel(level);
    LearnerConfig cfg;
    cfg.seed = seed;
    cfg.usePrediction = prediction;
    Agent agent(cfg);
    PatternMemory none;
    agent.attach(&mem, useGeneral ? &k.patterns : &none, useStrategies ? &k.strategies : nullptr);
    PredictionRig rig;
    if (useGeneral) rig.model.loadCalibration(k.patterns.physics);
    TrainResult r;
    if (prediction) {
        agent.setPredictor(&rig.predictor);
        r = train(agent, level, maxAttempts, &rig);
        if (useGeneral) k.patterns.physics = rig.model.calibration();
    } else {
        // no predictor, but the same observations as in game (the obstacles around the player)
        r = train(agent, level, maxAttempts, &rig);
    }
    if (totalOut) *totalOut = agent.status().transferTotal;
    if (messages) *messages = agent.status().transferLog;
    if (memOut) *memOut = mem;
    return r;
}

Knowledge trainedKnowledge(bool prediction, uint32_t seed, int maxAttempts = 400) {
    Knowledge k;
    for (uint32_t s : kTrainSeeds) playLevel(k, solvableGenerated(s), maxAttempts, prediction, true, true, seed);
    return k;
}

struct EvalStats {
    double firstProgress = 0;
    int firstCompleted = 0;
    double attempts = 0;
    int failed = 0;
    int runs = 0;
    TransferMetrics transfer;
};

void report(char const* label, EvalStats const& s) {
    std::printf("    %-44s first attempt: %5.1f%% mean progress, %d/%d completed | attempts to complete: mean %6.1f%s\n", label,
                100.0 * s.firstProgress / std::max(1, s.runs), s.firstCompleted, s.runs, s.attempts / std::max(1, s.runs),
                s.failed ? (" (" + std::to_string(s.failed) + " not completed)").c_str() : "");
}
} // namespace

TEST(situation_descriptor_is_relative_and_transferable) {
    // The same obstacles anywhere in a level (or in another level) give the
    // same description; different landing space or height gives another.
    auto at = [](sim::Level const& L, float playerBlock) {
        sim::World w(L);
        Observation o = w.observe();
        PredictionRig rig;
        while (o.x < playerBlock * 30.f && !w.dead() && !w.complete()) {
            w.step(false);
            o = rig.observe(w);
        }
        return describeSituation(o);
    };
    sim::Level a, b, c, d;
    a.spike(20); a.spike(21); a.finish(60);
    b.spike(40); b.spike(41); b.finish(80);           // same pair, elsewhere
    c.spike(20); c.spike(21); c.spike(23); c.finish(60); // a third spike right after: other layout
    d.block(20, 0, 1, 1); d.finish(60);               // a block instead
    Situation sa = at(a, 16), sb = at(b, 36), sc = at(c, 16), sd = at(d, 16);
    CHECK(sa.valid && sb.valid && sc.valid && sd.valid);
    CHECK(sa.fine == sb.fine);
    CHECK(sa.coarse == sb.coarse);
    CHECK(sa.fine != sc.fine);
    CHECK(sa.fine != sd.fine);
    CHECK(sa.coarse != sd.coarse);
    CHECK(sa.count == 2 && sa.el[0].kind == SituationElement::Hazard);
    CHECK(std::fabs(sa.contactSteps - sb.contactSteps) < 2.f);
    std::printf("    %s | %s | %s\n", sa.describe().c_str(), sc.describe().c_str(), sd.describe().c_str());
}

TEST(strategy_library_tracks_provenance_and_negative_transfer) {
    StrategyLibrary lib;
    Situation s;
    s.valid = true;
    s.fine = 1234;
    s.coarse = (uint64_t(1) << 52) | 99;
    s.speedClass = 1;
    for (int i = 0; i < 5; ++i) lib.record(s, Primitive::Jump, 20.f, -1.f, true, "level-a");
    for (int i = 0; i < 3; ++i) lib.record(s, Primitive::Jump, 21.f, -1.f, true, "level-b");
    auto c = lib.choose(s, "level-new");
    CHECK(c.usable && c.match == StrategyChoice::Match::Exact);
    CHECK(c.primitive == Primitive::Jump && std::fabs(c.leadSteps - 20.4f) < 0.2f);
    CHECK(c.otherLevels == 2 && !c.learnedHere);
    // at another speed: the same steps, flagged as adapted
    s.speedClass = 3;
    CHECK(lib.choose(s, "level-new").speedAdapted);
    s.speedClass = 1;
    // It keeps failing on a new level: rejected there, still used elsewhere.
    lib.record(s, Primitive::Jump, 20.f, -1.f, false, "level-new");
    CHECK(lib.choose(s, "level-new").usable);  // one failure may have had another cause
    lib.record(s, Primitive::Jump, 20.f, -1.f, false, "level-new");
    auto here = lib.choose(s, "level-new");
    CHECK(!here.usable && here.rejectedHere);
    CHECK(lib.choose(s, "level-c").usable);
    auto const& rec = lib.situations.at(s.fine)[0];
    CHECK(rec.successes == 8 && rec.failures == 2);
    CHECK(rec.on("level-a")->successes == 5 && rec.on("level-new")->failures == 2);
    // A different timing that works on the new level is preferred there.
    for (int i = 0; i < 3; ++i) lib.record(s, Primitive::Jump, 12.f, -1.f, true, "level-new");
    auto alt = lib.choose(s, "level-new");
    CHECK(alt.usable && std::fabs(alt.leadSteps - 12.f) < 0.5f);
}

TEST(strategy_memory_persists_and_resets_separately) {
    Knowledge k = trainedKnowledge(true, 1, 60);
    CHECK(k.strategies.size() > 0);
    // round trip, including provenance
    std::string text = storage::serializeStrategies(k.strategies);
    StrategyLibrary back;
    std::string err;
    CHECK(storage::deserializeStrategies(text, back, &err));
    CHECK(back.size() == k.strategies.size() && back.records() == k.strategies.records());
    CHECK(back.levels == k.strategies.levels);
    CHECK(storage::serializeStrategies(back) == text);
    // the whole cross-level knowledge as one export document
    PatternMemory p2;
    StrategyLibrary s2;
    std::string bundle = storage::serializeKnowledge(k.patterns, k.strategies);
    CHECK(storage::deserializeKnowledge(bundle, p2, s2, &err));
    CHECK(s2.records() == k.strategies.records() && p2.q.size() == k.patterns.q.size());
    // a pattern file written before v0.4.0 (no strategies) still loads
    std::string v2 = storage::serializePatterns(k.patterns);
    PatternMemory p3;
    CHECK(storage::deserializePatterns(v2, p3, &err));
    // a newer strategy format is refused, not misread
    std::string newer = text;
    newer.replace(newer.find("\"version\":1"), 11, "\"version\":9");
    StrategyLibrary s3;
    CHECK(!storage::deserializeStrategies(newer, s3, &err));
    // corrupt main file: the backup copy is used
    auto dir = tempDir();
    auto path = dir / "strategies.json";
    CHECK(storage::writeFileAtomic(path, text));
    CHECK(storage::writeFileAtomic(path, text)); // second write keeps the first as .bak
    { std::ofstream(path, std::ios::trunc) << "{garbage"; }
    StrategyLibrary s4;
    CHECK(storage::loadStrategyFile(path, s4, &err));
    CHECK(s4.records() == k.strategies.records());

    // Level reset (layer C) keeps the library; a general-knowledge reset
    // keeps the level memory.
    sim::Level L = solvableGenerated(101);
    LevelMemory mem;
    playLevel(k, L, 30, true, true, true, 1, nullptr, &mem);
    size_t const libBefore = k.strategies.records();
    CHECK(mem.q.size() > 0);
    LevelMemory fresh = freshLevel(L); // what "reset this level" does
    CHECK(fresh.q.size() == 0 && k.strategies.records() == libBefore);
    k.strategies.clear();             // what "reset cross-level knowledge" does
    k.patterns = PatternMemory{};
    CHECK(mem.q.size() > 0 && mem.totalAttempts > 0);
}

TEST(transfer_to_unseen_levels) {
    // Separate training and evaluation levels, several agent seeds. Reported:
    // first-attempt progress / completions and attempts to complete, for
    //   none      - no cross-level knowledge at all,
    //   general   - physics calibration, timing statistics, pattern prior (layer A),
    //   general + strategies - plus the strategy library (layer B).
    bool const full = std::getenv("AIGD_TRANSFER_FULL") != nullptr;
    int const seeds = full ? 3 : 2;
    int const evalLevels = full ? 6 : 4;
    int const cap = full ? 300 : 150;
    for (bool prediction : {false, true}) {
        std::printf("    -- %s --\n", prediction ? "with trajectory prediction" : "without trajectory prediction (learning only)");
        EvalStats none, general, both, onlyLib;
        for (int sd = 1; sd <= seeds; ++sd) {
            Knowledge trained = trainedKnowledge(prediction, static_cast<uint32_t>(sd));
            if (sd == 1)
                std::printf("    trained on %zu levels: %zu situations, %zu strategy records\n", std::size(kTrainSeeds),
                            trained.strategies.size(), trained.strategies.records());
            for (int i = 0; i < evalLevels; ++i) {
                sim::Level L = solvableGenerated(kEvalSeeds[i]);
                for (int cond = 0; cond < 4; ++cond) {
                    Knowledge k = trained; // every evaluation starts from the same trained knowledge
                    LevelMemory mem;
                    bool const useGeneral = cond == 1 || cond == 2;
                    bool const useLib = cond >= 2;
                    auto r = playLevel(k, L, cap, prediction, useGeneral, useLib, static_cast<uint32_t>(100 + sd), nullptr, &mem);
                    EvalStats& s = cond == 0 ? none : cond == 1 ? general : cond == 2 ? both : onlyLib;
                    ++s.runs;
                    s.firstProgress += r.attempts.empty() ? 0.0 : r.attempts[0].progress;
                    s.firstCompleted += (!r.attempts.empty() && r.attempts[0].completed) ? 1 : 0;
                    s.attempts += r.attemptsToComplete > 0 ? r.attemptsToComplete : cap + 1;
                    s.failed += r.attemptsToComplete > 0 ? 0 : 1;
                }
            }
        }
        report("none", none);
        report("general knowledge", general);
        report("general knowledge + strategy library", both);
        report("strategy library only", onlyLib);
        if (!prediction) {
            // Without prediction, the library is what can act on a new level:
            // it must help the first attempt and not slow learning down.
            CHECK(both.firstProgress >= general.firstProgress);
            CHECK(both.attempts <= general.attempts * 1.15 + 2);
        } else {
            CHECK(both.attempts <= general.attempts * 1.15 + 2);
            CHECK(both.firstCompleted >= general.firstCompleted - 1);
        }
    }
}

TEST(negative_transfer_is_detected_and_contained) {
    // Strategies learned on normal levels, then a level where the input takes
    // effect 8 steps late (a context the situation description cannot see):
    // the transferred timings fail there. They must stop being used on that
    // level, the level must still be learned, and what was learned on the
    // other levels must stay intact.
    Knowledge k = trainedKnowledge(false, 1);
    size_t const before = k.strategies.records();
    uint32_t successesBefore = 0;
    for (auto const& [key, recs] : k.strategies.situations)
        for (auto const& r : recs) successesBefore += r.successes;
    sim::Level L = solvableGenerated(kEvalSeeds[0]);
    L.name = "delayed";
    TrainResult withLib, without;
    TransferMetrics total;
    {
        SimHooks late(14, 0.f, 0.f);
        Knowledge k2 = k;
        without = playLevel(k2, L, 300, false, true, false, 7);
        withLib = playLevel(k, L, 300, false, true, true, 7, &total);
    }
    uint32_t localFailures = 0, otherSuccesses = 0;
    int rejected = 0, proposedHere = 0, proposedElsewhere = 0;
    for (auto const& [key, recs] : k.strategies.situations) {
        for (auto const& r : recs) {
            for (auto const& p : r.levels) {
                if (p.level == "sim-delayed") localFailures += p.failures;
                else otherSuccesses += p.successes;
            }
            bool rej = false;
            strategyRelevance(r, "sim-delayed", &rej);
            if (!rej || r.successes < 2) continue;
            ++rejected;
            // Is that timing still what the library proposes, here / on another level?
            bool const isCoarse = (key >> 52) != 0;
            Situation s;
            s.valid = true;
            s.fine = isCoarse ? 1 : key;
            s.coarse = isCoarse ? key : ((uint64_t(1) << 52) | 1);
            s.speedClass = 1;
            auto same = [&](StrategyChoice const& c) {
                return c.usable && c.primitive == r.primitive && r.leadBucket >= 0 &&
                       std::lround(c.leadSteps / StrategyRecord::kLeadBucket) == r.leadBucket;
            };
            proposedHere += same(k.strategies.choose(s, "sim-delayed")) ? 1 : 0;
            proposedElsewhere += same(k.strategies.choose(s, "sim-other")) ? 1 : 0;
        }
    }
    std::printf("    delayed input: completed after %d attempts with the library, %d without | %u situations met, %u failures recorded "
                "on this level, %d strategies rejected for it (%d still proposed here, %d proposed on other levels)\n",
                withLib.attemptsToComplete, without.attemptsToComplete, total.situations, localFailures, rejected, proposedHere,
                proposedElsewhere);
    CHECK(withLib.attemptsToComplete > 0);
    CHECK(localFailures > 0);
    CHECK(rejected > 0);
    CHECK(proposedHere == 0);
    // contained: not much slower than without transferred strategies
    CHECK(withLib.attemptsToComplete <= without.attemptsToComplete * 2 + 10);
    // the evidence from the other levels is untouched (only added to)
    CHECK(otherSuccesses >= successesBefore - 0);
    CHECK(k.strategies.records() >= before);
    // and still used on a level without the delay
    sim::Level M = solvableGenerated(kEvalSeeds[1]);
    TransferMetrics t2;
    playLevel(k, M, 1, false, true, true, 7, &t2);
    CHECK(t2.known + t2.similar > 0);
}

TEST(transfer_uses_relative_geometry_not_positions) {
    // No copying: a level's strategies are keyed by what is in front of the
    // player, never by where. The same level moved by 7 blocks adds no new
    // situation, and is played from the library as well as the original.
    Knowledge k;
    sim::Level L = solvableGenerated(kTrainSeeds[0]);
    playLevel(k, L, 300, false, true, true, 3);
    size_t const situations = k.strategies.size();
    Knowledge k2 = k;
    sim::Level S = sim::shiftedLevel(L, 7.f);
    TransferMetrics t;
    auto r = playLevel(k2, S, 300, false, true, true, 3, &t);
    std::printf("    shifted copy: %zu situations before, %zu after; first attempt %.0f%% (%u groups met, %u known)\n", situations,
                k2.strategies.size(), r.attempts.empty() ? 0.f : r.attempts[0].progress * 100.f, t.situations, t.known);
    CHECK(situations > 0);
    CHECK(k2.strategies.size() == situations);
    CHECK(t.known > 0);
    // nothing in the library refers to a level position
    std::string text = storage::serializeStrategies(k.strategies);
    CHECK(text.find("\"x\"") == std::string::npos);
}

TEST(strategies_guide_the_search_but_never_dictate) {
    // With a trusted model the library only seeds the search. A correct
    // strategy is confirmed by simulation; a wrong one (pressing far too
    // early) is simulated, rejected, and the level is still completed on the
    // first attempt. The physics model is not touched by the library.
    Knowledge good = trainedKnowledge(true, 1, 60);
    sim::Level L = solvableGenerated(kEvalSeeds[2]);
    TransferMetrics tg;
    std::vector<std::string> msgs;
    auto rg = playLevel(good, L, 5, true, true, true, 11, &tg, nullptr, &msgs);
    Knowledge bad = good;
    for (auto& [key, recs] : bad.strategies.situations)
        for (auto& r : recs) {
            r.lead = TimingStat{};
            r.lead.add(95.0); // press 95 steps before contact: lands in front of the obstacle
            r.primitive = Primitive::Jump;
        }
    PhysicsCalibration const calBefore = bad.patterns.physics;
    TransferMetrics tb;
    auto rb = playLevel(bad, L, 5, true, true, true, 11, &tb);
    std::printf("    correct strategies: %u seeded, %u confirmed, %u rejected | wrong strategies: %u seeded, %u confirmed, %u rejected; "
                "first attempt %s / %s\n",
                tg.seeded, tg.confirmed, tg.rejectedBySim, tb.seeded, tb.confirmed, tb.rejectedBySim,
                rg.attemptsToComplete == 1 ? "completed" : "died", rb.attemptsToComplete == 1 ? "completed" : "died");
    for (auto const& m : msgs) std::printf("      %s\n", m.c_str());
    CHECK(rg.attemptsToComplete == 1);
    CHECK(rb.attemptsToComplete == 1);
    CHECK(tg.confirmed > 0);
    CHECK(tb.rejectedBySim > 0);
    CHECK(tb.rejectedBySim > tb.confirmed);
    // the library never writes physics; calibration changes only from observed play
    for (size_t i = 0; i < PhysicsCalibration::kSlots; ++i)
        CHECK(bad.patterns.physics.modes[i].impulse.n >= calBefore.modes[i].impulse.n);
}

TEST(transfer_cost_per_step_is_small) {
    Knowledge k = trainedKnowledge(true, 1, 60);
    sim::Level L = solvableGenerated(kEvalSeeds[3]);
    double us[2] = {0, 0};
    for (int withLib = 0; withLib < 2; ++withLib) {
        Knowledge kk = k;
        LevelMemory mem = freshLevel(L);
        Agent agent;
        agent.attach(&mem, &kk.patterns, withLib ? &kk.strategies : nullptr);
        PredictionRig rig;
        rig.model.loadCalibration(kk.patterns.physics);
        agent.setPredictor(&rig.predictor);
        auto t0 = std::chrono::steady_clock::now();
        size_t steps = 0;
        for (int i = 0; i < 3; ++i) {
            runAttempt(agent, L, &rig);
            steps += agent.steps().size();
        }
        us[withLib] = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / std::max<size_t>(1, steps);
    }
    std::printf("    %.1f us per decision without the library, %.1f with it\n", us[0], us[1]);
    CHECK(us[1] < us[0] * 1.5 + 5.0);
}

TEST(upside_down_cube_contact_is_not_a_ceiling) {
    // In game (Dry Out, 48.8%) an upside-down cube rested on something the
    // observer did not report. v0.3.0 learned that contact as a level-wide
    // ceiling and then predicted a surface under hanging spikes (one-step
    // error 18.5, repeated deaths). Cube and robot contacts must not create a
    // ceiling; ship contacts still do.
    PhysicsModel model;
    Predictor p(model);
    std::vector<Obstacle> obs{{900.f, 90.f, 930.f, 120.f, Obstacle::Kind::Solid}};
    Observation o;
    o.mode = GameMode::Cube;
    o.upsideDown = true;
    o.onGround = true;
    o.playerSpeed = 0.9f;
    o.obstacles = obs;
    for (int t = 0; t < 10; ++t) {
        o.tick = t;
        o.x = 600.f + 1.3f * static_cast<float>(t);
        o.y = 305.7f;
        p.observe(o, false);
    }
    CHECK_MSG(!p.bounds().hasCeil || p.bounds().ceilEstimated, "cube learned a ceiling at %.1f", p.bounds().ceil);

    Predictor ship(model);
    o.mode = GameMode::Ship;
    for (int t = 0; t < 10; ++t) {
        o.tick = t;
        o.x = 600.f + 1.3f * static_cast<float>(t);
        ship.observe(o, false);
    }
    CHECK(ship.bounds().hasCeil && !ship.bounds().ceilEstimated);
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    bool bench = false;
    char const* filter = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--bench") == 0) bench = true;
        else filter = argv[i];
    }
    int ran = 0;
    for (auto const& t : registry()) {
        if (filter && !std::strstr(t.name, filter)) continue;
        int before = g_failures;
        auto t0 = std::chrono::steady_clock::now();
        std::printf("[ RUN  ] %s\n", t.name);
        t.fn();
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("[ %s ] %s (%.0f ms)\n", g_failures == before ? " OK " : "FAIL", t.name, ms);
        ++ran;
    }
    std::printf("\n%d tests, %d checks, %d failures\n", ran, g_checks, g_failures);
    if (bench) runBench();
    return g_failures == 0 ? 0 : 1;
}

// Diagnostic: print the attempt history for one level (run with "diag_<level>").
static void diag(sim::Level const& level, int attempts) {
    LevelMemory mem = freshLevel(level);
    PatternMemory pat;
    LearnerConfig cfg;
    if (char const* seed = std::getenv("AIGD_SEED")) cfg.seed = static_cast<uint32_t>(std::atoi(seed)) * 7919u;
    Agent agent(cfg);
    agent.attach(&mem, &pat);
    for (int i = 0; i < attempts; ++i) {
        auto r = runAttempt(agent, level);
        auto const& h = mem.history.back();
        if (i < 40 || i % 25 == 0 || r.completed)
            std::printf("    #%d end %.1f (blk %.2f) %s | %s | next: %s\n", i + 1, r.endX, r.endX / 30.f,
                        failureKindName(h.kind), h.change.c_str(), agent.pendingExperiment().active ? agent.pendingExperiment().description.c_str() : "-");
        if (r.completed) break;
        if (std::getenv("AIGD_PRESSES")) {
            std::printf("      presses:");
            auto const& st = agent.steps();
            for (size_t k = 1; k < st.size(); ++k)
                if (st[k].action != st[k - 1].action && st[k].x > r.endX - 200.f)
                    std::printf(" %s@%.0f%s%s", st[k].action ? "P" : "R", st[k].x, st[k].onGround ? "g" : "a",
                                st[k].source == DecisionSource::Policy ? "" : (std::string("[") + decisionSourceName(st[k].source) + "]").c_str());
            auto const& d = agent.lastReport();
            std::printf(" | kind=%s\n      credit: %s | reason: %s\n", failureKindName(d.kind), agent.status().lastCredit.c_str(),
                        agent.status().reason.c_str());
        }
    }
}
// Diagnostic with trajectory prediction (run with AIGD_DIAG_PRED=<level>):
// the safety decisions before every death.
TEST(diag_level_prediction) {
    char const* which = std::getenv("AIGD_DIAG_PRED");
    if (!which) return;
    for (auto const& level : sim::allLevels()) {
        if (level.name != which) continue;
        PredictionRig rig;
        warmUp(rig);
        LevelMemory mem = freshLevel(level);
        PatternMemory pat;
        Agent agent;
        agent.attach(&mem, &pat);
        agent.setPredictor(&rig.predictor);
        for (int i = 0; i < 50; ++i) {
            auto r = runAttempt(agent, level, &rig);
            std::printf("    #%d %s at x=%.1f | %s\n", i + 1, r.completed ? "COMPLETED" : "died", r.endX, agent.status().lastCredit.c_str());
            if (r.completed) break;
            auto const& log = agent.safetyLog();
            for (size_t k = log.size() > 160 ? log.size() - 160 : 0; k < log.size(); ++k) {
                auto const& s = log[k];
                std::printf("      t=%d x=%.1f y=%.1f vy=%.2f %s held=%d R:%s/%d/k%d P:%s/%d/k%d prop=%d(%s) exe=%d %s | %s\n", s.tick, s.x, s.y,
                            s.vy, gameModeName(s.mode), s.holdingBefore, certaintyName(s.outcome[0]), s.ttc[0], s.keepFor[0],
                            certaintyName(s.outcome[1]), s.ttc[1], s.keepFor[1], s.proposed, decisionSourceName(s.proposedSource), s.executed,
                            safetyVerdictName(s.verdict), s.reason.c_str());
            }
            std::printf("      prediction: %s\n", agent.status().predictionNote.c_str());
        }
    }
}

TEST(diag_level) {
    char const* which = std::getenv("AIGD_DIAG");
    if (!which) return;
    for (auto const& l : sim::allLevels()) {
        if (l.name == which) diag(l, 400);
    }
}

