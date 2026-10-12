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
#include <aigd/Storage.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
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
    rig.predictor.simulate(Predictor::startFrom(o, false), o.obstacles, plan, p);
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
    CHECK(std::fabs(rig.model.cube(false).gravity - sim::World::kGravity) > 0.004f);
    CHECK(!rig.model.calibrated(false));
    warmUp(rig);
    auto const& p = rig.model.cube(false);
    std::printf("    learned: %s, vx=%.4f, drift %.4f/step\n", rig.model.describe().c_str(), rig.model.vxPerTick(1), rig.model.driftPerTick());
    CHECK(rig.model.calibrated(false));
    CHECK(std::fabs(p.gravity - sim::World::kGravity) < 0.002f);
    CHECK(std::fabs(p.yScale - 1.f / sim::World::kVyScale) < 0.002f);
    CHECK(std::fabs(p.takeoffVy - (sim::World::kJumpVy - sim::World::kGravity)) < 0.05f);
    CHECK(std::fabs(p.floorY - sim::World::kHalf) < 0.5f);
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
        rig.predictor.simulate(Predictor::startFrom(o, false), o.obstacles, Plan{-1, false}, p);
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
    rig.predictor.simulate(Predictor::startFrom(o, false), o.obstacles, Plan{-1, false}, main);
    rig.predictor.simulate(Predictor::startFrom(o, false), o.obstacles, Plan{95, false}, alt);
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
    CHECK(std::fabs(m2.cube(false).gravity - rig.model.cube(false).gravity) < 1e-5f);
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
                    std::printf(" %s@%.0f%s%s", st[k].action ? "P" : "R", st[k].x, st[k].onGround ? "g" : "a", st[k].forced ? "*" : "");
            auto const& d = agent.lastReport();
            std::printf(" | kind=%s\n", failureKindName(d.kind));
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
