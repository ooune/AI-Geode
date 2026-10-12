#define _CRT_SECURE_NO_WARNINGS
// Unit and behaviour tests for the AI-Geode learning core.
//
//   aigd_tests            run all tests
//   aigd_tests --bench    also run the learning benchmark (several seeds/configs)
//   aigd_tests <filter>   run tests whose name contains <filter>

#include "Sim.hpp"

#include <aigd/Agent.hpp>
#include <aigd/FailureAnalyzer.hpp>
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

AttemptResult runAttempt(Agent& agent, sim::Level const& level, int maxTicks = 100000) {
    sim::World world(level);
    agent.beginAttempt(world.observe(), false);
    for (int t = 0; t < maxTicks; ++t) {
        Observation obs = world.observe();
        bool hold = agent.act(obs);
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

TrainResult train(Agent& agent, sim::Level const& level, int maxAttempts) {
    TrainResult r;
    for (int i = 0; i < maxAttempts; ++i) {
        auto a = runAttempt(agent, level);
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
// Benchmark (not run by default)

static void runBench() {
    std::printf("\n=== Learning benchmark: attempts to first completion (5 seeds) ===\n");
    struct Variant {
        char const* name;
        LearnerConfig cfg;
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
                auto r = train(agent, level, 3000);
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
