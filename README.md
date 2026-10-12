# AI Geode

A [Geode](https://geode-sdk.org) mod for **Geometry Dash 2.2081** that plays classic levels by itself and **learns from its own deaths**. It is not a macro recorder, auto-clicker or hard-coded bot. Its decisions come from a value table it builds through trial and error, and every death changes it.

- Targets: Geometry Dash **2.2081**, Geode **5.11.1**, Windows.
- Mod id: `aqvose.ai-geode`.

---

## Quick start

1. Install the `.geode` file: drop it into `Geometry Dash/geode/mods/` and restart the game.
2. Open a level and press **F8** (rebindable), or open the pause menu and click the green **AI** button.
3. Watch the status panel in the top-left corner. Press **F8** again to stop at once; the button is released immediately.

When the AI is on, your own jump inputs in that level are ignored so the two never fight over the button.

## Settings and UI

All settings live in the mod's Geode settings page. Most also appear in the in-game **AI** popup in the pause menu.

| Setting | Meaning |
|---|---|
| **AI Player** | Autonomous play on/off. |
| **Toggle AI** (keybind, F8) | Emergency stop/start, usable at any time. |
| **Learning** | When off, the AI replays what it knows without changing its memory. |
| **Learning Speed** | 0–1. Higher means larger learning rate and larger timing shifts per experiment. |
| **Auto Retry** + **Respawn Delay** | Restart automatically after a death. |
| **Practice Mode Learning** | Plays in practice mode and places its own checkpoints (see below). |
| **Training Speed** | 1–4× game speed while the AI plays. Physics stay at 240 steps/s, so learning is unaffected, but music desyncs. |
| **Show Status Panel** / **Show Debug Data** | The in-level overlay, and its debug lines (observation, Q-values, memory sizes). |

The **status panel** shows:
- the current state (Observing / Playing / Learning / Retrying / Completed);
- the attempt number, deaths, current and best progress, and completion rate over recent attempts;
- why the last attempt failed and where;
- what the AI changed afterwards;
- whether the change got further than before.

The **AI popup** in the pause menu holds:
- the toggles above;
- learning speed;
- memory stats for the level;
- **Reset Level / Reset All** (both ask for confirmation);
- **Export / Import**, which saves or loads a level's memory as JSON;
- **Open Folder** and **All Settings**.

Memory is stored under `%LOCALAPPDATA%\GeometryDash\geode\mods\aqvose.ai-geode\`:
- `levels/<level-key>.json` holds one file per level;
- `patterns.json` holds knowledge shared across levels.

Each write goes to a temporary file followed by a rename. The previous version is kept as `.bak` and used automatically if the main file is ever corrupt.

---

## How it works

```
 Geometry Dash ──hooks──▶ src/  (Geode glue)                       core/  (plain C++, no Geode)
  processCommands  ─────▶ Observer: PlayerObject + level objects ─▶ Observation
  (once per 240 Hz tick)  AIController ── act(obs) ──────────────▶ Agent ─▶ hold / release
  handleButton ◀───────── inject input ◀───────────────────────────┘
  destroyPlayer ────────▶ DeathInfo (killer object, relative pos) ▶ Agent::onDeath ─▶ FailureAnalyzer
  levelComplete/reset ──▶ attempt bookkeeping                      │                  │
  SaveWorker (thread) ◀── snapshot ◀───────────────────────────────┘    Q-update + next experiment
```

The learning code in `core/` knows nothing about Geode. That is what lets it be unit-tested against a simulator (see *Testing*).

### Observation (real game state, no screen capture)
From `PlayerObject` the observer reads:
- x/y position and vertical velocity;
- game mode (cube, ship, ball, UFO, wave, robot, spider, swing);
- grounded, upside-down and mini flags;
- speed-portal speed and death state.

From the level's `GameObject`s it uses an index of hazards, solids and orbs sorted by x. That gives the **nearest hazard, solid block and orb ahead** as (dx, dy) relative to the player, gravity-normalised. Progress comes from `x / m_levelLength`. Time is the GD physics tick (`m_levelTime × 240`).

### Acting
Inputs go through `GJBaseGameLayer::handleButton` from a `processCommands` hook, once per physics tick. That is the same mechanism established Geode replay bots use. A decision is taken whenever the discretised state changes. The button can never toggle faster than every 2 ticks, and decisions always use the current tick's state.

### Learning: Q-learning plus failure-driven timing experiments
1. **State.** The per-level state is (x cell of 3 units, mode, grounded, gravity, mini, button held, y band, vy band, speed). The cross-level pattern state is the same physics but, instead of x, the relative position of the nearby hazard, solid and orb.
2. **Policy.** Each state stores Q(release) and Q(hold), and the AI picks the better one. A new state is seeded from the pattern table, so for example "spike 2 blocks ahead at 1× while grounded" transfers to unseen places and other levels. Because pattern keys include speed and mode, timings are never transferred between different speeds or modes.
3. **Reward.** +1 per block of progress, −10 for dying, +50 for completing. Progress is the main signal, so the AI cannot farm reward without moving forward.
4. **Update.** At the end of each attempt the whole trajectory is replayed backwards with λ-returns (γ = 0.995 and λ = 0.99 per tick). Blame for a death therefore spreads over roughly the last 0.3 s of decisions in proportion to time, instead of always landing on the final input.
5. **Failure analysis.** `FailureAnalyzer` looks at the last ~5 blocks of the trajectory and at what killed the player (killer object above, below or ahead, rising or falling, which presses actually did something). It classifies the death as one of:
   - *no jump*;
   - *jumped too early* (came down onto it);
   - *jumped too late* (hit it on the way up);
   - *held too long* (flew into something above, or a long hold made an extra jump on landing);
   - *released too long*;
   - *hit a wall*.
6. **Timing experiments.** From that diagnosis the agent proposes one concrete change to the best-known way of playing that section, for example "press 9 units later", "release 4 cells earlier" or "add a press 1.6 blocks before the obstacle", and plays it. The search is a directed bisection:
   - "still too early" pushes the next try further in the same direction;
   - flipping from early to late halves the step;
   - a try that changes nothing, or makes things worse, counts as evidence too.
   - If every variation of the suspected input fails, it **escalates** to an earlier input, because the real cause may be a decision made before the obstacle.
7. **Keeping what works.** A successful experiment is committed into the Q-table, so the greedy policy repeats it. Its timing window is also stored as a constraint, so fixing a later obstacle never silently undoes an earlier fix. Changes that make things worse are dropped: the next experiment starts again from the best-known attempt.
8. **Stuck detection.** After 20 attempts without a new best, exploration widens around the trouble spot and the search reaches further back.
9. **Timing statistics.** Every successful jump over a hazard records its lead distance per mode and speed. These statistics seed future "add a press" experiments on any level.

### Practice-mode learning
With **Practice Mode Learning** on, the AI:
- switches the level to practice mode;
- places a checkpoint roughly every 6 blocks, but only on ground it has already survived past, at a calm moment, with no hazard close ahead;
- removes the last checkpoint after 15 deaths without progress past it, in case it was placed in an unwinnable spot.

Learning is keyed by position, so knowledge gained from checkpoints applies directly to full runs.

---

## Building

Requirements:
- CMake 3.29+, Ninja, and LLVM/Clang (clang-cl), as in the Geode docs;
- the MSVC C++ runtime libraries and the Windows SDK (Visual Studio Build Tools, or [xwin](https://github.com/Jake-Shadle/xwin));
- [Geode CLI](https://github.com/geode-sdk/cli) and the Geode SDK **v5.11.1** (`GEODE_SDK` env var), plus the prebuilt loader binaries (`geode sdk install-binaries`).

```bash
cmake -S . -B build -G Ninja -DCMAKE_C_COMPILER=clang-cl -DCMAKE_CXX_COMPILER=clang-cl -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

```bash
cmake --build build
```

This produces `build/aqvose.ai-geode.geode`, and by Geode's default also installs it into the active Geode CLI profile's game folder.

The development machine for this repo had no Visual Studio C++ workload installed. It used the MSVC CRT/STL and Windows SDK fetched by xwin, with a small toolchain file:

```cmake
set(CMAKE_C_COMPILER clang-cl)
set(CMAKE_CXX_COMPILER clang-cl)
set(CMAKE_LINKER lld-link)
set(CMAKE_C_FLAGS_INIT "/winsysroot <xwin-dir>")
set(CMAKE_CXX_FLAGS_INIT "/winsysroot <xwin-dir>")
foreach(kind EXE SHARED MODULE)
    set(CMAKE_${kind}_LINKER_FLAGS_INIT "/winsysroot:<xwin-dir>")
endforeach()
set(CMAKE_TRY_COMPILE_CONFIGURATION Release)   # xwin ships only the release CRT
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
```

xwin writes `lib/x86_64` folders; `/winsysroot` expects `lib/x64`, so add `x64` junctions pointing at them.

## Testing

```bash
cmake -S tests -B build-tests -G Ninja -DCMAKE_CXX_COMPILER=clang-cl
```

```bash
cmake --build build-tests
```

```bash
build-tests/aigd_tests
```

Add `--bench` to also run the multi-seed learning benchmark.

`tests/` contains unit tests and a small deterministic **GD-like simulator** (`tests/Sim.hpp`). The simulator has 240 Hz steps, cube jumps with hold-to-rejump, ship flight, spikes and solid blocks. A breadth-first search proves the cube test levels are solvable before the agent is asked to solve them. The tests cover:

- the JSON reader/writer, malformed input, and state-key packing;
- failure classification, including upside-down gravity, ship cases, and escalation to earlier inputs;
- the Q-update penalising the action before a death, and **learning-off leaving memory untouched**;
- storage round trips, atomic writes, `.bak` recovery from a corrupted file, and rejection of foreign files;
- the agent **changing its behaviour after a failure**;
- the agent **completing all four simulator levels**, with later attempts measurably further than early ones;
- **learned policy surviving save + reload**: a fresh agent with learning disabled completes the level from the saved file, while an untrained one does not;
- cross-level transfer through pattern memory;
- per-tick cost.

### Results (as of this commit)

Unit tests: 17 tests, 335 checks, 0 failures.

Per-tick cost: about 0.5 µs per decision in the simulator. The end-of-attempt update over about 1,100 decisions takes about 0.25 ms. Saving is done on a background thread.

Benchmark: attempts to first completion, median over 5 seeds (lower is better):

| Level | Q-learning only | **Default (Q + failure analysis)** | speed 1.0 |
|---|---|---|---|
| basics (single spikes) | 10 | **7** | 7 |
| spikes (doubles/triples) | **9** | 31 | 29 |
| blocks (stairs, platforms) | failed 3 of 5 | **17** | 16 |
| ship (cube→ship→cube) | failed 4 of 5 | **119** | 106 |

The failure analysis is what makes the harder levels solvable at all. Plain Q-learning is faster on the simplest spike patterns. A "Q-learning first, experiments after N deaths" hybrid was measured as well and rejected, because it broke the block level on every seed.

Cross-level transfer: after training on *spikes*, the *basics* level was completed in 1 attempt with shared pattern memory, versus 7 from scratch.

### In-game test (Stereo Madness, normal mode, GD 2.2081 + Geode 5.11.1)

- The mod loads, and the AI starts and stops with F8.
- The observer indexed 1,397 relevant objects.
- Deaths are detected and analysed, the level auto-retries, and memory is written to disk.
- Progress rose from 1.9% on attempt 1 to 13.3% by attempt 12, each attempt using a change derived from the previous death.

Later in-game results are in [TESTING.md](TESTING.md).

---

## Known limitations

- **Platformer levels** are not supported: they need left/right input. The AI disables itself there and says why.
- **Two-player levels**: only player 1 is controlled.
- **Moving obstacles**: hazards moved by triggers are indexed by their starting x. A padded lookup tolerates small movements, but heavily animated levels give the cross-level pattern features less accurate input. The per-level table is unaffected, because it is keyed by the player's own state.
- **Exact physics are not modelled.** The AI learns by trying, so hard levels can take hundreds or thousands of attempts, and very precise sections (frame-perfect inputs, tight wave corridors) may never be beaten. Timing resolution is a 3-unit x cell, about 2 ticks at 1× speed.
- **Determinism**: GD 2.2's physics are deterministic for identical inputs, which the learning relies on. Mods that change input timing, such as *Click Between Frames*, or physics settings can make attempts less repeatable and learning slower.
- The memory of a local/editor level is keyed by its content, so editing the level starts a fresh memory.
- **Training Speed** above 1× desyncs the music.
- Bot completions count as completions in your save. Don't use it on rated online levels if leaderboards or stars matter to you.

## Repository layout

```
core/      learning library (Agent, FailureAnalyzer, Memory, Storage, Json) – no Geode dependency
tests/     unit tests, GD-like simulator, benchmark
src/       Geode mod: Observer, AIController, SaveWorker, hooks/, ui/
mod.json   Geode metadata and settings
```

## License

No license has been chosen for this repository yet.
