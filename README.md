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
| **Use Prediction** | Simulate the player's path before each decision, in every game mode (see *Trajectory prediction*). |
| **Show Trajectory** / **Show Alternative Path** | Draw the predicted path of the action being executed, labelled with the action and its safety; optionally also the path for the other button state. |
| **Prediction Horizon** | 0.3–1.5 s of simulated future (default 0.75 s). |
| **Show Status Panel** / **Show Debug Data** | The in-level overlay, and its debug lines (observation, Q-values, memory sizes, physics calibration, planner cost, prediction error). |

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
2. **Policy.** Each state stores Q(release) and Q(hold), and the AI picks the better one. A state with no experience of its own uses the pattern table instead (read live, so knowledge gained later still reaches it). For example, "spike 2 blocks ahead at 1× while grounded" transfers to unseen places and other levels. The prior is never mixed with a state's own record: own experience always decides alone, so learning elsewhere cannot silently change a decision that works here. Because pattern keys include speed and mode, timings are never transferred between different speeds or modes.
3. **Reward.** +1 per block of progress, −10 for dying, +50 for completing. Progress is the main signal, so the AI cannot farm reward without moving forward. Each reward is paid once per event, so repeating an attempt earns nothing extra. A "new personal best" bonus (paid once, on the decision that first passes the old record) exists as `LearnerConfig::newBestReward`, but it is **off by default**: measured on the benchmark it slowed learning (blocks: mean 14 → 40 attempts), because progress is already rewarded and the bonus inflated whatever was done on the way.
4. **Update.** At the end of each attempt the whole trajectory is replayed backwards with λ-returns (γ = 0.995 and λ = 0.99 per tick), with terminal handling (no bootstrap after a death or completion). See *Credit assignment* below for what is blamed.
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
   An experiment plays only the change it tests as written. Around that change, the reference attempt's actions are reproduced **only while the player is in exactly the reference attempt's state** (same state key), so the comparison is fair. Once the change has made the situation different, decisions are made normally. Nothing is replayed by position. These reproduced decisions are labelled *reference replay* in the diagnostics.
7. **Keeping what works.** When an experiment succeeds, its tested change and the state-matched decisions leading into it are committed into the Q-table. Its timing window is also stored as a constraint, so fixing a later obstacle never silently undoes an earlier fix. Changes that make things worse are dropped: the next experiment starts again from the best-known attempt.
8. **Stuck detection.** After 20 attempts without a new best, exploration widens around the trouble spot and the search reaches further back. Right after progress it narrows (half the exploration) to refine what just worked.
9. **Timing statistics.** Every successful jump over a hazard records its lead distance per mode and speed. These statistics seed future "add a press" experiments on any level.

### Contextual failure memory and credit assignment (2.0)

An audit of the 1.x learner found why it repeated mistakes and copied successful runs:
- timing experiments replayed the previous attempt's actions per x position over about 8 blocks, regardless of the player's actual state;
- after every attempt the furthest run's actions were written back into the Q-table for the whole solved part, overriding newer evidence;
- deaths there were "fixed" by replaying that path up to 3 times;
- failures were remembered only per x position ("hotspots"), never per decision and situation;
- blame was spread over the last ~0.3 s by time, not by cause;
- the cross-level prior only seeded brand-new states.

What replaced it:

1. **Per-state outcome evidence.** Every state stores, per action, how often that decision was identified as *the cause* of a death (`fails`), how often the attempt got safely past it (`passes`), and the failures since its last pass (`streak`). A pass resets the streak, so newer evidence always overrides older evidence. The same evidence is kept per pattern key (across levels). It is saved in the memory files.
2. **Credit assignment.** After a death, one decision is identified as the cause; the rest are not blamed:
   - With trusted predictions, the cause is the **last decision whose alternative still had a way through** (recorded at decision time). Later decisions, where both actions were predicted to collide, are not blamed. Their value update also applies to the other action, so the agent does not switch around pointlessly in a doomed state. Earlier decisions are judged by the best value available at the cause, not by the death (a Watkins-style trace cut).
   - A predicted way through that has itself failed repeatedly in reality does not count (prediction vs. reality).
   - If the path that died had been predicted *safe*, it is a model error. The physics model learns from it, and the prediction is not used to blame a decision.
   - Without predictions, the failure analysis' suspect is used: for example the press for "jumped too early", or the wait before it for "too late".
   - If both actions have already failed in that state, neither can be the cause, so the search moves to the decision before.
   - Decisions safely before the death (more than 0.5 s and before the cause) count as passes. Those in between get no claim either way.
3. **Repeated-failure detection.** Before acting, the agent checks the evidence for exactly this state. If the chosen decision has failed twice in a row here, and the alternative less often:
   - it reports *"Repeated failure detected: an immediate jump failed 2 times here. Testing a delayed jump."*;
   - it evaluates the alternative with the trajectory prediction when available. If the alternative is predicted to collide while the current choice is not, the switch is rejected (*"…the cause is probably earlier"*), and credit assignment looks further back;
   - otherwise it plays the alternative, and the outcome becomes new evidence.

   This applies to experiments and reference replays too. Nothing repeats a known mistake in the same situation, and it is not a global blacklist: other places and other states are unaffected.
4. **Known good.** If one action got safely through a state at least 3 times without failing since, and the learned choice has no such record, the reliable action is taken. This guards against values disturbed by deaths further on, without replaying any run. (In the simulator benchmarks this override never actually triggered.)
5. **No copying.** The furthest run (`bestPath`) is kept for inspection, export and to know how far the level is solved. It is never replayed or written into the policy. What it did well reaches later attempts only through the values and evidence of the states it visited.
6. **Physics vs. policy.** The physics calibrator now detects a lasting change: 40 consecutive samples that disagree with the estimate but agree with each other replace the estimate (shown as "estimates replaced"). Before, after 20 samples, anything 10% off the estimate was rejected forever. Policy blame never comes from a wrong prediction.
7. **Bounded memory.** Above 400,000 states per level, or 200,000 pattern states, the least experienced states are dropped. Memory files are now format 2 (with evidence); format-1 files still load, and files from a newer version are refused.

The status panel shows each decision's source (learned, similar situations, planner, exploring, experiment, reference replay, known good, avoiding repeated failure, collision avoided), the last notable reason, and which decision was blamed for the last death and why. With *Show Debug Data* on, it also shows the evidence for the current state and the learning counters.

### Trajectory prediction (all game modes)
Learning only from deaths is slow: every bad timing has to be tried once. The
predictor simulates the cube's future instead, so most bad timings are rejected
before they are tried.

**Physics model** (`core/Physics`). One model per game mode and size.

Per 240 Hz step, every mode accelerates vertically (`vy += acc`, with
`y += vy * k` and velocity caps), with a separate acceleration for held and
released. What the button does differs by mode:

| Mode | Button |
|---|---|
| Cube | jump while touching the ground (holding re-jumps) |
| Robot | jump while touching the ground, boosted while held for a limited time; letting go ends the boost |
| Ship | changes the acceleration (up while held, down when released) |
| UFO | each new press kicks upward, also in the air |
| Ball | flips gravity while touching the ground |
| Swing | each new press flips gravity, also in the air |
| Spider | a new press on the ground teleports to the opposite surface and flips gravity |
| Wave | sets the velocity directly: a 45°-style diagonal up while held, down when released |

The cube defaults were measured from GD 2.2081 itself (saved traces: g = 0.2156,
k = 0.225, take-off vy = 10.964, i.e. the 11.18 jump impulse minus one step of
gravity; x = 1.298 units/step at 1×). The other modes start from rough guesses.

An online calibrator learns every parameter from your own play:
- gravity per mode, plus held/released acceleration (ship, robot);
- velocity caps and k;
- jump/flap impulse and post-flip speed;
- wave slope and robot boost window;
- level floor and x speed per speed class.

It detects jumps, flaps, flips and teleports from the observed state changes.
Single odd frames (orbs, pads, teleport portals) are rejected as outliers, and
parameters only move after enough consistent samples. **A mode is only used for
decisions once it is calibrated:** that takes a few of its own jumps, flaps,
flips or teleports. The spider borrows the cube's gravity, because it rarely
free-falls. The calibration is saved in `patterns.json`.

**Floor and ceiling.** In corridor modes the playing area's floor and ceiling
are learned by touching them. A contact only counts when no block explains it,
and flying modes must stay pinned against the bound for several steps (a ship
turning around mid-air also has zero speed for one step). A bound that has not
been confirmed yet makes paths that touch it *uncertain*.

**Collision prediction** (`core/Predictor`). The simulation steps the player
forward, with gravity direction able to flip mid-path, against the game's own
collision rectangles
(`GameObject::getObjectRect`, so spike hitboxes are the real, small ones).
- Hazards kill on contact with the outer hitbox.
- Blocks are landed on from above (from below when upside down); flying modes
  slide along them. They kill only if the small inner hitbox enters them.
- Walking off edges and holding to re-jump on landing are modelled.
- Each path is classed as **Collision** (overlap deeper than the error margin),
  **Uncertain** (a near miss within the margin, contact between steps, or
  touching an unmodelled object: orbs, pads, portals, speed changes, slopes,
  one-way blocks) or **Safe**.
- The margin grows with the measured model error and the distance into the
  future, so an inaccurate model makes the AI more cautious, not more confident.

**Receding-horizon planner.** Before each decision it simulates the button
being down vs. up now, each followed by continuations shaped for the mode:
- cube/robot: hold until take-off, hold through the landing, or hold for 12, 30
  or 60 steps (robot boost);
- ship/wave: hold for d steps, hold from k steps on, or hover patterns;
- UFO/swing: single taps or repeated taps every r steps;
- ball/spider: single taps.

Delays k are tried on a coarse schedule, then refined to single steps around
the best timing and wherever the outcome flips. If the best "wait" plan would
press within two steps anyway, it presses now rather than procrastinating.

Only the first action of the best plan is executed. The next step re-plans from
the new real state, or every 2 steps for ship, wave, UFO and swing (never when a
press is due within 2 steps). Plans that would press only after the "never
press" path has already collided are skipped. While a cube, ball or spider is
airborne with the button up, the last plan is reused until about 20 steps
before landing.

A plan is only ever reused when nothing can have changed. Advice is returned only for exactly the state it was computed for (position, velocity, mode, button). It is never reused across steps while the executed input would have to change within 16 steps or may collide, nor once the player has left the path predicted for the executed input (more than 0.5 units off).

For each branch the predictor also reports:
- how long the button can stay in that state while some plan still does not collide, and while a plan is still fully *safe*: the latest moment the input must change;
- the path of simply **keeping the current button state**.

**Where the Q-table has no preference yet**, the planner's best plan decides.

### Safety layer: prediction controls the executed input (v0.2.1)

**The bug.** The overlay could show a safe path while the AI kept an input that crashed, and the last-moment warning came too late. The cause, traced in the code:

1. **Optimistic safety.** An action's predicted outcome is its *best continuation*: "hold now" counts as safe if "hold 2 steps, then release" is safe. The old check only blocked an action whose *every* continuation collides. But the next decision comes from the learned values again, not from that continuation. So the AI kept holding while "release in 2 steps" stayed possible, until holding was doomed. By then releasing usually was too (both ship tests below died this way).
2. **Decisions skipped.** While the discretised state did not change (up to 24 steps), `act()` kept the previous input without looking at the prediction at all.
3. **Stale advice.** The advice cache was keyed by tick and button only, so a different state in the same tick got the old result. Ship, wave, UFO and swing reused the previous step's plan.
4. **Debounce.** The minimum time between input changes could undo a safety switch, while the status still said "Collision avoided".
5. **Overlay.** The overlay drew the planner's favourite plan, not the action being executed.

The input path itself (`act()` → `handleButton` whenever the button state changes) was correct.

**The fix** (`Agent::act`, after the policy, planner and debounce have proposed an action):
- **The check judges the action that will really be executed.** Each candidate is ranked:
  - 0: predicted collision;
  - 1: no safe continuation, with a near miss within 12 steps;
  - 2: safe, but the input would have to change within the next 3 steps (`safetySlackSteps`). This applies when keeping the current input, or to any input in ship/wave. The planner's own choice is exempt, since it follows its plan and re-plans every step;
  - 3: safe with time to spare.
- **If the proposed action is not rank 3 and the other one ranks higher, the other one is executed**, overriding the policy, the planner and the debounce. The button really changes; tests check the applied input.
- If both collide, the one that collides later is executed and the decision is reported as **NO SAFE OPTION**, never as safe.
- **While a change may be needed within 16 steps (`dangerWindowSteps`), every step is checked** with a freshly computed prediction, instead of keeping the input while the state key is unchanged. The same happens when the player leaves the predicted path.
- **Nothing is latched.** As soon as the executed action is safe with time to spare, the policy is in control again.
- All windows are in physics steps (time), so they hold at every game speed. A mode change (portal) changes the state, so it is always a new decision.

**Diagnostics.** Every safety-critical decision is recorded (`SafetyRecord`), with:
- the player state;
- the predicted outcome, time to collision and remaining time to change for both actions, and for keeping the input;
- the proposed action and who proposed it, and the executed action;
- the verdict (safe / kept / SAFETY OVERRIDE / NO SAFE OPTION), and whether the prediction was current or stale;
- whether an alternative existed, and the reason in words, e.g. *"Safety override: keeping hold leaves 1 step(s) to change course; executing release instead: release is safe (input can stay for 120 steps)"*.

The status panel shows the current record. On every death, the last 8 records are written to the Geode log. The controller logs an error if the input it sent ever differs from the recorded decision.

**What it cannot do.** It cannot save the player once no action can avoid the collision any more, for example a cube already in the air over a spike. It acts as early as the prediction allows, so its reach is limited by the prediction horizon and the model's accuracy.

**Checking the model against reality.**
- Every 16 steps the last 32 real steps are re-simulated with the inputs that
  were actually used, and the end positions are compared. This gives drift per
  step, shown in the debug panel; high drift lowers trust.
- A one-step prediction error is shown too.
- If a path was predicted safe a moment before an actual death, that counts as a
  *misprediction*: caution rises and the event is logged with the predicted vs.
  actual position. Successful predictions slowly restore trust.
- The predictor is not used at all until the model is calibrated (a couple of
  jumps).

**Overlay** (`src/ui/TrajectoryOverlay`). One `CCDrawNode` in the level's object
layer is redrawn once per frame from geometry built in the core (so tests can
check it). It shows:
- **the path of the action actually being executed** (never a plan that is not being followed): green = safe, yellow = uncertain, red = collision, grey = not computed for the current step;
- in purple, what keeping the previous button state would have done, when the executed input differs from it (e.g. right after a safety override);
- the predicted landing point and nearby spike hitboxes;
- the obstacle the path runs into, with the steps until the hit;
- optionally the other action's path, in blue;
- a label with the executed action and its safety, e.g. `HOLD - safe`, `RELEASE - UNSAFE, hit in 7 - NO SAFE OPTION`, `... - SAFETY OVERRIDE`, `... - calibrating, not used`, `... - prediction 1 step(s) old`.

With nothing relevant ahead, no path is drawn.

Until a mode is calibrated, its predictions are shown (marked as calibrating)
but not acted on.

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
- the 2.0 learning architecture:
  - **repeated failure**: a decision that failed twice in the same state is detected and the alternative is played, even though the values still favour the old choice. One failure is not enough, and a pass since then clears it;
  - **prediction veto**: a repeated failure whose alternative is predicted to collide is not switched;
  - **context**: a failure in one state changes nothing in other places or in other states at the same place;
  - **breaking a loop values cannot**: with no death penalty, plain Q-learning dies at the first spike on all 79 later attempts. With failure memory, the same agent gets past it and on through the level, with 0 deaths on an already known mistake;
  - **credit assignment**: one death blames exactly one decision (the wait before the spike). Earlier decisions are passed, the ones in between get no claim, and a completed run passes every decision;
  - **persistence**: the evidence survives save/reload exactly, format-1 files still load, future formats are refused, and the reloaded agent makes the same evidence-based decision;
  - **bounded memory**;
  - **generalisation**: the prior decides unseen states, also states created before it was learned, but never overrides a state's own experience;
  - **physics correction**: with gravity changed by 30% after calibration, the estimate is replaced (to the new value exactly) and the 32-step prediction error returns to 0.04 units. The agent with prediction still completes a level, and changing gravity back is recovered too;
  - **no copying**: an experiment reproduces reference actions only in identical states; values of states an attempt does not visit are left alone; a best-run decision that newer evidence says now fails is not repeated;
  - **learning regression with metrics**: every level × 3 seeds × with/without prediction, with attempts, regressions, known mistakes repeated, repeated failures detected, alternatives tried and helped, and deaths with a located cause;
- the agent **completing all four simulator levels**, with later attempts measurably further than early ones;
- **learned policy surviving save + reload**: a fresh agent with learning disabled completes the level from the saved file, while an untrained one does not;
- cross-level transfer through pattern memory;
- per-tick cost;
- **prediction controls the executed input** (real agent + predictor + simulated game input). A "stubborn" learned policy insists on one action each step, and the tests check:
  - **A**: a cube that will not jump before a spike, and one that wants to jump straight onto a spike, both survive;
  - **B**: a ship held toward spikes under the ceiling releases in time. Holding was safe at first; the held button is really released (**F**);
  - **C**: a ship kept low toward a wall climbs; also through a gap, with a policy alternating between "always hold" and "always release";
  - **D**: advice computed for one state is not reused for a different state in the same step, and the decision is based on the new state;
  - **E**: with every candidate colliding, the decision and the overlay say *NO SAFE OPTION* and never "safe";
  - **G**: on every step of these runs, the overlay's main path is the executed action's, its override label matches the record, and every record matches the input really applied;
  - no override after the danger has passed (not latched);
  - **H**: learning with prediction still completes ship and blocks, records per-state evidence and keeps it through save/reload;
  - the cost of a whole decision per step;
  - on the code before the fix, B, C and D failed (both ships crashed) and E reported nothing;
- trajectory prediction against the simulator:
  - physics calibration learning the simulator's (deliberately different) constants;
  - standing still before a spike;
  - every jump timing in a sweep over a single spike and over a triple spike
    (too early, too late, clearing), checking that no timing that dies is
    predicted safe;
  - landing positions on flat ground and on a block top;
  - 0.5×, 2× and 4× speed;
  - gravity portals (marked uncertain) and upside-down landing;
  - the overlay geometry matching the simulated path, collision point and hit hitbox;
  - the planner alone clearing levels;
  - learning speed with vs. without prediction;
  - calibration persistence;
  - misprediction caution;
  - no action before calibration.

### Results

Unit tests: 58 tests, 1935 checks, 0 failures.

Trajectory prediction against the simulator:
- calibration recovers the simulator's physics exactly (g 0.2200, k 0.2326,
  take-off 10.780, floor, x speed);
- standing still before a spike: collision predicted at step 125, actual 124;
- single-spike timing sweep: 106/111 timings classified correctly, 5 uncertain,
  0 falsely called safe;
- triple spike: 85/91 correct, 6 uncertain, 0 falsely called safe;
- landing on a block top: exact; on flat ground within 1.3 units (one step);
- at 0.5×, 2× and 4× speed the collision step is within 1 of the real one;
- the planner alone, without any learning, completes the basics, spikes and
  blocks levels with no deaths;
- every mode (cube, ship, ball, UFO, wave, robot, spider, swing):
  - calibration recovers the simulator's deliberately different constants;
  - predicting 32 steps ahead with random inputs: mean and max error 0.00
    units (103 samples per mode);
  - every predicted collision was confirmed when replayed, and **no path
    predicted safe died** (20–126 plans checked per mode);
  - the planner alone, without learning, completes each mode's obstacle course;
  - the learning agent with prediction completes each course on its first attempt;
- cost per physics step on average, planner alone: cube ~140 µs, ship/wave/UFO/swing
  ~215–240 µs, ball/robot ~285–300 µs, spider ~365 µs. That is about 10% more than
  before the safety layer (one extra simulation, less plan reuse near danger);
- cost of a whole decision of the learning agent with the safety layer (debug
  build): cube 95 µs, spider 182 µs, wave 217 µs, ship 227 µs per step. That is
  2–5% of one core at 1× speed and 9–22% at 4× training speed.

**Caveat:** the simulator implements the same structural rules for each mode
that the predictor assumes (which button does what), only with different
numbers. These results show that the calibration and planning work. They do
**not** prove the rules match Geometry Dash exactly; only in-game testing can.

With prediction, every simulator level is completed on attempt 1 (basics,
spikes, blocks) or attempt 3 (ship) over 3 seeds. Before the safety layer, ship
took 9–17 attempts. In the 5-seed benchmark, ship with prediction went from 10
to 5 attempts; every other line is unchanged.

Per-tick cost: about 0.7 µs per decision in the simulator. The end-of-attempt update (values, credit assignment and evidence) over about 1,200 decisions takes about 0.4 ms. Saving is done on a background thread.

Benchmark (`--bench`): attempts to first completion over 5 seeds, median / mean (lower is better). 1.x is the previous version (commit 6a6520a), built and run side by side:

| Level | Variant | 1.x | 2.0 |
|---|---|---|---|
| basics | default | 7 / 8 | **6 / 7** |
| | learning speed 0.1 / 1.0 | 9 / 11, 9 / 9 | **7 / 7, 6 / 7** |
| | Q-learning only | 8 / 8 | **7 / 7** |
| spikes | default | 16 / 16 | **14 / 16** |
| | learning speed 0.1 / 1.0 | 19 / 19, 13 / 193 | **15 / 16, 10 / 21** |
| | Q-learning only | 8 / 11 | 9 / 8 |
| blocks | default | 15 / 16 | **13 / 14** |
| | learning speed 0.1 / 1.0 | 18 / 840, 15 / 17 | 26 / 44, **13 / 13** |
| | Q-learning only | 225 / 188 (1 of 5 failed) | **93 / 85** (1 of 5 failed) |
| ship | default | 33 / 34 | **32 / 31** |
| | learning speed 0.1 / 1.0 | 282 / 254, 21 / 21 | **139 / 140, 18 / 19** |
| | Q-learning only | 303 / 264 (2 of 5 failed) | **165 / 166** |
| all | with trajectory prediction | 3, 3, 3, 10 | 3, 3, 3, 10 |

2.0 is better or equal on every line except two:
- blocks at learning speed 0.1 has a worse median (26 vs 18), but a far better mean (44 vs 840);
- spikes with Q-learning only is roughly equal (median 9 vs 8, mean 8 vs 11).

The learning-metrics test (3 seeds, without prediction) also shows:
- regressions into solved sections: 0–2 per run;
- deaths on a decision already known to fail there: 0 in every run except one ship run with prediction (1 of 16 deaths);
- a single cause located for nearly every death.

The ablation also measured each mechanism on blocks (mean attempts):
- without failure memory (measured while the new-best bonus was still on): default 40 → 110 with 1 of 5 seeds failing, and 4 of 5 failing at learning speed 0.1;
- blind replay by position instead of state-matched replay: better on blocks, but worse on basics, spikes and ship (for example ship at speed 0.1: 139 → 307), so it was not kept;
- new-best bonus: see *Reward* above.

The failure analysis is what makes the harder levels solvable at all.

Cross-level transfer: after training on *spikes*, the *basics* level was completed in 1 attempt with shared pattern memory, versus 6 from scratch.

### In-game test (Stereo Madness, normal mode, GD 2.2081 + Geode 5.11.1)

- The mod loads, and the AI starts and stops with F8.
- The observer indexed 1,397 relevant objects.
- Deaths are detected and analysed, the level auto-retries, and memory is written to disk.
- Progress rose from 1.9% on attempt 1 to 13.3% by attempt 12, each attempt using a change derived from the previous death.

Later in-game runs on Stereo Madness (before trajectory prediction) reached
51.5% after 386 attempts. Several problems were found in game and fixed:
- regressions into already-solved sections, at first fixed by best-path protection. 2.0 replaced that with per-state evidence (see *Contextual failure memory*), because it overrode newer evidence;
- ship deaths misclassified by obstacle centre, fixed by using obstacle edges;
- ship climbs needing a hold-range search instead of a single tap.

**The 2.0 learning changes (failure memory, credit assignment, state-matched replay) and the v0.2.1 safety layer have not been tested in game yet**, only in the simulator.

### In-game test procedure for the safety layer (v0.2.1)

Install: keep a copy of the working v0.2.0 package, then put
`aqvose.ai-geode-v0.2.1.geode` into Geometry Dash's `geode/mods` folder
(replacing `aqvose.ai-geode.geode`) and restart the game. To go back, put the
v0.2.0 package back the same way. Memory files stay compatible both ways
(format 2).

Setup:
- settings *Use Prediction*, *Show Trajectory*, *Show Status Panel* and *Show Debug Data* on, *Training Speed* 1×;
- use a level (e.g. made in the editor) with a ship portal near the start and, a few blocks later, a wall rising from the floor to half the corridor height, then a row of spikes hanging from the ceiling, then a gap between two blocks. Repeat the same section three times;
- calibrate the ship first: let the AI fly a few attempts until the panel no longer says *calibrating*.

Check, on several attempts:
1. **The overlay follows the executed action.** The label's HOLD/RELEASE changes exactly when the ship's input changes (the ship visibly starts climbing or falling). The coloured path starts at the ship and matches its next movement.
2. **An override changes the real input.** Approaching the wall low, the label should show *SAFETY OVERRIDE* with the purple "keep input" path running into the wall. The ship must start climbing at that moment, and the panel's *Executing* line must say *button down* even if the policy *wanted the other*.
3. **No blind continuation.** Under the hanging spikes, the ship must not stay pinned to the ceiling until the path turns red: release must happen while the path is still green or yellow.
4. **Panel and overlay agree.** The panel's *Safety* line gives the reason. On a death, the Geode log shows the last *Safety t=...* lines: the executed action, and whether every option was predicted to collide (NO SAFE OPTION) or the prediction was wrong (*MISPREDICTION* in the *Prediction check* line).
5. **The policy gets control back.** After the obstacle the label shows plain *HOLD - safe* / *RELEASE - safe*, without the override label.
6. **Learning still happens.** Deaths are still analysed (*Cause:* line in the panel), and attempts get further over time.

Report:
- any death where the log's last record says *safe* or *kept* with a current prediction: that is a model error;
- any input mismatch error in the log: that would be a controller bug.

**Trajectory prediction has not been verified in game yet.** Its accuracy on the
real game is measured by the debug panel (step error, 32-step error, drift,
mispredictions) and in the log at every death (`Prediction check: ...`). The
physics defaults come from real game data, but collision details such as hitbox
sizes and landing tolerance are still unconfirmed.

---

## Known limitations

- **Platformer levels** are not supported: they need left/right input. The AI disables itself there and says why.
- **Two-player levels**: only player 1 is controlled.
- **Moving obstacles**: hazards moved by triggers are indexed by their starting x. A padded lookup tolerates small movements, but heavily animated levels give the cross-level pattern features less accurate input. The per-level table is unaffected, because it is keyed by the player's own state.
- **Hard levels still take many attempts**, and very precise sections (frame-perfect inputs, tight wave corridors) may never be beaten. The learned table's timing resolution is a 3-unit x cell, about 2 ticks at 1× speed. The planner itself works per tick.
- **Trajectory prediction limits:**
  - player hitbox sizes (30 units, inner 9 units; mini 18/5.4) and the 2-unit landing tolerance are assumptions, not values read from the game;
  - orbs, pads, portals, speed changes, slopes and moving objects are not simulated: paths touching them are marked uncertain, and objects moved by triggers use their current position only;
  - all non-cube physics (and mini sizes) start from guesses until calibrated, and the button rule per mode is an assumption about GD that is not yet verified in game (for example whether a held spider re-teleports, or exactly when a robot's boost ends);
  - the wave hitbox (12 units, inner 6) and the ship hitbox (24 units) are guesses;
  - the game's own corridor bounds (`m_portalY` / `m_lastModeHeight`) are not used yet because their meaning is unconfirmed; bounds are learned by contact instead;
  - the planner tries a limited set of input shapes, so a sequence it never tries (for example an unusual tap rhythm) cannot be found by planning, only by learning.
- **The safety layer acts on predictions.** If the physics model or the hitboxes are wrong for a mode (see *Trajectory prediction limits*), it can override a correct policy decision, or fail to see a collision. Every override is recorded with its reason, so wrong ones can be found in the log. It only uses trusted, calibrated modes.
- **The safety slack (3 steps) and re-check window (16 steps) are fixed values**, measured in the simulator only.
- **Failure memory is per exact state.** It is keyed by the 3-unit x cell plus the physics state, so it reacts to repeated failures in a deterministic replay of the same situation. In a level where the same situation is reached in slightly different states every time (non-deterministic mods, moving objects), streaks build up slowly. The cross-level pattern evidence is recorded too, but it is only used through the values, not to switch decisions.
- **Credit assignment without prediction** relies on the failure analysis' guess. It walks back one decision per two deaths when that guess is wrong, which is slower than the timing experiments, so both run together.
- **Determinism**: GD 2.2's physics are deterministic for identical inputs, which the learning relies on. Mods that change input timing, such as *Click Between Frames*, or physics settings can make attempts less repeatable and learning slower.
- The memory of a local/editor level is keyed by its content, so editing the level starts a fresh memory.
- **Training Speed** above 1× desyncs the music.
- Bot completions count as completions in your save. Don't use it on rated online levels if leaderboards or stars matter to you.

## Repository layout

```
core/      learning library (Agent, FailureAnalyzer, Memory, Storage, Json,
           Physics, Predictor, Overlay) – no Geode dependency
tests/     unit tests, GD-like simulator, benchmark
src/       Geode mod: Observer, AIController, SaveWorker, hooks/, ui/ (StatusPanel, AIMenuPopup, TrajectoryOverlay)
mod.json   Geode metadata and settings
```

## License

No license has been chosen for this repository yet.
