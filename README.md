# AI Geode

**v0.6.0:** orb activation/skip prediction, dash modelling, interaction overlays
and reusable orb strategies are implemented. See [validation and remaining game
checks](V060-VALIDATION.md) for evidence and limitations. Installation and loading
are verified; orb behaviour in Geometry Dash is not yet verified. Older version
notes below describe their respective releases.

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
- **Reset Level / Export Level / Import Level**: one level's own memory;
- **Reset Shared / Export Shared / Import Shared**: the cross-level knowledge (physics, timings, pattern prior and the strategy library), without touching any level's own memory;
- **Reset All** (everything), **Open Folder** and **All Settings**.

Every reset asks for confirmation.

Memory is stored under `%LOCALAPPDATA%\GeometryDash\geode\mods\aqvose.ai-geode\`:
- `levels/<level-key>.json` holds one file per level (layer C);
- `patterns.json` holds general knowledge shared across levels (layer A);
- `strategies.json` holds the strategy library (layer B, from v0.4.0).

All three are versioned. Files from older versions load unchanged (there is simply no strategy library yet), and a file from a newer version is refused rather than misread.

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

### Decision pipeline: prediction quality, no unnecessary jumps (v0.2.2)

**What the in-game log of v0.2.1 showed.** 36 attempts on main level 4 (Dry Out):
- every death had a *Prediction check* line saying the executed path was predicted to collide only 3–5 steps ahead;
- there was **not a single safety record**. That is only possible if prediction was not trusted for those decisions: the planner, collision checks and safety layer never acted, while the overlay still drew paths;
- the last 22 deaths were all at 21.4% ("held too long (obstacle above)" / "jumped too late"). Every experiment there added or moved a jump, never removed one.

**Root causes, traced in the code:**
1. **Trust collapsed through the accuracy measurement itself.** Drift is measured by re-simulating the last 32 real steps. If that re-simulation "died", it counted as a 20-unit error, using the planner's error margins and the model's hitboxes. With hitbox sizes that do not exactly match the game, every graze that the player survived counted as a large model error. The higher drift widened the margins, which caused more false "deaths". One or two of these push drift over the 0.15 limit, and then the prediction is ignored.
2. **The planner preferred jumping.** A plan's score added the clearance to the nearest hazard *or block* (capped at 30 units), and nothing cost an input. Standing on a block counted as only 10.5 units of clearance (the inner hitbox to the block below), while a jump in the air scored up to 30. So whenever there were blocks or hazards nearby, a jump beat walking on.
3. **The learned policy's jumps went unchecked.** If both actions were "safe enough", the policy won, even when waiting was fully safe and the jump was not needed. A jump with a possible hit more than 12 steps ahead counted as just as safe as waiting. Where nothing was ahead at all, no prediction was made and any jump went through. In unseen states, the cross-level prior chose "press" on any tiny value difference.
4. **Learning could not remove a jump.** The failure analysis' experiments could add a press or move one, never remove one.
5. **Input timing was assumed, never measured.** The model assumed the button acts on the next physics step. If the game (or a mod such as Click Between Frames) applies it later, every planned jump is mistimed.

**Changes:**
- **Prediction accuracy is measured on movement only.** The re-simulation ignores hazards and margins and compares where the player really went. In a test where the model sees spikes 2 units bigger than they are, drift stays at 0.000 and the prediction stays trusted.
- **The input latency is measured.** Every accuracy check re-simulates the recent steps assuming 0, 1 and 2 steps of input delay. Once the evidence is clear (windows with an input change, at least twice as accurate), planning uses the best fit: the first steps of every candidate run with the buttons already sent. A simulated 1-step delay is detected exactly (error 1.53 / 0.00 / 1.57 units for 0 / 1 / 2 steps). Without a delay the estimate stays at 0.
- **Every candidate includes "never press".** Besides pressing now, waiting with every delayed-press timing, and keeping the current input, the predictor simulates releasing and not pressing at all within the horizon. Each candidate gets its outcome, first collision, time to collision, how long its input can stay, and the number of presses it costs.
- **The planner's objective prefers fewer inputs.** Safety first. Then clearance, capped at 10 units and counting only hazards and the *sides* of blocks (standing on a block is not a near miss). Then a cost per press. Among equally safe plans, not pressing wins; a jump is chosen because it is needed or clearly safer.
- **Candidate ranking:** collision < possible hit anywhere in the horizon < must change within the slack < unmodelled object ahead < safe. The executed action is replaced whenever it is not fully safe and the other ranks higher.
- **No press that is not needed now.** With a trusted prediction, a proposed press (a jump, or holding into a landing re-jump) is dropped when releasing is fully safe and the planner's best-timed plan does not press now. That applies whoever proposed it: policy, prior or exploration. If a jump is needed later, the planner times it. Where nothing is ahead within the horizon, a jump is never needed. Exceptions:
  - steered modes (ship, wave);
  - an experiment's deliberate test;
  - a state where not pressing has already failed (reality beats the prediction).
- **The prior needs a real margin to press:** at least 0.5 value units (about half a block of progress).
- **Learning can remove a jump.** After two timing variations of the same jump have failed, the jump is tested *removed*: no press from there until past the place of death. Only when the geometry allows it (what killed the player was above the head of a player staying at take-off height), and once per trouble spot. The timing search is paused meanwhile and resumes if not jumping fails too. When the death analysis says the jump carried the player into something above, removal is tried first.
- **Diagnostics generated from the evaluations.** The status panel shows, for example:

  `hazard 2.1 blocks ahead (+0 high) | Jump: collision in 5 steps | No jump: predicted clear | Selected: No jump | Executed: No jump`

  Then come who proposed it (and whether that was learned from earlier attempts), whether the prediction is trusted (with drift and latency) or **PREDICTION NOT USED**, and the verdict. Next is the reason, e.g. *"No jump needed: not pressing is predicted clear for 180 steps (pressing now: ...); learned proposed it"* or *"Prediction not used: not trusted: the model drifts 0.211 units/step from the real game (limit 0.15)"*. Last are the counts this attempt: presses, unnecessary presses skipped, safety overrides, no safe option.
- **Per-attempt log line (`Decisions: ...`)** with:
  - decisions with a trusted vs. untrusted prediction, and the reason;
  - presses made and skipped, overrides;
  - drift, the estimated input latency with its re-simulation errors, caution, and the one-step error.

  This is the in-game evidence for whether predictions drive the AI.

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

### Real time, live plans and timing optimisation (v0.3.0)

**What a real game session showed (v0.2.2 on Dry Out).** Every attempt logged *0 decisions with a trusted prediction*. The measured drift was 0.25 units/step, and the re-simulation error was about 8 units for every assumed input latency. The one-step error, however, was 0.00. A trace the mod saved in the game explains it: consecutive observations were **1, 2 or 3 physics steps apart**. x advanced by exactly 1.2983, 2.5967 or 3.895 units at 1× speed, and the vertical speed changed by 1–3 steps of gravity. The level-time based tick counted many of these as a single step. As a result:
- the x speed was calibrated at 1.2× the true value (1.5579 instead of 1.2983);
- the 32-step accuracy check compared the model with about 43 real steps;
- the prediction was never trusted, so none of the prediction-based decisions acted in game.

The simulator observes every step, which is why no test had shown this.

**Step clock** (`core/StepClock`). The number of physics steps between two calls is read from the horizontal movement, which advances by exactly one speed unit per step. Respawns and teleports are treated as discontinuities. The controller uses it for every tick, and skips a call only when no step has happened. The predictor's history accepts gaps of up to 8 steps:
- calibration uses only single-step pairs;
- the accuracy check and the latency estimate replay the button state step by step across gaps;
- the average steps per call is measured.

A game-like test (one call every 1–3 steps) reproduces the in-game failure with one tick per call: x speed 2.08 instead of 1.30, drift 0.186, not trusted. With the step clock the speed is exact, drift is 0.000, the prediction is trusted, and basics, spikes and blocks are each completed on the first attempt.

**Audit of plans and actions (before v0.3.0).**
- The planner stored no future inputs. It re-planned every step (receding horizon) and executed only the first action, so it could not keep obsolete actions.
- The learned per-state values are keyed by the full state (x cell, height band, vertical-speed band, grounded, button), not by x alone.
- Obsolete actions could survive in two places:
  - **timing experiments** forced their tested inputs by x cell, whatever state the player was in when it got there. A jump tested for a grounded approach was "pressed" in the air, or held into a landing re-jump;
  - **holding the button in the air** re-jumps on landing in GD. When the prediction was not trusted (always, in game, see above), nothing stopped a hold that no longer made sense after an early landing.
- No component compared the real state with what a plan expected, so a plan was never "cancelled": it was simply re-made, or (for experiments) not re-made at all.

**Live plan** (`core/PlanManager`). The predicted path of the action just executed becomes the live plan. For every physics step it records the input and the state expected after that step (position, height, grounded or airborne), plus the mode, speed and gravity it was made for. It is kept apart from all learned memory: cancelling a plan never changes the Q-table, the evidence or the history.
- **On every observation the plan is validated** against exactly that step's expected state, not just the x position. It is invalidated when:
  - the player landed earlier than expected, or is still airborne where a landing was expected;
  - the height or position differs by more than a tolerance that grows with the model's measured drift;
  - the mode, speed, gravity or size changed;
  - the state changed within the step the plan was made in.

  An invalid plan's pending input changes are **cancelled**, the record keeps the reason, and a new decision is made **in the same step** from the actual state (also when the AI would otherwise keep its input).
- **A valid plan is followed** without a new search when all of these hold:
  - the model is trusted and no danger is near;
  - the plan is younger than 12 steps, has more than 30 steps left, and is predicted safe;
  - no experiment is running;
  - the plan was made by the prediction itself (planner, safety check or "no input needed").

  Behind a learned-policy decision the plan is only validated, and the policy keeps deciding. Following plans cut the agent's cost from 95–230 µs to 25–55 µs per step.
- **Learned test sequences are checked too.** An experiment is only played if, at its first changed cell, the player is in the reference attempt's state. For jump modes that means grounded vs. airborne and height within 4 units; for all modes the same mode. Otherwise it is cancelled ("Experiment cancelled: the state at its first changed cell differs from the reference attempt"), and its search starts again from current play instead of re-testing the outdated sequence.

**Timing optimisation** (`Predictor::advise`):
- coarse-to-fine timing search as before (wait 1, 2, 4, 6, 9, ... steps, then refinement around the best and wherever the outcome changes). It now refines only to the resolution the model and the real decision rate support: 1 step when every step is observed and the model is accurate, 2 or more in game (about 1.7 steps per call) or with drift;
- **two jumps for consecutive obstacles.** Considered only when no single-jump plan is safe. First-jump timings that cleared the first obstacle and only ran into the next one after landing (earliest, middle and latest of them) get a second press timed from their landing, coarse then refined. The best pair's first jump is refined as well. The pair is scored as a whole, so a first jump that sets up an easy second one wins. In the test, the earliest working first jump is in 7 steps, the best pair starts in 17, and the search chooses 18 (score 1000.7 vs. 1002.0 at best, 71 simulations);
- **learned timings first.** Press lead distances that worked are kept per kind of obstacle (mode, speed, size, gravity, obstacle height relative to the player) and globally per mode and speed. The matching one is converted into a press delay and simulated first. It is never executed without simulation. In the test, a good learned timing finds the first safe candidate after 4 simulations instead of 8, and a wrong one (from a different situation) is simulated, found worse, and not used;
- **bounded work:** at most 200 simulations per decision (configurable), and at most 48 more when a collision is less than 30 steps away (find a viable action fast). In the test, a budget of 40 still completes the blocks level, with never more than 42 simulations (40 + the two fixed reference paths);
- the objective is unchanged from v0.2.2: safety first, then capped clearance, then a cost per press.

**Diagnostics.**
- The status panel shows the live plan:
  - following it or made this step, and its age;
  - pending input changes;
  - its parameters (e.g. *press in 18 steps, second press 0 steps after landing*);
  - where its timing came from (search / learned timing / fallback);
  - candidates simulated, search stage and score, and landing x.
- It also shows the search resolution, whether a learned timing was tried and kept, whether the budget stopped the search, and how many simulations it took to find the first safe candidate.
- Actual vs. planned: height and position difference, grounded vs. airborne.
- The last cancellation, e.g. *"Landing detected earlier than expected (landed at y=45.0, planned airborne at y=43.6) - cancelling 2 pending actions; replanning from the actual state"*.
- Per attempt: plans made, invalidated, pending actions cancelled, decisions that followed a plan.
- The Geode log gets a `Plans:` line per attempt, with the measured physics steps per AI call, the calibrated x speed, the plan counts and the last cancellation.

### Cross-level learning (v0.4.0)

**Audit (before v0.4.0).** What already carried over between levels:
- the physics calibration;
- press-lead statistics per mode/speed and per kind of obstacle ahead (used to order the timing search);
- a pattern-keyed value table used as a prior in states with no experience of their own.

The prior is keyed by a coarse feature of the nearest hazard and solid only. It has no notion of an obstacle *group*, of landing space, or of which levels its evidence came from. It cannot tell "this worked on five levels" from "this worked once here", and a failure on a new level was averaged into everything else. Everything else (the per-state values, failure memory, experiments, the best run) is per level and keyed by position.

**Three layers, kept apart:**

| Layer | Contents | File | Reset |
|---|---|---|---|
| A: general knowledge | physics calibration, timing statistics, pattern prior | `patterns.json` | Reset Shared |
| B: strategy library | per kind of situation: strategies with success/failure statistics and provenance | `strategies.json` | Reset Shared |
| C: level memory | per-state values, failure memory, experiments, hotspots, history | `levels/*.json` | Reset Level |

**Transferable situations** (`core/Situation`). When the player is on the ground in cube or robot mode, the obstacle group ahead (within 110 steps) is described by relative geometry only:
- each element: on the walking path or above it, hazard or block, start and width in half-blocks from the group's start, bottom and top relative to the player's feet (gravity-normalised);
- adjacent blocks of the same height are merged, so a platform is one element however the level built it;
- the free landing space after the group (<1, 1–2, 2–4, 4+ blocks), and whether an orb, pad or portal is nearby;
- the mode, size, gravity and ground state.

Two keys are made from it. The **fine** key covers the whole layout and the landing space. The **coarse** key covers the first element plus the number of elements, for "similar" situations. Nothing in a description refers to a level position: the same group anywhere, in any level, gives the same keys (tested by moving a level by 7 blocks: 11 situations before, 11 after).

**Strategy library** (`core/Strategy`). Movement primitives:
- no jump;
- jump (pressed *lead* steps before contact; a small lead is a delayed jump);
- jump and hold through the landing;
- two jumps (the second *n* steps after landing);
- robot long jump (held *n* steps).

A strategy is one primitive in one timing window (leads in 4-step buckets), so a failure counts against the timing that failed, not against jumping in general. Each strategy records:
- its successes and failures;
- the leads that worked;
- the speeds it worked at;
- per level, how often it worked and failed there (provenance, up to 8 levels).

Leads are in physics steps, not distance. A jump takes the same time at every speed, so a timing learned at 1× is reused at 2× as the same number of steps, which is a longer distance. This is reported as *"Known strategy adapted for current speed"*.

**Learning from every attempt.** Every group the player approaches on the ground is tracked:
- what was executed is recorded: the first press and its lead (only if that group was in front of the player), holding through the landing, and a second press after landing;
- when the group is passed, it is a success for that strategy;
- if the player dies at the group, it is a failure.

This happens on every attempt, whoever made the decision (planner, learned policy, experiment or a strategy).

**Transfer is guidance, never dictation.**
- *With a trusted prediction*, the library's strategy for the group ahead becomes a whole candidate plan for the timing search (`Predictor::addSeed`). It is simulated first, like any other candidate, and executed only if it scores best. When the executed plan matches it, the message reads *"New level detected - using previously learned spike-clearing strategy (jump, press 13 steps before contact; worked 3 times, on 2 other levels) - confirmed by simulation"*. Otherwise it reads *"Previous strategy rejected - simulation predicts a collision ..."* or *"... the search found a better plan"*, with *"landing space differs from where it was learned"* or *"the layout is only similar"* when that is the case.
- *Without a trusted prediction* (not calibrated yet, or prediction off), in a state with no experience of its own, the strategy is played directly (decision source *learned strategy*). It replaces the generic pattern prior there. The failure memory and, where the model can check, the safety layer still apply. As soon as the state has its own experience, that decides.

**Negative transfer.** A strategy's relevance is its overall success rate. On a given level, every net failure there beyond the first halves it. From the second net failure on that level, the strategy is *rejected for that level*: *"Previous strategy rejected - it failed N times on this level"*. Its evidence and its use on other levels are untouched. A new group with nothing usable in the library is reported as *"New obstacle pattern - gathering experience (double spike, landing space 2-4 blocks)"*.

**Physics stays separate.** The library never writes the physics model, and strategies only ever reach the controller through the predictor's search or, without a trusted model, through the same checks as every other policy decision.

**Diagnostics.**
- Status panel: the group ahead (*Ahead: double spike, landing space 4+ blocks*), the last transfer message, and per attempt: groups met (known / similar / new), strategies executed as learned, replaced by the search, played directly, and rejected after failing here.
- Geode log: one `Transfer:` line per attempt with the same counts, the library size and the last message.
- Popup: library size (situations, strategies, levels learned on).

**Results (simulator, procedurally generated levels).** Levels are generated from 8 kinds of obstacle pattern (single, double and triple spikes, a block, blocks with spikes, stairs, spikes under a raised platform, spike–gap–spike) with random order and spacing. Training seeds are 1–6, evaluation seeds 101–106, and evaluation levels are never played during training. Each evaluation level starts from a fresh level memory and a copy of the trained knowledge.

`AIGD_TRANSFER_FULL=1`, 3 agent seeds × 6 unseen levels = 18 runs per row:

| Condition | First attempt: mean progress | First attempt completed | Attempts to complete (mean) |
|---|---|---|---|
| *No prediction (learning only)* | | | |
| no cross-level knowledge | 11.9% | 0/18 | 40.5 |
| general knowledge (A) | 61.0% | 6/18 | 35.1 (1 not completed in 300) |
| A + strategy library (B) | 62.3% | 6/18 | **2.4** |
| *With trajectory prediction* | | | |
| no cross-level knowledge | 11.9% | 0/18 | 3.0 |
| general knowledge (A) | 100% | 18/18 | 1.0 |
| A + strategy library (B) | 100% | 18/18 | 1.0 |
| strategy library only (no physics calibration) | 100% | 18/18 | 1.0 |

What this shows, and what it does not:
- **Without prediction, the library makes learning a new level about 15× faster** (2.4 vs 35.1 attempts). The first attempt is barely better than with general knowledge alone (62.3% vs 61.0%, the same 6/18 completed).
- **With a calibrated prediction there was nothing left to improve.** The planner alone completes every unseen generated level on the first attempt once the physics calibration (layer A) has transferred. The library adds nothing measurable there.
- **Without the physics calibration, the library alone was enough here:** 18/18 first-attempt completions, against 0/18 with no knowledge. (Before a recording fix described below it was 15/18.)
- **Search guidance:**
  - with the trained library, 8 of 9 strategies handed to the search were executed as learned;
  - with deliberately wrong strategies (press 95 steps early), 8 of 9 were rejected by simulation, and the level was still completed on the first attempt;
  - the one "confirmed" wrong timing was really safe there.
- **Negative transfer:**
  - on a level whose input takes effect 14 steps late (a difference the situation description cannot see), 22 failures were recorded against that level only, and 2 strategies were rejected for it and are no longer proposed there;
  - the evidence from the training levels is intact;
  - that level took 12 attempts with the library and 10 without, so the bad strategies cost a little before they were rejected.
- **Cost:** within measurement noise. Two runs of the agent loop (simulator and predictor included) took 278 → 277 µs and 214 → 220 µs per decision, without → with the library.
- **Benchmark:** unchanged, because the existing learning benchmark does not attach a library.

### Full-level perception and strategic planning (v0.5.0)

**What can be read from the game.** GD 2.2 creates every object of a level when the level loads (`PlayLayer::m_objects`), so the whole level is available before the first step. Per object, the mod uses:
- the game's collision rectangle (`GameObject::getObjectRect`). For spikes this is the real hitbox, smaller than the artwork;
- the object type (`m_objectType`) and id (`m_objectID`), which give the kind: hazard, block, one-way platform (`m_isPassable`), slope, pad, orb, speed portal (ids 200–203, 1334), gravity portal (normal / inverse / toggle), mode, size, dual and teleport portals;
- `m_isNoTouch` (ignored) and `m_isGroupDisabled` (toggled off);
- the rotation: a hitbox rotated by a non-right angle is stored as its bounding box and marked *approximate*;
- the groups (`m_groups`, `m_groupCount`), and for triggers the target group (`EffectGameObject::m_targetGroupID`). Objects in a group targeted by a move, rotate, toggle, follow, follow-player-Y, animate, area, advanced-follow or keyframe trigger are marked *dynamic*.

What is **not** read or simulated:
- when a trigger fires and where it moves things (the map refreshes dynamic objects from the game instead of predicting them);
- spawn, count, item and collision triggers;
- the effect of orbs, pads, slopes, one-way platforms, mode, size, dual and teleport portals (paths touching them are uncertain, as before);
- the exact oriented hitboxes of rotated objects.

**Level map** (`core/LevelMap`):
- static objects are indexed once: sorted by left edge, plus a grid of 120-unit columns, so a window query never scans the level;
- dynamic objects are kept apart. Near the player they are re-read every step, and all of them every 8 steps;
- every 30 steps up to 64 static objects ahead are checked against the game, and one that moved is treated as dynamic from then on;
- every change bumps a version number, which invalidates plans made with the old geometry;
- speed and gravity portals become *physics events* for the predictor. Everything else that is not simulated stays an uncertain obstacle.

Optimised build: indexing 1 000 / 10 000 / 50 000 objects takes 0.14 / 0.74 / 3.8 ms, and a window query 0.3–0.5 µs.

**Whole-level analysis** (`core/LevelAnalysis`). Once per level load, before playing, the AI works out:
- the start, the finish, and the sections between physics changes (mode, speed, gravity, size, as far as the portals tell);
- the obstacle groups, with the same relative description the strategy library uses: the free path before and after each group, the headroom above it, and whether it contains unmodelled, moving or approximate objects;
- a difficulty estimate, the tight landings and the hardest groups;
- for corridor modes and flipped gravity, only the smallest vertical gap and the number of narrow passages (groups are described for cube and robot with normal gravity only);
- which groups the strategy library already knows (exactly or similar).

Nothing is simulated in the analysis; the walking surface in front of a group is assumed from the floor and from the blocks a jump could reach. In the optimised build it takes 0.21 / 0.96 / 6.0 ms for 1 000 / 10 000 / 50 000 objects. The Geode log gets `Level map:` and `Level analysis:` lines and the five hardest groups.

**Physics transitions in the prediction.**
- *Speed portals* are simulated: the horizontal speed changes when the hitbox reaches the portal. In the simulator, the position error after a 1× → 3× portal stayed below 0.01 units.
- *Gravity portals* are simulated too (the flip, then falling the other way). The simulator matches exactly (landing on the ceiling at x 253.88 in both). In game the model of the flip has not been checked, so a path through a gravity portal is never reported as safe there: it is *uncertain* from the portal on.

**Strategic planner** (`core/StrategicPlanner`). Two cooperating systems:
- the *strategic planner* decides which upcoming obstacles have to be handled together and with which input sequence;
- the *real-time controller* (the predictor's search, the safety layer and the live plan of v0.3.0) decides every input and cancels whatever no longer fits.

How a sequence plan is made:
1. **The sequence.** From the ground in cube or robot mode, the group ahead and every following group with less than 4 blocks of free path between them (at most 18 blocks, 6 groups) form one sequence. It is described, e.g. *"single spike -> 2-4-block landing -> double spike"*. A sequence is planned once its first group is at most 160 steps away.
2. **The search** uses the same trajectory simulation, covering the whole sequence (up to 420 steps), at most 160 simulations per plan:
   - when to press first (or not at all): a coarse grid, the timing from the strategy library if one fits, and single-step refinement around the three best;
   - then, for the most promising timings (beam of 4), a second press timed from the landing, and a third from the next landing;
   - deeper levels are only searched while no fully safe sequence with fewer presses exists.
3. **Scoring**, in order:
   - safety;
   - for sequences that are safe until the landing after their last press and only hit something later (*open-ended*), the number of groups they get past;
   - clearance;
   - a cost per press;
   - the free path in front of every landing;
   - robustness, i.e. whether neighbouring timings work too.
4. **What judging only the next obstacles would have chosen** is recorded too. When the chosen first press differs and the locally best one collides later, the plan says so: *"press in 0 clears the first obstacle but collides 184 steps ahead (1 of 2 groups handled); press in 20 handles ..."*.
5. **To the controller.** The plan is handed over only if it is safe at least up to its last landing:
   - its next press is added as a candidate that is evaluated *after* the controller's own and wins only if strictly better;
   - the controller's look-ahead is extended to the part the plan covers, so the controller's own candidates are judged by what they do to the later obstacles too;
   - if no such plan exists, the controller decides alone as before. Judging every candidate by a far-away collision made the next input worse: measured, see below.
6. **Staying correct.** The plan is checked on every step against the real state (position, height, on the ground or not, mode, speed, gravity) and against the map version. It is re-made from the actual state when they differ: *"state left the strategic plan (...) - replanning the sequence from the actual state"*. When the controller's own search finds a better sequence over the same look-ahead, the plan takes over the path of the action that was actually executed, so it keeps describing reality instead of being invalidated again and again.

Safety is never overridden: the safety ladder, the "no unnecessary jump" rule, the failure memory and plan cancellation all work as before.

**Sequence keys (cross-level).** Situations now also have a *sequence* key: the group plus the first element of what follows it. The library records and retrieves strategies under it first (then exact, then similar), so it learns that the same spike needs another timing when a double spike follows closely. On another level with the same sequence somewhere else, that timing is retrieved for it. Without prediction, the first attempt there reached 100% with the library and 30% without.

**Level-understanding debug view** (*Show Level Understanding*, off by default; every layer can be switched off):
- *extracted* objects, as filled boxes: red hazards, grey blocks, cyan simulated portals;
- *uncertain* objects, as outlines: yellow not simulated, magenta moved by triggers, orange approximate;
- *inferred* geometry, as outlines: white obstacle groups, orange tight landings, blue the sequence being planned, green landing space;
- *predicted* movement, as lines and dots: the strategic plan's path, its presses, landing targets and the other evaluated first presses;
- a strip at the bottom with the whole level, its obstacle groups and physics changes, and the player;
- warnings for uncertain geometry ahead.

The overlay builder is in the core and tested: the drawn path is exactly the plan's path.

**Results (simulator).**

| Test | Result |
|---|---|
| whole-level extraction | every object indexed; objects beyond the old 480-unit window known from the start; window queries agree with a brute-force scan everywhere |
| obstacle sequence (spike, short platform, two spikes, spike pit, low ceiling, speed and gravity portal) | 6 groups in order with their spacing (e.g. 101 units of landing space after the first spike), the low ceiling over the spike (headroom 60), 2 physics changes, 3 sections |
| landing planning: 40 dense sequence levels not used while writing the planner (seeds 400–439), first attempt, no learning | controller alone: 26 completed, 84.2% mean progress; with strategic planning: **29 completed, 86.8%** |
| an early version that also used plans without a safe sequence | 9 vs. 11 completed on 16 levels (worse); found, traced, fixed (see step 5 above) |
| no unnecessary jump (a spike, then hanging spikes over the landing) | 1 press, and the sequence plan has 1 press; hanging spikes only: 0 presses |
| physics transition (1× → 3× portal before a sequence) | the first press moves from 115 to 58 steps ahead; position error after the portal < 0.01 units |
| dynamic obstacle (a block drops onto the path at step 120) | map version 1 → 2, the plan is re-made (*"the level geometry changed"*), completed; with stale geometry the AI dies at x=656 |
| replanning (landing early on a block the AI cannot see) | sequence plan and live plan invalidated from the real state, completed |
| cross-level sequence transfer | sequence-key match on another level; first attempt 100% vs. 30% without the library |
| overlay consistency | drawn path = plan path (0 mismatches); 10 of 20 presses exactly at a planned press, the others where the controller's refinement moved them (the plan then follows them) |

All 95 unit tests pass (2583 checks, 0 failures).

**Cost** (optimised build, with the simulator, prediction and agent included):
- a sequence plan takes 0.54 ms on average and 1.8 ms at most;
- the agent loop takes 36 µs per physics step with strategic planning and 11.5 µs without (the 3× is the longer look-ahead the controller then searches over);
- the whole-level analysis runs once per load.

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
- **cross-level transfer (v0.4.0)**, with separate training (seeds 1–6) and evaluation levels (101–106), several agent seeds (`AIGD_TRANSFER_FULL=1` for the full protocol):
  - `situation_descriptor_is_relative_and_transferable`: the same group elsewhere gives the same keys, while a different layout or a block does not;
  - `strategy_library_tracks_provenance_and_negative_transfer`: provenance, speed adaptation, rejection on the failing level only, and a better local timing preferred;
  - `strategy_memory_persists_and_resets_separately`: round trip, export bundle, old pattern files, newer formats refused, `.bak` recovery, level reset vs. shared reset;
  - `transfer_to_unseen_levels`: first-attempt progress, completions and attempts to complete, for no / general / general + strategies / strategies only, with and without prediction;
  - `negative_transfer_is_detected_and_contained`, `transfer_uses_relative_geometry_not_positions` (no copying), `strategies_guide_the_search_but_never_dictate` (planning, physics untouched), `transfer_cost_per_step_is_small`;
- **real time, live plans and optimisation (v0.3.0):**
  - the step clock counts steps exactly (1, 2, 3 per call, repeated calls, respawns);
  - the wrong x speed saved in the real game (1.5579) is dropped on load, and a wrong estimate learned during play is replaced by correct samples;
  - a game-like observer (1–3 steps per call) reproduces the in-game failure with the old tick counting, and shows the fix (exact x speed, drift 0.000, trusted; basics, spikes, blocks completed on attempt 1);
  - **early landing** (a block the AI cannot see):
    - the landing is detected from the real state, *"Landing detected earlier than expected"*;
    - the 2 pending inputs of the old plan are cancelled;
    - a new plan is made in the same step from the landed state;
    - 0 inputs of the old plan are executed afterwards;
    - the level is completed;
    - the cancelled plan is in the plan history, and nothing learned is lost;
  - a valid plan is followed (1364 decisions on spikes, 0 invalidations, exactly 9 jumps, 1.8 simulations per step);
  - a gravity portal invalidates the plan (*"Plan invalidated by a gravity change"*);
  - two spikes in a row: a safe jump pair is found, later than the earliest workable first jump;
  - a learned timing speeds up the search when it fits and is not used when it does not;
  - the search stays within its budget and still finds viable actions;
  - the timing resolution follows the measured decision rate (2 steps at ~1.7 steps per call, 1 when every step is observed);
  - a learned test sequence is cancelled, not played, when the player reaches its first changed cell in a different state;
  - no contradictory inputs: in every run the executed input equals the recorded decision and, when following, the plan's input;
- **decision pipeline (v0.2.2):**
  - the planner makes **no jump** over a level of flat blocks with hazards far above (0 presses);
  - a learned policy that wants to jump at every chance makes **0 presses** on that level, and **exactly the 9 needed jumps** on the spikes level (415 proposed presses dropped). The planner alone also makes exactly 9;
  - a policy with an old habit of jumping before low hanging spikes learns to stop: "remove the press" is tested after two failed timings, and the level is completed on attempt 4. Before, 12 attempts and only by luck. The learned behaviour holds with learning off and after a save/reload;
  - a game the model cannot predict (random jitter) is detected (drift 0.211 > 0.15). The prediction is then not used, and the panel and overlay say so; no override ever came from an untrusted prediction;
  - hazards observed 2 units bigger than they kill do not destroy trust (drift 0.000 while grazing them);
  - a 1-step input delay is measured and corrected (spikes completed on attempt 1), and no delay is measured as 0;
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

Unit tests: 95 tests, 2583 checks, 0 failures.

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

v0.2.2 compared with v0.2.1 (same benchmark, median / mean), default settings:
- basics 6/7 → 3/3; learning speed 0.1: 7/7 → 2/2; speed 1.0: 6/7 → 4/4;
- spikes 14/16 → 14/14; speed 0.1: 15/16 → 14/14; speed 1.0: 10/21 → 10/10;
- blocks unchanged (13/14); speed 0.1: 26/44 → 26/30;
- ship unchanged (32/31); speed 1.0: 18/19 → 17/17; **speed 0.1 worse: 139/140 → 147/164**;
- with trajectory prediction: unchanged (3, 3, 3, 5).

The *Q-learning only* ablation (no failure analysis; not something the mod can be set to) got much worse: blocks fails on 3 of 5 seeds, ship 165 → 397. This comes from the prior's press margin: pure Q-learning leaned on the prior pressing at the slightest preference. A smaller margin (0.1, 0.2) did not fix the ablation but cost the default setup, so 0.5 was kept.

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

**For v0.4.0, check first** (Geode log):
1. At start-up: `Loaded strategy library: N situations, M strategies, learned on K levels` (or `No strategy library yet` on the first run; it fills while playing).
2. One `Transfer:` line per attempt: obstacle groups met (known / similar / new), strategies simulated and executed as learned or replaced by the search, strategies played directly, passes and failures recorded, and the last message.
3. On a level the AI has never played, after it has played a few others: the status panel's *Transfer:* line should read *New level detected - ...*, and the first attempts' `Transfer:` lines should show known or similar groups.
4. The popup shows the library size. **Reset Shared** clears it (with physics and timings) but keeps every level's memory; **Export Shared / Import Shared** move it between installs.

**For v0.3.0, check first** (Geode log, per attempt):
1. The `Plans:` line: *physics steps per AI call* tells how often the game really calls the AI (the v0.2.2 session implied ~1.3–1.7). *x speed* should be 1.2983 at normal speed (0.5×: 1.0465, 2×: 1.6143); 1.5579 was the old, wrong value. It is dropped automatically when the memory is loaded.
2. The `Decisions:` line: most decisions should now be *with a trusted prediction*, with drift well below 0.15.
3. Early landing: in a section where the AI jumps early and lands before the next obstacle, the panel should show *Replanned: Landing detected earlier than expected - cancelling N pending actions; replanning from the actual state*, and the next jump should be timed from the landed position (the plan line shows the new parameters).
4. Consecutive obstacles: the plan line should show a second press (*second press N steps after landing*, stage *two jumps*) when one jump cannot clear both.
5. Performance: the `Decisions:` and `Plans:` lines show how many decisions followed a plan; the AI's own cost is a few percent of one core at most.

**For v0.2.2, check first** (from the Geode log, one `Decisions:` line per attempt):
1. Most decisions near obstacles should be *with a trusted prediction*. If not, the line says why (drift, caution or calibration). That is the first thing to fix before judging behaviour.
2. *input latency*: 0 means the game applies the button on the next step, as the model assumes. 1 or 2 means it is later, and the predictor now compensates. Its three re-simulation errors show how clear the evidence is.
3. *presses skipped as unnecessary*: on a section that can be passed without jumping, the panel should show `No jump: predicted clear | Selected: No jump`, with no jump.
4. Where a jump is needed, the panel should show `No jump: collision in N steps | Selected: Jump`.
5. Repeated deaths at one spot after jumping should lead to a *"Removed the press at X%: testing not jumping there"* attempt in the log (when not jumping can work there).

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
- **v0.3.0 in game (Dry Out):** the step clock worked. The calibrated x speed was 1.2982 (true value 1.2983), most decisions had a trusted prediction, and the AI got from 37.8% to 48.8% (v0.2.2 was stuck at 21.4%). At 48.8% it died repeatedly in an upside-down cube section: it had learned a contact with something the observer does not report as a level-wide ceiling, and then predicted a surface under hanging spikes (one-step error 18.5).
- **v0.3.1:**
  - contacts in cube and robot mode no longer create a ceiling bound (test `upside_down_cube_contact_is_not_a_ceiling`, which fails without the fix);
  - the plan tolerance is 2.5 units, since the game snaps the player onto surfaces;
  - when both inputs only collide far ahead (60+ steps), the record no longer says "NO SAFE OPTION" (later decisions can still act).

  **In game (Dry Out):**
  - the first attempt went past the old 48.8% wall to 94.2%;
  - 15,973 decisions had a trusted prediction and 150 did not; x speed 1.2984;
  - the level was **completed on attempt 508**, 8 attempts after installing v0.3.1 (it had 1 completion in its first 500 attempts), and again on attempt 512.
  - Later in the same session, several new online levels were completed within 5–41 attempts.
- **v0.4.0 in game** (Dry Out, then two main levels it had never played, Base After Base and Can't Let Go):
  - the library fills while playing, and groups from Dry Out are recognised on the new levels (on Can't Let Go, per attempt: 1–2 known, 2–4 similar, the rest new);
  - the first in-game run showed a recording error: a group that an *earlier* jump had already cleared (the player was in the air over it) was recorded as a successful "no jump". Such groups are now not recorded ("not attributable": 46 of 82 groups on Dry Out, because in real levels the player is often already in the air when approaching the next group). The fix also improved the simulator result above (15/18 to 18/18);
  - a strategy that reached the search was **never executed as learned in game**: the search always found a plan it scored higher, often by a small margin (560.0 vs 549.0). With a calibrated, trusted model the library adds nothing measurable in game, matching the simulator;
  - completion was not affected: Dry Out was completed in practice mode in 2 attempts.
- **v0.5.0 in game** (GD 2.2081, three main levels):
  - *Level map:* Dry Out 1166 of 1493 objects, Time Machine 2563 of 3735, Cycles 2269 of 4069, each indexed in 0.5–1.1 ms. None of these levels uses move or toggle triggers, so the dynamic-object path was **not exercised in game**.
  - *Analysis at load* took 0.16–0.31 ms. Dry Out: 47 obstacle groups, 2 gravity and 2 mode portals, 11 groups already known to the library and 10 similar. Time Machine: 54 groups, 10 gravity portals. Cycles: 33 groups, 22 gravity portals. The first build reported 8.0 blocks of landing space for many groups (the default when the next obstacle lay outside the describing window); landing space is now measured on the whole map.
  - *Dry Out* (500+ earlier attempts of memory) was completed on the first attempt of the session: 15 043 decisions with a trusted prediction, 81 without; 216 sequence plans, 10 of them choosing another first press than the locally best one; the controller executed the plan in 980 of 1 720 decisions where it was a candidate.
  - *New levels:* Time Machine reached 44.5% in 21 practice attempts (stuck at a ship / cube section around 43%). Cycles stayed at 3.6% for 17 attempts and then at 13.2–13.6% up to attempt 57 (sections with orbs and pads, which are not simulated). **There is no in-game comparison with strategic planning off**, so whether it helps in game is not shown.
  - *Cost:* sequence plans took 0.8–1.5 ms on average with peaks up to 4.3 ms. With the 2 ms budget added after that, the peaks were 0.96 ms. 60–70% of plans were re-made after the state left them, mostly where the learned policy (not the planner) decided differently.
  - *Debug view:* checked visually on Cycles. It shows the planned sequences (blue), the inferred groups, the whole-level strip, and the warning about objects that are not simulated.
- **v0.5.0 limits:**
  - sequence planning and obstacle groups cover cube and robot only; other modes get the corridor check and the longer map window, but no sequence plans;
  - a sequence plan has at most three presses (an open-ended plan hands over to the next decision after its last landing);
  - the trigger list that marks objects as moving may be incomplete. Unlisted movement is only noticed by the runtime check (64 objects ahead every 30 steps), and an object a trigger moves *into* view from far away is only refreshed every 8 steps;
  - the gravity-portal model is unverified in game (such paths are uncertain), and mode / size / dual / teleport portals, orbs, pads, slopes and one-way platforms are still not simulated;
  - the whole-level analysis is geometric. Its walking surface and difficulty are estimates, and only the predictor decides what is safe;
  - a sequence plan can take up to ~1.8 ms (optimised build), in the game thread, at most once every few steps.
- **Cross-level transfer (v0.4.0) is limited to cube and robot.** Situations are only described from the ground in those modes. Ship, wave and the other modes still transfer only through the physics calibration, timing statistics and pattern prior.
- **The situation description does not see everything that matters:** orbs and pads (only flagged), the floor ending, moving objects, or a different input delay. Such differences are only discovered through failures on the new level (negative-transfer handling).
- **Strategies were evaluated on generated simulator levels.** Whether the in-game first attempt on an unseen level improves can only be shown by playing new levels in game.
- **The calibration saved by earlier versions was wrong** (x speed learned 1.2× too high, and the calibration then rejected every correct sample because it was more than 5% off). On loading, a saved speed more than 10% away from GD's exact speed for that speed class is now dropped. During play, 40 consistent samples that disagree with the estimate replace it. Both are tested with the value saved in the real game (1.5579 → 1.2982).
- **Between two AI calls the button cannot change.** At ~1.7 steps per call, a plan's input change can happen a step late. The search refines only to that resolution, and the plan tolerances absorb one step, but frame-perfect timings are out of reach while the game calls the AI less often than every step.
- **Plans behind learned-policy decisions are not followed**, only validated: the learned policy's future decisions cannot be simulated.
- **Two-jump search is only for cube and robot**, and only when no single-jump plan is safe. Ship, wave and the other modes optimise one input change ahead (hold durations, delays, repeated taps) as before.
- **Learned timings are per kind of obstacle** (mode, speed, size, gravity, obstacle height), not per complete obstacle layout. They only order the search; they are never executed unsimulated.
- **Benchmark vs. v0.2.2:** identical on every line except blocks (default median 13 → 15, mean 14 → 14; learning speed 1.0: 13 → 15; learning speed 0.1: mean 30 → 27). Two early versions of the experiment state check made learning much slower (spikes at learning speed 1.0: 10 → 34; ship at 0.1: 147 → 347). They were measured, traced to a cancel-and-retry loop and to over-strict height checks in flying modes, and fixed.
- **v0.2.2 has not been tested in game.** The in-game log of v0.2.1 shows that prediction was not trusted there. v0.2.2 removes the cause found in the code (false collisions inside the accuracy measurement), but whether trust now holds in game, and what input latency the game really has, can only be read from the new `Decisions:` log lines of a real session.
- **The "no unnecessary jump" rule depends on a trusted prediction.** While the prediction is not trusted, the learned policy decides alone, and only the prior's press margin and the "remove the press" experiment work against unnecessary jumps.
- **"Remove the press" uses the killer's geometry** to decide whether walking under it was possible. Where the observer reports no killer extent, it is tried once per trouble spot anyway.
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
           Physics, Predictor, PlanManager, StepClock, Situation, Strategy,
           LevelMap, LevelAnalysis, StrategicPlanner, Overlay) – no Geode dependency
tests/     unit tests, GD-like simulator, benchmark
src/       Geode mod: Observer, AIController, SaveWorker, hooks/, ui/ (StatusPanel, AIMenuPopup, TrajectoryOverlay)
mod.json   Geode metadata and settings
```

## License

No license has been chosen for this repository yet.
