# v0.6.3 — mode-specific predictive control

## Changes

- Ship and wave can evaluate a second independently timed hold after release.
- UFO and swing can evaluate a separately timed follow-up tap, instead of only
  a single tap or a fixed repeating rhythm.
- Ball and spider can evaluate a second independently timed switch. The physics
  simulation still decides whether the press is effective in the predicted state.
- Robot tries additional short and intermediate hold durations (4, 8, 18 and
  45 steps alongside existing candidates), enabling finer jump-height choices.
- Flying-mode lookahead cap increased from 120 to 180 physics steps, within the
  configured horizon. Existing simulation and safety ranking judge the full path.
- Follow-up search runs only when neither ordinary branch found a safe path,
  tries at most 32 additional candidates, and respects the existing total budget.
- Flying modes no longer require an obstacle object to evaluate known bounds.
  Orbs alone can also make a prediction relevant.
- Prediction reuse now checks movement speed, mini size and geometry revision,
  preventing reuse across changes previously omitted from the cache key.
- Executed input arrays still drive the live plan and selected trajectory.
  Follow-up actions remain subject to normal state validation and cancellation.

## Tests

Full suite: **112 tests, 2705 checks, zero failures**. Raw output:
`v063-validation-tests.txt`. New regressions verify follow-up press/release edges
in six modes and recomputation after speed, size and geometry changes.
Existing all-mode calibration, 32-step accuracy, controller courses and
false-safe collision regressions pass.

Comparison: three deterministic layouts per non-cube mode, with obstacles at
100%, 80% and 60% spacing, fresh calibrated simulator physics and no learned
level policy. The baseline disables the new follow-up search and restores the
120-step flying horizon. It shares the cache fixes and robot candidates, so it
is an ablation of lookahead, not a binary comparison with v0.6.2. These are small
synthetic fixtures, not a representative test of arbitrary GD levels.

| Mode | Baseline completed | New completed | Optimized mean search time, old -> new |
|---|---:|---:|---:|
| Ship | 3/3 | 3/3 | 80.8 -> 123.4 us |
| Wave | 3/3 | 3/3 | 75.3 -> 121.7 us |
| UFO | 3/3 | 3/3 | 73.0 -> 126.2 us |
| Swing | 3/3 | 3/3 | 69.9 -> 111.1 us |
| Ball | 3/3 | 3/3 | 103.7 -> 102.3 us |
| Spider | 2/3 | 3/3 | 123.3 -> 127.5 us |
| Robot | 3/3 | 3/3 | 171.8 -> 175.9 us |

Spider mean progress improved from 78.1% to 100%. Other modes showed no completion
improvement because the baseline already completed these fixtures. Search cost
increased, particularly in flying modes. Timings are standalone optimized tests,
not game frame-time measurements (`v063-mode-release.txt`). The earlier ungated
search comparison is retained in `v063-mode-tests.txt`; the final implementation
avoids extra follow-up work when a simpler safe route already exists.

## Installation

Build, release copy and installed package SHA-256:
`FAA2031B607D64B1FA548B2CE8B465402847A70898A994900F009C97CF5118DE`.

Release: `D:/GeodeDev/release/aqvose.ai-geode-v0.6.3.geode`.
Rollback: `D:/GeodeDev/release/aqvose.ai-geode-v0.6.2.geode`.
Installed in Geometry Dash's standard `geode/mods` folder. Restarted and verified
`Loading mod aqvose.ai-geode v0.6.3` at 20:09:58 in
`Geode 2026-10-14 20.09.55.log`. No further restart is needed for this package.
Live mode testing was interrupted by concurrent user input reported by desktop
automation. In-game performance improvements remain unverified.

## Limits

This extends bounded local input-sequence planning. The separate whole-level
strategic planner and reusable situation strategy library remain focused on
cube/robot. Other modes still benefit from their existing physics calibration,
learning tables and safety checks. This release does not add a full non-cube
strategic library or guarantee real-game physics accuracy. Mini, reversed gravity,
dual control, portals and unusual orb/dash behaviours need broader in-game tests.

Next gameplay checks: ship/wave alternating corridors, UFO uneven flap spacing,
swing gravity reversals, ball/spider alternating surfaces and robot low ceilings.
Compare matched attempts and search timings against the rollback package before
claiming real-game improvements.
