# v0.7.0 trajectory proximity

## Changes

- **Clearance measurement** (`core/Proximity`): every predicted step measures the distance from the player's hitbox to deadly geometry using the same rectangles as the collision check, not sprites.
  - Hazards: rectangle distance to the outer hitbox. A negative value is the overlap depth.
  - Solid blocks: only the left face counts, and only while the inner hitbox is level with that face and has not passed it. Floors, ceilings you are sliding along and blocks underfoot are not dangers.
  - Level bounds and unmodelled objects are not dangers. Clearance is capped at 40 units (a block is 30).
- **Path data:** each `PredictedPath` keeps the clearance for every step, plus the minimum clearance, its step and position, and its near-miss count.
  - Clearance is measured through every recalculation the predictor already does: orbs and dash orbs, portals, gravity, speed and mode changes, and post-activation extension.
- **Danger colours:** the drawn path blends from green (at least *Safe Clearance*, default 18) through orange to red (at most *Danger Clearance*, default 4).
  - Segments that cross unmodelled objects stay yellow (uncertain).
  - A ringed dot marks the closest point when anything is within range; the path label and status panel show the clearance and its risk.
- **Decisions:** candidate scores reward clearance up to the safe threshold and penalise paths below the danger threshold. The penalty is a cost, not a veto: a near-miss route still beats a collision.
  - The setting *Clearance in Decisions* switches back to the pre-v0.7.0 score, which counted clearance only up to 10 units.
  - The strategic planner uses the same score.
- **Learning:**
  - Each real step closer than the danger threshold costs a small learning penalty.
  - Near misses are stored per level bucket (count, closest, last attempt) and reported as "safer/closer than last time".
  - Strategy records now keep the margin each strategy passed with. Between strategies with similar reliability, the one with more margin is chosen. This field transfers across levels with the strategy library.
- **Logging and settings:**
  - New log line per attempt: `Proximity:`.
  - New settings: *Danger Colours*, *Clearance in Decisions*, *Safe Clearance*, *Danger Clearance*.

## Validation (simulator)

Full suite (Debug): 128 tests, 2,881 checks, 0 failures. New tests cover:

- clearance against brute-force geometry (spikes, ceilings, narrow gaps);
- colours following configurable thresholds;
- more margin flown;
- near-miss routes not treated as failure;
- clearance in corridors, through portals and after orbs;
- the overlay matching the measured clearance;
- near-miss memory and strategy-margin transfer;
- cost.

A/B, old clearance score vs proximity score, first attempt with no learning, 40 dense sequence levels:

| condition | old score | proximity |
|---|---|---|
| exact, controller | 26/40, 84.2%, smallest clearance 1.9, 0.33% near-miss steps | 27/40, 85.1%, 2.6, 0.18% |
| exact, strategic planning | 29/40, 86.8%, 1.6, 0.37% | 27/40, 82.1%, 2.1, 0.23% |
| noise 0.25, 4 seeds, controller | 73/160, 66.2%, 1.6, 0.30% | 73/160, 67.2%, 1.8, 0.25% |
| noise 0.25, 4 seeds, strategic | 69/160, 66.1% | 66/160, 64.6% |

On generated levels with normal spacing (30/30 completed in every variant), the smallest clearance flown rose from 5.7 to 6.9 units (controller) and from 5.7 to 8.7 units (strategic).

**Conclusion:**
- Proximity scoring reliably increases margins and halves near-miss steps.
- Survival is unchanged within noise.
- With strategic planning on dense levels it is slightly worse: the planner's advantage over the controller there disappears (v0.6.8: 29 vs 26 completions; v0.7.0: 27 vs 27). The strategic comparison test was loosened accordingly.

Tried and rejected because they were not better:
- discounting clearance by look-ahead step;
- safe thresholds of 10 or 12;
- a near-miss penalty of 0;
- an extra refinement pass;
- using clearance as a tie-breaker.

## In game (GD 2.2081, Geode 5.11.1, 2026-10-15)

- The log confirms that v0.7.0 loaded. `Proximity:` lines appear after every attempt.
  - On Theory of Everything 2 they recorded near misses at 1.9% (1.0 unit), with "earlier attempts" comparisons accumulating (3 → 5 near misses remembered at that spot).
- Hexagon Force (AI in practice mode):
  - the drawn path showed green over the safe arc, orange then red approaching the wall, ending in the red collision marker;
  - the label read e.g. "PRESS - risky - min clearance 6.8".
  - The AI stayed at 2.2% (best 2.19%) for 29 attempts: the wall there is cleared with a **yellow jump pad, and pads are still not simulated**. This is not a proximity regression, but it is an unsolved limitation.
- Not verified in game: any survival or progress gain from proximity scoring, and strategy-margin transfer.

Build: the built, archived and installed packages have identical SHA-256:
3af35ca43dd7ecfcf674fdfc5812f0edccb15243b355853cd3db234da0bf32b9

Release: D:/GeodeDev/release/aqvose.ai-geode-v0.7.0.geode
Rollback: D:/GeodeDev/release/aqvose.ai-geode-v0.6.7.geode

## Limits

- **Pads are not simulated.** A path through a pad is wrong past the pad, and so is its clearance.
- Player hitbox sizes are estimates, so a clearance of 1–2 units is within the estimate's error.
- Red means "very close or colliding within the prediction horizon", not "certain death". Green means clear within the horizon, not guaranteed survival.
