# Changelog

## v0.7.0 — Trajectory proximity
- Every predicted step measures how close the player's hitbox comes to deadly geometry, using the game's collision rectangles (not the artwork). Floors and blocks underfoot are not dangers.
- **Danger colours:** the drawn path blends from green (clear) through orange to red (very close or colliding), with a ringed marker at the closest point. Yellow still means uncertain.
- **Decisions prefer clearance:** candidate timings are rewarded for margin up to *Safe Clearance* and penalised below *Danger Clearance*. A near miss is still preferred over a collision.
- **Near-miss learning:** near misses are remembered per level ("safer / closer than last time"), and strategies in the cross-level library keep the margin they passed with.
- New settings: Danger Colours, Clearance in Decisions, Safe Clearance, Danger Clearance. A new `Proximity:` line in the log after each attempt.

## v0.6.8
- Mode and size portals are applied inside predictions. Such paths stay *uncertain* until the effect is verified in game.

## v0.6.7
- Prediction drift and safety margins are tracked separately for normal and mini size in every mode.
- Ship, wave and swing plans tolerate grounded-flag disagreements. Finer release timing for flying modes.

## v0.6.6 / v0.6.4
- Smoother wave steering, which prices presses and releases and holds a safe slope.
- Rotated hazards are treated as uncertain envelopes instead of definite hits.

## v0.6.3
- Mode-specific follow-up inputs, a longer look-ahead in flying modes, finer robot holds, and prediction-cache fixes.

## v0.6.2
- A GD-style AI popup with three tabs (Play / Display / Memory) and a compact HUD that stays on screen.

## v0.6.1
- Trajectory label and landing fixes, complete plan tails, and reliable refresh of moving level geometry.

## v0.6.0 — Orbs
- Orb activation and skip prediction (yellow, pink, red, blue, green, black), dash and gravity-dash orbs, calibration from the game, reusable orb strategies and orb path overlays.

## v0.5.0 — Whole-level understanding
- A map of the whole level built at load time, analysis into obstacle groups, landing zones and difficult sections, strategic planning of multi-jump sequences, and a "level understanding" debug view.

## v0.4.0 — Cross-level learning
- A strategy library that recognises obstacle situations across levels, reuses strategies that worked before, and rejects ones that fail on a new level.

## v0.3.1
- No false ceiling from cube and robot contacts. More honest records for collisions far ahead.

## v0.3.0
- Counts real physics steps (which fixed untrusted predictions in game), live plans with state-checked cancellation, two-jump timing search, and learned timings.

## v0.2.2
- A fixed decision pipeline: trusted predictions, no unnecessary jumps, and removable jumps.

## v0.2.1
- Trajectory prediction controls the executed input (the safety layer).

## v0.2.0
- Contextual failure memory and causal credit assignment.

## v0.1.0
- First version: Geode mod with game-state observer, input control, Q-learning, failure analysis, practice-mode learning, and trajectory prediction for every game mode.
