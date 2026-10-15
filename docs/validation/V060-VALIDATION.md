# v0.6.0 orb interaction validation

## Implementation

Orb objects are extracted into the existing level map. The observer reads game
object type, bounds, identity and rotation, plus dash speed and maximum duration
from `DashRingObject`. Bounds are an activation-region approximation, not proof
of the game's exact touch test. Yellow, pink, red, blue, green, black, dash and
gravity-dash types have simulation models. Spider, teleport and custom orbs are
classified but their effects are not simulated.

The predictor compares skipping with candidate activation times and hold lengths,
including a preceding jump. Simulation continues after activation to assess
hazards and landing. Dash simulation has a separate movement phase and ends on
release or its modelled duration limit. The existing strategic planner evaluates
these interactions within obstacle sequences; real-time safety and plan
invalidation remain active.

Hooks around `ringJump`, `startDashing` and `stopDashing` observe game state.
Confirmed effects calibrate the persistent physics model. Unconfirmed planned
activations cause replanning. Orb type and relative geometry enter reusable
strategy descriptions; absolute coordinates are not a transferable action script.

The optional **Show Orb Trajectories** setting adds activation and skip paths,
orb bounds, activation points and outcome labels. The selected path is taken
from the executed action, including its post-activation movement.

## Automated evidence

Recorded full suite: **107 tests, 2660 checks, 0 failures** (`D:/GeodeDev/t41.txt`).
Fresh rerun on 2026-10-14 also passed **107 tests, 2660 checks, 0 failures**;
output is in `logs/v060-validation-tests-unrestricted.txt`. The initial sandboxed
rerun had 16 failed checks in three file-storage tests. Rerunning the same binary
with permitted temporary-file access resolved them; all orb tests passed in
both runs. The sandboxed output is retained as `logs/v060-validation-tests.txt`.
Twelve orb regressions cover classification, calibrated effects, activation vs
skip, post-activation collision, useful interaction selection, dash movement and
termination, sequences, cancellation, model correction, transfer and overlay
consistency.

Simulator results (not measurements from Geometry Dash):

- Six ordinary orb models reached zero reported path error after calibration
  over 160 or more compared steps.
- Dash holds of 10, 24 and 40 steps ended at their predicted steps, with at most
  0.01 units of position error over 200 steps. A 0.05-second duration ended at
  step 12.
- A yellow-orb/dash-orb obstacle sequence completed with two activations.
- A missing activation was detected and replanned in the same decision step.
- One shifted transfer fixture reached 100% with learned strategies versus
  47% without. This is a single controlled fixture, not an unseen-level success
  rate for the real game.
- Overlay regression: 36 activation-selected decisions, 5 skip-selected
  decisions, zero inconsistencies.

The simulator uses its own effect constants, but its activation and dash rules
are still hypotheses. These tests establish implementation consistency, not
fidelity to every real orb mechanic.

## Installation and launch verification — 2026-10-14

The successful build package and installed package have identical SHA-256:

`6A65E7BC9642807E59C756BCAF3248362F3DA6B3EBD04D815814B635649DA57A`

- Build: `D:/GeodeDev/build-mod/aqvose.ai-geode.geode`
- Installed: `C:/Program Files (x86)/Steam/steamapps/common/Geometry Dash/geode/mods/aqvose.ai-geode.geode`
- Previous release retained: `D:/GeodeDev/release/aqvose.ai-geode-v0.5.0.geode`
- Verified release copy: `D:/GeodeDev/release/aqvose.ai-geode-v0.6.0.geode`
- Launch log: `Geode 2026-10-14 19.30.15.log`, 19:30:18:
  `Loading mod aqvose.ai-geode v0.6.0` and `AI Geode loaded (AI player off)`.

The game was launched after installation, so another restart is not required
for this exact package. Desktop automation returned an unchanged menu screenshot
even after the log recorded opening a level. Gameplay could not be inspected
reliably, and orb/dash behaviour and visual correctness remain **unverified in
Geometry Dash**. No successful in-game orb test is claimed.

## Repeatable in-game validation still required

1. Open a local test level containing a yellow orb over a pit and a second
   section where activating an orb would hit a ceiling. Enable the AI with F8,
   trajectory display, orb trajectories and debug data.
2. Verify that activation is chosen for the pit, skipping for the ceiling,
   and the selected path changes with the actual sent input. Compare the
   `Orbs:` / `Orb:` log records with visible activations and landing positions.
3. Repeat with pink, red, blue, green and black orbs, both gravities and sizes.
   Check observed effects before treating calibration as reliable.
4. Test horizontal and angled dash orbs, gravity dash, release at several hold
   lengths and maximum-duration termination. Include a hazard immediately after
   the dash and compare predicted versus actual end position and velocity.
5. Test held entry, release/repress, already-used orbs and a deliberately missed
   activation. Confirm cancellation and replanning from actual state.
6. Test the sequence on a separate shifted layout, then restart GD to check
   persistence. Record first-attempt outcomes with and without strategy memory.
7. Finish by turning the AI off with F8. To roll back, close GD, copy the retained
   v0.5.0 package to the installed path, then restart and verify the load log.

## Remaining model limitations

Initial orb impulses are explicitly uncertain guesses. Confidence becomes known
from observed samples; broad validation across modes and special settings is
still needed. Extracted per-object dash speed is not yet used to distinguish
calibration contexts. Dash end boost, collision overrides, stop-slide behaviour,
vertical/reverse directions, custom triggers and exact activation boundaries
are not established by the available tests. A safe result is a bounded model
prediction, not a guarantee. No separate real-game orb-planning performance
measurement has yet been collected.
