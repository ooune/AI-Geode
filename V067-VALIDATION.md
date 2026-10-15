# v0.6.7 trajectory precision

## Changes

- Normal and mini physics already had separate calibration parameters, but shared a drift score. Drift, trust checks, safety margins, strategic evaluation and plan tolerances now use the matching mode/size slot.
- Ship, wave and swing plans no longer cancel just because the observed grounded flag differs while position still matches. Actual position drift and mode/speed/gravity/size changes still invalidate them. Ground-triggered modes retain their landing checks.
- Continuous-control candidates refine finite hold durations at +/- one and two actual decision intervals. The existing candidate budget still applies. At one physics step per callback this searches individual 1/240-second input opportunities.
- Trusted safe predictions can supersede a learned choice whose predicted continuation is not safe. A safe learned choice is retained to avoid unnecessary steering.
- Fixed a dangling reference in the existing wave timing test fixture.

## Validation

New regressions cover independent drift for all eight normal/mini mode pairs, wave slope at five speeds in both gravity directions for both inputs and sizes, and continuous-flight plan cancellation.

An initial full run found one ship regression: changing safe input after the hazard. The corrected selection rule passes the original ship test with zero overrides after the danger and zero overlay/action mismatches.

Build: GD 2.2081, Geode 5.11.1, Windows.
The built, archived and installed packages have identical SHA256:
C4CAE39240935C1165278AB38716C36053E970D04B5ADF5C800633433151953F

Release: D:/GeodeDev/release/aqvose.ai-geode-v0.6.7.geode
Rollback: D:/GeodeDev/release/aqvose.ai-geode-v0.6.6.geode
Installed: C:/Program Files (x86)/Steam/steamapps/common/Geometry Dash/geode/mods/aqvose.ai-geode.geode

The game was restarted. Its 2026-10-15 13.26.39 startup log confirms loading v0.6.7. Live capture remained on the loading screen despite completed startup logs, so new gameplay improvement is unverified.

## Limits and repeatable game check

This does not establish 100% prediction trust or pixel-perfect play. Player collision dimensions are still estimated; slopes and some rotated/triggered geometry remain uncertain. Green indicates a simulated safe path within the current horizon, not guaranteed survival. A safe delayed jump or safe no-jump route should not force an immediate jump.

In Wave Practice 2 (15254724), run the same practice checkpoint around 45%, with prediction and alternative paths enabled. Compare normal/mini sections, hold/release changes, selected trajectory, and death locations. Review Decisions and Plans log lines for drift, input latency, prediction errors and invalidation reasons. Confirm matching flight positions no longer cause landing-only invalidations. Also test mini ship, cube, UFO, ball, robot, spider and swing transitions on known levels before claiming broad in-game accuracy.

Final full regression run: **117 tests, 2,812 checks, zero failures** (D:/GeodeDev/v067-tests-final.log).
