# v0.6.1 — trajectory and level-map fixes

## Changes

- Main trajectory labels use an explicit selected-path endpoint. Appending an
  alternative or orb path can no longer move the main label to the wrong route.
- Followed plans retain their final segment and actual landing height, rather
  than drawing the landing at y=0. Exhausted plans are not drawn as active paths.
- Clearing overlay geometry resets stale age, input and safety metadata.
- Text labels only rebuild their glyphs when their contents change. Collision
  points and selected trajectories still update immediately; geometry is not
  visually blended into a route the controller did not choose.
- Map refresh uses elapsed physics ticks instead of exact modulo equality.
  Skipped ticks can no longer indefinitely skip refresh intervals.
- Every 30 elapsed steps, up to 64 nearby static objects and 64 objects from a
  rotating whole-level sweep are checked. Disabled and off-screen objects are
  included in the sweep, allowing unexpected reactivation or movement to be
  discovered. Trigger-classified dynamic objects retain their frequent updates.
- Map movement tolerance is 0.01 units instead of 0.5, reducing stepping in
  slowly moving hitboxes.
- The simulation query covers 600 steps at the fastest supported speed plus
  padding, including longer post-orb paths and upcoming speed increases.
  It still queries the existing whole-level index rather than scanning every
  game object per frame.

## Validation

Full suite: **110 tests, 2674 checks, zero failures**. Raw output:
`logs/v061-validation-tests.txt`. Three new regressions cover overlay metadata and
selected endpoint, plan tail and landing height, and distant reactivation/small
movement. Existing prediction, strategy, orb and map tests also pass.

The Geode build succeeded and automatically installed the package. Final build,
installed package and release copy share SHA-256:

`85786FC7918A6299C38E42626DFA49A3A0E8F8176932EA513288977140A81ADB`

Release: `D:/GeodeDev/release/aqvose.ai-geode-v0.6.1.geode`.
Rollback: `D:/GeodeDev/release/aqvose.ai-geode-v0.6.0.geode`.
Installed: `C:/Program Files (x86)/Steam/steamapps/common/Geometry Dash/geode/mods/aqvose.ai-geode.geode`.

The first v0.6.1 build loaded successfully (19:48:30 in
`Geode 2026-10-14 19.48.27.log`). The final build additionally retains nearby
static audits. It was installed while that game session was open and **requires
restarting Geometry Dash**. Visual gameplay validation was interrupted by user
input/minimized-window responses from desktop automation. No measured improvement
in frame time or verified in-game smoothness is claimed.

## Limits and next checks

The map covers accessible loaded objects, not an omniscient simulation of trigger
execution. A full unexpected-change sweep takes multiple batches (approximately
ceil(object count / 64) * 30 physics steps); far-away changes are not immediate.
Known dynamic objects are still scanned frequently, so highly animated levels
need profiling. Dash mechanics and unsupported physics remain subject to the
v0.6.0 limitations. These fixes do not establish completion of arbitrary levels.

After restarting, test a platform landing with alternative/orb paths enabled,
then a level with moving and toggled objects. Confirm the main label stays on its
selected path, landings appear on the correct platform, and changed objects are
reflected in the map. Compare planning cost and frame-time spikes on the same
level before claiming a performance improvement.
