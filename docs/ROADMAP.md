# Roadmap and implementation plan

Released work is in [changelog.md](../changelog.md). This file tracks what is being built next and how it will be checked. Nothing here is promised until it is released.

## Done
| Version | Phase |
|---|---|
| v0.1 – v0.3 | Learning core, trajectory prediction for all modes, safety layer, real step counting, live plans |
| v0.4 | Cross-level learning (strategy library) |
| v0.5 | Whole-level map, level analysis, strategic sequence planning |
| v0.6 | Orbs and dash orbs, mode-specific precision, UI |
| v0.7 | Trajectory proximity: clearance-aware decisions, danger colours, near-miss learning |

## In progress: v0.8.0 — hitbox accuracy (Phase 6)

**Goal:** collision clearance and collision prediction based on Geometry Dash's real collision geometry instead of estimates and bounding rectangles, while keeping every existing system (learning, proximity, planning) unchanged on top of it.

**Research so far**
- QOLMod (TheSillyDoggo) was studied for how it displays hitboxes. It draws the game's own data: object rectangles (`getObjectRect`), oriented boxes for rotated objects (`getOrientedBox`), a radius for round objects (`m_objectRadius` × scale), triangles for slopes, the player's rectangle, an inner box at 0.3× size, and the player's own oriented box. Its licence forbids code reuse, so none of its code is used. AI-Geode reads the same game data itself, through the Geode bindings.
- A *displayed* hitbox is not necessarily the one the game collides with. Each rule is therefore verified against real gameplay before the AI relies on it, using the optional **Hitbox Trace** diagnostic. The trace records the game's player box, inner box, rotation and oriented box, every round or rotated hazard the player passes within 8 units, and the killing object at each death.

**Plan**
1. *Verify the game's rules from the trace.*
   - Player rectangle size per mode and size.
   - Inner-hitbox scale.
   - Whether the player's oriented box rotates with the icon.
   - Circle-vs-circle or circle-vs-box for saws.
   - Oriented-box tests for rotated hazards.
   - Each candidate rule must explain every recorded death and must not flag any pass the player survived.
2. *Exact shapes in the level map and observer:* circles, oriented boxes and (if verified) slope triangles alongside the existing rectangles, refreshed for moving objects.
3. *Exact collision and clearance in the predictor and proximity:*
   - signed distance to circles and convex polygons;
   - real player hitbox sizes read from the game instead of estimates;
   - danger colours, near-miss detection and decision scoring use the exact distances.
4. *Live validation in game:* every attempt logs how many survived passes the old (rectangle) model would have called collisions, and whether each death is explained by the new model.
5. *Narrow gaps and landing areas:* the level analysis uses the exact shapes for corridor widths and safe landing zones.
6. *Tests* for spikes, saws, rotated hazards, ceilings, narrow gaps, orbs and gravity changes. A simulator A/B against v0.7.0.
7. *In-game comparison* on levels with saws and rotated spikes before release. It is published as a pre-release (`v0.8.0-beta.N`) until that comparison is done.

## Planned after v0.8
- Jump pads (yellow, pink, red, blue, spider) in the predictor, calibrated from the game like orbs.
- Slopes: movement along slopes, not only their shape.
- Objects moved by triggers: predict their movement instead of using their current position only.
- Spider, teleport and custom orbs.
- Strategic planning and cross-level transfer for ship, wave and the other flying modes.
