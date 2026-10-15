# v0.8.0 (in progress): hitbox trace findings

These are first results from the *Hitbox Trace* diagnostic (setting `hitbox-trace`). The trace records the game's own collision geometry while the AI plays.

Data: GD 2.2081, Geode 5.11.1, 2026-10-15, Theory of Everything and Hexagon Force (AI playing, practice mode). It contains 700 player samples, 264 passes within 8 units of a round or rotated hazard, and 34 deaths. The first build had one format specifier too many in each row type (fixed in this branch). The rows could still be read with corrected column offsets.

## Player hitbox (what the game reports)
| Mode / size | `getObjectRect()` | `getObjectRect(0.3, 0.3)` | AI model so far |
|---|---|---|---|
| Cube | 30 × 30 | 9 × 9 | 30 / inner 9 |
| Ship | **30 × 30** | 9 × 9 | **24** (estimate) |
| Ship, mini | 18 × 18 | **9 × 9** | 18 / inner **5.4** |

- The player's oriented box (`PlayerObject::getOrientedBox`) is a square of the hitbox size, centred on the player and **rotated exactly with the icon** (998 of 998 samples). Whether the game uses it for every collision, or only against rotated objects, is not settled yet.
- The game reports the player rectangle up to about 1–3 units behind `m_position` in some samples (centre offset). This still has to be checked against the collision step.

## Saws (round hazards)
For each candidate rule: how many survived passes it wrongly calls collisions, and how many deaths it explains.

| Rule | Survived passes called collisions (of 237) | Deaths explained (of 4) |
|---|---|---|
| Bounding rectangle (v0.7.0 model) | **40** | 4 |
| Circle (radius `m_objectRadius` × max scale) vs the player's **axis-aligned box** | **0** (closest pass +0.4) | **4** (−0.1 each) |
| Circle vs circle (player radius = half width) | 0 | **0** (+6.1) |

**Conclusion:** saws collide as a circle against the player's axis-aligned box. The current rectangle model is wrong for about 1 in 6 close passes. It sees collisions where the player survives, which makes the AI dodge more than it needs to.

## Rotated hazards
- No deaths to rotated hazards were recorded yet, and the closest pass was about 5 units. More data is needed (levels with rotated spikes) before oriented-box collisions are relied on.

## Implications for v0.8.0
1. Use the player hitbox sizes the game reports: ship 30 (not 24), and a mini inner box of 9.
   - This is a likely cause of the ship deaths observed at Dry Out 76% with v0.7.0.
   - It still needs confirming that the inner box GD collides with is the 0.3-scale rectangle.
2. Model saws as circle vs axis-aligned box.
3. Collect rotated-hazard data before using oriented boxes in decisions. Until then, keep them uncertain.
4. Bug: the v0.7.0 `Proximity:` log sometimes reports a large negative clearance at 0.0% (for example −18 on Dry Out at the start of an attempt). This is a false reading at spawn and needs a fix.
