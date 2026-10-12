#pragma once

#include <aigd/Types.hpp>

#include <cstdint>

namespace aigd {

// Keys are packed into the low 53 bits so they survive a round trip through a
// JSON number (IEEE double) without loss.
inline constexpr uint64_t kKeyMask = (uint64_t(1) << 53) - 1;

int32_t xCell(float x, float cellSize);

// Level-specific state: where we are in the level plus the local physics state.
// Because GD physics are deterministic, the same inputs reproduce the same keys,
// which is what lets the per-level table remember timings precisely.
uint64_t encodeLevelKey(Observation const& obs, bool holding, float cellSize);

// Level-independent "situation" key: game mode, speed, local physics and the
// relative position of nearby hazards/solids/orbs. Used as a prior so that what
// was learned about e.g. "a spike two blocks ahead at 1x speed" transfers to
// unseen states and other levels with the same speed and mode.
uint64_t encodePatternKey(Observation const& obs, bool holding);

// Decoders used by tests and the debug overlay.
int32_t keyCell(uint64_t levelKey);
GameMode keyMode(uint64_t levelKey);

} // namespace aigd
