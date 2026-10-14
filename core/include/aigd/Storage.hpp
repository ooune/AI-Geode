#pragma once

#include <aigd/Memory.hpp>
#include <aigd/Strategy.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace aigd::storage {

// JSON documents. Format (level):
// {
//   "format": "ai-geode-level", "version": 1,
//   "key": "...", "name": "...", "levelLength": ..., stats...,
//   "history":  [[number, progress, endX, kind, ticks, completed, improved, fromCheckpoint, "change"], ...],
//   "hotspots": [[bucket, deaths, passes, lastKind, shift, experiments, escalation], ...],
//   "q":        [[key, q0, q1, n0, n1], ...],
//   "bestRun":  [[x, action], ...],
//   "lastFailure": [[x, y, vy, mode, onGround, action], ...]
// }
std::string serializeLevel(LevelMemory const& level);
bool deserializeLevel(std::string_view text, LevelMemory& out, std::string* error = nullptr);

std::string serializePatterns(PatternMemory const& patterns);
bool deserializePatterns(std::string_view text, PatternMemory& out, std::string* error = nullptr);

// Strategy library (layer B), its own file so that it can be reset, exported
// and imported separately. Format:
// {"format": "ai-geode-strategies", "version": 1, "clock": n, "levels": [...],
//  "situations": [[key, [[primitive, leadBucket, successes, failures,
//                         leadSum, leadSumSq, leadN, secondSum, secondSumSq, secondN,
//                         speedMask, lastUse, [[level, successes, failures], ...]], ...]], ...]}
std::string serializeStrategies(StrategyLibrary const& lib);
bool deserializeStrategies(std::string_view text, StrategyLibrary& out, std::string* error = nullptr);

// All cross-level knowledge in one document (export/import):
// {"format": "ai-geode-knowledge", "version": 1, "patterns": {...}, "strategies": {...}}
std::string serializeKnowledge(PatternMemory const& patterns, StrategyLibrary const& lib);
bool deserializeKnowledge(std::string_view text, PatternMemory& patterns, StrategyLibrary& lib, std::string* error = nullptr);

// Writes via a temporary file + rename so a crash never leaves a half-written
// file. The previous version is kept as "<name>.bak".
bool writeFileAtomic(std::filesystem::path const& path, std::string const& data, std::string* error = nullptr);
std::optional<std::string> readFile(std::filesystem::path const& path, std::string* error = nullptr);

// Loads a level file, falling back to the ".bak" copy if the main file is
// missing or corrupt. Returns false (and leaves `out` untouched) if neither works.
bool loadLevelFile(std::filesystem::path const& path, LevelMemory& out, std::string* error = nullptr);
bool loadPatternFile(std::filesystem::path const& path, PatternMemory& out, std::string* error = nullptr);
bool loadStrategyFile(std::filesystem::path const& path, StrategyLibrary& out, std::string* error = nullptr);

// Turns a level key into a safe file name.
std::string sanitizeFileName(std::string_view key);

} // namespace aigd::storage
