#pragma once

#include <algorithm>
#include <cstdint>

namespace aigd {

struct LearnerConfig {
    // --- State discretisation ---
    float cellSize = 3.f;             // x resolution of the per-level table (1 block = 30)

    // --- User facing ---
    float learningSpeed = 0.5f;       // 0..1, "how aggressively timing changes"
    bool learningEnabled = true;      // false = play greedily, never update memory

    // --- Reinforcement learning ---
    float gammaPerTick = 0.995f;      // discount per physics step (~0.30 per second)
    float lambdaPerTick = 0.99f;      // how far back (in time) a death's blame reaches
    float progressRewardPerBlock = 1.f;
    float deathPenalty = 10.f;
    float completionReward = 50.f;
    float priorWeight = 0.5f;         // how much cross-level pattern knowledge seeds new states

    // --- Exploration ---
    float focusEpsilon = 0.03f;       // random exploration inside the failure window
    bool useExperiments = true;       // failure-analysis-driven timing experiments
    int experimentAfterDeaths = 1;    // >1 lets plain Q-learning try a new death spot first
    int stuckAttempts = 20;           // attempts without a new best before escalating
    int maxExperimentsPerHotspot = 10;

    // --- Input hygiene ---
    int minToggleTicks = 2;           // never change the button state faster than this
    int maxStickTicks = 24;           // re-evaluate at least this often

    uint32_t seed = 0x5eed1234u;

    float alpha() const { return 0.25f + 0.6f * std::clamp(learningSpeed, 0.f, 1.f); }
    float patternAlpha() const { return alpha() * 0.3f; }
    // Size (in cells) of the first timing shift tried by an experiment.
    int initialShiftCells() const {
        return 1 + static_cast<int>(std::clamp(learningSpeed, 0.f, 1.f) * 4.f + 0.5f);
    }
};

} // namespace aigd
