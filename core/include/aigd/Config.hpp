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
    float priorWeight = 0.5f;         // how much cross-level pattern knowledge counts in states without own experience
    // Extra reward, once per attempt, on the decision that first goes past the
    // previous best. Off by default: measured on the benchmark it slowed
    // learning (blocks: mean 14 -> 40 attempts), because the progress reward
    // already pays for new ground and the bonus inflated the values of
    // whatever happened to be done on the way there.
    float newBestReward = 0.f;

    // --- Credit assignment / failure memory ---
    int passTicks = 120;              // a decision followed by this long without dying counts as a pass
    int blameTicks = 360;             // how far before a death its cause is searched for
    int repeatFailureThreshold = 2;   // failures in a row of one decision in one state before trying the alternative
    int knownGoodPasses = 3;          // passes (and no failure since) for an action to count as known good

    // --- Exploration ---
    float focusEpsilon = 0.03f;       // random exploration inside the failure window
    bool useExperiments = true;       // failure-analysis-driven timing experiments
    int experimentAfterDeaths = 1;    // >1 lets plain Q-learning try a new death spot first
    int stuckAttempts = 20;           // attempts without a new best before escalating
    int maxExperimentsPerHotspot = 10;

    // --- Trajectory prediction ---
    bool usePrediction = true;        // plan with the physics model and block predicted collisions
    float imaginedScale = 0.5f;       // weight of "imagined" deaths learned from predictions
    // Safety layer (all in physics steps, i.e. time, so they hold at any speed):
    int safetySlackSteps = 3;         // change the input while at least this many steps remain to do so
    int dangerWindowSteps = 16;       // re-check every step while a change may be needed within this many steps
    int imminentRiskSteps = 12;       // a possible (uncertain) hit this soon counts as unsafe if the other action is safe

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
