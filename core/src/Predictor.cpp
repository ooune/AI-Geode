#include <aigd/Predictor.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace aigd {

namespace {
constexpr float kNear = 40.f;          // obstacles further than this from the player are ignored per step
constexpr float kLandTolerance = 2.f;  // how far into a block top a falling player may sink and still land
constexpr int kCoarseDelays[] = {1, 2, 4, 6, 9, 12, 16, 20, 25, 31, 38, 46, 55, 65, 76, 88, 100, 115, 130, 150, 170};

float overlap(float a0, float a1, float b0, float b1) { return std::min(a1, b1) - std::max(a0, b0); }
} // namespace

char const* certaintyName(Certainty c) {
    switch (c) {
        case Certainty::Safe: return "safe";
        case Certainty::Uncertain: return "uncertain";
        case Certainty::Collision: return "collision";
        default: return "?";
    }
}

Predictor::Predictor(PhysicsModel& model) : m_model(model) {
    for (auto* p : {&m_press, &m_release, &m_tmp, &m_resim}) p->points.reserve(512);
    m_holdBuf.reserve(kHistory);
}

SimStart Predictor::startFrom(Observation const& obs, bool holding) {
    SimStart s;
    s.x = obs.x;
    s.y = obs.y;
    s.vy = obs.vy;
    s.onGround = obs.onGround;
    s.upsideDown = obs.upsideDown;
    s.mini = obs.mini;
    s.holding = holding;
    s.speedClass = obs.speedClass();
    s.tick = obs.tick;
    return s;
}

bool Predictor::trusted(bool mini) const {
    return m_model.calibrated(mini) && m_model.driftPerTick() < 0.15f && m_caution < 3.f;
}

float Predictor::margin(int step, bool mini) const {
    float base = trusted(mini) ? 0.5f : 3.f;
    return std::min(12.f, (base + m_model.driftPerTick() * static_cast<float>(step)) * m_caution);
}

void Predictor::simulate(SimStart const& s, std::span<Obstacle const> obstacles, Plan const& plan, PredictedPath& out,
                         std::span<uint8_t const> holds) const {
    CubePhysics const& P = m_model.cube(s.mini);
    float const vx = m_model.vxPerTick(s.speedClass);
    // Simulate in gravity-normalised coordinates (up = away from gravity).
    float const sg = s.upsideDown ? -1.f : 1.f;
    bool const hasFloor = !s.upsideDown;
    float const h = P.halfSize;
    float const in = P.innerHalf;
    int const steps = holds.empty() ? m_horizon : static_cast<int>(holds.size());

    out.points.clear();
    out.plan = plan;
    out.outcome = Certainty::Safe;
    out.collisionStep = -1;
    out.obstacle = -1;
    out.uncertainFrom = -1;
    out.landed = false;
    out.clearance = 1e9f;
    out.steps = 0;

    float maxW = 0.f;
    for (auto const& o : obstacles) maxW = std::max(maxW, o.r - o.l);

    float x = s.x;
    float y = s.y * sg;
    float vy = s.vy * sg;
    bool onGround = s.onGround;
    bool pressActive = false;
    int takeoffs = 0;
    int const targetTakeoffs = plan.rejump ? 2 : 1;
    bool potential = false;
    bool definite = false;
    // margin(step) = min(12, (base + drift * step) * caution), hoisted out of the loop
    float const m0 = (trusted(s.mini) ? 0.5f : 3.f) * m_caution;
    float const dm = m_model.driftPerTick() * m_caution;
    size_t first = 0;
    size_t const nObs = obstacles.size();

    auto recordHit = [&](int step, int idx, bool isDefinite) {
        if (isDefinite) {
            definite = true;
        } else if (potential) {
            return; // keep the first potential contact
        } else {
            potential = true;
        }
        out.collisionStep = step;
        out.collisionX = x;
        out.collisionY = y * sg;
        out.obstacle = idx;
        out.hit = obstacles[static_cast<size_t>(idx)];
    };

    for (int step = 0; step < steps; ++step) {
        bool hold;
        if (!holds.empty()) {
            hold = holds[static_cast<size_t>(step)] != 0;
        } else {
            if (plan.pressAt >= 0 && step == plan.pressAt) pressActive = true;
            hold = pressActive;
        }

        float const prevBottom = y - h;
        float const prevTop = y + h;
        float const px = x;
        float const py = y;
        bool const wasAir = !onGround;

        if (onGround && hold) {
            vy = P.takeoffVy;
            onGround = false;
            ++takeoffs;
        } else if (!onGround) {
            vy -= P.gravity;
            if (vy < -P.maxFall) vy = -P.maxFall;
        }
        y += vy * P.yScale;
        x += vx;
        if (holds.empty() && pressActive && takeoffs >= targetTakeoffs) pressActive = false;

        bool supported = false;
        if (hasFloor && y <= P.floorY) {
            y = P.floorY;
            if (vy < 0.f) vy = 0.f;
            onGround = true;
            supported = true;
        }

        float const m = std::min(12.f, m0 + dm * static_cast<float>(step));
        float const sweep = 0.5f * std::max(std::fabs(x - px), std::fabs(y - py));
        // x only increases, so the first candidate obstacle only moves forward
        float const lo = x - h - kNear - maxW;
        while (first < nObs && obstacles[first].l < lo) ++first;
        bool stop = false;
        for (size_t idxu = first; idxu < nObs && !stop; ++idxu) {
            Obstacle const& o = obstacles[idxu];
            if (o.l > x + h + kNear) break;
            if (o.r < x - h - kNear) continue;
            int const idx = static_cast<int>(idxu);
            float const ol = o.l, orr = o.r;
            float const ob = sg > 0.f ? o.b : -o.t;
            float const ot = sg > 0.f ? o.t : -o.b;
            float const ox = overlap(x - h, x + h, ol, orr);
            float const oy = overlap(y - h, y + h, ob, ot);

            switch (o.kind) {
                case Obstacle::Kind::Uncertain:
                    if (ox > 0.f && oy > 0.f) {
                        out.uncertainFrom = step;
                        stop = true;
                    }
                    break;
                case Obstacle::Kind::Hazard: {
                    float sep = (ox > 0.f && oy > 0.f) ? -std::min(ox, oy) : std::max(-ox, -oy);
                    out.clearance = std::min(out.clearance, sep);
                    if (ox > m && oy > m) {
                        recordHit(step, idx, true);
                        stop = true;
                    } else if (ox > -(m + sweep) && oy > -(m + sweep)) {
                        recordHit(step, idx, false);
                    }
                    break;
                }
                case Obstacle::Kind::Solid: {
                    if (ox <= 0.f) break;
                    // Falling onto the top: land.
                    if (vy <= 0.f && prevBottom >= ot - kLandTolerance && y - h < ot) {
                        y = ot + h;
                        vy = 0.f;
                        onGround = true;
                        supported = true;
                        break;
                    }
                    float const ix = overlap(x - in, x + in, ol, orr);
                    float const iy = overlap(y - in, y + in, ob, ot);
                    // Rising into the underside: bump, die if the inner hitbox gets in.
                    if (vy > 0.f && prevTop <= ob + kLandTolerance && y + h > ob) {
                        if (ix > m && iy > m) {
                            recordHit(step, idx, true);
                            stop = true;
                        } else {
                            y = ob - h;
                            vy = 0.f;
                        }
                        break;
                    }
                    if (onGround && std::fabs((y - h) - ot) < 0.5f) supported = true;
                    if (ix > m && iy > m) {
                        recordHit(step, idx, true);
                        stop = true;
                    } else if (oy > 0.f && ix > -(m + sweep) && iy > -(m + sweep)) {
                        recordHit(step, idx, false);
                    }
                    break;
                }
            }
        }

        out.points.push_back({x, y * sg});
        out.steps = step + 1;
        if (definite || out.uncertainFrom >= 0) break;

        // Walking off an edge: nothing under us any more.
        if (onGround && !supported) {
            bool sup = hasFloor && std::fabs(y - P.floorY) < 0.5f;
            for (size_t j = first; j < nObs && !sup; ++j) {
                Obstacle const& o = obstacles[j];
                if (o.l > x + h) break;
                if (o.kind != Obstacle::Kind::Solid || o.r <= x - h) continue;
                float const ot = sg > 0.f ? o.t : -o.b;
                if (std::fabs((y - h) - ot) < 0.5f) sup = true;
            }
            if (!sup) onGround = false;
        }
        if (wasAir && onGround && !out.landed) {
            out.landed = true;
            out.landX = x;
            out.landY = y * sg;
        }
    }

    if (definite) out.outcome = Certainty::Collision;
    else if (potential || out.uncertainFrom >= 0) out.outcome = Certainty::Uncertain;
    else out.outcome = Certainty::Safe;
}

float Predictor::score(PredictedPath const& p) const {
    if (p.outcome == Certainty::Collision) return -1000.f + static_cast<float>(p.collisionStep);
    float base = p.outcome == Certainty::Safe ? 1000.f : 500.f;
    if (p.outcome == Certainty::Uncertain && p.collisionStep >= 0) base += static_cast<float>(p.collisionStep) * 0.5f;
    return base + std::min(p.clearance, 30.f);
}

void Predictor::evaluate(SimStart const& s, std::span<Obstacle const> obs, Plan const& plan, PredictedPath& tmp,
                         PredictedPath& best, bool& haveBest) {
    simulate(s, obs, plan, tmp);
    ++m_advice.simulations;
    ++m_totalSims;
    if (!haveBest || score(tmp) > score(best)) {
        std::swap(tmp, best);
        haveBest = true;
    }
}

Predictor::Advice const& Predictor::advise(Observation const& obs, bool holding) {
    if (obs.tick == m_adviceTick && holding == m_adviceHolding) return m_advice;
    // In the air with the button up nothing changes the outcome until shortly
    // before landing (a press then buffers the next jump): keep the last plan.
    if (!holding && !obs.onGround && obs.tick <= m_reuseUntilTick && m_advice.valid && !m_advice.bestPresses &&
        obs.tick > m_adviceTick) {
        m_adviceTick = obs.tick;
        m_advice.simulations = 0;
        m_advice.micros = 0.f;
        ++m_reused;
        return m_advice;
    }
    m_reuseUntilTick = INT32_MIN;
    auto const t0 = std::chrono::steady_clock::now();
    m_advice = Advice{};
    m_adviceTick = obs.tick;
    m_adviceHolding = holding;
    if (!supports(obs.mode) || obs.obstacles.empty()) return m_advice;

    SimStart const s = startFrom(obs, holding);
    float const reach = s.x + static_cast<float>(m_horizon) * m_model.vxPerTick(s.speedClass) + kNear;
    bool relevant = false;
    for (auto const& o : obs.obstacles) {
        if (o.kind == Obstacle::Kind::Uncertain) continue;
        if (o.l < reach && o.r > s.x - 20.f) {
            relevant = true;
            break;
        }
    }
    if (!relevant) return m_advice;
    m_advice.valid = true;
    m_advice.trusted = trusted(s.mini);

    // Branch 1: button down this step (tap, or hold through the landing).
    bool havePress = false;
    evaluate(s, obs.obstacles, Plan{0, false}, m_tmp, m_press, havePress);
    evaluate(s, obs.obstacles, Plan{0, true}, m_tmp, m_press, havePress);

    // Branch 2: button up this step; never press, or press k steps later.
    // Coarse timings first, then refine around the best one and around the
    // places where the outcome changes (finer resolution near the boundary).
    bool haveRelease = false;
    evaluate(s, obs.obstacles, Plan{-1, false}, m_tmp, m_release, haveRelease);
    // Pressing at step k behaves exactly like never pressing up to step k: if
    // "never" already collides by then, so does every later k (no need to simulate).
    int32_t const neverHit = m_release.outcome == Certainty::Collision ? m_release.collisionStep : INT32_MAX;
    std::array<std::pair<int, Certainty>, std::size(kCoarseDelays)> coarse{};
    size_t nCoarse = 0;
    for (int k : kCoarseDelays) {
        if (k >= m_horizon) break;
        if (k > neverHit) {
            coarse[nCoarse++] = {k, Certainty::Collision};
            continue;
        }
        evaluate(s, obs.obstacles, Plan{k, false}, m_tmp, m_release, haveRelease);
        // After evaluate() the path for k is either the new best or left in m_tmp.
        coarse[nCoarse++] = {k, m_release.plan.pressAt == k ? m_release.outcome : m_tmp.outcome};
    }
    std::array<int, 16> refine{};
    size_t nRefine = 0;
    auto addRefine = [&](int k) {
        if (k < 1 || k >= m_horizon || nRefine >= refine.size()) return;
        for (size_t i = 0; i < nCoarse; ++i) if (coarse[i].first == k) return;
        for (size_t i = 0; i < nRefine; ++i) if (refine[i] == k) return;
        refine[nRefine++] = k;
    };
    int bestK = m_release.plan.pressAt;
    if (bestK > 0) {
        addRefine(bestK - 1);
        addRefine(bestK + 1);
        addRefine(bestK - 2);
        addRefine(bestK + 2);
    }
    for (size_t i = 1; i < nCoarse; ++i) {
        if (coarse[i].second != coarse[i - 1].second) addRefine((coarse[i].first + coarse[i - 1].first) / 2);
    }
    for (size_t i = 0; i < nRefine; ++i) {
        if (refine[i] > neverHit) continue;
        evaluate(s, obs.obstacles, Plan{refine[i], false}, m_tmp, m_release, haveRelease);
    }
    if (m_release.plan.pressAt > 0) evaluate(s, obs.obstacles, Plan{m_release.plan.pressAt, true}, m_tmp, m_release, haveRelease);

    m_advice.press = m_press.outcome;
    m_advice.release = m_release.outcome;
    m_advice.pressTTC = m_press.outcome == Certainty::Collision ? m_press.collisionStep : -1;
    m_advice.releaseTTC = m_release.outcome == Certainty::Collision ? m_release.collisionStep : -1;
    // Prefer not pressing unless pressing now is clearly better: waiting keeps options open.
    m_bestPresses = score(m_press) > score(m_release) + 1.f;
    m_advice.bestPresses = m_bestPresses;
    m_advice.bestPressAt = m_bestPresses ? 0 : m_release.plan.pressAt;
    if (!s.onGround && !holding && !m_bestPresses && m_release.landed) {
        // steps until the predicted landing (points are one per step)
        int32_t landStep = 0;
        for (auto const& pt : m_release.points) {
            ++landStep;
            if (pt.x >= m_release.landX) break;
        }
        if (landStep > 24) m_reuseUntilTick = obs.tick + landStep - 20;
    }
    m_advice.micros = std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - t0).count();
    return m_advice;
}

void Predictor::noteExecuted(bool pressed) {
    if (!m_advice.valid) {
        m_expectValid = false;
        return;
    }
    PredictedPath const& p = pressed ? m_press : m_release;
    m_expectValid = true;
    m_expectOutcome = p.outcome;
    m_expectTick = m_adviceTick;
    m_expectSteps = p.steps;
    m_expectCollisionStep = p.collisionStep;
    m_expectCX = p.collisionX;
    m_expectCY = p.collisionY;
}

void Predictor::onDeath(int32_t tick, float x, float y) {
    char buf[200];
    if (!m_expectValid) {
        m_deathComparison = "no prediction was active";
        return;
    }
    int32_t dt = tick - m_expectTick;
    if (m_expectOutcome == Certainty::Safe && dt <= 3) {
        // A safe continuation was predicted a moment before dying: the model is wrong here.
        ++m_mispredictions;
        m_caution = std::min(4.f, m_caution * 1.3f);
        std::snprintf(buf, sizeof(buf), "MISPREDICTION: predicted safe %d steps before dying at (%.1f, %.1f); caution now %.2f", dt, x, y,
                      m_caution);
    } else if (m_expectCollisionStep >= 0) {
        std::snprintf(buf, sizeof(buf), "predicted %s at (%.1f, %.1f) in %d steps; died at (%.1f, %.1f) after %d steps",
                      certaintyName(m_expectOutcome), m_expectCX, m_expectCY, m_expectCollisionStep + 1, x, y, dt);
    } else {
        std::snprintf(buf, sizeof(buf), "last prediction (%s) was %d steps before dying at (%.1f, %.1f)", certaintyName(m_expectOutcome), dt,
                      x, y);
    }
    m_deathComparison = buf;
    m_expectValid = false;
}

void Predictor::observe(Observation const& obs, bool heldDuringStep) {
    Rec r{obs.x, obs.y, obs.vy, obs.onGround, obs.upsideDown, obs.mini, heldDuringStep, obs.dead, obs.mode, 0, obs.tick};
    r.speedClass = obs.speedClass();

    if (m_histCount > 0) {
        Rec const& p = m_hist[(m_histHead + kHistory - 1) % kHistory];
        if (p.tick + 1 == r.tick) {
            Observation prev;
            prev.x = p.x;
            prev.y = p.y;
            prev.vy = p.vy;
            prev.onGround = p.onGround;
            prev.upsideDown = p.upsideDown;
            prev.mini = p.mini;
            prev.dead = p.dead;
            prev.mode = p.mode;
            prev.tick = p.tick;
            prev.playerSpeed = obs.playerSpeed;
            if (p.speedClass != r.speedClass) prev.playerSpeed = 0.f; // forces a mismatch: skip vx sample
            m_model.observeStep(prev, obs, heldDuringStep);

            // One-step prediction error (diagnostics).
            if (supports(p.mode) && p.mode == obs.mode && !p.dead && !obs.dead) {
                SimStart s{p.x, p.y, p.vy, p.onGround, p.upsideDown, p.mini, heldDuringStep, r.speedClass, p.tick};
                uint8_t hold = heldDuringStep ? 1 : 0;
                simulate(s, obs.obstacles, Plan{}, m_resim, std::span<uint8_t const>(&hold, 1));
                if (!m_resim.points.empty()) {
                    m_predY = m_resim.points.back().y;
                    m_actY = obs.y;
                    m_stepErr = std::fabs(m_predY - m_actY);
                }
            }
        } else {
            m_histCount = 0; // gap (reset, respawn): start a new history
        }
    }
    m_hist[m_histHead] = r;
    m_histHead = (m_histHead + 1) % kHistory;
    m_histCount = std::min(m_histCount + 1, kHistory);

    // Every 16 steps, re-simulate the last 32 real steps with the inputs that
    // were actually used and compare the end point with reality.
    if (++m_sinceResim >= 16 && m_histCount > static_cast<size_t>(kResimSteps)) {
        m_sinceResim = 0;
        auto at = [&](size_t back) -> Rec const& { return m_hist[(m_histHead + kHistory - 1 - back) % kHistory]; };
        Rec const& s0 = at(kResimSteps);
        bool ok = supports(s0.mode);
        for (int i = 0; i <= kResimSteps && ok; ++i) {
            Rec const& q = at(static_cast<size_t>(i));
            ok = q.mode == s0.mode && q.mini == s0.mini && q.upsideDown == s0.upsideDown && !q.dead && q.speedClass == s0.speedClass;
        }
        if (ok) {
            m_holdBuf.clear();
            for (int i = kResimSteps - 1; i >= 0; --i) m_holdBuf.push_back(at(static_cast<size_t>(i)).held ? 1 : 0);
            SimStart s{s0.x, s0.y, s0.vy, s0.onGround, s0.upsideDown, s0.mini, false, s0.speedClass, s0.tick};
            simulate(s, obs.obstacles, Plan{}, m_resim, m_holdBuf);
            Rec const& end = at(0);
            float err;
            if (m_resim.outcome == Certainty::Collision || static_cast<int>(m_resim.points.size()) < kResimSteps) {
                err = 20.f; // the model thought we would die (or hit something unmodelled) but we did not
                if (m_resim.uncertainFrom >= 0) err = -1.f; // unmodelled object: not the model's fault
            } else {
                auto const& e = m_resim.points.back();
                err = std::hypot(e.x - end.x, e.y - end.y);
            }
            if (err >= 0.f) m_model.recordAccuracy(err, kResimSteps);
        }
    }

    // A predicted-safe expectation that ran its course without a death slowly
    // restores trust.
    if (m_expectValid && obs.tick > m_expectTick + m_expectSteps) {
        if (m_expectOutcome == Certainty::Safe) m_caution = std::max(1.f, m_caution * 0.98f);
        m_expectValid = false;
    }
}

} // namespace aigd
