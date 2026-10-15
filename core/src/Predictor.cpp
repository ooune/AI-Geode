#include <aigd/Predictor.hpp>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>

namespace aigd {

namespace {
constexpr float kNear = 40.f;            // obstacles further than this from the player are ignored per step
constexpr float kLandTolerance = 2.f;    // how far into a surface a player may sink and still be put on it
constexpr float kEstimatedBoundZone = 10.f; // touching an unconfirmed floor/ceiling makes a path uncertain
constexpr int kCoarseDelays[] = {1, 2, 4, 6, 9, 12, 16, 20, 25, 31, 38, 46, 55, 65, 76, 88, 100, 115, 130, 150, 170};

float overlap(float a0, float a1, float b0, float b1) { return std::min(a1, b1) - std::max(a0, b0); }

bool continuousControl(GameMode m) { return m == GameMode::Ship || m == GameMode::Wave; }
bool tapControl(GameMode m) { return m == GameMode::Ufo || m == GameMode::Swing; }
// Modes where the button only acts while touching the ground, so in the air
// nothing changes until shortly before landing.
bool groundOnly(GameMode m) { return m == GameMode::Cube || m == GameMode::Ball || m == GameMode::Spider; }
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
    for (auto* p : {&m_press, &m_release, &m_tmp, &m_resim}) {
        p->points.reserve(512);
        p->holds.reserve(512);
    }
    m_holdBuf.reserve(kHistory);
}

SimStart Predictor::startFrom(Observation const& obs, bool holding) const {
    SimStart s;
    s.x = obs.x;
    s.y = obs.y;
    s.vy = obs.vy;
    s.mode = obs.mode;
    s.onGround = obs.onGround;
    s.upsideDown = obs.upsideDown;
    s.mini = obs.mini;
    s.holding = holding;
    s.speedClass = obs.speedClass();
    s.tick = obs.tick;
    s.bounds = m_bounds;
    s.events = obs.events;
    s.orbs = obs.orbs;
    if (obs.dashing && m_dashActive) {
        s.dashing = true;
        s.dashType = m_dashType;
        s.dashRotation = m_dashRotation;
        s.dashLeft = std::max(0, m_dashMaxSteps - (obs.tick - m_dashStartTick));
    }
    if (obs.mode == GameMode::Robot && m_robotAlive)
        s.robotBoostLeft = std::max(0, m_model.params(obs.mode, obs.mini).boostSteps - m_robotSince);
    // With input latency L, the next L steps still run with buttons that were
    // sent before: the last L recorded states (oldest first), and the state in
    // effect before them is the one L+1 records back.
    // (in physics steps: the steps T+1 .. T+L act with what was sent for
    // steps T+1-L .. T, all already known)
    if (m_latency > 0 && m_histCount > 0) {
        s.lag = m_latency;
        for (int j = 0; j < m_latency; ++j) s.pending[j] = sentAt(obs.tick + 1 + j - m_latency) ? 1 : 0;
        s.holding = sentAt(obs.tick - m_latency);
        (void)holding;
    }
    return s;
}

char const* searchStageName(SearchStage s) {
    switch (s) {
        case SearchStage::Coarse: return "coarse";
        case SearchStage::Refined: return "refined";
        case SearchStage::TwoJump: return "two jumps";
        default: return "-";
    }
}

bool Predictor::sentAt(int32_t step, int currentFrom, bool current) const {
    if (step >= currentFrom) return current;
    // records are newest first; record j covers the steps (tick_{j+1}, tick_j]
    for (size_t j = 0; j < m_histCount; ++j) {
        Rec const& r = m_hist[(m_histHead + kHistory - 1 - j) % kHistory];
        if (step > r.tick) return r.held; // after the newest covered record (only for j == 0)
        Rec const* older = j + 1 < m_histCount ? &m_hist[(m_histHead + kHistory - 2 - j) % kHistory] : nullptr;
        if (!older || step > older->tick) return r.held;
    }
    return false;
}

std::string Predictor::trustReason(GameMode mode, bool mini) const {
    char buf[160];
    if (!m_model.calibrated(mode, mini)) return "calibrating: not enough observed jumps/flights in this mode yet";
    float d = m_model.driftPerTick(mode, mini);
    if (d >= 0.15f) {
        std::snprintf(buf, sizeof(buf), "not trusted: the model drifts %.3f units/step from the real game (limit 0.15)", d);
        return buf;
    }
    if (m_caution >= 3.f) {
        std::snprintf(buf, sizeof(buf), "not trusted: %u recent deaths on paths predicted safe (caution %.2f)", m_mispredictions, m_caution);
        return buf;
    }
    std::snprintf(buf, sizeof(buf), "trusted: drift %.3f units/step, input latency %d step(s)", d, m_latency);
    return buf;
}

bool Predictor::trusted(GameMode mode, bool mini) const {
    return m_model.calibrated(mode, mini) && m_model.driftPerTick(mode, mini) < 0.15f && m_caution < 3.f;
}

float Predictor::margin(int step, GameMode mode, bool mini) const {
    float base = trusted(mode, mini) ? 0.5f : 3.f;
    return std::min(12.f, (base + m_model.driftPerTick(mode, mini) * static_cast<float>(step)) * m_caution);
}

void Predictor::simulate(SimStart const& s, std::span<Obstacle const> obstacles, Plan const& plan, PredictedPath& out,
                         std::span<uint8_t const> holds) const {
    GameMode mode = s.mode;
    bool mini = s.mini;
    ModePhysics P = m_model.params(mode, mini);
    float vx = m_model.vxPerTick(s.speedClass);
    float h = P.halfSize;
    float in = P.innerHalf;
    int const steps = holds.empty() ? (s.horizon > 0 ? s.horizon : m_horizon) : static_cast<int>(holds.size());
    uint32_t eventsDone = 0; // bit per event (the first 32)
    size_t const nEvents = std::min<size_t>(s.events.size(), 32);
    // orbs: used / overlapped (bit per orb, the first 64)
    size_t const nOrbs = std::min<size_t>(s.orbs.size(), 64);
    uint64_t orbUsed = 0, orbOver = 0;
    for (size_t k = 0; k < nOrbs; ++k) {
        if (s.orbs[k].used) orbUsed |= uint64_t(1) << k;
        // already inside at the start: not an entry
        Orb const& ob = s.orbs[k];
        if (ob.l <= s.x + P.halfSize && ob.r >= s.x - P.halfSize && ob.b <= s.y + P.halfSize && ob.t >= s.y - P.halfSize) orbOver |= uint64_t(1) << k;
    }
    int const heldRule = m_model.heldEntryActivates();
    bool dashing = s.dashing;
    OrbPhysics dashP = dashing ? m_model.orb(s.dashType, s.mode, s.mini) : OrbPhysics{};
    float dashDy = dashing ? vx * dashP.dashDx * std::tan(-s.dashRotation * 3.14159265f / 180.f) * dashP.dashSlope : 0.f;
    int32_t dashLeft = s.dashLeft;
    int32_t orbPressUntil = -1;
    out.orbActivations = 0;
    out.dashEndStep = -1;
    Bounds const& B = s.bounds;
    float floorC = B.floor + h;  // player centre when resting on the floor
    float ceilC = B.ceil - h;    // ... against the ceiling

    out.points.clear();
    out.holds.clear();
    out.ground.clear();
    out.plan = plan;
    out.outcome = Certainty::Safe;
    out.collisionStep = -1;
    out.obstacle = -1;
    out.uncertainFrom = -1;
    out.landed = false;
    out.clearance = 1e9f;
    out.steps = 0;
    out.landings = 0;
    out.eventStep = -1;
    out.unverifiedPhysics = false;
    out.proximity.clear();
    out.minProximity = 1e9f;
    out.minProxStep = -1;
    out.nearMisses = 0;
    out.scoredProximity = 1e9f;

    float maxW = 0.f;
    for (auto const& o : obstacles) maxW = std::max(maxW, o.r - o.l);

    float x = s.x;
    float y = s.y;
    float vy = s.vy;
    float g = s.upsideDown ? -1.f : 1.f; // gravity direction: +1 pulls down
    bool onGround = s.onGround;
    bool prevHold = s.holding;

    bool pressActive = false;
    int32_t pressStart = 0;
    int32_t nextPress = plan.pressAt;
    int effects = 0;
    int const target = plan.rejump ? 2 : 1;
    int32_t sinceTakeoff = 1 << 20;
    bool boostAlive = false; // robot: letting go ends the boost for this jump
    if (s.robotBoostLeft > 0) {
        boostAlive = true;
        sinceTakeoff = P.boostSteps - s.robotBoostLeft;
    }
    // edge-triggered modes: a plain press is a short tap
    bool tapRelease = P.press == PressAction::ImpulseOnEdge || P.press == PressAction::FlipOnEdge ||
                            P.press == PressAction::TeleportOnGround;

    bool potential = false;
    bool definite = false;
    float m0 = (trusted(s.mode, s.mini) ? 0.5f : 3.f) * m_caution;
    float dm = m_model.driftPerTick(s.mode, s.mini) * m_caution;
    size_t first = 0;
    size_t const nObs = obstacles.size();

    auto recordHit = [&](int step, int idx, bool isDefinite) {
        if (isDefinite) definite = true;
        else if (potential) return;
        else potential = true;
        out.collisionStep = step;
        out.collisionX = x;
        out.collisionY = y;
        out.obstacle = idx;
        out.hit = obstacles[static_cast<size_t>(idx)];
    };
    auto boundContact = [&](int step, bool estimated) {
        if (estimated && out.uncertainFrom < 0) {
            potential = true;
            out.uncertainFrom = step;
        }
    };

    int const lag = holds.empty() ? std::clamp(s.lag, 0, 4) : 0;
    out.presses = 0;
    out.landStep = -1;
    int32_t secondPress = -1;   // plan step of the second press (set at the first landing)
    int32_t thirdPress = -1;    // ... of the third (set at the landing after the second jump)
    int32_t activeDuration = plan.holdFor;
    int jumps = 0;              // take-offs so far
    bool jumped = false;
    // After an orb activation the path is followed until it lands (or 200
    // steps), whatever the horizon: an orb is only useful if what comes
    // after it is viable, and a horizon ending in mid-air would hide that.
    int stepLimit = steps;
    for (int step = 0; step < stepLimit; ++step) {
        bool hold;
        int const ps = step - lag; // plan step
        if (!holds.empty()) {
            hold = holds[static_cast<size_t>(step)] != 0;
        } else if (ps < 0) {
            hold = s.pending[step] != 0;
        } else {
            if (secondPress >= 0 && ps == secondPress) {
                pressActive = true;
                pressStart = ps;
                effects = 0;
                secondPress = -2;
            }
            if (thirdPress >= 0 && ps == thirdPress) {
                pressActive = true;
                pressStart = ps;
                effects = 0;
                thirdPress = -2;
            }
            if (nextPress >= 0 && ps == nextPress) {
                pressActive = true;
                pressStart = ps;
                effects = 0;
                nextPress = plan.repeatEvery > 0 ? ps + plan.repeatEvery : -1;
            }
            if (plan.followAt >= 0 && ps == plan.followAt) {
                pressActive = true;
                pressStart = ps;
                effects = 0;
                activeDuration = plan.followHold;
            }
            if (plan.orbPressAt >= 0 && ps == plan.orbPressAt) orbPressUntil = ps + std::max(2, plan.orbHoldFor);
            if (pressActive) {
                int32_t heldFor = ps - pressStart;
                if (activeDuration > 0 && heldFor >= activeDuration) pressActive = false;
                else if (activeDuration == 0 && tapRelease && heldFor >= 2) pressActive = false;
                else if (activeDuration == 0 && !tapRelease && effects >= target && heldFor >= 2) pressActive = false;
            }
            hold = pressActive || ps < orbPressUntil;
            // A scheduled orb tap needs an edge even if the approach held jump.
            if (plan.orbPressAt > 0 && ps == plan.orbPressAt - 1) hold = false;
        }
        bool const edge = hold && !prevHold;
        prevHold = hold;
        out.holds.push_back(hold ? 1 : 0);
        if (edge && ps >= 0) ++out.presses;

        float const prevBottom = y - h;
        float const prevTop = y + h;
        float const px = x;
        float const py = y;
        bool const wasAir = !onGround;
        float vyN = vy * g;
        bool effect = false;
        bool teleported = false;
        bool stop = false;

        // ---- orbs and dashes ----
        bool orbFired = false;
        if (dashing && (!hold || dashLeft <= 0)) {
            // the dash ends (released, or its time is up): normal physics again
            dashing = false;
            vyN = dashP.dashEndVy;
            out.dashEndStep = step;
        }
        if (!dashing) {
            for (size_t k = 0; k < nOrbs; ++k) {
                Orb const& ob = s.orbs[k];
                uint64_t const bit = uint64_t(1) << k;
                if (ob.l > x + h) break;
                bool const over = ob.r >= x - h && ob.t >= y - h && ob.b <= y + h;
                bool const entering = over && !(orbOver & bit);
                if (over) orbOver |= bit;
                else orbOver &= ~bit;
                if (!over || (orbUsed & bit)) continue;
                bool trigger = edge && ps >= 0;
                if (!trigger && hold && entering) {
                    // held from before reaching the orb: whether that activates
                    // it is learned from the game; until then it is uncertain
                    if (heldRule == 1) trigger = true;
                    else if (heldRule < 0 && out.uncertainFrom < 0) out.uncertainFrom = step;
                }
                if (!trigger) continue;
                orbUsed |= bit;
                if (!orbSimulated(ob.type)) {
                    if (out.uncertainFrom < 0) out.uncertainFrom = step;
                    stop = true;
                    break;
                }
                OrbPhysics const op = m_model.orb(ob.type, mode, mini);
                if (!op.known) out.unverifiedPhysics = true;
                if (out.eventStep < 0 && !op.known) out.eventStep = step;
                if (out.orbActivations < 2) {
                    int const i = out.orbActivations;
                    out.orbStep[i] = step;
                    out.orbX[i] = x;
                    out.orbY[i] = y;
                    out.orbType[i] = ob.type;
                    out.orbId[i] = ob.id;
                }
                ++out.orbActivations;
                orbFired = true;
                if (holds.empty()) stepLimit = std::max(stepLimit, std::min(step + 200, 600));
                effect = true;
                onGround = false;
                if (op.flips) {
                    g = -g;
                    vyN = -vyN;
                }
                if (op.dash) {
                    dashing = true;
                    dashP = op;
                    dashDy = vx * op.dashDx * std::tan(-ob.rotation * 3.14159265f / 180.f) * op.dashSlope;
                    dashLeft = ob.maxDuration > 0.f ? static_cast<int32_t>(std::lround(ob.maxDuration * kTicksPerSecond)) : (1 << 20);
                } else {
                    vyN = op.vy;
                }
                break;
            }
        }
        if (stop) {
            out.steps = step;
            break;
        }

        if (!orbFired && !dashing) switch (P.press) {
            case PressAction::JumpOnGround:
                if (onGround && hold) {
                    vyN = P.impulse;
                    onGround = false;
                    effect = true;
                    sinceTakeoff = 0;
                    boostAlive = true;
                } else if (!onGround) {
                    if (!hold) boostAlive = false;
                    bool boost = mode == GameMode::Robot && hold && boostAlive && sinceTakeoff < P.boostSteps;
                    vyN += boost ? P.accHold : P.accRelease;
                }
                break;
            case PressAction::ImpulseOnEdge:
                if (edge) {
                    vyN = P.impulse;
                    onGround = false;
                    effect = true;
                } else if (!onGround) {
                    vyN += P.accRelease;
                }
                break;
            case PressAction::FlipOnGround:
            case PressAction::FlipOnEdge: {
                bool trigger = P.press == PressAction::FlipOnEdge ? edge : (onGround && hold);
                if (trigger) {
                    g = -g;
                    vyN = P.flipVy;
                    onGround = false;
                    effect = true;
                } else if (!onGround) {
                    vyN += P.accRelease;
                }
                break;
            }
            case PressAction::TeleportOnGround:
                if (onGround && edge) {
                    // the nearest surface on the other side (blocks or the playing-area bound)
                    bool found = false;
                    bool estimated = false;
                    float dest = 0.f;
                    for (size_t j = 0; j < nObs; ++j) {
                        Obstacle const& o = obstacles[j];
                        if (o.kind != Obstacle::Kind::Solid || o.l >= x + h || o.r <= x - h) continue;
                        if (g > 0.f && o.b >= y + h - 1.f) {
                            float c = o.b - h;
                            if (!found || c < dest) dest = c, found = true;
                        } else if (g < 0.f && o.t <= y - h + 1.f) {
                            float c = o.t + h;
                            if (!found || c > dest) dest = c, found = true;
                        }
                    }
                    if (g > 0.f && B.hasCeil && (!found || ceilC < dest)) dest = ceilC, found = true, estimated = B.ceilEstimated;
                    if (g < 0.f && B.hasFloor && (!found || floorC > dest)) dest = floorC, found = true, estimated = B.floorEstimated;
                    if (!found) {
                        out.uncertainFrom = step;
                        stop = true;
                        break;
                    }
                    y = dest;
                    g = -g;
                    vyN = 0.f;
                    onGround = true;
                    effect = true;
                    teleported = true;
                    boundContact(step, estimated);
                } else if (!onGround) {
                    vyN += P.accRelease;
                }
                break;
            case PressAction::None:
                vyN += hold ? P.accHold : P.accRelease;
                if (vyN > 0.f) onGround = false;
                break;
            case PressAction::SetVelocity:
                vyN = (hold ? 1.f : -1.f) * P.waveVy * vx / P.yScale;
                if (vyN > 0.f) onGround = false;
                break;
        }
        if (stop) {
            out.steps = step;
            break;
        }
        if (effect) ++effects;
        if (!effect) ++sinceTakeoff;
        if (P.press != PressAction::SetVelocity && !dashing) vyN = std::clamp(vyN, -P.maxDown, P.maxUp);
        vy = vyN * g;
        if (dashing) {
            // a straight line in the orb direction, gravity off
            vy = dashDy / P.yScale;
            --dashLeft;
            onGround = false;
        }
        if (!teleported) y += vy * P.yScale;
        x += dashing ? vx * dashP.dashDx : vx;

        // Portals reached this step: speed and gravity changes are simulated
        // (an unverified model makes the rest of the path uncertain).
        for (size_t e = 0; e < nEvents; ++e) {
            PhysicsEvent const& ev = s.events[e];
            if (eventsDone & (1u << e)) continue;
            if (ev.l > x + h) break;
            if (ev.r < x - h || ev.t < y - h || ev.b > y + h) continue;
            eventsDone |= 1u << e;
            if (out.eventStep < 0) out.eventStep = step;
            if (!ev.verified) out.unverifiedPhysics = true;
            if (ev.kind == PhysicsEvent::Kind::Speed) {
                vx = m_model.vxPerTick(std::clamp<int>(ev.value, 0, 4));
            } else if (ev.kind == PhysicsEvent::Kind::Mode || ev.kind == PhysicsEvent::Kind::Size) {
                // Preview the destination physics, but do not certify portal
                // entry velocity/corridor changes that have not been measured.
                out.unverifiedPhysics = true;
                if (ev.kind == PhysicsEvent::Kind::Mode) {
                    if (ev.value >= static_cast<uint8_t>(GameMode::Count)) {
                        out.uncertainFrom = step;
                        stop = true;
                        break;
                    }
                    mode = static_cast<GameMode>(ev.value);
                } else mini = ev.value != 0;
                P = m_model.params(mode, mini);
                h = P.halfSize;
                in = P.innerHalf;
                floorC = B.floor + h;
                ceilC = B.ceil - h;
                tapRelease = P.press == PressAction::ImpulseOnEdge || P.press == PressAction::FlipOnEdge ||
                             P.press == PressAction::TeleportOnGround;
                m0 = (trusted(mode, mini) ? 0.5f : 3.f) * m_caution;
                dm = m_model.driftPerTick(mode, mini) * m_caution;
                boostAlive = false;
                onGround = false;
            } else {
                float const ng = ev.kind == PhysicsEvent::Kind::GravityNormal    ? 1.f
                                 : ev.kind == PhysicsEvent::Kind::GravityFlipped ? -1.f
                                                                                 : -g;
                if (ng != g) {
                    g = ng;
                    onGround = false;
                }
            }
        }

        bool supported = false;
        // playing-area bounds (GD's ground / ceiling)
        if (B.hasFloor && y <= floorC) {
            y = floorC;
            if (vy < 0.f) vy = 0.f;
            if (g > 0.f) {
                onGround = true;
                supported = true;
            }
            boundContact(step, B.floorEstimated);
        }
        if (B.hasCeil && y >= ceilC) {
            y = ceilC;
            if (vy > 0.f) vy = 0.f;
            if (g < 0.f) {
                onGround = true;
                supported = true;
            }
            boundContact(step, B.ceilEstimated);
        }

        float const m = std::min(12.f, m0 + dm * static_cast<float>(step));
        float const sweep = 0.5f * std::max(std::fabs(x - px), std::fabs(y - py));
        float const lo = x - h - kNear - maxW;
        while (first < nObs && obstacles[first].l < lo) ++first;
        for (size_t idxu = first; idxu < nObs && !stop; ++idxu) {
            Obstacle const& o = obstacles[idxu];
            if (o.l > x + h + kNear) break;
            if (o.r < x - h - kNear) continue;
            int const idx = static_cast<int>(idxu);
            float const ox = overlap(x - h, x + h, o.l, o.r);
            float const oy = overlap(y - h, y + h, o.b, o.t);

            switch (o.kind) {
                case Obstacle::Kind::Uncertain:
                    if (ox > 0.f && oy > 0.f) {
                        out.uncertainFrom = step;
                        stop = true;
                    }
                    break;
                case Obstacle::Kind::Hazard: {
                    if (s.measureOnly) break;
                    float sep = (ox > 0.f && oy > 0.f) ? -std::min(ox, oy) : std::max(-ox, -oy);
                    out.clearance = std::min(out.clearance, sep);
                    if (ox > m && oy > m) {
                        // A rotated object's axis-aligned envelope includes
                        // empty corners. Never claim a definite hit from it.
                        if (o.approximate) {
                            recordHit(step, idx, false);
                            out.uncertainFrom = step;
                            stop = true;
                        } else {
                            recordHit(step, idx, true);
                            stop = true;
                        }
                    } else if (ox > -(m + sweep) && oy > -(m + sweep)) {
                        recordHit(step, idx, false);
                    }
                    break;
                }
                case Obstacle::Kind::Solid: {
                    if (ox <= 0.f) break;
                    float const ix = overlap(x - in, x + in, o.l, o.r);
                    float const iy = overlap(y - in, y + in, o.b, o.t);
                    // How close the inner hitbox came to the side of the block (a
                    // side-on graze is risky). Standing on or sliding along a
                    // block is not a near miss.
                    if (iy > 0.f) out.clearance = std::min(out.clearance, -ix);
                    // Reaching a top or bottom surface: stand on it / slide along it,
                    // unless the small inner hitbox is already inside the block.
                    bool fromAbove = vy <= 0.f && prevBottom >= o.t - kLandTolerance && y - h < o.t;
                    bool fromBelow = vy >= 0.f && prevTop <= o.b + kLandTolerance && y + h > o.b;
                    if (fromAbove || fromBelow) {
                        if (!s.measureOnly && ix > m && iy > m) {
                            recordHit(step, idx, true);
                            stop = true;
                            break;
                        }
                        y = fromAbove ? o.t + h : o.b - h;
                        vy = 0.f;
                        if ((fromAbove && g > 0.f) || (fromBelow && g < 0.f)) {
                            onGround = true;
                            supported = true;
                        }
                        break;
                    }
                    if (onGround && ((g > 0.f && std::fabs((y - h) - o.t) < 0.5f) || (g < 0.f && std::fabs((y + h) - o.b) < 0.5f)))
                        supported = true;
                    if (s.measureOnly) break;
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

        if (!s.measureOnly) {
            // proximity: how close this step's hitbox is to dying
            float const cap = std::min(m_prox.range, kNear);
            ClearanceResult const cr = hitboxClearance(x, y, h, in, obstacles.subspan(first), cap);
            float const c = definite ? std::min(cr.clearance, 0.f) : cr.clearance;
            out.proximity.push_back(c);
            if (c < out.minProximity) {
                out.minProximity = c;
                out.minProxStep = step;
                out.minProxX = x;
                out.minProxY = y;
            }
            if (c < m_prox.danger && c > 0.f) ++out.nearMisses;
            out.scoredProximity = std::min(out.scoredProximity, c + m_prox.discountPerStep * static_cast<float>(step));
        }
        out.points.push_back({x, y});
        out.ground.push_back(onGround ? 1 : 0);
        out.steps = step + 1;
        if (definite || stop) break;

        // Walking off an edge: nothing under us (in the gravity direction) any more.
        if (onGround && !supported) {
            bool sup = (g > 0.f && B.hasFloor && std::fabs(y - floorC) < 0.5f) || (g < 0.f && B.hasCeil && std::fabs(y - ceilC) < 0.5f);
            for (size_t j = first; j < nObs && !sup; ++j) {
                Obstacle const& o = obstacles[j];
                if (o.l > x + h) break;
                if (o.kind != Obstacle::Kind::Solid || o.r <= x - h) continue;
                if (g > 0.f ? std::fabs((y - h) - o.t) < 0.5f : std::fabs((y + h) - o.b) < 0.5f) sup = true;
            }
            if (!sup) onGround = false;
        }
        if (effect) {
            jumped = true;
            ++jumps;
        }
        if (wasAir && onGround && !out.landed) {
            out.landed = true;
            out.landX = x;
            out.landY = y;
            out.landStep = step;
        }
        // landed after an orb: the extension only needs to see a little more
        if (wasAir && onGround && out.orbActivations > 0 && stepLimit > steps) stepLimit = std::max(steps, std::min(stepLimit, step + 30));
        if (wasAir && onGround && out.landings < PredictedPath::kMaxLandings) {
            out.landingX[out.landings] = x;
            out.landingStep[out.landings] = step;
            ++out.landings;
        }
        if (wasAir && onGround && jumped && plan.secondAt >= 0 && secondPress == -1 && holds.empty() && ps >= 0)
            secondPress = ps + 1 + plan.secondAt;
        if (wasAir && onGround && jumps >= 2 && plan.thirdAt >= 0 && secondPress == -2 && thirdPress == -1 && holds.empty() && ps >= 0)
            thirdPress = ps + 1 + plan.thirdAt;
        out.ground.back() = onGround ? 1 : 0; // after walking off an edge
    }

    if (definite) out.outcome = Certainty::Collision;
    else if (potential || out.uncertainFrom >= 0 || out.unverifiedPhysics) out.outcome = Certainty::Uncertain;
    else out.outcome = Certainty::Safe;
    if (out.unverifiedPhysics && out.outcome == Certainty::Uncertain && out.uncertainFrom < 0) out.uncertainFrom = out.eventStep;
}

float Predictor::proximityScore(PredictedPath const& p) const {
    if (p.proximity.empty()) return std::min(p.clearance, 10.f);
    if (m_prox.legacyScore) return std::min(p.clearance, 10.f);
    float const q = p.scoredProximity;
    float const c = std::min(q, m_prox.safe);
    float sc = m_prox.scoreWeight * std::max(c, 0.f) / std::max(0.01f, m_prox.safe);
    if (q < m_prox.danger) sc -= m_prox.nearMissPenalty * (m_prox.danger - std::max(q, 0.f)) / std::max(0.01f, m_prox.danger);
    return sc;
}

float Predictor::score(PredictedPath const& p) const {
    // Safety first; then a little extra clearance (it stops counting at 10
    // units, the margins are far smaller); then fewer inputs. Every press
    // costs, so among equally safe plans "do nothing" wins: a jump is chosen
    // because it is needed or clearly safer, never just because it is possible.
    constexpr float kClearanceCap = 10.f;
    constexpr float kInputCost = 4.f;
    if (p.outcome == Certainty::Collision) return -1000.f + static_cast<float>(p.collisionStep);
    float base = p.outcome == Certainty::Safe ? 1000.f : 500.f;
    if (p.outcome == Certainty::Uncertain) {
        int32_t when = p.collisionStep >= 0 ? p.collisionStep : p.uncertainFrom;
        if (when >= 0) base += static_cast<float>(when) * 0.5f;
    }
    float inputCost = kInputCost * static_cast<float>(p.presses);
    if (m_computedMode == GameMode::Wave) {
        // Presses alone do not price a release, yet every release reverses
        // the wave's slope. Penalise both changes to avoid a needless zigzag.
        bool held = m_adviceHolding;
        int changes = 0;
        for (auto state : p.holds) {
            if (static_cast<bool>(state) != held) ++changes;
            held = state != 0;
        }
        inputCost += 2.f * static_cast<float>(changes);
    }
    (void)kClearanceCap;
    // Safety margin (v0.7.0: the clearance of the whole path to deadly geometry)
    return base + proximityScore(p) - inputCost;
}

bool Predictor::sameState(Observation const& obs, bool holding) const {
    return obs.tick == m_adviceTick && holding == m_adviceHolding && obs.x == m_stateX && obs.y == m_stateY && obs.vy == m_stateVy &&
           obs.mode == m_computedMode && obs.onGround == m_stateGround && obs.upsideDown == m_stateFlipped &&
           obs.speedClass() == m_stateSpeed && obs.mini == m_stateMini && obs.geometryVersion == m_stateGeometry;
}

bool Predictor::offPredictedPath(Observation const& obs) const {
    // Where the executed branch said the player would be now.
    PredictedPath const& p = executedPath();
    int32_t i = obs.tick - m_computedTick - 1;
    if (!m_executedValid || i < 0 || i >= static_cast<int32_t>(p.points.size())) return true;
    return std::fabs(p.points[static_cast<size_t>(i)].x - obs.x) > 1.f || std::fabs(p.points[static_cast<size_t>(i)].y - obs.y) > 0.5f;
}

void Predictor::evaluate(SimStart const& s, std::span<Obstacle const> obs, Plan const& plan, PredictedPath& tmp,
                         PredictedPath& best, bool& haveBest, int branch) {
    if (haveBest && m_advice.simulations >= m_budgetNow) {
        m_advice.budgetHit = true; // bounded work per decision
        return;
    }
    simulate(s, obs, plan, tmp);
    ++m_advice.simulations;
    ++m_totalSims;
    if (tmp.outcome == Certainty::Safe && m_advice.firstSafeSim < 0) m_advice.firstSafeSim = m_advice.simulations;
    if (tmp.outcome != Certainty::Collision) {
        // How long this plan keeps the branch's first button state.
        size_t const first = static_cast<size_t>(std::clamp(s.lag, 0, 4));
        int32_t keep = static_cast<int32_t>(tmp.holds.size() > first ? tmp.holds.size() - first : 0);
        for (size_t i = first + 1; i < tmp.holds.size(); ++i) {
            if (tmp.holds[i] != tmp.holds[first]) {
                keep = static_cast<int32_t>(i - first);
                break;
            }
        }
        int32_t& k = branch ? m_advice.pressKeepFor : m_advice.releaseKeepFor;
        k = std::max(k, keep);
        if (tmp.outcome == Certainty::Safe) {
            int32_t& ks = branch ? m_advice.pressKeepForSafe : m_advice.releaseKeepForSafe;
            ks = std::max(ks, keep);
        }
    }
    if (!haveBest || score(tmp) > score(best)) {
        std::swap(tmp, best);
        haveBest = true;
    }
}

Predictor::Advice const& Predictor::advise(Observation const& obs, bool holding, bool allowReuse) {
    // The same state again in the same tick: nothing can have changed.
    // (A new learned-timing hint always gets a fresh search.)
    int const nSeeds = m_nSeeds;
    int const extend = m_extendOnce;
    m_extendOnce = 0;
    bool const hasLate = m_hasLateSeed;
    Plan const late = m_lateSeed;
    m_hasLateSeed = false;
    std::array<Plan, kMaxSeeds> const seeds = m_seeds;
    m_nSeeds = 0;
    if (sameState(obs, holding) && (allowReuse || m_computedTick == obs.tick) && m_hint < 0 && nSeeds == 0 && extend == 0 && !hasLate) return m_advice;
    // Reuse the last plan where re-planning cannot change anything (or every
    // other step for continuously controlled modes) - but never while a
    // collision is near, and never once the player is not where the executed
    // plan said it would be.
    bool reuse = false;
    bool const imminent = !m_advice.bestPresses && m_advice.bestPressAt >= 0 && m_advice.bestPressAt <= 2 + (obs.tick - m_computedTick);
    int const ex = m_executedPress ? 1 : 0;
    // (16 steps: several times the time an input needs to take effect)
    bool const danger = m_advice.outcome(ex) != Certainty::Safe || m_advice.keepForSafe(ex) < 16;
    if (allowReuse && m_advice.valid && obs.tick > m_adviceTick && holding == m_adviceHolding && obs.mode == m_computedMode && !imminent &&
        !danger && obs.speedClass() == m_stateSpeed && obs.mini == m_stateMini && obs.geometryVersion == m_stateGeometry &&
        holding == m_executedPress && !offPredictedPath(obs)) {
        // (not with an orb ahead: in the air, a press can activate it)
        bool orbAhead = false;
        for (auto const& o : obs.orbs) orbAhead = orbAhead || (!o.used && o.r > obs.x - 15.f);
        if (groundOnly(obs.mode) && !holding && !obs.onGround && obs.tick <= m_reuseUntilTick && !m_advice.bestPresses && !orbAhead) reuse = true;
        if ((continuousControl(obs.mode) || tapControl(obs.mode)) && obs.tick - m_computedTick < 2) reuse = true;
    }
    if (reuse) {
        m_adviceTick = obs.tick;
        m_stateX = obs.x;
        m_stateY = obs.y;
        m_stateVy = obs.vy;
        m_stateGround = obs.onGround;
        m_stateFlipped = obs.upsideDown;
        m_advice.reused = true;
        m_advice.simulations = 0;
        m_advice.micros = 0.f;
        ++m_reused;
        return m_advice;
    }
    m_reuseUntilTick = INT32_MIN;
    auto const t0 = std::chrono::steady_clock::now();
    m_advice = Advice{};
    m_adviceTick = obs.tick;
    m_computedTick = obs.tick;
    m_computedMode = obs.mode;
    m_stateSpeed = obs.speedClass();
    m_stateMini = obs.mini;
    m_stateGeometry = obs.geometryVersion;
    m_adviceHolding = holding;
    m_advice.computedTick = obs.tick;
    m_budgetNow = m_budget;
    int32_t const hint = m_hint;
    m_hint = -1;
    m_stateX = obs.x;
    m_stateY = obs.y;
    m_stateVy = obs.vy;
    m_stateGround = obs.onGround;
    m_stateFlipped = obs.upsideDown;
    m_executedValid = false;
    m_keep = PredictedPath{};
    m_never = PredictedPath{};
    m_press = PredictedPath{};
    m_release = PredictedPath{};
    if (!supports(obs.mode) || (obs.obstacles.empty() && obs.orbs.empty() && obs.events.empty() && !continuousControl(obs.mode) && !tapControl(obs.mode))) return m_advice;

    SimStart s = startFrom(obs, holding);
    int const savedHorizon = m_horizon;
    if (extend > m_horizon && (s.mode == GameMode::Cube || s.mode == GameMode::Robot)) m_horizon = extend;
    // flying modes: a shorter look-ahead (they re-plan every couple of steps anyway)
    if (continuousControl(s.mode) || tapControl(s.mode)) m_horizon = std::min(m_horizon, m_modeLookahead ? 180 : 120);

    float const reach = s.x + static_cast<float>(m_horizon) * m_model.vxPerTick(s.speedClass) + kNear;
    bool relevant = continuousControl(s.mode) || tapControl(s.mode) || !obs.orbs.empty() || !obs.events.empty();
    for (auto const& o : obs.obstacles) {
        if (o.kind == Obstacle::Kind::Uncertain) continue;
        if (o.l < reach && o.r > s.x - 20.f) {
            relevant = true;
            break;
        }
    }
    if (!relevant) {
        m_horizon = savedHorizon;
        return m_advice;
    }
    m_advice.valid = true;
    m_advice.trusted = trusted(s.mode, s.mini);

    auto const& span = obs.obstacles;
    bool havePress = false, haveRelease = false;
    auto press = [&](Plan p) { evaluate(s, span, p, m_tmp, m_press, havePress, 1); };

    // ---- branch 1: button down this step ----
    switch (s.mode) {
        case GameMode::Cube:
            press({0, 0, 0, false});
            press({0, 0, 0, true});
            break;
        case GameMode::Robot:
            for (int d : {0, 4, 8, 12, 18, 30, 45, 60}) press({0, d, 0, false});
            break;
        case GameMode::Ship:
        case GameMode::Wave:
            for (int d : {2, 6, 12, 24, 48, -1}) press({0, d, 0, false});
            for (auto [hh, r] : {std::pair{4, 10}, {6, 12}, {10, 20}}) press({0, hh, r, false});
            break;
        case GameMode::Ufo:
            for (int r : {0, 18, 30, 45}) press({0, 0, r, false});
            break;
        case GameMode::Swing:
            for (int r : {0, 25, 45}) press({0, 0, r, false});
            break;
        default: // ball, spider
            press({0, 0, 0, false});
            break;
    }

    // ---- branch 2: button up this step; never press, or press k steps later ----
    evaluate(s, span, Plan{-1, 0, 0, false}, m_tmp, m_release, haveRelease, 0);
    // Pressing at step k behaves exactly like never pressing up to step k: if
    // "never" already collides by then, so does every later k (no need to simulate).
    int32_t const neverHit = m_release.outcome == Certainty::Collision ? m_release.collisionStep : INT32_MAX;
    // Near an imminent hazard: find a viable action fast rather than the
    // finest one.
    if (neverHit < 30) m_budgetNow = std::min(m_budget, m_advice.simulations + 48);
    int const interactionBudget = m_budgetNow;
    if (!s.orbs.empty()) m_budgetNow = std::max(m_advice.simulations + 8, m_budgetNow - 64);
    // Drift changes confidence, not the spacing of button opportunities.
    // Search every physics step when callbacks actually arrive every step.
    int const res = std::max(1, static_cast<int>(std::lround(m_interval)));
    m_advice.resolution = res;

    // continuation shapes tried for each delay
    std::array<Plan, 3> shapes{};
    size_t nShapes = 0;
    switch (s.mode) {
        case GameMode::Robot: shapes[nShapes++] = {0, 0, 0, false}; shapes[nShapes++] = {0, 30, 0, false}; break;
        case GameMode::Ship:
        case GameMode::Wave: shapes[nShapes++] = {0, -1, 0, false}; shapes[nShapes++] = {0, 8, 0, false}; break;
        case GameMode::Ufo: shapes[nShapes++] = {0, 0, 0, false}; shapes[nShapes++] = {0, 0, 24, false}; break;
        case GameMode::Swing: shapes[nShapes++] = {0, 0, 0, false}; shapes[nShapes++] = {0, 0, 30, false}; break;
        default: shapes[nShapes++] = {0, 0, 0, false}; break;
    }

    struct Tried {
        int k;
        size_t shape;
        Certainty outcome;
        bool clearedFirst = false; // jumped, landed, and only ran into something after landing
    };
    std::array<Tried, 64> tried{};
    size_t nTried = 0;
    // A timing learned in comparable situations goes first (verified by
    // simulation like everything else, never forced).
    if (hint > 0 && hint < m_horizon && hint <= neverHit) {
        m_advice.seeded = true;
        for (int d : {0, -res, res}) {
            if (hint + d < 1) continue;
            Plan p = shapes[0];
            p.pressAt = hint + d;
            evaluate(s, span, p, m_tmp, m_release, haveRelease, 0);
        }
    }
    // Whole plans from the strategy library: simulated like every other
    // candidate, and only chosen if they score best.
    auto samePlan = [](Plan const& a, Plan const& b) {
        return a.pressAt == b.pressAt && a.holdFor == b.holdFor && a.repeatEvery == b.repeatEvery && a.rejump == b.rejump &&
               a.secondAt == b.secondAt && a.thirdAt == b.thirdAt && a.orbPressAt == b.orbPressAt && a.orbHoldFor == b.orbHoldFor &&
               a.followAt == b.followAt && a.followHold == b.followHold;
    };
    for (int i = 0; i < nSeeds; ++i) {
        Plan const& p = seeds[static_cast<size_t>(i)];
        if (p.pressAt < 0 || p.pressAt >= m_horizon || p.pressAt > neverHit) continue;
        int const branch = p.pressAt == 0 ? 1 : 0;
        PredictedPath& best = branch ? m_press : m_release;
        bool& have = branch ? havePress : haveRelease;
        int const simsBefore = m_advice.simulations;
        evaluate(s, span, p, m_tmp, best, have, branch);
        if (m_advice.simulations == simsBefore) continue;
        PredictedPath const& r = samePlan(best.plan, p) ? best : m_tmp;
        float const sc = score(r);
        if (!m_advice.strategySeeded || sc > m_advice.seedScore) {
            m_advice.seedScore = sc;
            m_advice.seedOutcome = r.outcome;
            m_advice.seedCollisionStep = r.collisionStep;
        }
        m_advice.strategySeeded = true;
    }
    for (int k : kCoarseDelays) {
        if (k >= m_horizon) break;
        for (size_t si = 0; si < nShapes; ++si) {
            // second shapes only for the nearer delays (where they matter most)
            if (si > 0 && k > 40) continue;
            Plan p = shapes[si];
            p.pressAt = k;
            Certainty out;
            bool cleared = false;
            if (k > neverHit) {
                out = Certainty::Collision;
            } else {
                int const simsBefore = m_advice.simulations;
                evaluate(s, span, p, m_tmp, m_release, haveRelease, 0);
                bool kept = m_release.plan.pressAt == k && m_release.plan.holdFor == p.holdFor && m_release.plan.repeatEvery == p.repeatEvery;
                PredictedPath const& r = kept ? m_release : m_tmp;
                out = r.outcome;
                cleared = m_advice.simulations > simsBefore && r.outcome != Certainty::Safe && r.landed && r.landStep >= 0 &&
                          r.collisionStep > r.landStep;
            }
            if (nTried < tried.size()) tried[nTried++] = {k, si, out, cleared};
        }
    }
    // Refine: single-step resolution around the best delay and wherever the
    // outcome changes between neighbouring coarse delays.
    std::array<std::pair<int, size_t>, 16> refine{};
    size_t nRefine = 0;
    auto addRefine = [&](int k, size_t si) {
        if (k < 1 || k >= m_horizon || k > neverHit || nRefine >= refine.size()) return;
        for (size_t i = 0; i < nTried; ++i) if (tried[i].k == k && tried[i].shape == si) return;
        for (size_t i = 0; i < nRefine; ++i) if (refine[i].first == k && refine[i].second == si) return;
        refine[nRefine++] = {k, si};
    };
    if (m_release.plan.pressAt > 0) {
        size_t si = 0;
        for (size_t i = 0; i < nShapes; ++i)
            if (shapes[i].holdFor == m_release.plan.holdFor && shapes[i].repeatEvery == m_release.plan.repeatEvery) si = i;
        for (int d : {-2 * res, -res, res, 2 * res}) addRefine(m_release.plan.pressAt + d, si);
    }
    for (size_t i = 1; i < nTried; ++i) {
        for (size_t j = i; j-- > 0;) {
            if (tried[j].shape != tried[i].shape) continue;
            if (tried[j].outcome != tried[i].outcome) addRefine((tried[i].k + tried[j].k) / 2, tried[i].shape);
            break;
        }
    }
    for (size_t i = 0; i < nRefine; ++i) {
        Plan p = shapes[refine[i].second];
        p.pressAt = refine[i].first;
        evaluate(s, span, p, m_tmp, m_release, haveRelease, 0);
    }
    if ((s.mode == GameMode::Cube || s.mode == GameMode::Robot) && m_release.plan.pressAt > 0) {
        Plan p = m_release.plan;
        p.rejump = true;
        evaluate(s, span, p, m_tmp, m_release, haveRelease, 0);
    }
    m_advice.stage = nRefine > 0 ? SearchStage::Refined : SearchStage::Coarse;

    // Refine both the start and end of a flying pulse at the actual input
    // resolution. Sparse hold lengths alone miss narrow mini-wave corridors.
    if (continuousControl(s.mode) && !m_advice.budgetHit) {
        Plan const bases[] = {m_release.plan, m_press.plan};
        for (int branch = 0; branch < 2; ++branch) {
            Plan const base = bases[branch];
            if (base.pressAt < 0 || base.holdFor <= 0 || base.repeatEvery > 0) continue;
            for (int d : {-2 * res, -res, res, 2 * res}) {
                Plan p = base;
                p.holdFor = base.holdFor + d;
                if (p.holdFor < 1 || p.pressAt + p.holdFor >= m_horizon) continue;
                evaluate(s, span, p, m_tmp, branch ? m_press : m_release,
                         branch ? havePress : haveRelease, branch);
            }
        }
    }
    // Independently timed follow-up inputs, not a fixed repeating rhythm.
    // Try both immediate-input branches and score the full simulated path.
    // The ordinary candidates remain available; this cannot lower their score.
    if (m_modeLookahead && s.mode != GameMode::Cube && s.mode != GameMode::Robot && !m_advice.budgetHit &&
        m_press.outcome != Certainty::Safe && m_release.outcome != Certainty::Safe) {
        Plan const bases[] = {m_release.plan, m_press.plan};
        int const remaining = std::min(32, std::max(0, m_budgetNow - m_advice.simulations));
        int attempted = 0;
        for (int gap : {4, 8, 16, 24, 40, 64, 96, 128}) {
            for (int branch = 0; branch < 2 && attempted < remaining; ++branch) {
                Plan p = bases[branch];
                if (p.pressAt < 0) p.pressAt = branch ? 0 : std::max(1, res);
                p.repeatEvery = 0;
                // Release before a second tap; continuous modes evaluate a
                // finite ascent/descent interval followed by another hold.
                if (continuousControl(s.mode) && p.holdFor <= 0) p.holdFor = 12;
                int firstDuration = p.holdFor > 0 ? p.holdFor : 2;
                p.followAt = p.pressAt + firstDuration + gap;
                if (p.followAt >= m_horizon) continue;
                for (int duration : {2, 16}) {
                    if (attempted >= remaining) break;
                    p.followHold = continuousControl(s.mode) ? duration : 0;
                    evaluate(s, span, p, m_tmp, branch ? m_press : m_release,
                             branch ? havePress : haveRelease, branch);
                    ++attempted;
                    if (!continuousControl(s.mode)) break;
                }
            }
        }
    }

    // ---- Consecutive obstacles: a second jump after the first landing ----
    // A plan whose first jump lands and then runs into the next obstacle gets
    // a second press, timed from its landing (coarse, then refined). The pair
    // is scored as a whole (two presses cost two inputs), so a first-jump
    // timing that sets up an easy second jump beats one that only clears the
    // first obstacle - also a later first jump than the earliest that works.
    // Only when no single-jump plan is safe: otherwise the second jump is a
    // later decision (and the search stays cheap).
    if ((s.mode == GameMode::Cube || s.mode == GameMode::Robot) && !m_advice.budgetHit && m_press.outcome != Certainty::Safe &&
        m_release.outcome != Certainty::Safe) {
        auto needsSecond = [](PredictedPath const& p) {
            return p.outcome != Certainty::Safe && p.landed && p.landStep >= 0 && p.collisionStep > p.landStep;
        };
        struct Base {
            Plan plan;
            int branch;
        };
        std::array<Base, 6> bases{};
        size_t nBases = 0;
        if (needsSecond(m_press) && !m_press.plan.rejump) bases[nBases++] = {m_press.plan, 1};
        // First-jump timings that cleared the first obstacle and only ran into
        // the next one after landing: a spread of them (earliest, middle,
        // latest), not just the one that happened to collide latest.
        std::array<int, 64> cleared{};
        size_t nCleared = 0;
        for (size_t i = 0; i < nTried; ++i)
            if (tried[i].clearedFirst && tried[i].shape == 0 && nCleared < cleared.size()) cleared[nCleared++] = tried[i].k;
        std::sort(cleared.begin(), cleared.begin() + static_cast<std::ptrdiff_t>(nCleared));
        for (size_t idx : {size_t(0), nCleared / 3, (2 * nCleared) / 3, nCleared ? nCleared - 1 : 0}) {
            if (idx >= nCleared || nBases >= bases.size()) continue;
            Plan p = shapes[0];
            p.pressAt = cleared[idx];
            bool dup = false;
            for (size_t b = 0; b < nBases; ++b) dup = dup || (bases[b].branch == 0 && bases[b].plan.pressAt == p.pressAt);
            if (!dup) bases[nBases++] = {p, 0};
        }
        if (needsSecond(m_release) && !m_release.plan.rejump && m_release.plan.pressAt > 0 && nBases < bases.size()) {
            bool dup = false;
            for (size_t b = 0; b < nBases; ++b) dup = dup || (bases[b].branch == 0 && bases[b].plan.pressAt == m_release.plan.pressAt);
            if (!dup) bases[nBases++] = {m_release.plan, 0};
        }
        bool improved = false;
        Plan bestPair;
        float bestPairScore = -1e9f;
        int bestPairBranch = -1;
        auto tryPair = [&](Plan const& p, int branch) {
            PredictedPath& best = branch ? m_press : m_release;
            bool& have = branch ? havePress : haveRelease;
            float const prev = score(best);
            int const simsBefore = m_advice.simulations;
            evaluate(s, span, p, m_tmp, best, have, branch);
            if (m_advice.simulations == simsBefore) return -1e9f; // budget
            bool const kept = best.plan.pressAt == p.pressAt && best.plan.secondAt == p.secondAt;
            float const sc = kept ? score(best) : score(m_tmp);
            if (score(best) > prev) improved = true;
            if (sc > bestPairScore) {
                bestPairScore = sc;
                bestPair = p;
                bestPairBranch = branch;
            }
            return sc;
        };
        for (size_t b = 0; b < nBases; ++b) {
            int bestK2 = -1;
            float bestHere = -1e9f;
            for (int k2 : {0, 4, 8, 12, 18, 26, 36, 50}) {
                Plan p = bases[b].plan;
                p.secondAt = k2;
                float sc = tryPair(p, bases[b].branch);
                if (sc > bestHere) {
                    bestHere = sc;
                    bestK2 = k2;
                }
            }
            for (int d : {-2 * res, -res, res, 2 * res}) {
                if (bestK2 < 0 || bestK2 + d < 0) continue;
                Plan p = bases[b].plan;
                p.secondAt = bestK2 + d;
                tryPair(p, bases[b].branch);
            }
        }
        // Refine the best pair's first jump too (a slightly later or earlier
        // first jump can make the landing, and so the second jump, better).
        if (bestPairBranch == 0 && bestPair.pressAt > 0) {
            Plan const center = bestPair;
            for (int d : {-2 * res, -res, res, 2 * res}) {
                if (center.pressAt + d < 1 || center.pressAt + d > neverHit) continue;
                for (int d2 : {-res, 0, res}) {
                    if (center.secondAt + d2 < 0) continue;
                    Plan p = center;
                    p.pressAt += d;
                    p.secondAt += d2;
                    tryPair(p, 0);
                }
            }
        }
        if (improved && (m_press.plan.secondAt >= 0 || m_release.plan.secondAt >= 0)) m_advice.stage = SearchStage::TwoJump;
    }

    // ---- orbs ahead: activating each one is a candidate action ----
    // For the nearest orbs, the paths of the best plans so far (and of not
    // pressing at all) show when the hitbox overlaps the orb; a press at the
    // start, the middle and the end of that window (for a dash orb with
    // several hold durations) is simulated through everything that follows.
    m_budgetNow = interactionBudget;
    m_advice.budgetHit = m_advice.simulations >= m_budgetNow;
    m_orbOn = PredictedPath{};
    m_orbOff = PredictedPath{};
    if (!s.orbs.empty()) {
        float const reach = s.x + static_cast<float>(m_horizon) * m_model.vxPerTick(s.speedClass) + kNear;
        float const ph = m_model.params(s.mode, s.mini).halfSize;
        int handled = 0;
        for (size_t k = 0; k < s.orbs.size() && handled < 2; ++k) {
            Orb const& ob = s.orbs[k];
            if (ob.used || ob.r < s.x - ph || ob.l > reach) continue;
            ++handled;
            bool const first = handled == 1;
            auto& O = m_advice.orb;
            if (first) {
                O.present = true;
                O.type = ob.type;
                O.id = ob.id;
                O.l = ob.l;
                O.b = ob.b;
                O.r = ob.r;
                O.t = ob.t;
                O.simulated = orbSimulated(ob.type);
                O.known = O.simulated && m_model.orb(ob.type, s.mode, s.mini).known;
                O.overlappingNow = ob.l <= s.x + ph && ob.r >= s.x - ph && ob.b <= s.y + ph && ob.t >= s.y - ph;
            }
            if (!orbSimulated(ob.type)) continue;
            // The jump before the orb decides where (and whether) the orb is
            // reached: the orb press is tried after several jump timings, not
            // only after the plan that is best without the orb.
            Plan bases[16];
            int nb = 0;
            bases[nb++] = Plan{-1, 0, 0, false};
            bases[nb++] = m_press.plan;
            if (m_release.plan.pressAt >= 0) bases[nb++] = m_release.plan;
            if (groundOnly(s.mode) && s.onGround) {
                for (int k : kCoarseDelays) {
                    if (nb >= 16 || k >= m_horizon || k > neverHit) break;
                    if (m_release.plan.pressAt == k) continue;
                    Plan p = shapes[0];
                    p.pressAt = k;
                    bases[nb++] = p;
                }
            }
            for (int bi = 0; bi < nb && m_advice.simulations < m_budgetNow; ++bi) {
                Plan const base = bases[bi];
                if (base.orbPressAt >= 0) continue;
                simulate(s, span, base, m_orbTmp);
                ++m_advice.simulations;
                ++m_totalSims;
                bool const insideNow = ob.l <= s.x + ph && ob.r >= s.x - ph && ob.b <= s.y + ph && ob.t >= s.y - ph;
                int32_t w0 = insideNow ? 0 : -1, w1 = insideNow ? 0 : -1;
                for (size_t i = 0; i < m_orbTmp.points.size(); ++i) {
                    PathPoint const& pt = m_orbTmp.points[i];
                    bool const over = ob.l <= pt.x + ph && ob.r >= pt.x - ph && ob.b <= pt.y + ph && ob.t >= pt.y - ph;
                    if (over && w0 < 0) w0 = static_cast<int32_t>(i) + 1;
                    if (over) w1 = static_cast<int32_t>(i) + 1;
                    if (!over && w0 >= 0) break;
                }
                if (w0 < 0) continue;
                if (first && (O.windowFrom < 0 || w0 < O.windowFrom)) {
                    O.windowFrom = w0;
                    O.windowTo = w1;
                }
                // (the point after step i is reached by the input of step i+1)
                int32_t const ks[3] = {w0, (w0 + w1) / 2, w1};
                static constexpr int32_t kDashHolds[] = {8, 16, 28, 44, 64, 96, 140};
                for (int ki = 0; ki < 3; ++ki) {
                    int32_t const kstep = std::max<int32_t>(0, ks[ki] - m_latency);
                    if (ki > 0 && kstep == std::max<int32_t>(0, ks[ki - 1] - m_latency)) continue;
                    int const nh = isDashOrb(ob.type) ? 7 : 1;
                    for (int hi = 0; hi < nh; ++hi) {
                        Plan p = base;
                        p.orbPressAt = kstep;
                        p.orbHoldFor = isDashOrb(ob.type) ? kDashHolds[hi] : 0;
                        int const branch = (p.pressAt == 0 || kstep == 0) ? 1 : 0;
                        PredictedPath& best = branch ? m_press : m_release;
                        bool& have = branch ? havePress : haveRelease;
                        int const simsBefore = m_advice.simulations;
                        evaluate(s, span, p, m_tmp, best, have, branch);
                        if (m_advice.simulations == simsBefore) continue;
                        PredictedPath const& r = samePlan(best.plan, p) ? best : m_tmp;
                        if (!first || !r.activates(ob.id)) continue;
                        ++O.candidates;
                        float const sc = score(r);
                        if (sc > O.activateScore) {
                            O.activateScore = sc;
                            O.activate = r.outcome;
                            O.activateTTC = r.outcome == Certainty::Collision ? r.collisionStep : -1;
                            O.activateHitsAbove = r.outcome == Certainty::Collision && r.hit.b > r.collisionY;
                            O.activatePlan = p;
                            m_orbOn = r;
                        }
                    }
                }
            }
        }
    }

    // The strategic sequence plan last: chosen only if strictly better.
    if (hasLate && late.pressAt >= 0 && late.pressAt < m_horizon && late.pressAt <= neverHit) {
        int const branch = late.pressAt == 0 ? 1 : 0;
        PredictedPath& best = branch ? m_press : m_release;
        bool& have = branch ? havePress : haveRelease;
        evaluate(s, span, late, m_tmp, best, have, branch);
        m_advice.lateSeeded = true;
    }

    m_advice.press = m_press.outcome;
    m_advice.release = m_release.outcome;
    m_advice.pressTTC = m_press.outcome == Certainty::Collision ? m_press.collisionStep : -1;
    m_advice.releaseTTC = m_release.outcome == Certainty::Collision ? m_release.collisionStep : -1;
    m_advice.pressRisk = m_press.outcome != Certainty::Safe ? m_press.collisionStep : -1;
    m_advice.releaseRisk = m_release.outcome != Certainty::Safe ? m_release.collisionStep : -1;

    // What happens if nobody touches the button: the current state kept.
    simulate(s, span, holding ? Plan{0, -1, 0, false} : Plan{-1, 0, 0, false}, m_keep);
    ++m_advice.simulations;
    ++m_totalSims;
    m_advice.keep = m_keep.outcome;
    m_advice.keepTTC = m_keep.outcome == Certainty::Collision ? m_keep.collisionStep : -1;
    // ... and what happens without pressing at all ("no jump").
    if (holding) {
        simulate(s, span, Plan{-1, 0, 0, false}, m_never);
        ++m_advice.simulations;
        ++m_totalSims;
    } else {
        m_never = m_keep;
    }
    m_advice.never = m_never.outcome;
    m_advice.neverTTC = m_never.outcome == Certainty::Collision ? m_never.collisionStep : -1;
    m_advice.neverRisk = m_never.outcome != Certainty::Safe ? m_never.collisionStep : -1;
    // Prefer not pressing unless pressing now is clearly better: waiting keeps
    // options open. But if the best "wait" plan presses within two steps anyway,
    // waiting gains nothing and risks being too late: press unless waiting is
    // clearly better.
    bool const reallyWaits = m_release.plan.pressAt < 0 || m_release.plan.pressAt >= 3;
    m_bestPresses = reallyWaits ? score(m_press) > score(m_release) + 1.f : score(m_press) + 1.f >= score(m_release);
    m_advice.bestPresses = m_bestPresses;
    m_advice.bestPressAt = m_bestPresses ? 0 : m_release.plan.pressAt;
    {
        PredictedPath const& b = m_bestPresses ? m_press : m_release;
        m_advice.bestPlan = b.plan;
        m_advice.bestScore = score(b);
        m_advice.minProximity = b.minProximity;
        m_advice.minProxStep = b.minProxStep;
        m_advice.proximityRisk = b.proximity.empty() ? 0.f : riskOf(b.minProximity, m_prox);
        m_advice.bestLanded = b.landed;
        m_advice.bestLandX = b.landX;
        m_advice.fromSeed = m_advice.seeded && !m_bestPresses && hint > 0 && std::abs(b.plan.pressAt - hint) <= res;
        for (int i = 0; i < nSeeds && m_advice.strategySeeded && !m_advice.fromStrategy; ++i) {
            Plan const& p = seeds[static_cast<size_t>(i)];
            m_advice.fromStrategy = std::abs(b.plan.pressAt - p.pressAt) <= res && (b.plan.pressAt == 0) == (p.pressAt == 0) &&
                                    (b.plan.secondAt < 0) == (p.secondAt < 0) && b.plan.rejump == p.rejump;
        }
        if (m_advice.fromStrategy) m_advice.fromSeed = true;
        if (m_advice.orb.present) {
            auto& O = m_advice.orb;
            O.chosenActivates = b.activates(O.id);
            // what happens without it: the best plan if it does not use the
            // orb, otherwise the better of the two branches that do not
            PredictedPath const* skip = nullptr;
            for (PredictedPath const* c : {&b, static_cast<PredictedPath const*>(&m_release), static_cast<PredictedPath const*>(&m_press)})
                if (!c->activates(O.id) && (!skip || score(*c) > score(*skip))) skip = c;
            if (!skip) {
                simulate(s, span, Plan{-1, 0, 0, false}, m_orbTmp);
                skip = &m_orbTmp;
            }
            m_orbOff = *skip;
            O.skip = m_orbOff.outcome;
            O.skipTTC = m_orbOff.outcome == Certainty::Collision ? m_orbOff.collisionStep : -1;
            O.skipScore = score(m_orbOff);
        }
        m_advice.fromLateSeed = m_advice.lateSeeded && std::abs(b.plan.pressAt - late.pressAt) <= res && (b.plan.pressAt == 0) == (late.pressAt == 0) &&
                                (b.plan.secondAt < 0) == (late.secondAt < 0) && (b.plan.thirdAt < 0) == (late.thirdAt < 0);
    }

    if (groundOnly(s.mode) && !s.onGround && !holding && !m_bestPresses && m_release.landed) {
        int32_t landStep = 0;
        for (auto const& pt : m_release.points) {
            ++landStep;
            if (pt.x >= m_release.landX) break;
        }
        if (landStep > 24) m_reuseUntilTick = obs.tick + landStep - 20;
    }
    m_horizon = savedHorizon;
    m_advice.micros = std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - t0).count();
    return m_advice;
}

void Predictor::noteExecuted(bool pressed) {
    m_executedPress = pressed;
    m_executedValid = m_advice.valid;
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

void Predictor::updateBounds(Observation const& obs, bool held) {
    ModePhysics const& P = m_model.params(obs.mode, obs.mini);
    float h = P.halfSize;
    bool const hintChanged = obs.hasBoundsHint &&
                             (std::fabs(obs.boundsHintBottom - m_boundsHintBottom) > 1.f || std::fabs(obs.boundsHintTop - m_boundsHintTop) > 1.f);
    if (obs.mode != m_boundsMode || hintChanged) {
        // New mode / corridor: start from the hint (unconfirmed) or the plain level floor.
        bool first = m_boundsMode == GameMode::Count;
        m_boundsMode = obs.mode;
        if (first) {
            m_bounds = Bounds{};
            m_bounds.floor = m_model.cubeFloorY(false) - 15.f;
        }
        // Without a hint the previous floor/ceiling stay, but must be re-confirmed.
        if (obs.mode != GameMode::Cube) {
            m_bounds.floorEstimated = true;
            m_bounds.ceilEstimated = true;
        } else {
            m_bounds.hasCeil = false;
        }
        if (obs.hasBoundsHint && obs.mode != GameMode::Cube) {
            m_bounds.floor = obs.boundsHintBottom;
            m_bounds.hasCeil = obs.boundsHintTop > obs.boundsHintBottom + 2.f * h;
            m_bounds.ceil = obs.boundsHintTop;
            m_bounds.ceilEstimated = true;
        }
        m_boundsHintBottom = obs.boundsHintBottom;
        m_boundsHintTop = obs.boundsHintTop;
    }

    // Confirm a bound by touching it where no block explains the contact.
    auto blockAt = [&](bool below) {
        for (auto const& o : obs.obstacles) {
            if (o.kind != Obstacle::Kind::Solid || o.l >= obs.x + h || o.r <= obs.x - h) continue;
            if (below && std::fabs(o.t - (obs.y - h)) < 1.f) return true;
            if (!below && std::fabs(o.b - (obs.y + h)) < 1.f) return true;
        }
        return false;
    };
    bool const resting = std::fabs(obs.vy) < 0.01f;
    bool bottom = false, top = false;
    if (obs.onGround) {
        (obs.upsideDown ? top : bottom) = true;
    } else if (obs.mode == GameMode::Ship || obs.mode == GameMode::Wave) {
        // flying pressed against a bound: no movement for several steps (a single
        // zero-velocity step also happens when the ship turns around mid-air)
        bool pinned = resting && std::fabs(obs.y - m_lastY) < 0.01f;
        m_pressedSteps = pinned ? m_pressedSteps + 1 : 0;
        if (m_pressedSteps >= 3) (held != obs.upsideDown ? top : bottom) = true;
    }
    m_lastY = obs.y;
    if (bottom || top) {
        bool atBottom = bottom;
        if (atBottom && !blockAt(true)) {
            float surf = obs.y - h;
            if (m_bounds.floorEstimated || surf < m_bounds.floor) {
                m_bounds.floor = surf;
                m_bounds.floorEstimated = false;
                m_bounds.hasFloor = true;
            }
        } else if (!atBottom && !blockAt(false) && obs.mode != GameMode::Cube && obs.mode != GameMode::Robot) {
            // (Cube and robot have no corridor ceiling: an upside-down contact
            // that no known block explains is something local the observer
            // does not report, not a level-wide ceiling. Learning it as one
            // made the model expect a surface under hanging spikes in game.)
            float surf = obs.y + h;
            if (!m_bounds.hasCeil || m_bounds.ceilEstimated || surf > m_bounds.ceil) {
                m_bounds.ceil = surf;
                m_bounds.ceilEstimated = false;
                m_bounds.hasCeil = true;
            }
        }
    }
}

void Predictor::observe(Observation const& obs, bool heldDuringStep) {
    Rec const* last = m_histCount ? &m_hist[(m_histHead + kHistory - 1) % kHistory] : nullptr;
    // Steps since the previous observation (real physics steps), or 0 when
    // the two are not connected (first observation, respawn, teleport).
    int32_t const gap = last && obs.tick > last->tick && obs.tick - last->tick <= 8 ? obs.tick - last->tick : 0;
    if (gap > 0) m_interval = m_interval * 0.95f + static_cast<float>(gap) * 0.05f;

    // robot boost bookkeeping (needed to simulate from the middle of a jump)
    if (obs.mode == GameMode::Robot) {
        if (last && gap > 0 && last->onGround && !obs.onGround && heldDuringStep) {
            m_robotSince = gap - 1;
            m_robotAlive = true;
        } else if (!obs.onGround) {
            m_robotSince += std::max(1, gap);
            if (!heldDuringStep) m_robotAlive = false;
        } else {
            m_robotAlive = false;
        }
    } else {
        m_robotAlive = false;
    }
    // ---- orbs: what the game reported, and dashes in progress ----
    for (auto const& e : obs.orbEvents) {
        m_lastOrbTick = obs.tick;
        if (e.kind == OrbEvent::Kind::Activated) {
            OrbPhysics const known = m_model.orb(e.type, e.mode, e.mini);
            m_orbCheck.tick = obs.tick;
            m_orbCheck.type = e.type;
            m_orbCheck.wasKnown = known.known;
            m_orbCheck.vyError = m_model.observeOrb(e);
            m_orbCheck.dash = isDashOrb(e.type);
            ++m_orbCheck.activations;
            if (!e.newPress) m_model.observeHeldEntry(true);
        } else if (e.kind == OrbEvent::Kind::DashStarted) {
            m_dashActive = true;
            m_dashType = e.type;
            m_dashStartTick = obs.tick;
            m_dashRotation = 0.f;
            m_dashMaxSteps = 1 << 20;
            for (auto const& o : obs.orbs) {
                if (o.id != e.id) continue;
                m_dashRotation = o.rotation;
                if (o.maxDuration > 0.f) m_dashMaxSteps = static_cast<int32_t>(std::lround(o.maxDuration * kTicksPerSecond));
            }
        } else if (e.kind == OrbEvent::Kind::DashStopped) {
            if (m_dashActive)
                m_model.observeDashEnd(m_dashType, e.mode, e.mini, e.flippedAfter ? -e.vyAfter : e.vyAfter, !heldDuringStep);
            m_dashActive = false;
        }
    }
    if (m_dashActive && !obs.dashing && obs.orbEvents.empty()) m_dashActive = false;
    // every step of a dash counts as an orb step (no mode calibration, no
    // accuracy windows over it: the dash is not mode physics)
    if (m_dashActive || obs.dashing) m_lastOrbTick = obs.tick;
    if (m_dashActive && obs.dashing && last && gap == 1 && last->tick >= m_dashStartTick) {
        float const dx = obs.x - last->x, dy = obs.y - last->y;
        // compare with the model before learning from it
        OrbPhysics const op = m_model.orb(m_dashType, obs.mode, obs.mini);
        float const vxs = m_model.vxPerTick(obs.speedClass());
        float const pdy = vxs * op.dashDx * std::tan(-m_dashRotation * 3.14159265f / 180.f) * op.dashSlope;
        m_orbCheck.dashError = std::hypot(dx - vxs * op.dashDx, dy - pdy);
        m_model.observeDashStep(m_dashType, obs.mode, obs.mini, dx, dy, m_dashRotation, obs.speedClass());
    }
    // holding the button into an orb without activating it
    {
        std::vector<int32_t> now;
        float const ph = m_model.params(obs.mode, obs.mini).halfSize;
        for (auto const& o : obs.orbs) {
            if (!(o.l <= obs.x + ph && o.r >= obs.x - ph && o.b <= obs.y + ph && o.t >= obs.y - ph)) continue;
            now.push_back(o.id);
            bool const entered = std::find(m_overOrbs.begin(), m_overOrbs.end(), o.id) == m_overOrbs.end();
            bool const heldBefore = last && last->held;
            if (!entered || !heldDuringStep || !heldBefore || o.used || !orbSimulated(o.type)) continue;
            bool fired = false;
            for (auto const& e : obs.orbEvents) fired = fired || e.id == o.id;
            if (!fired) m_model.observeHeldEntry(false);
        }
        m_overOrbs = std::move(now);
    }

    int32_t boostLeft = obs.mode == GameMode::Robot && m_robotAlive
                            ? std::max(0, m_model.params(obs.mode, obs.mini).boostSteps - m_robotSince) : 0;
    Rec r{obs.x, obs.y, obs.vy, obs.onGround, obs.upsideDown, obs.mini, heldDuringStep, obs.dead, obs.mode, obs.speedClass(), obs.tick,
          obs.playerSpeed, boostLeft};
    if (!obs.dead) updateBounds(obs, heldDuringStep);

    if (last) {
        Rec const& p = *last;
        if (gap == 1) {
            // A single physics step: calibrate the model on it. (Pairs several
            // steps apart are not used: the model learns per step.)
            int32_t const from = p.tick + 1; // steps from here on: heldDuringStep
            bool const effHeld = sentAt(obs.tick - m_latency, from, heldDuringStep);
            bool const effBefore = sentAt(obs.tick - 1 - m_latency, from, heldDuringStep);
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
            prev.playerSpeed = p.playerSpeed;
            // (steps with an orb activation or a dash are not normal movement)
            bool const orbStep = !obs.orbEvents.empty() || obs.dashing || m_dashActive;
            if (!orbStep) m_model.observeStep(prev, obs, effHeld, effBefore);

            // one-step prediction error (diagnostics)
            if (p.mode == obs.mode && p.mini == obs.mini && !p.dead && !obs.dead) {
                SimStart s{p.x, p.y, p.vy, p.mode, p.onGround, p.upsideDown, p.mini, effBefore, p.speedClass, p.tick, m_bounds,
                           p.robotBoostLeft};
                s.measureOnly = true;
                uint8_t hold = effHeld ? 1 : 0;
                simulate(s, obs.obstacles, Plan{}, m_resim, std::span<uint8_t const>(&hold, 1));
                if (!m_resim.points.empty()) {
                    m_predY = m_resim.points.back().y;
                    m_actY = obs.y;
                    m_stepErr = std::fabs(m_predY - m_actY);
                }
            }
        } else if (gap == 0) {
            m_histCount = 0; // not connected (reset, respawn, teleport): start a new history
        }
    }
    m_hist[m_histHead] = r;
    m_histHead = (m_histHead + 1) % kHistory;
    m_histCount = std::min(m_histCount + 1, kHistory);

    // About every 16 steps, re-simulate the last ~32 real physics steps with
    // the button states that were actually in effect (step by step, also
    // across observations several steps apart) and compare the end point
    // with reality. This measures movement only (measureOnly): hazards and
    // error margins play no part, so a near miss that the model's hitboxes
    // call a hit cannot count as an error. It is done for each assumed input
    // latency (0, 1, 2 steps); the one that matches the real game best is
    // used for planning.
    m_sinceResim += std::max(1, gap);
    // (windows with an orb activation or a dash are not re-simulated: the
    // accuracy measurement is for the mode physics)
    if (m_sinceResim >= 16 && m_histCount > 2 && obs.tick - m_lastOrbTick > kResimSteps + kLags + 2) {
        auto at = [&](size_t back) -> Rec const& { return m_hist[(m_histHead + kHistory - 1 - back) % kHistory]; };
        // the newest record at least kResimSteps steps back, with kLags more steps of history before it
        size_t B = 0;
        for (size_t b = 1; b + 1 < m_histCount; ++b) {
            if (at(0).tick - at(b).tick >= kResimSteps) {
                if (at(b).tick - at(m_histCount - 1).tick >= kLags) B = b;
                break;
            }
        }
        if (B > 0) {
            m_sinceResim = 0;
            Rec const& s0 = at(B);
            Rec const& end = at(0);
            int32_t const steps = end.tick - s0.tick;
            bool ok = supports(s0.mode) && steps <= 64;
            for (size_t i = 0; i <= B && ok; ++i) {
                Rec const& q = at(i);
                ok = q.mode == s0.mode && q.mini == s0.mini && !q.dead && q.speedClass == s0.speedClass;
            }
            if (ok) {
                std::array<float, kLags> errs{};
                bool valid = true;
                bool differs = false;
                std::vector<uint8_t> base;
                for (int L = 0; L < kLags && valid; ++L) {
                    m_holdBuf.clear();
                    for (int32_t t = s0.tick + 1; t <= end.tick; ++t) m_holdBuf.push_back(sentAt(t - L) ? 1 : 0);
                    if (L == 0) base = m_holdBuf;
                    else if (m_holdBuf != base) differs = true;
                    SimStart s{s0.x, s0.y, s0.vy, s0.mode, s0.onGround, s0.upsideDown, s0.mini, sentAt(s0.tick - L), s0.speedClass, s0.tick,
                               m_bounds, s0.robotBoostLeft};
                    s.measureOnly = true;
                    simulate(s, obs.obstacles, Plan{}, m_resim, m_holdBuf);
                    if (m_resim.uncertainFrom >= 0 || m_resim.points.empty()) {
                        valid = false; // unmodelled object involved: not the model's fault
                        break;
                    }
                    auto const& e = m_resim.points.back();
                    errs[static_cast<size_t>(L)] = std::hypot(e.x - end.x, e.y - end.y);
                }
                if (valid) {
                    m_model.recordAccuracy(s0.mode, errs[static_cast<size_t>(m_latency)], steps, s0.mini);
                    if (differs) {
                        // Only windows with an input change say anything about latency.
                        for (int L = 0; L < kLags; ++L) {
                            float& a = m_lagErr[static_cast<size_t>(L)];
                            a = m_lagSamples == 0 ? errs[static_cast<size_t>(L)] : a * 0.9f + errs[static_cast<size_t>(L)] * 0.1f;
                        }
                        ++m_lagSamples;
                        if (m_lagSamples >= 12) {
                            int best = m_latency;
                            for (int L = 0; L < kLags; ++L)
                                if (m_lagErr[static_cast<size_t>(L)] < m_lagErr[static_cast<size_t>(best)]) best = L;
                            float cur = m_lagErr[static_cast<size_t>(m_latency)];
                            if (best != m_latency && cur > 0.5f && m_lagErr[static_cast<size_t>(best)] < 0.5f * cur) m_latency = best;
                        }
                    }
                }
            }
        }
    }

    if (m_expectValid && obs.tick > m_expectTick + m_expectSteps) {
        if (m_expectOutcome == Certainty::Safe) m_caution = std::max(1.f, m_caution * 0.98f);
        m_expectValid = false;
    }
}

} // namespace aigd
