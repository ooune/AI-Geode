#pragma once

// A small deterministic Geometry-Dash-like simulator used to test the learning
// core without the game. It is NOT a faithful GD physics engine. It models the
// parts that matter for learning and prediction: fixed 240 Hz steps, constant
// horizontal speed, all eight game modes (cube, ship, ball, UFO, wave, robot,
// spider, swing), gravity portals, spikes and solid blocks, with roughly
// GD-like proportions (30-unit blocks, ~2 block cube jump at 1x).
//
// Its per-mode constants deliberately differ from the predictor's defaults, so
// the predictor has to calibrate itself, as it must in the real game.

#include <aigd/Types.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <deque>
#include <random>
#include <vector>

namespace sim {

using aigd::DeathInfo;
using aigd::Feature;
using aigd::GameMode;
using aigd::KillerKind;
using aigd::Observation;

enum class ObjKind { Spike, Block, ModePortal, GravityPortal, SpeedPortal, Orb, Pad };

struct Obj {
    ObjKind kind;
    float x, y, w, h; // bottom-left corner and size
    GameMode mode = GameMode::Cube; // for mode portals
    bool hidden = false;            // solid in the world, but not reported to the AI (tests the unexpected)
    int speed = 1;                  // speed portals: the new speed class
    int32_t moveAt = -1;            // moves by moveDy (vertically) at this tick (-1 never): a triggered obstacle
    float moveDy = 0.f;
    aigd::OrbType orbType = aigd::OrbType::Yellow; // orbs
    float rotation = 0.f;           // dash orbs: direction (degrees, clockwise like cocos)
    float maxDuration = 0.f;        // dash orbs: seconds (0 = until released)
    aigd::PadType padType = aigd::PadType::Yellow; // pads (rotation 0 = floor pad, 180 = ceiling pad)
};

struct Level {
    std::string name;
    float length = 0.f;
    std::vector<Obj> objs;

    // helpers to build levels in block units
    void spike(float bx, float by = 0.f) { objs.push_back({ObjKind::Spike, bx * 30.f, by * 30.f, 30.f, 30.f}); }
    void block(float bx, float by = 0.f, float bw = 1.f, float bh = 1.f) {
        objs.push_back({ObjKind::Block, bx * 30.f, by * 30.f, bw * 30.f, bh * 30.f});
    }
    void hiddenBlock(float bx, float by = 0.f, float bw = 1.f, float bh = 1.f) {
        objs.push_back({ObjKind::Block, bx * 30.f, by * 30.f, bw * 30.f, bh * 30.f, GameMode::Cube, true});
    }
    void modePortal(float bx, GameMode m) { objs.push_back({ObjKind::ModePortal, bx * 30.f, 0.f, 30.f, 600.f, m}); }
    void shipPortal(float bx) { modePortal(bx, GameMode::Ship); }
    void cubePortal(float bx) { modePortal(bx, GameMode::Cube); }
    void gravityPortal(float bx) { objs.push_back({ObjKind::GravityPortal, bx * 30.f, 0.f, 30.f, 600.f}); }
    void speedPortal(float bx, int speedClass) {
        Obj o{ObjKind::SpeedPortal, bx * 30.f, 0.f, 30.f, 600.f};
        o.speed = speedClass;
        objs.push_back(o);
    }
    // an orb (1 block, its activation area) / a dash orb pointing at `rotation`
    void orb(float bx, float by, aigd::OrbType t) {
        Obj o{ObjKind::Orb, bx * 30.f, by * 30.f, 30.f, 30.f};
        o.orbType = t;
        objs.push_back(o);
    }
    void dashOrb(float bx, float by, float rotation, float maxDuration = 0.f, bool gravity = false) {
        Obj o{ObjKind::Orb, bx * 30.f, by * 30.f, 30.f, 30.f};
        o.orbType = gravity ? aigd::OrbType::GravityDash : aigd::OrbType::Dash;
        o.rotation = rotation;
        o.maxDuration = maxDuration;
        objs.push_back(o);
    }
    // a jump pad: a thin strip on the floor of block cell (bx, by), or under
    // the top of the cell when it hangs from a ceiling (ceiling = true)
    void pad(float bx, float by, aigd::PadType t, bool ceiling = false) {
        Obj o{ObjKind::Pad, bx * 30.f + 2.f, by * 30.f + (ceiling ? 22.f : 0.f), 26.f, 8.f};
        o.padType = t;
        o.rotation = ceiling ? 180.f : 0.f;
        objs.push_back(o);
    }
    // a block that moves vertically by dyBlocks at the given tick
    void movingBlock(float bx, float by, float bw, float bh, int32_t atTick, float dyBlocks) {
        Obj o{ObjKind::Block, bx * 30.f, by * 30.f, bw * 30.f, bh * 30.f};
        o.moveAt = atTick;
        o.moveDy = dyBlocks * 30.f;
        objs.push_back(o);
    }
    void finish(float blocks) {
        length = blocks * 30.f;
        std::stable_sort(objs.begin(), objs.end(), [](Obj const& a, Obj const& b) { return a.x < b.x; });
    }
};

struct Rect {
    float l, b, r, t;
    bool overlaps(Rect const& o) const { return l < o.r && r > o.l && b < o.t && t > o.b; }
};

// Per-mode physics of the simulator (vy units; y changes by vy / kVyScale).
struct ModeSpec {
    float half, inner;
    float accHold, accRelease;
    float maxUp, maxDown;
    float impulse;
    float flipVy;
    float slope;      // wave
    int boostSteps;   // robot
};

class World {
public:
    static constexpr float kHalf = 15.f;     // cube half size
    static constexpr float kVyScale = 4.3f;  // GD-like vy units -> units per tick
    static constexpr float kJumpVy = 11.f;
    static constexpr float kGravity = 0.22f; // cube gravity, vy units per tick

    // Test hook: scales every mode's vertical accelerations, to simulate the
    // game's physics changing (or the model having learned the wrong values).
    static inline float s_gravityScale = 1.f;
    // Test hooks for model/game mismatches:
    static inline int s_inputDelay = 0;        // steps between the button being sent and acting
    static inline float s_noise = 0.f;         // random vertical jitter per airborne step (unpredictable)
    static inline float s_hazardInflate = 0.f; // hazards are reported this much bigger than they kill
    static inline uint32_t s_noiseSeed = 12345u; // seed of the vertical noise (per world reset)
    static inline bool s_orbsDisabled = false;   // orbs never activate (an activation the AI expects does not happen)
    static inline float s_orbScale = 1.f;        // orb velocities scaled (the game differs from what was learned)
    static inline float s_padScale = 1.f;        // pad velocities scaled (the game differs from what was learned)

    static ModeSpec spec(GameMode m) {
        ModeSpec s = baseSpec(m);
        s.accHold *= s_gravityScale;
        s.accRelease *= s_gravityScale;
        return s;
    }
    static ModeSpec baseSpec(GameMode m) {
        switch (m) {
            case GameMode::Robot: return {15.f, 4.5f, -0.06f, -0.22f, 99.f, 15.f, 7.5f, 0.f, 0.f, 50};
            case GameMode::Ship: return {12.f, 4.5f, 0.16f, -0.20f, 7.5f, 7.5f, 0.f, 0.f, 0.f, 0};
            case GameMode::Ufo: return {15.f, 4.5f, -0.12f, -0.12f, 99.f, 9.f, 6.2f, 0.f, 0.f, 0};
            case GameMode::Ball: return {15.f, 4.5f, -0.14f, -0.14f, 99.f, 11.f, 0.f, -1.5f, 0.f, 0};
            case GameMode::Wave: return {6.f, 3.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.1f, 0};
            case GameMode::Spider: return {15.f, 4.5f, -0.22f, -0.22f, 99.f, 14.f, 0.f, 0.f, 0.f, 0};
            case GameMode::Swing: return {15.f, 4.5f, -0.12f, -0.12f, 99.f, 8.5f, 0.f, -0.8f, 0.f, 0};
            default: return {15.f, 4.5f, -kGravity, -kGravity, 99.f, 15.f, kJumpVy, 0.f, 0.f, 0};
        }
    }
    static Rect spikeHitbox(Obj const& o) { return Rect{o.x + 10.5f, o.y + 3.f, o.x + 19.5f, o.y + 18.f}; }
    static Rect spikeHitboxAt(Obj const& o, float y) { return Rect{o.x + 10.5f, y + 3.f, o.x + 19.5f, y + 18.f}; }
    // where a (possibly triggered) object is now
    float yOf(Obj const& o) const { return o.y + (o.moveAt >= 0 && m_tick >= o.moveAt ? o.moveDy : 0.f); }
    int32_t tick() const { return m_tick; }
    int speedClass() const { return m_speedClass; }

    explicit World(Level const& level, int speedClass = 1) : m_level(level), m_speedClass(speedClass), m_startSpeed(speedClass) { reset(); }

    void reset(float startX = 0.f) {
        m_x = startX + kHalf;
        m_y = kHalf;
        m_vy = 0.f;
        m_mode = GameMode::Cube;
        m_onGround = true;
        m_upsideDown = false;
        m_prevHold = false;
        m_boostLeft = 0;
        m_dead = false;
        m_complete = false;
        m_tick = 0;
        m_pending.clear();
        m_noise = s_noiseSeed;
        m_death = DeathInfo{};
        m_portalsPassed = 0;
        m_speedPassed = 0;
        m_speedClass = m_startSpeed;
        m_orbUsed.assign(m_level.objs.size(), 0);
        m_orbInside.assign(m_level.objs.size(), 0);
        m_padUsed.assign(m_level.objs.size(), 0);
        m_padEvents.clear();
        m_events.clear();
        m_dashing = false;
        m_dashLeft = 0;
        for (auto const& o : m_level.objs) {
            if (o.x > m_x + kHalf) break;
            if (o.kind == ObjKind::ModePortal && o.x + o.w * 0.5f <= m_x) m_mode = o.mode;
            if (o.kind == ObjKind::GravityPortal) {
                m_upsideDown = !m_upsideDown;
                ++m_portalsPassed;
            }
            if (o.kind == ObjKind::SpeedPortal) {
                m_speedClass = o.speed;
                ++m_speedPassed;
            }
        }
    }

    // Starts the player at an arbitrary state (for predictor tests).
    void place(float x, float y, float vy, bool onGround, bool upsideDown = false, GameMode mode = GameMode::Cube) {
        m_x = x;
        m_y = y;
        m_vy = vy;
        m_onGround = onGround;
        m_upsideDown = upsideDown;
        m_mode = mode;
        m_prevHold = false;
        m_portalsPassed = 0;
        for (auto const& o : m_level.objs)
            if (o.kind == ObjKind::GravityPortal && o.x <= x + kHalf) ++m_portalsPassed;
    }

    // Collision shapes near x, sorted by left edge, in the form the predictor uses.
    void collectObstacles(float x0, float x1, std::vector<aigd::Obstacle>& out) const {
        out.clear();
        for (auto const& o : m_level.objs) {
            if (o.x > x1) break;
            if (o.x + o.w < x0) continue;
            if (o.hidden) continue;
            if (o.kind == ObjKind::GravityPortal || o.kind == ObjKind::SpeedPortal || o.kind == ObjKind::Orb || o.kind == ObjKind::Pad)
                continue; // events / orbs / pads
            aigd::Obstacle ob;
            float const oy = yOf(o);
            if (o.kind == ObjKind::Spike) {
                Rect r = spikeHitboxAt(o, oy);
                float const g = s_hazardInflate;
                ob = {r.l - g, r.b - g, r.r + g, r.t + g, aigd::Obstacle::Kind::Hazard};
            } else if (o.kind == ObjKind::Block) {
                ob = {o.x, oy, o.x + o.w, oy + o.h, aigd::Obstacle::Kind::Solid};
            } else {
                ob = {o.x, o.y, o.x + o.w, o.y + o.h, aigd::Obstacle::Kind::Uncertain};
            }
            out.push_back(ob);
        }
        std::sort(out.begin(), out.end(), [](aigd::Obstacle const& a, aigd::Obstacle const& b) { return a.l < b.l; });
    }

    // Speed and gravity portals in [x0, x1], as the predictor's physics events
    // (the simulator's own rules, so verified).
    void collectEvents(float x0, float x1, std::vector<aigd::PhysicsEvent>& out) const {
        out.clear();
        for (auto const& o : m_level.objs) {
            if (o.x > x1) break;
            if (o.x + o.w < x0) continue;
            if (o.kind == ObjKind::GravityPortal)
                out.push_back({o.x, o.y, o.x + o.w, o.y + o.h, aigd::PhysicsEvent::Kind::GravityToggle, 0, true});
            else if (o.kind == ObjKind::SpeedPortal)
                out.push_back({o.x, o.y, o.x + o.w, o.y + o.h, aigd::PhysicsEvent::Kind::Speed, static_cast<uint8_t>(o.speed), true});
        }
    }

    // Orbs in [x0, x1] as the observer reports them (used = already activated).
    void collectOrbs(float x0, float x1, std::vector<aigd::Orb>& out) const {
        out.clear();
        for (size_t i = 0; i < m_level.objs.size(); ++i) {
            auto const& o = m_level.objs[i];
            if (o.x > x1) break;
            if (o.kind != ObjKind::Orb || o.x + o.w < x0) continue;
            aigd::Orb b;
            b.l = o.x;
            b.b = o.y;
            b.r = o.x + o.w;
            b.t = o.y + o.h;
            b.type = o.orbType;
            b.rotation = o.rotation;
            b.maxDuration = o.maxDuration;
            b.used = m_orbUsed.size() > i && m_orbUsed[i];
            b.id = static_cast<int32_t>(i);
            out.push_back(b);
        }
    }
    // Pads in [x0, x1] as the observer reports them (used = already activated).
    void collectPads(float x0, float x1, std::vector<aigd::Pad>& out) const {
        out.clear();
        for (size_t i = 0; i < m_level.objs.size(); ++i) {
            auto const& o = m_level.objs[i];
            if (o.x > x1) break;
            if (o.kind != ObjKind::Pad || o.x + o.w < x0) continue;
            aigd::Pad p;
            p.l = o.x;
            p.b = o.y;
            p.r = o.x + o.w;
            p.t = o.y + o.h;
            p.type = o.padType;
            p.rotation = o.rotation;
            p.used = m_padUsed.size() > i && m_padUsed[i];
            p.id = static_cast<int32_t>(i);
            out.push_back(p);
        }
    }
    // Pad activations of the last step (what the game's hook reports in game).
    std::vector<aigd::PadEvent> const& padEvents() const { return m_padEvents; }
    // Orb activations / dash starts and stops of the last step (what the
    // game's hooks report in game).
    std::vector<aigd::OrbEvent> const& orbEvents() const { return m_events; }
    bool dashing() const { return m_dashing; }

    // The simulator's orb effects (its own constants: the predictor learns
    // them from the events, as it must in game).
    static constexpr float kYellowOrb = 11.2f, kPinkOrb = 7.9f, kRedOrb = 14.6f, kBlueOrb = -3.3f, kGreenOrb = 11.f, kBlackOrb = -17.6f;
    static constexpr float kDashEndVy = 2.f;
    // The simulator's pad effects (learned by the predictor from the events).
    static constexpr float kYellowPad = 16.f, kPinkPad = 10.6f, kRedPad = 20.f, kBluePad = -9.f;

    Observation observe() const {
        Observation o;
        o.tick = m_tick;
        o.x = m_x;
        o.y = m_y;
        o.vy = m_vy;
        o.mode = m_mode;
        o.onGround = m_onGround;
        o.upsideDown = m_upsideDown;
        o.dead = m_dead;
        o.playerSpeed = kSpeeds[m_speedClass];
        o.levelLength = m_level.length;
        o.hazard = nearest(ObjKind::Spike);
        o.solid = nearest(ObjKind::Block);
        return o;
    }

    // Advances one physics step with the button held or released.
    //  - cube: holding on the ground jumps (also right after landing, next step)
    //  - robot: like cube with a lower jump that is boosted while held, briefly
    //  - ship: holding accelerates up, releasing down
    //  - UFO: every new press kicks upward
    //  - ball: holding on the ground flips gravity
    //  - wave: held = diagonal up, released = diagonal down
    //  - spider: a new press on the ground teleports to the surface opposite
    //  - swing: every new press flips gravity
    // Blocks: reaching a top/bottom surface puts the player on it (or slides
    // along it); they only kill when the small inner hitbox enters them.
    // Spikes kill on contact with the outer hitbox.
    void step(bool hold) {
        if (m_dead || m_complete) return;
        ++m_tick;
        if (s_inputDelay > 0) {
            m_pending.push_back(hold);
            hold = false;
            if (static_cast<int>(m_pending.size()) > s_inputDelay) {
                hold = m_pending.front();
                m_pending.pop_front();
            }
        }
        ModeSpec const S = spec(m_mode);
        float const vx = aigd::unitsPerSecond(m_speedClass) / aigd::kTicksPerSecond;
        float const half = S.half;
        float const prevBottom = m_y - half;
        float const prevTop = m_y + half;
        bool const edge = hold && !m_prevHold;
        m_prevHold = hold;

        float sg = m_upsideDown ? -1.f : 1.f;
        float vyN = m_vy * sg;
        bool teleported = false;

        // ---- orbs: a new press inside one, or holding when entering it ----
        m_events.clear();
        bool orbFired = false;
        auto event = [&](aigd::OrbEvent::Kind k, aigd::OrbType t, int32_t id, float vyBefore, bool flippedBefore) {
            aigd::OrbEvent e;
            e.kind = k;
            e.type = t;
            e.id = id;
            e.mode = m_mode;
            e.x = m_x;
            e.y = m_y;
            e.vyBefore = vyBefore;
            e.flippedBefore = flippedBefore;
            e.newPress = edge;
            return e;
        };
        if (m_dashing && (!hold || m_dashLeft <= 0)) {
            m_dashing = false;
            aigd::OrbEvent e = event(aigd::OrbEvent::Kind::DashStopped, m_dashType, m_dashId, m_vy, m_upsideDown);
            vyN = kDashEndVy;
            e.vyAfter = vyN * sg;
            e.flippedAfter = m_upsideDown;
            m_events.push_back(e);
        }
        if (!m_dashing) {
            Rect const player{m_x - half, m_y - half, m_x + half, m_y + half};
            for (size_t i = 0; i < m_level.objs.size(); ++i) {
                auto const& o = m_level.objs[i];
                if (o.x > m_x + 60.f) break;
                if (o.kind != ObjKind::Orb) continue;
                bool const over = player.overlaps(Rect{o.x, o.y, o.x + o.w, o.y + o.h});
                bool const entering = over && !m_orbInside[i];
                m_orbInside[i] = over ? 1 : 0;
                if (!over || m_orbUsed[i] || !(edge || (hold && entering)) || s_orbsDisabled) continue;
                m_orbUsed[i] = 1;
                aigd::OrbEvent e = event(aigd::OrbEvent::Kind::Activated, o.orbType, static_cast<int32_t>(i), m_vy, m_upsideDown);
                bool const flips = o.orbType == aigd::OrbType::Blue || o.orbType == aigd::OrbType::Green || o.orbType == aigd::OrbType::GravityDash;
                if (flips) {
                    m_upsideDown = !m_upsideDown;
                    sg = -sg;
                    vyN = -vyN;
                }
                switch (o.orbType) {
                    case aigd::OrbType::Yellow: vyN = kYellowOrb; break;
                    case aigd::OrbType::Pink: vyN = kPinkOrb; break;
                    case aigd::OrbType::Red: vyN = kRedOrb; break;
                    case aigd::OrbType::Blue: vyN = kBlueOrb; break;
                    case aigd::OrbType::Green: vyN = kGreenOrb; break;
                    case aigd::OrbType::Black: vyN = kBlackOrb; break;
                    case aigd::OrbType::Dash:
                    case aigd::OrbType::GravityDash: {
                        m_dashing = true;
                        m_dashType = o.orbType;
                        m_dashId = static_cast<int32_t>(i);
                        m_dashDy = vx * std::tan(-o.rotation * 3.14159265f / 180.f);
                        m_dashLeft = o.maxDuration > 0.f ? static_cast<int>(std::lround(o.maxDuration * 240.f)) : (1 << 20);
                        break;
                    }
                    default: break;
                }
                if (!m_dashing) vyN *= s_orbScale;
                m_onGround = false;
                orbFired = true;
                e.vyAfter = vyN * sg;
                e.flippedAfter = m_upsideDown;
                m_events.push_back(e);
                if (m_dashing) {
                    aigd::OrbEvent d = e;
                    d.kind = aigd::OrbEvent::Kind::DashStarted;
                    m_events.push_back(d);
                }
                break;
            }
        }

        // ---- pads: they act by themselves on the first touch ----
        m_padEvents.clear();
        bool padFired = false;
        if (!m_dashing && !orbFired) {
            Rect const player{m_x - half, m_y - half, m_x + half, m_y + half};
            for (size_t i = 0; i < m_level.objs.size(); ++i) {
                auto const& o = m_level.objs[i];
                if (o.x > m_x + 60.f) break;
                if (o.kind != ObjKind::Pad || m_padUsed[i]) continue;
                if (!player.overlaps(Rect{o.x, o.y, o.x + o.w, o.y + o.h})) continue;
                m_padUsed[i] = 1;
                aigd::PadEvent e;
                e.type = o.padType;
                e.id = static_cast<int32_t>(i);
                e.mode = m_mode;
                e.x = m_x;
                e.y = m_y;
                e.vyBefore = m_vy;
                e.flippedBefore = m_upsideDown;
                e.rotation = o.rotation;
                if (o.padType == aigd::PadType::Blue) {
                    m_upsideDown = !m_upsideDown;
                    sg = -sg;
                }
                switch (o.padType) {
                    case aigd::PadType::Yellow: vyN = kYellowPad; break;
                    case aigd::PadType::Pink: vyN = kPinkPad; break;
                    case aigd::PadType::Red: vyN = kRedPad; break;
                    case aigd::PadType::Blue: vyN = kBluePad; break;
                    default: break;
                }
                vyN *= s_padScale;
                m_onGround = false;
                padFired = true;
                e.vyAfter = vyN * sg;
                e.flippedAfter = m_upsideDown;
                m_padEvents.push_back(e);
                break;
            }
        }

        if (!orbFired && !padFired && !m_dashing) switch (m_mode) {
            case GameMode::Cube:
                if (m_onGround && hold) {
                    vyN = S.impulse;
                    m_onGround = false;
                }
                if (!m_onGround) vyN += S.accRelease;
                break;
            case GameMode::Robot:
                if (m_onGround && hold) {
                    vyN = S.impulse;
                    m_onGround = false;
                    m_boostLeft = S.boostSteps;
                }
                if (!m_onGround) {
                    if (!hold) m_boostLeft = 0; // letting go ends the boost
                    bool boost = hold && m_boostLeft > 0;
                    vyN += boost ? S.accHold : S.accRelease;
                    if (m_boostLeft > 0) --m_boostLeft;
                }
                break;
            case GameMode::Ship:
                vyN += hold ? S.accHold : S.accRelease;
                if (vyN > 0.f) m_onGround = false;
                break;
            case GameMode::Ufo:
                if (edge) {
                    vyN = S.impulse;
                    m_onGround = false;
                } else if (!m_onGround) {
                    vyN += S.accRelease;
                }
                break;
            case GameMode::Ball:
            case GameMode::Swing: {
                bool trigger = m_mode == GameMode::Ball ? (m_onGround && hold) : edge;
                if (trigger) {
                    m_upsideDown = !m_upsideDown;
                    sg = -sg;
                    vyN = S.flipVy;
                    m_onGround = false;
                } else if (!m_onGround) {
                    vyN += S.accRelease;
                }
                break;
            }
            case GameMode::Spider:
                if (m_onGround && edge) {
                    float dest = 0.f;
                    bool found = false;
                    for (auto const& o : m_level.objs) {
                        if (o.kind != ObjKind::Block || o.x >= m_x + half || o.x + o.w <= m_x - half) continue;
                        if (!m_upsideDown && o.y >= m_y + half - 1.f) {
                            float c = o.y - half;
                            if (!found || c < dest) dest = c, found = true;
                        } else if (m_upsideDown && o.y + o.h <= m_y - half + 1.f) {
                            float c = o.y + o.h + half;
                            if (!found || c > dest) dest = c, found = true;
                        }
                    }
                    if (m_upsideDown && (!found || half > dest)) dest = half, found = true; // the floor
                    if (found) {
                        m_y = dest;
                        m_upsideDown = !m_upsideDown;
                        sg = -sg;
                        vyN = 0.f;
                        m_onGround = true;
                        teleported = true;
                    }
                } else if (!m_onGround) {
                    vyN += S.accRelease;
                }
                break;
            case GameMode::Wave:
                vyN = (hold ? 1.f : -1.f) * S.slope * vx * kVyScale;
                if (vyN > 0.f) m_onGround = false;
                break;
            default:
                break;
        }
        if (m_mode != GameMode::Wave && !m_dashing) vyN = std::clamp(vyN, -S.maxDown, S.maxUp);
        m_vy = vyN * sg;
        if (m_dashing) {
            m_vy = m_dashDy * kVyScale; // a straight line, gravity off
            --m_dashLeft;
        }
        m_x += vx;
        if (!teleported) m_y += m_vy / kVyScale;
        if (s_noise > 0.f && !m_onGround) {
            m_noise = m_noise * 1664525u + 1013904223u;
            m_y += s_noise * (static_cast<float>(m_noise >> 8) / 8388608.f - 1.f);
        }

        // the level floor (stood on only when gravity is normal)
        bool grounded = teleported;
        if (m_y - half <= 0.f) {
            m_y = half;
            if (m_vy < 0.f) m_vy = 0.f;
            if (!m_upsideDown) grounded = true;
        }

        Rect player{m_x - half, m_y - half, m_x + half, m_y + half};
        for (auto const& o : m_level.objs) {
            if (o.x > m_x + 60.f) break;
            if (o.x + o.w < m_x - 60.f) continue;
            if (o.kind == ObjKind::ModePortal) {
                if (m_x >= o.x + o.w * 0.5f && m_x - vx < o.x + o.w * 0.5f && m_mode != o.mode) {
                    m_mode = o.mode;
                    m_boostLeft = 0;
                }
                continue;
            }
            if (o.kind == ObjKind::SpeedPortal) {
                int passed = 0;
                int speed = m_speedClass;
                for (auto const& g : m_level.objs)
                    if (g.kind == ObjKind::SpeedPortal && g.x <= m_x + half) ++passed, speed = g.speed;
                if (passed > m_speedPassed) {
                    m_speedPassed = passed;
                    m_speedClass = speed;
                }
                continue;
            }
            if (o.kind == ObjKind::GravityPortal) {
                int passed = 0;
                for (auto const& g : m_level.objs)
                    if (g.kind == ObjKind::GravityPortal && g.x <= m_x + half) ++passed;
                if (passed > m_portalsPassed) {
                    m_portalsPassed = passed;
                    m_upsideDown = !m_upsideDown;
                    m_onGround = false;
                    grounded = false;
                }
                continue;
            }
            if (o.kind == ObjKind::Orb || o.kind == ObjKind::Pad) continue; // orbs and pads are not solid (handled before the move)
            float const oy = yOf(o);
            if (o.kind == ObjKind::Spike) {
                if (player.overlaps(spikeHitboxAt(o, oy))) {
                    die(o, KillerKind::Hazard);
                    return;
                }
                continue;
            }
            Rect blk{o.x, oy, o.x + o.w, oy + o.h};
            if (!player.overlaps(blk)) continue;
            Rect inner{m_x - S.inner, m_y - S.inner, m_x + S.inner, m_y + S.inner};
            bool fromAbove = m_vy <= 0.f && prevBottom >= blk.t - 2.f && m_y - half < blk.t;
            bool fromBelow = m_vy >= 0.f && prevTop <= blk.b + 2.f && m_y + half > blk.b;
            if (fromAbove || fromBelow) {
                if (inner.overlaps(blk)) {
                    die(o, KillerKind::Solid);
                    return;
                }
                m_y = fromAbove ? blk.t + half : blk.b - half;
                m_vy = 0.f;
                if ((fromAbove && !m_upsideDown) || (fromBelow && m_upsideDown)) grounded = true;
            } else if (inner.overlaps(blk)) {
                die(o, KillerKind::Solid);
                return;
            }
            player = Rect{m_x - half, m_y - half, m_x + half, m_y + half};
        }

        // still supported? (walking off the edge of a block)
        if (!grounded && m_onGround && m_vy * (m_upsideDown ? -1.f : 1.f) <= 0.f) {
            grounded = supported(half);
            if (grounded) m_vy = 0.f;
        }
        m_onGround = grounded;

        if (m_x >= m_level.length) m_complete = true;
    }

    bool dead() const { return m_dead; }
    bool complete() const { return m_complete; }
    DeathInfo const& death() const { return m_death; }
    float x() const { return m_x; }
    GameMode mode() const { return m_mode; }

private:
    static constexpr float kSpeeds[5] = {0.7f, 0.9f, 1.1f, 1.3f, 1.6f};

    Level const& m_level;
    int m_speedClass;
    int m_startSpeed = 1;
    int m_speedPassed = 0;
    std::vector<uint8_t> m_orbUsed, m_orbInside, m_padUsed;
    std::vector<aigd::PadEvent> m_padEvents;
    std::vector<aigd::OrbEvent> m_events;
    bool m_dashing = false;
    aigd::OrbType m_dashType = aigd::OrbType::Dash;
    int32_t m_dashId = 0;
    float m_dashDy = 0.f;
    int m_dashLeft = 0;
    float m_x = 0, m_y = 0, m_vy = 0;
    GameMode m_mode = GameMode::Cube;
    bool m_onGround = true;
    bool m_upsideDown = false;
    bool m_prevHold = false;
    std::deque<bool> m_pending;
    uint32_t m_noise = 12345u;
    int m_boostLeft = 0;
    int m_portalsPassed = 0;
    bool m_dead = false;
    bool m_complete = false;
    int32_t m_tick = 0;
    DeathInfo m_death;

    bool supported(float half) const {
        if (!m_upsideDown && m_y - half <= 0.5f) return true;
        for (auto const& o : m_level.objs) {
            if (o.kind != ObjKind::Block) continue;
            if (o.x > m_x + half) break;
            if (o.x + o.w <= m_x - half) continue;
            float surface = m_upsideDown ? yOf(o) : yOf(o) + o.h;
            float edge = m_upsideDown ? m_y + half : m_y - half;
            if (std::fabs(surface - edge) < 0.75f) return true;
        }
        return false;
    }

    void die(Obj const& o, KillerKind kind) {
        m_dead = true;
        m_death.x = m_x;
        m_death.y = m_y;
        m_death.hasKiller = true;
        m_death.killer = kind;
        m_death.killerDx = (o.x + o.w * 0.5f) - m_x;
        float sg = m_upsideDown ? -1.f : 1.f;
        float cy = o.kind == ObjKind::Spike ? o.y + 10.f : o.y + o.h * 0.5f;
        m_death.killerDy = (cy - m_y) * sg;
        m_death.hasExtent = true;
        float bottom = o.kind == ObjKind::Spike ? o.y + 3.f : o.y;
        float top = o.kind == ObjKind::Spike ? o.y + 18.f : o.y + o.h;
        float a = (top - m_y) * sg, b = (bottom - m_y) * sg;
        m_death.killerTop = std::max(a, b);
        m_death.killerBottom = std::min(a, b);
    }

    Feature nearest(ObjKind kind) const {
        Feature f;
        float best = 1e9f;
        for (auto const& o : m_level.objs) {
            if (o.x > m_x + 240.f) break;
            if (o.kind != kind) continue;
            float cx = o.x + o.w * 0.5f;
            float cy = o.y + o.h * 0.5f;
            float dx = cx - m_x;
            float dy = (cy - m_y) * (m_upsideDown ? -1.f : 1.f);
            if (dx < -15.f || std::fabs(dy) > 90.f) continue;
            if (dx < best) {
                best = dx;
                f.present = true;
                f.dx = dx;
                f.dy = dy;
            }
        }
        return f;
    }
};

// ---------------------------------------------------------------------------
// Test levels, from easy to harder.

inline Level levelBasics() {
    Level L;
    L.name = "basics";
    for (float b = 12; b < 100; b += 9) L.spike(b);
    L.finish(110);
    return L;
}

inline Level levelSpikes() {
    Level L;
    L.name = "spikes";
    L.spike(12);
    L.spike(22); L.spike(23);
    L.spike(33);
    L.spike(42); L.spike(43); L.spike(44);
    L.spike(55); L.spike(56);
    L.spike(64);
    L.spike(70);
    L.spike(78); L.spike(79); L.spike(80);
    L.spike(92); L.spike(93);
    L.finish(105);
    return L;
}

inline Level levelBlocks() {
    Level L;
    L.name = "blocks";
    L.block(12);
    L.block(20); L.block(21); L.spike(22); L.spike(23);
    L.block(30); L.block(31, 0, 1, 2); L.block(32, 0, 3, 2);
    L.spike(38); L.spike(39);
    L.block(46, 1, 4, 1); L.spike(47); L.spike(48);
    L.spike(57); L.block(62); L.spike(63);
    L.spike(72); L.spike(73); L.spike(74);
    L.finish(84);
    return L;
}

inline Level levelShip() {
    Level L;
    L.name = "ship";
    L.spike(12);
    L.spike(20); L.spike(21);
    L.shipPortal(28);
    // ceiling for the ship section
    L.block(30, 6, 60, 1);
    L.block(36, 0, 1, 2);
    L.block(44, 4, 1, 2);
    L.block(52, 0, 1, 3);
    L.block(60, 3, 1, 3);
    L.block(68, 0, 1, 2);
    L.block(74, 4, 1, 2);
    L.cubePortal(90);
    L.spike(100);
    L.spike(108); L.spike(109);
    L.finish(118);
    return L;
}

inline std::vector<Level> allLevels() { return {levelBasics(), levelSpikes(), levelBlocks(), levelShip()}; }


// ---------------------------------------------------------------------------
// Procedurally generated cube levels, for testing transfer between levels.
// A level is a random sequence of obstacle patterns with random spacing (and
// optionally a random offset), so two seeds give different levels made of the
// same kinds of obstacles. Training and evaluation use disjoint seeds.

enum class Pattern : int {
    Spike1 = 0, Spike2, Spike3, Block1, BlockSpike, Stairs, RaisedSpikes, SpikeGapSpike, Count
};

inline float placePattern(Level& L, Pattern p, float x) {
    switch (p) {
        case Pattern::Spike1: L.spike(x); return 1;
        case Pattern::Spike2: L.spike(x); L.spike(x + 1); return 2;
        case Pattern::Spike3: L.spike(x); L.spike(x + 1); L.spike(x + 2); return 3;
        case Pattern::Block1: L.block(x); return 1;
        case Pattern::BlockSpike: L.block(x); L.block(x + 1); L.spike(x + 2); L.spike(x + 3); return 4;
        case Pattern::Stairs: L.block(x); L.block(x + 1, 0, 1, 2); L.block(x + 2, 0, 3, 2); return 5;
        case Pattern::RaisedSpikes: L.block(x, 1, 4, 1); L.spike(x + 1); L.spike(x + 2); return 4;
        case Pattern::SpikeGapSpike: L.spike(x); L.spike(x + 4); return 5;
        default: return 0;
    }
}

// `allowed`: bit mask of the patterns that may appear (all by default).
inline Level generatedLevel(uint32_t seed, int segments = 9, uint32_t allowed = ~0u) {
    std::mt19937 rng(seed * 2654435761u + 12345u);
    Level L;
    L.name = "gen-" + std::to_string(seed);
    std::vector<Pattern> pool;
    for (int i = 0; i < static_cast<int>(Pattern::Count); ++i)
        if (allowed & (1u << i)) pool.push_back(static_cast<Pattern>(i));
    float x = 12.f + static_cast<float>(rng() % 6);
    for (int i = 0; i < segments; ++i) {
        Pattern p = pool[rng() % pool.size()];
        x += placePattern(L, p, x);
        x += 6.f + static_cast<float>(rng() % 6); // free run before the next pattern
    }
    L.finish(x + 8.f);
    return L;
}

// The same level with everything moved by `blocks` (a level that only
// memorised positions would not recognise anything in it).
inline Level shiftedLevel(Level const& base, float blocks) {
    Level L = base;
    L.name = base.name + "-shifted";
    for (auto& o : L.objs) o.x += blocks * 30.f;
    L.finish(base.length / 30.f + blocks);
    return L;
}

// A corridor course for one game mode: a portal into the mode right away,
// a floor and a ceiling (block rows), and a handful of obstacles that need the
// mode's own control (fly over / under, flip, teleport, tap).
inline Level modeCourse(GameMode m) {
    Level L;
    L.name = std::string("course-") + aigd::gameModeName(m);
    L.modePortal(2, m);
    float len = 90.f;
    switch (m) {
        case GameMode::Cube:
        case GameMode::Robot:
            for (float b = 14; b < 80; b += 11) L.spike(b);
            L.block(40, 0, 3, 1);
            break;
        case GameMode::Ship:
        case GameMode::Wave:
            L.block(6, 7, len, 1); // ceiling (y 210..240)
            L.block(18, 0, 1, 3);
            L.block(30, 4, 1, 3);
            L.block(42, 0, 1, 3);
            L.block(54, 4, 1, 3);
            L.block(66, 0, 1, 3);
            break;
        case GameMode::Ufo:
            L.block(6, 7, len, 1);
            L.block(20, 0, 1, 2);
            L.block(34, 0, 1, 3);
            L.block(48, 5, 1, 2);
            L.block(62, 0, 1, 3);
            break;
        case GameMode::Ball:
        case GameMode::Swing:
        case GameMode::Spider:
            L.block(6, 5, len, 1); // ceiling (y 150..180)
            for (float b = 16; b < 80; b += 14) L.spike(b);          // floor spikes
            for (float b = 23; b < 80; b += 14) L.spike(b, 4.f);    // ceiling-side spikes (hang near the top)
            break;
        default:
            break;
    }
    L.finish(len);
    return L;
}

} // namespace sim
