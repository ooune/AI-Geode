#pragma once

// A small deterministic Geometry-Dash-like simulator used to test the learning
// core without the game. It is NOT a faithful GD physics engine: it models the
// parts that matter for learning (fixed 240 Hz steps, constant horizontal
// speed, cube jumps with hold-to-rejump, ship flight, spikes and solid blocks)
// with roughly GD-like proportions (30-unit blocks, ~2 block jump height,
// ~4 block jump length at 1x speed).

#include <aigd/Types.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace sim {

using aigd::DeathInfo;
using aigd::Feature;
using aigd::GameMode;
using aigd::KillerKind;
using aigd::Observation;

enum class ObjKind { Spike, Block, ShipPortal, CubePortal };

struct Obj {
    ObjKind kind;
    float x, y, w, h; // bottom-left corner and size
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
    void shipPortal(float bx) { objs.push_back({ObjKind::ShipPortal, bx * 30.f, 0.f, 30.f, 600.f}); }
    void cubePortal(float bx) { objs.push_back({ObjKind::CubePortal, bx * 30.f, 0.f, 30.f, 600.f}); }
    void finish(float blocks) {
        length = blocks * 30.f;
        std::sort(objs.begin(), objs.end(), [](Obj const& a, Obj const& b) { return a.x < b.x; });
    }
};

struct Rect {
    float l, b, r, t;
    bool overlaps(Rect const& o) const { return l < o.r && r > o.l && b < o.t && t > o.b; }
};

class World {
public:
    static constexpr float kHalf = 15.f;          // player half size
    static constexpr float kVyScale = 4.3f;       // GD-like vy units -> units per tick
    static constexpr float kJumpVy = 11.f;
    static constexpr float kGravity = 0.22f;      // vy units per tick
    static constexpr float kShipUp = 0.12f;
    static constexpr float kShipDown = 0.10f;
    static constexpr float kShipMax = 8.f;

    explicit World(Level const& level, int speedClass = 1) : m_level(level), m_speedClass(speedClass) { reset(); }

    void reset(float startX = 0.f) {
        m_x = startX + kHalf;
        m_y = kHalf;
        m_vy = 0.f;
        m_mode = GameMode::Cube;
        m_onGround = true;
        m_dead = false;
        m_complete = false;
        m_tick = 0;
        m_death = DeathInfo{};
        for (auto const& o : m_level.objs) {
            if (o.x + o.w > startX) break;
            if (o.kind == ObjKind::ShipPortal) m_mode = GameMode::Ship;
            if (o.kind == ObjKind::CubePortal) m_mode = GameMode::Cube;
        }
    }

    Observation observe() const {
        Observation o;
        o.tick = m_tick;
        o.x = m_x;
        o.y = m_y;
        o.vy = m_vy;
        o.mode = m_mode;
        o.onGround = m_onGround;
        o.dead = m_dead;
        o.playerSpeed = kSpeeds[m_speedClass];
        o.levelLength = m_level.length;
        o.hazard = nearest(ObjKind::Spike);
        o.solid = nearest(ObjKind::Block);
        return o;
    }

    // Advances one physics step with the button held or released.
    void step(bool hold) {
        if (m_dead || m_complete) return;
        ++m_tick;
        float vx = aigd::unitsPerSecond(m_speedClass) / aigd::kTicksPerSecond;
        float prevBottom = m_y - kHalf;
        float prevTop = m_y + kHalf;

        if (m_mode == GameMode::Cube) {
            if (m_onGround && hold) {
                m_vy = kJumpVy;
                m_onGround = false;
            }
            if (!m_onGround) m_vy -= kGravity;
        } else {
            m_vy += hold ? kShipUp : -kShipDown;
            m_vy = std::clamp(m_vy, -kShipMax, kShipMax);
        }
        m_x += vx;
        m_y += m_vy / kVyScale;

        // ground
        bool grounded = false;
        if (m_y - kHalf <= 0.f) {
            m_y = kHalf;
            if (m_vy < 0.f) m_vy = 0.f;
            grounded = true;
        }

        Rect player{m_x - kHalf, m_y - kHalf, m_x + kHalf, m_y + kHalf};
        for (auto const& o : m_level.objs) {
            if (o.x > m_x + 60.f) break;
            if (o.x + o.w < m_x - 60.f) continue;
            if (o.kind == ObjKind::ShipPortal || o.kind == ObjKind::CubePortal) {
                if (m_x >= o.x && m_x <= o.x + o.w) {
                    m_mode = o.kind == ObjKind::ShipPortal ? GameMode::Ship : GameMode::Cube;
                }
                continue;
            }
            if (o.kind == ObjKind::Spike) {
                Rect hb{o.x + 10.5f, o.y + 3.f, o.x + 19.5f, o.y + 18.f};
                if (player.overlaps(hb)) {
                    die(o, KillerKind::Hazard);
                    return;
                }
                continue;
            }
            Rect blk{o.x, o.y, o.x + o.w, o.y + o.h};
            if (!player.overlaps(blk)) continue;
            if (prevBottom >= blk.t - 2.f && m_vy <= 0.f) {
                // landed on top
                m_y = blk.t + kHalf;
                m_vy = 0.f;
                grounded = true;
                player = Rect{m_x - kHalf, m_y - kHalf, m_x + kHalf, m_y + kHalf};
            } else if (m_mode == GameMode::Ship && prevTop <= blk.b + 2.f && m_vy >= 0.f) {
                // ship slides along ceilings
                m_y = blk.b - kHalf;
                m_vy = 0.f;
                player = Rect{m_x - kHalf, m_y - kHalf, m_x + kHalf, m_y + kHalf};
            } else {
                die(o, KillerKind::Solid);
                return;
            }
        }

        // still supported? (walking off the edge of a block)
        if (!grounded && m_mode == GameMode::Cube && m_vy <= 0.f) {
            grounded = supported();
            if (grounded) m_vy = 0.f;
        }
        m_onGround = grounded;
        if (m_mode == GameMode::Cube && m_onGround && hold) {
            // GD 2.2: holding the button re-jumps immediately on landing.
            m_vy = kJumpVy;
            m_onGround = false;
        }

        if (m_x >= m_level.length) m_complete = true;
    }

    bool dead() const { return m_dead; }
    bool complete() const { return m_complete; }
    DeathInfo const& death() const { return m_death; }
    float x() const { return m_x; }

private:
    static constexpr float kSpeeds[5] = {0.7f, 0.9f, 1.1f, 1.3f, 1.6f};

    Level const& m_level;
    int m_speedClass;
    float m_x = 0, m_y = 0, m_vy = 0;
    GameMode m_mode = GameMode::Cube;
    bool m_onGround = true;
    bool m_dead = false;
    bool m_complete = false;
    int32_t m_tick = 0;
    DeathInfo m_death;

    bool supported() const {
        if (m_y - kHalf <= 0.5f) return true;
        for (auto const& o : m_level.objs) {
            if (o.kind != ObjKind::Block) continue;
            if (o.x > m_x + kHalf) break;
            if (o.x + o.w <= m_x - kHalf) continue;
            if (std::fabs((o.y + o.h) - (m_y - kHalf)) < 0.75f) return true;
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
        float cy = o.kind == ObjKind::Spike ? o.y + 10.f : o.y + o.h * 0.5f;
        m_death.killerDy = cy - m_y;
        m_death.hasExtent = true;
        float bottom = o.kind == ObjKind::Spike ? o.y + 3.f : o.y;
        float top = o.kind == ObjKind::Spike ? o.y + 18.f : o.y + o.h;
        m_death.killerTop = top - m_y;
        m_death.killerBottom = bottom - m_y;
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
            float dy = cy - m_y;
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

} // namespace sim
