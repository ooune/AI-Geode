#include <aigd/StateKey.hpp>
#include <aigd/Types.hpp>

#include <algorithm>
#include <cmath>

namespace aigd {

char const* gameModeName(GameMode mode) {
    switch (mode) {
        case GameMode::Cube: return "Cube";
        case GameMode::Ship: return "Ship";
        case GameMode::Ball: return "Ball";
        case GameMode::Ufo: return "UFO";
        case GameMode::Wave: return "Wave";
        case GameMode::Robot: return "Robot";
        case GameMode::Spider: return "Spider";
        case GameMode::Swing: return "Swing";
        default: return "?";
    }
}

int speedClassFromPlayerSpeed(float s) {
    // 0.7 (0.5x), 0.9 (1x), 1.1 (2x), 1.3 (3x), 1.6 (4x)
    static constexpr float kSpeeds[] = {0.7f, 0.9f, 1.1f, 1.3f, 1.6f};
    int best = 1;
    float bestDist = 1e9f;
    for (int i = 0; i < 5; ++i) {
        float d = std::fabs(kSpeeds[i] - s);
        if (d < bestDist) {
            bestDist = d;
            best = i;
        }
    }
    return best;
}

float unitsPerSecond(int speedClass) {
    static constexpr float kUps[] = {251.16f, 311.58f, 387.42f, 468.0f, 576.0f};
    return kUps[std::clamp(speedClass, 0, 4)];
}

float Observation::progress() const {
    if (levelLength <= 1.f) return 0.f;
    return std::clamp(x / levelLength, 0.f, 1.f);
}

char const* failureKindName(FailureKind kind) {
    switch (kind) {
        case FailureKind::None: return "none";
        case FailureKind::NoJump: return "no jump";
        case FailureKind::TooEarly: return "jumped too early";
        case FailureKind::TooLate: return "jumped too late";
        case FailureKind::HeldTooLong: return "held too long";
        case FailureKind::ReleasedTooLong: return "released too long";
        case FailureKind::HitWall: return "hit a wall";
        case FailureKind::Unknown: return "unclear";
        default: return "?";
    }
}

char const* agentPhaseName(AgentPhase phase) {
    switch (phase) {
        case AgentPhase::Idle: return "Idle";
        case AgentPhase::Observing: return "Observing";
        case AgentPhase::Playing: return "Playing";
        case AgentPhase::Learning: return "Learning";
        case AgentPhase::Retrying: return "Retrying";
        case AgentPhase::Completed: return "Completed";
        case AgentPhase::Disabled: return "Disabled";
        default: return "?";
    }
}

// ---------------------------------------------------------------------------
// State keys

namespace {
struct Packer {
    uint64_t value = 0;
    int shift = 0;
    void put(int64_t v, int bits) {
        uint64_t mask = (uint64_t(1) << bits) - 1;
        value |= (static_cast<uint64_t>(v) & mask) << shift;
        shift += bits;
    }
};

int64_t clampI(int64_t v, int64_t lo, int64_t hi) { return std::clamp(v, lo, hi); }

int64_t bucket(float v, float size) { return static_cast<int64_t>(std::floor(v / size)); }

void putFeature(Packer& p, Feature const& f) {
    p.put(f.present ? 1 : 0, 1);
    if (!f.present) {
        p.put(0, 5);
        p.put(0, 4);
        return;
    }
    // dx in half-block buckets from -0.5 to +8 blocks, dy in half blocks +-3.5 blocks
    p.put(clampI(bucket(f.dx + 15.f, 15.f), 0, 17), 5);
    p.put(clampI(bucket(f.dy, 15.f), -7, 7) + 7, 4);
}
} // namespace

int32_t xCell(float x, float cellSize) {
    return static_cast<int32_t>(std::floor(x / cellSize));
}

uint64_t encodeLevelKey(Observation const& obs, bool holding, float cellSize) {
    Packer p;
    p.put(clampI(xCell(obs.x, cellSize), 0, (1 << 20) - 1), 20);
    p.put(static_cast<int64_t>(obs.mode), 3);
    p.put(obs.onGround, 1);
    p.put(obs.upsideDown, 1);
    p.put(obs.mini, 1);
    p.put(holding, 1);
    p.put(clampI(bucket(obs.y, 10.f), 0, 1023), 10);
    p.put(clampI(static_cast<int64_t>(std::lround(obs.vy / 2.f)), -15, 15) + 15, 5);
    p.put(obs.speedClass(), 3);
    return p.value & kKeyMask;
}

uint64_t encodePatternKey(Observation const& obs, bool holding) {
    Packer p;
    p.put(static_cast<int64_t>(obs.mode), 3);
    p.put(obs.onGround, 1);
    p.put(obs.upsideDown, 1);
    p.put(obs.mini, 1);
    p.put(holding, 1);
    p.put(obs.speedClass(), 3);
    p.put(clampI(static_cast<int64_t>(std::lround(obs.vyNorm() / 4.f)), -3, 3) + 3, 3);
    putFeature(p, obs.hazard);
    putFeature(p, obs.solid);
    putFeature(p, obs.orb);
    return p.value & kKeyMask;
}

int32_t keyCell(uint64_t levelKey) {
    return static_cast<int32_t>(levelKey & ((1u << 20) - 1));
}

GameMode keyMode(uint64_t levelKey) {
    return static_cast<GameMode>((levelKey >> 20) & 0x7);
}

} // namespace aigd
