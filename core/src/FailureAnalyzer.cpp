#include <aigd/FailureAnalyzer.hpp>

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>

namespace aigd {

std::vector<PressBlock> findPressBlocks(std::vector<Step> const& steps, float endX, int32_t endTick) {
    std::vector<PressBlock> blocks;
    bool open = false;
    PressBlock cur;
    for (size_t i = 0; i < steps.size(); ++i) {
        auto const& s = steps[i];
        if (s.action == 1 && !open) {
            open = true;
            cur = PressBlock{};
            cur.firstStep = i;
            cur.startX = s.x;
            cur.startTick = s.tick;
        } else if (s.action == 0 && open) {
            open = false;
            cur.lastStep = i - 1;
            cur.endX = s.x;
            cur.endTick = s.tick;
            blocks.push_back(cur);
        }
    }
    if (open) {
        cur.lastStep = steps.size() - 1;
        cur.endX = endX;
        cur.endTick = endTick;
        cur.openAtEnd = true;
        blocks.push_back(cur);
    }
    for (auto& b : blocks) {
        bool anyGround = false;
        bool groundMode = false;
        bool prevGround = false;
        for (size_t i = b.firstStep; i <= b.lastStep && i < steps.size(); ++i) {
            groundMode |= needsGroundToAct(steps[i].mode);
            anyGround |= steps[i].onGround;
            if (steps[i].onGround && !prevGround) ++b.groundContacts;
            prevGround = steps[i].onGround;
        }
        b.effective = !groundMode || anyGround;
    }
    return blocks;
}

float analysisWindow(int speedClass, uint32_t escalation) {
    float blocks = 5.f + 3.f * static_cast<float>(std::min<uint32_t>(escalation, 5));
    float scale = unitsPerSecond(speedClass) / unitsPerSecond(1);
    return std::min(blocks, 20.f) * kBlock * scale;
}

FailureReport analyzeFailure(
    std::vector<Step> const& steps,
    Observation const& last,
    DeathInfo const& death,
    uint32_t escalation
) {
    FailureReport rep;
    rep.mode = last.mode;
    rep.deathX = std::max(death.x, last.x);
    rep.deathY = death.hasKiller ? death.y : last.y;
    rep.deathProgress = last.levelLength > 1.f ? std::clamp(rep.deathX / last.levelLength, 0.f, 1.f) : 0.f;
    rep.hotspot = static_cast<int32_t>(std::floor(rep.deathX / kBlock));
    rep.windowStartX = rep.deathX - analysisWindow(last.speedClass(), escalation);
    rep.blocks = findPressBlocks(steps, rep.deathX, last.tick);

    // Press blocks that started inside the analysis window.
    std::vector<int> relevant;
    for (int i = 0; i < static_cast<int>(rep.blocks.size()); ++i) {
        auto const& b = rep.blocks[i];
        if (b.effective && b.startX >= rep.windowStartX && b.startX <= rep.deathX) relevant.push_back(i);
    }

    // Where was the thing that killed us? Prefer what the game reported; fall back
    // to the nearest hazard the observer saw.
    float killerDy = 0.f;
    bool haveKiller = false;
    if (death.hasKiller) {
        killerDy = death.killerDy;
        haveKiller = true;
    } else if (last.hazard.present && last.hazard.dx < kBlock) {
        killerDy = last.hazard.dy;
        haveKiller = true;
    }

    // Which way would have avoided the obstacle? +1 = should have been higher,
    // -1 = should have been lower, 0 = unknown.
    int needDir = 0;
    if (death.hasKiller && death.hasExtent) {
        if (death.killerBottom > 0.f) needDir = -1;        // entirely above us
        else if (death.killerTop < 0.f) needDir = +1;      // entirely below us
        else needDir = 0; // straddles our height: over or under is ambiguous (floor/ceiling unknown)
    } else if (haveKiller) {
        if (killerDy > 8.f) needDir = -1;
        else if (killerDy < -8.f) needDir = +1;
    }

    float vyN = last.vyNorm();
    // Escalation shifts suspicion to earlier presses: the cause of a death is not
    // necessarily the last thing the AI did.
    auto pickBlock = [&](int fromEnd) -> int {
        if (relevant.empty()) return -1;
        int idx = static_cast<int>(relevant.size()) - 1 - fromEnd;
        idx = std::max(idx, 0);
        return relevant[idx];
    };
    int esc = static_cast<int>(std::min<uint32_t>(escalation, 3));

    if (isFlyingMode(last.mode)) {
        bool above = haveKiller ? needDir < 0 : vyN > 0.f;
        bool below = haveKiller ? needDir > 0 : vyN < 0.f;
        if (above) {
            rep.kind = FailureKind::HeldTooLong;
            rep.responsibleBlock = pickBlock(esc);
        } else if (below) {
            rep.kind = FailureKind::ReleasedTooLong;
            rep.responsibleBlock = pickBlock(esc);
        } else {
            rep.kind = FailureKind::HitWall;
            rep.responsibleBlock = pickBlock(esc);
        }
    } else {
        int lastIdx = pickBlock(0);
        int32_t sincePress = lastIdx >= 0 ? last.tick - rep.blocks[lastIdx].startTick : INT_MAX;
        // ~0.45 s is a full cube jump; after that the jump has resolved.
        constexpr int32_t kAirtimeTicks = 108;
        bool recentPress = lastIdx >= 0 && sincePress <= kAirtimeTicks + 24;
        bool landedSincePress = last.onGround && sincePress > kAirtimeTicks;

        bool rejumped = lastIdx >= 0 && rep.blocks[lastIdx].groundContacts > 1 &&
                        (rep.blocks[lastIdx].openAtEnd || rep.blocks[lastIdx].endX > rep.deathX - kBlock * 3.f);
        if (rejumped && esc == 0) {
            // One long hold produced an extra jump on landing; that extra jump is
            // what went wrong, not the timing of the first press.
            rep.kind = FailureKind::HeldTooLong;
            rep.responsibleBlock = lastIdx;
        } else if (!recentPress || landedSincePress) {
            rep.kind = FailureKind::NoJump;
            rep.responsibleBlock = esc > 0 ? pickBlock(esc - 1) : -1;
        } else if (haveKiller && (death.hasExtent ? death.killerBottom > 0.f : killerDy > 20.f)) {
            rep.kind = FailureKind::HeldTooLong;
            rep.responsibleBlock = pickBlock(esc);
        } else if (!last.onGround && vyN > 0.5f) {
            rep.kind = FailureKind::TooLate;
            rep.responsibleBlock = pickBlock(esc);
        } else {
            rep.kind = FailureKind::TooEarly;
            rep.responsibleBlock = pickBlock(esc);
        }
    }

    char buf[256];
    char const* where = !haveKiller ? "" : (needDir < 0 ? " (obstacle above)" : (needDir > 0 ? " (obstacle below)" : " (obstacle ahead)"));
    std::snprintf(
        buf, sizeof(buf), "%s died at %.1f%%: %s%s%s",
        gameModeName(last.mode), rep.deathProgress * 100.f, failureKindName(rep.kind), where,
        esc > 0 ? ", suspecting an earlier input" : ""
    );
    rep.text = buf;
    return rep;
}

} // namespace aigd
