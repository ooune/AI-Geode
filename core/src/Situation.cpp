#include <aigd/Situation.hpp>
#include <aigd/StateKey.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace aigd {

namespace {

constexpr float kHalfBlock = kBlock * 0.5f;
// Obstacles on the path closer than this to the previous one belong to the
// same group (one jump, or one sequence of jumps without a stop between).
constexpr float kJoinGap = 2.f * kBlock;

int8_t halfBlocks(float v) {
    return static_cast<int8_t>(std::clamp(static_cast<int>(std::lround(v / kHalfBlock)), -60, 60));
}

int landingBandOf(float space) {
    if (space < kBlock) return 0;
    if (space < 2.f * kBlock) return 1;
    if (space < 4.f * kBlock) return 2;
    return 3;
}

uint64_t fnv(uint64_t h, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        h ^= (v >> (8 * i)) & 0xFF;
        h *= 1099511628211ull;
    }
    return h;
}

uint64_t contextHash(Situation const& s) {
    uint64_t h = 1469598103934665603ull;
    h = fnv(h, static_cast<uint64_t>(s.mode));
    h = fnv(h, (s.mini ? 1u : 0u) | (s.upsideDown ? 2u : 0u) | (s.grounded ? 4u : 0u) | (s.unmodelled ? 8u : 0u));
    return h;
}

uint64_t elementHash(uint64_t h, SituationElement const& e) {
    uint64_t v = static_cast<uint64_t>(e.kind) | (static_cast<uint64_t>(e.orb) << 40) | (static_cast<uint64_t>(static_cast<uint8_t>(e.start)) << 8) |
                 (static_cast<uint64_t>(static_cast<uint8_t>(e.width)) << 16) |
                 (static_cast<uint64_t>(static_cast<uint8_t>(e.bottom)) << 24) |
                 (static_cast<uint64_t>(static_cast<uint8_t>(e.top)) << 32);
    return fnv(h, v);
}

uint64_t fineKey(Situation const& s, int landingBand) {
    uint64_t h = fnv(contextHash(s), 0x46u); // 'F'
    h = fnv(h, static_cast<uint64_t>(s.count) | (s.truncated ? 0x100u : 0u));
    for (int i = 0; i < std::min(s.count, Situation::kMaxElements); ++i) h = elementHash(h, s.el[i]);
    h = fnv(h, static_cast<uint64_t>(landingBand));
    return h & kKeyMask;
}

uint64_t coarseKey(Situation const& s) {
    // The first element's kind and height (whole blocks), its width class, and
    // how many elements follow (1, 2, 3+).
    uint64_t h = fnv(contextHash(s), 0x43u); // 'C'
    SituationElement const& e = s.el[0];
    int const topBlocks = static_cast<int>(std::lround(e.top / 2.f));
    int const widthClass = e.width <= 2 ? 0 : e.width <= 4 ? 1 : 2;
    h = fnv(h, static_cast<uint64_t>(e.kind) | (static_cast<uint64_t>(topBlocks + 64) << 8) | (static_cast<uint64_t>(widthClass) << 16) |
                   (static_cast<uint64_t>(std::min(s.count, 3)) << 24));
    return (h & kKeyMask) | (uint64_t(1) << 52); // coarse keys never collide with fine ones
}

} // namespace

uint64_t situationKeyWithLanding(Situation const& s, int landingBand) { return fineKey(s, landingBand); }

Situation describeSituation(Observation const& obs, float lookAheadSteps) {
    Situation s;
    s.mode = obs.mode;
    s.mini = obs.mini;
    s.upsideDown = obs.upsideDown;
    s.grounded = obs.onGround;
    s.speedClass = obs.speedClass();
    if (obs.mode != GameMode::Cube && obs.mode != GameMode::Robot) return s;
    if (obs.obstacles.empty()) return s;

    float const h = situationHalfSize(obs.mini);
    float const vx = unitsPerSecond(s.speedClass) / static_cast<float>(kTicksPerSecond);
    float const front = obs.x + h;
    float const reach = front + lookAheadSteps * vx;
    // Gravity-normalised height above the player's feet.
    float const feet = obs.upsideDown ? obs.y + h : obs.y - h;
    auto up = [&](float v) { return obs.upsideDown ? feet - v : v - feet; };
    float const bodyTop = 2.f * h - 2.f;      // a thing lower than this is on the walking path
    float const aboveLimit = 4.f * kBlock;    // things higher than this above the feet do not matter

    struct Item {
        float l, r, lo, hi;
        uint8_t kind;
    };
    Item path[24];
    int nPath = 0;
    Item above[16];
    int nAbove = 0;
    float firstUnmodelled = 1e30f;
    for (auto const& o : obs.obstacles) {
        if (o.r <= obs.x - h || o.l > reach + 6.f * kBlock) continue;
        float lo = up(o.b), hi = up(o.t);
        if (lo > hi) std::swap(lo, hi);
        if (o.kind == Obstacle::Kind::Uncertain) {
            if (o.l >= obs.x - h) firstUnmodelled = std::min(firstUnmodelled, o.l);
            continue;
        }
        if (hi <= 1.f) continue; // the floor (or below it)
        bool const hazard = o.kind == Obstacle::Kind::Hazard;
        if (lo < bodyTop) {
            if (nPath < 24) path[nPath++] = {o.l, o.r, lo, hi, static_cast<uint8_t>(hazard ? SituationElement::Hazard : SituationElement::Solid)};
        } else if (lo < aboveLimit) {
            if (nAbove < 16)
                above[nAbove++] = {o.l, o.r, lo, hi, static_cast<uint8_t>(hazard ? SituationElement::HazardAbove : SituationElement::SolidAbove)};
        }
    }
    auto byLeft = [](Item const& a, Item const& b) { return a.l < b.l; };
    std::sort(path, path + nPath, byLeft);
    std::sort(above, above + nAbove, byLeft);

    // The group: the first thing on the path ahead (not behind the player's
    // front edge), plus whatever follows it within kJoinGap. Adjacent blocks
    // of the same height are one element (a platform), whatever the level
    // happened to build it from.
    int first = -1;
    for (int i = 0; i < nPath; ++i) {
        if (path[i].r > front - 1.f) {
            first = i;
            break;
        }
    }
    if (first < 0 || path[first].l > reach) return s;

    Item merged[Situation::kMaxElements + 1];
    int nMerged = 0;
    float groupEnd = path[first].r;
    int next = nPath;
    for (int i = first; i < nPath; ++i) {
        Item const& it = path[i];
        if (it.l > groupEnd + kJoinGap) {
            next = i;
            break;
        }
        groupEnd = std::max(groupEnd, it.r);
        if (nMerged > 0) {
            Item& last = merged[nMerged - 1];
            if (last.kind == it.kind && it.l <= last.r + 1.f && std::fabs(last.hi - it.hi) < 1.f && std::fabs(last.lo - it.lo) < 1.f) {
                last.r = std::max(last.r, it.r);
                continue;
            }
        }
        if (nMerged < Situation::kMaxElements + 1) merged[nMerged++] = it;
    }
    float const groupStart = path[first].l;
    // Orbs in the group or up to 3 blocks before it (what the button can do there).
    struct OrbItem {
        float l, r, lo, hi;
        uint8_t type;
    };
    OrbItem orbItems[4];
    int nOrbs = 0;
    for (auto const& o : obs.orbs) {
        if (o.used || o.r < groupStart - 3.f * kBlock || o.l > groupEnd + kBlock || nOrbs >= 4) continue;
        float lo = up(o.b), hi = up(o.t);
        if (lo > hi) std::swap(lo, hi);
        if (hi < -kBlock || lo > 6.f * kBlock) continue;
        orbItems[nOrbs++] = {o.l, o.r, lo, hi, static_cast<uint8_t>(o.type)};
    }
    // Things above the path over the group (and a block before and after it).
    for (int i = 0; i < nAbove && nMerged < Situation::kMaxElements + 1; ++i) {
        Item const& it = above[i];
        if (it.r < groupStart - kBlock || it.l > groupEnd + kBlock) continue;
        merged[nMerged++] = it;
    }

    s.valid = true;
    // orbs take precedence over things above the path (they change what the button does)
    int const pathItems = std::min(nMerged, Situation::kMaxElements);
    int nPathFirst = 0;
    for (int i = 0; i < pathItems; ++i) nPathFirst += (merged[i].kind == SituationElement::Hazard || merged[i].kind == SituationElement::Solid) ? 1 : 0;
    if (nOrbs > 0) {
        s.orbs = nOrbs;
        s.firstOrb = static_cast<OrbType>(orbItems[0].type);
        s.orbX = orbItems[0].l;
        int slot = std::min(nPathFirst, Situation::kMaxElements - 1);
        for (int k = 0; k < nOrbs && slot < Situation::kMaxElements; ++k, ++slot) {
            Item it{orbItems[k].l, orbItems[k].r, orbItems[k].lo, orbItems[k].hi, static_cast<uint8_t>(SituationElement::OrbAt)};
            if (slot < nMerged) merged[slot] = it;
            else merged[nMerged++] = it;
        }
    }
    s.truncated = nMerged > Situation::kMaxElements;
    s.count = std::min(nMerged, Situation::kMaxElements);
    for (int i = 0; i < s.count; ++i) {
        Item const& it = merged[i];
        SituationElement& e = s.el[i];
        e.kind = it.kind;
        if (it.kind == SituationElement::OrbAt) {
            for (int k = 0; k < nOrbs; ++k)
                if (orbItems[k].l == it.l && orbItems[k].lo == it.lo) e.orb = orbItems[k].type;
        }
        e.start = halfBlocks(it.l - groupStart);
        e.width = halfBlocks(it.r - it.l);
        e.bottom = halfBlocks(it.lo);
        e.top = halfBlocks(it.hi);
    }
    s.startX = groupStart;
    s.endX = groupEnd;
    s.contactSteps = (groupStart - front) / vx;
    s.landingSpace = next < nPath ? path[next].l - groupEnd : 8.f * kBlock;
    s.landingBand = landingBandOf(s.landingSpace);
    s.unmodelled = firstUnmodelled < groupEnd + 2.f * kBlock;
    s.fine = fineKey(s, s.landingBand);
    s.coarse = coarseKey(s);
    {
        uint64_t q = fnv(s.fine, 0x53u); // 'S'
        if (next < nPath) {
            Item const& n = path[next];
            int const w = static_cast<int>(std::lround((n.r - n.l) / kHalfBlock));
            q = fnv(q, static_cast<uint64_t>(n.kind) | (static_cast<uint64_t>(static_cast<uint8_t>(halfBlocks(n.hi))) << 8) |
                           (static_cast<uint64_t>(w <= 2 ? 0 : w <= 4 ? 1 : 2) << 16));
        } else {
            q = fnv(q, 0xFFu);
        }
        s.sequence = (q & (kKeyMask >> 2)) | (uint64_t(1) << 51); // never equal to a fine or coarse key
    }
    return s;
}

std::string Situation::describe() const {
    if (!valid) return "nothing ahead";
    int hazards = 0, solids = 0, above = 0;
    for (int i = 0; i < count; ++i) {
        switch (el[i].kind) {
            case SituationElement::Hazard: ++hazards; break;
            case SituationElement::Solid: ++solids; break;
            case SituationElement::OrbAt: break;
            default: ++above; break;
        }
    }
    std::string d;
    char buf[96];
    if (hazards == 1 && solids == 0) d = "single spike";
    else if (hazards == 2 && solids == 0) d = "double spike";
    else if (hazards >= 3 && solids == 0) d = "spike row";
    else if (solids >= 1 && hazards == 0) {
        std::snprintf(buf, sizeof(buf), "%s %.1f blocks high", solids == 1 ? "block" : "blocks", el[0].top / 2.f);
        d = buf;
    } else if (solids >= 1 && hazards >= 1) d = "blocks with spikes";
    else d = "overhead obstacles";
    if (above > 0 && (hazards + solids) > 0) d += " under something";
    if (truncated) d += " (and more)";
    if (orbs > 0) d += std::string(" with ") + orbTypeName(firstOrb);
    if (unmodelled) d += ", orb/pad nearby";
    static char const* const kSpace[] = {"<1 block", "1-2 blocks", "2-4 blocks", "4+ blocks"};
    std::snprintf(buf, sizeof(buf), ", landing space %s", kSpace[std::clamp(landingBand, 0, 3)]);
    d += buf;
    return d;
}

char const* Situation::category() const {
    if (!valid) return "obstacle";
    bool hazard = false, solid = false;
    for (int i = 0; i < count; ++i) {
        if (el[i].kind == SituationElement::Hazard) hazard = true;
        if (el[i].kind == SituationElement::Solid) solid = true;
    }
    if (hazard && !solid) return "spike-clearing";
    if (solid && !hazard) return "block-climbing";
    if (solid && hazard) return "block-and-spike";
    return "overhead-obstacle";
}

} // namespace aigd
