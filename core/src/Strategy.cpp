#include <aigd/Strategy.hpp>
#include <aigd/StateKey.hpp>

#include <algorithm>
#include <cmath>

namespace aigd {

char const* primitiveName(Primitive p) {
    switch (p) {
        case Primitive::Wait: return "no jump";
        case Primitive::Jump: return "jump";
        case Primitive::RejumpChain: return "jump and hold through the landing";
        case Primitive::DoubleJump: return "two jumps";
        case Primitive::LongJump: return "long jump";
        case Primitive::OrbActivate: return "orb activation";
        default: return "?";
    }
}

Primitive primitiveOfPlan(GameMode mode, int32_t pressAt, int32_t holdFor, bool rejump, int32_t secondAt) {
    if (pressAt < 0) return Primitive::Wait;
    if (secondAt >= 0) return Primitive::DoubleJump;
    if (rejump) return Primitive::RejumpChain;
    if (mode == GameMode::Robot && (holdFor < 0 || holdFor >= 12)) return Primitive::LongJump;
    return Primitive::Jump;
}

Provenance const* StrategyRecord::on(std::string const& level) const {
    for (auto const& p : levels)
        if (p.level == level) return &p;
    return nullptr;
}

int StrategyRecord::levelsWithSuccess() const {
    int n = 0;
    for (auto const& p : levels) n += p.successes > 0 ? 1 : 0;
    return n;
}

int StrategyRecord::levelsWithSuccessExcept(std::string const& level) const {
    int n = 0;
    for (auto const& p : levels) n += (p.successes > 0 && p.level != level) ? 1 : 0;
    return n;
}

double strategyRelevance(StrategyRecord const& r, std::string const& level, bool* rejectedHere) {
    double rel = r.successRate();
    uint32_t localS = 0, localF = 0;
    if (auto const* p = r.on(level)) {
        localS = p->successes;
        localF = p->failures;
    }
    // One net failure here may have had another cause; from the second on,
    // every further one halves the relevance on this level.
    int const excess = static_cast<int>(localF) - static_cast<int>(localS);
    bool const rejected = excess >= 2;
    if (excess >= 2) rel *= std::pow(0.5, excess - 1);
    if (rejectedHere) *rejectedHere = rejected;
    return rel;
}

size_t StrategyLibrary::records() const {
    size_t n = 0;
    for (auto const& [k, v] : situations) n += v.size();
    return n;
}

void StrategyLibrary::recordInto(uint64_t key, int speed, Primitive p, float leadSteps, float secondSteps, bool success,
                                 std::string const& level, float margin) {
    auto& recs = situations[key];
    int16_t const bucket =
        p == Primitive::Wait || leadSteps < 0.f
            ? int16_t(-1)
            : static_cast<int16_t>(std::clamp(static_cast<int>(std::lround(leadSteps / StrategyRecord::kLeadBucket)), 0, 1000));
    StrategyRecord* r = nullptr;
    for (auto& x : recs)
        if (x.primitive == p && x.leadBucket == bucket) r = &x;
    if (!r) {
        recs.push_back({});
        r = &recs.back();
        r->primitive = p;
        r->leadBucket = bucket;
    }
    if (success) {
        ++r->successes;
        if (leadSteps >= 0.f) r->lead.add(leadSteps);
        if (secondSteps >= 0.f) r->second.add(secondSteps);
        if (margin >= 0.f) r->margin.add(margin);
        if (speed >= 0 && speed < 8) r->speedMask |= static_cast<uint8_t>(1u << speed);
    } else {
        ++r->failures;
    }
    r->lastUse = clock;
    Provenance* pv = nullptr;
    for (auto& x : r->levels)
        if (x.level == level) pv = &x;
    if (!pv) {
        if (r->levels.size() >= StrategyRecord::kMaxLevels) {
            // keep the levels with the most evidence
            auto weakest = std::min_element(r->levels.begin(), r->levels.end(), [](Provenance const& a, Provenance const& b) {
                return a.successes + a.failures < b.successes + b.failures;
            });
            *weakest = Provenance{};
            pv = &*weakest;
        } else {
            r->levels.push_back({});
            pv = &r->levels.back();
        }
        pv->level = level;
    }
    if (success) ++pv->successes;
    else ++pv->failures;
}

void StrategyLibrary::record(Situation const& s, Primitive p, float leadSteps, float secondSteps, bool success, std::string const& level,
                             float margin) {
    if (!s.valid) return;
    ++clock;
    if (s.sequence) recordInto(s.sequence, s.speedClass, p, leadSteps, secondSteps, success, level, margin);
    recordInto(s.fine, s.speedClass, p, leadSteps, secondSteps, success, level, margin);
    recordInto(s.coarse, s.speedClass, p, leadSteps, secondSteps, success, level, margin);
    if (std::find(levels.begin(), levels.end(), level) == levels.end() && levels.size() < 256) levels.push_back(level);
    if (situations.size() > kMaxSituations) prune(kMaxSituations * 9 / 10);
}

bool StrategyLibrary::best(uint64_t key, int speed, std::string const& level, StrategyChoice& out) const {
    auto it = situations.find(key);
    if (it == situations.end()) return false;
    StrategyRecord const* bestRec = nullptr;
    double bestRel = -1.0;
    bool anyRejected = false;
    uint32_t rejectedFailures = 0;
    for (auto const& r : it->second) {
        if (r.successes < 2) continue; // not enough evidence that it works at all
        bool rejected = false;
        double rel = strategyRelevance(r, level, &rejected);
        if (rejected) {
            anyRejected = true;
            if (auto const* p = r.on(level)) rejectedFailures = std::max(rejectedFailures, p->failures);
            continue;
        }
        // Among timings that work about as reliably, the one flown with more
        // margin is preferred (v0.7.0): + up to 0.05 for 18 units of clearance.
        double const marginBonus = r.margin.count ? 0.05 * std::min(r.margin.mean(), 18.0) / 18.0 : 0.0;
        double const key = rel + marginBonus;
        double const bestKey = bestRec ? bestRel + (bestRec->margin.count ? 0.05 * std::min(bestRec->margin.mean(), 18.0) / 18.0 : 0.0) : -1.0;
        if (!bestRec || key > bestKey || (key == bestKey && r.successes > bestRec->successes)) {
            bestRel = rel;
            bestRec = &r;
        }
    }
    if (!bestRec) {
        if (anyRejected) {
            out.rejectedHere = true;
            out.localFailures = rejectedFailures;
        }
        return anyRejected;
    }
    out.primitive = bestRec->primitive;
    out.leadSteps = bestRec->lead.count ? static_cast<float>(bestRec->lead.mean()) : -1.f;
    out.secondSteps = bestRec->second.count ? static_cast<float>(bestRec->second.mean()) : -1.f;
    out.speedAdapted = speed >= 0 && speed < 8 && !(bestRec->speedMask & (1u << speed));
    out.learnedSpeed = -1;
    for (int k = 0; k < 8; ++k)
        if (bestRec->speedMask & (1u << k)) {
            out.learnedSpeed = k;
            break;
        }
    out.successes = bestRec->successes;
    out.failures = bestRec->failures;
    out.otherLevels = bestRec->levelsWithSuccessExcept(level);
    auto const* here = bestRec->on(level);
    out.learnedHere = here && here->successes > 0;
    out.localFailures = here ? here->failures : 0;
    out.relevance = bestRel;
    out.margin = bestRec->margin.count ? static_cast<float>(bestRec->margin.mean()) : -1.f;
    out.usable = bestRel >= kMinRelevance;
    out.rejectedHere = false;
    return true;
}

StrategyChoice StrategyLibrary::choose(Situation const& s, std::string const& level) const {
    StrategyChoice c;
    if (!s.valid) return c;
    if (s.sequence) {
        StrategyChoice seq;
        if (best(s.sequence, s.speedClass, level, seq) && seq.usable) {
            seq.match = StrategyChoice::Match::Exact;
            seq.sequence = true;
            return seq;
        }
    }
    StrategyChoice exact;
    if (best(s.fine, s.speedClass, level, exact)) {
        exact.match = StrategyChoice::Match::Exact;
        if (exact.usable) return exact;
        c = exact;
    }
    bool landingDiffers = false;
    for (int b = 0; b < 4 && !landingDiffers; ++b) {
        if (b == s.landingBand) continue;
        landingDiffers = situations.count(situationKeyWithLanding(s, b)) > 0;
    }
    StrategyChoice similar;
    if (best(s.coarse, s.speedClass, level, similar)) {
        similar.match = StrategyChoice::Match::Similar;
        similar.landingDiffers = landingDiffers;
        if (similar.usable || c.match == StrategyChoice::Match::None) return similar;
    }
    c.landingDiffers = landingDiffers;
    return c;
}

void StrategyLibrary::prune(size_t keep) {
    if (situations.size() <= keep) return;
    struct Row {
        uint64_t key;
        uint32_t evidence;
        uint32_t lastUse;
    };
    std::vector<Row> rows;
    rows.reserve(situations.size());
    for (auto const& [k, v] : situations) {
        Row r{k, 0, 0};
        for (auto const& x : v) {
            r.evidence += x.successes + x.failures;
            r.lastUse = std::max(r.lastUse, x.lastUse);
        }
        rows.push_back(r);
    }
    std::sort(rows.begin(), rows.end(), [](Row const& a, Row const& b) {
        return a.evidence != b.evidence ? a.evidence < b.evidence : a.lastUse < b.lastUse;
    });
    for (size_t i = 0; i < rows.size() && situations.size() > keep; ++i) situations.erase(rows[i].key);
}

} // namespace aigd
