#include <aigd/Json.hpp>
#include <aigd/StateKey.hpp>
#include <aigd/Storage.hpp>

#include <algorithm>

namespace aigd::storage {

namespace {
using json::appendNumber;
using json::appendString;

void key(std::string& out, char const* k) {
    appendString(out, k);
    out += ':';
}
} // namespace

std::string serializeStrategies(StrategyLibrary const& lib) {
    std::string out;
    out.reserve(64 + lib.size() * 120);
    out += '{';
    key(out, "format"); appendString(out, "ai-geode-strategies"); out += ',';
    key(out, "version"); appendNumber(out, StrategyLibrary::kFormatVersion); out += ',';
    key(out, "clock"); appendNumber(out, lib.clock); out += ',';
    key(out, "levels");
    out += '[';
    for (size_t i = 0; i < lib.levels.size(); ++i) {
        if (i) out += ',';
        appendString(out, lib.levels[i]);
    }
    out += "],";
    key(out, "situations");
    out += '[';
    std::vector<uint64_t> keys;
    keys.reserve(lib.situations.size());
    for (auto const& [k, v] : lib.situations) keys.push_back(k);
    std::sort(keys.begin(), keys.end());
    bool firstSit = true;
    for (uint64_t k : keys) {
        auto const& recs = lib.situations.at(k);
        if (!firstSit) out += ',';
        firstSit = false;
        out += '[';
        appendNumber(out, static_cast<double>(k & kKeyMask));
        out += ",[";
        for (size_t i = 0; i < recs.size(); ++i) {
            StrategyRecord const& r = recs[i];
            if (i) out += ',';
            out += '[';
            for (double v : {static_cast<double>(r.primitive), static_cast<double>(r.leadBucket), static_cast<double>(r.successes),
                             static_cast<double>(r.failures), r.lead.sum, r.lead.sumSq, static_cast<double>(r.lead.count), r.second.sum,
                             r.second.sumSq, static_cast<double>(r.second.count), static_cast<double>(r.speedMask),
                             static_cast<double>(r.lastUse)}) {
                appendNumber(out, v);
                out += ',';
            }
            out += '[';
            for (size_t j = 0; j < r.levels.size(); ++j) {
                if (j) out += ',';
                out += '[';
                appendString(out, r.levels[j].level);
                out += ',';
                appendNumber(out, r.levels[j].successes);
                out += ',';
                appendNumber(out, r.levels[j].failures);
                out += ']';
            }
            out += "]]";
        }
        out += "]]";
    }
    out += "]}";
    return out;
}

bool deserializeStrategies(std::string_view text, StrategyLibrary& out, std::string* error) {
    auto doc = json::parse(text, error);
    if (!doc) return false;
    auto const& d = *doc;
    if (!d.isObject() || d.get("format").asString() != "ai-geode-strategies") {
        if (error) *error = "not an AI-Geode strategy file";
        return false;
    }
    if (d.get("version").asInt() > StrategyLibrary::kFormatVersion) {
        if (error) *error = "file was written by a newer version of the mod";
        return false;
    }
    StrategyLibrary L;
    L.clock = static_cast<uint32_t>(d.get("clock").asInt());
    for (auto const& v : d.get("levels").items())
        if (v.isString()) L.levels.push_back(v.asString());
    auto const& sits = d.get("situations");
    if (!sits.isArray()) {
        if (error) *error = "situations is not an array";
        return false;
    }
    for (auto const& row : sits.items()) {
        if (!row.isArray() || row.size() < 2 || !row.at(1).isArray()) {
            if (error) *error = "malformed situation row";
            return false;
        }
        double const k = row.at(0).asNumber(-1.0);
        if (k < 0) {
            if (error) *error = "malformed situation key";
            return false;
        }
        auto& recs = L.situations[static_cast<uint64_t>(k)];
        for (auto const& rr : row.at(1).items()) {
            if (!rr.isArray() || rr.size() < 13) {
                if (error) *error = "malformed strategy record";
                return false;
            }
            StrategyRecord r;
            int const prim = static_cast<int>(rr.at(0).asInt());
            if (prim < 0 || prim >= static_cast<int>(Primitive::Count)) continue;
            r.primitive = static_cast<Primitive>(prim);
            r.leadBucket = static_cast<int16_t>(rr.at(1).asInt(-1));
            r.successes = static_cast<uint32_t>(rr.at(2).asInt());
            r.failures = static_cast<uint32_t>(rr.at(3).asInt());
            r.lead.sum = rr.at(4).asNumber();
            r.lead.sumSq = rr.at(5).asNumber();
            r.lead.count = static_cast<uint32_t>(rr.at(6).asInt());
            r.second.sum = rr.at(7).asNumber();
            r.second.sumSq = rr.at(8).asNumber();
            r.second.count = static_cast<uint32_t>(rr.at(9).asInt());
            r.speedMask = static_cast<uint8_t>(rr.at(10).asInt());
            r.lastUse = static_cast<uint32_t>(rr.at(11).asInt());
            for (auto const& pv : rr.at(12).items()) {
                if (!pv.isArray() || pv.size() < 3 || r.levels.size() >= StrategyRecord::kMaxLevels) continue;
                r.levels.push_back({pv.at(0).asString(), static_cast<uint32_t>(pv.at(1).asInt()), static_cast<uint32_t>(pv.at(2).asInt())});
            }
            recs.push_back(std::move(r));
        }
    }
    out = std::move(L);
    return true;
}

std::string serializeKnowledge(PatternMemory const& patterns, StrategyLibrary const& lib) {
    std::string out = "{";
    key(out, "format"); appendString(out, "ai-geode-knowledge"); out += ',';
    key(out, "version"); appendNumber(out, 1); out += ',';
    key(out, "patterns"); out += serializePatterns(patterns); out += ',';
    key(out, "strategies"); out += serializeStrategies(lib);
    out += '}';
    return out;
}

bool deserializeKnowledge(std::string_view text, PatternMemory& patterns, StrategyLibrary& lib, std::string* error) {
    auto doc = json::parse(text, error);
    if (!doc) return false;
    auto const& d = *doc;
    if (!d.isObject() || d.get("format").asString() != "ai-geode-knowledge") {
        if (error) *error = "not an AI-Geode knowledge file";
        return false;
    }
    if (d.get("version").asInt() > 1) {
        if (error) *error = "file was written by a newer version of the mod";
        return false;
    }
    PatternMemory P;
    StrategyLibrary S;
    if (!deserializePatterns(json::write(d.get("patterns")), P, error)) return false;
    if (!deserializeStrategies(json::write(d.get("strategies")), S, error)) return false;
    patterns = std::move(P);
    lib = std::move(S);
    return true;
}

bool loadStrategyFile(std::filesystem::path const& path, StrategyLibrary& out, std::string* error) {
    std::string err;
    if (auto text = readFile(path, &err)) {
        if (deserializeStrategies(*text, out, &err)) return true;
    }
    std::filesystem::path bak = path;
    bak += ".bak";
    std::string bakErr;
    if (auto text = readFile(bak, &bakErr)) {
        if (deserializeStrategies(*text, out, &bakErr)) return true;
    }
    if (error) *error = err;
    return false;
}

} // namespace aigd::storage
