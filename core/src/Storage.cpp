#include <aigd/Json.hpp>
#include <aigd/StateKey.hpp>
#include <aigd/Storage.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <system_error>

namespace aigd::storage {

namespace {
using json::appendNumber;
using json::appendString;

void key(std::string& out, char const* k) {
    appendString(out, k);
    out += ':';
}

void writeQTable(std::string& out, QTable const& table) {
    std::vector<std::pair<uint64_t, QEntry>> rows(table.begin(), table.end());
    std::sort(rows.begin(), rows.end(), [](auto const& a, auto const& b) { return a.first < b.first; });
    out += '[';
    bool first = true;
    for (auto const& [k, e] : rows) {
        if (!first) out += ',';
        first = false;
        out += '[';
        appendNumber(out, static_cast<double>(k & kKeyMask));
        out += ',';
        appendNumber(out, e.q[0]);
        out += ',';
        appendNumber(out, e.q[1]);
        out += ',';
        appendNumber(out, e.n[0]);
        out += ',';
        appendNumber(out, e.n[1]);
        out += ']';
    }
    out += ']';
}

bool readQTable(json::Value const& arr, QTable& table, std::string* error) {
    if (!arr.isArray()) {
        if (error) *error = "q table is not an array";
        return false;
    }
    table.reserve(arr.size());
    for (auto const& row : arr.items()) {
        if (!row.isArray() || row.size() < 5) {
            if (error) *error = "malformed q table row";
            return false;
        }
        double k = row.at(0).asNumber(-1.0);
        if (k < 0) {
            if (error) *error = "malformed q table key";
            return false;
        }
        QEntry& e = table.getOrCreate(static_cast<uint64_t>(k));
        e.q[0] = static_cast<float>(row.at(1).asNumber());
        e.q[1] = static_cast<float>(row.at(2).asNumber());
        e.n[0] = static_cast<uint16_t>(std::clamp<int64_t>(row.at(3).asInt(), 0, 0xFFFF));
        e.n[1] = static_cast<uint16_t>(std::clamp<int64_t>(row.at(4).asInt(), 0, 0xFFFF));
    }
    return true;
}

FailureKind toKind(int64_t v) {
    if (v < 0 || v >= static_cast<int64_t>(FailureKind::Count)) return FailureKind::Unknown;
    return static_cast<FailureKind>(v);
}
} // namespace

std::string serializeLevel(LevelMemory const& L) {
    std::string out;
    out.reserve(64 + L.q.size() * 40);
    out += '{';
    key(out, "format"); appendString(out, "ai-geode-level"); out += ',';
    key(out, "version"); appendNumber(out, LevelMemory::kFormatVersion); out += ',';
    key(out, "key"); appendString(out, L.key); out += ',';
    key(out, "name"); appendString(out, L.name); out += ',';
    key(out, "levelLength"); appendNumber(out, L.levelLength); out += ',';
    key(out, "totalAttempts"); appendNumber(out, L.totalAttempts); out += ',';
    key(out, "completions"); appendNumber(out, L.completions); out += ',';
    key(out, "deaths"); appendNumber(out, L.deaths); out += ',';
    key(out, "bestProgress"); appendNumber(out, L.bestProgress); out += ',';
    key(out, "bestX"); appendNumber(out, L.bestX); out += ',';
    key(out, "attemptsSinceBest"); appendNumber(out, L.attemptsSinceBest); out += ',';

    key(out, "history");
    out += '[';
    for (size_t i = 0; i < L.history.size(); ++i) {
        auto const& h = L.history[i];
        if (i) out += ',';
        out += '[';
        appendNumber(out, h.number); out += ',';
        appendNumber(out, h.progress); out += ',';
        appendNumber(out, h.endX); out += ',';
        appendNumber(out, static_cast<int>(h.kind)); out += ',';
        appendNumber(out, h.ticks); out += ',';
        out += h.completed ? "true" : "false"; out += ',';
        out += h.improved ? "true" : "false"; out += ',';
        out += h.fromCheckpoint ? "true" : "false"; out += ',';
        appendString(out, h.change);
        out += ']';
    }
    out += "],";

    key(out, "hotspots");
    out += '[';
    bool first = true;
    for (auto const& [bucket, h] : L.hotspots) {
        if (!first) out += ',';
        first = false;
        out += '[';
        appendNumber(out, bucket); out += ',';
        appendNumber(out, h.deaths); out += ',';
        appendNumber(out, h.passes); out += ',';
        appendNumber(out, static_cast<int>(h.lastKind)); out += ',';
        appendNumber(out, h.shift); out += ',';
        appendNumber(out, h.experiments); out += ',';
        appendNumber(out, h.escalation);
        out += ']';
    }
    out += "],";

    key(out, "bestRun");
    out += '[';
    for (size_t i = 0; i < L.bestRunInputs.size(); ++i) {
        if (i) out += ',';
        out += '[';
        appendNumber(out, L.bestRunInputs[i].first); out += ',';
        appendNumber(out, L.bestRunInputs[i].second);
        out += ']';
    }
    out += "],";

    key(out, "lastFailure");
    out += '[';
    for (size_t i = 0; i < L.lastFailureTrace.size(); ++i) {
        auto const& t = L.lastFailureTrace[i];
        if (i) out += ',';
        out += '[';
        appendNumber(out, t.x); out += ',';
        appendNumber(out, t.y); out += ',';
        appendNumber(out, t.vy); out += ',';
        appendNumber(out, t.mode); out += ',';
        out += t.onGround ? "true" : "false"; out += ',';
        appendNumber(out, t.action);
        out += ']';
    }
    out += "],";

    key(out, "q");
    writeQTable(out, L.q);
    out += '}';
    return out;
}

bool deserializeLevel(std::string_view text, LevelMemory& out, std::string* error) {
    auto doc = json::parse(text, error);
    if (!doc) return false;
    auto const& d = *doc;
    if (!d.isObject() || d.get("format").asString() != "ai-geode-level") {
        if (error) *error = "not an AI-Geode level file";
        return false;
    }
    if (d.get("version").asInt() > LevelMemory::kFormatVersion) {
        if (error) *error = "file was written by a newer version of the mod";
        return false;
    }

    LevelMemory L;
    L.key = d.get("key").asString();
    if (L.key.empty()) {
        if (error) *error = "missing level key";
        return false;
    }
    L.name = d.get("name").asString();
    L.levelLength = static_cast<float>(d.get("levelLength").asNumber());
    L.totalAttempts = static_cast<uint32_t>(d.get("totalAttempts").asInt());
    L.completions = static_cast<uint32_t>(d.get("completions").asInt());
    L.deaths = static_cast<uint32_t>(d.get("deaths").asInt());
    L.bestProgress = static_cast<float>(d.get("bestProgress").asNumber());
    L.bestX = static_cast<float>(d.get("bestX").asNumber());
    L.attemptsSinceBest = static_cast<uint32_t>(d.get("attemptsSinceBest").asInt());

    for (auto const& row : d.get("history").items()) {
        if (row.size() < 9) continue;
        AttemptRecord r;
        r.number = static_cast<uint32_t>(row.at(0).asInt());
        r.progress = static_cast<float>(row.at(1).asNumber());
        r.endX = static_cast<float>(row.at(2).asNumber());
        r.kind = toKind(row.at(3).asInt());
        r.ticks = static_cast<int32_t>(row.at(4).asInt());
        r.completed = row.at(5).asBool();
        r.improved = row.at(6).asBool();
        r.fromCheckpoint = row.at(7).asBool();
        r.change = row.at(8).asString();
        L.pushHistory(std::move(r));
    }
    for (auto const& row : d.get("hotspots").items()) {
        if (row.size() < 7) continue;
        Hotspot h;
        h.bucket = static_cast<int32_t>(row.at(0).asInt());
        h.deaths = static_cast<uint32_t>(row.at(1).asInt());
        h.passes = static_cast<uint32_t>(row.at(2).asInt());
        h.lastKind = toKind(row.at(3).asInt());
        h.shift = static_cast<int32_t>(row.at(4).asInt());
        h.experiments = static_cast<uint32_t>(row.at(5).asInt());
        h.escalation = static_cast<uint32_t>(row.at(6).asInt());
        L.hotspots[h.bucket] = h;
    }
    for (auto const& row : d.get("bestRun").items()) {
        if (row.size() < 2) continue;
        L.bestRunInputs.emplace_back(static_cast<float>(row.at(0).asNumber()), static_cast<uint8_t>(row.at(1).asInt()));
    }
    for (auto const& row : d.get("lastFailure").items()) {
        if (row.size() < 6) continue;
        TraceSample t;
        t.x = static_cast<float>(row.at(0).asNumber());
        t.y = static_cast<float>(row.at(1).asNumber());
        t.vy = static_cast<float>(row.at(2).asNumber());
        t.mode = static_cast<uint8_t>(row.at(3).asInt());
        t.onGround = row.at(4).asBool();
        t.action = static_cast<uint8_t>(row.at(5).asInt());
        L.lastFailureTrace.push_back(t);
    }
    if (!readQTable(d.get("q"), L.q, error)) return false;

    out = std::move(L);
    return true;
}

std::string serializePatterns(PatternMemory const& P) {
    std::string out;
    out.reserve(64 + P.q.size() * 40);
    out += '{';
    key(out, "format"); appendString(out, "ai-geode-patterns"); out += ',';
    key(out, "version"); appendNumber(out, PatternMemory::kFormatVersion); out += ',';
    key(out, "leads");
    out += '[';
    bool first = true;
    for (auto const& [k, s] : P.leads) {
        if (!first) out += ',';
        first = false;
        out += '[';
        appendNumber(out, k); out += ',';
        appendNumber(out, s.sum); out += ',';
        appendNumber(out, s.sumSq); out += ',';
        appendNumber(out, s.count);
        out += ']';
    }
    out += "],";
    key(out, "q");
    writeQTable(out, P.q);
    out += '}';
    return out;
}

bool deserializePatterns(std::string_view text, PatternMemory& out, std::string* error) {
    auto doc = json::parse(text, error);
    if (!doc) return false;
    auto const& d = *doc;
    if (!d.isObject() || d.get("format").asString() != "ai-geode-patterns") {
        if (error) *error = "not an AI-Geode pattern file";
        return false;
    }
    PatternMemory P;
    for (auto const& row : d.get("leads").items()) {
        if (row.size() < 4) continue;
        TimingStat s;
        s.sum = row.at(1).asNumber();
        s.sumSq = row.at(2).asNumber();
        s.count = static_cast<uint32_t>(row.at(3).asInt());
        P.leads[static_cast<uint32_t>(row.at(0).asInt())] = s;
    }
    if (!readQTable(d.get("q"), P.q, error)) return false;
    out = std::move(P);
    return true;
}

bool writeFileAtomic(std::filesystem::path const& path, std::string const& data, std::string* error) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (path.has_parent_path()) fs::create_directories(path.parent_path(), ec);

    fs::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) {
            if (error) *error = "cannot open " + tmp.string() + " for writing";
            return false;
        }
        f.write(data.data(), static_cast<std::streamsize>(data.size()));
        f.flush();
        if (!f) {
            if (error) *error = "write failed for " + tmp.string();
            return false;
        }
    }
    if (fs::exists(path, ec)) {
        fs::path bak = path;
        bak += ".bak";
        fs::copy_file(path, bak, fs::copy_options::overwrite_existing, ec);
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        if (error) *error = "rename failed: " + ec.message();
        fs::remove(tmp, ec);
        return false;
    }
    return true;
}

std::optional<std::string> readFile(std::filesystem::path const& path, std::string* error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "cannot open " + path.string();
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool loadLevelFile(std::filesystem::path const& path, LevelMemory& out, std::string* error) {
    std::string err;
    if (auto text = readFile(path, &err)) {
        if (deserializeLevel(*text, out, &err)) return true;
    }
    std::filesystem::path bak = path;
    bak += ".bak";
    std::string bakErr;
    if (auto text = readFile(bak, &bakErr)) {
        if (deserializeLevel(*text, out, &bakErr)) return true;
    }
    if (error) *error = err;
    return false;
}

bool loadPatternFile(std::filesystem::path const& path, PatternMemory& out, std::string* error) {
    std::string err;
    if (auto text = readFile(path, &err)) {
        if (deserializePatterns(*text, out, &err)) return true;
    }
    std::filesystem::path bak = path;
    bak += ".bak";
    std::string bakErr;
    if (auto text = readFile(bak, &bakErr)) {
        if (deserializePatterns(*text, out, &bakErr)) return true;
    }
    if (error) *error = err;
    return false;
}

std::string sanitizeFileName(std::string_view k) {
    std::string out;
    for (char c : k) {
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
        out += ok ? c : '_';
    }
    if (out.empty()) out = "level";
    if (out.size() > 80) out.resize(80);
    return out;
}

} // namespace aigd::storage
