#include <aigd/Json.hpp>

#include <charconv>
#include <cmath>
#include <cstdio>

namespace aigd::json {

namespace {
Value const& nullValue() {
    static Value const v;
    return v;
}

class Parser {
public:
    explicit Parser(std::string_view text) : m_text(text) {}

    std::optional<Value> run(std::string* error) {
        skipWs();
        auto v = parseValue(0);
        if (v) {
            skipWs();
            if (m_pos != m_text.size()) {
                fail("trailing characters after document");
                v.reset();
            }
        }
        if (!v && error) *error = m_error + " at offset " + std::to_string(m_pos);
        return v;
    }

private:
    static constexpr int kMaxDepth = 64;

    std::string_view m_text;
    size_t m_pos = 0;
    std::string m_error;

    void fail(std::string msg) {
        if (m_error.empty()) m_error = std::move(msg);
    }

    void skipWs() {
        while (m_pos < m_text.size()) {
            char c = m_text[m_pos];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') ++m_pos;
            else break;
        }
    }

    bool consume(char c) {
        if (m_pos < m_text.size() && m_text[m_pos] == c) {
            ++m_pos;
            return true;
        }
        return false;
    }

    bool consumeLiteral(std::string_view lit) {
        if (m_text.substr(m_pos, lit.size()) == lit) {
            m_pos += lit.size();
            return true;
        }
        return false;
    }

    std::optional<Value> parseValue(int depth) {
        if (depth > kMaxDepth) {
            fail("document nested too deeply");
            return std::nullopt;
        }
        if (m_pos >= m_text.size()) {
            fail("unexpected end of input");
            return std::nullopt;
        }
        char c = m_text[m_pos];
        if (c == '{') return parseObject(depth);
        if (c == '[') return parseArray(depth);
        if (c == '"') {
            auto s = parseString();
            if (!s) return std::nullopt;
            return Value(std::move(*s));
        }
        if (consumeLiteral("true")) return Value(true);
        if (consumeLiteral("false")) return Value(false);
        if (consumeLiteral("null")) return Value(nullptr);
        if (c == '-' || (c >= '0' && c <= '9')) return parseNumber();
        fail(std::string("unexpected character '") + c + "'");
        return std::nullopt;
    }

    std::optional<Value> parseNumber() {
        size_t start = m_pos;
        if (m_text[m_pos] == '-') ++m_pos;
        while (m_pos < m_text.size()) {
            char c = m_text[m_pos];
            if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') ++m_pos;
            else break;
        }
        double d = 0.0;
        auto first = m_text.data() + start;
        auto last = m_text.data() + m_pos;
        auto res = std::from_chars(first, last, d);
        if (res.ec != std::errc() || res.ptr != last) {
            fail("invalid number");
            return std::nullopt;
        }
        return Value(d);
    }

    static void appendUtf8(std::string& out, uint32_t cp) {
        if (cp < 0x80) out += static_cast<char>(cp);
        else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    std::optional<uint32_t> parseHex4() {
        if (m_pos + 4 > m_text.size()) return std::nullopt;
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) {
            char c = m_text[m_pos++];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= static_cast<uint32_t>(c - '0');
            else if (c >= 'a' && c <= 'f') v |= static_cast<uint32_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= static_cast<uint32_t>(c - 'A' + 10);
            else return std::nullopt;
        }
        return v;
    }

    std::optional<std::string> parseString() {
        if (!consume('"')) {
            fail("expected string");
            return std::nullopt;
        }
        std::string out;
        while (m_pos < m_text.size()) {
            char c = m_text[m_pos++];
            if (c == '"') return out;
            if (c != '\\') {
                out += c;
                continue;
            }
            if (m_pos >= m_text.size()) break;
            char e = m_text[m_pos++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    auto cp = parseHex4();
                    if (!cp) {
                        fail("invalid unicode escape");
                        return std::nullopt;
                    }
                    uint32_t code = *cp;
                    if (code >= 0xD800 && code <= 0xDBFF && consumeLiteral("\\u")) {
                        auto lo = parseHex4();
                        if (!lo) {
                            fail("invalid surrogate pair");
                            return std::nullopt;
                        }
                        code = 0x10000 + ((code - 0xD800) << 10) + (*lo - 0xDC00);
                    }
                    appendUtf8(out, code);
                    break;
                }
                default:
                    fail("invalid escape sequence");
                    return std::nullopt;
            }
        }
        fail("unterminated string");
        return std::nullopt;
    }

    std::optional<Value> parseArray(int depth) {
        consume('[');
        Value arr = Value::array();
        skipWs();
        if (consume(']')) return arr;
        while (true) {
            skipWs();
            auto v = parseValue(depth + 1);
            if (!v) return std::nullopt;
            arr.push(std::move(*v));
            skipWs();
            if (consume(',')) continue;
            if (consume(']')) return arr;
            fail("expected ',' or ']'");
            return std::nullopt;
        }
    }

    std::optional<Value> parseObject(int depth) {
        consume('{');
        Value obj = Value::object();
        skipWs();
        if (consume('}')) return obj;
        while (true) {
            skipWs();
            auto key = parseString();
            if (!key) return std::nullopt;
            skipWs();
            if (!consume(':')) {
                fail("expected ':'");
                return std::nullopt;
            }
            skipWs();
            auto v = parseValue(depth + 1);
            if (!v) return std::nullopt;
            obj.set(*key, std::move(*v));
            skipWs();
            if (consume(',')) continue;
            if (consume('}')) return obj;
            fail("expected ',' or '}'");
            return std::nullopt;
        }
    }
};

void writeValue(std::string& out, Value const& v, bool pretty, int indent) {
    switch (v.type()) {
        case Value::Type::Null: out += "null"; break;
        case Value::Type::Bool: out += v.asBool() ? "true" : "false"; break;
        case Value::Type::Number: appendNumber(out, v.asNumber()); break;
        case Value::Type::String: appendString(out, v.asString()); break;
        case Value::Type::Array: {
            out += '[';
            bool first = true;
            for (auto const& item : v.items()) {
                if (!first) out += ',';
                first = false;
                writeValue(out, item, false, indent + 1);
            }
            out += ']';
            break;
        }
        case Value::Type::Object: {
            out += '{';
            bool first = true;
            for (auto const& [key, item] : v.members()) {
                if (!first) out += ',';
                first = false;
                if (pretty) {
                    out += '\n';
                    out.append(static_cast<size_t>(indent + 1) * 2, ' ');
                }
                appendString(out, key);
                out += pretty ? ": " : ":";
                writeValue(out, item, pretty, indent + 1);
            }
            if (pretty && !v.members().empty()) {
                out += '\n';
                out.append(static_cast<size_t>(indent) * 2, ' ');
            }
            out += '}';
            break;
        }
    }
}
} // namespace

Value const& Value::at(size_t i) const {
    if (m_type != Type::Array || i >= m_arr.size()) return nullValue();
    return m_arr[i];
}

Value const& Value::get(std::string const& key) const {
    if (m_type != Type::Object) return nullValue();
    auto it = m_obj.find(key);
    return it == m_obj.end() ? nullValue() : it->second;
}

std::optional<Value> parse(std::string_view text, std::string* error) {
    return Parser(text).run(error);
}

std::string write(Value const& value, bool pretty) {
    std::string out;
    writeValue(out, value, pretty, 0);
    return out;
}

void appendNumber(std::string& out, double v) {
    if (!std::isfinite(v)) {
        out += '0';
        return;
    }
    char buf[40];
    // Integers (Q-table keys, counters) are written exactly.
    if (std::fabs(v) < 9007199254740992.0 && v == std::floor(v)) {
        auto res = std::to_chars(buf, buf + sizeof(buf), static_cast<int64_t>(v));
        out.append(buf, res.ptr);
        return;
    }
    auto res = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::general, 9);
    out.append(buf, res.ptr);
}

void appendString(std::string& out, std::string_view s) {
    out += '"';
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    out += '"';
}

} // namespace aigd::json
