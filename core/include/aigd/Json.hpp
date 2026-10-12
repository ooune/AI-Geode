#pragma once

// Minimal JSON reader/writer used by the learning core.
// The core deliberately does not depend on Geode (or matjson) so that it can be
// unit-tested as a plain C++ library outside of Geometry Dash.

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace aigd::json {

class Value {
public:
    enum class Type : uint8_t { Null, Bool, Number, String, Array, Object };

    Value() = default;
    Value(std::nullptr_t) {}
    Value(bool b) : m_type(Type::Bool), m_bool(b) {}
    Value(double d) : m_type(Type::Number), m_num(d) {}
    Value(int v) : m_type(Type::Number), m_num(static_cast<double>(v)) {}
    Value(int64_t v) : m_type(Type::Number), m_num(static_cast<double>(v)) {}
    Value(uint32_t v) : m_type(Type::Number), m_num(static_cast<double>(v)) {}
    Value(uint64_t v) : m_type(Type::Number), m_num(static_cast<double>(v)) {}
    Value(float v) : m_type(Type::Number), m_num(static_cast<double>(v)) {}
    Value(std::string s) : m_type(Type::String), m_str(std::move(s)) {}
    Value(char const* s) : m_type(Type::String), m_str(s) {}

    static Value array() { Value v; v.m_type = Type::Array; return v; }
    static Value object() { Value v; v.m_type = Type::Object; return v; }

    Type type() const { return m_type; }
    bool isNull() const { return m_type == Type::Null; }
    bool isBool() const { return m_type == Type::Bool; }
    bool isNumber() const { return m_type == Type::Number; }
    bool isString() const { return m_type == Type::String; }
    bool isArray() const { return m_type == Type::Array; }
    bool isObject() const { return m_type == Type::Object; }

    bool asBool(bool def = false) const { return m_type == Type::Bool ? m_bool : def; }
    double asNumber(double def = 0.0) const { return m_type == Type::Number ? m_num : def; }
    int64_t asInt(int64_t def = 0) const {
        return m_type == Type::Number ? static_cast<int64_t>(m_num) : def;
    }
    std::string const& asString() const { return m_str; }

    // Array access
    std::vector<Value> const& items() const { return m_arr; }
    std::vector<Value>& items() { return m_arr; }
    void push(Value v) { m_arr.push_back(std::move(v)); }
    size_t size() const { return m_type == Type::Array ? m_arr.size() : m_obj.size(); }
    Value const& at(size_t i) const;

    // Object access (keys are kept sorted for deterministic output)
    std::map<std::string, Value> const& members() const { return m_obj; }
    void set(std::string const& key, Value v) { m_obj[key] = std::move(v); }
    bool has(std::string const& key) const { return m_obj.find(key) != m_obj.end(); }
    // Returns a shared null value when the key is absent.
    Value const& get(std::string const& key) const;

private:
    Type m_type = Type::Null;
    bool m_bool = false;
    double m_num = 0.0;
    std::string m_str;
    std::vector<Value> m_arr;
    std::map<std::string, Value> m_obj;
};

// Parses a JSON document. On failure returns std::nullopt and fills `error`.
std::optional<Value> parse(std::string_view text, std::string* error = nullptr);

// Serialises a value. `pretty` adds newlines/indentation for objects.
std::string write(Value const& value, bool pretty = false);

// Low-level helpers shared with the streaming writer in Storage.cpp.
void appendNumber(std::string& out, double v);
void appendString(std::string& out, std::string_view s);

} // namespace aigd::json
