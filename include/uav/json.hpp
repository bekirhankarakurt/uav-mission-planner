#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace uav::json {

/// Minimal JSON document node (enough for GeoJSON and QGroundControl plans).
struct Value {
  enum class Type { Null, Bool, Number, String, Array, Object };

  Type type = Type::Null;
  bool boolean = false;
  double number = 0.0;
  std::string text;
  std::vector<Value> items;        // array elements or object values
  std::vector<std::string> keys;   // object keys (parallel to items)

  [[nodiscard]] static Value makeNull() { return Value(); }
  [[nodiscard]] static Value makeBool(bool b);
  [[nodiscard]] static Value makeNumber(double n);
  [[nodiscard]] static Value makeString(std::string s);
  [[nodiscard]] static Value makeArray();
  [[nodiscard]] static Value makeObject();

  Value& push(Value v);                       // append to array
  Value& set(const std::string& key, Value v);  // insert or replace in object

  [[nodiscard]] bool isNull() const { return type == Type::Null; }
  [[nodiscard]] bool isBool() const { return type == Type::Bool; }
  [[nodiscard]] bool isNumber() const { return type == Type::Number; }
  [[nodiscard]] bool isString() const { return type == Type::String; }
  [[nodiscard]] bool isArray() const { return type == Type::Array; }
  [[nodiscard]] bool isObject() const { return type == Type::Object; }

  [[nodiscard]] std::size_t size() const { return items.size(); }
  [[nodiscard]] const Value* find(const std::string& key) const;  // nullptr when absent
  [[nodiscard]] const Value& at(const std::string& key) const;    // throws uav::Error
  [[nodiscard]] const Value& at(std::size_t index) const;         // throws uav::Error
  [[nodiscard]] double asNumber() const;                          // throws uav::Error
  [[nodiscard]] const std::string& asString() const;              // throws uav::Error
  [[nodiscard]] bool asBool() const;                              // throws uav::Error
};

/// Parse a JSON document. Throws uav::Error with a position on malformed input.
[[nodiscard]] Value parse(const std::string& text);
/// Serialise a value. indent < 0 produces compact output.
[[nodiscard]] std::string dump(const Value& v, int indent = -1);

}  // namespace uav::json
