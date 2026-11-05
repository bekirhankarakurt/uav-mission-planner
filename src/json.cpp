#include "uav/json.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

#include "uav/common.hpp"

namespace uav::json {

Value Value::makeBool(bool b) {
  Value v;
  v.type = Type::Bool;
  v.boolean = b;
  return v;
}

Value Value::makeNumber(double n) {
  Value v;
  v.type = Type::Number;
  v.number = n;
  return v;
}

Value Value::makeString(std::string s) {
  Value v;
  v.type = Type::String;
  v.text = std::move(s);
  return v;
}

Value Value::makeArray() {
  Value v;
  v.type = Type::Array;
  return v;
}

Value Value::makeObject() {
  Value v;
  v.type = Type::Object;
  return v;
}

Value& Value::push(Value v) {
  if (type != Type::Array) throw Error("json: push on a non-array value");
  items.push_back(std::move(v));
  return items.back();
}

Value& Value::set(const std::string& key, Value v) {
  if (type != Type::Object) throw Error("json: set on a non-object value");
  for (std::size_t i = 0; i < keys.size(); ++i) {
    if (keys[i] == key) {
      items[i] = std::move(v);
      return items[i];
    }
  }
  keys.push_back(key);
  items.push_back(std::move(v));
  return items.back();
}

const Value* Value::find(const std::string& key) const {
  if (type != Type::Object) return nullptr;
  for (std::size_t i = 0; i < keys.size(); ++i) {
    if (keys[i] == key) return &items[i];
  }
  return nullptr;
}

const Value& Value::at(const std::string& key) const {
  const Value* v = find(key);
  if (v == nullptr) throw Error("json: missing key '" + key + "'");
  return *v;
}

const Value& Value::at(std::size_t index) const {
  if (type != Type::Array || index >= items.size()) throw Error("json: array index out of range");
  return items[index];
}

double Value::asNumber() const {
  if (type != Type::Number) throw Error("json: expected a number");
  return number;
}

const std::string& Value::asString() const {
  if (type != Type::String) throw Error("json: expected a string");
  return text;
}

bool Value::asBool() const {
  if (type != Type::Bool) throw Error("json: expected a boolean");
  return boolean;
}

namespace {

class Parser {
 public:
  explicit Parser(const std::string& s) : s_(s) {}

  Value parseDocument() {
    skipWs();
    Value v = parseValue();
    skipWs();
    if (pos_ != s_.size()) fail("trailing characters after document");
    return v;
  }

 private:
  const std::string& s_;
  std::size_t pos_ = 0;
  int depth_ = 0;

  [[noreturn]] void fail(const std::string& msg) const {
    throw Error("json: " + msg + " at offset " + std::to_string(pos_));
  }

  void skipWs() {
    while (pos_ < s_.size()) {
      const char c = s_[pos_];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        ++pos_;
      } else {
        break;
      }
    }
  }

  [[nodiscard]] char peek() const { return pos_ < s_.size() ? s_[pos_] : '\0'; }

  void expectLiteral(const char* lit) {
    for (const char* p = lit; *p != '\0'; ++p) {
      if (pos_ >= s_.size() || s_[pos_] != *p) fail("invalid literal");
      ++pos_;
    }
  }

  Value parseValue() {
    if (++depth_ > 256) fail("document nested too deeply");
    skipWs();
    Value v;
    const char c = peek();
    if (c == '{') {
      v = parseObject();
    } else if (c == '[') {
      v = parseArray();
    } else if (c == '"') {
      v = Value::makeString(parseString());
    } else if (c == 't') {
      expectLiteral("true");
      v = Value::makeBool(true);
    } else if (c == 'f') {
      expectLiteral("false");
      v = Value::makeBool(false);
    } else if (c == 'n') {
      expectLiteral("null");
    } else if (c == '-' || (c >= '0' && c <= '9')) {
      v = parseNumber();
    } else {
      fail("unexpected character");
    }
    --depth_;
    return v;
  }

  Value parseNumber() {
    const char* begin = s_.c_str() + pos_;
    char* end = nullptr;
    const double d = std::strtod(begin, &end);
    if (end == begin) fail("invalid number");
    pos_ += static_cast<std::size_t>(end - begin);
    return Value::makeNumber(d);
  }

  unsigned parseHex4() {
    unsigned code = 0;
    for (int i = 0; i < 4; ++i) {
      if (pos_ >= s_.size()) fail("truncated unicode escape");
      const char h = s_[pos_++];
      code <<= 4U;
      if (h >= '0' && h <= '9') {
        code |= static_cast<unsigned>(h - '0');
      } else if (h >= 'a' && h <= 'f') {
        code |= static_cast<unsigned>(h - 'a' + 10);
      } else if (h >= 'A' && h <= 'F') {
        code |= static_cast<unsigned>(h - 'A' + 10);
      } else {
        fail("invalid unicode escape");
      }
    }
    return code;
  }

  static void appendUtf8(std::string& out, unsigned cp) {
    if (cp < 0x80U) {
      out.push_back(static_cast<char>(cp));
    } else if (cp < 0x800U) {
      out.push_back(static_cast<char>(0xC0U | (cp >> 6U)));
      out.push_back(static_cast<char>(0x80U | (cp & 0x3FU)));
    } else if (cp < 0x10000U) {
      out.push_back(static_cast<char>(0xE0U | (cp >> 12U)));
      out.push_back(static_cast<char>(0x80U | ((cp >> 6U) & 0x3FU)));
      out.push_back(static_cast<char>(0x80U | (cp & 0x3FU)));
    } else {
      out.push_back(static_cast<char>(0xF0U | (cp >> 18U)));
      out.push_back(static_cast<char>(0x80U | ((cp >> 12U) & 0x3FU)));
      out.push_back(static_cast<char>(0x80U | ((cp >> 6U) & 0x3FU)));
      out.push_back(static_cast<char>(0x80U | (cp & 0x3FU)));
    }
  }

  std::string parseString() {
    ++pos_;  // opening quote
    std::string out;
    while (true) {
      if (pos_ >= s_.size()) fail("unterminated string");
      const char c = s_[pos_++];
      if (c == '"') break;
      if (c != '\\') {
        out.push_back(c);
        continue;
      }
      if (pos_ >= s_.size()) fail("unterminated escape");
      const char e = s_[pos_++];
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          unsigned cp = parseHex4();
          if (cp >= 0xD800U && cp < 0xDC00U && pos_ + 1 < s_.size() && s_[pos_] == '\\' &&
              s_[pos_ + 1] == 'u') {
            pos_ += 2;
            const unsigned lo = parseHex4();
            cp = 0x10000U + ((cp - 0xD800U) << 10U) + (lo - 0xDC00U);
          }
          appendUtf8(out, cp);
          break;
        }
        default: fail("invalid escape sequence");
      }
    }
    return out;
  }

  Value parseArray() {
    Value arr = Value::makeArray();
    ++pos_;  // [
    skipWs();
    if (peek() == ']') {
      ++pos_;
      return arr;
    }
    while (true) {
      arr.push(parseValue());
      skipWs();
      const char c = peek();
      ++pos_;
      if (c == ']') break;
      if (c != ',') fail("expected ',' or ']'");
    }
    return arr;
  }

  Value parseObject() {
    Value obj = Value::makeObject();
    ++pos_;  // {
    skipWs();
    if (peek() == '}') {
      ++pos_;
      return obj;
    }
    while (true) {
      skipWs();
      if (peek() != '"') fail("expected object key");
      std::string key = parseString();
      skipWs();
      if (peek() != ':') fail("expected ':'");
      ++pos_;
      obj.set(key, parseValue());
      skipWs();
      const char c = peek();
      ++pos_;
      if (c == '}') break;
      if (c != ',') fail("expected ',' or '}'");
    }
    return obj;
  }
};

void escapeInto(std::string& out, const std::string& s) {
  out.push_back('"');
  for (const char ch : s) {
    switch (ch) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20U) {
          char buf[16];
          std::snprintf(buf, sizeof buf, "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(ch)));
          out += buf;
        } else {
          out.push_back(ch);
        }
    }
  }
  out.push_back('"');
}

void numberInto(std::string& out, double v) {
  if (!std::isfinite(v)) {
    out += "null";
    return;
  }
  char buf[48];
  if (v == std::floor(v) && std::fabs(v) < 1e15) {
    std::snprintf(buf, sizeof buf, "%.0f", v);
  } else {
    std::snprintf(buf, sizeof buf, "%.12g", v);
  }
  out += buf;
}

void newline(std::string& out, int indent, int level) {
  if (indent < 0) return;
  out.push_back('\n');
  out.append(static_cast<std::size_t>(indent) * static_cast<std::size_t>(level), ' ');
}

void dumpInto(std::string& out, const Value& v, int indent, int level) {
  switch (v.type) {
    case Value::Type::Null: out += "null"; break;
    case Value::Type::Bool: out += v.boolean ? "true" : "false"; break;
    case Value::Type::Number: numberInto(out, v.number); break;
    case Value::Type::String: escapeInto(out, v.text); break;
    case Value::Type::Array: {
      if (v.items.empty()) {
        out += "[]";
        break;
      }
      out.push_back('[');
      for (std::size_t i = 0; i < v.items.size(); ++i) {
        if (i > 0) out.push_back(',');
        newline(out, indent, level + 1);
        dumpInto(out, v.items[i], indent, level + 1);
      }
      newline(out, indent, level);
      out.push_back(']');
      break;
    }
    case Value::Type::Object: {
      if (v.items.empty()) {
        out += "{}";
        break;
      }
      out.push_back('{');
      for (std::size_t i = 0; i < v.items.size(); ++i) {
        if (i > 0) out.push_back(',');
        newline(out, indent, level + 1);
        escapeInto(out, v.keys[i]);
        out.push_back(':');
        if (indent >= 0) out.push_back(' ');
        dumpInto(out, v.items[i], indent, level + 1);
      }
      newline(out, indent, level);
      out.push_back('}');
      break;
    }
  }
}

}  // namespace

Value parse(const std::string& text) {
  Parser p(text);
  return p.parseDocument();
}

std::string dump(const Value& v, int indent) {
  std::string out;
  dumpInto(out, v, indent, 0);
  return out;
}

}  // namespace uav::json
