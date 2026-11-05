#include <string>

#include "test_util.hpp"
#include "uav/json.hpp"

using namespace uav;

TEST(parse_and_query) {
  const json::Value v = json::parse(R"({"a": 1.5, "b": [true, null, "x"], "c": {"d": -2e3}})");
  CHECK(v.isObject());
  CHECK_NEAR(v.at("a").asNumber(), 1.5, 1e-12);
  CHECK(v.at("b").size() == 3);
  CHECK(v.at("b").at(0).asBool());
  CHECK(v.at("b").at(1).isNull());
  CHECK(v.at("b").at(2).asString() == "x");
  CHECK_NEAR(v.at("c").at("d").asNumber(), -2000.0, 1e-9);
  CHECK(v.find("missing") == nullptr);
}

TEST(dump_is_compact_and_round_trips) {
  const std::string src = R"({"a":1,"b":[true,null,"x\n"],"c":{"d":2.5}})";
  const json::Value v = json::parse(src);
  CHECK(json::dump(v) == src);
  const json::Value again = json::parse(json::dump(v, 2));
  CHECK(json::dump(again) == src);
}

TEST(string_escapes) {
  const json::Value v = json::parse(R"("a\"b\\cé😀")");
  CHECK(v.asString() == std::string("a\"b\\c") + "\xC3\xA9" + "\xF0\x9F\x98\x80");
}

TEST(builders) {
  json::Value o = json::Value::makeObject();
  o.set("n", json::Value::makeNumber(3));
  o.set("n", json::Value::makeNumber(4));
  o.set("s", json::Value::makeString("hi"));
  CHECK(o.size() == 2);
  CHECK(json::dump(o) == R"({"n":4,"s":"hi"})");
}

TEST(rejects_malformed_documents) {
  CHECK_THROWS(json::parse(""));
  CHECK_THROWS(json::parse("{"));
  CHECK_THROWS(json::parse("[1, 2"));
  CHECK_THROWS(json::parse("{\"a\" 1}"));
  CHECK_THROWS(json::parse("[1] x"));
  CHECK_THROWS(json::parse("\"unterminated"));
  CHECK_THROWS(json::parse("tru"));
  CHECK_THROWS(json::parse(std::string(500, '[')));
}

TEST(typed_accessors_throw) {
  const json::Value v = json::parse("[1]");
  CHECK_THROWS(v.asString());
  CHECK_THROWS(v.at(3));
  CHECK_THROWS(v.at("key"));
}

TEST_MAIN()
