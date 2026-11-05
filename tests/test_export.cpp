#include <cmath>
#include <string>
#include <vector>

#include "test_util.hpp"
#include "uav/export.hpp"
#include "uav/json.hpp"
#include "uav/mission.hpp"

using namespace uav;

namespace {

struct Fixture {
  Terrain terrain;
  ZoneSet zones;
  MissionPlan plan;
};

Fixture makeFixture() {
  SyntheticParams sp;
  sp.exponent = 5;
  sp.seed = 12;
  Fixture f{Terrain::synthetic(sp), ZoneSet(), MissionPlan()};
  f.zones.addCircle(Vec2(480, 480), 120.0);
  f.zones.addPolygon({Vec2(600, 100), Vec2(850, 150), Vec2(800, 350), Vec2(580, 300)});
  f.plan = planMission(f.terrain, f.zones, Vec2(60, 60), Vec2(900, 900), PlannerKind::ThetaStar, PlannerConfig(),
                       ProfileConfig(), RrtParams(), true);
  return f;
}

void checkSameMission(const Mission& a, const Mission& b, double angTol, double altTol) {
  CHECK_NEAR(a.homeLat, b.homeLat, angTol);
  CHECK_NEAR(a.homeLon, b.homeLon, angTol);
  CHECK_NEAR(a.homeAlt, b.homeAlt, altTol);
  CHECK(a.items.size() == b.items.size());
  const std::size_t n = a.items.size() < b.items.size() ? a.items.size() : b.items.size();
  for (std::size_t i = 0; i < n; ++i) {
    CHECK(a.items[i].command == b.items[i].command);
    CHECK(a.items[i].frame == b.items[i].frame);
    CHECK(a.items[i].autocontinue == b.items[i].autocontinue);
    CHECK_NEAR(a.items[i].lat, b.items[i].lat, angTol);
    CHECK_NEAR(a.items[i].lon, b.items[i].lon, angTol);
    CHECK_NEAR(a.items[i].alt, b.items[i].alt, altTol);
  }
}

}  // namespace

TEST(fixture_mission_is_usable) {
  const Fixture f = makeFixture();
  CHECK(f.plan.result.found);
  CHECK(f.plan.mission.items.size() >= 2);
}

TEST(wpl_round_trip) {
  const Fixture f = makeFixture();
  const std::string text = toWpl(f.plan.mission);
  CHECK(text.rfind("QGC WPL 110\n", 0) == 0);
  const std::size_t firstLine = text.find('\n') + 1;
  const std::string second = text.substr(firstLine, text.find('\n', firstLine) - firstLine);
  std::size_t tabs = 0;
  for (const char ch : second) {
    if (ch == '\t') ++tabs;
  }
  CHECK(tabs == 11);
  const Mission back = parseWpl(text);
  checkSameMission(f.plan.mission, back, 1e-7, 1e-5);
}

TEST(wpl_rejects_malformed_files) {
  CHECK_THROWS(parseWpl("not a waypoint file\n"));
  CHECK_THROWS(parseWpl(""));
  CHECK_THROWS(parseWpl("QGC WPL 110\n0\t1\t0\t16\n"));
  CHECK_THROWS(parseWpl("QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\tx\t0\t0\t1\n"));
}

TEST(qgc_plan_round_trip) {
  const Fixture f = makeFixture();
  const std::string text = toQgcPlan(f.plan.mission, f.terrain, f.zones);
  const json::Value doc = json::parse(text);
  CHECK(doc.at("fileType").asString() == "Plan");
  CHECK(doc.at("mission").at("items").size() == f.plan.mission.items.size());
  CHECK(doc.at("geoFence").at("circles").size() == 1);
  CHECK(doc.at("geoFence").at("polygons").size() == 1);
  CHECK(!doc.at("geoFence").at("circles").at(0).at("inclusion").asBool());
  CHECK_NEAR(doc.at("geoFence").at("circles").at(0).at("circle").at("radius").asNumber(), 120.0, 1e-9);
  const Mission back = parseQgcPlan(text);
  checkSameMission(f.plan.mission, back, 1e-9, 1e-6);
}

TEST(qgc_plan_rejects_other_documents) {
  CHECK_THROWS(parseQgcPlan("{}"));
  CHECK_THROWS(parseQgcPlan(R"({"fileType": "Other"})"));
  CHECK_THROWS(parseQgcPlan("[]"));
}

TEST(mavlink_item_list) {
  const Fixture f = makeFixture();
  const json::Value list = json::parse(toMavlinkItemsJson(f.plan.mission));
  CHECK(list.isArray());
  CHECK(list.size() == f.plan.mission.items.size() + 1);
  CHECK(list.at(0).at("seq").asNumber() == 0.0);
  CHECK(list.at(0).at("current").asNumber() == 1.0);
  CHECK_NEAR(list.at(0).at("x").asNumber(), std::round(f.plan.mission.homeLat * 1e7), 1.0);
  CHECK(list.at(1).at("command").asNumber() == 16.0);
  CHECK_NEAR(list.at(1).at("z").asNumber(), f.plan.mission.items[0].alt, 1e-6);
  CHECK(list.at(list.size() - 1).at("seq").asNumber() == static_cast<double>(list.size() - 1));
}

TEST(geojson_export_can_be_read_back) {
  const Fixture f = makeFixture();
  const std::string text = toGeoJson(f.plan.mission, f.terrain, f.zones);
  const json::Value doc = json::parse(text);
  CHECK(doc.at("type").asString() == "FeatureCollection");
  const json::Value& features = doc.at("features");
  CHECK(features.size() == 3);  // path + circle + polygon
  CHECK(features.at(0).at("geometry").at("type").asString() == "LineString");
  const json::Value& coords = features.at(0).at("geometry").at("coordinates");
  CHECK(coords.size() == f.plan.mission.items.size());
  CHECK_NEAR(coords.at(0).at(0).asNumber(), f.plan.mission.items[0].lon, 1e-9);
  CHECK_NEAR(coords.at(0).at(1).asNumber(), f.plan.mission.items[0].lat, 1e-9);
  CHECK_NEAR(coords.at(0).at(2).asNumber(), f.plan.mission.items[0].alt, 1e-6);
  CHECK(features.at(1).at("geometry").at("type").asString() == "Polygon");

  // The zones written for viewing can be fed back in as no-fly zones.
  const std::vector<GeoZone> zones = parseGeoJsonZones(text);
  CHECK(zones.size() == 2);
  const ZoneSet again = ZoneSet::fromGeo(zones, f.terrain);
  CHECK(again.blocked(Vec2(480, 480)));
  CHECK(again.blocked(Vec2(700, 220)));
  CHECK(!again.blocked(Vec2(60, 60)));
}

TEST(kml_export_structure) {
  const Fixture f = makeFixture();
  const std::string kml = toKml(f.plan.mission, f.terrain, f.zones);
  CHECK(kml.find("<kml") != std::string::npos);
  CHECK(kml.find("<LineString>") != std::string::npos);
  CHECK(kml.find("<Polygon>") != std::string::npos);
  CHECK(kml.find("</kml>") != std::string::npos);
  std::size_t placemarks = 0;
  for (std::size_t pos = kml.find("<Placemark>"); pos != std::string::npos; pos = kml.find("<Placemark>", pos + 1)) {
    ++placemarks;
  }
  CHECK(placemarks == 3);
}

TEST_MAIN()
