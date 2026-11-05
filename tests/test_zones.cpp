#include <string>
#include <vector>

#include "test_util.hpp"
#include "uav/terrain.hpp"
#include "uav/zones.hpp"

using namespace uav;

TEST(circle_zone_queries) {
  ZoneSet z;
  z.addCircle(Vec2(100, 100), 50.0);
  CHECK(z.blocked(Vec2(100, 140)));
  CHECK(!z.blocked(Vec2(100, 151)));
  CHECK(z.blocked(Vec2(100, 154), 5.0));
  CHECK(z.segmentBlocked(Vec2(0, 100), Vec2(200, 100)));
  CHECK(!z.segmentBlocked(Vec2(0, 0), Vec2(200, 0)));
  CHECK(z.segmentBlocked(Vec2(0, 0), Vec2(200, 0), 100.0));
  CHECK(!z.segmentBlocked(Vec2(0, 0), Vec2(0, 90)));  // ends before reaching the circle
  CHECK(z.segmentBlocked(Vec2(90, 90), Vec2(110, 110)));  // entirely inside
}

TEST(polygon_zone_queries) {
  ZoneSet z;
  z.addPolygon({Vec2(200, 200), Vec2(300, 200), Vec2(300, 300), Vec2(200, 300)});
  CHECK(z.blocked(Vec2(250, 250)));
  CHECK(!z.blocked(Vec2(350, 250)));
  CHECK(z.blocked(Vec2(350, 250), 60.0));
  CHECK(!z.blocked(Vec2(350, 250), 40.0));
  CHECK(z.segmentBlocked(Vec2(150, 250), Vec2(350, 250)));
  CHECK(!z.segmentBlocked(Vec2(150, 150), Vec2(350, 150)));
  CHECK(z.segmentBlocked(Vec2(220, 220), Vec2(280, 280)));  // entirely inside
  CHECK(z.segmentBlocked(Vec2(150, 150), Vec2(350, 150), 60.0));
}

TEST(concave_polygon) {
  ZoneSet z;
  // "U" shape opening upwards: the notch is free space.
  z.addPolygon({Vec2(0, 0), Vec2(30, 0), Vec2(30, 30), Vec2(20, 30), Vec2(20, 10), Vec2(10, 10), Vec2(10, 30),
                Vec2(0, 30)});
  CHECK(z.blocked(Vec2(5, 20)));
  CHECK(!z.blocked(Vec2(15, 20)));
  CHECK(z.blocked(Vec2(15, 5)));
}

TEST(invalid_zones_are_rejected) {
  ZoneSet z;
  CHECK_THROWS(z.addCircle(Vec2(0, 0), 0.0));
  CHECK_THROWS(z.addPolygon({Vec2(0, 0), Vec2(1, 1)}));
  CHECK(z.empty());
}

TEST(geojson_zones) {
  const std::string text = R"({
    "type": "FeatureCollection",
    "features": [
      {"type": "Feature", "properties": {"name": "airport"},
       "geometry": {"type": "Polygon", "coordinates":
         [[[8.0, 47.0], [8.1, 47.0], [8.1, 47.1], [8.0, 47.1], [8.0, 47.0]]]}},
      {"type": "Feature", "properties": {"name": "mast", "radius_m": 500},
       "geometry": {"type": "Point", "coordinates": [8.05, 47.05]}},
      {"type": "Feature", "properties": {},
       "geometry": {"type": "LineString", "coordinates": [[8.0, 47.0], [8.1, 47.1]]}},
      {"type": "Feature", "properties": {},
       "geometry": {"type": "Point", "coordinates": [8.0, 47.0]}}
    ]})";
  const std::vector<GeoZone> zones = parseGeoJsonZones(text);
  CHECK(zones.size() == 2);
  CHECK(zones[0].kind == GeoZone::Kind::Polygon);
  CHECK(zones[0].name == "airport");
  CHECK(zones[0].ring.size() == 4);  // closing vertex removed
  CHECK_NEAR(zones[0].ring[1].lon, 8.1, 1e-12);
  CHECK_NEAR(zones[0].ring[1].lat, 47.0, 1e-12);
  CHECK(zones[1].kind == GeoZone::Kind::Circle);
  CHECK_NEAR(zones[1].radiusM, 500.0, 1e-9);

  const Terrain t = Terrain::synthetic(SyntheticParams());
  const ZoneSet set = ZoneSet::fromGeo(zones, t);
  CHECK(set.circles().size() == 1);
  CHECK(set.polygons().size() == 1);
  const Vec2 c = t.toXY(LatLon(47.05, 8.05));
  CHECK_NEAR(set.circles()[0].center.x, c.x, 1e-6);
  CHECK(set.blocked(t.toXY(LatLon(47.05, 8.05))));
}

TEST(geojson_multipolygon_and_geometry) {
  const std::string text = R"({"type": "MultiPolygon", "coordinates": [
      [[[0, 0], [1, 0], [1, 1], [0, 0]]],
      [[[2, 2], [3, 2], [3, 3], [2, 2]]]]})";
  CHECK(parseGeoJsonZones(text).size() == 2);
}

TEST(geojson_errors) {
  CHECK_THROWS(parseGeoJsonZones("{}"));
  CHECK_THROWS(parseGeoJsonZones("[1]"));
  CHECK_THROWS(parseGeoJsonZones("not json"));
  CHECK_THROWS(parseGeoJsonZones(R"({"type": "Polygon", "coordinates": [[[0, 0], [1, 1]]]})"));
}

TEST_MAIN()
