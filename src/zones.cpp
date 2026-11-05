#include "uav/zones.hpp"

#include <algorithm>
#include <utility>

#include "uav/common.hpp"
#include "uav/json.hpp"

namespace uav {

namespace {

double cross(Vec2 a, Vec2 b, Vec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

bool properIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
  const double d1 = cross(c, d, a);
  const double d2 = cross(c, d, b);
  const double d3 = cross(a, b, c);
  const double d4 = cross(a, b, d);
  return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
         ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
}

double pointSegDist(Vec2 p, Vec2 a, Vec2 b) {
  const Vec2 ab = b - a;
  const double len2 = ab.x * ab.x + ab.y * ab.y;
  if (len2 <= 0.0) return distance(p, a);
  const Vec2 ap = p - a;
  const double t = std::clamp((ap.x * ab.x + ap.y * ab.y) / len2, 0.0, 1.0);
  return distance(p, a + ab * t);
}

double segSegDist(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
  if (properIntersect(a, b, c, d)) return 0.0;
  return std::min(std::min(pointSegDist(a, c, d), pointSegDist(b, c, d)),
                  std::min(pointSegDist(c, a, b), pointSegDist(d, a, b)));
}

bool pointInPolygon(Vec2 p, const std::vector<Vec2>& pts) {
  bool inside = false;
  for (std::size_t i = 0, j = pts.size() - 1; i < pts.size(); j = i++) {
    const Vec2 a = pts[i];
    const Vec2 b = pts[j];
    if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
      inside = !inside;
    }
  }
  return inside;
}

LatLon lonLat(const json::Value& coord) {
  if (!coord.isArray() || coord.size() < 2) throw Error("geojson: coordinate must be [lon, lat]");
  return {coord.at(1).asNumber(), coord.at(0).asNumber()};
}

std::vector<LatLon> parseRing(const json::Value& ring) {
  if (!ring.isArray()) throw Error("geojson: ring must be an array");
  std::vector<LatLon> pts;
  for (const auto& c : ring.items) pts.push_back(lonLat(c));
  if (pts.size() > 1 && pts.front().lat == pts.back().lat && pts.front().lon == pts.back().lon) pts.pop_back();
  if (pts.size() < 3) throw Error("geojson: polygon ring needs at least three points");
  return pts;
}

std::string propertyName(const json::Value* props) {
  if (props != nullptr && props->isObject()) {
    const json::Value* n = props->find("name");
    if (n != nullptr && n->isString()) return n->text;
  }
  return std::string();
}

void collect(const json::Value& node, const json::Value* props, std::vector<GeoZone>& out) {
  if (!node.isObject()) throw Error("geojson: expected an object");
  const std::string type = node.at("type").asString();
  if (type == "FeatureCollection") {
    for (const auto& f : node.at("features").items) collect(f, nullptr, out);
  } else if (type == "Feature") {
    const json::Value* geom = node.find("geometry");
    if (geom != nullptr && geom->isObject()) collect(*geom, node.find("properties"), out);
  } else if (type == "GeometryCollection") {
    for (const auto& g : node.at("geometries").items) collect(g, props, out);
  } else if (type == "Polygon") {
    GeoZone z;
    z.kind = GeoZone::Kind::Polygon;
    z.name = propertyName(props);
    z.ring = parseRing(node.at("coordinates").at(0));
    out.push_back(std::move(z));
  } else if (type == "MultiPolygon") {
    for (const auto& poly : node.at("coordinates").items) {
      GeoZone z;
      z.kind = GeoZone::Kind::Polygon;
      z.name = propertyName(props);
      z.ring = parseRing(poly.at(0));
      out.push_back(std::move(z));
    }
  } else if (type == "Point") {
    double radius = 0.0;
    if (props != nullptr && props->isObject()) {
      const json::Value* r = props->find("radius_m");
      if (r == nullptr) r = props->find("radius");
      if (r != nullptr && r->isNumber()) radius = r->number;
    }
    if (radius > 0.0) {
      GeoZone z;
      z.kind = GeoZone::Kind::Circle;
      z.name = propertyName(props);
      z.center = lonLat(node.at("coordinates"));
      z.radiusM = radius;
      out.push_back(std::move(z));
    }
  }
  // Other geometry types (LineString, ...) carry no area and are ignored.
}

}  // namespace

std::vector<GeoZone> parseGeoJsonZones(const std::string& text) {
  const json::Value doc = json::parse(text);
  std::vector<GeoZone> out;
  collect(doc, nullptr, out);
  return out;
}

void ZoneSet::addCircle(Vec2 center, double radius) {
  if (!(radius > 0.0)) throw Error("zone: circle radius must be positive");
  circles_.push_back({center, radius});
}

void ZoneSet::addPolygon(std::vector<Vec2> points) {
  if (points.size() < 3) throw Error("zone: polygon needs at least three points");
  Polygon poly;
  poly.lo = points.front();
  poly.hi = points.front();
  for (const Vec2 p : points) {
    poly.lo = {std::min(poly.lo.x, p.x), std::min(poly.lo.y, p.y)};
    poly.hi = {std::max(poly.hi.x, p.x), std::max(poly.hi.y, p.y)};
  }
  poly.points = std::move(points);
  polygons_.push_back(std::move(poly));
}

ZoneSet ZoneSet::fromGeo(const std::vector<GeoZone>& zones, const Terrain& terrain) {
  ZoneSet set;
  for (const GeoZone& z : zones) {
    if (z.kind == GeoZone::Kind::Circle) {
      set.addCircle(terrain.toXY(z.center), z.radiusM);
    } else {
      std::vector<Vec2> pts;
      pts.reserve(z.ring.size());
      for (const LatLon& ll : z.ring) pts.push_back(terrain.toXY(ll));
      set.addPolygon(std::move(pts));
    }
  }
  return set;
}

bool ZoneSet::blocked(Vec2 p, double margin) const {
  const double m = std::max(margin, 0.0);
  for (const Circle& c : circles_) {
    if (distance(p, c.center) <= c.radius + m) return true;
  }
  for (const Polygon& poly : polygons_) {
    if (p.x < poly.lo.x - m || p.x > poly.hi.x + m || p.y < poly.lo.y - m || p.y > poly.hi.y + m) continue;
    if (pointInPolygon(p, poly.points)) return true;
    for (std::size_t i = 0, j = poly.points.size() - 1; i < poly.points.size(); j = i++) {
      if (pointSegDist(p, poly.points[i], poly.points[j]) <= m) return true;
    }
  }
  return false;
}

bool ZoneSet::segmentBlocked(Vec2 a, Vec2 b, double margin) const {
  const double m = std::max(margin, 0.0);
  for (const Circle& c : circles_) {
    if (pointSegDist(c.center, a, b) <= c.radius + m) return true;
  }
  for (const Polygon& poly : polygons_) {
    if (std::max(a.x, b.x) < poly.lo.x - m || std::min(a.x, b.x) > poly.hi.x + m ||
        std::max(a.y, b.y) < poly.lo.y - m || std::min(a.y, b.y) > poly.hi.y + m) {
      continue;
    }
    if (pointInPolygon(a, poly.points)) return true;
    for (std::size_t i = 0, j = poly.points.size() - 1; i < poly.points.size(); j = i++) {
      if (segSegDist(a, b, poly.points[i], poly.points[j]) <= m) return true;
    }
  }
  return false;
}

}  // namespace uav
