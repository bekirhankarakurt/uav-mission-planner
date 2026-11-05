#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "uav/geo.hpp"
#include "uav/terrain.hpp"

namespace uav {

/// No-fly zone expressed in geographic coordinates (circle or polygon).
struct GeoZone {
  enum class Kind { Circle, Polygon };
  Kind kind = Kind::Circle;
  std::string name;
  LatLon center;                 ///< circle centre
  double radiusM = 0.0;          ///< circle radius
  std::vector<LatLon> ring;      ///< polygon outer ring (not closed)
};

/// Parse zones from GeoJSON text. Polygon / MultiPolygon features become polygons;
/// Point features with a "radius_m" (or "radius") property become circles.
[[nodiscard]] std::vector<GeoZone> parseGeoJsonZones(const std::string& text);

/// No-fly zones in the planar (metric) frame of a terrain.
class ZoneSet {
 public:
  struct Circle {
    Vec2 center;
    double radius = 0.0;
  };
  struct Polygon {
    std::vector<Vec2> points;
    Vec2 lo;
    Vec2 hi;
  };

  void addCircle(Vec2 center, double radius);
  void addPolygon(std::vector<Vec2> points);

  [[nodiscard]] static ZoneSet fromGeo(const std::vector<GeoZone>& zones, const Terrain& terrain);

  /// True when p is inside a zone or closer than `margin` metres to its boundary.
  [[nodiscard]] bool blocked(Vec2 p, double margin = 0.0) const;
  /// True when the segment a-b touches a zone (expanded by `margin` metres).
  [[nodiscard]] bool segmentBlocked(Vec2 a, Vec2 b, double margin = 0.0) const;

  [[nodiscard]] bool empty() const { return circles_.empty() && polygons_.empty(); }
  [[nodiscard]] const std::vector<Circle>& circles() const { return circles_; }
  [[nodiscard]] const std::vector<Polygon>& polygons() const { return polygons_; }

 private:
  std::vector<Circle> circles_;
  std::vector<Polygon> polygons_;
};

}  // namespace uav
