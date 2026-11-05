#pragma once

#include <cmath>

namespace uav {

constexpr double kEarthRadiusM = 6371008.8;
constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] constexpr double deg2rad(double d) { return d * kPi / 180.0; }
[[nodiscard]] constexpr double rad2deg(double r) { return r * 180.0 / kPi; }

struct LatLon {
  double lat = 0.0;
  double lon = 0.0;
  constexpr LatLon() = default;
  constexpr LatLon(double la, double lo) : lat(la), lon(lo) {}
};

/// Planar vector in metres (x = east, y = north).
struct Vec2 {
  double x = 0.0;
  double y = 0.0;
  constexpr Vec2() = default;
  constexpr Vec2(double px, double py) : x(px), y(py) {}
};

[[nodiscard]] constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
[[nodiscard]] constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
[[nodiscard]] constexpr Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
[[nodiscard]] inline double length(Vec2 a) { return std::hypot(a.x, a.y); }
[[nodiscard]] inline double distance(Vec2 a, Vec2 b) { return length(a - b); }

struct Enu {
  double e = 0.0;
  double n = 0.0;
  double u = 0.0;
};

/// Great-circle distance in metres (haversine, spherical Earth).
[[nodiscard]] double haversine(LatLon a, LatLon b);
/// Initial bearing from a to b in degrees [0, 360).
[[nodiscard]] double initialBearingDeg(LatLon a, LatLon b);
/// Point reached from `from` after travelling `distM` metres along `bearingDeg`.
[[nodiscard]] LatLon destination(LatLon from, double bearingDeg, double distM);
/// Point at fraction f (0..1) along the great circle from a to b.
[[nodiscard]] LatLon intermediate(LatLon a, LatLon b, double f);

/// Local East-North-Up frame anchored at a geodetic origin (spherical Earth).
class LocalFrame {
 public:
  explicit LocalFrame(LatLon origin, double originAltM = 0.0);
  [[nodiscard]] Enu toEnu(LatLon p, double altM = 0.0) const;
  [[nodiscard]] LatLon toLatLon(const Enu& enu, double* altM = nullptr) const;
  [[nodiscard]] LatLon origin() const { return origin_; }

 private:
  LatLon origin_;
  double sinLat_ = 0.0, cosLat_ = 1.0, sinLon_ = 0.0, cosLon_ = 1.0;
  double ox_ = 0.0, oy_ = 0.0, oz_ = 0.0;
};

}  // namespace uav
