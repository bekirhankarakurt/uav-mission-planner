#include "uav/geo.hpp"

#include <algorithm>

namespace uav {

double haversine(LatLon a, LatLon b) {
  const double p1 = deg2rad(a.lat);
  const double p2 = deg2rad(b.lat);
  const double dp = p2 - p1;
  const double dl = deg2rad(b.lon - a.lon);
  const double s1 = std::sin(dp / 2.0);
  const double s2 = std::sin(dl / 2.0);
  const double h = s1 * s1 + std::cos(p1) * std::cos(p2) * s2 * s2;
  return 2.0 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(h)));
}

double initialBearingDeg(LatLon a, LatLon b) {
  const double p1 = deg2rad(a.lat);
  const double p2 = deg2rad(b.lat);
  const double dl = deg2rad(b.lon - a.lon);
  const double y = std::sin(dl) * std::cos(p2);
  const double x = std::cos(p1) * std::sin(p2) - std::sin(p1) * std::cos(p2) * std::cos(dl);
  const double deg = rad2deg(std::atan2(y, x));
  return std::fmod(deg + 360.0, 360.0);
}

LatLon destination(LatLon from, double bearingDeg, double distM) {
  const double d = distM / kEarthRadiusM;
  const double br = deg2rad(bearingDeg);
  const double p1 = deg2rad(from.lat);
  const double l1 = deg2rad(from.lon);
  const double sinP2 = std::sin(p1) * std::cos(d) + std::cos(p1) * std::sin(d) * std::cos(br);
  const double p2 = std::asin(std::max(-1.0, std::min(1.0, sinP2)));
  const double l2 = l1 + std::atan2(std::sin(br) * std::sin(d) * std::cos(p1),
                                    std::cos(d) - std::sin(p1) * std::sin(p2));
  const double lon = std::fmod(rad2deg(l2) + 540.0, 360.0) - 180.0;
  return {rad2deg(p2), lon};
}

LatLon intermediate(LatLon a, LatLon b, double f) {
  const double delta = haversine(a, b) / kEarthRadiusM;
  if (delta < 1e-12) return a;
  const double p1 = deg2rad(a.lat);
  const double l1 = deg2rad(a.lon);
  const double p2 = deg2rad(b.lat);
  const double l2 = deg2rad(b.lon);
  const double ka = std::sin((1.0 - f) * delta) / std::sin(delta);
  const double kb = std::sin(f * delta) / std::sin(delta);
  const double x = ka * std::cos(p1) * std::cos(l1) + kb * std::cos(p2) * std::cos(l2);
  const double y = ka * std::cos(p1) * std::sin(l1) + kb * std::cos(p2) * std::sin(l2);
  const double z = ka * std::sin(p1) + kb * std::sin(p2);
  return {rad2deg(std::atan2(z, std::sqrt(x * x + y * y))), rad2deg(std::atan2(y, x))};
}

LocalFrame::LocalFrame(LatLon origin, double originAltM) : origin_(origin) {
  const double p = deg2rad(origin.lat);
  const double l = deg2rad(origin.lon);
  sinLat_ = std::sin(p);
  cosLat_ = std::cos(p);
  sinLon_ = std::sin(l);
  cosLon_ = std::cos(l);
  const double r = kEarthRadiusM + originAltM;
  ox_ = r * cosLat_ * cosLon_;
  oy_ = r * cosLat_ * sinLon_;
  oz_ = r * sinLat_;
}

Enu LocalFrame::toEnu(LatLon p, double altM) const {
  const double ph = deg2rad(p.lat);
  const double la = deg2rad(p.lon);
  const double r = kEarthRadiusM + altM;
  const double dx = r * std::cos(ph) * std::cos(la) - ox_;
  const double dy = r * std::cos(ph) * std::sin(la) - oy_;
  const double dz = r * std::sin(ph) - oz_;
  Enu out;
  out.e = -sinLon_ * dx + cosLon_ * dy;
  out.n = -sinLat_ * cosLon_ * dx - sinLat_ * sinLon_ * dy + cosLat_ * dz;
  out.u = cosLat_ * cosLon_ * dx + cosLat_ * sinLon_ * dy + sinLat_ * dz;
  return out;
}

LatLon LocalFrame::toLatLon(const Enu& enu, double* altM) const {
  const double dx = -sinLon_ * enu.e - sinLat_ * cosLon_ * enu.n + cosLat_ * cosLon_ * enu.u;
  const double dy = cosLon_ * enu.e - sinLat_ * sinLon_ * enu.n + cosLat_ * sinLon_ * enu.u;
  const double dz = cosLat_ * enu.n + sinLat_ * enu.u;
  const double x = ox_ + dx;
  const double y = oy_ + dy;
  const double z = oz_ + dz;
  const double rad = std::sqrt(x * x + y * y + z * z);
  if (altM != nullptr) *altM = rad - kEarthRadiusM;
  return {rad2deg(std::atan2(z, std::sqrt(x * x + y * y))), rad2deg(std::atan2(y, x))};
}

}  // namespace uav
