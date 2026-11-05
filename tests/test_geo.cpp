#include "test_util.hpp"
#include "uav/geo.hpp"

using namespace uav;

TEST(haversine_one_degree_of_latitude) {
  CHECK_NEAR(haversine(LatLon(0, 0), LatLon(1, 0)), 111194.93, 1.0);
  CHECK_NEAR(haversine(LatLon(0, 0), LatLon(0, 1)), 111194.93, 1.0);
  CHECK_NEAR(haversine(LatLon(10, 20), LatLon(10, 20)), 0.0, 1e-9);
}

TEST(haversine_is_symmetric) {
  const LatLon a(47.3769, 8.5417);
  const LatLon b(40.7128, -74.0060);
  CHECK_NEAR(haversine(a, b), haversine(b, a), 1e-6);
  CHECK(haversine(a, b) > 6.0e6 && haversine(a, b) < 6.5e6);
}

TEST(bearing_cardinal_directions) {
  CHECK_NEAR(initialBearingDeg(LatLon(0, 0), LatLon(1, 0)), 0.0, 1e-6);
  CHECK_NEAR(initialBearingDeg(LatLon(0, 0), LatLon(0, 1)), 90.0, 1e-6);
  CHECK_NEAR(initialBearingDeg(LatLon(0, 0), LatLon(-1, 0)), 180.0, 1e-6);
  CHECK_NEAR(initialBearingDeg(LatLon(0, 0), LatLon(0, -1)), 270.0, 1e-6);
}

TEST(destination_round_trip) {
  const LatLon a(47.0, 8.0);
  const LatLon d = destination(a, 45.0, 10000.0);
  CHECK_NEAR(haversine(a, d), 10000.0, 1e-3);
  CHECK_NEAR(initialBearingDeg(a, d), 45.0, 0.01);
}

TEST(intermediate_midpoint) {
  const LatLon a(47.0, 8.0);
  const LatLon b(48.0, 10.0);
  const LatLon m = intermediate(a, b, 0.5);
  CHECK_NEAR(haversine(a, m), haversine(m, b), 1e-3);
  CHECK_NEAR(haversine(a, m) + haversine(m, b), haversine(a, b), 1e-3);
  const LatLon same = intermediate(a, a, 0.3);
  CHECK_NEAR(same.lat, a.lat, 1e-12);
}

TEST(enu_known_offsets) {
  const LocalFrame frame(LatLon(47.0, 8.0), 500.0);
  const Enu north = frame.toEnu(LatLon(47.01, 8.0), 500.0);
  CHECK_NEAR(north.n, 1111.95, 2.0);
  CHECK_NEAR(north.e, 0.0, 1e-6);
  const Enu east = frame.toEnu(LatLon(47.0, 8.02), 500.0);
  CHECK_NEAR(east.e, 0.02 * 111194.93 * std::cos(deg2rad(47.0)), 5.0);
  const Enu up = frame.toEnu(LatLon(47.0, 8.0), 600.0);
  CHECK_NEAR(up.u, 100.0, 1e-6);
}

TEST(enu_round_trip) {
  const LocalFrame frame(LatLon(-33.9, 151.2), 20.0);
  const LatLon p(-33.85, 151.3);
  double alt = 0.0;
  const Enu e = frame.toEnu(p, 250.0);
  const LatLon back = frame.toLatLon(e, &alt);
  CHECK_NEAR(back.lat, p.lat, 1e-9);
  CHECK_NEAR(back.lon, p.lon, 1e-9);
  CHECK_NEAR(alt, 250.0, 1e-4);
}

TEST_MAIN()
