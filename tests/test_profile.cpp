#include <cstddef>
#include <cstdint>
#include <vector>

#include "test_util.hpp"
#include "uav/profile.hpp"

using namespace uav;

namespace {

/// Linear interpolation of a profile's altitude at path distance s.
double altAt(const Profile& p, double s) {
  const auto& pts = p.points;
  if (s <= pts.front().s) return pts.front().alt;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    if (s <= pts[i].s) {
      const double span = pts[i].s - pts[i - 1].s;
      const double f = span > 1e-12 ? (s - pts[i - 1].s) / span : 0.0;
      return pts[i - 1].alt + (pts[i].alt - pts[i - 1].alt) * f;
    }
  }
  return pts.back().alt;
}

Terrain cliff() {
  // 11 x 3 cells of 100 m; the ground jumps from 0 to 500 m between x = 400 and x = 500.
  std::vector<float> data(11 * 3, 0.0F);
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t c = 5; c < 11; ++c) data[r * 11 + c] = 500.0F;
  }
  return Terrain(11, 3, 100.0, 100.0, LatLon(47.0, 8.0), 47.0, data);
}

}  // namespace

TEST(profile_keeps_clearance_and_gradient_limits) {
  SyntheticParams sp;
  sp.exponent = 6;
  sp.seed = 9;
  const Terrain t = Terrain::synthetic(sp);
  ProfileConfig cfg;
  cfg.agl_m = 40.0;
  cfg.max_climb_gradient = 0.3;
  cfg.max_descent_gradient = 0.3;
  const std::vector<Vec2> path = {Vec2(100, 100), Vec2(900, 700), Vec2(1700, 1700)};
  const Profile prof = buildProfile(t, path, cfg);

  CHECK(prof.points.size() > 3);
  CHECK(prof.min_agl >= 40.0 - 1e-9);
  CHECK(minClearance(t, prof) >= 40.0 - 1e-9);
  int vertices = 0;
  for (std::size_t i = 0; i < prof.points.size(); ++i) {
    const ProfilePoint& pp = prof.points[i];
    CHECK(pp.alt - t.elevation(pp.p) >= 40.0 - 1e-9);
    if (pp.vertex) ++vertices;
    if (i == 0) continue;
    const double ds = pp.s - prof.points[i - 1].s;
    CHECK(ds >= 0.0);
    CHECK(pp.alt - prof.points[i - 1].alt <= 0.3 * ds + 1e-9);
    CHECK(prof.points[i - 1].alt - pp.alt <= 0.3 * ds + 1e-9);
  }
  CHECK(vertices == 3);
  CHECK(prof.max_climb <= 0.3 + 1e-9);
  CHECK(prof.max_descent <= 0.3 + 1e-9);
  CHECK_NEAR(prof.points.back().s, distance(path[0], path[1]) + distance(path[1], path[2]), 1e-6);
}

TEST(profile_starts_climbing_before_a_cliff) {
  const Terrain t = cliff();
  ProfileConfig cfg;
  cfg.agl_m = 10.0;
  cfg.max_climb_gradient = 0.2;
  cfg.max_descent_gradient = 0.2;
  const Profile up = buildProfile(t, {Vec2(0, 100), Vec2(1000, 100)}, cfg);
  CHECK(up.points.front().alt >= 410.0 - 1e-6);  // must already be climbing 500 m before the wall
  CHECK(up.max_climb <= 0.2 + 1e-9);
  CHECK(minClearance(t, up) >= 10.0 - 1e-9);

  const Profile down = buildProfile(t, {Vec2(1000, 100), Vec2(0, 100)}, cfg);
  CHECK(down.points.back().alt >= 410.0 - 1e-6);  // cannot drop faster than the descent limit
  CHECK(down.max_descent <= 0.2 + 1e-9);
  CHECK(minClearance(t, down) >= 10.0 - 1e-9);
}

TEST(profile_over_flat_ground_is_level) {
  const Terrain t(5, 5, 50.0, 50.0, LatLon(47.0, 8.0), 47.0, std::vector<float>(25, 100.0F));
  ProfileConfig cfg;
  cfg.agl_m = 30.0;
  const Profile prof = buildProfile(t, {Vec2(0, 0), Vec2(200, 200)}, cfg);
  for (const ProfilePoint& pp : prof.points) CHECK_NEAR(pp.alt, 130.0, 1e-9);
  const Profile slim = decimateProfile(prof);
  CHECK(slim.points.size() == 2);
}

TEST(decimation_never_lowers_the_profile) {
  SyntheticParams sp;
  sp.exponent = 6;
  sp.seed = 4;
  const Terrain t = Terrain::synthetic(sp);
  const std::vector<Vec2> path = {Vec2(60, 60), Vec2(1000, 400), Vec2(1800, 1800)};
  const Profile full = buildProfile(t, path, ProfileConfig());
  const Profile slim = decimateProfile(full);
  CHECK(slim.points.size() <= full.points.size());
  CHECK(slim.points.size() >= 3);
  CHECK_NEAR(slim.points.front().s, 0.0, 1e-12);
  CHECK_NEAR(slim.points.back().s, full.points.back().s, 1e-9);
  int vertices = 0;
  for (const ProfilePoint& pp : slim.points) {
    if (pp.vertex) ++vertices;
  }
  CHECK(vertices == 3);
  for (const ProfilePoint& pp : full.points) CHECK(altAt(slim, pp.s) >= pp.alt - 1e-6);
}

TEST(profile_reports_ceiling_violations) {
  const Terrain t = cliff();
  ProfileConfig cfg;
  cfg.agl_m = 10.0;
  cfg.max_altitude_m = 300.0;
  const Profile prof = buildProfile(t, {Vec2(0, 100), Vec2(1000, 100)}, cfg);
  CHECK(!prof.within_ceiling);
  cfg.max_altitude_m = 5000.0;
  CHECK(buildProfile(t, {Vec2(0, 100), Vec2(1000, 100)}, cfg).within_ceiling);
}

TEST(profile_rejects_bad_input) {
  const Terrain t = cliff();
  CHECK_THROWS(buildProfile(t, {Vec2(0, 0)}, ProfileConfig()));
  ProfileConfig bad;
  bad.max_climb_gradient = 0.0;
  CHECK_THROWS(buildProfile(t, {Vec2(0, 0), Vec2(10, 10)}, bad));
}

TEST_MAIN()
