#include <cmath>
#include <cstdint>
#include <vector>

#include "test_util.hpp"
#include "uav/mission.hpp"
#include "uav/planner.hpp"

using namespace uav;

namespace {

Terrain flatTerrain(std::size_t n, double cell) {
  return Terrain(n, n, cell, cell, LatLon(47.0, 8.0), 47.0, std::vector<float>(n * n, 0.0F));
}

Terrain hilly(int exponent, std::uint64_t seed) {
  SyntheticParams sp;
  sp.exponent = exponent;
  sp.seed = seed;
  return Terrain::synthetic(sp);
}

/// True when no vertex and no point sampled every 2 m along the path lies inside a zone.
bool pathAvoidsZones(const ZoneSet& zones, const std::vector<Vec2>& path, double margin) {
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (zones.blocked(path[i], margin)) return false;
    if (i == 0) continue;
    if (zones.segmentBlocked(path[i - 1], path[i], margin)) return false;
    const double len = distance(path[i - 1], path[i]);
    const auto n = static_cast<std::size_t>(len / 2.0) + 1;
    for (std::size_t k = 0; k <= n; ++k) {
      const Vec2 q = path[i - 1] + (path[i] - path[i - 1]) * (static_cast<double>(k) / static_cast<double>(n));
      if (zones.blocked(q, margin)) return false;
    }
  }
  return true;
}

}  // namespace

TEST(open_flat_terrain_gives_the_straight_line) {
  const Terrain t = flatTerrain(41, 10.0);
  const ZoneSet zones;
  const PlanningContext ctx(t, zones, PlannerConfig());
  const Vec2 s(0, 0);
  const Vec2 g(400, 400);
  const double straight = 400.0 * std::sqrt(2.0);
  const PlanResult a = planAStar(ctx, s, g);
  const PlanResult th = planThetaStar(ctx, s, g);
  CHECK(a.found);
  CHECK(th.found);
  CHECK_NEAR(a.length_m, straight, 1e-6);
  CHECK_NEAR(th.length_m, straight, 1e-6);
  CHECK(th.path.size() < a.path.size());
  CHECK(a.nodes > 0);
}

TEST(planners_route_around_a_circle) {
  const Terrain t = flatTerrain(41, 10.0);
  ZoneSet zones;
  zones.addCircle(Vec2(200, 200), 60.0);
  const PlanningContext ctx(t, zones, PlannerConfig());
  const Vec2 s(0, 0);
  const Vec2 g(400, 400);
  const double straight = distance(s, g);

  const PlanResult a = planAStar(ctx, s, g);
  const PlanResult th = planThetaStar(ctx, s, g);
  const PlanResult rrt = planRrtStar(ctx, s, g);
  CHECK(a.found && th.found && rrt.found);
  for (const PlanResult* r : {&a, &th, &rrt}) {
    CHECK(pathAvoidsZones(zones, r->path, 0.0));
    CHECK(r->length_m >= straight - 1e-6);
    CHECK_NEAR(r->path.front().x, 0.0, 1e-12);
    CHECK_NEAR(r->path.back().y, 400.0, 1e-12);
  }
  CHECK(th.length_m <= straight * 1.10);
  CHECK(a.length_m <= straight * 1.25);
  CHECK(rrt.length_m <= straight * 1.6);
  CHECK(th.length_m <= a.length_m * 1.02);
}

TEST(planners_on_rugged_terrain_with_zones) {
  const Terrain t = hilly(6, 11);
  ZoneSet zones;
  zones.addCircle(Vec2(960, 960), 220.0);
  zones.addPolygon({Vec2(300, 900), Vec2(700, 1000), Vec2(650, 1500), Vec2(250, 1400)});
  PlannerConfig cfg;
  cfg.zone_margin_m = 20.0;
  const PlanningContext ctx(t, zones, cfg);
  const Vec2 s(150, 150);
  const Vec2 g(1750, 1750);
  const double straight = distance(s, g);
  for (const PlannerKind kind : {PlannerKind::AStar, PlannerKind::ThetaStar, PlannerKind::RrtStar}) {
    const PlanResult r = plan(kind, ctx, s, g);
    CHECK(r.found);
    CHECK(pathAvoidsZones(zones, r.path, 19.9));
    CHECK(r.length_m >= straight - 1e-6);
    CHECK(r.length_m <= 2.5 * straight);
    CHECK(r.cost >= r.length_m);
    CHECK(r.runtime_ms >= 0.0);
  }
}

TEST(blocked_endpoints_and_unreachable_goals_fail_cleanly) {
  const Terrain t = flatTerrain(21, 10.0);
  ZoneSet zones;
  zones.addCircle(Vec2(100, 100), 30.0);
  const PlanningContext ctx(t, zones, PlannerConfig());
  for (const PlannerKind kind : {PlannerKind::AStar, PlannerKind::ThetaStar, PlannerKind::RrtStar}) {
    CHECK(!plan(kind, ctx, Vec2(100, 100), Vec2(190, 190)).found);   // start inside the zone
    CHECK(!plan(kind, ctx, Vec2(10, 10), Vec2(500, 500)).found);     // goal off the map
  }

  // A zone that walls off the whole map leaves no route.
  ZoneSet wall;
  wall.addPolygon({Vec2(90, -10), Vec2(110, -10), Vec2(110, 210), Vec2(90, 210)});
  const PlanningContext ctx2(t, wall, PlannerConfig());
  RrtParams quick;
  quick.max_iterations = 400;
  CHECK(!planAStar(ctx2, Vec2(10, 100), Vec2(190, 100)).found);
  CHECK(!planThetaStar(ctx2, Vec2(10, 100), Vec2(190, 100)).found);
  CHECK(!planRrtStar(ctx2, Vec2(10, 100), Vec2(190, 100), quick).found);
}

TEST(altitude_ceiling_blocks_high_ground) {
  const Terrain t = hilly(5, 3);
  const ZoneSet zones;
  PlannerConfig cfg;
  cfg.agl_m = 50.0;
  cfg.max_altitude_m = 10.0;  // below any terrain + clearance
  const PlanningContext ctx(t, zones, cfg);
  CHECK(!planAStar(ctx, Vec2(50, 50), Vec2(900, 900)).found);
}

TEST(smoothing_never_makes_paths_worse) {
  const Terrain t = hilly(6, 5);
  ZoneSet zones;
  zones.addCircle(Vec2(900, 900), 250.0);
  const PlanningContext ctx(t, zones, PlannerConfig());
  const Vec2 s(100, 100);
  const Vec2 g(1800, 1800);
  for (const PlannerKind kind : {PlannerKind::AStar, PlannerKind::RrtStar}) {
    const PlanResult r = plan(kind, ctx, s, g);
    CHECK(r.found);
    const std::vector<Vec2> smooth = smoothPath(ctx, r.path);
    CHECK(smooth.size() <= r.path.size());
    CHECK(pathLength(smooth) <= r.length_m + 1e-6);
    CHECK(ctx.pathCost(smooth) <= r.cost + 1e-6);
    CHECK(pathAvoidsZones(zones, smooth, 0.0));
    CHECK_NEAR(smooth.front().x, s.x, 1e-12);
    CHECK_NEAR(smooth.back().x, g.x, 1e-12);
  }
}

TEST(rrt_is_deterministic_for_a_seed) {
  const Terrain t = flatTerrain(41, 10.0);
  ZoneSet zones;
  zones.addCircle(Vec2(200, 200), 60.0);
  const PlanningContext ctx(t, zones, PlannerConfig());
  RrtParams p;
  p.seed = 42;
  p.max_iterations = 1500;
  const PlanResult a = planRrtStar(ctx, Vec2(0, 0), Vec2(400, 400), p);
  const PlanResult b = planRrtStar(ctx, Vec2(0, 0), Vec2(400, 400), p);
  CHECK(a.found && b.found);
  CHECK(a.path.size() == b.path.size());
  CHECK_NEAR(a.length_m, b.length_m, 1e-9);
}

TEST(climb_weight_changes_the_route) {
  // A ridge across the map with a low saddle at the far side: climbing is expensive.
  const std::size_t n = 41;
  std::vector<float> data(n * n, 0.0F);
  for (std::size_t r = 0; r < n; ++r) {
    for (std::size_t c = 18; c < 23; ++c) {
      if (r < 35) data[r * n + c] = 30.0F;  // ridge, with a gap at the north end
    }
  }
  const Terrain t(n, n, 10.0, 10.0, LatLon(47.0, 8.0), 47.0, data);
  const ZoneSet zones;
  PlannerConfig flat;
  flat.climb_weight = 0.0;
  flat.terrain_weight = 0.0;
  PlannerConfig costly;
  costly.climb_weight = 20.0;
  costly.terrain_weight = 0.0;
  const PlanningContext cheap(t, zones, flat);
  const PlanningContext avoid(t, zones, costly);
  const PlanResult direct = planThetaStar(cheap, Vec2(10, 20), Vec2(390, 20));
  const PlanResult detour = planThetaStar(avoid, Vec2(10, 20), Vec2(390, 20));
  CHECK(direct.found && detour.found);
  CHECK(detour.length_m > direct.length_m + 100.0);
}

TEST(pipeline_produces_valid_missions) {
  const Terrain t = hilly(6, 21);
  ZoneSet zones;
  zones.addCircle(Vec2(900, 1000), 240.0);
  zones.addPolygon({Vec2(1300, 300), Vec2(1700, 400), Vec2(1600, 800), Vec2(1250, 700)});
  PlannerConfig pc;
  pc.zone_margin_m = 10.0;
  ProfileConfig prof;
  prof.agl_m = 60.0;
  prof.max_climb_gradient = 0.3;
  prof.max_descent_gradient = 0.4;
  RrtParams rrt;
  rrt.seed = 5;
  const Vec2 s(150, 150);
  const Vec2 g(1750, 1750);

  for (const PlannerKind kind : {PlannerKind::AStar, PlannerKind::ThetaStar, PlannerKind::RrtStar}) {
    const MissionPlan mp = planMission(t, zones, s, g, kind, pc, prof, rrt, true);
    CHECK(mp.result.found);
    CHECK(pathAvoidsZones(zones, mp.path, 0.0));
    CHECK(mp.profile.min_agl >= prof.agl_m - 1e-6);
    CHECK(minClearance(t, mp.profile) >= prof.agl_m - 1e-6);
    CHECK(mp.profile.max_climb <= prof.max_climb_gradient + 1e-9);
    CHECK(mp.profile.max_descent <= prof.max_descent_gradient + 1e-9);
    CHECK(mp.mission.items.size() >= 2);
    CHECK(mp.mission.items.size() <= mp.profile.points.size());
    for (const MissionItem& it : mp.mission.items) {
      const double ground = t.elevation(t.toXY(LatLon(it.lat, it.lon)));
      CHECK(it.alt - ground >= prof.agl_m - 1e-3);
    }
    CHECK_NEAR(distance(mp.path.front(), s), 0.0, 1e-9);
    CHECK_NEAR(distance(mp.path.back(), g), 0.0, 1e-9);
  }
}

TEST(planner_names) {
  CHECK(parsePlannerKind("A*") == PlannerKind::AStar);
  CHECK(parsePlannerKind("Theta*") == PlannerKind::ThetaStar);
  CHECK(parsePlannerKind("rrtstar") == PlannerKind::RrtStar);
  CHECK(plannerName(PlannerKind::ThetaStar) == "thetastar");
  CHECK_THROWS(parsePlannerKind("dijkstra"));
}

TEST_MAIN()
