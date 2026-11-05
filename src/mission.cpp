#include "uav/mission.hpp"

#include <utility>

#include "uav/common.hpp"

namespace uav {

Mission buildMission(const Terrain& terrain, const Profile& profile) {
  if (profile.points.empty()) throw Error("mission: empty profile");
  Mission m;
  const LatLon home = terrain.toLatLon(profile.points.front().p);
  m.homeLat = home.lat;
  m.homeLon = home.lon;
  m.homeAlt = profile.points.front().ground;
  for (const ProfilePoint& pp : profile.points) {
    const LatLon ll = terrain.toLatLon(pp.p);
    MissionItem item;
    item.lat = ll.lat;
    item.lon = ll.lon;
    item.alt = pp.alt;
    m.items.push_back(item);
  }
  return m;
}

MissionPlan planMission(const Terrain& terrain, const ZoneSet& zones, Vec2 start, Vec2 goal,
                        PlannerKind kind, const PlannerConfig& planner, const ProfileConfig& profile,
                        const RrtParams& rrt, bool smooth) {
  PlannerConfig pc = planner;
  pc.agl_m = profile.agl_m;
  if (profile.max_altitude_m > 0.0) pc.max_altitude_m = profile.max_altitude_m;
  const PlanningContext ctx(terrain, zones, pc);

  MissionPlan out;
  out.result = plan(kind, ctx, start, goal, rrt);
  if (!out.result.found) return out;
  out.path = out.result.path;
  if (smooth) {
    out.path = smoothPath(ctx, out.path);
    out.result.length_m = pathLength(out.path);
    out.result.cost = ctx.pathCost(out.path);
  }
  out.profile = buildProfile(terrain, out.path, profile);
  out.exported = decimateProfile(out.profile);
  out.mission = buildMission(terrain, out.exported);
  return out;
}

}  // namespace uav
