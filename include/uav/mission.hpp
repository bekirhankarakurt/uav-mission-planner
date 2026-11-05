#pragma once

#include <vector>

#include "uav/planner.hpp"
#include "uav/profile.hpp"
#include "uav/terrain.hpp"
#include "uav/zones.hpp"

namespace uav {

constexpr int kMavCmdNavWaypoint = 16;
constexpr int kMavFrameGlobal = 0;               ///< altitude is MSL
constexpr int kMavFrameGlobalRelativeAlt = 3;

/// One MAVLink-style mission item (MISSION_ITEM semantics, degrees / metres).
struct MissionItem {
  int command = kMavCmdNavWaypoint;
  int frame = kMavFrameGlobal;
  double param1 = 0.0;
  double param2 = 0.0;
  double param3 = 0.0;
  double param4 = 0.0;
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  bool autocontinue = true;
};

struct Mission {
  double homeLat = 0.0;
  double homeLon = 0.0;
  double homeAlt = 0.0;
  std::vector<MissionItem> items;
};

/// Turn an altitude profile into waypoint items (MSL altitudes) with the home at the start.
[[nodiscard]] Mission buildMission(const Terrain& terrain, const Profile& profile);

struct MissionPlan {
  PlanResult result;
  std::vector<Vec2> path;     ///< final planar path (after optional smoothing)
  Profile profile;            ///< full-resolution altitude profile
  Profile exported;           ///< decimated profile used for the mission
  Mission mission;
};

/// Plan, optionally smooth, add the altitude profile and build the mission.
/// `result.found` is false (and the rest empty) when no path exists.
[[nodiscard]] MissionPlan planMission(const Terrain& terrain, const ZoneSet& zones, Vec2 start, Vec2 goal,
                                      PlannerKind kind, const PlannerConfig& planner,
                                      const ProfileConfig& profile, const RrtParams& rrt, bool smooth);

}  // namespace uav
