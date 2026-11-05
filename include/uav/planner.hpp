#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "uav/geo.hpp"
#include "uav/terrain.hpp"
#include "uav/zones.hpp"

namespace uav {

enum class PlannerKind { AStar, ThetaStar, RrtStar };

[[nodiscard]] std::string plannerName(PlannerKind kind);
/// Accepts "astar", "a*", "thetastar", "theta*", "rrtstar", "rrt*" (case-insensitive).
[[nodiscard]] PlannerKind parsePlannerKind(const std::string& name);

struct PlannerConfig {
  double agl_m = 50.0;             ///< clearance used for the ceiling check
  double zone_margin_m = 0.0;      ///< extra stand-off distance around no-fly zones
  double climb_weight = 1.0;       ///< extra cost per metre of climb
  double terrain_weight = 0.25;    ///< penalty per metre flown over high ground (0..1 scaled)
  double max_altitude_m = std::numeric_limits<double>::infinity();  ///< MSL ceiling
};

struct RrtParams {
  std::size_t max_iterations = 3000;
  double step_m = 0.0;             ///< 0 selects 4% of the longer map side
  double goal_bias = 0.08;
  std::uint64_t seed = 1;
};

struct PlanResult {
  bool found = false;
  std::string planner;
  std::string message;
  std::vector<Vec2> path;
  double length_m = 0.0;           ///< horizontal length
  double cost = 0.0;               ///< planner cost (3D length + climb + terrain penalties)
  double runtime_ms = 0.0;
  std::size_t nodes = 0;           ///< expanded nodes (A*, Theta*) or tree size (RRT*)
};

/// Cost model and collision checks shared by all planners.
class PlanningContext {
 public:
  PlanningContext(const Terrain& terrain, const ZoneSet& zones, PlannerConfig config);

  [[nodiscard]] const Terrain& terrain() const { return terrain_; }
  [[nodiscard]] const ZoneSet& zones() const { return zones_; }
  [[nodiscard]] const PlannerConfig& config() const { return config_; }

  [[nodiscard]] bool pointFree(Vec2 p) const;
  [[nodiscard]] bool segmentFree(Vec2 a, Vec2 b) const;
  [[nodiscard]] double edgeCost(Vec2 a, Vec2 b) const;
  [[nodiscard]] double pathCost(const std::vector<Vec2>& path) const;

 private:
  const Terrain& terrain_;
  const ZoneSet& zones_;
  PlannerConfig config_;
  double sampleStep_ = 1.0;
  double zMin_ = 0.0;
  double zSpan_ = 1.0;
};

[[nodiscard]] double pathLength(const std::vector<Vec2>& path);

[[nodiscard]] PlanResult planAStar(const PlanningContext& ctx, Vec2 start, Vec2 goal);
[[nodiscard]] PlanResult planThetaStar(const PlanningContext& ctx, Vec2 start, Vec2 goal);
[[nodiscard]] PlanResult planRrtStar(const PlanningContext& ctx, Vec2 start, Vec2 goal,
                                     const RrtParams& params = RrtParams());
[[nodiscard]] PlanResult plan(PlannerKind kind, const PlanningContext& ctx, Vec2 start, Vec2 goal,
                              const RrtParams& rrt = RrtParams());

/// Greedy shortcutting with line-of-sight checks; never increases the planner cost.
[[nodiscard]] std::vector<Vec2> smoothPath(const PlanningContext& ctx, const std::vector<Vec2>& path);

}  // namespace uav
