#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "uav/planner.hpp"

namespace uav {

struct BenchmarkConfig {
  int maps = 10;                    ///< number of seeded synthetic maps
  int exponent = 7;                 ///< map size is 2^exponent + 1 samples per side
  std::uint64_t seed = 1;
  std::size_t rrt_iterations = 3000;
  PlannerConfig planner;
};

struct BenchmarkRow {
  std::string planner;
  int maps = 0;
  int solved = 0;
  double mean_length_m = 0.0;       ///< mean horizontal path length over solved maps
  double mean_stretch = 0.0;        ///< mean of length / straight-line distance
  double mean_cost = 0.0;
  double mean_runtime_ms = 0.0;
  double mean_nodes = 0.0;
};

/// Run A*, Theta* and RRT* on seeded synthetic maps with random circular no-fly zones.
[[nodiscard]] std::vector<BenchmarkRow> runBenchmark(const BenchmarkConfig& config);
[[nodiscard]] std::string benchmarkMarkdown(const std::vector<BenchmarkRow>& rows);
[[nodiscard]] std::string benchmarkCsv(const std::vector<BenchmarkRow>& rows);

}  // namespace uav
