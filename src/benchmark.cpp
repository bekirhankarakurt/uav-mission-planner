#include "uav/benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "uav/common.hpp"
#include "uav/terrain.hpp"
#include "uav/zones.hpp"

namespace uav {

namespace {

std::string fmt(double v, int decimals) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
  return buf;
}

double uniform(std::mt19937_64& rng) {
  return static_cast<double>(rng() >> 11U) * (1.0 / 9007199254740992.0);
}

}  // namespace

std::vector<BenchmarkRow> runBenchmark(const BenchmarkConfig& cfg) {
  if (cfg.maps < 1) throw Error("benchmark: need at least one map");
  const PlannerKind kinds[3] = {PlannerKind::AStar, PlannerKind::ThetaStar, PlannerKind::RrtStar};
  std::vector<BenchmarkRow> rows(3);
  for (std::size_t k = 0; k < 3; ++k) {
    rows[k].planner = plannerName(kinds[k]);
    rows[k].maps = cfg.maps;
  }

  for (int m = 0; m < cfg.maps; ++m) {
    SyntheticParams sp;
    sp.exponent = cfg.exponent;
    sp.seed = cfg.seed + static_cast<std::uint64_t>(m);
    const Terrain terrain = Terrain::synthetic(sp);
    const double w = terrain.widthM();
    const double h = terrain.heightM();
    const Vec2 start(0.08 * w, 0.08 * h);
    const Vec2 goal(0.92 * w, 0.92 * h);
    const double straight = distance(start, goal);

    std::mt19937_64 rng(sp.seed * 7919ULL + 17ULL);
    ZoneSet zones;
    const double clearOfEnds = cfg.planner.zone_margin_m + 2.0 * terrain.cellX();
    for (int placed = 0, tries = 0; placed < 5 && tries < 200; ++tries) {
      const Vec2 c((0.15 + 0.7 * uniform(rng)) * w, (0.15 + 0.7 * uniform(rng)) * h);
      const double r = (0.04 + 0.06 * uniform(rng)) * std::min(w, h);
      if (distance(c, start) < r + clearOfEnds || distance(c, goal) < r + clearOfEnds) continue;
      zones.addCircle(c, r);
      ++placed;
    }

    const PlanningContext ctx(terrain, zones, cfg.planner);
    RrtParams rrt;
    rrt.max_iterations = cfg.rrt_iterations;
    rrt.seed = sp.seed;
    for (std::size_t k = 0; k < 3; ++k) {
      const PlanResult r = plan(kinds[k], ctx, start, goal, rrt);
      if (!r.found) continue;
      BenchmarkRow& row = rows[k];
      ++row.solved;
      row.mean_length_m += r.length_m;
      row.mean_stretch += r.length_m / straight;
      row.mean_cost += r.cost;
      row.mean_runtime_ms += r.runtime_ms;
      row.mean_nodes += static_cast<double>(r.nodes);
    }
  }
  for (BenchmarkRow& row : rows) {
    if (row.solved > 0) {
      const double n = static_cast<double>(row.solved);
      row.mean_length_m /= n;
      row.mean_stretch /= n;
      row.mean_cost /= n;
      row.mean_runtime_ms /= n;
      row.mean_nodes /= n;
    }
  }
  return rows;
}

std::string benchmarkMarkdown(const std::vector<BenchmarkRow>& rows) {
  std::string out =
      "| Planner | Solved | Path length (m) | Length / straight | Cost | Runtime (ms) | Nodes |\n"
      "|---|---|---|---|---|---|---|\n";
  for (const BenchmarkRow& r : rows) {
    out += "| " + r.planner + " | " + std::to_string(r.solved) + "/" + std::to_string(r.maps) + " | " +
           fmt(r.mean_length_m, 1) + " | " + fmt(r.mean_stretch, 3) + " | " + fmt(r.mean_cost, 1) + " | " +
           fmt(r.mean_runtime_ms, 2) + " | " + fmt(r.mean_nodes, 0) + " |\n";
  }
  return out;
}

std::string benchmarkCsv(const std::vector<BenchmarkRow>& rows) {
  std::string out = "planner,solved,maps,mean_length_m,mean_stretch,mean_cost,mean_runtime_ms,mean_nodes\n";
  for (const BenchmarkRow& r : rows) {
    out += r.planner + "," + std::to_string(r.solved) + "," + std::to_string(r.maps) + "," +
           fmt(r.mean_length_m, 3) + "," + fmt(r.mean_stretch, 5) + "," + fmt(r.mean_cost, 3) + "," +
           fmt(r.mean_runtime_ms, 4) + "," + fmt(r.mean_nodes, 1) + "\n";
  }
  return out;
}

}  // namespace uav
