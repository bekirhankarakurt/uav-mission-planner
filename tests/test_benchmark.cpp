#include <string>
#include <vector>

#include "test_util.hpp"
#include "uav/benchmark.hpp"

using namespace uav;

namespace {

BenchmarkConfig smallConfig() {
  BenchmarkConfig cfg;
  cfg.maps = 2;
  cfg.exponent = 5;
  cfg.seed = 3;
  cfg.rrt_iterations = 800;
  return cfg;
}

}  // namespace

TEST(benchmark_covers_all_planners) {
  const std::vector<BenchmarkRow> rows = runBenchmark(smallConfig());
  CHECK(rows.size() == 3);
  CHECK(rows[0].planner == "astar");
  CHECK(rows[1].planner == "thetastar");
  CHECK(rows[2].planner == "rrtstar");
  CHECK(rows[0].solved == 2);
  CHECK(rows[1].solved == 2);
  for (const BenchmarkRow& r : rows) {
    CHECK(r.maps == 2);
    if (r.solved > 0) {
      CHECK(r.mean_stretch >= 1.0 - 1e-9);
      CHECK(r.mean_length_m > 0.0);
      CHECK(r.mean_nodes > 0.0);
      CHECK(r.mean_runtime_ms >= 0.0);
    }
  }
  CHECK(rows[1].mean_length_m <= rows[0].mean_length_m * 1.02);
}

TEST(benchmark_is_deterministic) {
  const std::vector<BenchmarkRow> a = runBenchmark(smallConfig());
  const std::vector<BenchmarkRow> b = runBenchmark(smallConfig());
  for (std::size_t i = 0; i < a.size(); ++i) {
    CHECK_NEAR(a[i].mean_length_m, b[i].mean_length_m, 1e-9);
    CHECK_NEAR(a[i].mean_nodes, b[i].mean_nodes, 1e-9);
  }
}

TEST(benchmark_formatting) {
  const std::vector<BenchmarkRow> rows = runBenchmark(smallConfig());
  const std::string md = benchmarkMarkdown(rows);
  CHECK(md.find("| Planner |") == 0);
  CHECK(md.find("thetastar") != std::string::npos);
  const std::string csv = benchmarkCsv(rows);
  std::size_t lines = 0;
  for (const char ch : csv) {
    if (ch == '\n') ++lines;
  }
  CHECK(lines == 4);
}

TEST(benchmark_rejects_empty_runs) {
  BenchmarkConfig cfg = smallConfig();
  cfg.maps = 0;
  CHECK_THROWS(runBenchmark(cfg));
}

TEST_MAIN()
