#include "uav/planner.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <queue>
#include <random>
#include <utility>

#include "uav/common.hpp"

namespace uav {

namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

double toD(std::size_t v) { return static_cast<double>(v); }

constexpr std::size_t kNone = static_cast<std::size_t>(-1);

}  // namespace

std::string plannerName(PlannerKind kind) {
  switch (kind) {
    case PlannerKind::AStar: return "astar";
    case PlannerKind::ThetaStar: return "thetastar";
    case PlannerKind::RrtStar: return "rrtstar";
  }
  return "unknown";
}

PlannerKind parsePlannerKind(const std::string& name) {
  std::string n;
  for (const char ch : name) n.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  if (n == "astar" || n == "a*") return PlannerKind::AStar;
  if (n == "thetastar" || n == "theta*" || n == "theta") return PlannerKind::ThetaStar;
  if (n == "rrtstar" || n == "rrt*" || n == "rrt") return PlannerKind::RrtStar;
  throw Error("unknown planner '" + name + "' (use astar, thetastar or rrtstar)");
}

PlanningContext::PlanningContext(const Terrain& terrain, const ZoneSet& zones, PlannerConfig config)
    : terrain_(terrain), zones_(zones), config_(config) {
  if (terrain.cols() < 2) throw Error("planner: terrain is empty");
  sampleStep_ = std::min(terrain.cellX(), terrain.cellY());
  zMin_ = terrain.minElevation();
  zSpan_ = std::max(terrain.maxElevation() - zMin_, 1.0);
}

bool PlanningContext::pointFree(Vec2 p) const {
  if (!terrain_.contains(p)) return false;
  if (zones_.blocked(p, config_.zone_margin_m)) return false;
  return !(terrain_.elevation(p) + config_.agl_m > config_.max_altitude_m);
}

bool PlanningContext::segmentFree(Vec2 a, Vec2 b) const {
  if (!pointFree(a) || !pointFree(b)) return false;
  if (zones_.segmentBlocked(a, b, config_.zone_margin_m)) return false;
  if (std::isfinite(config_.max_altitude_m)) {
    const double len = distance(a, b);
    const auto n = static_cast<std::size_t>(std::ceil(len / sampleStep_));
    for (std::size_t i = 1; i < n; ++i) {
      const Vec2 q = a + (b - a) * (toD(i) / toD(n));
      if (terrain_.elevation(q) + config_.agl_m > config_.max_altitude_m) return false;
    }
  }
  return true;
}

double PlanningContext::edgeCost(Vec2 a, Vec2 b) const {
  const double len = distance(a, b);
  if (len <= 0.0) return 0.0;
  const std::size_t n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(len / sampleStep_)));
  double cost = 0.0;
  Vec2 prev = a;
  double hPrev = terrain_.elevation(a);
  for (std::size_t i = 1; i <= n; ++i) {
    const Vec2 q = a + (b - a) * (toD(i) / toD(n));
    const double h = terrain_.elevation(q);
    const double dh = h - hPrev;
    const double dl = distance(prev, q);
    const double norm = std::clamp((0.5 * (h + hPrev) - zMin_) / zSpan_, 0.0, 1.0);
    cost += std::sqrt(dl * dl + dh * dh) + config_.climb_weight * std::max(dh, 0.0) +
            config_.terrain_weight * dl * norm;
    prev = q;
    hPrev = h;
  }
  return cost;
}

double PlanningContext::pathCost(const std::vector<Vec2>& path) const {
  double c = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) c += edgeCost(path[i - 1], path[i]);
  return c;
}

double pathLength(const std::vector<Vec2>& path) {
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) len += distance(path[i - 1], path[i]);
  return len;
}

namespace {

PlanResult finish(PlanResult res, const PlanningContext& ctx, Clock::time_point t0) {
  if (res.found) {
    res.length_m = pathLength(res.path);
    res.cost = ctx.pathCost(res.path);
  }
  res.runtime_ms = elapsedMs(t0);
  return res;
}

bool endpointsOk(const PlanningContext& ctx, Vec2 start, Vec2 goal, PlanResult& res) {
  if (!ctx.pointFree(start)) {
    res.message = "start is outside the map, inside a no-fly zone or above the ceiling";
    return false;
  }
  if (!ctx.pointFree(goal)) {
    res.message = "goal is outside the map, inside a no-fly zone or above the ceiling";
    return false;
  }
  return true;
}

PlanResult gridSearch(const PlanningContext& ctx, Vec2 start, Vec2 goal, bool theta) {
  const auto t0 = Clock::now();
  PlanResult res;
  res.planner = theta ? "thetastar" : "astar";
  if (!endpointsOk(ctx, start, goal, res)) return finish(res, ctx, t0);

  const Terrain& t = ctx.terrain();
  const std::size_t cols = t.cols();
  const std::size_t rows = t.rows();
  const std::size_t total = cols * rows;

  auto snap = [&](Vec2 p) {
    const long c = std::clamp<long>(std::lround(p.x / t.cellX()), 0, static_cast<long>(cols) - 1);
    const long r = std::clamp<long>(std::lround(p.y / t.cellY()), 0, static_cast<long>(rows) - 1);
    return static_cast<std::size_t>(r) * cols + static_cast<std::size_t>(c);
  };
  const std::size_t sIdx = snap(start);
  const std::size_t gIdx = snap(goal);

  if (sIdx == gIdx) {
    if (ctx.segmentFree(start, goal)) {
      res.found = true;
      res.path = {start, goal};
      res.nodes = 1;
    } else {
      res.message = "start and goal share a cell but the direct segment is blocked";
    }
    return finish(res, ctx, t0);
  }

  auto pos = [&](std::size_t i) {
    if (i == sIdx) return start;
    if (i == gIdx) return goal;
    return Vec2(toD(i % cols) * t.cellX(), toD(i / cols) * t.cellY());
  };

  std::vector<double> g(total, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(total, kNone);
  std::vector<char> closed(total, 0);
  std::vector<signed char> freeCache(total, -1);
  auto cellFree = [&](std::size_t i) {
    if (freeCache[i] < 0) freeCache[i] = ctx.pointFree(pos(i)) ? 1 : 0;
    return freeCache[i] == 1;
  };

  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  g[sIdx] = 0.0;
  parent[sIdx] = sIdx;
  open.emplace(distance(start, goal), sIdx);

  static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool reached = false;
  while (!open.empty()) {
    const std::size_t cur = open.top().second;
    open.pop();
    if (closed[cur] != 0) continue;
    closed[cur] = 1;
    ++res.nodes;
    if (cur == gIdx) {
      reached = true;
      break;
    }
    const long cc = static_cast<long>(cur % cols);
    const long cr = static_cast<long>(cur / cols);
    const Vec2 curPos = pos(cur);
    for (std::size_t k = 0; k < 8; ++k) {
      const long nc = cc + kDx[k];
      const long nr = cr + kDy[k];
      if (nc < 0 || nr < 0 || nc >= static_cast<long>(cols) || nr >= static_cast<long>(rows)) continue;
      const std::size_t ni = static_cast<std::size_t>(nr) * cols + static_cast<std::size_t>(nc);
      if (closed[ni] != 0 || !cellFree(ni)) continue;
      const Vec2 nPos = pos(ni);
      if (!ctx.segmentFree(curPos, nPos)) continue;

      double cand = g[cur] + ctx.edgeCost(curPos, nPos);
      std::size_t candParent = cur;
      if (theta && parent[cur] != cur) {
        const Vec2 pPos = pos(parent[cur]);
        if (ctx.segmentFree(pPos, nPos)) {
          const double viaParent = g[parent[cur]] + ctx.edgeCost(pPos, nPos);
          if (viaParent <= cand + 1e-9) {
            cand = viaParent;
            candParent = parent[cur];
          }
        }
      }
      if (cand < g[ni]) {
        g[ni] = cand;
        parent[ni] = candParent;
        open.emplace(cand + distance(nPos, goal), ni);
      }
    }
  }

  if (!reached) {
    res.message = "no collision-free path found";
    return finish(res, ctx, t0);
  }
  std::vector<Vec2> path;
  for (std::size_t i = gIdx;; i = parent[i]) {
    path.push_back(pos(i));
    if (parent[i] == i) break;
  }
  std::reverse(path.begin(), path.end());
  res.found = true;
  res.path = std::move(path);
  return finish(res, ctx, t0);
}

class Rng {
 public:
  explicit Rng(std::uint64_t seed) : engine_(seed) {}
  double uniform() { return static_cast<double>(engine_() >> 11U) * (1.0 / 9007199254740992.0); }

 private:
  std::mt19937_64 engine_;
};

struct TreeNode {
  Vec2 p;
  std::size_t parent = kNone;
  double cost = 0.0;
};

}  // namespace

PlanResult planAStar(const PlanningContext& ctx, Vec2 start, Vec2 goal) {
  return gridSearch(ctx, start, goal, false);
}

PlanResult planThetaStar(const PlanningContext& ctx, Vec2 start, Vec2 goal) {
  return gridSearch(ctx, start, goal, true);
}

PlanResult planRrtStar(const PlanningContext& ctx, Vec2 start, Vec2 goal, const RrtParams& params) {
  const auto t0 = Clock::now();
  PlanResult res;
  res.planner = "rrtstar";
  if (!endpointsOk(ctx, start, goal, res)) return finish(res, ctx, t0);

  const Terrain& terr = ctx.terrain();
  const double width = terr.widthM();
  const double height = terr.heightM();
  const double step = params.step_m > 0.0 ? params.step_m : 0.04 * std::max(width, height);
  const double nearRadius = 2.5 * step;
  Rng rng(params.seed);

  std::vector<TreeNode> tree;
  std::vector<std::vector<std::size_t>> children;
  tree.push_back({start, kNone, 0.0});
  children.emplace_back();

  auto propagate = [&](std::size_t root, double delta) {
    std::vector<std::size_t> stack(children[root].begin(), children[root].end());
    while (!stack.empty()) {
      const std::size_t i = stack.back();
      stack.pop_back();
      tree[i].cost += delta;
      stack.insert(stack.end(), children[i].begin(), children[i].end());
    }
  };

  for (std::size_t iter = 0; iter < params.max_iterations; ++iter) {
    const Vec2 sample = rng.uniform() < params.goal_bias ? goal : Vec2(rng.uniform() * width, rng.uniform() * height);

    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < tree.size(); ++i) {
      const double d = distance(tree[i].p, sample);
      if (d < best) {
        best = d;
        nearest = i;
      }
    }
    Vec2 q = sample;
    if (best > step) q = tree[nearest].p + (sample - tree[nearest].p) * (step / best);
    if (!ctx.pointFree(q) || !ctx.segmentFree(tree[nearest].p, q)) continue;

    std::vector<std::size_t> nearIdx;
    for (std::size_t i = 0; i < tree.size(); ++i) {
      if (distance(tree[i].p, q) <= nearRadius) nearIdx.push_back(i);
    }

    std::size_t bestParent = nearest;
    double bestCost = tree[nearest].cost + ctx.edgeCost(tree[nearest].p, q);
    for (const std::size_t i : nearIdx) {
      if (i == nearest) continue;
      const double c = tree[i].cost + ctx.edgeCost(tree[i].p, q);
      if (c < bestCost && ctx.segmentFree(tree[i].p, q)) {
        bestCost = c;
        bestParent = i;
      }
    }

    const std::size_t newIdx = tree.size();
    tree.push_back({q, bestParent, bestCost});
    children.emplace_back();
    children[bestParent].push_back(newIdx);

    for (const std::size_t i : nearIdx) {
      if (i == bestParent || i == 0) continue;
      const double c = bestCost + ctx.edgeCost(q, tree[i].p);
      if (c + 1e-9 < tree[i].cost && ctx.segmentFree(q, tree[i].p)) {
        const double delta = c - tree[i].cost;
        auto& siblings = children[tree[i].parent];
        siblings.erase(std::remove(siblings.begin(), siblings.end(), i), siblings.end());
        tree[i].parent = newIdx;
        tree[i].cost = c;
        children[newIdx].push_back(i);
        propagate(i, delta);
      }
    }
  }

  // Pick the cheapest connection to the goal among all nodes that can see it.
  std::size_t goalParent = kNone;
  double goalCost = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < tree.size(); ++i) {
    if (distance(tree[i].p, goal) > step * 1.5 && i != 0) continue;
    if (tree[i].cost >= goalCost) continue;
    if (!ctx.segmentFree(tree[i].p, goal)) continue;
    const double c = tree[i].cost + ctx.edgeCost(tree[i].p, goal);
    if (c < goalCost) {
      goalCost = c;
      goalParent = i;
    }
  }
  res.nodes = tree.size();
  if (goalParent == kNone) {
    res.message = "no collision-free path found within the iteration budget";
    return finish(res, ctx, t0);
  }
  std::vector<Vec2> path{goal};
  for (std::size_t i = goalParent; i != kNone; i = tree[i].parent) path.push_back(tree[i].p);
  std::reverse(path.begin(), path.end());
  res.found = true;
  res.path = std::move(path);
  return finish(res, ctx, t0);
}

PlanResult plan(PlannerKind kind, const PlanningContext& ctx, Vec2 start, Vec2 goal, const RrtParams& rrt) {
  switch (kind) {
    case PlannerKind::AStar: return planAStar(ctx, start, goal);
    case PlannerKind::ThetaStar: return planThetaStar(ctx, start, goal);
    case PlannerKind::RrtStar: return planRrtStar(ctx, start, goal, rrt);
  }
  throw Error("unknown planner kind");
}

std::vector<Vec2> smoothPath(const PlanningContext& ctx, const std::vector<Vec2>& path) {
  if (path.size() < 3) return path;
  std::vector<double> prefix(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) prefix[i] = prefix[i - 1] + ctx.edgeCost(path[i - 1], path[i]);

  std::vector<Vec2> out{path.front()};
  std::size_t i = 0;
  while (i + 1 < path.size()) {
    std::size_t next = i + 1;
    for (std::size_t j = path.size() - 1; j > i + 1; --j) {
      if (!ctx.segmentFree(path[i], path[j])) continue;
      if (ctx.edgeCost(path[i], path[j]) <= prefix[j] - prefix[i] + 1e-9) {
        next = j;
        break;
      }
    }
    out.push_back(path[next]);
    i = next;
  }
  return out;
}

}  // namespace uav
