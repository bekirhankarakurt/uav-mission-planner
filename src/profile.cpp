#include "uav/profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "uav/common.hpp"

namespace uav {

namespace {

double toD(std::size_t v) { return static_cast<double>(v); }

void computeStats(const Terrain& terrain, Profile& prof, double ceiling) {
  prof.min_agl = minClearance(terrain, prof);
  prof.max_climb = 0.0;
  prof.max_descent = 0.0;
  prof.within_ceiling = true;
  for (std::size_t i = 0; i < prof.points.size(); ++i) {
    if (ceiling > 0.0 && prof.points[i].alt > ceiling + 1e-9) prof.within_ceiling = false;
    if (i == 0) continue;
    const double ds = prof.points[i].s - prof.points[i - 1].s;
    if (ds <= 1e-9) continue;
    const double grad = (prof.points[i].alt - prof.points[i - 1].alt) / ds;
    prof.max_climb = std::max(prof.max_climb, grad);
    prof.max_descent = std::max(prof.max_descent, -grad);
  }
}

}  // namespace

Profile buildProfile(const Terrain& terrain, const std::vector<Vec2>& path, const ProfileConfig& cfg) {
  if (path.size() < 2) throw Error("profile: path needs at least two points");
  if (cfg.max_climb_gradient <= 0.0 || cfg.max_descent_gradient <= 0.0) {
    throw Error("profile: gradient limits must be positive");
  }
  const double step = cfg.sample_m > 0.0 ? cfg.sample_m : std::min(terrain.cellX(), terrain.cellY());

  Profile prof;
  auto& pts = prof.points;
  {
    ProfilePoint first;
    first.p = path.front();
    first.vertex = true;
    pts.push_back(first);
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Vec2 a = path[i - 1];
    const Vec2 b = path[i];
    const std::size_t n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(distance(a, b) / step)));
    for (std::size_t k = 1; k <= n; ++k) {
      ProfilePoint pp;
      pp.p = a + (b - a) * (toD(k) / toD(n));
      pp.vertex = (k == n);
      pts.push_back(pp);
    }
  }

  const std::size_t m = pts.size();
  std::vector<double> required(m, 0.0);
  for (std::size_t i = 0; i < m; ++i) {
    pts[i].ground = terrain.elevation(pts[i].p);
    required[i] = pts[i].ground;
    if (i > 0) pts[i].s = pts[i - 1].s + distance(pts[i - 1].p, pts[i].p);
  }
  for (std::size_t i = 0; i + 1 < m; ++i) {
    const double mid = terrain.elevation((pts[i].p + pts[i + 1].p) * 0.5);
    required[i] = std::max(required[i], mid);
    required[i + 1] = std::max(required[i + 1], mid);
  }
  for (std::size_t i = 0; i < m; ++i) pts[i].alt = required[i] + cfg.agl_m;

  // Climb limit (backwards pass) then descent limit (forwards pass). Both only ever raise
  // the profile, so the clearance requirement stays satisfied.
  for (std::size_t i = m - 1; i > 0; --i) {
    const double ds = pts[i].s - pts[i - 1].s;
    pts[i - 1].alt = std::max(pts[i - 1].alt, pts[i].alt - cfg.max_climb_gradient * ds);
  }
  for (std::size_t i = 1; i < m; ++i) {
    const double ds = pts[i].s - pts[i - 1].s;
    pts[i].alt = std::max(pts[i].alt, pts[i - 1].alt - cfg.max_descent_gradient * ds);
  }

  computeStats(terrain, prof, cfg.max_altitude_m);
  return prof;
}

double minClearance(const Terrain& terrain, const Profile& profile) {
  double best = std::numeric_limits<double>::infinity();
  const auto& pts = profile.points;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    best = std::min(best, pts[i].alt - terrain.elevation(pts[i].p));
    if (i + 1 < pts.size()) {
      const Vec2 mid = (pts[i].p + pts[i + 1].p) * 0.5;
      best = std::min(best, 0.5 * (pts[i].alt + pts[i + 1].alt) - terrain.elevation(mid));
    }
  }
  return best;
}

Profile decimateProfile(const Profile& in) {
  const auto& pts = in.points;
  if (pts.size() <= 2) return in;
  std::vector<char> keep(pts.size(), 0);
  keep.front() = 1;
  keep.back() = 1;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (pts[i].vertex) keep[i] = 1;
  }

  auto refine = [&](std::size_t from, std::size_t to) {
    std::vector<std::pair<std::size_t, std::size_t>> stack{{from, to}};
    while (!stack.empty()) {
      const auto [lo, hi] = stack.back();
      stack.pop_back();
      if (hi - lo < 2) continue;
      const double span = pts[hi].s - pts[lo].s;
      std::size_t worst = lo;
      double worstGap = 1e-9;
      for (std::size_t k = lo + 1; k < hi; ++k) {
        const double f = span > 1e-12 ? (pts[k].s - pts[lo].s) / span : 0.0;
        const double chord = pts[lo].alt + (pts[hi].alt - pts[lo].alt) * f;
        const double gap = pts[k].alt - chord;  // positive: the chord would cut below the profile
        if (gap > worstGap) {
          worstGap = gap;
          worst = k;
        }
      }
      if (worst == lo) continue;
      keep[worst] = 1;
      stack.emplace_back(lo, worst);
      stack.emplace_back(worst, hi);
    }
  };

  std::size_t prev = 0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    if (keep[i] != 0) {
      refine(prev, i);
      prev = i;
    }
  }

  Profile out;
  out.min_agl = in.min_agl;
  out.max_climb = in.max_climb;
  out.max_descent = in.max_descent;
  out.within_ceiling = in.within_ceiling;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (keep[i] != 0) out.points.push_back(pts[i]);
  }
  return out;
}

}  // namespace uav
