#include "uav/render.hpp"

#include <algorithm>
#include <cmath>

#include "uav/common.hpp"

namespace uav {

std::string renderAscii(const Terrain& terrain, const ZoneSet& zones, const std::vector<Vec2>& path, Vec2 start,
                        Vec2 goal, int width) {
  if (width < 8) throw Error("render: width must be at least 8");
  const auto w = static_cast<std::size_t>(width);
  const double aspect = terrain.heightM() / terrain.widthM();
  const auto h = static_cast<std::size_t>(std::max(4L, std::lround(static_cast<double>(width) * aspect * 0.5)));
  const double cellW = terrain.widthM() / static_cast<double>(w);
  const double cellH = terrain.heightM() / static_cast<double>(h);

  static const char kShade[] = " .:-=+*#%@";
  const double lo = terrain.minElevation();
  const double span = std::max(terrain.maxElevation() - lo, 1.0);

  std::vector<std::string> grid(h, std::string(w, ' '));
  auto cellOf = [&](Vec2 p, std::size_t& cx, std::size_t& cy) {
    const double fx = std::clamp(p.x / cellW, 0.0, static_cast<double>(w - 1));
    const double fy = std::clamp(p.y / cellH, 0.0, static_cast<double>(h - 1));
    cx = static_cast<std::size_t>(fx);
    cy = static_cast<std::size_t>(fy);
  };

  for (std::size_t y = 0; y < h; ++y) {
    for (std::size_t x = 0; x < w; ++x) {
      const Vec2 c((static_cast<double>(x) + 0.5) * cellW, (static_cast<double>(y) + 0.5) * cellH);
      if (zones.blocked(c, 0.0)) {
        grid[y][x] = 'X';
      } else {
        const double t = std::clamp((terrain.elevation(c) - lo) / span, 0.0, 0.999);
        grid[y][x] = kShade[static_cast<std::size_t>(t * 10.0)];
      }
    }
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double len = distance(path[i - 1], path[i]);
    const auto n = static_cast<std::size_t>(std::ceil(len / std::min(cellW, cellH))) * 2 + 1;
    for (std::size_t k = 0; k <= n; ++k) {
      std::size_t cx = 0;
      std::size_t cy = 0;
      cellOf(path[i - 1] + (path[i] - path[i - 1]) * (static_cast<double>(k) / static_cast<double>(n)), cx, cy);
      grid[cy][cx] = 'o';
    }
  }
  std::size_t cx = 0;
  std::size_t cy = 0;
  cellOf(start, cx, cy);
  grid[cy][cx] = 'S';
  cellOf(goal, cx, cy);
  grid[cy][cx] = 'G';

  std::string out;
  for (std::size_t row = h; row-- > 0;) {
    out += grid[row];
    out.push_back('\n');
  }
  return out;
}

}  // namespace uav
