#pragma once

#include <string>
#include <vector>

#include "uav/geo.hpp"
#include "uav/terrain.hpp"
#include "uav/zones.hpp"

namespace uav {

/// ASCII picture of the terrain (shaded by elevation), no-fly zones ('X'), the path ('o'),
/// the start ('S') and the goal ('G'). North is up.
[[nodiscard]] std::string renderAscii(const Terrain& terrain, const ZoneSet& zones, const std::vector<Vec2>& path,
                                      Vec2 start, Vec2 goal, int width = 72);

}  // namespace uav
