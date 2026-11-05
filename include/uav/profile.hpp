#pragma once

#include <cstddef>
#include <vector>

#include "uav/geo.hpp"
#include "uav/terrain.hpp"

namespace uav {

struct ProfileConfig {
  double agl_m = 50.0;                ///< minimum clearance above ground
  double max_climb_gradient = 0.35;   ///< maximum rise / run
  double max_descent_gradient = 0.35; ///< maximum drop / run
  double sample_m = 0.0;              ///< sampling distance, 0 selects the smaller terrain cell size
  double max_altitude_m = 0.0;        ///< MSL ceiling, 0 disables the check
};

struct ProfilePoint {
  Vec2 p;
  double s = 0.0;        ///< distance along the path
  double ground = 0.0;   ///< terrain elevation (MSL)
  double alt = 0.0;      ///< flight altitude (MSL)
  bool vertex = false;   ///< true for the original path vertices
};

struct Profile {
  std::vector<ProfilePoint> points;
  double min_agl = 0.0;
  double max_climb = 0.0;      ///< steepest climb gradient actually flown
  double max_descent = 0.0;    ///< steepest descent gradient actually flown
  bool within_ceiling = true;
};

/// Terrain-following altitude profile: at least `agl_m` above the ground everywhere and
/// never steeper than the configured climb / descent gradients.
[[nodiscard]] Profile buildProfile(const Terrain& terrain, const std::vector<Vec2>& path,
                                   const ProfileConfig& config);

/// Smallest height above ground over all profile points and segment midpoints.
[[nodiscard]] double minClearance(const Terrain& terrain, const Profile& profile);

/// Drop intermediate samples that lie on or below the chord between kept points, so the
/// decimated profile is never lower than the original one.
[[nodiscard]] Profile decimateProfile(const Profile& profile);

}  // namespace uav
