#pragma once

#include <string>

#include "uav/mission.hpp"
#include "uav/terrain.hpp"
#include "uav/zones.hpp"

namespace uav {

/// QGroundControl / ArduPilot "QGC WPL 110" text file (item 0 is the home position).
[[nodiscard]] std::string toWpl(const Mission& mission);
[[nodiscard]] Mission parseWpl(const std::string& text);

/// QGroundControl .plan JSON. No-fly zones are written as exclusion geofences.
[[nodiscard]] std::string toQgcPlan(const Mission& mission, const Terrain& terrain, const ZoneSet& zones);
[[nodiscard]] Mission parseQgcPlan(const std::string& text);

/// MAVLink MISSION_ITEM_INT style list as JSON (x / y are degrees * 1e7).
[[nodiscard]] std::string toMavlinkItemsJson(const Mission& mission);

/// GeoJSON FeatureCollection: the flight path (LineString, [lon, lat, alt]) and the zones.
[[nodiscard]] std::string toGeoJson(const Mission& mission, const Terrain& terrain, const ZoneSet& zones);
/// KML document with the flight path and the zones.
[[nodiscard]] std::string toKml(const Mission& mission, const Terrain& terrain, const ZoneSet& zones);

}  // namespace uav
