#include "uav/export.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <utility>
#include <vector>

#include "uav/common.hpp"
#include "uav/json.hpp"

namespace uav {

namespace {

std::string fixed(double v, int decimals) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
  return buf;
}

std::vector<std::string> splitWhitespace(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (const char ch : line) {
    if (ch == ' ' || ch == '\t' || ch == '\r') {
      if (!cur.empty()) {
        out.push_back(cur);
        cur.clear();
      }
    } else {
      cur.push_back(ch);
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

double parseNumberToken(const std::string& tok, std::size_t lineNo) {
  char* end = nullptr;
  const double v = std::strtod(tok.c_str(), &end);
  if (end == tok.c_str() || *end != '\0') {
    throw Error("wpl: invalid number '" + tok + "' on line " + std::to_string(lineNo));
  }
  return v;
}

json::Value num(double v) { return json::Value::makeNumber(v); }

json::Value numberOrZero(const json::Value& v) { return v.isNumber() ? v : num(0.0); }

/// Zone outlines as geographic [lat, lon] rings; circles are approximated by a polygon.
std::vector<std::vector<LatLon>> zoneRings(const Terrain& terrain, const ZoneSet& zones) {
  std::vector<std::vector<LatLon>> rings;
  for (const ZoneSet::Circle& c : zones.circles()) {
    std::vector<LatLon> ring;
    constexpr int kSegments = 48;
    for (int i = 0; i < kSegments; ++i) {
      const double a = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSegments);
      ring.push_back(terrain.toLatLon(c.center + Vec2(std::cos(a), std::sin(a)) * c.radius));
    }
    rings.push_back(std::move(ring));
  }
  for (const ZoneSet::Polygon& poly : zones.polygons()) {
    std::vector<LatLon> ring;
    for (const Vec2 p : poly.points) ring.push_back(terrain.toLatLon(p));
    rings.push_back(std::move(ring));
  }
  return rings;
}

json::Value lonLatAlt(double lat, double lon, double alt) {
  json::Value c = json::Value::makeArray();
  c.push(num(lon));
  c.push(num(lat));
  c.push(num(alt));
  return c;
}

}  // namespace

std::string toWpl(const Mission& mission) {
  std::string out = "QGC WPL 110\n";
  auto line = [&](std::size_t seq, int current, int frame, int command, double p1, double p2, double p3,
                  double p4, double lat, double lon, double alt, bool autocontinue) {
    out += std::to_string(seq) + "\t" + std::to_string(current) + "\t" + std::to_string(frame) + "\t" +
           std::to_string(command) + "\t" + fixed(p1, 8) + "\t" + fixed(p2, 8) + "\t" + fixed(p3, 8) + "\t" +
           fixed(p4, 8) + "\t" + fixed(lat, 8) + "\t" + fixed(lon, 8) + "\t" + fixed(alt, 6) + "\t" +
           (autocontinue ? "1" : "0") + "\n";
  };
  line(0, 1, kMavFrameGlobal, kMavCmdNavWaypoint, 0, 0, 0, 0, mission.homeLat, mission.homeLon, mission.homeAlt, true);
  for (std::size_t i = 0; i < mission.items.size(); ++i) {
    const MissionItem& it = mission.items[i];
    line(i + 1, 0, it.frame, it.command, it.param1, it.param2, it.param3, it.param4, it.lat, it.lon, it.alt,
         it.autocontinue);
  }
  return out;
}

Mission parseWpl(const std::string& text) {
  std::istringstream in(text);
  std::string line;
  std::size_t lineNo = 0;
  bool header = false;
  bool haveHome = false;
  Mission m;
  while (std::getline(in, line)) {
    ++lineNo;
    const std::vector<std::string> tok = splitWhitespace(line);
    if (tok.empty()) continue;
    if (!header) {
      if (tok.size() < 3 || tok[0] != "QGC" || tok[1] != "WPL") throw Error("wpl: missing 'QGC WPL' header");
      header = true;
      continue;
    }
    if (tok.size() < 12) throw Error("wpl: expected 12 fields on line " + std::to_string(lineNo));
    double v[12];
    for (std::size_t i = 0; i < 12; ++i) v[i] = parseNumberToken(tok[i], lineNo);
    if (!haveHome) {
      m.homeLat = v[8];
      m.homeLon = v[9];
      m.homeAlt = v[10];
      haveHome = true;
      continue;
    }
    MissionItem it;
    it.frame = static_cast<int>(v[2]);
    it.command = static_cast<int>(v[3]);
    it.param1 = v[4];
    it.param2 = v[5];
    it.param3 = v[6];
    it.param4 = v[7];
    it.lat = v[8];
    it.lon = v[9];
    it.alt = v[10];
    it.autocontinue = v[11] != 0.0;
    m.items.push_back(it);
  }
  if (!header) throw Error("wpl: empty file");
  return m;
}

std::string toQgcPlan(const Mission& mission, const Terrain& terrain, const ZoneSet& zones) {
  using json::Value;
  Value root = Value::makeObject();
  root.set("fileType", Value::makeString("Plan"));
  root.set("groundStation", Value::makeString("QGroundControl"));
  root.set("version", num(1));

  Value fence = Value::makeObject();
  Value circles = Value::makeArray();
  for (const ZoneSet::Circle& c : zones.circles()) {
    const LatLon ll = terrain.toLatLon(c.center);
    Value circ = Value::makeObject();
    Value inner = Value::makeObject();
    Value centre = Value::makeArray();
    centre.push(num(ll.lat));
    centre.push(num(ll.lon));
    inner.set("center", centre);
    inner.set("radius", num(c.radius));
    circ.set("circle", inner);
    circ.set("inclusion", Value::makeBool(false));
    circ.set("version", num(1));
    circles.push(circ);
  }
  Value polygons = Value::makeArray();
  for (const ZoneSet::Polygon& poly : zones.polygons()) {
    Value pg = Value::makeObject();
    Value ring = Value::makeArray();
    for (const Vec2 p : poly.points) {
      const LatLon ll = terrain.toLatLon(p);
      Value pt = Value::makeArray();
      pt.push(num(ll.lat));
      pt.push(num(ll.lon));
      ring.push(pt);
    }
    pg.set("inclusion", Value::makeBool(false));
    pg.set("polygon", ring);
    pg.set("version", num(1));
    polygons.push(pg);
  }
  fence.set("circles", circles);
  fence.set("polygons", polygons);
  fence.set("version", num(2));
  root.set("geoFence", fence);

  Value mis = Value::makeObject();
  mis.set("cruiseSpeed", num(15));
  mis.set("firmwareType", num(3));
  mis.set("globalPlanAltitudeMode", num(1));
  mis.set("hoverSpeed", num(5));
  Value items = Value::makeArray();
  for (std::size_t i = 0; i < mission.items.size(); ++i) {
    const MissionItem& it = mission.items[i];
    Value item = Value::makeObject();
    item.set("AMSLAltAboveTerrain", Value::makeNull());
    item.set("Altitude", num(it.alt));
    item.set("AltitudeMode", num(1));
    item.set("autoContinue", Value::makeBool(it.autocontinue));
    item.set("command", num(it.command));
    item.set("doJumpId", num(static_cast<double>(i + 1)));
    item.set("frame", num(it.frame));
    Value params = Value::makeArray();
    params.push(num(it.param1));
    params.push(num(it.param2));
    params.push(num(it.param3));
    params.push(num(it.param4));
    params.push(num(it.lat));
    params.push(num(it.lon));
    params.push(num(it.alt));
    item.set("params", params);
    item.set("type", Value::makeString("SimpleItem"));
    items.push(item);
  }
  mis.set("items", items);
  Value home = Value::makeArray();
  home.push(num(mission.homeLat));
  home.push(num(mission.homeLon));
  home.push(num(mission.homeAlt));
  mis.set("plannedHomePosition", home);
  mis.set("vehicleType", num(2));
  mis.set("version", num(2));
  root.set("mission", mis);

  Value rally = Value::makeObject();
  rally.set("points", Value::makeArray());
  rally.set("version", num(2));
  root.set("rallyPoints", rally);
  return json::dump(root, 2) + "\n";
}

Mission parseQgcPlan(const std::string& text) {
  const json::Value root = json::parse(text);
  const json::Value* type = root.find("fileType");
  if (type == nullptr || !type->isString() || type->text != "Plan") throw Error("plan: fileType is not 'Plan'");
  const json::Value& mis = root.at("mission");
  Mission m;
  const json::Value& home = mis.at("plannedHomePosition");
  if (home.size() < 3) throw Error("plan: plannedHomePosition needs [lat, lon, alt]");
  m.homeLat = home.at(0).asNumber();
  m.homeLon = home.at(1).asNumber();
  m.homeAlt = home.at(2).asNumber();
  for (const json::Value& item : mis.at("items").items) {
    const json::Value* t = item.find("type");
    if (t == nullptr || !t->isString() || t->text != "SimpleItem") continue;
    const json::Value& params = item.at("params");
    if (params.size() < 7) throw Error("plan: SimpleItem needs seven params");
    MissionItem it;
    it.command = static_cast<int>(item.at("command").asNumber());
    it.frame = static_cast<int>(item.at("frame").asNumber());
    it.param1 = numberOrZero(params.at(0)).number;
    it.param2 = numberOrZero(params.at(1)).number;
    it.param3 = numberOrZero(params.at(2)).number;
    it.param4 = numberOrZero(params.at(3)).number;
    it.lat = params.at(4).asNumber();
    it.lon = params.at(5).asNumber();
    it.alt = params.at(6).asNumber();
    const json::Value* ac = item.find("autoContinue");
    it.autocontinue = ac == nullptr || !ac->isBool() || ac->boolean;
    m.items.push_back(it);
  }
  return m;
}

std::string toMavlinkItemsJson(const Mission& mission) {
  using json::Value;
  Value list = Value::makeArray();
  auto add = [&](std::size_t seq, int current, int frame, int command, double p1, double p2, double p3,
                 double p4, double lat, double lon, double alt, bool autocontinue) {
    Value o = Value::makeObject();
    o.set("seq", num(static_cast<double>(seq)));
    o.set("frame", num(frame));
    o.set("command", num(command));
    o.set("current", num(current));
    o.set("autocontinue", num(autocontinue ? 1 : 0));
    o.set("param1", num(p1));
    o.set("param2", num(p2));
    o.set("param3", num(p3));
    o.set("param4", num(p4));
    o.set("x", num(std::round(lat * 1e7)));
    o.set("y", num(std::round(lon * 1e7)));
    o.set("z", num(alt));
    list.push(o);
  };
  add(0, 1, kMavFrameGlobal, kMavCmdNavWaypoint, 0, 0, 0, 0, mission.homeLat, mission.homeLon, mission.homeAlt, true);
  for (std::size_t i = 0; i < mission.items.size(); ++i) {
    const MissionItem& it = mission.items[i];
    add(i + 1, 0, it.frame, it.command, it.param1, it.param2, it.param3, it.param4, it.lat, it.lon, it.alt,
        it.autocontinue);
  }
  return json::dump(list, 2) + "\n";
}

std::string toGeoJson(const Mission& mission, const Terrain& terrain, const ZoneSet& zones) {
  using json::Value;
  Value fc = Value::makeObject();
  fc.set("type", Value::makeString("FeatureCollection"));
  Value features = Value::makeArray();

  Value pathFeature = Value::makeObject();
  pathFeature.set("type", Value::makeString("Feature"));
  Value props = Value::makeObject();
  props.set("name", Value::makeString("flight path"));
  props.set("kind", Value::makeString("path"));
  pathFeature.set("properties", props);
  Value geom = Value::makeObject();
  geom.set("type", Value::makeString("LineString"));
  Value coords = Value::makeArray();
  for (const MissionItem& it : mission.items) coords.push(lonLatAlt(it.lat, it.lon, it.alt));
  geom.set("coordinates", coords);
  pathFeature.set("geometry", geom);
  features.push(pathFeature);

  const auto rings = zoneRings(terrain, zones);
  for (std::size_t i = 0; i < rings.size(); ++i) {
    Value f = Value::makeObject();
    f.set("type", Value::makeString("Feature"));
    Value p = Value::makeObject();
    p.set("name", Value::makeString("no-fly zone " + std::to_string(i + 1)));
    p.set("kind", Value::makeString("no-fly"));
    f.set("properties", p);
    Value g = Value::makeObject();
    g.set("type", Value::makeString("Polygon"));
    Value ring = Value::makeArray();
    for (const LatLon& ll : rings[i]) {
      Value c = Value::makeArray();
      c.push(num(ll.lon));
      c.push(num(ll.lat));
      ring.push(c);
    }
    Value first = Value::makeArray();
    first.push(num(rings[i].front().lon));
    first.push(num(rings[i].front().lat));
    ring.push(first);
    Value rs = Value::makeArray();
    rs.push(ring);
    g.set("coordinates", rs);
    f.set("geometry", g);
    features.push(f);
  }
  fc.set("features", features);
  return json::dump(fc, 2) + "\n";
}

std::string toKml(const Mission& mission, const Terrain& terrain, const ZoneSet& zones) {
  std::string out;
  out += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  out += "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n<name>UAV mission</name>\n";
  out += "<Placemark>\n<name>Flight path</name>\n<LineString>\n<altitudeMode>absolute</altitudeMode>\n<coordinates>\n";
  for (const MissionItem& it : mission.items) {
    out += fixed(it.lon, 8) + "," + fixed(it.lat, 8) + "," + fixed(it.alt, 2) + "\n";
  }
  out += "</coordinates>\n</LineString>\n</Placemark>\n";
  const auto rings = zoneRings(terrain, zones);
  for (std::size_t i = 0; i < rings.size(); ++i) {
    out += "<Placemark>\n<name>No-fly zone " + std::to_string(i + 1) +
           "</name>\n<Polygon>\n<outerBoundaryIs>\n<LinearRing>\n<coordinates>\n";
    for (const LatLon& ll : rings[i]) out += fixed(ll.lon, 8) + "," + fixed(ll.lat, 8) + ",0\n";
    out += fixed(rings[i].front().lon, 8) + "," + fixed(rings[i].front().lat, 8) + ",0\n";
    out += "</coordinates>\n</LinearRing>\n</outerBoundaryIs>\n</Polygon>\n</Placemark>\n";
  }
  out += "</Document>\n</kml>\n";
  return out;
}

}  // namespace uav
