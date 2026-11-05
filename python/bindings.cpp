#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "uav/benchmark.hpp"
#include "uav/common.hpp"
#include "uav/export.hpp"
#include "uav/geo.hpp"
#include "uav/mission.hpp"
#include "uav/planner.hpp"
#include "uav/profile.hpp"
#include "uav/render.hpp"
#include "uav/terrain.hpp"
#include "uav/zones.hpp"

namespace py = pybind11;
using namespace uav;

namespace {

using XY = std::pair<double, double>;

Vec2 toVec(const XY& p) { return Vec2(p.first, p.second); }

std::vector<XY> toPairs(const std::vector<Vec2>& v) {
  std::vector<XY> out;
  out.reserve(v.size());
  for (const Vec2 p : v) out.emplace_back(p.x, p.y);
  return out;
}

/// Reads keyword arguments into struct fields and rejects unknown names.
class KwReader {
 public:
  explicit KwReader(const py::kwargs& kw) : kw_(kw) {}
  template <class T>
  void get(const char* key, T& field) {
    if (kw_.contains(key)) {
      field = kw_[key].template cast<T>();
      ++used_;
    }
  }
  void finish() const {
    if (used_ != py::len(kw_)) throw py::type_error("unexpected keyword argument");
  }

 private:
  const py::kwargs& kw_;
  std::size_t used_ = 0;
};

}  // namespace

PYBIND11_MODULE(_core, m) {
  m.doc() = "Terrain-aware UAV mission planning core (C++17).";
  m.attr("__version__") = uav::version();

  py::register_exception<uav::Error>(m, "PlannerError", PyExc_RuntimeError);

  // ---- geodesy -------------------------------------------------------------------------------
  m.def("haversine", [](double lat1, double lon1, double lat2, double lon2) {
    return haversine(LatLon(lat1, lon1), LatLon(lat2, lon2));
  }, py::arg("lat1"), py::arg("lon1"), py::arg("lat2"), py::arg("lon2"),
        "Great-circle distance in metres.");
  m.def("bearing", [](double lat1, double lon1, double lat2, double lon2) {
    return initialBearingDeg(LatLon(lat1, lon1), LatLon(lat2, lon2));
  }, py::arg("lat1"), py::arg("lon1"), py::arg("lat2"), py::arg("lon2"),
        "Initial bearing in degrees [0, 360).");
  m.def("destination", [](double lat, double lon, double bearing_deg, double distance_m) {
    const LatLon d = destination(LatLon(lat, lon), bearing_deg, distance_m);
    return XY(d.lat, d.lon);
  }, py::arg("lat"), py::arg("lon"), py::arg("bearing_deg"), py::arg("distance_m"),
        "Point reached after travelling distance_m along bearing_deg; returns (lat, lon).");
  m.def("to_enu", [](double origin_lat, double origin_lon, double lat, double lon, double alt, double origin_alt) {
    const Enu e = LocalFrame(LatLon(origin_lat, origin_lon), origin_alt).toEnu(LatLon(lat, lon), alt);
    return std::make_tuple(e.e, e.n, e.u);
  }, py::arg("origin_lat"), py::arg("origin_lon"), py::arg("lat"), py::arg("lon"), py::arg("alt") = 0.0,
        py::arg("origin_alt") = 0.0, "Local East-North-Up coordinates of a geodetic point.");
  m.def("from_enu", [](double origin_lat, double origin_lon, double e, double n, double u, double origin_alt) {
    double alt = 0.0;
    Enu enu;
    enu.e = e;
    enu.n = n;
    enu.u = u;
    const LatLon p = LocalFrame(LatLon(origin_lat, origin_lon), origin_alt).toLatLon(enu, &alt);
    return std::make_tuple(p.lat, p.lon, alt);
  }, py::arg("origin_lat"), py::arg("origin_lon"), py::arg("e"), py::arg("n"), py::arg("u") = 0.0,
        py::arg("origin_alt") = 0.0, "Geodetic (lat, lon, alt) of a local ENU point.");

  // ---- terrain -------------------------------------------------------------------------------
  py::class_<Terrain>(m, "Terrain", "Regular elevation grid; row 0 is the southern edge.")
      .def_static("synthetic", [](int exponent, std::uint64_t seed, double roughness, double max_height_m,
                                   double cell_m, double origin_lat, double origin_lon) {
        SyntheticParams sp;
        sp.exponent = exponent;
        sp.seed = seed;
        sp.roughness = roughness;
        sp.max_height_m = max_height_m;
        sp.cell_m = cell_m;
        sp.origin = LatLon(origin_lat, origin_lon);
        return Terrain::synthetic(sp);
      }, py::arg("exponent") = 7, py::arg("seed") = 1, py::arg("roughness") = 0.55,
         py::arg("max_height_m") = 1500.0, py::arg("cell_m") = 30.0, py::arg("origin_lat") = 47.0,
         py::arg("origin_lon") = 8.0, "Seeded diamond-square terrain with 2**exponent + 1 samples per side.")
      .def_static("from_hgt", [](const std::string& path) { return Terrain::loadHgt(path); }, py::arg("path"),
                  "Load an SRTM .hgt tile; the file name (N47E008.hgt) gives the tile origin.")
      .def_static("from_hgt_bytes", [](const py::bytes& data, double lat, double lon) {
        const std::string s = data;
        return Terrain::fromHgt(std::vector<unsigned char>(s.begin(), s.end()), LatLon(lat, lon));
      }, py::arg("data"), py::arg("south_west_lat"), py::arg("south_west_lon"))
      .def_static("from_rows", [](const std::vector<std::vector<float>>& rows, double cell_m, double origin_lat,
                                  double origin_lon) {
        if (rows.size() < 2 || rows[0].size() < 2) throw uav::Error("from_rows needs at least a 2x2 grid");
        std::vector<float> flat;
        for (const auto& row : rows) {
          if (row.size() != rows[0].size()) throw uav::Error("from_rows: ragged rows");
          flat.insert(flat.end(), row.begin(), row.end());
        }
        return Terrain(rows[0].size(), rows.size(), cell_m, cell_m, LatLon(origin_lat, origin_lon), origin_lat,
                       std::move(flat));
      }, py::arg("rows"), py::arg("cell_m") = 30.0, py::arg("origin_lat") = 47.0, py::arg("origin_lon") = 8.0,
         "Build a terrain from nested lists (row 0 = south).")
      .def_property_readonly("cols", &Terrain::cols)
      .def_property_readonly("rows", &Terrain::rows)
      .def_property_readonly("cell_x", &Terrain::cellX)
      .def_property_readonly("cell_y", &Terrain::cellY)
      .def_property_readonly("width_m", &Terrain::widthM)
      .def_property_readonly("height_m", &Terrain::heightM)
      .def_property_readonly("min_elevation", &Terrain::minElevation)
      .def_property_readonly("max_elevation", &Terrain::maxElevation)
      .def_property_readonly("voids_filled", &Terrain::voidsFilled)
      .def_property_readonly("origin", [](const Terrain& t) { return XY(t.origin().lat, t.origin().lon); })
      .def("elevation", [](const Terrain& t, double x, double y) { return t.elevation(Vec2(x, y)); },
           py::arg("x"), py::arg("y"))
      .def("contains", [](const Terrain& t, double x, double y) { return t.contains(Vec2(x, y)); })
      .def("to_xy", [](const Terrain& t, double lat, double lon) {
        const Vec2 p = t.toXY(LatLon(lat, lon));
        return XY(p.x, p.y);
      }, py::arg("lat"), py::arg("lon"))
      .def("to_latlon", [](const Terrain& t, double x, double y) {
        const LatLon p = t.toLatLon(Vec2(x, y));
        return XY(p.lat, p.lon);
      }, py::arg("x"), py::arg("y"))
      .def("heights", [](const Terrain& t) {
        std::vector<std::vector<float>> rows(t.rows(), std::vector<float>(t.cols()));
        for (std::size_t r = 0; r < t.rows(); ++r) {
          for (std::size_t c = 0; c < t.cols(); ++c) rows[r][c] = t.at(c, r);
        }
        return rows;
      }, "Elevations as a list of rows (row 0 = south).")
      .def("crop", &Terrain::crop, py::arg("col0"), py::arg("row0"), py::arg("width"), py::arg("height"))
      .def("downsample", &Terrain::downsample, py::arg("factor"))
      .def("to_hgt", [](const Terrain& t) {
        const std::vector<unsigned char> b = t.toHgt();
        return py::bytes(reinterpret_cast<const char*>(b.data()), b.size());
      });

  // ---- no-fly zones --------------------------------------------------------------------------
  py::class_<ZoneSet>(m, "Zones", "No-fly zones in the planar frame of a terrain.")
      .def(py::init<>())
      .def_static("from_geojson", [](const std::string& text, const Terrain& t) {
        return ZoneSet::fromGeo(parseGeoJsonZones(text), t);
      }, py::arg("text"), py::arg("terrain"), "Parse Polygon / MultiPolygon / Point(radius_m) GeoJSON.")
      .def("add_circle_xy", [](ZoneSet& z, double x, double y, double r) { z.addCircle(Vec2(x, y), r); },
           py::arg("x"), py::arg("y"), py::arg("radius_m"))
      .def("add_circle", [](ZoneSet& z, const Terrain& t, double lat, double lon, double r) {
        z.addCircle(t.toXY(LatLon(lat, lon)), r);
      }, py::arg("terrain"), py::arg("lat"), py::arg("lon"), py::arg("radius_m"))
      .def("add_polygon_xy", [](ZoneSet& z, const std::vector<XY>& pts) {
        std::vector<Vec2> v;
        for (const XY& p : pts) v.push_back(toVec(p));
        z.addPolygon(std::move(v));
      }, py::arg("points"))
      .def("add_polygon", [](ZoneSet& z, const Terrain& t, const std::vector<XY>& latlon) {
        std::vector<Vec2> v;
        for (const XY& p : latlon) v.push_back(t.toXY(LatLon(p.first, p.second)));
        z.addPolygon(std::move(v));
      }, py::arg("terrain"), py::arg("points"), "Polygon from (lat, lon) vertices.")
      .def("blocked", [](const ZoneSet& z, double x, double y, double margin) { return z.blocked(Vec2(x, y), margin); },
           py::arg("x"), py::arg("y"), py::arg("margin") = 0.0)
      .def("segment_blocked", [](const ZoneSet& z, double x0, double y0, double x1, double y1, double margin) {
        return z.segmentBlocked(Vec2(x0, y0), Vec2(x1, y1), margin);
      }, py::arg("x0"), py::arg("y0"), py::arg("x1"), py::arg("y1"), py::arg("margin") = 0.0)
      .def_property_readonly("circles", [](const ZoneSet& z) {
        std::vector<std::tuple<double, double, double>> out;
        for (const auto& c : z.circles()) out.emplace_back(c.center.x, c.center.y, c.radius);
        return out;
      })
      .def_property_readonly("polygons", [](const ZoneSet& z) {
        std::vector<std::vector<XY>> out;
        for (const auto& p : z.polygons()) out.push_back(toPairs(p.points));
        return out;
      })
      .def("__len__", [](const ZoneSet& z) { return z.circles().size() + z.polygons().size(); });

  // ---- configuration -------------------------------------------------------------------------
  py::class_<PlannerConfig>(m, "PlannerConfig")
      .def(py::init([](const py::kwargs& kw) {
        PlannerConfig c;
        KwReader r(kw);
        r.get("agl_m", c.agl_m);
        r.get("zone_margin_m", c.zone_margin_m);
        r.get("climb_weight", c.climb_weight);
        r.get("terrain_weight", c.terrain_weight);
        r.get("max_altitude_m", c.max_altitude_m);
        r.finish();
        return c;
      }))
      .def_readwrite("agl_m", &PlannerConfig::agl_m)
      .def_readwrite("zone_margin_m", &PlannerConfig::zone_margin_m)
      .def_readwrite("climb_weight", &PlannerConfig::climb_weight)
      .def_readwrite("terrain_weight", &PlannerConfig::terrain_weight)
      .def_readwrite("max_altitude_m", &PlannerConfig::max_altitude_m);

  py::class_<ProfileConfig>(m, "ProfileConfig")
      .def(py::init([](const py::kwargs& kw) {
        ProfileConfig c;
        KwReader r(kw);
        r.get("agl_m", c.agl_m);
        r.get("max_climb_gradient", c.max_climb_gradient);
        r.get("max_descent_gradient", c.max_descent_gradient);
        r.get("sample_m", c.sample_m);
        r.get("max_altitude_m", c.max_altitude_m);
        r.finish();
        return c;
      }))
      .def_readwrite("agl_m", &ProfileConfig::agl_m)
      .def_readwrite("max_climb_gradient", &ProfileConfig::max_climb_gradient)
      .def_readwrite("max_descent_gradient", &ProfileConfig::max_descent_gradient)
      .def_readwrite("sample_m", &ProfileConfig::sample_m)
      .def_readwrite("max_altitude_m", &ProfileConfig::max_altitude_m);

  py::class_<RrtParams>(m, "RrtParams")
      .def(py::init([](const py::kwargs& kw) {
        RrtParams c;
        KwReader r(kw);
        r.get("max_iterations", c.max_iterations);
        r.get("step_m", c.step_m);
        r.get("goal_bias", c.goal_bias);
        r.get("seed", c.seed);
        r.finish();
        return c;
      }))
      .def_readwrite("max_iterations", &RrtParams::max_iterations)
      .def_readwrite("step_m", &RrtParams::step_m)
      .def_readwrite("goal_bias", &RrtParams::goal_bias)
      .def_readwrite("seed", &RrtParams::seed);

  // ---- planning ------------------------------------------------------------------------------
  py::class_<PlanResult>(m, "PlanResult")
      .def_readonly("found", &PlanResult::found)
      .def_readonly("planner", &PlanResult::planner)
      .def_readonly("message", &PlanResult::message)
      .def_readonly("length_m", &PlanResult::length_m)
      .def_readonly("cost", &PlanResult::cost)
      .def_readonly("runtime_ms", &PlanResult::runtime_ms)
      .def_readonly("nodes", &PlanResult::nodes)
      .def_property_readonly("path", [](const PlanResult& r) { return toPairs(r.path); });

  m.def("plan_path", [](const Terrain& t, const ZoneSet& z, const XY& start, const XY& goal,
                        const std::string& planner, const PlannerConfig& cfg, const RrtParams& rrt) {
    const PlanningContext ctx(t, z, cfg);
    return plan(parsePlannerKind(planner), ctx, toVec(start), toVec(goal), rrt);
  }, py::arg("terrain"), py::arg("zones"), py::arg("start"), py::arg("goal"), py::arg("planner") = "thetastar",
        py::arg("config") = PlannerConfig(), py::arg("rrt") = RrtParams(),
        "Plan a 2D path (no altitude profile) between two (x, y) points in metres.");

  py::class_<MissionItem>(m, "MissionItem")
      .def(py::init<>())
      .def_readwrite("command", &MissionItem::command)
      .def_readwrite("frame", &MissionItem::frame)
      .def_readwrite("param1", &MissionItem::param1)
      .def_readwrite("param2", &MissionItem::param2)
      .def_readwrite("param3", &MissionItem::param3)
      .def_readwrite("param4", &MissionItem::param4)
      .def_readwrite("lat", &MissionItem::lat)
      .def_readwrite("lon", &MissionItem::lon)
      .def_readwrite("alt", &MissionItem::alt)
      .def_readwrite("autocontinue", &MissionItem::autocontinue);

  py::class_<Mission>(m, "Mission")
      .def(py::init<>())
      .def_readwrite("home_lat", &Mission::homeLat)
      .def_readwrite("home_lon", &Mission::homeLon)
      .def_readwrite("home_alt", &Mission::homeAlt)
      .def_readwrite("items", &Mission::items);

  py::class_<MissionPlan>(m, "MissionPlan")
      .def_readonly("result", &MissionPlan::result)
      .def_readonly("mission", &MissionPlan::mission)
      .def_property_readonly("found", [](const MissionPlan& p) { return p.result.found; })
      .def_property_readonly("path", [](const MissionPlan& p) { return toPairs(p.path); })
      .def_property_readonly("profile", [](const MissionPlan& p) {
        std::vector<std::tuple<double, double, double, double, double>> out;
        for (const ProfilePoint& pp : p.profile.points) out.emplace_back(pp.p.x, pp.p.y, pp.s, pp.ground, pp.alt);
        return out;
      }, "Full profile as (x, y, distance, ground, altitude) tuples.")
      .def_property_readonly("min_agl", [](const MissionPlan& p) { return p.profile.min_agl; })
      .def_property_readonly("max_climb", [](const MissionPlan& p) { return p.profile.max_climb; })
      .def_property_readonly("max_descent", [](const MissionPlan& p) { return p.profile.max_descent; })
      .def_property_readonly("within_ceiling", [](const MissionPlan& p) { return p.profile.within_ceiling; });

  m.def("plan_mission", [](const Terrain& t, const ZoneSet& z, const XY& start, const XY& goal,
                           const std::string& planner, const PlannerConfig& pc, const ProfileConfig& prof,
                           const RrtParams& rrt, bool smooth) {
    return planMission(t, z, toVec(start), toVec(goal), parsePlannerKind(planner), pc, prof, rrt, smooth);
  }, py::arg("terrain"), py::arg("zones"), py::arg("start"), py::arg("goal"), py::arg("planner") = "thetastar",
        py::arg("planner_config") = PlannerConfig(), py::arg("profile_config") = ProfileConfig(),
        py::arg("rrt") = RrtParams(), py::arg("smooth") = true,
        "Plan, smooth, add the terrain-following altitude profile and build the mission.");

  // ---- export / import -----------------------------------------------------------------------
  m.def("to_wpl", &toWpl, py::arg("mission"));
  m.def("parse_wpl", &parseWpl, py::arg("text"));
  m.def("to_qgc_plan", &toQgcPlan, py::arg("mission"), py::arg("terrain"), py::arg("zones"));
  m.def("parse_qgc_plan", &parseQgcPlan, py::arg("text"));
  m.def("to_mavlink_items_json", &toMavlinkItemsJson, py::arg("mission"));
  m.def("to_geojson", &toGeoJson, py::arg("mission"), py::arg("terrain"), py::arg("zones"));
  m.def("to_kml", &toKml, py::arg("mission"), py::arg("terrain"), py::arg("zones"));

  m.def("render_ascii", [](const Terrain& t, const ZoneSet& z, const std::vector<XY>& path, const XY& start,
                           const XY& goal, int width) {
    std::vector<Vec2> p;
    for (const XY& q : path) p.push_back(toVec(q));
    return renderAscii(t, z, p, toVec(start), toVec(goal), width);
  }, py::arg("terrain"), py::arg("zones"), py::arg("path"), py::arg("start"), py::arg("goal"),
        py::arg("width") = 72);

  // ---- benchmark -----------------------------------------------------------------------------
  py::class_<BenchmarkRow>(m, "BenchmarkRow")
      .def_readonly("planner", &BenchmarkRow::planner)
      .def_readonly("maps", &BenchmarkRow::maps)
      .def_readonly("solved", &BenchmarkRow::solved)
      .def_readonly("mean_length_m", &BenchmarkRow::mean_length_m)
      .def_readonly("mean_stretch", &BenchmarkRow::mean_stretch)
      .def_readonly("mean_cost", &BenchmarkRow::mean_cost)
      .def_readonly("mean_runtime_ms", &BenchmarkRow::mean_runtime_ms)
      .def_readonly("mean_nodes", &BenchmarkRow::mean_nodes);

  m.def("run_benchmark", [](int maps, int exponent, std::uint64_t seed, std::size_t rrt_iterations,
                            const PlannerConfig& pc) {
    BenchmarkConfig cfg;
    cfg.maps = maps;
    cfg.exponent = exponent;
    cfg.seed = seed;
    cfg.rrt_iterations = rrt_iterations;
    cfg.planner = pc;
    return runBenchmark(cfg);
  }, py::arg("maps") = 10, py::arg("exponent") = 7, py::arg("seed") = 1, py::arg("rrt_iterations") = 3000,
        py::arg("planner_config") = PlannerConfig(), "Compare A*, Theta* and RRT* on seeded synthetic maps.");
  m.def("benchmark_markdown", &benchmarkMarkdown, py::arg("rows"));
  m.def("benchmark_csv", &benchmarkCsv, py::arg("rows"));
}
