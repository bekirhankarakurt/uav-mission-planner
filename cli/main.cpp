#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "uav/benchmark.hpp"
#include "uav/common.hpp"
#include "uav/export.hpp"
#include "uav/mission.hpp"
#include "uav/render.hpp"
#include "uav/terrain.hpp"
#include "uav/zones.hpp"

namespace {

const char* kUsage =
    "uav-plan " UAV_VERSION " - terrain-aware UAV mission planner\n"
    "\n"
    "Usage:\n"
    "  uav-plan plan [options]        plan a mission and export it\n"
    "  uav-plan benchmark [options]   compare A*, Theta* and RRT* on seeded synthetic maps\n"
    "  uav-plan gen [options]         write a synthetic terrain as an SRTM .hgt tile\n"
    "  uav-plan info --hgt FILE       print tile statistics\n"
    "  uav-plan --help | --version\n"
    "\n"
    "Terrain (plan, gen):\n"
    "  --hgt FILE              SRTM .hgt tile (name like N47E008.hgt)\n"
    "  --crop X0,Y0,W,H        crop window in cells        --downsample N   keep every Nth sample\n"
    "  --synthetic EXP         synthetic map of 2^EXP+1 cells per side (default 7)\n"
    "  --seed N                synthetic seed (default 1)  --cell M   cell size in metres (30)\n"
    "  --max-height M          synthetic peak height (1500) --roughness R  (0.55)\n"
    "\n"
    "Plan:\n"
    "  --start LAT,LON | --start-xy X,Y     --goal LAT,LON | --goal-xy X,Y   (default: map corners)\n"
    "  --zones FILE            GeoJSON no-fly zones (repeatable)\n"
    "  --circle LAT,LON,R      circular no-fly zone, radius in metres (repeatable)\n"
    "  --circle-xy X,Y,R       same, in map metres (repeatable)\n"
    "  --planner NAME          astar | thetastar (default) | rrtstar\n"
    "  --agl M                 minimum height above ground (50)\n"
    "  --margin M              stand-off distance around zones (0)\n"
    "  --max-climb G --max-descent G   gradient limits, rise/run (0.35)\n"
    "  --climb-weight W --terrain-weight W   cost model weights (1.0, 0.25)\n"
    "  --max-alt M             MSL ceiling\n"
    "  --rrt-iterations N --rrt-step M --rrt-seed N\n"
    "  --smooth                shortcut the path with line-of-sight checks\n"
    "  --ascii [--ascii-width N]   print an ASCII map\n"
    "  --out-wpl F --out-plan F --out-geojson F --out-kml F --out-items F\n"
    "\n"
    "Benchmark: --maps N (10) --size EXP (7) --seed N (1) --rrt-iterations N (3000) --csv\n";

class Args {
 public:
  Args(int argc, char** argv, int first) {
    static const std::set<std::string> kFlags = {"smooth", "ascii", "csv", "help"};
    for (int i = first; i < argc; ++i) {
      const std::string tok = argv[i];
      if (tok.rfind("--", 0) != 0) throw uav::Error("unexpected argument '" + tok + "'");
      const std::string name = tok.substr(2);
      if (kFlags.count(name) == 0 && i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
        opts_[name].push_back(argv[++i]);
      } else {
        opts_[name].push_back("true");
      }
    }
  }

  bool has(const std::string& name) const { return opts_.count(name) != 0; }
  std::string get(const std::string& name, const std::string& def) const {
    const auto it = opts_.find(name);
    return it == opts_.end() ? def : it->second.back();
  }
  std::vector<std::string> all(const std::string& name) const {
    const auto it = opts_.find(name);
    return it == opts_.end() ? std::vector<std::string>() : it->second;
  }
  double number(const std::string& name, double def) const {
    return has(name) ? toNumber(get(name, ""), name) : def;
  }
  long integer(const std::string& name, long def) const {
    return has(name) ? static_cast<long>(toNumber(get(name, ""), name)) : def;
  }

  static double toNumber(const std::string& s, const std::string& what) {
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0') throw uav::Error("invalid number '" + s + "' for --" + what);
    return v;
  }

 private:
  std::map<std::string, std::vector<std::string>> opts_;
};

std::vector<double> parseList(const std::string& s, std::size_t expected, const std::string& what) {
  std::vector<double> out;
  std::stringstream ss(s);
  std::string part;
  while (std::getline(ss, part, ',')) out.push_back(Args::toNumber(part, what));
  if (out.size() != expected) {
    throw uav::Error("--" + what + " expects " + std::to_string(expected) + " comma-separated numbers");
  }
  return out;
}

std::string readFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw uav::Error("cannot open '" + path + "'");
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void writeFile(const std::string& path, const std::string& data) {
  std::ofstream out(path, std::ios::binary);
  if (!out) throw uav::Error("cannot write '" + path + "'");
  out.write(data.data(), static_cast<std::streamsize>(data.size()));
}

uav::Terrain loadTerrain(const Args& args) {
  if (args.has("hgt")) {
    uav::Terrain t = uav::Terrain::loadHgt(args.get("hgt", ""));
    if (args.has("crop")) {
      const auto c = parseList(args.get("crop", ""), 4, "crop");
      t = t.crop(static_cast<std::size_t>(c[0]), static_cast<std::size_t>(c[1]), static_cast<std::size_t>(c[2]),
                 static_cast<std::size_t>(c[3]));
    }
    if (args.has("downsample")) t = t.downsample(static_cast<std::size_t>(args.integer("downsample", 1)));
    return t;
  }
  uav::SyntheticParams sp;
  const std::string expStr = args.get("synthetic", "7");
  sp.exponent = expStr == "true" ? 7 : static_cast<int>(Args::toNumber(expStr, "synthetic"));
  sp.seed = static_cast<std::uint64_t>(args.integer("seed", 1));
  sp.cell_m = args.number("cell", sp.cell_m);
  sp.max_height_m = args.number("max-height", sp.max_height_m);
  sp.roughness = args.number("roughness", sp.roughness);
  return uav::Terrain::synthetic(sp);
}

int cmdInfo(const Args& args) {
  if (!args.has("hgt")) throw uav::Error("info needs --hgt FILE");
  const uav::Terrain t = uav::Terrain::loadHgt(args.get("hgt", ""));
  std::cout << "samples: " << t.cols() << " x " << t.rows() << "\n"
            << "origin: " << std::fixed << std::setprecision(4) << t.origin().lat << ", " << t.origin().lon << "\n"
            << std::setprecision(2) << "cell size: " << t.cellX() << " x " << t.cellY() << " m\n"
            << std::setprecision(1) << "elevation: " << t.minElevation() << " .. " << t.maxElevation() << " m\n"
            << "voids filled: " << t.voidsFilled() << "\n";
  return 0;
}

int cmdGen(const Args& args) {
  if (!args.has("out")) throw uav::Error("gen needs --out FILE.hgt");
  const uav::Terrain t = loadTerrain(args);
  const std::vector<unsigned char> bytes = t.toHgt();
  writeFile(args.get("out", ""), std::string(bytes.begin(), bytes.end()));
  std::cout << "wrote " << args.get("out", "") << " (" << t.cols() << "x" << t.rows() << " samples)\n";
  return 0;
}

int cmdBenchmark(const Args& args) {
  uav::BenchmarkConfig cfg;
  cfg.maps = static_cast<int>(args.integer("maps", 10));
  cfg.exponent = static_cast<int>(args.integer("size", 7));
  cfg.seed = static_cast<std::uint64_t>(args.integer("seed", 1));
  cfg.rrt_iterations = static_cast<std::size_t>(args.integer("rrt-iterations", 3000));
  cfg.planner.agl_m = args.number("agl", cfg.planner.agl_m);
  const auto rows = uav::runBenchmark(cfg);
  if (args.has("csv")) {
    std::cout << uav::benchmarkCsv(rows);
  } else {
    std::cout << "Benchmark: " << cfg.maps << " maps, " << ((1 << cfg.exponent) + 1) << "x" << ((1 << cfg.exponent) + 1)
              << " cells, seed " << cfg.seed << ", RRT* " << cfg.rrt_iterations << " iterations\n\n"
              << uav::benchmarkMarkdown(rows);
  }
  return 0;
}

int cmdPlan(const Args& args) {
  const uav::Terrain terrain = loadTerrain(args);

  std::vector<uav::GeoZone> geo;
  for (const std::string& file : args.all("zones")) {
    const auto z = uav::parseGeoJsonZones(readFile(file));
    geo.insert(geo.end(), z.begin(), z.end());
  }
  for (const std::string& c : args.all("circle")) {
    const auto v = parseList(c, 3, "circle");
    uav::GeoZone z;
    z.kind = uav::GeoZone::Kind::Circle;
    z.center = uav::LatLon(v[0], v[1]);
    z.radiusM = v[2];
    geo.push_back(z);
  }
  uav::ZoneSet zones = uav::ZoneSet::fromGeo(geo, terrain);
  for (const std::string& c : args.all("circle-xy")) {
    const auto v = parseList(c, 3, "circle-xy");
    zones.addCircle(uav::Vec2(v[0], v[1]), v[2]);
  }

  auto point = [&](const std::string& geoOpt, const std::string& xyOpt, double fraction) {
    if (args.has(geoOpt)) {
      const auto v = parseList(args.get(geoOpt, ""), 2, geoOpt);
      return terrain.toXY(uav::LatLon(v[0], v[1]));
    }
    if (args.has(xyOpt)) {
      const auto v = parseList(args.get(xyOpt, ""), 2, xyOpt);
      return uav::Vec2(v[0], v[1]);
    }
    return uav::Vec2(fraction * terrain.widthM(), fraction * terrain.heightM());
  };
  const uav::Vec2 start = point("start", "start-xy", 0.08);
  const uav::Vec2 goal = point("goal", "goal-xy", 0.92);

  uav::PlannerConfig pc;
  pc.zone_margin_m = args.number("margin", pc.zone_margin_m);
  pc.climb_weight = args.number("climb-weight", pc.climb_weight);
  pc.terrain_weight = args.number("terrain-weight", pc.terrain_weight);
  uav::ProfileConfig prof;
  prof.agl_m = args.number("agl", prof.agl_m);
  prof.max_climb_gradient = args.number("max-climb", prof.max_climb_gradient);
  prof.max_descent_gradient = args.number("max-descent", prof.max_descent_gradient);
  prof.max_altitude_m = args.number("max-alt", prof.max_altitude_m);
  uav::RrtParams rrt;
  rrt.max_iterations = static_cast<std::size_t>(args.integer("rrt-iterations", static_cast<long>(rrt.max_iterations)));
  rrt.step_m = args.number("rrt-step", rrt.step_m);
  rrt.seed = static_cast<std::uint64_t>(args.integer("rrt-seed", args.integer("seed", 1)));

  const uav::PlannerKind kind = uav::parsePlannerKind(args.get("planner", "thetastar"));
  const uav::MissionPlan mp = uav::planMission(terrain, zones, start, goal, kind, pc, prof, rrt, args.has("smooth"));

  std::cout << std::fixed;
  std::cout << "terrain:  " << terrain.cols() << "x" << terrain.rows() << " cells, " << std::setprecision(1)
            << terrain.cellX() << " m spacing, elevation " << terrain.minElevation() << ".."
            << terrain.maxElevation() << " m\n";
  std::cout << "planner:  " << mp.result.planner << (args.has("smooth") ? " + smoothing" : "") << "\n";
  if (!mp.result.found) {
    std::cout << "status:   no path (" << mp.result.message << ")\n";
    return 1;
  }
  std::cout << "status:   path found\n"
            << "path:     " << mp.path.size() << " vertices, " << mp.result.length_m << " m horizontal (straight line "
            << uav::distance(start, goal) << " m)\n"
            << "cost:     " << mp.result.cost << "\n"
            << "runtime:  " << std::setprecision(2) << mp.result.runtime_ms << " ms, " << mp.result.nodes << " nodes\n"
            << std::setprecision(2) << "altitude: min AGL " << mp.profile.min_agl << " m, max climb gradient "
            << mp.profile.max_climb << ", max descent gradient " << mp.profile.max_descent << ", "
            << mp.mission.items.size() << " waypoints\n";
  if (!mp.profile.within_ceiling) std::cout << "warning:  profile exceeds the altitude ceiling\n";

  if (args.has("out-wpl")) writeFile(args.get("out-wpl", ""), uav::toWpl(mp.mission));
  if (args.has("out-plan")) writeFile(args.get("out-plan", ""), uav::toQgcPlan(mp.mission, terrain, zones));
  if (args.has("out-geojson")) writeFile(args.get("out-geojson", ""), uav::toGeoJson(mp.mission, terrain, zones));
  if (args.has("out-kml")) writeFile(args.get("out-kml", ""), uav::toKml(mp.mission, terrain, zones));
  if (args.has("out-items")) writeFile(args.get("out-items", ""), uav::toMavlinkItemsJson(mp.mission));

  if (args.has("ascii")) {
    std::cout << "\n"
              << uav::renderAscii(terrain, zones, mp.path, start, goal, static_cast<int>(args.integer("ascii-width", 72)));
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 2) {
      std::cout << kUsage;
      return 2;
    }
    const std::string cmd = argv[1];
    if (cmd == "--help" || cmd == "-h" || cmd == "help") {
      std::cout << kUsage;
      return 0;
    }
    if (cmd == "--version") {
      std::cout << "uav-plan " << uav::version() << "\n";
      return 0;
    }
    const Args args(argc, argv, 2);
    if (args.has("help")) {
      std::cout << kUsage;
      return 0;
    }
    if (cmd == "plan") return cmdPlan(args);
    if (cmd == "benchmark") return cmdBenchmark(args);
    if (cmd == "gen") return cmdGen(args);
    if (cmd == "info") return cmdInfo(args);
    std::cerr << "unknown command '" << cmd << "'\n\n" << kUsage;
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 2;
  }
}
