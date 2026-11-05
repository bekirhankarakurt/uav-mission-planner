# uav-mission-planner

Plan terrain-following UAV missions around no-fly zones with A*, Theta* or RRT* and export them straight to QGroundControl or any MAVLink ground station.

[![CI](https://github.com/bekirhankarakurt/uav-mission-planner/actions/workflows/ci.yml/badge.svg)](https://github.com/bekirhankarakurt/uav-mission-planner/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/bekirhankarakurt/uav-mission-planner)](https://github.com/bekirhankarakurt/uav-mission-planner/releases)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)
![Python](https://img.shields.io/badge/python-3.9%2B-3776AB.svg)

## What and why

- **Plans in 2.5D.** Real SRTM elevation (`.hgt`, 1 and 3 arc-second) or a seeded fractal terrain generator for reproducible tests.
- **Three planners behind one interface.** Grid A* (8-connected), Theta* (any-angle) and RRT* (continuous, with rewiring), all using a cost model that charges for climbing and for flying over high ground.
- **Never flies where it should not.** Circular and polygon no-fly zones (GeoJSON), with an optional stand-off margin.
- **Terrain-following altitude profile.** Minimum height above ground (AGL) plus climb and descent gradient limits, so a ridge ahead starts a climb early enough.
- **Ready for the aircraft.** Exports QGroundControl WPL 110, QGroundControl `.plan` (with geofence), a MAVLink-style mission item list, GeoJSON and KML.
- **Small and dependency-free.** C++17 core with no third-party code; the Python package is a thin pybind11 wrapper with an optional matplotlib plot.
- **Tested.** C++ unit tests under GCC, Clang and ASan/UBSan, plus a pytest suite, on every push.

> This tool produces flight plans, it is not certified flight software. Always review a plan in your ground station and check it against airspace rules and current terrain data before flying.

## Example output

`uav-plan plan --synthetic 7 --seed 4 --planner thetastar --smooth --circle-xy 1500,1700,350 --circle-xy 2400,1000,300 --ascii`
on a seeded 129 x 129 map (30 m cells). Terrain is shaded by elevation (` .:-=+*#%@`), `X` is a no-fly zone, `o` the path, `S` the start and `G` the goal. North is up.

```text
terrain:  129x129 cells, 30.0 m spacing, elevation 0.0..1500.0 m
planner:  thetastar + smoothing
status:   path found
path:     30 vertices, 4687.2 m horizontal (straight line 4561.7 m)
cost:     5971.8
runtime:  11.32 ms, 7511 nodes
altitude: min AGL 50.00 m, max climb gradient 0.35, max descent gradient 0.35, 63 waypoints

++++******#******++********+********+++=======-----::::::::..:::
+*+******#******************+*******+++====---------::::::::.::-
**********#*#*#****************+++++=======----------::::oG:::::
*#*********#############******+++++=========----------:ooo::::::
#####******############******++++=======-=-----------ooo::::::::
#####*******##**######*******++++=========---------ooo----::::--
%###*###**********###*****+++++++==========-------oo------------
%%########*************+++++++++++============---oo-----=----===
%%%###########********++++++++=+================oo-----=-------=
%@@%%%%%%%%%#######****+++++===========-=====ooo----------------
@@@@@@%%%%%%%%%%%%##******+++========----===oo====-==----------:
@@@@@@@%@@@@%%%%%%%##*******+++=======----oo==========-----:::::
@@@@@@@@%%%%%%%%%%%%#####*****++++=====-oo-==--=====-----::::...
@@@@@@@@@@@%%%%@@%%%%%%####***+++=====ooo---======---:::::::....
@%%%@@@@@@@%%%%%%%%%%%%###***+++====ooo===-======---::...::...
@%@@@@@@@@@@%%%%%%%%%XXXXXXXX+++++=oo=+++========--::::.......
%%%@@@@@@@@%%%%%%%%%XXXXXXXXXX++++oo========-------::::......
%%%%%%%%%%%########XXXXXXXXXXXX+oo======--=--------:::.....
#%%##%%%%####***###XXXXXXXXXXXXoo+=====--------::-::::.:...
##%#%%#####*********XXXXXXXXXXoo====-----------::::::......
##########************+XXXX++oo====-----------:::::::.......
#########***********++++==ooo=====----XXXX-----:::::::..........
####**#######*******++++=oo=====--==XXXXXXXX----:::.............
###**#########*****+++++oo=========XXXXXXXXXX----:::::::::......
################******oo++======-==XXXXXXXXXX==---:::::::::.....
#####**#**###**#***ooo++++===========XXXXXX=====----:::::::::...
#######********ooooo*++++====--============+=====-----:::-::::..
###*#*****oooooo***+++++====-----=---====++++======----:::::-:::
###***oooo***+*+++++++++===-----------====++========--------:--:
##***So*****+++++++++========-----------=====++++========-------
#********+++++++++++=========---:::-------===+++++============--
***++++++++=========+=+====---::::::::-----====++++++=+++=======
```

The exported WPL file (first waypoints; altitudes are MSL and follow the terrain):

```text
QGC WPL 110
0	1	0	16	0.00000000	0.00000000	0.00000000	0.00000000	47.00276271	8.00405222	967.931671	1
1	0	0	16	0.00000000	0.00000000	0.00000000	0.00000000	47.00276271	8.00405222	1017.931671	1
2	0	0	16	0.00000000	0.00000000	0.00000000	0.00000000	47.00348731	8.00568034	1002.042534	1
```

## Quick start

### C++ command line tool

Requires CMake 3.16+ and a C++17 compiler.

```bash
git clone https://github.com/bekirhankarakurt/uav-mission-planner.git
cd uav-mission-planner
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure

# plan on a synthetic map and write every export format
./build/uav-plan plan --synthetic 7 --seed 4 --planner thetastar --smooth \
    --circle-xy 1500,1700,350 --ascii \
    --out-wpl mission.waypoints --out-plan mission.plan --out-geojson mission.geojson --out-kml mission.kml

# plan on real SRTM data (tile origin comes from the file name)
./build/uav-plan plan --hgt N47E008.hgt --downsample 4 \
    --start 47.10,8.20 --goal 47.60,8.70 --zones no-fly.geojson --circle 47.30,8.40,1500 \
    --agl 80 --out-plan mission.plan
```

Prebuilt Linux binaries are attached to each [release](https://github.com/bekirhankarakurt/uav-mission-planner/releases).

### Python

```bash
pip install .            # builds the pybind11 extension (needs CMake and a C++17 compiler)
pip install ".[plot]"    # adds matplotlib for plotting
```

```python
import uav_planner as up

terrain = up.Terrain.synthetic(exponent=7, seed=4)        # or up.Terrain.from_hgt("N47E008.hgt")
zones = up.Zones()
zones.add_circle_xy(1500, 1700, 350)                       # x, y, radius in metres
mp = up.plan_mission(
    terrain, zones, start=(300, 300), goal=(3500, 3500),
    planner="thetastar", smooth=True,
    profile_config=up.ProfileConfig(agl_m=50, max_climb_gradient=0.35),
)
print(mp.result.length_m, mp.min_agl, len(mp.mission.items))

with open("mission.waypoints", "w") as f:
    f.write(up.to_wpl(mp.mission))
open("mission.plan", "w").write(up.to_qgc_plan(mp.mission, terrain, zones))

from uav_planner.viz import plot_mission                   # needs matplotlib
plot_mission(terrain, zones, mp, save="mission.png")
```

The Python command line mirrors the C++ one:

```bash
uav-planner plan --synthetic 7 --seed 4 --smooth --ascii --plot mission.png --out-wpl mission.waypoints
uav-planner benchmark --maps 10 --size 7
```

## Command line reference

| Command | Purpose |
|---|---|
| `uav-plan plan` | Plan a mission and export it |
| `uav-plan benchmark` | Compare A*, Theta* and RRT* on seeded synthetic maps |
| `uav-plan gen` | Write a synthetic terrain as an SRTM `.hgt` tile |
| `uav-plan info --hgt FILE` | Print tile size, origin, elevation range and filled voids |

Main options of `plan`:

| Option | Meaning | Default |
|---|---|---|
| `--hgt FILE` | SRTM tile, named like `N47E008.hgt` | |
| `--crop X0,Y0,W,H` / `--downsample N` | Crop window in cells / keep every Nth sample | |
| `--synthetic EXP` | Synthetic map of 2^EXP + 1 cells per side | 7 |
| `--seed N`, `--cell M`, `--max-height M`, `--roughness R` | Synthetic terrain parameters | 1, 30, 1500, 0.55 |
| `--start LAT,LON`, `--goal LAT,LON` | Endpoints (or `--start-xy X,Y`, `--goal-xy X,Y` in map metres) | map corners |
| `--zones FILE` | GeoJSON no-fly zones (repeatable) | |
| `--circle LAT,LON,R`, `--circle-xy X,Y,R` | Circular no-fly zone (repeatable) | |
| `--planner NAME` | `astar`, `thetastar` or `rrtstar` | `thetastar` |
| `--agl M` | Minimum height above ground | 50 |
| `--margin M` | Stand-off distance around zones | 0 |
| `--max-climb G`, `--max-descent G` | Gradient limits (rise / run) | 0.35 |
| `--climb-weight W`, `--terrain-weight W` | Cost model weights | 1.0, 0.25 |
| `--max-alt M` | MSL ceiling | none |
| `--rrt-iterations N`, `--rrt-step M`, `--rrt-seed N` | RRT* settings | 3000, auto, `--seed` |
| `--smooth` | Shortcut the path using line-of-sight checks | off |
| `--ascii` | Print an ASCII map | off |
| `--out-wpl`, `--out-plan`, `--out-geojson`, `--out-kml`, `--out-items` | Output files | |

Exit codes: `0` success, `1` no path found, `2` usage or input error.

## Planner comparison

`uav-plan benchmark --maps 20 --size 7 --seed 1`: 20 seeded 129 x 129 maps (30 m cells, 0 to 1500 m relief), five random circular no-fly zones each, from (8%, 8%) to (92%, 92%) of the map. RRT* runs 3000 iterations. Numbers are means over the solved maps, measured on a GitHub Actions runner (runtime varies by machine, the other columns are deterministic).

| Planner | Solved | Path length (m) | Length / straight | Cost | Runtime (ms) | Nodes |
|---|---|---|---|---|---|---|
| astar | 20/20 | 4983.2 | 1.092 | 7322.3 | 9.55 | 10813 |
| thetastar | 20/20 | 4803.0 | 1.053 | 6786.5 | 18.67 | 10027 |
| rrtstar | 20/20 | 4785.2 | 1.049 | 6962.1 | 115.61 | 2796 |

Cost is the planner objective (3D length plus climb and high-ground penalties). Nodes are expanded cells for A* and Theta*, and tree size for RRT*. Theta* gives the lowest cost, A* is the fastest, and RRT* finds the shortest horizontal paths but not the cheapest ones.

## Architecture

```text
 SRTM .hgt ----+                      +--> QGC WPL 110
 synthetic ----+--> Terrain --+       +--> QGC .plan (+ geofence)
                              |       +--> MAVLink item list (JSON)
 GeoJSON zones --> ZoneSet ---+--> PlanningContext --> A* / Theta* / RRT*
 circles ---------^                (cost model,            |
                                    collision checks)      v
                                                    smoothPath (shortcuts)
                                                           |
                                                           v
                              Profile (AGL clearance, climb/descent limits)
                                                           |
                                                           v
                                           Mission --> GeoJSON / KML / ASCII
```

| Path | Contents |
|---|---|
| `include/uav`, `src/` | Core library: `geo`, `json`, `terrain`, `zones`, `planner`, `profile`, `mission`, `export`, `render`, `benchmark` |
| `cli/main.cpp` | The `uav-plan` tool |
| `python/bindings.cpp` | pybind11 module `uav_planner._core` |
| `python/uav_planner/` | Python API re-exports, CLI (`cli.py`) and matplotlib plot (`viz.py`) |
| `tests/`, `python/tests/` | CTest unit tests (dependency-free) and pytest suite |

Coordinates inside the planner are metres east / north of the terrain's south-west corner; geographic positions are converted with an equirectangular mapping around the tile centre. `LocalFrame` in `geo.hpp` provides a proper East-North-Up conversion and haversine / bearing helpers.

## File formats

- **SRTM `.hgt`**: square tile of big-endian `int16` metres, rows north to south. 3601 x 3601 samples is 1 arc-second, 1201 x 1201 is 3 arc-second. `-32768` marks a void; voids are filled by repeatedly averaging valid neighbours. The tile origin is read from the file name (`N47E008.hgt` is the tile whose south-west corner is 47N 8E).
- **No-fly zones (GeoJSON)**: `Polygon` and `MultiPolygon` geometries (outer ring, `[lon, lat]`), and `Point` features with a `radius_m` property (metres) for circles. Other geometries are ignored.
  ```json
  {"type": "FeatureCollection", "features": [
    {"type": "Feature", "properties": {"name": "airfield"},
     "geometry": {"type": "Polygon", "coordinates": [[[8.1, 47.1], [8.2, 47.1], [8.2, 47.2], [8.1, 47.2], [8.1, 47.1]]]}},
    {"type": "Feature", "properties": {"name": "mast", "radius_m": 500},
     "geometry": {"type": "Point", "coordinates": [8.3, 47.3]}}]}
  ```
- **QGC WPL 110** (`--out-wpl`): tab separated `seq current frame command p1 p2 p3 p4 lat lon alt autocontinue`. Item 0 is the home position; waypoints use `NAV_WAYPOINT` (16) in the global MSL frame (0).
- **QGroundControl `.plan`** (`--out-plan`): JSON plan file with the waypoints as simple items and the no-fly zones as exclusion geofences.
- **MAVLink item list** (`--out-items`): JSON array of `MISSION_ITEM_INT` style objects (`x` and `y` are degrees x 1e7, `z` is metres MSL).
- **GeoJSON / KML** (`--out-geojson`, `--out-kml`): the flight path with altitudes plus the zones as polygons (circles become 48-gons), for viewing in geojson.io, QGIS or Google Earth. The GeoJSON can be fed back in as `--zones`.

## Troubleshooting

- **`status: no path`**: the start or goal is inside a zone, outside the map, or above the `--max-alt` ceiling; or zones (plus `--margin`) wall off the route. RRT* may also need more `--rrt-iterations`.
- **`hgt: file name must look like N47E008.hgt`**: the loader takes the tile origin from the file name. Rename the file or use the SRTM naming scheme.
- **`hgt: file is not a square tile`**: only raw SRTM `.hgt` files are supported, not zipped or GeoTIFF data. Unzip first.
- **Slow planning on full tiles**: a 3601 x 3601 tile has 13 million cells. Use `--downsample 4` (or more) or `--crop` to plan on a region.
- **Mission looks too high**: altitudes are `terrain + AGL`, raised further where the climb gradient limits require it. Lower `--agl` or raise `--max-climb` / `--max-descent` if the aircraft allows it.
- **`pip install .` fails to build**: the extension needs CMake 3.16+, a C++17 compiler and network access to fetch the pinned pybind11 release.
- **`matplotlib is required for plotting`**: install the extra with `pip install ".[plot]"`.

## Roadmap

- Wind and battery-aware cost models, and a fixed-wing turn-radius constraint.
- Multi-vehicle and multi-goal (survey) missions.
- GeoTIFF and zipped SRTM input, and automatic tile stitching.
- Takeoff, landing and return-to-launch mission items.
- Time-based and altitude-band airspace zones.

## License

MIT, see [LICENSE](LICENSE). Contributions are welcome, see [CONTRIBUTING.md](CONTRIBUTING.md); report security issues as described in [SECURITY.md](SECURITY.md).

Bekirhan Karakurt - github.com/bekirhankarakurt
