# Changelog

All notable changes to this project are documented in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [1.0.0] - 2026-11-05

### Added
- C++17 core library with no external dependencies.
- SRTM `.hgt` loader (1-arc-second and 3-arc-second tiles, big-endian int16) with void filling.
- Seeded diamond-square synthetic terrain generator.
- No-fly zones as circles and polygons, with a GeoJSON parser (Polygon, MultiPolygon, Point with radius).
- Grid A*, Theta* (any-angle) and RRT* (with rewiring) planners that account for terrain and climb cost.
- Path smoothing by greedy shortcutting with line-of-sight checks.
- Terrain-following altitude profile with minimum AGL clearance and climb / descent gradient limits.
- Great-circle (haversine) helpers and local ENU conversion.
- Export to QGroundControl WPL 110, QGroundControl `.plan`, a MAVLink-style mission item list, GeoJSON and KML.
- `uav-plan` command line tool (`plan`, `benchmark`, `gen`, `info`).
- `uav_planner` Python package (pybind11) with a CLI and an optional matplotlib plot.
- Benchmark comparing the three planners on seeded synthetic maps.
- C++ unit tests (CTest), pytest suite, and CI with GCC, Clang, ASan/UBSan and Python jobs.
