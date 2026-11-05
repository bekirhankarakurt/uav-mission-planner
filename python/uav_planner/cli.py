"""Command line interface: ``uav-planner plan | benchmark``."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

from . import (
    PLANNERS,
    PlannerConfig,
    PlannerError,
    ProfileConfig,
    RrtParams,
    Terrain,
    Zones,
    __version__,
    benchmark_csv,
    benchmark_markdown,
    plan_mission,
    render_ascii,
    run_benchmark,
    to_geojson,
    to_kml,
    to_mavlink_items_json,
    to_qgc_plan,
    to_wpl,
)


def _floats(text: str, count: int, name: str) -> Tuple[float, ...]:
    try:
        values = tuple(float(part) for part in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"--{name}: expected numbers separated by commas") from None
    if len(values) != count:
        raise argparse.ArgumentTypeError(f"--{name}: expected {count} comma-separated numbers")
    return values


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="uav-planner", description="Terrain-aware UAV mission planner.")
    parser.add_argument("--version", action="version", version=f"uav-planner {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("plan", help="plan a mission and export it")
    terrain = p.add_argument_group("terrain")
    terrain.add_argument("--hgt", help="SRTM .hgt tile (file name like N47E008.hgt)")
    terrain.add_argument("--downsample", type=int, default=1, help="keep every Nth sample of an .hgt tile")
    terrain.add_argument("--synthetic", type=int, default=7, metavar="EXP", help="synthetic map of 2^EXP+1 cells")
    terrain.add_argument("--seed", type=int, default=1)
    terrain.add_argument("--cell", type=float, default=30.0, help="synthetic cell size in metres")
    terrain.add_argument("--max-height", type=float, default=1500.0)
    zones = p.add_argument_group("zones and endpoints")
    zones.add_argument("--zones", action="append", default=[], help="GeoJSON no-fly zones (repeatable)")
    zones.add_argument("--circle", action="append", default=[], metavar="LAT,LON,R")
    zones.add_argument("--circle-xy", action="append", default=[], metavar="X,Y,R")
    zones.add_argument("--start", metavar="LAT,LON")
    zones.add_argument("--goal", metavar="LAT,LON")
    zones.add_argument("--start-xy", metavar="X,Y")
    zones.add_argument("--goal-xy", metavar="X,Y")
    planning = p.add_argument_group("planning")
    planning.add_argument("--planner", choices=PLANNERS, default="thetastar")
    planning.add_argument("--agl", type=float, default=50.0, help="minimum height above ground (m)")
    planning.add_argument("--margin", type=float, default=0.0, help="stand-off distance around zones (m)")
    planning.add_argument("--max-climb", type=float, default=0.35)
    planning.add_argument("--max-descent", type=float, default=0.35)
    planning.add_argument("--rrt-iterations", type=int, default=3000)
    planning.add_argument("--smooth", action="store_true", help="shortcut the path with line-of-sight checks")
    out = p.add_argument_group("output")
    out.add_argument("--out-wpl")
    out.add_argument("--out-plan")
    out.add_argument("--out-geojson")
    out.add_argument("--out-kml")
    out.add_argument("--out-items")
    out.add_argument("--plot", metavar="IMAGE", help="save a matplotlib figure (needs the 'plot' extra)")
    out.add_argument("--show", action="store_true", help="open an interactive matplotlib window")
    out.add_argument("--ascii", action="store_true", help="print an ASCII map")

    b = sub.add_parser("benchmark", help="compare A*, Theta* and RRT* on seeded synthetic maps")
    b.add_argument("--maps", type=int, default=10)
    b.add_argument("--size", type=int, default=7, metavar="EXP")
    b.add_argument("--seed", type=int, default=1)
    b.add_argument("--rrt-iterations", type=int, default=3000)
    b.add_argument("--csv", action="store_true")
    return parser


def _load_terrain(args: argparse.Namespace) -> Terrain:
    if args.hgt:
        terrain = Terrain.from_hgt(args.hgt)
        return terrain.downsample(args.downsample) if args.downsample > 1 else terrain
    return Terrain.synthetic(exponent=args.synthetic, seed=args.seed, cell_m=args.cell, max_height_m=args.max_height)


def _point(terrain: Terrain, geo: Optional[str], xy: Optional[str], fraction: float, name: str):
    if geo:
        lat, lon = _floats(geo, 2, name)
        return terrain.to_xy(lat, lon)
    if xy:
        x, y = _floats(xy, 2, f"{name}-xy")
        return (x, y)
    return (fraction * terrain.width_m, fraction * terrain.height_m)


def _write(path: str, text: str) -> None:
    Path(path).write_text(text, encoding="utf-8")


def _cmd_plan(args: argparse.Namespace) -> int:
    terrain = _load_terrain(args)
    zones = Zones()
    for name in args.zones:
        extra = Zones.from_geojson(Path(name).read_text(encoding="utf-8"), terrain)
        for x, y, r in extra.circles:
            zones.add_circle_xy(x, y, r)
        for pts in extra.polygons:
            zones.add_polygon_xy(pts)
    for text in args.circle:
        lat, lon, r = _floats(text, 3, "circle")
        zones.add_circle(terrain, lat, lon, r)
    for text in args.circle_xy:
        x, y, r = _floats(text, 3, "circle-xy")
        zones.add_circle_xy(x, y, r)

    start = _point(terrain, args.start, args.start_xy, 0.08, "start")
    goal = _point(terrain, args.goal, args.goal_xy, 0.92, "goal")
    mp = plan_mission(
        terrain,
        zones,
        start,
        goal,
        planner=args.planner,
        planner_config=PlannerConfig(zone_margin_m=args.margin),
        profile_config=ProfileConfig(
            agl_m=args.agl, max_climb_gradient=args.max_climb, max_descent_gradient=args.max_descent
        ),
        rrt=RrtParams(max_iterations=args.rrt_iterations, seed=args.seed),
        smooth=args.smooth,
    )
    res = mp.result
    print(
        f"terrain:  {terrain.cols}x{terrain.rows} cells, {terrain.cell_x:.1f} m spacing, "
        f"elevation {terrain.min_elevation:.1f}..{terrain.max_elevation:.1f} m"
    )
    print(f"planner:  {res.planner}{' + smoothing' if args.smooth else ''}")
    if not res.found:
        print(f"status:   no path ({res.message})")
        return 1
    print("status:   path found")
    print(f"path:     {len(mp.path)} vertices, {res.length_m:.1f} m horizontal")
    print(f"cost:     {res.cost:.1f}")
    print(f"runtime:  {res.runtime_ms:.2f} ms, {res.nodes} nodes")
    print(
        f"altitude: min AGL {mp.min_agl:.2f} m, max climb gradient {mp.max_climb:.2f}, "
        f"max descent gradient {mp.max_descent:.2f}, {len(mp.mission.items)} waypoints"
    )

    if args.out_wpl:
        _write(args.out_wpl, to_wpl(mp.mission))
    if args.out_plan:
        _write(args.out_plan, to_qgc_plan(mp.mission, terrain, zones))
    if args.out_geojson:
        _write(args.out_geojson, to_geojson(mp.mission, terrain, zones))
    if args.out_kml:
        _write(args.out_kml, to_kml(mp.mission, terrain, zones))
    if args.out_items:
        _write(args.out_items, to_mavlink_items_json(mp.mission))
    if args.ascii:
        print()
        print(render_ascii(terrain, zones, mp.path, start, goal), end="")
    if args.plot or args.show:
        from .viz import plot_mission

        plot_mission(terrain, zones, mp, start=start, goal=goal, save=args.plot, show=args.show)
    return 0


def _cmd_benchmark(args: argparse.Namespace) -> int:
    rows = run_benchmark(maps=args.maps, exponent=args.size, seed=args.seed, rrt_iterations=args.rrt_iterations)
    if args.csv:
        print(benchmark_csv(rows), end="")
    else:
        side = 2**args.size + 1
        print(f"Benchmark: {args.maps} maps, {side}x{side} cells, seed {args.seed}, RRT* {args.rrt_iterations} iterations")
        print()
        print(benchmark_markdown(rows), end="")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "plan":
            return _cmd_plan(args)
        return _cmd_benchmark(args)
    except (PlannerError, OSError, argparse.ArgumentTypeError, ImportError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
