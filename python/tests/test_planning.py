import math

import pytest

import uav_planner as up

START = (150.0, 150.0)
GOAL = (1750.0, 1750.0)


def _segment_samples(path, step=3.0):
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        for k in range(n + 1):
            f = k / n
            yield x0 + (x1 - x0) * f, y0 + (y1 - y0) * f


@pytest.mark.parametrize("planner", up.PLANNERS)
def test_mission_is_valid(terrain, zones, planner):
    profile = up.ProfileConfig(agl_m=60.0, max_climb_gradient=0.3, max_descent_gradient=0.4)
    mp = up.plan_mission(
        terrain, zones, START, GOAL, planner=planner,
        planner_config=up.PlannerConfig(zone_margin_m=10.0),
        profile_config=profile, rrt=up.RrtParams(seed=5), smooth=True,
    )
    assert mp.found, mp.result.message
    assert mp.path[0] == pytest.approx(START) and mp.path[-1] == pytest.approx(GOAL)

    # never inside a no-fly zone, including between waypoints
    for x, y in _segment_samples(mp.path):
        assert not zones.blocked(x, y)

    # AGL clearance and gradient limits
    assert mp.min_agl >= 60.0 - 1e-6
    prof = mp.profile
    for x, y, _s, ground, alt in prof:
        assert alt - terrain.elevation(x, y) >= 60.0 - 1e-6
        assert ground == pytest.approx(terrain.elevation(x, y))
    for (_, _, s0, _, a0), (_, _, s1, _, a1) in zip(prof, prof[1:]):
        assert a1 - a0 <= 0.3 * (s1 - s0) + 1e-9
        assert a0 - a1 <= 0.4 * (s1 - s0) + 1e-9
    for item in mp.mission.items:
        ground = terrain.elevation(*terrain.to_xy(item.lat, item.lon))
        assert item.alt - ground >= 60.0 - 1e-3


@pytest.mark.parametrize("planner", up.PLANNERS)
def test_path_is_not_shorter_than_straight_line(terrain, zones, planner):
    res = up.plan_path(terrain, zones, START, GOAL, planner=planner)
    assert res.found
    straight = math.dist(START, GOAL)
    assert straight - 1e-6 <= res.length_m <= 2.5 * straight


def test_open_flat_ground_gives_the_straight_line():
    flat = up.Terrain.from_rows([[0.0] * 41 for _ in range(41)], cell_m=10.0)
    res = up.plan_path(flat, up.Zones(), (0.0, 0.0), (400.0, 400.0), planner="thetastar")
    assert res.found
    assert res.length_m == pytest.approx(400.0 * math.sqrt(2.0), abs=1e-6)


def test_blocked_start_reports_failure(terrain, zones):
    mp = up.plan_mission(terrain, zones, (960.0, 960.0), GOAL)
    assert not mp.found
    assert mp.result.message


def test_unknown_planner_is_rejected(terrain, zones):
    with pytest.raises(RuntimeError):
        up.plan_path(terrain, zones, START, GOAL, planner="dijkstra")


def test_config_keywords():
    cfg = up.PlannerConfig(agl_m=80.0, climb_weight=2.5)
    assert cfg.agl_m == 80.0 and cfg.climb_weight == 2.5
    with pytest.raises(TypeError):
        up.PlannerConfig(bogus=1)
    assert up.RrtParams(max_iterations=10, seed=3).max_iterations == 10


def test_benchmark_rows():
    rows = up.run_benchmark(maps=2, exponent=5, seed=2, rrt_iterations=600)
    assert [r.planner for r in rows] == ["astar", "thetastar", "rrtstar"]
    assert rows[0].solved == 2 and rows[1].solved == 2
    assert "| Planner |" in up.benchmark_markdown(rows)
    assert up.benchmark_csv(rows).count("\n") == 4
