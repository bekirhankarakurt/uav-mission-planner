import json

import pytest

from uav_planner import cli


def test_plan_command_writes_all_formats(tmp_path, capsys):
    out = {k: tmp_path / f"mission.{k}" for k in ("wpl", "plan", "geojson", "kml", "items")}
    rc = cli.main([
        "plan", "--synthetic", "6", "--seed", "3", "--planner", "thetastar", "--smooth", "--ascii",
        "--circle-xy", "960,960,200",
        "--out-wpl", str(out["wpl"]), "--out-plan", str(out["plan"]), "--out-geojson", str(out["geojson"]),
        "--out-kml", str(out["kml"]), "--out-items", str(out["items"]),
    ])
    assert rc == 0
    text = capsys.readouterr().out
    assert "path found" in text and "S" in text and "G" in text
    assert out["wpl"].read_text().startswith("QGC WPL 110")
    assert json.loads(out["plan"].read_text())["fileType"] == "Plan"
    assert json.loads(out["geojson"].read_text())["type"] == "FeatureCollection"
    assert "<kml" in out["kml"].read_text()
    assert isinstance(json.loads(out["items"].read_text()), list)


def test_plan_reports_missing_path(capsys):
    rc = cli.main(["plan", "--synthetic", "5", "--start-xy", "100,100", "--goal-xy", "100,100000"])
    assert rc == 1
    assert "no path" in capsys.readouterr().out


def test_bad_arguments_exit_with_error(capsys):
    assert cli.main(["plan", "--synthetic", "5", "--circle", "1,2"]) == 2
    assert "error" in capsys.readouterr().err
    with pytest.raises(SystemExit):
        cli.main(["plan", "--planner", "dijkstra"])


def test_benchmark_command(capsys):
    assert cli.main(["benchmark", "--maps", "2", "--size", "5", "--rrt-iterations", "500"]) == 0
    out = capsys.readouterr().out
    assert "astar" in out and "thetastar" in out and "rrtstar" in out
    assert cli.main(["benchmark", "--maps", "1", "--size", "5", "--rrt-iterations", "300", "--csv"]) == 0
    assert capsys.readouterr().out.startswith("planner,solved")


def test_plot(tmp_path, terrain, zones):
    pytest.importorskip("matplotlib")
    from uav_planner import plan_mission
    from uav_planner.viz import plot_mission

    mp = plan_mission(terrain, zones, (150.0, 150.0), (1750.0, 1750.0))
    target = tmp_path / "plan.png"
    plot_mission(terrain, zones, mp, save=str(target))
    assert target.stat().st_size > 1000
