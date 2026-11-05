import pytest

import uav_planner as up


def test_synthetic_terrain_is_seeded():
    a = up.Terrain.synthetic(exponent=5, seed=3)
    b = up.Terrain.synthetic(exponent=5, seed=3)
    c = up.Terrain.synthetic(exponent=5, seed=4)
    assert a.heights() == b.heights()
    assert a.heights() != c.heights()
    assert a.cols == a.rows == 33
    assert a.min_elevation == pytest.approx(0.0, abs=1e-3)
    assert a.max_elevation == pytest.approx(1500.0, abs=1e-1)


def test_bilinear_elevation_from_rows():
    t = up.Terrain.from_rows([[0, 10], [20, 30]], cell_m=10.0)
    assert t.elevation(5.0, 5.0) == pytest.approx(15.0)
    assert t.elevation(-3.0, 100.0) == pytest.approx(20.0)


def test_latlon_round_trip(terrain):
    lat, lon = terrain.to_latlon(500.0, 700.0)
    x, y = terrain.to_xy(lat, lon)
    assert (x, y) == (pytest.approx(500.0, abs=1e-6), pytest.approx(700.0, abs=1e-6))


def test_hgt_round_trip():
    t = up.Terrain.synthetic(exponent=3, seed=2, max_height_m=2000.0)
    raw = t.to_hgt()
    assert len(raw) == 9 * 9 * 2
    back = up.Terrain.from_hgt_bytes(raw, 47.0, 8.0)
    for row_a, row_b in zip(t.heights(), back.heights()):
        assert row_a == pytest.approx(row_b, abs=0.51)


def test_hgt_void_is_filled():
    samples = [10, 10, 10, 10, -32768, 10, 10, 10, 10]
    raw = b"".join(v.to_bytes(2, "big", signed=True) for v in samples)
    t = up.Terrain.from_hgt_bytes(raw, 0.0, 0.0)
    assert t.voids_filled == 1
    assert t.heights()[1][1] == pytest.approx(10.0)


def test_invalid_hgt_is_rejected():
    with pytest.raises(RuntimeError):
        up.Terrain.from_hgt_bytes(b"\x00" * 10, 0.0, 0.0)
    with pytest.raises(up.PlannerError):
        up.Terrain.from_hgt("missing/N47E008.hgt")


def test_zones(terrain, zones):
    assert len(zones) == 2
    assert zones.blocked(960.0, 960.0)
    assert not zones.blocked(50.0, 50.0)
    assert zones.segment_blocked(0.0, 960.0, 1900.0, 960.0)
    assert not zones.segment_blocked(0.0, 0.0, 1900.0, 0.0)
    with pytest.raises(RuntimeError):
        zones.add_circle_xy(0.0, 0.0, -1.0)


def test_geojson_zones(terrain):
    text = (
        '{"type":"FeatureCollection","features":['
        '{"type":"Feature","properties":{},"geometry":{"type":"Polygon","coordinates":'
        "[[[8.0,47.0],[8.01,47.0],[8.01,47.01],[8.0,47.01],[8.0,47.0]]]}},"
        '{"type":"Feature","properties":{"radius_m":300},"geometry":{"type":"Point","coordinates":[8.02,47.02]}}]}'
    )
    z = up.Zones.from_geojson(text, terrain)
    assert len(z.polygons) == 1 and len(z.circles) == 1
    assert z.circles[0][2] == pytest.approx(300.0)


def test_geodesy_helpers():
    assert up.haversine(0.0, 0.0, 1.0, 0.0) == pytest.approx(111194.93, abs=1.0)
    assert up.bearing(0.0, 0.0, 0.0, 1.0) == pytest.approx(90.0)
    lat, lon = up.destination(47.0, 8.0, 45.0, 10000.0)
    assert up.haversine(47.0, 8.0, lat, lon) == pytest.approx(10000.0, abs=1e-3)
    e, n, u = up.to_enu(47.0, 8.0, 47.01, 8.0)
    assert n == pytest.approx(1111.95, abs=2.0)
    lat2, lon2, alt = up.from_enu(47.0, 8.0, e, n, u)
    assert (lat2, lon2) == (pytest.approx(47.01, abs=1e-9), pytest.approx(8.0, abs=1e-9))
