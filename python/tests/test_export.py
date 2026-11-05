import json
import xml.etree.ElementTree as ET

import pytest

import uav_planner as up


@pytest.fixture()
def mission_plan(terrain, zones):
    return up.plan_mission(terrain, zones, (150.0, 150.0), (1750.0, 1750.0), smooth=True)


def _same(a, b, ang=1e-7, alt=1e-5):
    assert a.home_lat == pytest.approx(b.home_lat, abs=ang)
    assert a.home_lon == pytest.approx(b.home_lon, abs=ang)
    assert len(a.items) == len(b.items)
    for x, y in zip(a.items, b.items):
        assert (x.command, x.frame) == (y.command, y.frame)
        assert x.lat == pytest.approx(y.lat, abs=ang)
        assert x.lon == pytest.approx(y.lon, abs=ang)
        assert x.alt == pytest.approx(y.alt, abs=alt)


def test_wpl_round_trip(mission_plan):
    text = up.to_wpl(mission_plan.mission)
    lines = text.splitlines()
    assert lines[0] == "QGC WPL 110"
    assert all(len(line.split("\t")) == 12 for line in lines[1:])
    _same(mission_plan.mission, up.parse_wpl(text))


def test_wpl_rejects_garbage():
    with pytest.raises(RuntimeError):
        up.parse_wpl("hello")


def test_qgc_plan_round_trip(mission_plan, terrain, zones):
    text = up.to_qgc_plan(mission_plan.mission, terrain, zones)
    doc = json.loads(text)
    assert doc["fileType"] == "Plan"
    assert len(doc["mission"]["items"]) == len(mission_plan.mission.items)
    assert len(doc["geoFence"]["circles"]) == 1 and len(doc["geoFence"]["polygons"]) == 1
    assert doc["geoFence"]["circles"][0]["inclusion"] is False
    _same(mission_plan.mission, up.parse_qgc_plan(text), ang=1e-9, alt=1e-6)


def test_mavlink_items(mission_plan):
    items = json.loads(up.to_mavlink_items_json(mission_plan.mission))
    assert len(items) == len(mission_plan.mission.items) + 1
    assert [i["seq"] for i in items] == list(range(len(items)))
    assert items[0]["current"] == 1
    assert items[1]["x"] == round(mission_plan.mission.items[0].lat * 1e7)


def test_geojson_and_kml(mission_plan, terrain, zones):
    gj = json.loads(up.to_geojson(mission_plan.mission, terrain, zones))
    assert gj["type"] == "FeatureCollection"
    line = gj["features"][0]["geometry"]
    assert line["type"] == "LineString"
    assert len(line["coordinates"]) == len(mission_plan.mission.items)
    assert len(gj["features"]) == 3

    root = ET.fromstring(up.to_kml(mission_plan.mission, terrain, zones))
    ns = {"k": "http://www.opengis.net/kml/2.2"}
    assert len(root.findall(".//k:Placemark", ns)) == 3
