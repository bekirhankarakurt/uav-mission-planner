import pytest

import uav_planner as up


@pytest.fixture(scope="session")
def terrain():
    return up.Terrain.synthetic(exponent=6, seed=11)


@pytest.fixture()
def zones(terrain):
    z = up.Zones()
    z.add_circle_xy(960.0, 960.0, 220.0)
    z.add_polygon_xy([(300, 900), (700, 1000), (650, 1500), (250, 1400)])
    return z
