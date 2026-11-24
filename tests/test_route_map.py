"""경로 위 횡단보도 위치는 버스 경로가 횡단보도 띠에 들어서는 곳이다."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from env.road_config import EGO, ROAD, configure_map  # noqa: E402
from env.route_map import RouteMap  # noqa: E402


@pytest.fixture(scope="module")
def route_map():
    configure_map("baseline", 0, None, "hapjeong")
    network = ROAD["network"]
    return RouteMap(network["net_path"], ROAD["ego_route"], network["routes"][ROAD["ego_route"]],
                    network.get("bus_stops", []), EGO["vclass"])


def _road_start(route_map, edge_id):
    return route_map.start[route_map.edge_ids.index(edge_id)]


def test_crosswalk_at_the_stop_line_is_where_the_road_ends(route_map):
    crossings = {name: at for at, name in route_map.crossings}
    r111b = route_map.edge_ids.index("R111b")
    assert crossings[":J82_c3"] == pytest.approx(route_map.end[r111b])


def test_crosswalk_on_the_way_out_of_a_left_turn_starts_inside_the_junction(route_map):
    """J82를 좌회전해 나가는 R112 앞 횡단보도(폭 4m)를 버스는 비스듬히 지난다. 3.18.0에서 이 횡단보도를 바깥쪽으로
    0.5m 옮겨 띠가 R112 시작 뒤 0.5m까지 걸친다. R112 시작을 횡단보도 위치로 적으면 그 3m 앞에 서는 버스가 횡단보도 위에
    선다(dev busy 1601, 보행자 충돌). 3.21.0부터 버스는 안쪽 차로끼리(R111b 3 -> R112 2) 돌아 띠에 R112 시작 3.75m 앞에서
    들어선다(3.20.0까지 바깥 좌회전 R111b 2 -> R112 1은 8.7m)."""
    crossings = {name: at for at, name in route_map.crossings}
    assert _road_start(route_map, "R112") - crossings[":J82_c0"] == pytest.approx(3.75, abs=0.2)
    assert _road_start(route_map, "R54a") - crossings[":J42_c2"] == pytest.approx(14.6, abs=0.2)   # 3.17.0 곡선 좌회전, 3.18.0에서 0.5m 옮김(전 15m)


def test_bus_on_the_crosswalk_no_longer_sees_it_ahead(route_map):
    """띠에 들어선 뒤에는 그 횡단보도가 앞에 있는 것이 아니다. 버스는 서지 않고 빠져나간다."""
    on_strip = _road_start(route_map, "R112") - 2.0
    assert route_map.ahead(on_strip)["crossing_id"] != ":J82_c0"
    assert route_map.ahead(on_strip - 10.0)["crossing_id"] == ":J82_c0"
