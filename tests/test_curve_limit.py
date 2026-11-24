"""버스의 곡선 속도 제한은 차로 형상에서 나오고, 그 속도로 돌면 횡가속도가 한계를 넘지 않는다."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from env.road_config import EGO, ROAD, configure_map  # noqa: E402
from env.route_map import RouteMap, curve_speeds  # noqa: E402

LIMIT, BUS = 2.0, 12.0


def _arc(radius, degrees):
    """원점에서 +x로 출발해 왼쪽으로 도는 원호의 점들(0.5m 간격)."""
    step_deg = math.degrees(0.5 / radius)
    count = int(degrees / step_deg)
    return [(radius * math.sin(math.radians(k * step_deg)), radius * (1.0 - math.cos(math.radians(k * step_deg))))
            for k in range(count + 1)]


def test_straight_lane_has_no_limit():
    assert curve_speeds([(-50.0, 0.0), (0.0, 0.0)], [(0.0, 0.0), (80.0, 0.0)], 80.0, BUS, LIMIT) == [math.inf] * 80


@pytest.mark.parametrize("radius", [15.0, 40.0, 100.0])
def test_arc_limit_is_the_speed_with_that_lateral_acceleration(radius):
    """반지름 R인 원호 위에서는 v²/R = 한계인 속도다. 차체 길이의 현으로 재므로 R이 작을수록 조금(R 15m에서 3% 미만) 높다."""
    shape = _arc(radius, 170.0)
    length = math.radians(170.0) * radius
    speeds = curve_speeds([(-50.0, 0.0), (0.0, 0.0)], shape, length, BUS, LIMIT)
    inside = speeds[int(BUS) + 1:int(length) - 1]     # 차체 전체가 원호 위에 있는 구간
    assert min(inside) == pytest.approx(math.sqrt(LIMIT * radius), rel=0.03)
    assert max(inside) == pytest.approx(math.sqrt(LIMIT * radius), rel=0.03)


def test_kink_limits_until_the_rear_bumper_has_passed_it():
    """40도로 꺾인 점 하나. 차체(12m)가 그 점을 지나는 동안 차체는 40도/12m로 돌고, 뒤범퍼가 지나면 제한이 없다."""
    turn = math.radians(40.0)
    shape = [(0.0, 0.0), (30.0, 0.0), (30.0 + 50.0 * math.cos(turn), 50.0 * math.sin(turn))]
    speeds = curve_speeds([], shape, 80.0, BUS, LIMIT)
    assert speeds[:29] == [math.inf] * 29
    assert min(speeds) == pytest.approx(math.sqrt(LIMIT * BUS / turn), rel=0.08)
    assert all(speed < 7.0 for speed in speeds[31:41])
    assert speeds[44:] == [math.inf] * (80 - 44)


def test_kink_at_the_lane_boundary_belongs_to_the_lane_after_it():
    """앞 차로 끝과 이 차로 시작 사이가 꺾여 있으면(합정 -R96 -> -R96.64, 43도) 이 차로의 첫 12m가 제한된다."""
    turn = math.radians(43.0)
    speeds = curve_speeds([(-40.0, 0.0), (0.0, 0.0)], [(0.0, 0.0), (60.0 * math.cos(turn), 60.0 * math.sin(turn))],
                          60.0, BUS, LIMIT)
    assert max(speeds[:11]) < 6.5 and speeds[14:] == [math.inf] * 46


@pytest.fixture(scope="module")
def route_map():
    configure_map("baseline", 0, None, "hapjeong")
    network = ROAD["network"]
    return RouteMap(network["net_path"], ROAD["ego_route"], network["routes"][ROAD["ego_route"]],
                    network.get("bus_stops", []), EGO["vclass"], EGO["comfort"]["lateral_acceleration_limit"], EGO["length"])


def test_hapjeong_left_turns_are_limited_below_the_network_speed(route_map):
    """J82 좌회전(3.21.0부터 안쪽 차로끼리 R111b 3 -> R112 2, 대기 위치 뒤 :J82_23_0)의 도로망 제한은 9.21m/s,
    J42 좌회전 끝은 13.89m/s다."""
    assert 5.0 < route_map.curve_speed(":J82_23_0", 11.0) < 6.5
    assert 4.0 < route_map.curve_speed(":J42_24_0", 6.0) < 6.5
    # 좌회전을 마친 뒤에도 뒤범퍼가 교차로를 벗어날 때까지는 제한이 이어지고, 그 뒤 곧은 도로에는 제한이 없다.
    assert route_map.curve_speed("R54a_1", 5.0) < 6.5
    assert route_map.curve_speed("R54a_1", 100.0) == math.inf
    assert route_map.curve_speed("R71_2", 50.0) == math.inf


def test_curve_speeds_ahead_lists_the_rest_of_the_lane(route_map):
    ahead = list(route_map.curve_speeds_ahead(":J82_23_0", 12.4))
    assert ahead[0] == (12.4, route_map.curve_speed(":J82_23_0", 12.4))
    assert [at for at, _speed in ahead[1:]] == [13.0, 14.0, 15.0]
    assert list(route_map.curve_speeds_ahead("R71_2")) == []


def test_no_limit_without_a_comfort_limit():
    """승용차 ego(팔달문)는 횡가속도 한계가 없어 곡선 제한도 없다."""
    configure_map("baseline", 0, None, "hapjeong")
    network = ROAD["network"]
    plain = RouteMap(network["net_path"], ROAD["ego_route"], network["routes"][ROAD["ego_route"]],
                     network.get("bus_stops", []), EGO["vclass"])
    assert plain.curve_speed(":J82_23_0", 11.0) == math.inf
