"""정지선에서 옆 차로로 끝내 못 옮긴 차는 지금 차로가 가는 쪽으로 간다(traffic/lane_route.py)."""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic import lane_route  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.lane_route import LaneRouteAdapter, reachable_next  # noqa: E402
from traffic.presence import boundary_nodes  # noqa: E402

SOURCE = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong", "source")
NET = os.path.join(SOURCE, "network.net.xml")
R94_END = 88.5          # R94 길이. 정지선 바로 앞
_cache = {}


def _net():
    if "net" not in _cache:
        _cache["net"] = read_net(NET)
    return _cache["net"]


def test_reachable_next_lists_only_the_roads_this_lane_connects_to():
    # J42 R94: 차로 1은 R92(직진)·R54a(좌회전), 차로 2는 R54a(좌회전)뿐이다(3.8.0: 우회전 R53 없음)
    assert reachable_next(_net(), "R94", 1) == {"R92", "R54a"}
    assert reachable_next(_net(), "R94", 2) == {"R54a"}


class _V:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return list(self.tc.cars)

    def getRoadID(self, v):
        return self.tc.cars[v]["road"]

    def getLaneIndex(self, v):
        return self.tc.cars[v]["lane"]

    def getLanePosition(self, v):
        return self.tc.cars[v]["pos"]

    def getSpeed(self, v):
        return self.tc.cars[v]["speed"]

    def getWaitingTime(self, v):
        return self.tc.cars[v]["wait"]

    def getRoute(self, v):
        return self.tc.cars[v]["route"]

    def getRouteIndex(self, v):
        return self.tc.cars[v]["route"].index(self.tc.cars[v]["road"])

    def setRoute(self, v, edges):
        self.tc.cars[v]["route"] = list(edges)
        self.tc.set_routes.append((v, list(edges)))


class _TC:
    def __init__(self, cars):
        self.cars, self.set_routes = cars, []
        self.vehicle = _V(self)


def _car(lane, route, road="R94", pos=R94_END - 1.0, speed=0.0, wait=10_000.0):
    """기본값은 정지선에 서서 오래 기다린 차다."""
    return {"road": road, "lane": lane, "pos": pos, "speed": speed, "wait": wait, "route": route}


def _update(cars):
    tc = _TC(cars)
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    return tc.set_routes


def test_moving_car_in_a_lane_that_does_not_go_its_way_keeps_its_route():
    # 정지선 18 m 앞에서 아직 달리는 차는 차로를 옮기는 중이다. SUMO 차선 변경 모델에 맡긴다
    assert _update({"v": _car(2, ["R94", "R92"], pos=70.0, speed=8.0, wait=0.0)}) == []


def test_car_stopped_at_the_stop_line_for_less_than_a_signal_cycle_keeps_its_route():
    # 적색에 선 지 1분: 녹색이 오면 옆 차로로 옮길 수 있다
    assert _update({"v": _car(2, ["R94", "R92"], wait=60.0)}) == []


def test_car_stuck_at_the_stop_line_takes_the_way_its_lane_goes():
    # 직진(R92)할 차가 좌회전 전용 차로 2의 정지선에서 끝내 못 옮겼다: 차로 2가 가는 R54a로 간다
    new = _update({"v": _car(2, ["R94", "R92"])})[0][1]
    assert new[:2] == ["R94", "R54a"] and "R94" not in new[1:]
    # 목적지 R92로 돌아가려면 크게 돌아야 해서 가까운 지도 경계로 나간다
    assert _net().getEdge(new[-1]).getToNode().getID() in boundary_nodes(_net())
    assert sum(_net().getEdge(e).getLength() for e in new[1:]) < 800.0


def test_new_route_starts_at_the_current_road_not_at_the_past_roads():
    # SUMO setRoute는 지금 있는 도로에서 시작하는 경로를 받는다
    new = _update({"v": _car(2, ["-R96.119", "-R96.201", "R94", "R92"])})[0][1]
    assert new[:2] == ["R94", "R54a"]


def test_a_failed_route_does_not_stop_the_other_cars():
    tc = _TC({"v": _car(2, ["R94", "R92"]), "w": _car(2, ["R94", "R92"])})
    real = tc.vehicle.setRoute

    def flaky(v, edges):
        if v == "v":
            raise RuntimeError("no connection")
        real(v, edges)
    tc.vehicle.setRoute = flaky
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    assert [r[0] for r in tc.set_routes] == ["w"]


def test_stuck_car_in_a_lane_that_goes_its_way_keeps_its_route():
    assert _update({"a": _car(1, ["R94", "R92"]), "b": _car(2, ["R94", "R54a", "R54b"])}) == []


def test_never_u_turns():
    # R128 차로 2는 지도 끝의 회차(-R128)로만 이어진다. 회차는 진로 바꾸기 후보가 아니다
    end = _net().getEdge("R128").getLength() - 1.0
    assert _update({"v": _car(2, ["R128", "R129"], road="R128", pos=end)}) == []


def test_test_bus_is_never_touched():
    assert _update({"bus0": _car(2, ["R94", "R92"])}) == []


def _cycles():
    """도로망과 signals.add.xml의 신호 프로그램마다 주기(초)."""
    out = {}
    for name in ("network.net.xml", "signals.add.xml"):
        for logic in ET.parse(os.path.join(SOURCE, name)).getroot().iter("tlLogic"):
            out[(logic.get("id"), logic.get("programID"))] = sum(float(p.get("duration")) for p in logic.iter("phase"))
    return out


def test_drivers_wait_longer_than_any_signal_cycle_before_giving_up():
    # 포기하기 전에 어느 신호에서든 녹색을 한 번은 본다(J42 주기 182초가 가장 길다)
    assert lane_route.GIVE_UP_S > max(_cycles().values())
