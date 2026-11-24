"""차량은 지금 있는 차로에서 갈 수 있는 쪽으로 진로를 바꾼다(traffic/lane_route.py)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import read_net  # noqa: E402
from traffic.lane_route import LaneRouteAdapter, reachable_next  # noqa: E402

NET = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong", "source", "network.net.xml")
_cache = {}


def _net():
    if "net" not in _cache:
        _cache["net"] = read_net(NET)
    return _cache["net"]


def test_reachable_next_lists_only_the_roads_this_lane_connects_to():
    # J42 R94: 차로 1은 R53(우회전)·R92(직진)·R54a(좌회전), 차로 2는 R54a(좌회전)뿐이다
    assert reachable_next(_net(), "R94", 1) == {"R92"}      # 3.8.0: 우회전(R53) 없음, 3.22.0: 차로 1 좌회전 없음
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

    def getRoute(self, v):
        return self.tc.cars[v]["route"]

    def getRouteIndex(self, v):
        return self.tc.cars[v]["route"].index(self.tc.cars[v]["road"])

    def setRoute(self, v, edges):
        self.tc.cars[v]["route"] = list(edges)
        self.tc.set_routes.append((v, list(edges)))

    def getTypeID(self, v):
        return self.tc.cars[v].get("type", "car_krauss")


class _Lane:
    def getLength(self, lane):
        return _net().getEdge(lane.rsplit("_", 1)[0]).getLength()


class _TC:
    def __init__(self, cars):
        self.cars, self.set_routes = cars, []
        self.vehicle, self.lane = _V(self), _Lane()


def _car(lane, pos, route):
    return {"road": "R94", "lane": lane, "pos": pos, "route": route}


def test_right_turner_in_left_only_lane_turns_left_instead():
    # 우회전(R53)으로 갈 차가 어쩔 수 없이 좌회전 차로(차로 2)에 있다: 그 차로가 가는 R54a로 간다
    tc = _TC({"v": _car(2, 70.0, ["R94", "R53", "R53b"])})
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    assert tc.set_routes and tc.set_routes[0][1][:2] == ["R94", "R54a"]
    # 목적지 R53b로는 크게 돌아야 해서 포기하고 가까운 지도 경계로 나간다
    assert _net().getEdge(tc.set_routes[0][1][-1]).getToNode().getID() in __import__("traffic.presence", fromlist=["x"]).boundary_nodes(_net())


def test_straight_car_in_left_only_lane_leaves_by_the_nearest_edge_not_a_big_loop():
    # R92(직진)로 갈 차가 좌회전 전용 차로에 있다. 목적지를 찾아 1 km를 돌아가지 않고 가까운 지도 경계로 나간다
    tc = _TC({"v": _car(2, 70.0, ["R94", "R92"])})
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    new = tc.set_routes[0][1]
    assert new[:2] == ["R94", "R54a"] and "R94" not in new[1:]
    assert sum(_net().getEdge(e).getLength() for e in new[1:]) < 800.0


def test_new_route_starts_at_the_current_road_not_at_the_past_roads():
    # SUMO setRoute는 지금 있는 도로에서 시작하는 경로를 받는다. 지나온 도로까지 넣으면 이어지지 않아 실패했다(실제 실행에서 확인)
    car = {"road": "R94", "lane": 2, "pos": 70.0, "route": ["-R96.119", "-R96.201", "R94", "R53", "R53b"]}
    tc = _TC({"v": car})
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    assert tc.set_routes and tc.set_routes[0][1][0] == "R94" and tc.set_routes[0][1][1] == "R54a"


def test_a_failed_route_does_not_stop_the_other_cars():
    tc = _TC({"v": _car(2, 70.0, ["R94", "R53", "R53b"]), "w": _car(2, 75.0, ["R94", "R53", "R53b"])})
    real = tc.vehicle.setRoute
    calls = []

    def flaky(v, edges):
        calls.append(v)
        if v == "v":
            raise RuntimeError("no connection")
        real(v, edges)
    tc.vehicle.setRoute = flaky
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    assert "w" in [r[0] for r in tc.set_routes]


def test_vehicle_in_a_lane_that_connects_keeps_its_route():
    tc = _TC({"a": _car(1, 70.0, ["R94", "R92"]), "b": _car(2, 70.0, ["R94", "R54a", "R54b"])})
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    assert tc.set_routes == []


def test_right_turn_was_removed_so_lane1_cars_for_r53_go_straight_or_left():
    # 3.8.0: J42 R94에는 우회전(R53)이 없다. 우회전하려던 차는 갈 수 있는 도로로 간다
    tc = _TC({"v": _car(1, 70.0, ["R94", "R53", "R53b"])})
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    assert tc.set_routes and tc.set_routes[0][1][1] in ("R92", "R54a")


def test_far_from_the_stop_line_it_is_left_alone():
    tc = _TC({"v": _car(2, 5.0, ["R94", "R53", "R53b"])})        # 정지선까지 83 m: 아직 차로를 바꿀 수 있다
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    assert tc.set_routes == []


def test_test_bus_is_never_touched():
    tc = _TC({"bus0": _car(2, 80.0, ["R94", "R53", "R53b"])})
    LaneRouteAdapter(tc, _net(), bus_id="bus0").update(0.0)
    assert tc.set_routes == []
