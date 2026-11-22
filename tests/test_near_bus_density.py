"""NearBusDensity: 버스 주변 50 m 안에 차가 모자라면 뒤차는 빨리 들어오게, 앞차는 느리게 해서 밀도를 맞춘다. 버스에는 명령하지 않는다."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.common import Ownership  # noqa: E402
from traffic.near_bus_density import NearBusDensity, route_gap  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")
_c = {}


def _net():
    if "net" not in _c:
        _c["net"] = read_net(os.path.join(ROOT, "source", "network.net.xml"))
    return _c["net"]


def _loop():
    if "loop" not in _c:
        _c["loop"] = load_map("hapjeong")["routes"]["bus_loop"]
    return _c["loop"]


def _len(e):
    return _net().getEdge(e).getLength()


class _V:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return ["bus0"] + list(self.tc.cars)

    def getRoadID(self, v):
        return self.tc.bus_road if v == "bus0" else self.tc.cars[v][0]

    def getLanePosition(self, v):
        return self.tc.bus_pos if v == "bus0" else self.tc.cars[v][1]

    def getSpeed(self, v):
        return self.tc.bus_v if v == "bus0" else 10.0

    def getRoute(self, v):
        return tuple(_loop())

    def getRouteIndex(self, v):
        return _loop().index(self.tc.bus_road)

    def getStopState(self, v):
        return self.tc.stop_state

    def getAllowedSpeed(self, v):
        return 13.89

    def setSpeed(self, vid, sp):
        self.tc.speeds.append((vid, sp))


class _Tc:
    def __init__(self, road="R71", pos=20.0):
        self.bus_road, self.bus_pos, self.bus_v, self.stop_state = road, pos, 10.0, 0
        self.cars, self.speeds = {}, []
        self.vehicle = _V(self)


def _nb(tc, owner=None, **kw):
    return NearBusDensity(tc, _net(), "bus0", owner or Ownership(), **kw)


def _run(nb, t0, t1):
    t = t0
    while t < t1:
        nb.update(round(t, 3))
        t += 0.1


def test_route_gap_sign_and_none():
    loop, L = _loop(), _len
    i = loop.index("R71")
    g = lambda road, fpos: route_gap(loop, i, 20.0, L, road, fpos)  # noqa: E731
    assert g("R71", 120.0) == pytest.approx(100.0)
    assert g("R71.135", 10.0) == pytest.approx(L("R71") - 20.0 + 10.0)
    assert g("R69", 10.0) == pytest.approx(-(20.0 + L("R69a") + L("R69") - 10.0))
    assert g(":J_0", 3.0) is None
    assert g("R54", 3.0) is None


def test_follower_behind_range_is_sped_up_until_it_enters_radius():
    tc = _Tc()
    tc.cars = {"a": ("R71", 50.0), "b": ("R69", 40.0)}        # 앞 +30(충분), 뒤 -70(반경 밖)
    ow = Ownership()
    nb = _nb(tc, ow)
    _run(nb, 0.0, 0.6)
    assert ("b", pytest.approx(14.0)) in tc.speeds and ("a", 6.0) not in tc.speeds
    assert ow.owner("b") == "density"
    tc.cars["b"] = ("R69a", 0.0)                                # 뒤 -31: 반경 안으로 들어왔다
    n = len(tc.speeds)
    _run(nb, 1.0, 1.6)
    assert tc.speeds[n:][0] == ("b", -1) and ow.owner("b") is None
    assert nb.summary()["released_in_range"] == 1 and nb.summary()["pulled"] == 1


def test_leader_ahead_is_slowed_when_nothing_ahead_in_radius():
    tc = _Tc()
    tc.cars = {"f": ("R71", 100.0), "b": ("R69a", 5.0)}         # 앞 +80(반경 밖), 뒤 -26(충분)
    nb = _nb(tc)
    _run(nb, 0.0, 0.6)
    assert ("f", pytest.approx(13.89 - 4.0)) in tc.speeds                    # 도로 제한 속도 - 4 m/s(버스 속도 기준이 아니다)
    assert nb.summary()["slowed"] == 1


def test_no_action_when_radius_already_has_enough_cars():
    tc = _Tc()
    tc.cars = {"a": ("R71", 50.0), "b": ("R69a", 5.0), "far": ("R71", 150.0)}
    _run(_nb(tc), 0.0, 3.0)
    assert tc.speeds == []


def test_never_touches_vehicle_owned_by_another_behavior():
    tc = _Tc()
    tc.cars = {"a": ("R71", 50.0), "b": ("R69", 40.0)}
    ow = Ownership()
    ow.claim("b", "A1")
    _run(_nb(tc, ow), 0.0, 3.0)
    assert all(v != "b" for v, _ in tc.speeds)


def test_no_new_action_while_bus_is_in_signal_zone():
    tc2 = _Tc(road="R71.256", pos=45.0)                         # 신호 정지선 앞 금지 구간(52 m 도로의 끝 20 m)
    tc2.cars = {"a": ("R71.256", 50.0), "b": ("R71.206", 5.0)}
    tc2.cars["x"] = ("R71.135", 10.0)
    _run(_nb(tc2), 0.0, 3.0)
    assert tc2.speeds == []


def test_while_bus_is_stopped_followers_are_timed_to_arrive_at_departure_and_leaders_are_left_alone():
    tc = _Tc(pos=20.0)
    tc.stop_state = 1
    tc.bus_v = 0.0
    tc.cars = {"f": ("R71", 100.0), "b": ("R69", 40.0)}        # 앞 +80(모자람), 뒤 -70(모자람)
    ow = Ownership()
    nb = _nb(tc, ow, dwell_s=8.0)
    gap = -route_gap(_loop(), _loop().index("R71"), 20.0, _len, "R69", 40.0)         # 뒤차까지의 노선 거리(약 71 m)
    _run(nb, 0.0, 0.6)
    pulled = [(v, sp) for v, sp in tc.speeds if v == "b"]
    assert pulled and pulled[0][1] == pytest.approx((gap - 40.0) / 8.0)       # 8초 남았을 때: (거리 - 40 m) / 남은 시간
    assert all(v != "f" for v, _ in tc.speeds)                  # 앞차는 정차 중 늦추지 않는다
    _run(nb, 4.0, 4.05)                                         # 4초 지났다: 남은 시간이 줄어 더 빨라진다(같은 거리라면)
    later = [sp for v, sp in tc.speeds if v == "b"][-1]
    assert later == pytest.approx((gap - 40.0) / 4.0, rel=0.05)


def test_slowed_leader_is_released_when_bus_stops():
    tc = _Tc(pos=20.0)
    tc.cars = {"f": ("R71", 100.0), "b": ("R69a", 5.0)}
    ow = Ownership()
    nb = _nb(tc, ow)
    _run(nb, 0.0, 0.6)
    assert ow.owner("f") == "density"
    tc.stop_state, tc.bus_v = 1, 0.0                            # 버스가 정류장에 섰다
    _run(nb, 1.0, 1.6)
    assert ow.owner("f") is None and ("f", -1) in tc.speeds


def test_released_on_timeout_and_when_vehicle_leaves():
    tc = _Tc()
    tc.cars = {"a": ("R71", 50.0), "b": ("R69", 40.0)}
    ow = Ownership()
    nb = _nb(tc, ow, timeout=20.0)
    _run(nb, 0.0, 0.6)
    assert ow.owner("b") == "density"
    _run(nb, 21.0, 21.6)
    assert ("b", -1) in tc.speeds and nb.summary()["timeout"] == 1
    tc.cars = {"a": ("R71", 50.0), "c": ("R69", 40.0)}
    _run(nb, 30.0, 30.6)
    assert ow.owner("c") == "density"
    del tc.cars["c"]
    tc.cars["d"] = ("R71", 60.0)
    _run(nb, 31.0, 31.6)
    assert ow.owner("c") is None


def test_never_commands_the_bus():
    tc = _Tc()
    tc.cars = {"f": ("R71", 100.0), "b": ("R69", 40.0)}
    _run(_nb(tc), 0.0, 5.0)
    assert tc.speeds and all(v != "bus0" for v, _ in tc.speeds)


def test_never_commands_scenario_vehicles():
    # 주입기·정류장 앞 정차·정차 차량·택시는 시나리오가 다루므로 밀도 맞추기가 속도를 건드리지 않는다
    for vid in ("inj_s55", "inj_lane3", "stopahead2", "stopped0", "taxi0"):
        tc = _Tc()
        tc.cars = {"a": ("R71", 50.0), vid: ("R69", 40.0)}
        ow = Ownership()
        _run(_nb(tc, ow), 0.0, 3.0)
        assert all(v != vid for v, _ in tc.speeds) and ow.owner(vid) is None, vid


def test_slowed_leader_target_does_not_depend_on_the_bus_speed():
    # 버스 속도 기준이면 버스가 느려진 앞차를 뒤따라 줄 때 목표가 계속 낮아져 버스가 기어갔다
    targets = []
    for bus_v in (13.0, 8.0, 4.0):
        tc = _Tc()
        tc.bus_v = bus_v
        tc.cars = {"f": ("R71", 100.0), "b": ("R69a", 5.0)}
        _run(_nb(tc), 0.0, 0.6)
        targets.append([sp for v, sp in tc.speeds if v == "f"][0])
    assert targets == [pytest.approx(13.89 - 4.0)] * 3
