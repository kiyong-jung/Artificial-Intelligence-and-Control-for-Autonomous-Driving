"""시험용 버스 반경 안 출현·소멸 지표와 지도 경계 도우미(traffic/presence.py)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import read_net  # noqa: E402
from traffic.presence import (PRESENCE_RADIUS_M, PresenceMonitor, boundary_nodes,  # noqa: E402
                              ends_at_boundary, nearest_boundary_exit, starts_at_boundary)

NET = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong", "source", "network.net.xml")
R = PRESENCE_RADIUS_M
_cache = {}


def _net():
    if "net" not in _cache:
        _cache["net"] = read_net(NET)
    return _cache["net"]


class _Vehicle:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return list(self.tc.vehicles) + (["bus0"] if self.tc.bus_xy is not None else [])

    def getPosition(self, vid):
        return self.tc.bus_xy if vid == "bus0" else self.tc.vehicles[vid][2]

    def getRoadID(self, vid):
        return self.tc.vehicles[vid][0]

    def getRoute(self, vid):
        return self.tc.routes[vid]

    def getRouteIndex(self, vid):
        return self.tc.routes[vid].index(self.tc.vehicles[vid][0])

    def subscribeContext(self, *a, **k):
        self.tc.subscribed = True

    def getContextSubscriptionResults(self, vid):
        return {v: {"pos": x[2], "road": x[0]} for v, x in self.tc.vehicles.items()}


class _Person:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return list(self.tc.persons)

    def getPosition(self, pid):
        return self.tc.persons[pid][1]

    def getRoadID(self, pid):
        return self.tc.persons[pid][0]


class _Sim:
    def __init__(self, tc):
        self.tc = tc

    def getDepartedIDList(self):
        return list(self.tc.departed)

    def getDepartedPersonIDList(self):
        return list(self.tc.departed_p)

    def getArrivedIDList(self):
        return list(self.tc.arrived)

    def getArrivedPersonIDList(self):
        return list(self.tc.arrived_p)

    def getStartingTeleportIDList(self):
        return list(self.tc.teleports)


class _FakeTC:
    def __init__(self, bus_xy=(0.0, 0.0), vehicles=None, routes=None, persons=None):
        self.bus_xy = bus_xy
        self.vehicles = dict(vehicles or {})       # id -> (road, pos, (x, y))
        self.routes = dict(routes or {})
        self.persons = dict(persons or {})         # id -> (road, (x, y))
        self.departed, self.departed_p, self.arrived, self.arrived_p, self.teleports = [], [], [], [], []
        self.vehicle, self.person, self.simulation = _Vehicle(self), _Person(self), _Sim(self)


def test_boundary_nodes_are_map_cuts_not_junctions():
    net = _net()
    b = boundary_nodes(net)
    assert net.getEdge("R48").getToNode().getID() in b      # 양화로 남서 끝
    assert net.getEdge("R124").getToNode().getID() in b     # 양화로 북동 끝
    assert "J1" not in b and "X_dongmak3" not in b
    assert ends_at_boundary(net, "R48") and starts_at_boundary(net, "R51")
    assert not ends_at_boundary(net, "R71.206")


def test_nearest_boundary_exit_starts_at_edge_and_ends_at_boundary():
    path = nearest_boundary_exit(_net(), "R71.206")
    assert path[0] == "R71.206" and ends_at_boundary(_net(), path[-1])


def test_monitor_counts_vanish_inside_radius_but_not_at_boundary():
    tc = _FakeTC(vehicles={"a": ("R71.206", 10.0, (0.3 * R, 0.0)), "b": ("R48", 200.0, (0.3 * R, 1.0))})
    mon = PresenceMonitor(tc, _net(), "bus0")
    mon.update(0.0)
    del tc.vehicles["a"], tc.vehicles["b"]
    tc.arrived = ["a", "b"]
    mon.update(0.1)
    s = mon.summary()
    assert s["vanish"] == 1 and s["examples"][0][1] == "a"


def test_monitor_counts_appear_and_teleport_inside_radius():
    tc = _FakeTC(vehicles={"c": ("R71.206", 10.0, (0.5 * R, 0.0)), "d": ("R71.206", 10.0, (3 * R, 0.0))},
                 persons={"p": ("R71.206", (0.2 * R, 0.0))})
    tc.departed, tc.departed_p = ["c", "d"], ["p"]
    mon = PresenceMonitor(tc, _net(), "bus0")
    mon.update(0.0)
    tc.departed, tc.departed_p, tc.teleports = [], [], ["c"]
    mon.update(0.1)
    s = mon.summary()
    assert s["appear"] == 2 and s["teleport"] == 1 and s["radius"] == R


def test_departure_inside_a_hidden_alley_is_not_counted():
    # 골목 안은 건물에 가려 학습 차량이 못 본다(입구에 닿아야 보이고, 차는 골목 끝에서 출발한다)
    tc = _FakeTC(vehicles={"f_G31_to_R126.0": ("G31", 5.0, (0.4 * R, 0.0)), "f_G23_to_R1.0": ("G23", 5.0, (0.4 * R, 0.0))},
                 routes={"f_G31_to_R126.0": ["G31"], "f_G23_to_R1.0": ["G23"]})
    tc.departed = ["f_G31_to_R126.0", "f_G23_to_R1.0"]
    mon = PresenceMonitor(tc, _net(), "bus0",)
    mon.update(0.0)
    assert mon.summary()["appear"] == 0


def test_vanish_inside_a_hidden_alley_is_not_counted():
    tc = _FakeTC(vehicles={"a": ("G31", 10.0, (0.3 * R, 0.0))})
    mon = PresenceMonitor(tc, _net(), "bus0")
    mon.update(0.0)
    del tc.vehicles["a"]
    tc.arrived = ["a"]
    mon.update(0.1)
    assert mon.summary()["vanish"] == 0
