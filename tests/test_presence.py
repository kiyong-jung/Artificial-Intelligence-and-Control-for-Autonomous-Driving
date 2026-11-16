"""시험용 버스 반경 안에서 물체가 순간이동으로 생기거나 사라지지 않게 하는 규칙(traffic/presence.py)."""
import importlib
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import read_net  # noqa: E402
from traffic.presence import (PRESENCE_RADIUS_M, PresenceMonitor, VehicleArrivalGuard, boundary_nodes,  # noqa: E402
                              ends_at_boundary, far_from_bus, nearest_boundary_exit, starts_at_boundary)

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

    def changeTarget(self, vid, edge):
        self.tc.targets[vid] = edge

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
        self.targets = {}
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


def test_far_from_bus_uses_straight_distance():
    tc = _FakeTC(bus_xy=(0.0, 0.0))
    assert not far_from_bus(tc, "bus0", (0.8 * R, 0.0))
    assert far_from_bus(tc, "bus0", (0.6 * R, 0.9 * R))     # 1.08 R
    assert far_from_bus(_FakeTC(bus_xy=None), "bus0", (0.0, 0.0))


def test_nearest_boundary_exit_starts_at_edge_and_ends_at_boundary():
    path = nearest_boundary_exit(_net(), "R71.206")
    assert path[0] == "R71.206" and ends_at_boundary(_net(), path[-1])


def test_arrival_guard_extends_route_ending_inside_map():
    tc = _FakeTC(vehicles={"v1": ("R71.206", 10.0, (0.5 * R, 0.0))}, routes={"v1": ["R71.135", "R71.206"]})
    VehicleArrivalGuard(tc, _net(), "bus0").update(0.0)
    assert ends_at_boundary(_net(), tc.targets["v1"])


def test_arrival_guard_leaves_far_or_boundary_bound_vehicles():
    tc = _FakeTC(vehicles={"far": ("R71.206", 10.0, (5 * R, 0.0)), "edge": ("R48", 10.0, (0.5 * R, 0.0))},
                 routes={"far": ["R71.135", "R71.206"], "edge": ["R44", "R47", "R48"]})
    VehicleArrivalGuard(tc, _net(), "bus0").update(0.0)
    assert tc.targets == {}


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


def test_radius_has_single_definition():
    for name in ("traffic.person_spawner", "traffic.scenario_injector", "traffic.stop_ahead"):
        try:
            mod = importlib.import_module(name)
        except ImportError:
            continue                      # 아직 없는 모듈(이후 태스크에서 생긴다)
        assert getattr(mod, "PRESENCE_RADIUS_M", PRESENCE_RADIUS_M) is PRESENCE_RADIUS_M
