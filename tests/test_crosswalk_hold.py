"""버스가 신호 교차로 가까이 있을 때 횡단보도로 가는 보행자는 교차로 대기 구역에서 3 m 떨어져 기다린다."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from maps import load_map  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.crosswalk_hold import CrosswalkHold  # noqa: E402

_cache = {}


def _net():
    if "net" not in _cache:
        _cache["net"] = read_net(load_map("hapjeong")["net_path"])
    return _cache["net"]


class _Person:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return list(self.tc.persons)

    def getRoadID(self, p):
        return self.tc.persons[p]["road"]

    def getNextEdge(self, p):
        return self.tc.persons[p]["next"]

    def getPosition(self, p):
        return self.tc.persons[p]["xy"]

    def setSpeed(self, p, v):
        self.tc.speeds.append((p, v))


class _Lane:
    def getShape(self, lane):
        return [(0.0, 0.0), (0.0, 4.0)]                # 대기 구역 모양: x=0, y 0~4


class _Veh:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return ["bus0"] if self.tc.bus_xy is not None else []

    def getPosition(self, v):
        return self.tc.bus_xy


class _TC:
    def __init__(self):
        self.persons, self.speeds, self.bus_xy = {}, [], None
        self.person, self.lane, self.vehicle = _Person(self), _Lane(), _Veh(self)


def _mk(tc):
    # 교차로 중심은 실제 J82 좌표, 대기 구역 모양은 시험용(0, 0)~(0, 4). 보행자 위치 비교는 구역 모양으로 한다
    return CrosswalkHold(tc, _net(), "bus0", hold_m=3.0, bus_radius=40.0, max_hold_s=30.0)


def _near_j82():
    x, y = _net().getNode("J82").getCoord()
    return (x + 10.0, y)


def test_holds_a_walker_within_3m_of_the_waiting_area_while_the_bus_is_near():
    tc = _TC()
    tc.bus_xy = _near_j82()
    tc.persons["p"] = {"road": "R112", "next": ":J82_w1", "xy": (2.0, 2.0)}     # 구역에서 2 m
    _mk(tc).update(0.0)
    assert tc.speeds == [("p", 0.0)]


def test_does_not_hold_a_walker_farther_than_3m_or_when_the_bus_is_far_or_when_unsignalized():
    tc = _TC()
    tc.bus_xy = _near_j82()
    tc.persons["far"] = {"road": "R112", "next": ":J82_w1", "xy": (6.0, 2.0)}     # 6 m: 아직 걷는다
    tc.persons["inside"] = {"road": ":J82_w1", "next": ":J82_c0", "xy": (0.0, 1.0)}   # 이미 교차로 안
    tc.persons["s6"] = {"road": "R112.112", "next": ":S6_wausanro_w0", "xy": (1.0, 1.0)}   # 신호 없는 횡단보도
    _mk(tc).update(0.0)
    assert tc.speeds == []
    tc2 = _TC()
    tc2.bus_xy = (_near_j82()[0] + 500.0, _near_j82()[1])                          # 버스가 멀다
    tc2.persons["p"] = {"road": "R112", "next": ":J82_w1", "xy": (1.0, 1.0)}
    _mk(tc2).update(0.0)
    assert tc2.speeds == []


def test_releases_when_the_bus_leaves_and_does_not_hold_the_same_walker_again():
    tc = _TC()
    tc.bus_xy = _near_j82()
    tc.persons["p"] = {"road": "R112", "next": ":J82_w1", "xy": (1.0, 1.0)}
    hold = _mk(tc)
    hold.update(0.0)
    tc.bus_xy = (_near_j82()[0] + 500.0, _near_j82()[1])
    hold.update(0.1)
    assert tc.speeds == [("p", 0.0), ("p", -1)]
    tc.bus_xy = _near_j82()
    hold.update(0.2)
    assert tc.speeds == [("p", 0.0), ("p", -1)]                                    # 풀려 난 보행자는 다시 세우지 않는다


def test_releases_after_the_max_hold_time():
    tc = _TC()
    tc.bus_xy = _near_j82()
    tc.persons["p"] = {"road": "R112", "next": ":J82_w1", "xy": (1.0, 1.0)}
    hold = _mk(tc)
    hold.update(0.0)
    hold.update(31.0)
    assert tc.speeds[-1] == ("p", -1)
