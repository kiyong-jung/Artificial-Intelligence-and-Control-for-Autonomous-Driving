"""A1: 양보형은 버스보다 느려져 길을 터 주고, 비양보형은 버스보다 빨라져 막는다(traffic/yield_logic.py)."""
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.yield_logic import (BLINKER_LEFT, BLOCK_DV, BLOCK_MAX_S, BLOCK_MIN_SPEED, DEFAULT_TYPES,  # noqa: E402
                                 YIELD_DV, YieldManager)


class _V:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return ["bus0", "f"]

    def getSignals(self, v):
        return BLINKER_LEFT

    def getLanePosition(self, v):
        return self.tc.pos[v]

    def getRoadID(self, v):
        return "E"

    def getLaneID(self, v):
        return "E_1"

    def getLaneIndex(self, v):
        return 1 if v == "bus0" else 2

    def getLeftFollowers(self, v):
        return [("f", self.tc.pos["bus0"] - self.tc.pos["f"] - 12.0)]

    def getRightFollowers(self, v):
        return []

    def getParameter(self, v, k):
        return self.tc.params[k]

    def setParameter(self, v, k, val):
        self.tc.params[k] = val

    def getSpeed(self, v):
        return self.tc.speed[v]

    def setSpeed(self, v, s):
        self.tc.set_speed[v] = s

    def getLength(self, v):
        return 12.0 if v == "bus0" else 4.5

    def getTypeID(self, v):
        return "car_krauss"

    def changeLane(self, *a):
        pass

    def openGap(self, *a):
        self.tc.opengap += 1

    def setAcceleration(self, *a):
        self.tc.accel += 1


class _Lane:
    def __init__(self, tc):
        self.tc = tc

    def getLength(self, lane):
        return self.tc.lane_len


class _FakeTC:
    def __init__(self, bus_speed, action):
        self.speed = {"bus0": bus_speed, "f": bus_speed}
        self.pos = {"bus0": 100.0, "f": 80.0}
        self.params = {"driver.type": "yielder" if action == "yield" else "nonyielder", "driver.action": action,
                       "driver.delay": "0.5", "driver.newTau": "2.0"}
        self.set_speed, self.opengap, self.accel = {}, 0, 0
        self.lane_len = 103.0          # 버스가 정지선 3 m 앞: 서 있어도 진입 의사로 본다(_waiting_at_line)
        self.vehicle, self.lane = _V(self), _Lane(self)


def _step(mgr, tc, t0, t1):
    t = t0
    while t < t1 - 1e-9:
        mgr.update(t)
        t = round(t + 0.1, 6)
    return t


def test_yielder_slows_below_bus_speed():
    tc = _FakeTC(8.0, "yield")
    _step(YieldManager(tc, "bus0"), tc, 0.0, 2.0)
    assert tc.set_speed["f"] == pytest.approx(8.0 - YIELD_DV) and tc.opengap == 0


def test_blocker_speeds_above_bus_even_when_bus_stopped():
    tc = _FakeTC(0.0, "block")
    _step(YieldManager(tc, "bus0"), tc, 0.0, 2.0)
    assert tc.set_speed["f"] == pytest.approx(BLOCK_MIN_SPEED) and tc.accel == 0


def test_blocker_targets_bus_plus_dv():
    tc = _FakeTC(8.0, "block")
    _step(YieldManager(tc, "bus0"), tc, 0.0, 2.0)
    assert tc.set_speed["f"] == pytest.approx(8.0 + BLOCK_DV)


def test_blocker_never_slows_below_its_own_speed():
    # 버스가 막 출발해 느릴 때 "버스 + 2 m/s"로 맞추면 9 m/s로 오던 비양보형이 오히려 감속해 간격이 열렸다(A1 시험에서 확인)
    tc = _FakeTC(1.0, "block")
    tc.speed["f"] = 9.0
    _step(YieldManager(tc, "bus0"), tc, 0.0, 2.0)
    assert tc.set_speed["f"] == pytest.approx(9.0)


def test_blocker_released_after_passing_bus():
    tc = _FakeTC(8.0, "block")
    mgr = YieldManager(tc, "bus0")
    _step(mgr, tc, 0.0, 1.0)
    tc.pos["f"] = tc.pos["bus0"] + 4.5 + 6.0          # 뒤끝이 버스 앞끝보다 6 m 앞섰다
    _step(mgr, tc, 1.0, 1.2)
    assert tc.set_speed["f"] == -1 and "f" not in mgr.inter


def test_blocker_released_after_max_time():
    tc = _FakeTC(8.0, "block")
    mgr = YieldManager(tc, "bus0")
    _step(mgr, tc, 0.0, BLOCK_MAX_S + 1.0)
    assert tc.set_speed["f"] == -1


def test_nonyielders_always_block():
    from traffic.yield_logic import YieldManager as YM

    class _D(_V):
        def getTypeID(self, v):
            return "nonyielder"
    for seed in range(20):
        tc = _FakeTC(8.0, "block")
        tc.vehicle = _D(tc)
        YM(tc, "bus0", rng=random.Random(seed)).on_departed(["f"])
        assert tc.params["driver.action"] == "block"
    assert not hasattr(DEFAULT_TYPES["nonyielder"], "p_close_gap")
