"""A1·S1은 관찰만 한다: 배경 차량을 조종하지 않고, 운전자 성향은 투입 때 연속 분포에서 뽑는다."""
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.stop_bypass import StopBypassManager  # noqa: E402
from traffic.yield_logic import BLINKER_LEFT, LC_ATTRS, DriverDist, YieldManager, draw_driver  # noqa: E402

FORBIDDEN = ("setSpeed", "slowDown", "changeLane", "openGap", "setAcceleration", "setLaneChangeMode",
             "setSpeedMode", "moveTo", "moveToXY", "remove", "changeTarget", "setRoute")


class _V:
    """버스 bus0 옆 차로에 후행차 f가 있는 가짜 traci.vehicle. 조종 명령이 오면 실패한다."""

    def __init__(self):
        self.signals = BLINKER_LEFT
        self.pos = {"bus0": 100.0, "f": 80.0}
        self.types = {"bus0": "bus_test", "f": "car_idm", "t": "bgtaxi"}
        self.left_followers = [("f", 5.0)]
        self.left_leaders = []
        self.leader_of_f = None
        self.params, self.speed_factor = {}, {}
        self.stop_state, self.follower = 0, ("f", 8.0)

    def __getattr__(self, name):
        if name in FORBIDDEN:
            raise AssertionError(f"관찰자가 {name}을 불렀다")
        raise AttributeError(name)

    def getIDList(self): return list(self.pos)
    def getTypeID(self, v): return self.types[v]
    def getSignals(self, v): return self.signals
    def getLanePosition(self, v): return self.pos[v]
    def getRoadID(self, v): return "E"
    def getLaneID(self, v): return "E_1"
    def getSpeed(self, v): return 8.0
    def getLeftFollowers(self, v): return list(self.left_followers)
    def getRightFollowers(self, v): return []
    def getLeftLeaders(self, v): return list(self.left_leaders)
    def getRightLeaders(self, v): return []
    def getLeader(self, v, dist): return self.leader_of_f
    def getFollower(self, v, dist): return self.follower
    def getStopState(self, v): return self.stop_state
    def getParameter(self, v, k): return self.params.get((v, k), "")
    def setParameter(self, v, k, val): self.params[(v, k)] = val
    def setSpeedFactor(self, v, f): self.speed_factor[v] = f


class _Lane:
    def getLength(self, lane): return 1000.0


class _Tc:
    def __init__(self):
        self.vehicle, self.lane = _V(), _Lane()


def _run(mgr, t0, t1):
    t = t0
    while t < t1 - 1e-9:
        mgr.update(t)
        t = round(t + 0.1, 6)
    return t


def _advancing(tc, mgr, t0, t1):
    """버스가 1 m/s로 나아가며 방향지시등을 켠 채 갱신한다(진입 의사의 전진 조건)."""
    t = t0
    while t < t1 - 1e-9:
        tc.vehicle.pos["bus0"] += 0.1
        mgr.update(t)
        t = round(t + 0.1, 6)
    return t


def test_driver_draws_are_continuous_and_inside_sumo_ranges():
    rng = random.Random(0)
    draws = [draw_driver(rng) for _ in range(500)]
    for d in draws:
        assert 0.0 <= d["aggressiveness"] <= 1.0
        assert 0.0 <= d["lcCooperative"] <= 1.0 and d["lcCooperative"] == d["lcCooperativeSpeed"]
        assert d["lcSpeedGain"] >= 0.0 and d["lcAssertive"] > 0.0 and -1.0 <= d["lcImpatience"] <= 1.0
        assert 0.8 <= d["speedFactor"] <= 1.2
    assert len({round(d["lcCooperative"], 3) for d in draws}) > 200       # 두 집단이 아니라 연속 분포
    coop_agg = [d["lcCooperative"] for d in draws if d["aggressiveness"] > 0.7]
    coop_calm = [d["lcCooperative"] for d in draws if d["aggressiveness"] < 0.3]
    assert sum(coop_agg) / len(coop_agg) < sum(coop_calm) / len(coop_calm)   # 공격적인 운전자일수록 덜 협조한다
    assert DriverDist().cooperative[1] > 0.0                                   # lcCooperative=-1(협조 차선 변경 끔)을 쓰지 않는다


def test_on_departed_gives_background_cars_their_own_parameters_and_leaves_others():
    tc = _Tc()
    mgr = YieldManager(tc, "bus0", rng=random.Random(1))
    mgr.on_departed(["bus0", "f", "t"])
    assert set(tc.vehicle.speed_factor) == {"f"}
    for attr in LC_ATTRS:
        assert tc.vehicle.params[("f", f"laneChangeModel.{attr}")]
    assert ("bus0", "driver.aggressiveness") not in tc.vehicle.params
    assert ("t", "driver.aggressiveness") not in tc.vehicle.params


def test_observer_logs_bus_first_without_touching_the_follower():
    tc = _Tc()
    mgr = YieldManager(tc, "bus0")
    mgr.on_departed(["f"])
    t = _advancing(tc, mgr, 0.0, 2.5)
    assert list(mgr.inter) == ["f"]                                   # 후행차를 관찰하기 시작했다
    tc.vehicle.leader_of_f = ("bus0", 3.0)                            # 버스가 후행차 앞으로 들어갔다
    _advancing(tc, mgr, t, t + 0.2)
    assert mgr.log[-1]["outcome"] == "bus_first" and mgr.log[-1]["follower"] == "f"
    assert mgr.log[-1]["aggressiveness"] == pytest.approx(float(tc.vehicle.params[("f", "driver.aggressiveness")]))


def test_observer_logs_follower_first_and_ended():
    tc = _Tc()
    mgr = YieldManager(tc, "bus0")
    t = _advancing(tc, mgr, 0.0, 2.5)
    tc.vehicle.left_followers, tc.vehicle.left_leaders = [], [("f", 1.0)]   # 후행차가 버스 옆을 지나 앞섰다
    t = _advancing(tc, mgr, t, t + 0.2)
    assert mgr.log[-1]["outcome"] == "follower_first"
    tc.vehicle.left_followers, tc.vehicle.left_leaders = [("f", 5.0)], []
    t = _advancing(tc, mgr, t, t + 0.3)
    tc.vehicle.signals = 0                                             # 버스가 방향지시등을 껐다
    _run(mgr, t, t + 0.2)
    assert mgr.log[-1]["outcome"] == "ended"


def test_observer_ignores_followers_beyond_range():
    tc = _Tc()
    tc.vehicle.left_followers = [("f", 45.0)]
    mgr = YieldManager(tc, "bus0")
    _advancing(tc, mgr, 0.0, 3.0)
    assert not mgr.inter and not mgr.log


def test_stop_observer_logs_waited_and_overtook_without_control():
    tc = _Tc()
    s = StopBypassManager(tc, "bus0")
    tc.vehicle.stop_state = 1 | 16                                    # 버스가 정류장에 섰다
    tc.vehicle.leader_of_f = ("bus0", 8.0)
    _run(s, 0.0, 1.0)
    tc.vehicle.stop_state = 0                                         # 버스가 떠났다. 후행차는 뒤에 있었다
    _run(s, 1.0, 1.2)
    assert s.log[-1]["outcome"] == "waited"
    tc.vehicle.stop_state, tc.vehicle.leader_of_f = 1 | 16, ("bus0", 8.0)
    t = _run(s, 2.0, 2.5)
    tc.vehicle.leader_of_f = None                                     # 옆 차로로 옮겨 버스 뒤를 떠났다
    _run(s, t, t + 0.2)
    assert s.log[-1]["outcome"] == "overtook" and s.summary() == {"events": 2, "waited": 1, "overtook": 1, "gone": 0}
