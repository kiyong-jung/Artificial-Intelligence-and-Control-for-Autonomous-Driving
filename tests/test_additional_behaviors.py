"""추가 행동(R1 일반화, S1 정차 버스 뒤 차량, S4 정차 차량, S5 골목 자동 탐지, 소유권)의 빠른 검증."""
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.common import AGGRESSION, LaneLock, Ownership  # noqa: E402
from traffic.cutin import StoppedSpec, cutin_bins_once, stopped_vehicle_xml  # noqa: E402
from traffic.scenarios_a2 import build_a2_net, build_s4_kind_net, run_s4  # noqa: E402
from traffic.scenarios_s1 import build_s1_net, run_s1_episode  # noqa: E402
from traffic.yield_logic import DEFAULT_TYPES, Trigger, YieldManager, Zone  # noqa: E402


class _Veh:
    """YieldManager의 진입 의사·대상 선택만 시험하는 가짜 traci.vehicle."""

    def __init__(self, signals, edge, pos, left=(), right=()):
        self.signals, self.edge, self.pos, self.left, self.right = signals, edge, pos, left, right

    def getSignals(self, v): return self.signals
    def getLanePosition(self, v): return self.pos
    def getRoadID(self, v): return self.edge
    def getLeftFollowers(self, v): return list(self.left)
    def getRightFollowers(self, v): return list(self.right)


class _Tc:
    def __init__(self, veh):
        self.vehicle = veh


def _mgr(veh, **kw):
    return YieldManager(_Tc(veh), "bus0", types=DEFAULT_TYPES, **kw)


def _advance(m, veh, t0=0.0):
    """직전 2초에 앞으로 나아간 것으로 만들어 진입 의사의 세 번째 조건을 채운다."""
    start = veh.pos
    for i in range(21):
        veh.pos = start + i * 0.1
        side = m._intent(t0 + i * 0.1)
    return side


def test_trigger_generalizes_to_zone_and_side():
    left, right = 2, 1                               # BLINKER_LEFT, BLINKER_RIGHT
    s2 = Trigger(+1, [Zone("A", 100, 200)])
    s3 = Trigger(-1, [Zone("B", 50, 150)])
    assert s2.matches(+1, "A", 150) and not s2.matches(+1, "A", 250) and not s2.matches(-1, "A", 150)
    assert s3.matches(-1, "B", 60) and not s3.matches(+1, "B", 60) and not s3.matches(-1, "A", 60)
    veh = _Veh(left, "A", 140.0)
    assert _advance(_mgr(veh, triggers=[s2, s3]), veh) == +1          # S2: 좌측 방향지시등, 구간 안
    veh = _Veh(right, "B", 60.0)
    assert _advance(_mgr(veh, triggers=[s2, s3]), veh) == -1          # S3: 우측 방향지시등, 정류장 접근 구간
    veh = _Veh(right, "A", 140.0)
    assert _advance(_mgr(veh, triggers=[s2, s3]), veh) == 0           # 방향이 다르면 의사 없음
    veh = _Veh(left, "A", 140.0)
    assert _advance(_mgr(veh, stop_exit_pos=("A", 150.0), exit_max_dist=30.0), veh) == +1   # 예전 방식도 그대로
    veh = _Veh(left, "A", 140.0)
    assert _advance(_mgr(veh), veh) == +1                              # 구간 조건 없음: 방향지시등만


def test_target_is_only_the_nearest_follower_in_range():
    veh = _Veh(2, "A", 0.0, left=[("far", 40.0), ("near", 12.0), ("mid", 25.0), ("out", 80.0), ("behind", -3.0)])
    assert _mgr(veh, follower_range=50.0)._target_follower(+1) == "near"
    veh = _Veh(1, "A", 0.0, right=[("r1", 9.0)], left=[("l1", 5.0)])
    assert _mgr(veh, follower_range=50.0)._target_follower(-1) == "r1"  # 우측 차로로 들어갈 때는 우측 차로의 후행차
    assert _mgr(veh)._target_follower(-1) is None                       # 기본은 3 m 안일 때만(A1_TRIGGER_GAP_M)
    assert _mgr(_Veh(2, "A", 0.0, left=[("x", 99.0)]), follower_range=50.0)._target_follower(+1) is None


def test_ownership_is_exclusive():
    o = Ownership()
    assert o.claim("v", "A1") and o.claim("v", "A1")
    assert not o.claim("v", "S1") and o.owner("v") == "A1"
    o.release("v", "S1")
    assert o.owner("v") == "A1"                                        # 남이 놓을 수 없다
    o.release("v", "A1")
    assert o.owner("v") is None and o.claim("v", "S1")


def test_aggressiveness_maps_to_scenario_probabilities():
    a = AGGRESSION
    assert a.p_yield(0.0) == 1.0 and a.p_yield(1.0) == 0.0
    assert a.p_overtake(0.0) < a.p_overtake(0.5) < a.p_overtake(1.0)
    assert a.p_ignore(0.0) == pytest.approx(0.2) and a.p_ignore(1.0) == pytest.approx(1.0)
    assert a.aggressiveness_from_ignore(a.p_ignore(0.3)) == pytest.approx(0.3)


def test_cutin_bins_once_realize_p_ignore_by_weight():
    bins = cutin_bins_once(6)
    assert sum(b[3] for b in bins) == pytest.approx(1.0, abs=1e-3)
    assert {b[0] for b in bins} <= {0.0, 1.0}                          # 무시 확률은 0 또는 1만(진입 판정은 차량당 한 번)
    mean_p = sum(b[0] * b[3] for b in bins)
    assert mean_p == pytest.approx(sum(AGGRESSION.p_ignore(i / 5) for i in range(6)) / 6, abs=1e-3)


def test_stopped_vehicle_kinds_and_area_length():
    for kind, length in (("taxi", 4.5), ("bus", 12.0), ("car", 4.5)):
        vts, vehs, pa = stopped_vehicle_xml(StoppedSpec(kind, "E0", 1, 100.0, 5.0, 30.0, 1.0, "s0"))
        assert len(vts) == 1 and 'duration="30.0"' in vehs[0] and 'departLane="1"' in vehs[0]
        assert 'lane="E0_1"' in pa and 'onRoad="true"' in pa
    with pytest.raises(ValueError):
        stopped_vehicle_xml(StoppedSpec("tram", "E0", 0, 0, 0))
    bus_pa = stopped_vehicle_xml(StoppedSpec("bus", "E0", 0, 100.0, 0))[2]
    assert 'endPos="116.0"' in bus_pa                                  # 버스는 정차 구역이 더 길다


@pytest.mark.parametrize("kind", ["taxi", "bus", "car"])
def test_s4_any_stopped_vehicle_blocks_bus_then_leaves(kind):
    rows = run_s4(build_s4_kind_net(kind, stop_duration=30), "stopped0")
    parked = [r for r in rows if r[5]]
    assert len(parked) * 0.1 >= 29.0                                   # 정차 시간만큼 서 있다
    assert not any(r[3] > r[1] for r in parked)                        # 서 있는 동안 버스가 추월하지 못한다
    assert any(r[4] < 0.1 for r in parked)                             # 버스도 뒤에서 선다
    assert rows[-1][4] > 5.0                                           # 출발한 뒤 통과한다


def test_merge_alley_is_found_wherever_it_joins():
    from traffic.alleys import find_merge_alleys, flow_merges, read_net
    for x, side in ((150.0, -1), (450.0, 1)):
        cfg = build_a2_net(single_bin=(1.0, 0.5, 0.5), alley_x=x, alley_side=side, name=f"t_a2_{int(x)}_{side}")
        net = read_net(os.path.join(os.path.dirname(cfg), "net.net.xml"))
        found = find_merge_alleys(net, ["main_in", "main_out"])
        assert [(a.junction, a.feeder, a.minor, a.regulated) for a in found] == [("J", "alley", True, True)]
        assert flow_merges(net, {"from": "alley", "to": "main_out"}, found)
        assert not flow_merges(net, {"from": "main_in", "to": "main_out"}, found)


def test_s1_wait_is_held_by_lane_lock_and_overtake_follows_decision():
    cfg = build_s1_net()
    for model in ("krauss", "idm"):
        locked = [run_s1_episode(cfg, model, s, p_overtake=0.0)["event"] for s in range(2)]
        free = [run_s1_episode(cfg, model, s, p_overtake=0.0, lock_wait=False)["event"] for s in range(2)]
        over = [run_s1_episode(cfg, model, s, p_overtake=1.0)["event"] for s in range(2)]
        assert all(e["outcome"] == "waited" for e in locked)           # 고정하면 기다린다
        assert all(e["outcome"] == "leaked" for e in free)             # 고정하지 않으면 SUMO가 스스로 추월시킨다(누출)
        assert all(e["outcome"] == "overtook" for e in over)           # 추월 판정은 실제로 옆 차로로 옮겨 간다


def test_s1_and_a1_never_own_the_same_vehicle():
    r = run_s1_episode(build_s1_net(), "krauss", 0, p_overtake=0.0)
    assert r["owner_left"] is None                                     # 끝나면 소유권이 풀린다
    own = Ownership()
    assert own.claim("f0", "A1") and not own.claim("f0", "S1")


def test_lane_lock_release_clears_state():
    class V:
        calls = []

        def changeLane(self, v, i, d): self.calls.append((v, i, d))
        def getLaneIndex(self, v): return 1

    class T:
        vehicle = V()
    lk = LaneLock(T())
    lk.hold("v", 0.0)
    lk.hold("v", 0.2)                                                  # 갱신 주기 안에서는 다시 내지 않는다
    assert T.vehicle.calls == [("v", 1, 1.0)] and lk.held("v")
    lk.release("v")
    assert T.vehicle.calls[-1] == ("v", 1, 0.0) and not lk.held("v")
