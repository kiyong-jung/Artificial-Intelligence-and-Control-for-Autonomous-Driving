"""S5: 끼어들기 차를 골목 끝(정지선)에 세워 두고, 버스가 합류점에 곧 닿을 때 풀어 끼어들게 한다. 버스는 기다리지 않고, 골목 차를 보면 줄인다."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import test_injector_plan as T  # noqa: E402
from traffic import scenario_injector as SI  # noqa: E402
from traffic.presence import ends_at_boundary  # noqa: E402
from traffic.test_bus import _back  # noqa: E402

JUNCTION = "X_dongmak3"                                   # G3 -> R71.135 (신호 없는 합류점)


def _alley():
    return [a for a in T._alleys() if a.junction == JUNCTION]


def _place_bus(tc, back):
    """버스를 합류점 back m 앞에 둔다."""
    a = _alley()[0]
    edge, pos = _back(T._net(), T._route(), T._route().index(a.into), 0.0, back)
    tc.f.bus_road, tc.f.bus_pos = edge, pos
    return a


def _setup(back, speed=8.0):
    tc = T._TC2(bus_road="R71", bus_pos=20.0, bus_speed=speed)
    _place_bus(tc, back)
    inj = T._inj(tc, alleys=_alley())
    inj.update(0.0)
    return tc, inj


def _placed(tc, inj):
    """`_setup`의 update가 이미 차를 골목 끝에 세웠다. 다음 스텝에 정지 명령까지 받게 하고 차 id를 돌려준다."""
    vid = next(a["id"] for a in tc.f.added if a["id"].startswith("inj_s5"))
    tc.f.roads[vid] = _alley()[0].feeder
    inj._s5(0.2, inj._state())
    return vid


def test_cutter_is_placed_at_the_alley_end_when_the_bus_is_90_to_300_m_before_the_merge(monkeypatch):
    tc, inj = _setup(120.0)                                   # update가 버스 앞 150 m에서 차를 세운다
    cut = [a for a in tc.f.added if a["id"].startswith("inj_s5")]
    a = _alley()[0]
    L = T._net().getEdge(a.feeder).getLength()
    assert cut and cut[0]["type"] == "carDist" and cut[0]["route"][0] == a.feeder and cut[0]["departSpeed"] == "0"
    assert float(cut[0]["departPos"]) == pytest.approx(L - SI.S5_HOLD_BACK_M, abs=0.1)
    assert ends_at_boundary(T._net(), cut[0]["route"][-1])
    monkeypatch.setattr(SI, "S5_PLACE_MAX_M", 100.0)             # 120 m는 이제 너무 멀다: 아직 세우지 않는다
    tc2, inj2 = _setup(120.0)
    assert not any(x["id"].startswith("inj_s5") for x in tc2.f.added)


def test_cutter_is_told_to_stop_with_its_front_at_the_merge():
    tc, inj = _setup(120.0)
    vid = _placed(tc, inj)
    a = _alley()[0]
    L = T._net().getEdge(a.feeder).getLength()
    assert tc.f.stops_set and tc.f.stops_set[0][:2] == (vid, a.feeder)
    assert tc.f.stops_set[0][2] == pytest.approx(L - SI.S5_STOP_BEFORE_END_M, abs=0.01) and tc.f.stops_set[0][3] == 0


def test_too_close_bus_marks_s5_late_instead_of_placing_a_cutter():
    tc, inj = _setup(40.0)
    assert not any(x["id"].startswith("inj_s5") for x in tc.f.added)
    assert JUNCTION in inj._lap["s5_done"] and inj.log[-1]["ok"] == "late"


def test_cutter_is_released_to_cut_in_only_when_the_bus_is_2_5_to_5_seconds_away_and_ignores_the_bus():
    for back, released in ((60.0, False), (32.0, True), (14.0, False)):          # 8 m/s: 7.5초, 4초, 1.75초
        tc, inj = _setup(120.0)
        vid = _placed(tc, inj)
        _place_bus(tc, back)
        inj._s5(0.3, inj._state())
        assert (vid in tc.f.resumed) == released, back
        if released:
            assert tc.f.params[(vid, "junctionModel.ignoreIDs")] == "bus0"
        else:
            assert (vid, "junctionModel.ignoreIDs") not in tc.f.params


def test_a_too_fast_bus_passes_first_and_then_the_cutter_comes_out_without_ignoring():
    tc, inj = _setup(120.0)
    vid = _placed(tc, inj)
    _place_bus(tc, 14.0)                                       # 1.75초: 너무 빠르다
    inj._s5(0.3, inj._state())
    assert tc.f.resumed == []
    tc.f.bus_road, tc.f.bus_pos = _alley()[0].into, 20.0       # 버스가 합류점을 지났다
    inj._s5(0.4, inj._state())
    assert tc.f.resumed == [vid] and (vid, "junctionModel.ignoreIDs") not in tc.f.params


def test_release_waits_while_another_vehicle_is_between_the_bus_and_the_merge_or_in_the_entry_lane():
    a = _alley()[0]
    lanes = {c.getToLane().getIndex() for c in T._net().getEdge(a.feeder).getConnections(T._net().getEdge(a.into))}
    entry = min(lanes)
    cases = [("between", lambda tc: (tc.f.edge_vehicles.update({tc.f.bus_road: ["o"]}), tc.f.lane_pos.update(o=tc.f.bus_pos + 10.0))),
             ("entry_lane_ahead", lambda tc: (tc.f.edge_vehicles.update({a.into: ["o"]}), tc.f.lane_pos.update(o=10.0), tc.f.lanes.update(o=entry))),
             ("in_junction", lambda tc: (tc.f.roads.update(o=":" + JUNCTION + "_0_0"), tc.f.cars.append("o")))]
    for name, setup in cases:
        tc, inj = _setup(120.0)
        vid = _placed(tc, inj)
        _place_bus(tc, 32.0)
        setup(tc)
        inj._s5(0.3, inj._state())
        assert tc.f.resumed == [], name
    other = [i for i in range(T._net().getEdge(a.into).getLaneNumber()) if i not in lanes]
    if other:                                                  # 다른 차로의 차나 버스 뒤의 차는 막지 않는다
        tc, inj = _setup(120.0)
        vid = _placed(tc, inj)
        _place_bus(tc, 32.0)
        tc.f.edge_vehicles.update({a.into: ["o"], tc.f.bus_road: ["b"]})
        tc.f.lane_pos.update(o=10.0, b=max(0.0, tc.f.bus_pos - 20.0))
        tc.f.lanes.update(o=other[0])
        inj._s5(0.3, inj._state())
        assert tc.f.resumed == [vid]


def test_failed_resume_is_retried_next_step():
    tc, inj = _setup(120.0)
    vid = _placed(tc, inj)
    _place_bus(tc, 32.0)
    tc.f.resume_fail = True                                    # 아직 정지선에 서지 않았다
    inj._s5(0.3, inj._state())
    assert tc.f.resumed == []
    tc.f.resume_fail = False
    inj._s5(0.4, inj._state())
    assert tc.f.resumed == [vid]


def test_bus_slows_to_half_the_limit_when_it_sees_the_waiting_cutter_and_is_free_afterwards():
    tc, inj = _setup(120.0)
    assert inj.bus_speed_limit() is None                       # 대기 차가 없다
    vid = _placed(tc, inj)
    _place_bus(tc, 60.0)
    assert inj.bus_speed_limit() is None                       # 보이는 거리(45 m) 밖
    _place_bus(tc, 40.0)
    assert inj.bus_speed_limit() == pytest.approx(max(SI.S5_SEE_MIN_SPEED, SI.S5_SEE_FRAC * 13.89))
    tc.f.roads[vid] = _alley()[0].into                         # 차가 골목을 벗어나 합류했다
    inj._s5(0.5, inj._state())
    assert inj.bus_speed_limit() is None


def test_cutter_is_not_placed_while_the_bus_waits_at_the_signal_before_the_merge():
    # X_jandari2(G23 -> -R96.119)는 J74 신호 뒤에서 합류한다. 버스가 적색으로 서 있을 때 세우면 차가 일찍 나가 버스와 만나지 못한다
    alley = [a for a in T._alleys() if a.junction == "X_jandari2"]
    length = T._net().getEdge("R113").getLength()
    tc = T._TC2(bus_road="R113", bus_pos=length - 3.0, bus_speed=0.0)
    inj = T._inj(tc, alleys=alley)
    inj.update(0.0)
    assert not any(a["id"].startswith("inj_s5") for a in tc.f.added) and "X_jandari2" not in inj._lap["s5_done"]
    tc.f.bus_road, tc.f.bus_pos, tc.f.bus_speed = "-R96", 5.0, 8.0                   # 신호를 지났다(합류점까지 약 99 m)
    inj._s5(5.0, inj._state())
    assert any(a["id"].startswith("inj_s5") for a in tc.f.added)


def test_cutter_can_be_placed_while_the_bus_dwells_at_a_stop_just_before_the_merge():
    # X_dongmak6(G12 -> R71.256)는 성산중학교(s2) 바로 뒤에서 합류한다. 버스가 정류장에 서 있으면 합류점이 가까워도 늦음이 아니다
    a = [x for x in T._alleys() if x.junction == "X_dongmak6"]
    tc = T._TC2(bus_road="R71.206", bus_pos=38.0, bus_speed=0.0)
    tc.f.stop_state = 1 | 16                                   # 정류장에 서 있다
    inj = T._inj(tc, alleys=a)
    inj.update(0.0)                                            # 정차 시작 시각을 기록하고 S5를 본다
    assert any(x["id"].startswith("inj_s5") for x in tc.f.added) and "X_dongmak6" not in inj._lap["s5_done"]


def test_no_spawn_within_radius_of_bus():
    from sumolib.geomhelper import positionAtShapeOffset
    from traffic.presence import PRESENCE_RADIUS_M
    g3 = T._net().getEdge("G3")
    xy = positionAtShapeOffset(g3.getShape(), g3.getLength() - SI.S5_HOLD_BACK_M)
    tc = T._TC2(bus_road="R71", bus_pos=20.0, bus_xy=(xy[0] + 0.5 * PRESENCE_RADIUS_M, xy[1]))
    _place_bus(tc, 150.0)
    inj = T._inj(tc, alleys=_alley())
    inj.update(0.0)
    assert not any(a["id"].startswith("inj_s5") for a in tc.f.added)
