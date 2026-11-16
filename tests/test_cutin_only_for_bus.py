"""끼어들기 차는 시험용 버스만 무시하고 배경 차와는 상호작용(앞차 무시)하지 않는다."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import test_injector_plan as T  # noqa: E402


def test_cutter_ignores_only_the_test_bus():
    tc = T._TC2(bus_road="R71", bus_pos=T._near_merge())
    inj = T._inj(tc, alleys=[a for a in T._alleys() if a.junction == "X_dongmak3"])
    inj.update(0.0)
    inj._s5(0.1, inj._state())
    cut = [a for a in tc.f.added if a["id"].startswith("inj_s5")]
    assert cut
    assert cut[0]["type"] == "carDist"                             # 앞차를 모두 무시하는 vType(cutin_*)이 아니다
    vid = cut[0]["id"]
    tc.f.bus_pos = T._near_merge() - 60.0                          # 버스가 아직 멀다: 차가 먼저 합류점에 닿는다
    tc.f.roads[vid], tc.f.lane_pos[vid] = "G3", 5.0
    inj.update(0.2)                                                # 나타난 다음 스텝에 무시 대상을 지정한다
    assert tc.f.params[(vid, "junctionModel.ignoreIDs")] == "bus0"


def test_cutter_yields_when_the_bus_reaches_the_merge_first():
    # 끼어들기 차가 시험용 버스를 항상 무시하게 했더니 버스가 먼저 합류점을 지나는데도 비켜 주지 않고 부딪혔다(busy 측정 12 샘플).
    # 차가 먼저 닿을 때만 무시하고, 버스가 먼저 닿으면 보통 차처럼 양보한다
    tc = T._TC2(bus_road="R71", bus_pos=T._near_merge())
    inj = T._inj(tc, alleys=[a for a in T._alleys() if a.junction == "X_dongmak3"])
    inj.update(0.0)
    inj._s5(0.1, inj._state())
    vid = next(a["id"] for a in tc.f.added if a["id"].startswith("inj_s5"))
    tc.f.roads[vid], tc.f.lane_pos[vid] = "G3", 1.0                # 차는 아직 골목 시작 가까이, 버스는 합류점 40 m 앞
    tc.f.bus_pos = T._near_merge() + 30.0                          # 버스가 합류점 10 m 앞까지 와 있다
    inj.update(0.2)
    assert (vid, "junctionModel.ignoreIDs") not in tc.f.params


def test_cutter_waits_until_bus_has_passed_the_signal_before_the_merge():
    # X_jandari7(G16 -> -R96.64)은 J74 신호 뒤에서 합류한다. 버스가 J74에서 적색으로 서 있을 때 넣으면 끼어들기 차가 먼저 합류점을
    # 지나 멀리 앞서가 버스와 만나지 못했다(busy 측정 3바퀴 모두)
    alley = [a for a in T._alleys() if a.junction == "X_jandari7"]
    length = T._net().getEdge("R113").getLength()
    tc = T._TC2(bus_road="R113", bus_pos=length - 3.0, bus_speed=0.0)      # J74 정지선 앞에서 적색 대기
    inj = T._inj(tc, alleys=alley)
    inj.update(0.0)
    inj._s5(0.1, inj._state())
    assert not any(a["id"].startswith("inj_s5") for a in tc.f.added)
    assert "X_jandari7" not in inj._lap["s5_done"]                           # '늦음'으로 끝내지도 않는다
    tc.f.bus_road, tc.f.bus_pos, tc.f.bus_speed = "-R96", 5.0, 8.0          # 신호를 지났다
    inj._s5(5.0, inj._state())
    assert any(a["id"].startswith("inj_s5") for a in tc.f.added)


def test_cutter_timing_uses_the_alley_speed_limit_not_a_fixed_8_mps():
    # 골목(G23)은 제한 속도가 5.56 m/s라 끼어들기 차가 8 m/s로 달리지 못한다. 8 m/s로 계산하면 차가 3초 늦게 닿아 버스가 먼저
    # 지나가거나 버스 바로 뒤에 합류했다(quiet·baseline 측정 S5 0). 골목 제한 속도로 계산해 더 일찍 출발시킨다
    alley = [a for a in T._alleys() if a.junction == "X_jandari2"]
    route = T._route()
    from traffic.test_bus import _back
    edge, pos = _back(T._net(), route, route.index(alley[0].into), 0.0, 100.0)     # J74를 지난 -R96 위, 합류점 100 m 앞(8 m/s로 12.5초)
    tc = T._TC2(bus_road=edge, bus_pos=pos, bus_speed=8.0)
    tc.f.lane.getMaxSpeed = lambda lane: 13.89 if not lane.startswith("G") else 5.56
    inj = T._inj(tc, alleys=alley)
    inj.update(0.0)
    inj._s5(0.1, inj._state())
    assert any(a["id"].startswith("inj_s5") for a in tc.f.added)                    # 5.56 m/s면 이미 출발할 때다


def test_cutter_is_sent_4_seconds_before_the_bus_and_ignores_only_with_2_seconds_margin():
    # 여유 1초로는 버스가 가속해 예상보다 일찍 닿아 교차로 안에서 겹쳤다(busy 측정 충돌 12 샘플). 주입은 4초 먼저, 무시는 2초 이상
    import traffic.scenario_injector as SI
    assert SI.CUT_LEAD_S == 4.0 and SI.IGNORE_MARGIN_S == 2.0


def test_no_cutter_when_bus_is_far_from_the_alley():
    tc = T._TC2(bus_road="R69", bus_pos=10.0)                      # 합류점에서 멀다
    inj = T._inj(tc, alleys=[a for a in T._alleys() if a.junction == "X_dongmak3"])
    inj.update(0.0)
    inj._s5(0.1, inj._state())
    assert not any(a["id"].startswith("inj_s5") for a in tc.f.added)
