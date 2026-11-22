"""신호 교차로 우회전(right_turn)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.right_turn import right_turn_links  # noqa: E402


@pytest.fixture(scope="module")
def world(tmp_path_factory):
    w = HapjeongTraffic(str(tmp_path_factory.mktemp("dens")), seed=0)
    w.build()
    yield w
    w.close()


def test_right_on_red_changes_every_signal_but_the_main_crossing(world):
    tc = world.start(warmup=5)
    ror = world.summary["right_on_red"]
    assert "J1" not in ror and "J82" not in ror and "J83" not in ror and "J42" not in ror   # 합정역·상수역·삼거리포차 앞·서교동사거리는 제외
    assert set(ror) == {"J74", "J96", "cluster_J52_J56_J58"}
    assert all(v["links"] > 0 and v["opened_share"] > 0 for v in ror.values())
    # 횡단보도 보행자 신호가 켜져 있는(건너는 사람이 있을 수 있는) 위상에서는 열지 않는다
    assert ror["J74"]["blocked_by_ped"] > 0.1
    assert all(v["opened_share"] + v["blocked_by_ped"] <= 1.0 for v in ror.values())
    # 적색이던 우회전은 일시정지 후 진행('s')으로만 열린다. 서지 않고 지나는 녹색('G', 'g')은 원래 녹색이던 위상뿐이다
    import sumolib
    net = sumolib.net.readNet(os.path.join(world.out_dir, "network.net.xml"), withInternal=False, withPrograms=True)
    for tid, info in ror.items():
        tls = next(t for t in net.getTrafficLights() if t.getID() == tid)
        links = right_turn_links(tls)
        now = [ph.state for ph in tc.trafficlight.getAllProgramLogics(tid)[-1].phases]   # info["orig"]는 마지막 프로그램의 것
        assert any(st[i] == "s" for st in now for i in links)
        for st, orig in zip(now, info["orig"]):
            for i in links:
                assert st[i] == orig[i] or (orig[i] in "ryu" and st[i] == "s"), (tid, i, orig[i], st[i])


def test_right_turn_on_red_stops_before_turning(world):
    # 's'로 연 위상에 우회전한 차는 정지선 앞에서 멈췄다가 지난다(도로교통법의 일시정지)
    tc = world.start(warmup=5)
    ror = world.summary["right_on_red"]
    import sumolib
    net = sumolib.net.readNet(os.path.join(world.out_dir, "network.net.xml"), withInternal=False, withPrograms=True)
    via = {}                                       # 우회전 내부 차로 -> (교차로, 링크 번호)
    for tid in ror:
        tls = next(t for t in net.getTrafficLights() if t.getID() == tid)
        for i in right_turn_links(tls):
            for _, _, v in tc.trafficlight.getControlledLinks(tid)[i]:
                via[v] = (tid, i)
    approach_min, passed = {}, []
    for _ in range(4000):
        tc.simulationStep()
        for v in tc.vehicle.getIDList():
            nxt = tc.vehicle.getNextLinks(v)
            if nxt and nxt[0][4] in via and tc.vehicle.getRoadID(v)[:1] != ":":
                key = (v, nxt[0][4])
                approach_min[key] = min(approach_min.get(key, 99.0), tc.vehicle.getSpeed(v))
                if tc.vehicle.getLanePosition(v) >= tc.lane.getLength(tc.vehicle.getLaneID(v)) - 15.0:
                    tid, i = via[nxt[0][4]]
                    approach_min[key + ("state",)] = tc.trafficlight.getRedYellowGreenState(tid)[i]
            lane = tc.vehicle.getLaneID(v)
            if lane in via and (v, lane) in approach_min and (v, lane) not in {p[0] for p in passed}:
                passed.append(((v, lane), approach_min[(v, lane)], approach_min.get((v, lane, "state"))))
    on_red = [p for p in passed if p[2] == "s"]
    assert len(on_red) >= 5
    assert all(speed < 0.5 for _, speed, _ in on_red), on_red[:5]
