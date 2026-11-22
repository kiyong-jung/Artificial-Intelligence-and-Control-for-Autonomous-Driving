"""합정역 실행 루프가 시험용 버스 주변의 배경 차와 보행자를 조종하지 않는다(시나리오는 수요와 SUMO 운전자 모델에서 생긴다)."""
import collections
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import run_hapjeong_scenario as RS  # noqa: E402
from maps import load_map  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.test_bus import BUS, TestBusPilot  # noqa: E402

# 배경 차의 움직임이나 존재를 바꾸는 TraCI 명령. 운전자 성향(setParameter, setSpeedFactor), 출발(add),
# 정지선에서 옆 차로로 끝내 못 옮긴 차의 진로 바꾸기(setRoute, LaneRouteAdapter)는 버스와 무관해 허용한다.
VEHICLE_CONTROL = ("setSpeed", "slowDown", "changeLane", "changeSublane", "changeTarget", "remove", "moveTo",
                   "moveToXY", "setStop", "resume", "setLaneChangeMode", "setSpeedMode", "openGap", "setAcceleration",
                   "setMaxSpeed", "replaceStop")
PERSON_CONTROL = ("add", "remove", "appendWaitingStage", "appendWalkingStage", "removeStages", "moveTo", "moveToXY",
                  "setParameter", "setSpeed")


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    from traffic.sumo_conn import traci
    w = HapjeongTraffic(str(tmp_path_factory.mktemp("natural")), scenario="busy", seed=3, bus_traffic=True)
    w.build()
    tc = w.start(warmup=60)
    m = load_map("hapjeong")
    bus = TestBusPilot(tc, read_net(os.path.join(w.out_dir, "network.net.xml")), m["routes"]["bus_loop"],
                       RS.map_stops(m))
    calls = collections.Counter()
    saved = []
    for domain, names in ((traci.vehicle, VEHICLE_CONTROL), (traci.person, PERSON_CONTROL)):
        for name in names:
            orig = getattr(domain, name)
            saved.append((domain, name))

            def wrapped(oid, *a, _orig=orig, _name=name, _dom=domain.__class__.__name__, **k):
                calls[(_dom, _name, oid == BUS)] += 1
                return _orig(oid, *a, **k)
            setattr(domain, name, wrapped)
    try:
        w.run(tc.simulation.getTime() + 300, bus_id=BUS, bus_hook=bus.tick, pilot=bus, triggers="scenario")
    finally:
        for domain, name in saved:
            delattr(domain, name)
    yield w, calls
    w.close()


def test_no_control_command_reaches_a_background_vehicle_or_person(run):
    w, calls = run
    on_others = {k: n for k, n in calls.items() if not k[2]}
    assert on_others == {}
    assert sum(n for k, n in calls.items() if k[2]) > 0          # 기록기가 동작한다: 시험용 버스에 내린 명령은 잡혔다


def test_observers_still_record_partners(run):
    w, _ = run
    assert w.mgr.log or w.mgr.inter or w.stop.log                  # 관찰 기록은 남는다(조종 없이)
    assert w.ped is None                                           # PedSUMO 판정은 기본으로 끈다
