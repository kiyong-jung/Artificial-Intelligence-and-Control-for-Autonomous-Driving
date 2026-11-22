"""합정역 공식 도로(maps/hapjeong + env.road_builder) 위에 주변 교통이 붙는지 확인한다."""
import os
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.scenarios import ScriptedBus  # noqa: E402


@pytest.fixture(scope="module")
def world(tmp_path_factory):
    w = HapjeongTraffic(str(tmp_path_factory.mktemp("hap")), seed=0)
    w.build()
    yield w
    w.close()


def test_built_from_official_map_without_touching_it(world):
    net = os.path.join(world.out_dir, "network.net.xml")
    assert open(net, "rb").read() == open(load_map("hapjeong")["net_path"], "rb").read()


def test_overlay_is_registered_before_other_route_files(world):
    inputs = ET.parse(world.cfg).getroot().find("input")
    assert inputs.find("route-files").get("value").split(",")[0] == "traffic.rou.xml"
    assert "traffic.add.xml" in inputs.find("additional-files").get("value")


def test_merge_alleys_are_found_from_the_map(world):
    """합류 골목은 지도(도로망 + 버스 경로)에서 자동으로 찾는다. 합정역 3.6.0은 흐름이 있는 합류점이 8곳이고,
    그중 신호 교차로(J74, J48)는 신호가 순서를 정해 끼어들기가 없으므로 뺀다. 남는 곳은 6곳이다."""
    alleys = world.summary["cutin_alleys"]
    active = {a["junction"] for a in alleys if a["flows"] > 0}
    assert active == {"X_dongmak3", "X_dongmak6", "X_wausan11", "X_jandari7", "X_jandari2", "X_jandari3"}
    assert not any(a["junction"] in {"J74", "J48", "cluster_J52_J56_J58"} for a in alleys)
    assert all(a["minor"] and a["regulated"] for a in alleys)        # 골목이 비우선이고 충돌이 기록되는 교차로


def test_s5_candidates_are_only_the_two_chosen_alleys_and_never_signalized(world):
    # S5 끼어들기 후보는 독막로6길(X_dongmak6, G12 -> R71.256)과 잔다리로2길(X_jandari2, G23 -> -R96.119) 두 곳뿐이다.
    # 신호 교차로(cluster_J52_J56_J58, J74, J48)는 원래대로 제외한다. S4(정류장 근처 정차)는 다른 골목도 쓸 수 있어 그대로다
    got = {(a.junction, a.feeder, a.into) for a in world.s5_alleys}
    assert got == {("X_dongmak6", "G12", "R71.256"), ("X_jandari2", "G23", "-R96.119")}
    assert not any(a.signalized for a in world.s5_alleys)
    assert {a.junction for a in world.alleys} >= {"X_dongmak3", "X_wausan11", "X_jandari7", "X_jandari3"}   # S4용은 유지


def test_background_flows_never_use_cutin_type(world):
    # 끼어들기는 시험용 버스와만 상호작용한다. 배경 골목 흐름을 끼어들기 vType(앞차 무시)으로 바꾸면 배경 차와 부딪혔다
    # (busy 측정의 cutin_0 대 배경 차 충돌 11회). 골목 흐름도 보통 배경 차다. 끼어들기 차는 주입기만 시험용 버스가 가까울 때 넣는다
    flows = list(ET.parse(os.path.join(world.out_dir, "routes.rou.xml")).getroot().iter("flow"))
    assert flows and all(f.get("type") != "cutin" for f in flows)
    assert world.summary["cutin_flows"] == 0
    # 합류 골목은 주입기가 쓰도록 계속 찾아 둔다(고리 골목 G12로 합류하는 X_dongmak6 포함)
    assert any(a["junction"] == "X_dongmak6" for a in world.summary["cutin_alleys"])


def test_s6_pedestrians_get_both_types(world):
    # 보행자는 경로 파일에서 떼어 persons.json으로 옮긴다(PersonSpawner가 버스 반경 밖에서 넣는다)
    import json
    assert list(ET.parse(os.path.join(world.out_dir, "pedestrians.rou.xml")).getroot().iter("person")) == []
    with open(os.path.join(world.out_dir, "persons.json"), encoding="utf-8") as f:
        persons = json.load(f)
    typed = [p["type"] for p in persons if p.get("type")]
    assert len(typed) == world.summary["s6_persons"] > 0
    assert set(typed) == {"ped_p1", "ped_p2"}


def test_a1_personality_reaches_background_car_on_hapjeong(world):
    tc = world.start(warmup=30)
    tc.route.add("rb", ["R43"])
    bus = ScriptedBus("bus0", route="rb", bus_lane=1, target_lane=2)
    state = {}

    def hook(tc, t):
        if "added" not in state:
            bus.add(tc)
            tc.vehicle.add("f0", "rb", typeID="car_krauss", departPos="21.5", departLane="2", departSpeed="9")
            tc.vehicle.setMaxSpeed("f0", 9.0)
            state["added"] = True
        if "bus0" in tc.vehicle.getIDList():
            state.setdefault("t_in", t)
            bus.act(tc, t - state["t_in"])

    mgr, _ = world.run(25, bus_hook=hook, stop_exit_pos=("R43", 100.0), exit_max_dist=60.0)
    assert tc.vehicle.getTypeID("f0").startswith("car_krauss")   # 주행 모델은 그대로(setMaxSpeed가 vType을 복제해 접미사가 붙는다)
    assert tc.vehicle.getParameter("f0", "driver.type") in ("yielder", "nonyielder")
    assert any(r["follower"] == "f0" for r in mgr.log)


def test_density_keeper_is_wired_and_can_be_disabled(tmp_path):
    on = HapjeongTraffic(str(tmp_path / "on"), seed=0)
    on.build()
    on.start(warmup=3)
    try:
        assert on.keeper is not None and set(on.keeper.density) == {"양화로", "독막로"}
    finally:
        on.close()
    off = HapjeongTraffic(str(tmp_path / "off"), seed=0, lane_density=None)
    off.build()
    off.start(warmup=3)
    try:
        assert off.keeper is None
    finally:
        off.close()
