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


def test_every_unsignalized_merge_alley_flow_uses_the_alley_driver_distribution(world):
    # 두 골목만 고르지 않는다. 신호 없는 합류 골목으로 나오는 흐름은 모두 골목 운전자 분포(cutin)이고, 그 밖의 흐름은 그대로다
    from traffic.alleys import flow_merges, read_net
    net = read_net(os.path.join(world.out_dir, "network.net.xml"))
    flows = list(ET.parse(os.path.join(world.out_dir, "routes.rou.xml")).getroot().iter("flow"))
    merging = [f for f in flows if flow_merges(net, f.attrib, world.alleys)]
    assert {a.junction for a in world.alleys} >= {"X_dongmak3", "X_dongmak6", "X_wausan11", "X_jandari7",
                                                  "X_jandari2", "X_jandari3"}
    assert merging and all(f.get("type") == "cutin" for f in merging)
    assert all(f.get("type") == "carDist" for f in flows if f not in merging)
    assert world.summary["cutin_flows"] == len(merging)
    rou = ET.parse(os.path.join(world.out_dir, "traffic.rou.xml")).getroot()
    cutin_types = [v for v in rou.iter("vType") if v.get("id").startswith("cutin_")]
    assert cutin_types and all(float(v.get("jmIgnoreFoeProb")) == 0.0 for v in cutin_types)
    assert len({v.get("impatience") for v in cutin_types}) == len(cutin_types)
    assert not any("ignoreIDs" in (p.get("key") or "") for p in rou.iter("param"))


def test_pedestrians_stay_in_the_route_file_and_all_use_the_walker_distribution(world):
    # 보행자는 경로 파일에 남아 SUMO가 출발시킨다(버스 가까이에서 미루거나 세워 두지 않는다). 모두 같은 보행자 분포를 쓴다
    persons = list(ET.parse(os.path.join(world.out_dir, "pedestrians.rou.xml")).getroot().iter("person"))
    assert len(persons) == world.summary["typed_persons"] > 100
    assert {p.get("type") for p in persons} == {"pedDist"}
    assert any(p.get("id").startswith("s6ped") for p in persons)
    assert not os.path.exists(os.path.join(world.out_dir, "persons.json"))
    rou = ET.parse(os.path.join(world.out_dir, "traffic.rou.xml")).getroot()
    dist = next(d for d in rou.iter("vTypeDistribution") if d.get("id") == "pedDist")
    walkers = [v for v in rou.iter("vType") if v.get("id") in dist.get("vTypes").split()]
    assert len({v.get("impatience") for v in walkers}) == len(walkers) >= 4


def test_driver_parameters_reach_background_car_on_hapjeong(world):
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
    assert 0.0 <= float(tc.vehicle.getParameter("f0", "driver.aggressiveness")) <= 1.0
    assert 0.0 <= float(tc.vehicle.getParameter("f0", "laneChangeModel.lcCooperative")) <= 1.0
    assert any(r["follower"] == "f0" for r in mgr.log)
