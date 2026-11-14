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
    """합류 골목은 지도(도로망 + 버스 경로)에서 자동으로 찾는다. 합정역 3.6.0은 흐름이 있는 합류점이 8곳이다."""
    alleys = world.summary["cutin_alleys"]
    active = {a["junction"] for a in alleys if a["flows"] > 0}
    assert active == {"X_dongmak3", "X_dongmak6", "X_wausan11", "J74", "X_jandari7", "X_jandari2",
                      "X_jandari3", "J48"}
    assert all(a["minor"] and a["regulated"] for a in alleys)        # 골목이 비우선이고 충돌이 기록되는 교차로


def test_merge_alley_flows_use_cutin_type(world):
    from traffic.alleys import flow_merges, read_net
    net = read_net(os.path.join(world.out_dir, "network.net.xml"))
    flows = list(ET.parse(os.path.join(world.out_dir, "routes.rou.xml")).getroot().iter("flow"))
    merging = [f for f in flows if flow_merges(net, f.attrib, world.alleys)]
    assert all(f.get("type") == "cutin" for f in merging)
    assert all(f.get("type") == "carDist" for f in flows if f not in merging)
    assert world.summary["cutin_flows"] == len(merging) > 0
    # a8b98cd(되돌리면 안 됨): 고리 골목(G30 G11 G12)을 via로 지나 버스 경로로 합류하는 흐름도 대상이다.
    # 고리를 나와 서쪽(버스 반대 방향, -R71.55)으로 가는 흐름은 버스와 만나지 않아 대상이 아니다.
    via_loop = [f for f in flows if "G12" in (f.get("via") or "").split()]
    assert via_loop
    assert any(f.get("type") == "cutin" for f in via_loop)
    assert all((f.get("type") == "cutin") == (f in merging) for f in via_loop)


def test_s6_pedestrians_get_both_types(world):
    persons = ET.parse(os.path.join(world.out_dir, "pedestrians.rou.xml")).getroot().iter("person")
    typed = [p.get("type") for p in persons if p.get("type")]
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
