"""합정역 공식 도로(maps/hapjeong + env.road_builder) 위에 주변 교통이 붙는지 확인한다."""
import os
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.hapjeong import HapjeongTraffic, MERGE_ALLEYS  # noqa: E402
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


def test_merge_alley_flows_use_cutin_type(world):
    flows = list(ET.parse(os.path.join(world.out_dir, "routes.rou.xml")).getroot().iter("flow"))
    types = {f.get("from"): f.get("type") for f in flows}
    assert all(types[a] == "cutin" for a in MERGE_ALLEYS)
    assert all(t == "carDist" for a, t in types.items() if a not in MERGE_ALLEYS)
    assert world.summary["cutin_flows"] == sum(f.get("from") in MERGE_ALLEYS for f in flows) > 0


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
