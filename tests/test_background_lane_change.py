"""배경 차가 신호 대기열 앞에서 차로를 바꾸다 두 차로에 걸친 채 서 있지 않고, 느린 차는 여전히 앞지른다(실제 SUMO).

배경 차 종류는 env/road_builder.build가 쓰는 routes.rou.xml에서 그대로 가져온다. 도로는 3차로 직선 400m와 신호
(녹색 30초, 적색 90초)뿐인 합성 도로다.
"""
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import traci
from sumolib import checkBinary

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from env import road_builder  # noqa: E402
from env.road_config import EGO, ROAD, TRAFFIC, configure_map  # noqa: E402

STEP = 0.1


@pytest.fixture(scope="module")
def road(tmp_path_factory):
    d = tmp_path_factory.mktemp("queue")
    configure_map("busy", 0, None, "hapjeong")
    built = ET.parse(os.path.dirname(road_builder.build(ROAD, EGO, TRAFFIC, str(d / "hapjeong"))) + "/routes.rou.xml")
    background = [el for el in built.getroot() if el.tag == "vTypeDistribution"]
    (d / "n.nod.xml").write_text('<nodes><node id="W" x="-400" y="0"/><node id="C" x="0" y="0" type="traffic_light"/>'
                                 '<node id="E" x="300" y="0"/></nodes>')
    (d / "e.edg.xml").write_text('<edges><edge id="in" from="W" to="C" numLanes="3" speed="13.89" width="3.2"/>'
                                 '<edge id="out" from="C" to="E" numLanes="3" speed="13.89" width="3.2"/></edges>')
    for name, extra in (("signal.net.xml", ["--tls.green.time", "30", "--tls.red.time", "90"]),
                        ("open.net.xml", ["--tls.unset", "C"])):
        subprocess.run([checkBinary("netconvert"), "-n", str(d / "n.nod.xml"), "-e", str(d / "e.edg.xml"),
                        "-o", str(d / name), "--no-warnings", *extra], check=True, capture_output=True)
    return d, background


def _routes(path, background, *vehicles):
    root = ET.Element("routes")
    root.extend(background)
    ET.SubElement(root, "route", id="r", edges="in out")
    for tag, attrs in vehicles:
        ET.SubElement(root, tag, attrs)
    ET.ElementTree(root).write(path)


def _start(net, routes, seed, end):
    if traci.isLoaded():
        traci.close()
    traci.start([checkBinary("sumo"), "-n", str(net), "-r", str(routes), "--step-length", str(STEP),
                 "--lateral-resolution", str(TRAFFIC["lateral_resolution"]), "--seed", str(seed), "--end", str(end),
                 "--no-step-log", "--no-warnings"])


def test_queued_cars_do_not_stand_across_two_lanes(road):
    """한 차로를 3.2m 옮기려면 옆 속도 한계(속도의 0.2배)로 16m를 달려야 한다. 대기열 앞에서 옮기던 차가 차로 경계에서
    되돌아오면 그 거리가 모자라 차로 가운데에서 0.5m 넘게 벗어난 채 다음 녹색까지 선다."""
    d, background = road
    _routes(d / "queue.rou.xml", background, ("flow", {
        "id": "f", "type": "carDist", "route": "r", "begin": "0", "end": "600", "period": "exp(0.4)",
        "departLane": "best", "departSpeed": "max"}))
    frozen = []
    for seed in (0, 1):
        _start(d / "signal.net.xml", d / "queue.rou.xml", seed, 600)
        since = {}
        while traci.simulation.getTime() < 600:
            traci.simulationStep(traci.simulation.getTime() + 0.5)
            t = traci.simulation.getTime()
            for vid in traci.vehicle.getIDList():
                off = (traci.vehicle.getSpeed(vid) < 0.5 and abs(traci.vehicle.getLateralLanePosition(vid)) > 0.5
                       and not traci.vehicle.getLaneID(vid).startswith(":"))
                if not off:
                    since.pop(vid, None)
                elif t - since.setdefault(vid, t) >= 10.0 and (seed, vid) not in frozen:
                    frozen.append((seed, vid))
        traci.close()
    assert len(frozen) <= 2, f"차로 가운데에서 0.5m 넘게 벗어나 10초 넘게 선 배경 차 {len(frozen)}대: {frozen}"


def test_cars_still_overtake_a_slow_bus(road):
    d, background = road
    _routes(d / "slow.rou.xml", background,
            ("flow", {"id": "f", "type": "carDist", "route": "r", "begin": "0", "end": "120", "period": "exp(0.25)",
                      "departLane": "best", "departSpeed": "max"}),
            ("vType", {"id": "slow", "vClass": "bus", "length": "12", "width": "2.5", "maxSpeed": "6"}),
            ("vehicle", {"id": "bus", "type": "slow", "route": "r", "depart": "20", "departLane": "0",
                         "departSpeed": "max"}))
    passed = set()
    for seed in (0, 2):
        _start(d / "open.net.xml", d / "slow.rou.xml", seed, 300)
        behind = set()
        while traci.simulation.getTime() < 300 and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            if "bus" not in traci.vehicle.getIDList():
                continue
            bus, lane = traci.vehicle.getDistance("bus"), traci.vehicle.getLaneIndex("bus")
            for vid in traci.vehicle.getIDList():
                gap = bus - traci.vehicle.getDistance(vid)
                if 12.0 < gap < 40.0 and traci.vehicle.getLaneIndex(vid) == lane:
                    behind.add(vid)
                elif gap < -5.0 and vid in behind:
                    passed.add((seed, vid))
        traci.close()
    assert len(passed) >= 3, f"버스 뒤 같은 차로에 있다가 버스를 앞지른 배경 차 {len(passed)}대"
