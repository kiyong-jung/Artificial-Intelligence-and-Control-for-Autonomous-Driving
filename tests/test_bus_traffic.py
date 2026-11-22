"""배경 버스·택시 투입(bus_traffic)."""
import os
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.bus_traffic import BusTrafficCfg, CENTRAL_GROUPS  # noqa: E402
from traffic.hapjeong import HapjeongTraffic  # noqa: E402


def test_no_pass_through_buses_in_config():
    assert not hasattr(BusTrafficCfg(), "pass_buses_per_h")        # 배경 버스는 전용 차로 노선뿐이다


@pytest.fixture(scope="module")
def world(tmp_path_factory):
    w = HapjeongTraffic(str(tmp_path_factory.mktemp("bus")), seed=0, bus_traffic=True)
    w.build()
    yield w
    w.close()


def test_bus_lines_use_the_dedicated_lanes_and_stop(world):
    lines = {l["id"]: l for l in world.summary["bus_traffic"]["lines"]}
    for name, via, stop in CENTRAL_GROUPS:
        l = lines[f"bgbus_line_{name}"]
        assert l["stop"] == stop and all(e in l["edges"] for e in via)      # 중앙 우회 차로를 지나 그 정류장에 선다
    assert len(lines) == len(CENTRAL_GROUPS) == 6                            # 중앙 4 + 가로변 2
    rou = ET.parse(os.path.join(world.out_dir, "traffic.rou.xml")).getroot()
    flows = {f.get("id"): f for f in rou.iter("flow")}
    assert all(flows[k].find("stop") is not None for k in flows if k.startswith("bgbus_line"))   # 정차 후 출발
    assert all(flows[k].find("stop") is None for k in flows if k.startswith("bgtaxi"))   # 택시는 정류장 흐름이 없다
    assert not any(k.startswith("bgbus_pass") for k in flows)                            # 통과 버스는 투입하지 않는다
    assert all(k.startswith(("bgbus_line", "bgtaxi", "curbtaxi_")) for k in flows)


def test_central_routes_run_boundary_to_boundary(world):
    # 배경 버스는 J1 같은 교차로에서 생기거나 사라지지 않고 양화로 양 끝 지도 경계에서 경계까지 달린다
    from traffic.alleys import read_net
    from traffic.presence import ends_at_boundary, starts_at_boundary
    net = read_net(os.path.join(world.out_dir, "network.net.xml"))
    lines = {l["id"]: l for l in world.summary["bus_traffic"]["lines"]}
    for name, via, _ in CENTRAL_GROUPS:
        r = lines[f"bgbus_line_{name}"]["edges"]
        assert starts_at_boundary(net, r[0]) and ends_at_boundary(net, r[-1]), (name, r[0], r[-1])
        assert all(v in r for v in via)


def test_curb_taxis_stop_near_bus_stops_on_their_own_schedule(world):
    # S4: 정류장 근처 가로변 차로에 서는 택시는 포아송 흐름이다(시험용 버스 위치에 맞춰 넣지 않는다)
    from traffic.alleys import read_net
    from traffic.signal_areas import forbidden_intervals
    net = read_net(os.path.join(world.out_dir, "network.net.xml"))
    m = load_map("hapjeong")
    spots = [c for c in world.summary["curb_taxis"] if "skipped" not in c]
    loop_stops = [s for s in m["bus_stops"] if s["lane"].rsplit("_", 1)[0] in m["routes"]["bus_loop"]]
    assert len(spots) == len(loop_stops)                                    # 노선 위 정류장 6곳 모두
    bad = forbidden_intervals(net, m["routes"]["bus_loop"])
    for c in spots:
        assert not any(a < c["end"] and c["start"] < b for a, b in bad.get(c["edge"], []))   # 신호 정지선 앞이 아니다
        assert net.getEdge(c["edge"]).getLanes()[c["lane"]].allows("taxi")
    rou = ET.parse(os.path.join(world.out_dir, "traffic.rou.xml")).getroot()
    flows = [f for f in rou.iter("flow") if f.get("id").startswith("curbtaxi_")]
    assert len(flows) == len(spots)
    assert all(f.get("probability") and f.find("stop").get("parkingArea") for f in flows)
    add = ET.parse(os.path.join(world.out_dir, "traffic.add.xml")).getroot()
    assert {p.get("id") for p in add.iter("parkingArea")} >= {f.find("stop").get("parkingArea") for f in flows}


def test_taxis_are_flowing(world):
    s = world.summary["bus_traffic"]
    assert "pass_od" not in s
    assert sum(o["per_h"] for o in s["taxi_od"]) > 0


def test_world_loads_with_bus_traffic(world):
    tc = world.start(warmup=60)
    types = set(tc.vehicletype.getIDList())
    assert {"bgbus", "bgtaxi"} <= types and "bgbus_pass" not in types
    kinds = {tc.vehicle.getTypeID(v).split("@")[0] for v in tc.vehicle.getIDList()}
    assert "bgbus_pass" not in kinds                  # 배경 버스는 전용 차로 노선뿐(예열 60초 안에는 아직 안 나올 수 있다)
