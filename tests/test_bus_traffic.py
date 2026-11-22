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


def test_bus_lines_have_room_to_reach_the_lane_for_the_next_road(world):
    # 도로마다 앞 도로에서 들어오는 차로와 다음 도로로 가는 차로 사이를 옮길 거리가 있다. 차로를 보지 않고 고르면
    # B1(R114 R49a B1)은 22.9 m에서 세 칸, B4(B4b R55 R92)는 102.7 m에서 네 칸을 옮겨야 해 R55에 320초 넘게 섰다.
    from traffic.alleys import read_net
    net = read_net(os.path.join(world.out_dir, "network.net.xml"))
    for line in world.summary["bus_traffic"]["lines"]:
        r = line["edges"]
        for prev, edge, nxt in zip(r, r[1:], r[2:]):
            e = net.getEdge(edge)
            arrive = {c.getToLane().getIndex() for ln in net.getEdge(prev).getLanes() if ln.allows("bus")
                      for c in ln.getOutgoing() if c.getTo() == e and c.getToLane().allows("bus")}
            leave = {ln.getIndex() for ln in e.getLanes() if ln.allows("bus")
                     for c in ln.getOutgoing() if c.getTo().getID() == nxt}
            need = min(abs(a - b) for a in arrive for b in leave)
            assert e.getLength() >= need * e.getSpeed() * 10.0, (line["id"], edge, need, round(e.getLength(), 1))
    lines = {l["id"]: " ".join(l["edges"]) for l in world.summary["bus_traffic"]["lines"]}
    assert "R114 R49a B1" not in lines["bgbus_line_B1"] and "B4b R55 R92" not in lines["bgbus_line_B4"]


def test_curb_taxis_stop_near_every_curbside_bus_stop_on_their_own_schedule(world):
    # S4: 도로망의 가로변 정류장마다 근처에 서는 택시 포아송 흐름이 있다(시험용 버스 노선·위치와 무관하다)
    from traffic.alleys import read_net
    from traffic.signal_areas import is_signalized, no_change_m
    net = read_net(os.path.join(world.out_dir, "network.net.xml"))
    m = load_map("hapjeong")
    taxis = {c["stop"]: c for c in world.summary["curb_taxis"]}
    assert {"stop_4", "stop_5"} <= {k for k, c in taxis.items() if "skipped" not in c}     # 시험용 버스가 서지 않는 정류장
    assert {s["id"] for s in m["bus_stops"]} <= set(taxis)
    assert all(taxis[f"stop_{i}"].get("skipped") == "taxi not allowed" for i in range(4))   # 중앙 버스 전용 차로
    spots = [c for c in taxis.values() if "skipped" not in c]
    for c in spots:
        e = net.getEdge(c["edge"])
        if is_signalized(e.getToNode()):                                     # 신호 정지선 앞이 아니다
            assert c["end"] <= e.getLength() - no_change_m(e.getToNode().getID())
        assert e.getLanes()[c["lane"]].allows("taxi")
    rou = ET.parse(os.path.join(world.out_dir, "traffic.rou.xml")).getroot()
    flows = [f for f in rou.iter("flow") if f.get("id").startswith("curbtaxi_")]
    assert len(flows) == len(spots)
    assert all(f.get("probability") and f.find("stop").get("parkingArea") for f in flows)
    add = ET.parse(os.path.join(world.out_dir, "traffic.add.xml")).getroot()
    assert {p.get("id") for p in add.iter("parkingArea")} >= {f.find("stop").get("parkingArea") for f in flows}


def test_taxis_share_the_demand_of_ordinary_cars(world):
    # 택시의 출발·도착 짝과 비율은 지도 흐름과 같다(시험용 버스 노선을 보고 고르지 않는다)
    s = world.summary["bus_traffic"]
    assert "pass_od" not in s
    flows = load_map("hapjeong")["flows"]
    car = {}
    for f in flows:
        car[(f["from"], f["to"])] = car.get((f["from"], f["to"]), 0.0) + f["vehs_per_hour"]
    taxi = {(o["from"], o["to"]): o["per_h"] for o in s["taxi_od"]}
    assert set(taxi) == set(car)
    k = sum(taxi.values()) / sum(car.values())
    assert sum(taxi.values()) == pytest.approx(BusTrafficCfg().taxis_per_h, rel=1e-2)
    assert all(taxi[od] == pytest.approx(k * car[od], abs=0.01) for od in car)


def test_world_loads_with_bus_traffic(world):
    tc = world.start(warmup=60)
    types = set(tc.vehicletype.getIDList())
    assert {"bgbus", "bgtaxi"} <= types and "bgbus_pass" not in types
    kinds = {tc.vehicle.getTypeID(v).split("@")[0] for v in tc.vehicle.getIDList()}
    assert "bgbus_pass" not in kinds                  # 배경 버스는 전용 차로 노선뿐(예열 60초 안에는 아직 안 나올 수 있다)
