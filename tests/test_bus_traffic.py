"""배경 버스·택시 투입(bus_traffic)과 정류장 앞 정차(stop_ahead)."""
import os
import random
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.bus_traffic import BusTrafficCfg, CENTRAL_GROUPS  # noqa: E402
from traffic.common import Ownership  # noqa: E402
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.stop_ahead import StopAheadManager  # noqa: E402


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
    # 시험용 버스가 서는 정류장(지도 bus_stops 6곳)에는 배경 버스가 서지 않는다
    test_stops = {s["id"] for s in load_map("hapjeong")["bus_stops"]}
    rou = ET.parse(os.path.join(world.out_dir, "traffic.rou.xml")).getroot()
    used = {s.get("busStop") for s in rou.iter("stop")}
    assert used and not (used & test_stops)
    rou = ET.parse(os.path.join(world.out_dir, "traffic.rou.xml")).getroot()
    flows = {f.get("id"): f for f in rou.iter("flow")}
    assert all(flows[k].find("stop") is not None for k in flows if k.startswith("bgbus_line"))   # 정차 후 출발
    assert all(flows[k].find("stop") is None for k in flows if k.startswith("bgtaxi"))   # 택시는 정류장 흐름이 없다
    assert not any(k.startswith("bgbus_pass") for k in flows)                            # 통과 버스는 투입하지 않는다
    assert all(k.startswith(("bgbus_line", "bgtaxi")) for k in flows)


def test_taxis_are_flowing(world):
    s = world.summary["bus_traffic"]
    assert "pass_od" not in s
    assert sum(o["per_h"] for o in s["taxi_od"]) > 0


class _FakeTc:
    class vehicle:
        pass


def _mgr(seed):
    stops = [{"id": f"s{i}", "edge": f"E{i}", "lane": 1, "startPos": 50.0, "endPos": 70.0} for i in range(6)]
    return StopAheadManager(_FakeTc(), "bus0", stops, rng=random.Random(seed), owner=Ownership())


def test_each_lap_blocks_one_or_two_random_distinct_stops():
    counts, picked = set(), set()
    for seed in range(200):
        m = _mgr(seed)
        m._new_lap(0.0)
        counts.add(len(m.plan))
        picked |= set(m.plan)
        assert len(set(m.plan)) == len(m.plan)
    assert counts == {1, 2}                  # 1 이상 3 미만
    assert picked == {f"s{i}" for i in range(6)}      # 6곳 모두 뽑힐 수 있다(무작위)


def test_world_loads_with_bus_traffic(world):
    tc = world.start(warmup=60)
    types = set(tc.vehicletype.getIDList())
    assert {"bgbus", "bgtaxi"} <= types and "bgbus_pass" not in types
    kinds = {tc.vehicle.getTypeID(v).split("@")[0] for v in tc.vehicle.getIDList()}
    assert "bgbus_pass" not in kinds                  # 배경 버스는 전용 차로 노선뿐(예열 60초 안에는 아직 안 나올 수 있다)
