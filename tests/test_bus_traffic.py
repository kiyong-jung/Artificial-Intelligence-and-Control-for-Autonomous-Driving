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


def test_pass_through_buses_must_be_fewer_than_lane_buses():
    with pytest.raises(ValueError):
        BusTrafficCfg(lane_buses_per_h=10, pass_buses_per_h=10)
    BusTrafficCfg(lane_buses_per_h=30, pass_buses_per_h=12)


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
    assert lines["bgbus_line_loop"]["stops"] == 6                            # 가로변 노선은 정류장 6곳 모두
    rou = ET.parse(os.path.join(world.out_dir, "traffic.rou.xml")).getroot()
    flows = {f.get("id"): f for f in rou.iter("flow")}
    assert all(flows[k].find("stop") is not None for k in flows if k.startswith("bgbus_line"))   # 정차 후 출발
    assert all(flows[k].find("stop") is None for k in flows if k.startswith(("bgbus_pass", "bgtaxi")))   # 통과 차는 안 선다


def test_pass_buses_are_fewer_than_lane_buses(world):
    s = world.summary["bus_traffic"]
    lane = sum(l["per_h"] for l in s["lines"])
    passing = sum(o["per_h"] for o in s["pass_od"])
    assert 0 < passing < lane
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
    assert {"bgbus", "bgbus_pass", "bgtaxi"} <= types
    kinds = {tc.vehicle.getTypeID(v).split("@")[0] for v in tc.vehicle.getIDList()}
    assert kinds & {"bgbus", "bgbus_pass", "bgtaxi"}
