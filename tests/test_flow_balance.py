"""배경 흐름 재배분(flow_balance): 노선 흐름은 키우고 전체 대수는 지키며, 와우산로·잔다리로에는 투입하지 않고 그 도로를 피하게 한다."""
import os
import random
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.alleys import close_loop, read_net  # noqa: E402
from traffic.flow_balance import (balance_factors, edges_named, path_ids, redirect_flows, route_touching,  # noqa: E402
                                  segment_before_node)
from traffic.hapjeong import HapjeongTraffic  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")
AVOID_NAMES = ("와우산로", "잔다리로")
_c = {}


def _net():
    if "net" not in _c:
        _c["net"] = read_net(os.path.join(ROOT, "source", "network.net.xml"))
    return _c["net"]


def _flows():
    return [{"id": f["id"], "from": f["from"], "to": f["to"], "rate": f["vehs_per_hour"]} for f in load_map("hapjeong")["flows"]]


def test_balance_conserves_total_and_scales_route_flows_by_boost():
    rates = {"a": 100.0, "b": 200.0, "c": 300.0, "d": 400.0}
    touching = {"a", "b"}
    on, off = balance_factors(rates, touching, 1.5)
    assert on == pytest.approx(1.5)
    new = {k: r * (on if k in touching else off) for k, r in rates.items()}
    assert sum(new.values()) == pytest.approx(sum(rates.values()))
    assert off == pytest.approx((1000 - 450) / 700) and off < 1.0


def test_balance_can_target_a_total_different_from_the_current_sum():
    rates = {"a": 100.0, "b": 300.0}                           # 지금 400, 목표 600
    on, off = balance_factors(rates, {"a"}, 1.5, target_total=600.0)
    assert on == pytest.approx(1.5) and on * 100 + off * 300 == pytest.approx(600.0)


def test_boost_is_capped_so_that_other_flows_keep_at_least_min_off():
    rates = {"a": 500.0, "b": 500.0}
    on, off = balance_factors(rates, {"a"}, 10.0, min_off=0.3)
    assert off == pytest.approx(0.3) and on == pytest.approx((1000 - 0.3 * 500) / 500)
    assert on * 500 + off * 500 == pytest.approx(1000)


def test_no_change_without_boost_or_without_both_kinds_of_flows():
    rates = {"a": 100.0, "b": 100.0}
    assert balance_factors(rates, {"a"}, 1.0) == (1.0, 1.0)
    assert balance_factors(rates, set(), 2.0) == (1.0, 1.0)
    assert balance_factors(rates, {"a", "b"}, 2.0) == (1.0, 1.0)


def test_only_part_of_the_map_flows_touch_the_bus_route():
    net = _net()
    m = load_map("hapjeong")
    loop = close_loop(net, m["routes"]["bus_loop"])
    pairs = [(f["from"], f["to"]) for f in m["flows"]]
    touching = route_touching(net, pairs, loop)
    on = sum(f["vehs_per_hour"] for f in m["flows"] if (f["from"], f["to"]) in touching)
    total = sum(f["vehs_per_hour"] for f in m["flows"])
    assert 0.3 < on / total < 0.45                        # 분석: 87개 402/1061 = 38 %
    assert 0 < len(touching) < len(pairs)


def test_forbidden_edges_exclude_flows_from_route_touching():
    net = _net()
    loop = close_loop(net, load_map("hapjeong")["routes"]["bus_loop"])
    pairs = [(f["from"], f["to"]) for f in _flows()]
    all_hit = route_touching(net, pairs, loop)
    corridor = set(segment_before_node(net, loop, "J82"))
    some = route_touching(net, pairs, loop, forbidden=corridor)
    assert some < all_hit and all(not any(e in corridor for e in path_ids(net, *p)) for p in some)


def test_segment_before_j82_is_the_independent_approach():
    loop = close_loop(_net(), load_map("hapjeong")["routes"]["bus_loop"])
    assert segment_before_node(_net(), loop, "J82") == ["R77", "R111", "R111b"]
    assert segment_before_node(_net(), loop, "NOPE") == []


def test_edges_named_includes_both_directions_and_the_entrances():
    avoid = edges_named(_net(), AVOID_NAMES)
    assert {"R112", "R112.112", "R112.221", "R113", "R94", "-R96"} <= avoid       # 노선의 와우산로·잔다리로
    assert {"-R129", "-R130", "-R92.163", "R129", "R130", "R92"} <= avoid         # 입구 D·E·H와 출구 2·3·9


def test_redirect_removes_avoid_road_entrances_and_keeps_every_flow_off_the_avoid_roads():
    net = _net()
    avoid = edges_named(net, AVOID_NAMES)
    flows = _flows()
    before = sum(f["rate"] for f in flows)
    stats = redirect_flows(net, flows, avoid, random.Random(0))
    for f in flows:
        if f["rate"] <= 0:
            continue
        ids = path_ids(net, f["from"], f["to"])
        assert f["from"] not in avoid and f["to"] not in avoid and not any(e in avoid for e in ids), f["id"]
    assert all(f["rate"] == 0.0 for f in flows if f["from"] in {"-R129", "-R130", "-R92.163"})     # 투입 없음
    assert stats["no_insert"] >= 3 and stats["redirected"] > 0
    assert stats["removed_per_h"] == pytest.approx(before - sum(f["rate"] for f in flows))
    assert stats["removed_per_h"] >= 3 * 86.0                                                       # 입구 D·E·H 86.3 x 3


def test_redirect_changes_destination_to_another_exit_instead_of_dropping_when_possible():
    net = _net()
    avoid = edges_named(net, AVOID_NAMES)
    flows = [{"id": "x", "from": "-R128", "to": "R129", "rate": 50.0},          # 독막로 입구 -> 와우산로 출구
             {"id": "y", "from": "-R128", "to": "R115", "rate": 50.0}]
    redirect_flows(net, flows, avoid, random.Random(1))
    x = flows[0]
    assert x["rate"] == 50.0 and x["to"] not in avoid and x["to"] != "R129"
    assert not any(e in avoid for e in path_ids(net, x["from"], x["to"]))


def test_build_removes_avoid_entrances_keeps_total_and_does_not_boost_the_j82_approach(tmp_path):
    m = load_map("hapjeong")
    base = sum(f["vehs_per_hour"] for f in m["flows"])
    w = HapjeongTraffic(str(tmp_path / "bal"), seed=0, route_flow_boost=1.3, interior_share=0.0)
    w.build()
    try:
        rou = ET.parse(os.path.join(w.out_dir, "routes.rou.xml")).getroot()
        net = read_net(os.path.join(w.out_dir, "network.net.xml"))
        avoid = edges_named(net, AVOID_NAMES)
        loop = close_loop(net, m["routes"]["bus_loop"])
        corridor = set(segment_before_node(net, loop, "J82"))
        cars = [f for f in rou.iter("flow") if f.get("type") == "carDist"]
        total = sum(float(f.get("period")[4:-1]) * 3600 for f in cars)
        assert total == pytest.approx(base, rel=0.01)                                       # 전체 대수 그대로
        info = w.summary["flow_balance"]
        assert info["no_boost_edges"] == ["R111", "R111b", "R77"] and info["no_insert"] >= 3 and info["redirected"] > 0
        assert info["on_factor"] == pytest.approx(1.3) and info["off_factor"] > 0.3
        for f in cars:
            assert f.get("from") not in avoid and f.get("to") not in avoid                  # 와우산로·잔다리로에 투입하지 않는다
            ids = path_ids(net, f.get("from"), f.get("to"))
            assert not any(e in avoid for e in ids), f.get("id")                            # 지나지도 않는다(J82·J83에서 빠진다)
        # J82 접근로를 지나는 흐름은 노선 흐름으로 치지 않으므로 boost 배율이 아닌 나머지 배율을 받는다
        factors = {}
        orig = {x["id"]: x for x in m["flows"]}
        for f in cars:
            if any(e in corridor for e in path_ids(net, f.get("from"), f.get("to"))) and f.get("id") in orig:
                factors[f.get("id")] = float(f.get("period")[4:-1]) * 3600 / orig[f.get("id")]["vehs_per_hour"]
        assert factors and all(v == pytest.approx(info["off_factor"], rel=0.03) for v in factors.values())
    finally:
        w.close()


def test_boost_one_still_keeps_the_avoid_roads_clear_without_boosting(tmp_path):
    w = HapjeongTraffic(str(tmp_path / "same"), seed=0, route_flow_boost=1.0, interior_share=0.0)
    w.build()
    try:
        rou = ET.parse(os.path.join(w.out_dir, "routes.rou.xml")).getroot()
        net = read_net(os.path.join(w.out_dir, "network.net.xml"))
        avoid = edges_named(net, AVOID_NAMES)
        info = w.summary["flow_balance"]
        assert info["on_factor"] == 1.0 and info["no_insert"] >= 3                              # 키우지는 않지만 입구는 없앤다
        assert not any(f.get("from") in avoid for f in rou.iter("flow") if f.get("type") == "carDist")
    finally:
        w.close()
