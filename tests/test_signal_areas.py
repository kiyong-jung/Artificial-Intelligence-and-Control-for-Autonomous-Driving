"""신호 교차로 영역에서는 A1·A2(S5)를 켜지 않는다(traffic/signal_areas.py)."""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import close_loop, find_merge_alleys, read_net  # noqa: E402
from traffic.signal_areas import (SIGNAL_NO_CHANGE_M, clip_zones, is_signalized, no_change_m,  # noqa: E402
                                  unsignalized_alleys)
from traffic.yield_logic import Zone  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")
NET = os.path.join(ROOT, "source", "network.net.xml")
_cache = {}


def _net():
    if "net" not in _cache:
        _cache["net"] = read_net(NET)
    return _cache["net"]


def _loop():
    with open(os.path.join(ROOT, "map.json"), encoding="utf-8") as f:
        return close_loop(_net(), json.load(f)["routes"]["bus_loop"])


def test_no_change_distance_is_20m_with_10m_at_j1():
    # J1 앞은 빠듯해 사용자가 정한 범위(5~10 m)의 10 m로 둔다(감속 문제를 고친 뒤 다시 10 m로 시험)
    assert SIGNAL_NO_CHANGE_M == 20.0
    assert no_change_m("J1") == 10.0 and no_change_m("J82") == 20.0


def test_is_signalized():
    assert is_signalized(_net().getNode("J82")) and not is_signalized(_net().getNode("X_dongmak6"))


def test_alleys_without_signal_include_dongmak6_right_turn():
    names = {(a.junction, a.feeder, a.into) for a in unsignalized_alleys(find_merge_alleys(_net(), _loop()))}
    assert ("X_dongmak6", "G12", "R71.256") in names
    assert ("X_dongmak3", "G3", "R71.135") in names and ("X_wausan11", "G24", "R112.112") in names
    assert not any(j in {"cluster_J52_J56_J58", "J74", "J48"} for j, _, _ in names)


def test_clip_removes_last_20m_before_j82():
    z = clip_zones(_net(), _loop(), [Zone("R111b", 0.0, 27.0), Zone("R111", 0.0, 91.0)])
    assert any(x.edge == "R111b" and x.end == pytest.approx(7.0, abs=0.5) for x in z)    # 27 - 20
    assert any(x.edge == "R111" and x.end == pytest.approx(91.0, abs=0.5) for x in z)


def test_clip_uses_10m_override_at_j1():
    z = clip_zones(_net(), _loop(), [Zone("R42", 0.0, 16.0)])
    assert any(x.edge == "R42" and x.end == pytest.approx(6.0, abs=0.5) for x in z)       # 16 - 10


def test_clip_crosses_short_edge_backwards():
    # -R96.201(30 m)은 신호 교차로 J48로 끝난다. 끝 20 m를 빼면 앞 10 m만 남는다
    L = _net().getEdge("-R96.201").getLength()
    z = clip_zones(_net(), _loop(), [Zone("-R96.201", 0.0, L)])
    assert [(x.edge, round(x.end)) for x in z] == [("-R96.201", 10)]


def test_scenario_triggers_follow_the_pilot_windows():
    # A1 구간은 시험용 버스의 차선 변경 창과 같다(J1 앞 첫 변경은 R54b에서 일어난다)
    from traffic.hapjeong import HapjeongTraffic
    from traffic.test_bus import lane_plans, lap_route
    with open(os.path.join(ROOT, "map.json"), encoding="utf-8") as f:
        m = json.load(f)
    stops = [{"id": s["id"], "edge": s["lane"].rsplit("_", 1)[0], "startPos": s["startPos"],
              "lane_idx": int(s["lane"].rsplit("_", 1)[1])} for s in m["bus_stops"]]
    plans = lane_plans(_net(), lap_route(_net(), m["routes"]["bus_loop"]), stops)
    h = HapjeongTraffic.__new__(HapjeongTraffic)
    h.out_dir, h.summary = os.path.join(ROOT, "source"), {}
    trig = h.scenario_triggers(plans=plans)
    left = next(t for t in trig if t.side == +1).zones
    right = next(t for t in trig if t.side == -1).zones
    assert {"R54b", "R54", "R42", "R111", "R111b"} <= {z.edge for z in left}
    assert all(not (z.edge == "R42" and z.end > 6.5) for z in left)            # J1 앞 10 m는 뺀다
    assert {z.edge for z in right} <= {"R71.206", "R71.135", "R71"}            # S3는 s2 앞 창뿐
