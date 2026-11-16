"""신호 교차로 영역에서는 A1·A2(S5)·A3를 켜지 않는다(traffic/signal_areas.py)."""
import json
import os
import sys

import pytest
import sumolib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import close_loop, find_merge_alleys, read_net  # noqa: E402
from traffic.signal_areas import (SIGNAL_NO_CHANGE_M, clip_zones, is_signalized, no_change_m,  # noqa: E402
                                  unsignalized_alleys, unsignalized_crossings)
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


def _lane_ids():
    if "lanes" not in _cache:
        full = sumolib.net.readNet(NET, withInternal=True)
        _cache["lanes"] = [ln.getID() for e in full.getEdges(withInternal=True) for ln in e.getLanes()]
    return _cache["lanes"]


def test_no_change_distance_is_20m_with_10m_at_j1():
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


def test_crossings_at_signals_are_excluded():
    c = unsignalized_crossings(_net(), _lane_ids())
    assert any(x.startswith(":S6_wausanro_c") for x in c)
    assert not any(x.startswith(":J1_c") or x.startswith(":J82_c") for x in c)
