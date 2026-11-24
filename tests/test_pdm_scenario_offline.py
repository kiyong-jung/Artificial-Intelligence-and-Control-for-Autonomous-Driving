"""PDM 버스 데이터: 전처리에서 하는 S5·S6 판정(pdm_data/scenario_offline.py). 규칙은 scripts/measure_s1_s6.py와 같다."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_fakes import hapjeong_ctx  # noqa: E402
from pdm_fixtures import make_row  # noqa: E402
from pdm_data.scenario_offline import s5_events, s6_events  # noqa: E402

CTX = hapjeong_ctx()


def car(vid, road, lane_pos):
    return {"id": vid, "road_id": road, "lane_position": lane_pos, "x": 0.0, "y": 0.0, "speed": 5.0, "heading": 0.0}


def test_s5_cutin_detected_from_rows():
    rows = [make_row(t=0, edge="R96", lane_pos=10.0, vehicles=[car("inj_S51", "G23", 30.0)]),
            make_row(t=1, edge="R96", lane_pos=12.0, vehicles=[car("inj_S51", "R96", 30.0)]),
            make_row(t=2, edge="R96", lane_pos=14.0, vehicles=[car("inj_S51", "R96", 22.0)])]
    rows[2]["ego"]["acceleration"] = -2.0
    ev = s5_events(rows, {"G23"})
    assert len(ev) == 1 and ev[0]["ids"] == ["inj_S51"] and ev[0]["injected"] is True
    d = ev[0]["detail"]
    assert d["alley"] == "G23" and d["gap"] == pytest.approx(18.0) and d["min_gap"] == pytest.approx(8.0)
    assert d["bus_min_accel"] == pytest.approx(-2.0)


def test_s5_not_counted_after_15s_or_behind():
    late = [make_row(t=0, vehicles=[car("c1", "G23", 30.0)])] + \
           [make_row(t=i, sim_time=100.0 + 16.0 + i, vehicles=[car("c1", "R69", 30.0)]) for i in range(1, 3)]
    behind = [make_row(t=0, vehicles=[car("c2", "G23", 30.0)]), make_row(t=1, vehicles=[car("c2", "R69", 5.0)])]
    assert s5_events(late, {"G23"}) == [] and s5_events(behind, {"G23"}) == []


def _passing_rows(cid, at, peds_on):
    """버스가 횡단보도 at 20 m 앞에서 0.1 s마다 1 m씩 지나간다. peds_on(k) = 그 substep의 횡단보도 보행자."""
    rows, k, pos = [], 0, at - 20.0
    for t in range(9):
        row = make_row(t=t, speed=10.0, lap_position=pos)
        for s in row["window"]["substeps"]:
            s["lap_position"], s["t"] = pos, round(100.0 + 0.1 * k, 6)
            s["crossings"] = {cid: peds_on(k)} if -50.0 <= pos - at <= 16.0 else {}
            pos += 1.0
            k += 1
        rows.append(row)
    return rows


def test_s6_crossings_are_unsignalized_only():
    assert [cid for _, cid in CTX.s6_crossings] == [":S6_wausanro_c0"]


def test_s6_pass_with_pet():
    at, cid = CTX.s6_crossings[0]
    ev = s6_events(_passing_rows(cid, at, lambda k: ["p1"] if k < 5 else []), CTX)
    assert len(ev) == 1 and ev[0]["ids"] == ["p1"] and ev[0]["detail"]["crossing"] == cid
    assert ev[0]["detail"]["pet"]["p1"] == pytest.approx(1.5)        # 보행자 100.5에 나감, 버스 앞이 102.0에 진입


def test_s6_ignores_signalized_crossing():
    at, cid = next((a, c) for a, c in CTX.route_map.crossings if c.startswith(":J82_"))
    assert s6_events(_passing_rows(cid, at, lambda k: ["p1"] if k < 5 else []), CTX) == []


def test_s6_no_event_without_pedestrians():
    at, cid = CTX.s6_crossings[0]
    assert s6_events(_passing_rows(cid, at, lambda k: []), CTX) == []
