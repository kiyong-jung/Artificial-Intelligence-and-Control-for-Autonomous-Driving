"""PDM 버스 데이터: 정차·신호 재생(pdm_data/replay.py)과 오프라인 93차원 관측(pdm_data/obs93.py)."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_fakes import hapjeong_ctx  # noqa: E402
from pdm_fixtures import make_row  # noqa: E402
from pdm_data.obs93 import obs93  # noqa: E402
from pdm_data.replay import replay_episode  # noqa: E402
from pdm_data.schema import STOP_IDS  # noqa: E402

CTX = hapjeong_ctx()


def fresh_state():
    return {"tracker_index": 0, "dwell": 0.0, "completed": [], "missed": False}


def veh(vid, x, y, speed, heading):
    return {"id": vid, "x": x, "y": y, "speed": speed, "heading": heading}


def _stop_lane(stop):
    edge, idx = stop[3].rsplit("_", 1)
    return edge, int(idx)


def rows_stopped_in(stop, seconds):
    edge, idx = _stop_lane(stop)
    n = int(np.ceil(seconds / 0.5))
    return [make_row(t=i, speed=0.0, edge=edge, lane_index=idx, lap_position=stop[0] + 5.0) for i in range(n)]


def rows_passing(stop):
    edge, idx = _stop_lane(stop)
    return [make_row(t=i, speed=5.0, edge=edge, lane_index=idx, lap_position=stop[1] + 5.0) for i in range(2)]


def rows_crossing_red():
    a = make_row(t=0, speed=5.0, edge="R42", lane_index=1, distance=100.0, tls=[{"id": "J1", "link_index": 0,
                                                                                    "distance": 0.3, "state": "r"}])
    subs = a["window"]["substeps"]
    for k, s in enumerate(subs):
        s["edge"] = "R42" if k < 2 else "R68"
        s["tls0"] = ["J1", 0, 0.3 - 0.5 * (k + 1), "r"] if k < 2 else None
    return [a]


def test_obs93_free_road_shape_and_speed():
    o = obs93(make_row(speed=6.945), CTX, fresh_state())
    assert o.shape == (93,) and o.dtype == np.float32 and abs(o[0] - 0.5) < 1e-6


def test_obs93_vehicle_slot_ahead():
    row = make_row(heading=0.0, vehicles=[veh("a", x=0.0, y=10.0, speed=0.0, heading=0.0)])
    o = obs93(row, CTX, fresh_state())
    assert list(o[6:11]) == pytest.approx([1.0, 0.2, 0.0, -row["ego"]["speed"] / 13.89, 0.0], abs=1e-6)


def test_obs93_vehicle_beyond_50m_not_seen():
    o = obs93(make_row(vehicles=[veh("a", x=0.0, y=60.0, speed=0.0, heading=0.0)]), CTX, fresh_state())
    assert list(o[6:46]) == [0.0] * 40


def test_obs93_internal_lane_mask():
    o = obs93(make_row(edge=":J1_3_0", lane_id=":J1_3_0"), CTX, fresh_state())
    assert list(o[1:6]) == [0.0, 0.0, 1.0, 0.0, 0.0]


def test_obs93_tls_block_red():
    row = make_row(tls=[{"id": "J1", "link_index": 0, "distance": 25.0, "state": "r"}])
    o = obs93(row, CTX, fresh_state())
    assert list(o[81:87]) == pytest.approx([1.0, 0.5, 1.0, 0.0, 0.0, 0.0])


def test_obs93_all_served_stop_block():
    o = obs93(make_row(), CTX, {"tracker_index": 4, "dwell": 0.0, "completed": list(STOP_IDS), "missed": False})
    assert list(o[87:93]) == [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]


def test_replay_service_after_8s_in_stop():
    rows = rows_stopped_in(CTX.service_stops[0], seconds=8.5)
    rep = replay_episode(rows, CTX)
    assert sum(r["services"] for r in rep) == 1 and rep[-1]["completed"] == [STOP_IDS[0]]


def test_replay_state_is_before_window():
    rows = rows_stopped_in(CTX.service_stops[0], seconds=1.0)
    rep = replay_episode(rows, CTX)
    assert rep[0]["dwell"] == 0.0 and rep[1]["dwell"] == pytest.approx(0.5)


def test_replay_missed_when_passing_stop():
    rep = replay_episode(rows_passing(CTX.service_stops[0]), CTX)
    assert rep[-1]["missed"] is True and rep[0]["missed_now"] is True


def test_replay_red_violation():
    rep = replay_episode(rows_crossing_red(), CTX)
    assert any(r["red_violation"] for r in rep)


def test_replay_progress_sums_distance():
    rep = replay_episode([make_row(t=0, speed=4.0, distance=50.0)], CTX)
    assert rep[0]["progress_m"] == pytest.approx(2.0)
