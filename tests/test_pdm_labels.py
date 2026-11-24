"""PDM 버스 데이터: 행동 라벨, 보상 재계산, 에피소드 끝 규칙(pdm_data/labels.py)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from env.bus_task import REWARDS  # noqa: E402
from pdm_fakes import hapjeong_ctx  # noqa: E402
from pdm_fixtures import make_row  # noqa: E402
from pdm_data.labels import (accel_mps2, accel_report, episode_end, lane_change_applied, lane_change_label,  # noqa: E402
                             normalize_accel, reward_terms)
from pdm_data.schema import STOP_IDS  # noqa: E402

CTX = hapjeong_ctx()


def fresh_rep(rows, **over):
    return [dict({"tracker_index": 0, "dwell": 0.0, "completed": [], "missed": False, "services": 0,
                  "red_violation": False, "missed_now": False, "progress_m": 1.0}, **over) for _ in rows]


def rows_with_missed_stop():
    rows = [make_row(t=i) for i in range(4)] + [make_row(t=4, final=True, end="time_limit")]
    rep = fresh_rep(rows)
    rep[1]["missed_now"] = True
    return rows, rep


def test_accel_mps2_and_normalize_clip():
    row = make_row(speed=5.0, window_speeds=[5.2, 5.4, 5.6, 5.8, 6.0])
    assert accel_mps2(row) == pytest.approx(2.0)
    assert normalize_accel(2.0, 1.5, 3.0) == (1.0, True)
    assert normalize_accel(-1.5, 1.5, 3.0) == (-0.5, False)


def test_lane_change_label_whole_request_phase():
    assert lane_change_label(make_row(window_phases=["request"] * 5, window_dirs=[1] * 5)) == 1
    assert lane_change_label(make_row(window_phases=["signal"] * 5, window_dirs=[1] * 5)) == 0
    assert lane_change_label(make_row(window_phases=["idle", "request", "request", "idle", "idle"],
                                      window_dirs=[0, -1, -1, 0, 0])) == -1


def test_lane_change_applied():
    assert lane_change_applied(make_row(lane_before=1, lane_after=2, edge_before="R69", edge_after="R69"), 1)
    assert not lane_change_applied(make_row(lane_before=1, lane_after=1), 1)
    assert not lane_change_applied(make_row(lane_before=2, lane_after=1), 1)
    assert not lane_change_applied(make_row(lane_before=1, lane_after=2, edge_before="R69", edge_after="R69a"), 1)
    assert lane_change_applied(make_row(), 0)


def test_reward_matches_bus_task_formula():
    r = reward_terms(make_row(), {"progress_m": 5.0, "services": 1}, CTX, REWARDS, None)
    assert r["total"] == pytest.approx(10 * 5.0 / CTX.route_map.length - 0.001 + 2.0)
    assert reward_terms(make_row(), {"progress_m": 5.0, "services": 0}, CTX, REWARDS, "collision")["total"] == -10.0
    assert reward_terms(make_row(), {"progress_m": 5.0, "services": 1}, CTX, REWARDS, "arrived")["total"] == 10.0
    assert reward_terms(make_row(), {"progress_m": 5.0, "services": 0}, CTX, REWARDS,
                        "incomplete_service")["total"] == -5.0


def test_collision_terminates():
    rows = [make_row(t=i) for i in range(5)]
    rows[3]["window"]["collisions"] = [{"ego": True}]
    assert episode_end(rows, fresh_rep(rows)) == {"last_index": 3, "terminated": True, "truncated": False,
                                                  "reason": "collision"}


def test_other_collision_does_not_terminate():
    rows = [make_row(t=i) for i in range(3)] + [make_row(t=3, final=True, end="time_limit")]
    rows[1]["window"]["collisions"] = [{"ego": False}]
    assert episode_end(rows, fresh_rep(rows))["reason"] == "time_limit"


def test_teleport_cuts_before():
    rows = [make_row(t=i) for i in range(5)]
    rows[3]["window"]["teleports"] = [{"id": "bus0", "ego": True}]
    end = episode_end(rows, fresh_rep(rows))
    assert end["last_index"] == 2 and end["truncated"] and end["reason"] == "teleport"


def test_time_limit_truncates():
    rows = [make_row(t=i) for i in range(4)] + [make_row(t=4, final=True, end="time_limit")]
    assert episode_end(rows, fresh_rep(rows)) == {"last_index": 3, "terminated": False, "truncated": True,
                                                  "reason": "time_limit"}


def test_lap_done_arrived_or_incomplete():
    rows = [make_row(t=i) for i in range(3)] + [make_row(t=3, final=True, end="lap")]
    rows[2]["window"]["lap_done"] = True
    rep = fresh_rep(rows)
    rep[2].update(completed=list(STOP_IDS[:3]), services=1)
    assert episode_end(rows, rep)["reason"] == "arrived"
    rep[2].update(services=0)
    assert episode_end(rows, rep)["reason"] == "incomplete_service"


def test_missed_stop_cut_and_kept():
    rows, rep = rows_with_missed_stop()
    assert episode_end(rows, rep)["reason"] == "missed_stop" and episode_end(rows, rep)["last_index"] == 1
    assert episode_end(rows, rep, cut_on_task_failure=False)["reason"] != "missed_stop"


def test_accel_report_clip_ratio():
    r = accel_report([0.5, 1.2, 2.0, -1.0, -3.5], [(1.0, 2.0), (2.0, 4.0)])
    assert r["n"] == 5 and r["max"] == 2.0 and r["min"] == -3.5
    assert r["clip_ratio"]["1.0/2.0"] == pytest.approx(3 / 5) and r["clip_ratio"]["2.0/4.0"] == 0.0
