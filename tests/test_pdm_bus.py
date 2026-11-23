"""PDM-Closed 버스 계산(traffic/pdm_bus.py): SUMO 없이 순수 계산만 시험한다."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.pdm_bus import Centerline, PDMCfg, emergency_brake_speed, idm_rollout, score_proposals  # noqa: E402

C = PDMCfg()


def _line(n=200.0):
    return Centerline([(0.0, 0.0), (0.0, n)])                 # 북쪽(+y)으로 곧게


def _obj(y, v, x=0.0, length=4.5, width=1.8):
    return {"xy": np.array([x, y]), "dir": np.array([0.0, 1.0]), "v": v, "length": length, "width": width}


def test_idm_free_road_accelerates_to_v0():
    s, v, acc = idm_rollout(0.0, 10.0, None, 0.0, C)
    assert v[-1] > 4.0 and np.all(acc <= C.accel + 1e-9) and np.all(np.diff(s) >= 0)


def test_idm_brakes_for_stopped_leader_within_limits():
    _, v, acc = idm_rollout(12.0, 15.0, 15.0, 0.0, C)
    assert acc.min() >= -C.decel - 1e-9 and v[-1] < v[0]


def test_free_road_picks_top_speed():
    res = score_proposals(_line(), 8.0, 13.9, None, [], C)
    assert C.fractions[res["best"]] == 1.0 and res["collide_t"] is None


def test_stopped_car_ahead_picks_lower_speed():
    res = score_proposals(_line(), 13.0, 13.9, (25.0, 0.0), [_obj(25.0 + 2.25, 0.0)], C)
    assert C.fractions[res["best"]] < 1.0


def test_object_behind_is_not_at_fault():
    res = score_proposals(_line(), 0.5, 13.9, None, [_obj(-20.0, 8.0)], C)
    assert res["mult"].min() == 1.0


def test_unavoidable_collision_zeroes_score_and_reports_time():
    res = score_proposals(_line(), 3.0, 13.9, None, [_obj(8.0, 0.0)], C)
    assert res["collide_t"] is not None and res["collide_t"] <= 2.0
    assert res["scores"][res["best"]] == 0.0 and res["best"] == 0


def test_emergency_brake_is_monotone_and_stops():
    v = 4.0
    for _ in range(20):
        v = emergency_brake_speed(v, C)
    assert v == 0.0
