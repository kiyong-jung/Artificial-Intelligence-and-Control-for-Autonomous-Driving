"""PDM 버스 데이터: 상호작용 지표(pdm_data/interaction.py)와 입력 설계 특징(pdm_data/features.py)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_fakes import hapjeong_ctx  # noqa: E402
from pdm_fixtures import make_row  # noqa: E402
from pdm_data.features import FEATURE_NAMES, input_design_features, stage, time_to_green_red  # noqa: E402
from pdm_data.interaction import bumper_gap, pair_metrics, row_interaction  # noqa: E402

CTX = hapjeong_ctx()
DIMS = {"length": 12.0, "width": 2.5, "min_gap": 1.0}
PHASES = [{"duration": 30, "state": "Gr"}, {"duration": 3, "state": "yr"}, {"duration": 30, "state": "rG"}]
PROGRAMS = {"J1": [{"program": "0", "phases": PHASES}]}


def busy_row():
    lead = {"id": "lead", "x": 0.0, "y": 21.0, "speed": 3.0, "heading": 0.0, "acceleration": -1.0, "lateral_speed": 0.0,
            "min_gap": 2.5, "route_distance": 21.0, "lane_index": 1}
    fol = {"id": "fol", "x": 0.0, "y": -18.0, "speed": 9.0, "heading": 0.0, "acceleration": 0.0, "lateral_speed": 0.0,
           "min_gap": 2.5, "route_distance": -18.0, "lane_index": 1}
    ped = {"id": "p1", "x": 4.0, "y": 20.0, "speed": 1.2, "heading": 270.0, "road_id": "x", "next_edge": "",
           "waiting_time": 3.0, "on_crossing": False}
    nb = {k: None for k in ("leader", "follower", "left_leader", "right_leader", "left_follower", "right_follower")}
    nb["leader"] = {"id": "lead", "gap_raw": 8.0, "secure_gap": 6.0}
    nb["follower"] = {"id": "fol", "gap_raw": 3.5, "secure_gap": 9.0}
    row = make_row(speed=8.0, vehicles=[lead, fol], pedestrians=[ped], neighbors=nb,
                   tls=[{"id": "J1", "link_index": 0, "distance": 40.0, "state": "r", "time_to_switch": 10.0,
                         "spent": 20.0, "phase": 2, "program": "0"}], next_stop_distance=150.0)
    row["junction_foes"] = [{"foe_id": "lead", "ego_dist": 30.0, "foe_dist": 20.0, "ego_exit_dist": 40.0,
                             "foe_exit_dist": 30.0, "ego_lane": "a", "foe_lane": "b", "ego_response": True,
                             "foe_response": False}]
    return row


def test_pair_metrics_closing():
    m = pair_metrics(20.0, 10.0, 5.0)
    assert m["thw"] == 2.0 and m["ttc"] == 4.0 and m["inv_ttc"] == 0.25 and m["drac"] == pytest.approx(0.625)


def test_pair_metrics_opening_has_no_ttc():
    m = pair_metrics(20.0, 5.0, 10.0)
    assert m["ttc"] is None and m["inv_ttc"] == 0.0 and m["drac"] == 0.0
    assert pair_metrics(20.0, 0.0, 0.0)["thw"] is None


def test_bumper_gap_adds_min_gap():
    assert bumper_gap({"gap_raw": 9.0}, "leader", 1.0, None) == 10.0
    assert bumper_gap({"gap_raw": 3.5}, "follower", 1.0, 2.5) == 6.0


def test_row_interaction_slots_and_pedestrians():
    it = row_interaction(busy_row(), CTX, DIMS)
    assert it["leader"]["ttc"] == pytest.approx(9.0 / 5.0) and it["leader"]["margin"] == pytest.approx(3.0)
    assert it["follower"]["ttc"] == pytest.approx(6.0 / 1.0) and it["left_leader"] is None
    p = it["pedestrians"][0]
    assert p["id"] == "p1" and p["t_ego"] == pytest.approx(20.0 / 8.0) and p["t_ped"] == pytest.approx(4.0 / 1.2)


def test_time_to_green_red_cycle():
    assert time_to_green_red(PHASES, 2, 10.0, 0) == (10.0, 0.0)
    assert time_to_green_red(PHASES, 0, 5.0, 0) == (0.0, 8.0)
    assert time_to_green_red([{"duration": 30, "state": "G"}], 0, 5.0, 0) == (0.0, math.inf)


def test_stage_rules():
    assert stage(make_row(stop_state=1 | 16), None, 0.0) == "dwell"
    assert stage(make_row(), "dwell", 3.0) == "depart"
    assert stage(make_row(next_stop_distance=80.0), "cruise", 99.0) == "approach"
    assert stage(make_row(next_stop_distance=300.0), "cruise", 99.0) == "cruise"


def test_features_length_and_range():
    f = input_design_features(make_row(), None, CTX, PROGRAMS, DIMS, None, {})
    assert f.shape == (len(FEATURE_NAMES),) and np.all(np.isfinite(f)) and np.all(np.abs(f) <= 1.0)
    assert len(FEATURE_NAMES) == len(set(FEATURE_NAMES)) and 100 <= len(FEATURE_NAMES) <= 130


def test_features_busy_row_values():
    f = dict(zip(FEATURE_NAMES, input_design_features(busy_row(), None, CTX, PROGRAMS, DIMS, (0.5, 1), {})))
    assert f["slot_leader_present"] == 1.0 and f["slot_leader_ds"] == pytest.approx(9.0 / 160.0)
    assert f["slot_left_leader_present"] == 0.0 and f["slot_left_leader_ds"] == 1.0
    assert f["slot_follower_ds"] == pytest.approx(-6.0 / 80.0)
    assert f["tls_r"] == 1.0 and f["tls_ttg"] == pytest.approx(10.0 / 120.0) and f["tls_ttr"] == 0.0
    assert f["ego_prev_accel"] == 0.5 and f["ego_prev_lane_change"] == 1.0
    assert f["foe0_present"] == 1.0 and f["foe0_has_priority"] == 0.0 and f["foe1_phantom"] == 1.0
    assert f["ped0_present"] == 1.0 and f["ped1_present"] == 0.0


def test_features_no_tls_sentinel():
    f = dict(zip(FEATURE_NAMES, input_design_features(make_row(), None, CTX, PROGRAMS, DIMS, None, {})))
    assert f["tls_present"] == 0.0 and f["tls_ttg"] == -1.0 and f["tls_ttr"] == -1.0
