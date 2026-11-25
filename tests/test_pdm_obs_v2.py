"""관측 v2(pdm_data/obs_v2.py, docs/observation_features_v2.md)와 5값 차로 라벨."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_fakes import hapjeong_ctx  # noqa: E402
from pdm_fixtures import make_row  # noqa: E402
from pdm_data.labels import lane_change_label5, lateral_move  # noqa: E402
from pdm_data.obs_v2 import (EGO_DIM, OBS_V2_DIM, PED_DIM, VEH_DIM, V2Context, _crossing_shapes,  # noqa: E402
                             FEATURE_NAMES, obs_v2)

CTX = hapjeong_ctx()
V2 = V2Context(CTX)
VMAX = 13.89
VEH0, PED0 = EGO_DIM, EGO_DIM + 8 * VEH_DIM
E = {n: i for i, n in enumerate(FEATURE_NAMES[:EGO_DIM])}


def state(index=0, dwell=0.0):
    return {"tracker_index": index, "dwell": dwell, "completed": [], "missed": False}


def ego(row):
    return obs_v2(row, V2, state())[:EGO_DIM]


def veh(vid, x, y, speed=0.0, heading=0.0, **kw):
    return {"id": vid, "x": x, "y": y, "speed": speed, "heading": heading, "road_id": "R69", "lane_index": 1,
            "signals": 0, **kw}


def ped(pid, x, y, speed=0.0, heading=0.0, on_crossing=False):
    return {"id": pid, "x": x, "y": y, "speed": speed, "heading": heading, "on_crossing": on_crossing}


def test_shape_and_names():
    o = obs_v2(make_row(), V2, state())
    assert o.shape == (OBS_V2_DIM,) == (166,) and o.dtype == np.float32 and len(FEATURE_NAMES) == 166


def test_speed_and_asymmetric_acceleration():
    row = make_row(speed=VMAX / 2)
    for a, want in ((1.5, 1.0), (-9.0, -1.0), (-3.0, -1 / 3), (0.0, 0.0)):
        row["ego"]["acceleration"] = a
        o = ego(row)
        assert o[E["speed"]] == pytest.approx(0.5) and o[E["acceleration"]] == pytest.approx(want)


def test_prev_action_lane_divided_by_two():
    row = make_row()
    assert list(obs_v2(row, V2, state(), None)[2:4]) == [0.0, 0.0]
    assert list(obs_v2(row, V2, state(), (0.4, 2))[2:4]) == pytest.approx([0.4, 1.0])
    assert obs_v2(row, V2, state(), (0.0, -1))[3] == pytest.approx(-0.5)


def test_lane_exists_is_map_only_and_zero_inside_junction():
    o = ego(make_row(lane_index=2))                       # 3차로의 맨 왼쪽
    assert o[E["left_lane_exists"]] == 0.0 and o[E["right_lane_exists"]] == 1.0
    v2 = V2Context(hapjeong_ctx({"R69_0": ["pedestrian"], "R69_1": [], "R69_2": []}))
    o = obs_v2(make_row(lane_index=1), v2, state())[:EGO_DIM]   # 오른쪽 R69_0은 버스 통행 불가
    assert o[E["left_lane_exists"]] == 1.0 and o[E["right_lane_exists"]] == 0.0
    o = ego(make_row(edge=":J1_3_0", lane_id=":J1_3_0", lane_index=0))
    assert o[E["left_lane_exists"]] == 0.0 and o[E["right_lane_exists"]] == 0.0


def test_continuation_follows_active_plan_and_deadline():
    start = CTX.route_map.start[1]
    v2 = V2Context(CTX, [{"kind": "S2", "edge": "R69", "end_pos": 50.0, "target_lane": 2, "length": 100.0,
                          "ok_lanes": [2]}])
    o = obs_v2(make_row(lane_index=1, lap_position=start + 10.0), v2, state())[:EGO_DIM]
    assert (o[E["cont_current"]], o[E["cont_left"]], o[E["cont_right"]]) == (0.0, 1.0, 0.0)
    assert o[E["lane_deadline_distance"]] == pytest.approx(40.0 / 200.0)
    o = obs_v2(make_row(lane_index=1, lap_position=start + 60.0), v2, state())[:EGO_DIM]   # 창을 지났다
    assert o[E["lane_deadline_distance"]] == 1.0 and o[E["cont_current"]] == 1.0          # 지도 연결: R69_1은 이어진다


def test_signal_and_stop_point():
    o = ego(make_row(tls=[{"id": "J1", "link_index": 0, "distance": 25.0, "state": "r"}]))
    assert o[E["stop_point_distance"]] == pytest.approx(0.125) and o[E["signal_state"]] == 0.0
    o = ego(make_row(tls=[{"id": "J1", "link_index": 0, "distance": 25.0, "state": "y"}]))
    assert o[E["signal_state"]] == 0.5
    o = ego(make_row())
    assert o[E["stop_point_distance"]] == 1.0 and o[E["signal_state"]] == 1.0


def test_unsignalized_crossing_is_a_stop_point():
    at = CTX.s6_crossings[0][0]
    o = ego(make_row(lap_position=at - 20.0, tls=[{"id": "J83", "link_index": 0, "distance": 90.0, "state": "r"}]))
    assert o[E["stop_point_distance"]] == pytest.approx(0.1) and o[E["signal_state"]] == 1.0


def test_dwell_remaining_and_lanes_to_stop():
    start, end, _, lane = CTX.service_stops[0]
    edge, idx = lane.rsplit("_", 1)
    k = CTX.lap_edges.index(edge)
    row = make_row(edge=edge, lane_index=int(idx), lap_position=start + 3.0, speed=0.0, lap_route_index=k)
    assert obs_v2(row, V2, state(0, 2.0))[E["dwell_remaining"]] == pytest.approx(6.0 / 8.0)
    row = make_row(edge=edge, lane_index=int(idx) + 1, lap_position=start - 50.0, lap_route_index=k)
    o = obs_v2(row, V2, state())[:EGO_DIM]
    assert o[E["lanes_to_stop"]] == pytest.approx(-1 / 5) and o[E["stop_distance"]] == pytest.approx(0.25)
    assert o[E["dwell_remaining"]] == 0.0


def test_vehicle_ahead_slot():
    v = 6.0
    row = make_row(speed=v, heading=0.0, vehicles=[veh("a", 0.0, 10.0, lane_index=2, signals=2)])
    s = obs_v2(row, V2, state())[VEH0:VEH0 + VEH_DIM]
    q = v / 10.0
    want = [1.0, 0.2, 0.0, -v / (2 * VMAX), 0.0, 1.0, 0.0, q / (1 + q), 1 / 5, 0.0, 1.0, 0.0]
    assert list(s) == pytest.approx(want, abs=1e-6)


def test_vehicle_left_crossing_other_road_and_far():
    # ego 북쪽(0°). 서쪽(x -5)은 왼쪽, 동쪽으로 가는 차(90°)는 오른쪽으로 향하므로 sin -1
    row = make_row(speed=0.0, vehicles=[veh("b", -5.0, 0.0, speed=2.0, heading=90.0, road_id="G1")])
    s = obs_v2(row, V2, state())[VEH0:VEH0 + VEH_DIM]
    assert s[2] == pytest.approx(0.1) and s[5] == pytest.approx(0.0, abs=1e-6) and s[6] == pytest.approx(-1.0)
    assert s[4] == pytest.approx(-2.0 / (2 * VMAX)) and s[8] == 0.0 and s[9] == 1.0
    assert s[7] == pytest.approx(0.4 / 1.4)            # ego 쪽으로 2 m/s, 거리 5 m
    away = obs_v2(make_row(speed=0.0, vehicles=[veh("b", -5.0, 0.0, speed=2.0, heading=270.0)]), V2, state())
    assert away[VEH0 + 7] == 0.0                       # 멀어지면 0
    far = obs_v2(make_row(vehicles=[veh("c", 0.0, 60.0)]), V2, state())[VEH0:PED0]
    assert not far.any()


def test_pedestrian_own_velocity_and_crossing():
    row = make_row(speed=8.0, pedestrians=[ped("p", 3.0, 10.0, speed=1.0, heading=90.0, on_crossing=True)])
    s = obs_v2(row, V2, state())[PED0:PED0 + PED_DIM]
    assert list(s) == pytest.approx([1.0, 0.2, -0.06, 0.0, -0.5, 0.0, -1.0, 1.0, 0.0], abs=1e-6)


def test_pedestrian_crossing_distance():
    shape, _ = _crossing_shapes(CTX.net_path)[0]
    x, y = shape[0]
    row = make_row(x=x, y=y - 20.0, pedestrians=[ped("p", x, y), ped("q", x + 0.0, y - 49.0)])
    o = obs_v2(row, V2, state())
    near = o[PED0:PED0 + PED_DIM]
    assert near[7] == 0.0 and near[8] == pytest.approx(0.0, abs=1e-6)
    assert 0.0 <= o[PED0 + PED_DIM + 8] <= 1.0


def test_lane_label5():
    assert lane_change_label5(make_row(window_phases=["signal"] * 5, window_dirs=[1] * 5)) == 1
    assert lane_change_label5(make_row(window_phases=["request"] * 5, window_dirs=[-1] * 5)) == -1
    assert lane_change_label5(make_row(window_phases=["request"] * 5, window_dirs=[1] * 5,
                                       lane_before=1, lane_after=2)) == 2
    assert lane_change_label5(make_row(lane_before=1, lane_after=0)) == -2
    # 도로 경계에서 번호만 바뀐 것은 이동이 아니다
    row = make_row(lane_index=1, lane_before=1, lane_after=2, edge_before="R69", edge_after="R69a")
    assert lateral_move(row) == 0 and lane_change_label5(row) == 0
    assert lane_change_label5(make_row()) == 0


def test_ranges_on_random_rows():
    rng = np.random.default_rng(0)
    vs = [veh(f"v{i}", *rng.uniform(-40, 40, 2), speed=rng.uniform(0, VMAX), heading=rng.uniform(0, 360),
              lane_index=int(rng.integers(0, 6)), signals=int(rng.integers(0, 4))) for i in range(12)]
    ps = [ped(f"p{i}", *rng.uniform(-40, 40, 2), speed=rng.uniform(0, 1.9), heading=rng.uniform(0, 360))
          for i in range(9)]
    o = obs_v2(make_row(speed=VMAX, vehicles=vs, pedestrians=ps), V2, state(), (-1.0, -2))
    assert np.abs(o).max() <= 1.0 + 1e-6 and math.isfinite(float(o.sum()))
