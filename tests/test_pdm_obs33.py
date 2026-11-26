"""관측 33차원(pdm_data/obs33.py, docs/superpowers/specs/2026-11-26-obs33-mlp-bc-design.md 5절)."""
import math
import os
import sys

import numpy as np
import pytest
from sumolib.geomhelper import positionAtShapeOffset

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_fakes import hapjeong_ctx  # noqa: E402
from pdm_fixtures import make_row  # noqa: E402
from pdm_data.obs33 import FEATURE_NAMES_33, OBS33_DIM, Obs33Context, obs33  # noqa: E402

CTX = hapjeong_ctx()
C = Obs33Context(CTX)
NET, RM, E = C.net, CTX.route_map, CTX.lap_edges
VMAX = CTX.vmax
I = {n: i for i, n in enumerate(FEATURE_NAMES_33)}
STOP_S4 = [s[2] for s in CTX.service_stops].index("osm_stop_5930628870")


def state(index=0, dwell=0.0):
    return {"tracker_index": index, "dwell": dwell, "completed": [], "missed": False}


def pose(lane_id, pos):
    shape = NET.getLane(lane_id).getShape()
    x, y = positionAtShapeOffset(shape, pos)
    x1, y1 = positionAtShapeOffset(shape, pos + 0.5)
    return x, y, math.degrees(math.atan2(x1 - x, y1 - y)) % 360.0


def bus_row(lane_id, pos, speed=5.0, **kw):
    edge, idx = lane_id.rsplit("_", 1)
    k = E.index(edge)
    x, y, h = pose(lane_id, pos)
    row = make_row(edge=edge, lane_index=int(idx), lane_id=lane_id, lane_pos=pos, lap_position=RM.start[k] + pos,
                   lap_route_index=k, x=x, y=y, heading=h, speed=speed, **kw)
    row["ego"]["lane_count"] = NET.getEdge(edge).getLaneNumber()
    return row


def car(vid, lane_id, pos, speed=0.0, length=5.0, width=2.0, heading=None):
    x, y, h = pose(lane_id, pos)
    edge, idx = lane_id.rsplit("_", 1)
    return {"id": vid, "x": x, "y": y, "heading": h if heading is None else heading, "speed": speed, "length": length,
            "width": width, "lane_id": lane_id, "lane_position": pos, "road_id": edge, "lane_index": int(idx)}


def ped(pid, x, y, speed=0.0, heading=0.0):
    return {"id": pid, "x": x, "y": y, "speed": speed, "heading": heading, "on_crossing": True}


def o(row, st=None):
    return obs33(row, C, st or state())


def test_shape_names_range():
    v = o(bus_row("R69_1", 10.0))
    assert v.shape == (OBS33_DIM,) == (33,) and v.dtype == np.float32 and np.abs(v).max() <= 1.0
    assert len(FEATURE_NAMES_33) == 33 and FEATURE_NAMES_33[-2:] == ["stop_point_distance", "signal_state"]


def test_absent_lane_defaults():
    v = o(bus_row("R69_1", 10.0))            # 차로 -1은 없다
    assert [v[I[f"right2_{n}"]] for n in ("leader_gap", "leader_relative_speed", "follower_gap",
                                          "follower_relative_speed", "route_connectivity", "density")] == \
        [1.0, 1.0, -1.0, -1.0, -1.0, 1.0]


def test_same_lane_leader_gap_and_speed():
    v = o(bus_row("R69_1", 10.0, speed=5.0, vehicles=[car("a", "R69_1", 40.0, speed=5.0)]))
    assert v[I["current_leader_gap"]] == pytest.approx(25.0 / 50.0, abs=1e-4)
    assert v[I["current_leader_relative_speed"]] == pytest.approx(0.0, abs=1e-3)


def test_crossing_pedestrian_is_obstacle():
    x, y, h = pose("R69_1", 30.0)
    v = o(bus_row("R69_1", 10.0, speed=10.0, pedestrians=[ped("p", x, y, speed=1.2, heading=(h + 90) % 360)]))
    assert v[I["current_leader_gap"]] == pytest.approx(19.7 / 50.0, abs=0.003)
    assert v[I["current_leader_relative_speed"]] == pytest.approx(-10.0 / VMAX, abs=0.01)


def test_crossing_vehicle_is_obstacle():
    x, y, h = pose("R69_1", 35.0)
    hc = (h + 90) % 360
    fx, fy = x + math.sin(math.radians(hc)) * 2.5, y + math.cos(math.radians(hc)) * 2.5   # 앞 범퍼(몸체 중심이 경로 위)
    cross = {"id": "c", "x": fx, "y": fy, "heading": hc, "speed": 6.0, "length": 5.0, "width": 2.0,
             "lane_id": ":J_other_0", "lane_position": 3.0, "road_id": ":J_other", "lane_index": 0}
    v = o(bus_row("R69_1", 10.0, speed=8.0, vehicles=[cross]))
    assert v[I["current_leader_gap"]] == pytest.approx((25.0 - 1.0) / 50.0, abs=0.02)
    assert v[I["current_leader_relative_speed"]] == pytest.approx(-8.0 / VMAX, abs=0.02)


def test_follower_gap_from_rear():
    v = o(bus_row("R69_1", 30.0, vehicles=[car("f", "R69_1", 10.0, speed=7.0)]))
    assert v[I["current_follower_gap"]] == pytest.approx(-(30.0 - 12.0 - 10.0) / 50.0, abs=1e-4)
    assert v[I["current_follower_relative_speed"]] == pytest.approx(2.0 / VMAX, abs=1e-3)


def test_overlapping_leader_gap_zero():
    v = o(bus_row("R69_1", 10.0, vehicles=[car("a", "R69_1", 12.0)]))
    assert v[I["current_leader_gap"]] == 0.0


def test_density_counts_objects():
    x, y, h = pose("R69_1", 40.0)
    row = bus_row("R69_1", 10.0, vehicles=[car("a", "R69_1", 25.0)], pedestrians=[ped("p", x, y)])
    v = o(row)
    from pdm_data.lane_paths import forward_segments, path_length
    usable = path_length(forward_segments(NET, E, "R69_1", 10.0, E.index("R69"), 50.0))
    assert v[I["current_density"]] == pytest.approx(min(1.0, 2.0 / (usable / 7.5)), abs=1e-4)


def test_route_connectivity_left_turn_J1():
    pos = NET.getEdge("R54").getLength() - 100.0
    v = o(bus_row("R54_2", pos))
    assert v[I["right1_route_connectivity"]] == pytest.approx(100.0 / 200.0, abs=1e-3)   # R54_1은 R65로만 간다
    assert v[I["current_route_connectivity"]] > 0.5 + 0.005


def test_stop_connectivity_and_red():
    k = E.index("R112.112")
    end = CTX.service_stops[STOP_S4][1]
    v = o(bus_row("R112.112_2", 50.0), state(STOP_S4))
    want = (end - (RM.start[k] + 50.0)) / 200.0
    assert v[I["right1_route_connectivity"]] == pytest.approx(1.0)                         # 정류장 차로 1번
    assert v[I["current_route_connectivity"]] == pytest.approx(want, abs=1e-3)
    assert v[I["stop_point_distance"]] == pytest.approx(want, abs=1e-3)
    assert v[I["signal_state"]] == 0.0


def test_stop_green_after_service():
    v = o(bus_row("R112.112_1", 97.3, speed=0.0), state(STOP_S4 + 1))
    assert v[I["signal_state"]] == 1.0 and v[I["stop_point_distance"]] == pytest.approx(0.0, abs=2e-3)
    k = E.index("R112.112")
    passed = o(bus_row("R112.112_1", 98.0), state(STOP_S4 + 1))     # 정류장 끝을 지남: 그 정류장 후보는 없다
    s6 = next(at for at, _ in CTX.s6_crossings if at >= RM.start[k] + 98.0)
    assert passed[I["stop_point_distance"]] == pytest.approx((s6 - RM.start[k] - 98.0) / 200.0, abs=1e-3)
    assert passed[I["signal_state"]] == 1.0


def test_nearest_point_wins():
    k = E.index("R112.112")
    base = dict(lane_id="R112.112_1", pos=0.0)
    red = o(bus_row(base["lane_id"], base["pos"], tls=[{"id": "T", "link_index": 0, "distance": 30.0, "state": "r"}]),
            state(STOP_S4))
    assert red[I["stop_point_distance"]] == pytest.approx(30.0 / 200.0) and red[I["signal_state"]] == 0.0
    green = o(bus_row(base["lane_id"], base["pos"], tls=[{"id": "T", "link_index": 0, "distance": 10.0, "state": "G"}]),
              state(STOP_S4))
    assert green[I["stop_point_distance"]] == pytest.approx(10.0 / 200.0) and green[I["signal_state"]] == 1.0
    stop_only = o(bus_row(base["lane_id"], base["pos"]), state(STOP_S4))
    assert stop_only[I["stop_point_distance"]] == pytest.approx(
        (CTX.service_stops[STOP_S4][1] - RM.start[k]) / 200.0, abs=1e-3) and stop_only[I["signal_state"]] == 0.0


def test_internal_lane_only_current():
    ln = NET.getLane("R69_1")
    via = next(c.getViaLaneID() for c in ln.getOutgoing() if c.getViaLaneID())
    row = bus_row("R69_1", ln.getLength() - 1.0)
    x, y, h = pose(via, 1.0)
    row["ego"].update(road_id=via.rsplit("_", 1)[0], lane_id=via, lane_index=int(via.rsplit("_", 1)[1]),
                      lane_position=1.0, x=x, y=y, heading=h)
    v = o(row)
    for lane in ("left2", "left1", "right1", "right2"):
        assert v[I[f"{lane}_route_connectivity"]] == -1.0
    assert v[I["current_route_connectivity"]] >= 0.0


def test_after_last_stop():
    v = o(bus_row("R54_2", 10.0), state(len(CTX.service_stops)))
    assert v.shape == (33,) and np.isfinite(v).all()
