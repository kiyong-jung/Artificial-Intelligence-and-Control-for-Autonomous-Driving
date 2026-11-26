"""실제 옆 이동 기준 3값 차로 라벨(A안, pdm_data/labels.lane_change_labels3)."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_fixtures import make_row  # noqa: E402
from pdm_data.labels import lane_change_labels3  # noqa: E402

WIDTH = lambda _lane: 3.2  # noqa: E731


def episode(lat_speed, change_at=None, lane=(1, 2), lat_pos=None, edge_change=False, **kw):
    rows = []
    for t, v in enumerate(lat_speed):
        lb = la = lane[0] if change_at is None or t < change_at else lane[1]
        eb = ea = "R69"
        if t == change_at:
            if edge_change:
                eb, ea, lb, la = "R69", "R69a", lane[0], lane[0]
            else:
                lb, la = lane
        r = make_row(t=t, lane_index=lb, lane_before=lb, lane_after=la, edge_before=eb, edge_after=ea, **kw)
        r["ego"]["lateral_speed"] = v
        if lat_pos is not None:
            r["ego"]["lateral_position"] = lat_pos[t]
        rows.append(r)
    return rows


def test_same_edge_change():
    assert lane_change_labels3(episode([0, 0, 0.6, 1.0, 1.0, 0.8, 0], change_at=3), WIDTH) == [0, 1, 1, 1, 0, 0, 0]


def test_right_change_negative():
    rows = episode([0, 0, -0.6, -1.0, -1.0, -0.8, 0], change_at=3, lane=(2, 1))
    assert lane_change_labels3(rows, WIDTH) == [0, -1, -1, -1, 0, 0, 0]


def test_edge_boundary_change():
    pos = [0, 0, 0.3, 1.5, -1.6, -1.0, -0.5]
    rows = episode([0, 0, 0.6, 1.0, 1.0, 0.8, 0], change_at=3, lat_pos=pos, edge_change=True)
    assert lane_change_labels3(rows, WIDTH) == [0, 1, 1, 1, 0, 0, 0]


def test_drift_without_change_is_zero():
    assert lane_change_labels3(episode([0, 0.3, 0.3, 0, 0, 0, 0]), WIDTH) == [0] * 7


def test_intent_only_is_zero():
    rows = episode([0] * 7, phase="signal", request_direction=1)
    assert lane_change_labels3(rows, WIDTH) == [0] * 7


def test_first_row_moving():
    assert lane_change_labels3(episode([0.6, 1.0, 0.5, 0, 0, 0, 0], change_at=1), WIDTH) == [1, 1, 0, 0, 0, 0, 0]


def test_final_row_without_window_is_zero():
    rows = episode([0, 0, 0.6, 1.0, 1.0, 0.8, 0], change_at=3)
    rows[-1]["window"] = None
    assert lane_change_labels3(rows, WIDTH)[-1] == 0
