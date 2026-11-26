"""노선을 따라가는 차로 구간과 물체 위치 판정(pdm_data/lane_paths.py)."""
import math
import os
import sys

import pytest
from sumolib.geomhelper import positionAtShapeOffset

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_fakes import _ctx_args  # noqa: E402
from pdm_data.lane_paths import Segment, backward_segments, forward_segments, locate, path_length  # noqa: E402
from pdm_data.obs_v2 import _net  # noqa: E402

NET_PATH, EDGES, _, _ = _ctx_args()
NET = _net(NET_PATH)
E = list(EDGES)


def k_of(edge):
    return E.index(edge)


def length(edge):
    return NET.getEdge(edge).getLength()


def test_forward_follows_route_and_horizon():
    segs = forward_segments(NET, E, "R69_1", 10.0, k_of("R69"), 200.0)
    assert segs[0] == Segment("R69_1", 10.0, pytest.approx(length("R69")), 0.0)
    for a, b in zip(segs, segs[1:]):
        assert b.offset == pytest.approx(a.offset + a.end - a.start)
    assert path_length(segs) <= 200.0 + 1e-6
    assert any(s.lane_id.startswith("R69a_") for s in segs)        # 다음 노선 도로로 이어진다


def test_forward_stops_where_lane_leaves_route():
    pos = length("R54") - 100.0
    off = forward_segments(NET, E, "R54_1", pos, k_of("R54"), 200.0)   # R54_1은 R65로만 간다(노선 아님)
    assert path_length(off) == pytest.approx(100.0)
    on = forward_segments(NET, E, "R54_2", pos, k_of("R54"), 200.0)    # R54_2는 R42로 이어진다
    assert path_length(on) > 100.0 + 1.0


def test_forward_wraps_lap():
    assert E[-1] == "R42" and E[0] == "R68"
    segs = forward_segments(NET, E, "R42_4", length("R42") - 5.0, k_of("R42"), 100.0)
    assert any(s.lane_id.startswith("R68_") for s in segs)
    assert path_length(segs) == pytest.approx(100.0)


def test_backward_offsets():
    segs = backward_segments(NET, E, "R69_2", 30.0, k_of("R69"), 50.0)
    assert segs[0] == Segment("R69_2", 0.0, 30.0, 0.0)
    assert segs[1].offset == pytest.approx(30.0)
    assert path_length(segs) == pytest.approx(50.0)
    assert any(s.lane_id.startswith("R68_") or s.lane_id.startswith(":") for s in segs[1:])


def _center(lane_id, pos):
    return positionAtShapeOffset(NET.getLane(lane_id).getShape(), pos)


def test_locate_same_lane_vehicle():
    segs = forward_segments(NET, E, "R69_1", 10.0, k_of("R69"), 50.0)
    v = {"id": "v", "lane_id": "R69_1", "lane_position": 40.0, "length": 5.0, "width": 2.0, "x": 0, "y": 0,
         "heading": 0.0, "speed": 0.0}
    assert locate(v, segs, NET) == pytest.approx((25.0, 30.0))
    behind = dict(v, lane_position=8.0)                            # 앞 범퍼가 버스 앞 범퍼보다 뒤면 선행이 아니다
    assert locate(behind, segs, NET) is None


def test_locate_backward_same_lane_vehicle():
    segs = backward_segments(NET, E, "R69_1", 30.0, k_of("R69"), 62.0)
    v = {"id": "v", "lane_id": "R69_1", "lane_position": 10.0, "length": 5.0, "width": 2.0, "x": 0, "y": 0,
         "heading": 0.0, "speed": 0.0}
    assert locate(v, segs, NET, backward=True) == pytest.approx((20.0, 25.0))


def test_locate_pedestrian_geometry():
    segs = forward_segments(NET, E, "R69_1", 10.0, k_of("R69"), 50.0)
    x, y = _center("R69_1", 30.0)
    p = {"id": "p", "x": x, "y": y, "heading": 90.0, "speed": 1.2}
    near, far = locate(p, segs, NET)
    assert near == pytest.approx(19.7, abs=0.05) and far == pytest.approx(20.3, abs=0.05)
    shape = NET.getLane("R69_1").getShape()
    (x0, y0), (x1, y1) = positionAtShapeOffset(shape, 29.0), positionAtShapeOffset(shape, 31.0)
    nx, ny = -(y1 - y0) / 2.0, (x1 - x0) / 2.0                      # 진행 방향 왼쪽 단위 벡터
    off = NET.getLane("R69_1").getWidth() / 2 + 1.0
    far_ped = dict(p, x=x + nx * off, y=y + ny * off)
    assert locate(far_ped, segs, NET) is None
