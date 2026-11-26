"""노선을 따라가는 차로 구간과 물체 위치 판정(관측 33차원, docs/superpowers/specs/2026-11-26-obs33-mlp-bc-design.md 5.1~5.2절).

env/sumo_env.py의 _forward_segments와 같은 규칙을 TraCI 없이 sumolib 도로망으로 계산한다.
  - 다음 노선 도로(lap_edges[(k + 1) % n])로 가는 연결 중 차로 번호 차이가 가장 작은 것(같으면 차로 id 순)을 따라간다.
  - 연결에 내부 차로(via)가 있으면 그것을 먼저 넣는다. 내부 차로에서는 노선 번호 k를 올리지 않는다(SUMO getRouteIndex와 같음).
  - 노선으로 이어지는 연결이 없으면 거기서 멈춘다. 이것이 노선 연결성의 "끊김"이다.
구간(Segment)의 start/end는 그 차로 위 위치(m)다. 앞 구간의 offset은 버스 앞 범퍼에서 구간 시작(start)까지, 뒤 구간의 offset은
버스 앞 범퍼에서 구간 끝(end)까지 경로를 따라 잰 거리다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from sumolib.geomhelper import polygonOffsetAndDistanceToPoint

PED_RADIUS_M = 0.3
SAMPLE_M = 0.5           # 차량 몸체 점 간격


@dataclass(frozen=True)
class Segment:
    lane_id: str
    start: float
    end: float
    offset: float


_cache = {}


def _lane(net, lane_id):
    key = (id(net), lane_id)
    if key not in _cache:
        ln = net.getLane(lane_id)
        _cache[key] = {"length": ln.getLength(), "shape": ln.getShape(), "width": ln.getWidth(),
                       "out": [(c.getToLane().getID(), c.getToLane().getEdge().getID(), c.getViaLaneID() or None)
                               for c in ln.getOutgoing()]}
    return _cache[key]


def _index(lane_id):
    try:
        return int(lane_id.rsplit("_", 1)[1])
    except (IndexError, ValueError):
        return 0


def path_length(segments) -> float:
    return sum(s.end - s.start for s in segments)


def forward_segments(net, lap_edges, lane_id: str, lane_pos: float, lap_route_index: int, horizon: float) -> list:
    n, k, lane, pos = len(lap_edges), lap_route_index, lane_id, lane_pos
    segs, dist, seen = [], 0.0, set()
    while dist < horizon - 1e-9 and (lane, k) not in seen:
        seen.add((lane, k))
        info = _lane(net, lane)
        start = min(max(pos, 0.0), info["length"])
        end = min(info["length"], start + horizon - dist)
        segs.append(Segment(lane, start, end, dist))
        dist += end - start
        if dist >= horizon - 1e-9:
            break
        target = lap_edges[(k + 1) % n]
        cands = [(to, via) for to, edge, via in info["out"] if edge == target]
        if not cands:
            break
        cur = _index(lane)
        to, via = min(cands, key=lambda c: (abs(_index(c[0]) - cur), c[0]))
        if via:
            lane = via
        else:
            lane, k = to, (k + 1) % n
        pos = 0.0
    return segs


def _predecessor(net, lap_edges, lane_id, k):
    """(앞 차로 id, 그 차로의 노선 번호) 또는 None. 내부 차로의 노선 번호는 직전 일반 도로다."""
    n = len(lap_edges)
    if lane_id.startswith(":"):
        for ln in net.getEdge(lap_edges[k]).getLanes():
            if any(via == lane_id for _, _, via in _lane(net, ln.getID())["out"]):
                return ln.getID(), k
        return None
    pk = (k - 1) % n
    cands = []
    for ln in net.getEdge(lap_edges[pk]).getLanes():
        for to, _, via in _lane(net, ln.getID())["out"]:
            if to == lane_id:
                cands.append((ln.getID(), via))
    if not cands:
        return None
    cur = _index(lane_id)
    frm, via = min(cands, key=lambda c: (abs(_index(c[0]) - cur), c[0]))
    return (via, pk) if via else (frm, pk)


def backward_segments(net, lap_edges, lane_id: str, lane_pos: float, lap_route_index: int, horizon: float) -> list:
    k, lane, end = lap_route_index, lane_id, lane_pos
    segs, dist, seen = [], 0.0, set()
    while dist < horizon - 1e-9 and (lane, k) not in seen:
        seen.add((lane, k))
        info = _lane(net, lane)
        end = min(max(end, 0.0), info["length"])
        start = max(0.0, end - (horizon - dist))
        segs.append(Segment(lane, start, end, dist))
        dist += end - start
        if dist >= horizon - 1e-9:
            break
        prev = _predecessor(net, lap_edges, lane, k)
        if prev is None:
            break
        lane, k = prev
        end = _lane(net, lane)["length"]
    return segs


def _points(obj):
    """(몸체 점 목록, 반지름). 차량은 앞 범퍼 위치·방향·길이·폭의 사각형, 보행자는 중심점."""
    x, y = float(obj["x"]), float(obj["y"])
    if obj.get("length") is None:
        return [(x, y)], PED_RADIUS_M
    h = math.radians(float(obj.get("heading", 0.0)))
    dx, dy = math.sin(h), math.cos(h)            # 진행 방향
    px, py = math.cos(h), -math.sin(h)           # 오른쪽
    ln, hw = float(obj["length"]), float(obj.get("width", 2.0)) / 2.0
    # 몸체가 좁은 차로 띠를 가로지를 수 있어 꼭짓점만으로는 놓친다: 길이 방향 0.5 m 간격, 폭 방향 세 줄로 뽑는다
    n = max(2, int(math.ceil(ln / SAMPLE_M)) + 1)
    pts = []
    for i in range(n):
        t = ln * i / (n - 1)
        cx, cy = x - dx * t, y - dy * t
        pts += [(cx, cy), (cx + px * hw, cy + py * hw), (cx - px * hw, cy - py * hw)]
    return pts, 0.0


def locate(obj: dict, segments: list, net, backward: bool = False):
    """물체 몸체가 걸친 경로 구간 (s_near, s_far)(m, 버스 앞 범퍼 기준 경로 거리). 겹치지 않으면 None."""
    lane_id = obj.get("lane_id")
    if lane_id and obj.get("lane_position") is not None and any(s.lane_id == lane_id for s in segments):
        hi = float(obj["lane_position"])
        lo = hi - float(obj.get("length") or 0.0)
        for s in segments:
            if s.lane_id != lane_id:
                continue
            if backward:
                if s.start < hi <= s.end + 1e-9:
                    return s.offset + (s.end - hi), s.offset + (s.end - lo)
            elif hi > s.start + 1e-9 and lo < s.end:
                return s.offset + (lo - s.start), s.offset + (hi - s.start)
        return None
    pts, r = _points(obj)
    hits = []
    for s in segments:
        info = _lane(net, s.lane_id)
        half = info["width"] / 2.0 + r
        for p in pts:
            off, d = polygonOffsetAndDistanceToPoint(p, info["shape"])
            if off < 0 or d > half or not (s.start - r <= off <= s.end + r):
                continue
            v = s.offset + (s.end - off) if backward else s.offset + (off - s.start)
            hits += [v - r, v + r]
    return (min(hits), max(hits)) if hits else None
