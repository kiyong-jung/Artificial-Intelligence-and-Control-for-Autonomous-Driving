"""관측 33차원(docs/superpowers/specs/2026-11-26-obs33-mlp-bc-design.md 5절).

env/sumo_env.py의 31차원과 같은 자리·기본값에 정지 지점 거리와 신호를 더한다. 원시 행(pdm_data/recorder.py)과 도로망만으로 계산해
전처리와 closed-loop 평가가 같은 함수를 쓴다. 시뮬레이터만 아는 값과 SUMO 판단, metadata lane_plans는 쓰지 않는다.

    [0]        속도 / vmax
    [1+4k..4+4k] 차로 k(좌2, 좌1, 현재, 우1, 우2)의 선행 간격/W, 선행 상대속도/vmax, -후행 간격/W, 후행 상대속도/vmax
    [21..25]   연결성: min(노선 연결 길이/H, 정류장 규칙). 없는 차로 -1
    [26..30]   밀도: 앞 W 안 물체 수 / (쓸 수 있는 길이 / 7.5 m). 없는 차로 1
    [31]       가장 가까운 정지 지점(신호 정지선, 신호 없는 횡단보도, 정류장 끝)까지 거리 / H
    [32]       그 지점의 신호(적 0, 황 0.5, 녹 1). 정류장은 8초 정차 전 0, 후 1
물체는 차량과 보행자를 가리지 않는다. 상대속도 = (물체 속도 벡터 · 버스 진행 방향) - 버스 속도.
"""
from __future__ import annotations

import math

import numpy as np

from env.data_schema import OBSERVATION_FEATURES

from .lane_paths import backward_segments, forward_segments, locate, path_length
from .lap_context import LapContext, lane_usable
from .obs_v2 import V2Context, _net, _signal_value

OBS33_DIM = 33
FEATURE_NAMES_33 = list(OBSERVATION_FEATURES) + ["stop_point_distance", "signal_state"]
assert len(FEATURE_NAMES_33) == OBS33_DIM

W_M = 50.0                 # 물체 범위
H_M = 200.0                # 연결성·정지 지점 범위
DENSITY_VEH_M = 7.5
BUS_LENGTH_M = 12.0
OFFSETS = (2, 1, 0, -1, -2)   # 좌2, 좌1, 현재, 우1, 우2 (+ 왼쪽)
NEAR_M = W_M + BUS_LENGTH_M + 20.0   # 이 거리 밖 물체는 판정하지 않는다
BUS = "bus0"


class Obs33Context:
    def __init__(self, ctx: LapContext):
        self.ctx = ctx
        self.net = _net(ctx.net_path)
        self.reach = V2Context(ctx).reach


def _velocity(o):
    a = math.radians(float(o.get("heading", 0.0)))
    s = float(o.get("speed", 0.0))
    return s * math.sin(a), s * math.cos(a)


def _clip(v, lo=-1.0, hi=1.0):
    return float(min(hi, max(lo, v)))


def _lane_ids(row, c):
    """offset마다 (차로 id 또는 None)."""
    e = row["ego"]
    road, idx = e["road_id"], e["lane_index"]
    if road.startswith(":"):
        return [e["lane_id"] if off == 0 else None for off in OFFSETS]
    n = e.get("lane_count") or c.net.getEdge(road).getLaneNumber()
    out = []
    for off in OFFSETS:
        i = idx + off
        lid = f"{road}_{i}"
        out.append(lid if 0 <= i < n and lane_usable(c.ctx, lid) else None)
    return out


def _stop_connectivity(row, c, tracker_state, lane_id, segs):
    """정류장 규칙 값(5.3절). 적용하지 않으면 None."""
    ctx, e = c.ctx, row["ego"]
    pos, k, si = e.get("lap_position"), e.get("lap_route_index"), tracker_state["tracker_index"]
    if pos is None or k is None or si >= len(ctx.service_stops):
        return None
    _, end, _, stop_lane = ctx.service_stops[si]
    if end - pos > H_M:
        return None
    s_edge, s_idx = stop_lane.rsplit("_", 1)
    j = ctx.lap_edges.index(s_edge)
    ratio = _clip((end - pos) / H_M, 0.0, 1.0)
    if lane_id.startswith(":"):                   # 교차로 안: 이어지는 다음 일반 차로로 판정
        nxt = next((s.lane_id for s in segs if not s.lane_id.startswith(":")), None)
        if nxt is None:
            return ratio
        lane_id, k = nxt, (k + 1) % len(ctx.lap_edges)
    if k > j:
        return None
    good = c.reach(j, frozenset({int(s_idx)}), k)
    return 1.0 if int(lane_id.rsplit("_", 1)[1]) in good else ratio


def _stop_point(row, c, tracker_state):
    ctx, e = c.ctx, row["ego"]
    pos, si = e.get("lap_position"), tracker_state["tracker_index"]
    cands = []
    if row.get("tls"):
        t = row["tls"][0]
        cands.append((max(0.0, float(t["distance"])), _signal_value(t["state"])))
    if pos is not None:
        s6 = next((at - pos for at, _ in (ctx.s6_crossings or []) if at >= pos), None)
        if s6 is not None:
            cands.append((s6, 1.0))
        stops = ctx.service_stops
        if si < len(stops) and stops[si][1] >= pos:
            cands.append((stops[si][1] - pos, 0.0))
        if 1 <= si <= len(stops) and pos <= stops[si - 1][1] + 1e-6:
            cands.append((max(0.0, stops[si - 1][1] - pos), 1.0))
    cands = [x for x in cands if x[0] <= H_M]
    if not cands:
        return 1.0, 1.0
    d, sig = min(cands, key=lambda x: x[0])
    return _clip(d / H_M, 0.0, 1.0), sig


def obs33(row: dict, c: Obs33Context, tracker_state: dict) -> np.ndarray:
    ctx, e = c.ctx, row["ego"]
    vmax = ctx.vmax
    v_ego = float(e["speed"])
    h = math.radians(float(e["heading"]))
    hx, hy = math.sin(h), math.cos(h)
    k = e.get("lap_route_index")
    objs = [o for o in list(row.get("vehicles") or []) + list(row.get("pedestrians") or [])
            if o.get("id") != BUS and math.hypot(o["x"] - e["x"], o["y"] - e["y"]) <= NEAR_M]

    def rel_speed(o):
        vx, vy = _velocity(o)
        return _clip((vx * hx + vy * hy - v_ego) / vmax)

    blocks, conn, dens = [], [], []
    for lane_id in _lane_ids(row, c):
        if lane_id is None or k is None:
            blocks += [1.0, 1.0, -1.0, -1.0]
            conn.append(-1.0)
            dens.append(1.0)
            continue
        pos = float(e["lane_position"])
        fwd_w = forward_segments(c.net, ctx.lap_edges, lane_id, pos, k, W_M)
        back = backward_segments(c.net, ctx.lap_edges, lane_id, pos, k, W_M + BUS_LENGTH_M)
        lead, foll, count = None, None, 0
        for o in objs:
            hit = locate(o, fwd_w, c.net)
            if hit is not None and hit[0] <= W_M:
                count += 1
                if lead is None or hit[0] < lead[0]:
                    lead = (hit[0], o)
                continue
            hit = locate(o, back, c.net, backward=True)
            if hit is not None and hit[0] <= W_M + BUS_LENGTH_M:
                gap = max(0.0, hit[0] - BUS_LENGTH_M)
                if foll is None or gap < foll[0]:
                    foll = (gap, o)
        blocks += ([max(0.0, lead[0]) / W_M, rel_speed(lead[1])] if lead else [1.0, 1.0])
        blocks += ([-foll[0] / W_M, rel_speed(foll[1])] if foll else [-1.0, -1.0])
        fwd_h = forward_segments(c.net, ctx.lap_edges, lane_id, pos, k, H_M)
        value = _clip(path_length(fwd_h) / H_M, 0.0, 1.0)
        stop = _stop_connectivity(row, c, tracker_state, lane_id, fwd_h)
        conn.append(value if stop is None else min(value, stop))
        usable = path_length(fwd_w)
        dens.append(1.0 if usable <= 1e-6 else _clip(count / (usable / DENSITY_VEH_M), 0.0, 1.0))
    sp, sig = _stop_point(row, c, tracker_state)
    out = np.asarray([_clip(v_ego / vmax, 0.0, 1.0)] + blocks + conn + dens + [sp, sig], dtype=np.float32)
    assert out.shape == (OBS33_DIM,) and np.isfinite(out).all()
    return out
