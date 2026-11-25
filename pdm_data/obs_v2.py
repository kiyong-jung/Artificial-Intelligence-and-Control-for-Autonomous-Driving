"""관측 v2 (docs/observation_features_v2.md): Ego 16 + HV 8 x 12 + 보행자 6 x 9 = 166차원.

원시 행(pdm_data/recorder.py)과 한 바퀴 노선 문맥으로 계산한다. 시뮬레이터만 아는 값(운전자 성향, 주입 여부, 보행자 다음 도로,
다른 차의 경로)과 SUMO의 판단(couldChangeLane)은 쓰지 않는다. 거리만 판단 범위에서 1로 포화하고 나머지는 자르지 않는다.

좌표는 ego 기준(x 진행 방향 +, y 왼쪽 +). 상대 방향은 (ego 방향 - 대상 방향)이라 SUMO 각도(시계 방향)에서 왼쪽이 +다.
"""
from __future__ import annotations

import functools
import math
from dataclasses import dataclass

import numpy as np
from sumolib.net import readNet

from .lap_context import LapContext, lane_usable
from .schema import DETECT_RADIUS_M, SERVICE_SECONDS

VEHICLE_SLOTS, PEDESTRIAN_SLOTS = 8, 6
EGO_DIM, VEH_DIM, PED_DIM = 16, 12, 9
OBS_V2_DIM = EGO_DIM + VEHICLE_SLOTS * VEH_DIM + PEDESTRIAN_SLOTS * PED_DIM
HORIZON_M = 200.0           # 거리 특징 포화 범위
ACC_POS, ACC_NEG = 1.5, 9.0  # 버스 vType accel, emergencyDecel
LANE_SPAN = 5.0             # 노선 최대 6차로 -> 차로 차이 최대 5
PED_SPEED = 2.0
CROSSING_RANGE_M = 10.0
STOP_PLAN_LEAD_M = 30.0     # 정류장 시작에서 이만큼 안쪽에 끝나는 S3/STOP 창이 그 정류장의 차로 창

EGO_NAMES = ("speed", "acceleration", "prev_accel_cmd", "prev_lane_cmd", "left_lane_exists", "right_lane_exists",
             "cont_current", "cont_left", "cont_right", "lane_deadline_distance", "next_turn_direction",
             "stop_distance", "lanes_to_stop", "dwell_remaining", "stop_point_distance", "signal_state")
VEH_NAMES = ("presence", "x", "y", "vx", "vy", "cos", "sin", "inv_ttc", "lane_diff", "other_road", "blinker_left",
             "blinker_right")
PED_NAMES = ("presence", "x", "y", "vx", "vy", "cos", "sin", "on_crossing", "crossing_distance")
FEATURE_NAMES = (list(EGO_NAMES) + [f"veh{k}_{n}" for k in range(VEHICLE_SLOTS) for n in VEH_NAMES]
                 + [f"ped{k}_{n}" for k in range(PEDESTRIAN_SLOTS) for n in PED_NAMES])
LAYOUT = {"ego": [0, EGO_DIM], "vehicles": [EGO_DIM, EGO_DIM + VEHICLE_SLOTS * VEH_DIM],
          "pedestrians": [EGO_DIM + VEHICLE_SLOTS * VEH_DIM, OBS_V2_DIM]}
assert len(FEATURE_NAMES) == OBS_V2_DIM == 166


@functools.lru_cache(maxsize=4)
def _net(net_path):
    return readNet(net_path, withInternal=True, withPedestrianConnections=True)


@functools.lru_cache(maxsize=4)
def _crossing_shapes(net_path):
    """횡단보도마다 (중심선 점 목록, 반폭)."""
    out = []
    for e in _net(net_path).getEdges(withInternal=True):
        if e.getFunction() == "crossing":
            lane = e.getLanes()[0]
            out.append((lane.getShape(), lane.getWidth() / 2.0))
    return out


def _seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    n = ax * ax + ay * ay
    t = 0.0 if n == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / n))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def crossing_distance(net_path, x, y) -> float:
    """점에서 가장 가까운 횡단보도 가장자리까지 거리(안이면 0)."""
    best = math.inf
    for shape, half in _crossing_shapes(net_path):
        d = min(_seg_dist((x, y), shape[i], shape[i + 1]) for i in range(len(shape) - 1)) - half
        best = min(best, max(d, 0.0))
    return best


@dataclass
class Plan:
    kind: str
    edge_index: int
    end: float          # 창 끝 노선 위치
    start: float        # 창 시작 노선 위치
    ok: frozenset       # 창 끝 도로의 목표 차로 번호


class V2Context:
    """한 run의 고정 정보: 노선 문맥, 차로 변경 창(metadata route.lane_plans), 연결 기반 도달 차로 캐시."""

    def __init__(self, ctx: LapContext, lane_plans=()):
        self.ctx = ctx
        self.net = _net(ctx.net_path)
        rm = ctx.route_map
        self.plans = []
        for p in lane_plans or ():
            if p["edge"] not in ctx.lap_edges:
                continue
            j = ctx.lap_edges.index(p["edge"])
            end = rm.start[j] + float(p["end_pos"])
            ok = frozenset(p["ok_lanes"]) if p.get("ok_lanes") else frozenset({int(p["target_lane"])})
            self.plans.append(Plan(p["kind"], j, end, end - float(p["length"]), ok))
        self.plans.sort(key=lambda q: q.end)
        self._reach = {}

    def reach(self, j: int, lanes: frozenset, k: int) -> frozenset:
        """노선 k번 도로의 차로 중 연결을 따라 j번 도로의 lanes에 닿는 차로 번호(k <= j)."""
        key = (j, lanes, k)
        if key not in self._reach:
            edges, out = self.ctx.lap_edges, lanes
            for m in range(j - 1, k - 1, -1):
                a, b = self.net.getEdge(edges[m]), self.net.getEdge(edges[m + 1])
                out = frozenset(c.getFromLane().getIndex() for c in a.getConnections(b)
                                if c.getToLane().getIndex() in out)
            self._reach[key] = out
        return self._reach[key]

    def active_plan(self, pos):
        if pos is None:
            return None
        hits = [p for p in self.plans if p.start <= pos <= p.end]
        return hits[0] if hits else None


def _rel(ego, x, y):
    h = math.radians(ego["heading"])
    hx, hy = math.sin(h), math.cos(h)
    dx, dy = x - ego["x"], y - ego["y"]
    return dx * hx + dy * hy, -dx * hy + dy * hx, hx, hy


def _vel(speed, heading):
    a = math.radians(heading)
    return speed * math.sin(a), speed * math.cos(a)


def _to_ego(vx, vy, hx, hy):
    return vx * hx + vy * hy, -vx * hy + vy * hx


def _rel_angle(ego_heading, other_heading):
    r = math.radians(ego_heading - other_heading)
    return math.cos(r), math.sin(r)


def _signal_value(state: str) -> float:
    if state in "rRu":
        return 0.0
    if state in "yY":
        return 0.5
    return 1.0


def _ego(row, v2: V2Context, tracker_state, prev_action):
    ctx, e = v2.ctx, row["ego"]
    rm, pos, road = ctx.route_map, e["lap_position"], e["road_id"]
    internal = road.startswith(":")
    k = e.get("lap_route_index")
    idx, n = e["lane_index"], e["lane_count"]
    a = e["acceleration"]
    pa, pl = prev_action if prev_action is not None else (0.0, 0)

    def usable(i):
        return not internal and 0 <= i < n and lane_usable(ctx, f"{road}_{i}")

    left, right = float(usable(idx + 1)), float(usable(idx - 1))
    plan = v2.active_plan(pos)
    if internal or k is None or pos is None:
        cont = (1.0, 0.0, 0.0)
    else:
        if plan is not None:
            good = v2.reach(plan.edge_index, plan.ok, k)
        else:
            good = frozenset(int(lid.rsplit("_", 1)[1]) for lid in rm.target_lane_ids[k])
        cont = (float(idx in good), float(usable(idx + 1) and idx + 1 in good), float(usable(idx - 1) and idx - 1 in good))
    deadline = 1.0 if plan is None else min(1.0, (plan.end - pos) / HORIZON_M)

    turn = 0.0
    if pos is not None:
        nav = rm.ahead(pos)
        if nav["next_turn"] is not None and nav["distance_to_turn"] <= HORIZON_M:
            turn = -1.0 if nav["next_turn"] == "right" else 1.0

    stops, si = ctx.service_stops, tracker_state["tracker_index"]
    stop_d, lanes_to, dwell = 1.0, 0.0, 0.0
    if si < len(stops) and pos is not None:
        start, end, _, stop_lane = stops[si]
        gap = max(0.0, start - pos)
        stop_d = min(1.0, gap / HORIZON_M)
        if gap <= HORIZON_M and not internal and k is not None:
            s_edge, s_idx = stop_lane.rsplit("_", 1)
            j = ctx.lap_edges.index(s_edge)
            if k <= j:
                good = v2.reach(j, frozenset({int(s_idx)}), k)
                if good:
                    lanes_to = min((g - idx for g in good), key=lambda d: (abs(d), -d)) / LANE_SPAN
        if start <= pos <= end and e["lane_id"] == stop_lane and e["speed"] <= 0.1:
            dwell = max(0.0, SERVICE_SECONDS - tracker_state["dwell"]) / SERVICE_SECONDS

    point, signal = math.inf, 1.0
    if row["tls"]:
        t = row["tls"][0]
        point, signal = max(0.0, t["distance"]), _signal_value(t["state"])
    if pos is not None:
        s6 = next((at - pos for at, _ in (ctx.s6_crossings or []) if at >= pos), None)
        if s6 is not None and s6 < point:
            point, signal = s6, 1.0
    point_d = 1.0 if math.isinf(point) else min(1.0, point / HORIZON_M)

    return [e["speed"] / ctx.vmax, a / ACC_POS if a >= 0 else a / ACC_NEG, float(pa), float(pl) / 2.0, left, right,
            *cont, deadline, turn, stop_d, lanes_to, dwell, point_d, signal]


def _near(ego, objs, count):
    items = []
    for o in objs:
        d2 = (o["x"] - ego["x"]) ** 2 + (o["y"] - ego["y"]) ** 2
        if d2 <= DETECT_RADIUS_M ** 2:
            items.append((d2, o["id"], o))
    items.sort(key=lambda it: (it[0], it[1]))
    return [it[2] for it in items[:count]]


def _vehicles(row, vmax):
    e = row["ego"]
    ego_internal = e["road_id"].startswith(":")
    evx, evy = _vel(e["speed"], e["heading"])
    out = []
    for o in _near(e, row["vehicles"], VEHICLE_SLOTS):
        x, y, hx, hy = _rel(e, o["x"], o["y"])
        ovx, ovy = _vel(o["speed"], o["heading"])
        rvx, rvy = ovx - evx, ovy - evy
        vx, vy = _to_ego(rvx, rvy, hx, hy)
        c, s = _rel_angle(e["heading"], o["heading"])
        dx, dy = o["x"] - e["x"], o["y"] - e["y"]
        d = math.hypot(dx, dy)
        closing = -(dx * rvx + dy * rvy) / d if d > 1e-9 else 0.0
        q = max(0.0, closing) / max(d, 0.1)
        same = o.get("road_id") == e["road_id"] and not ego_internal and o.get("lane_index") is not None
        sig = int(o.get("signals") or 0)
        out += [1.0, x / DETECT_RADIUS_M, y / DETECT_RADIUS_M, vx / (2 * vmax), vy / (2 * vmax), c, s, q / (1.0 + q),
                (o["lane_index"] - e["lane_index"]) / LANE_SPAN if same else 0.0, 0.0 if same else 1.0,
                float(bool(sig & 2)), float(bool(sig & 1))]
    return out + [0.0] * VEH_DIM * (VEHICLE_SLOTS - len(out) // VEH_DIM)


def _pedestrians(row, net_path):
    e = row["ego"]
    out = []
    for p in _near(e, row["pedestrians"], PEDESTRIAN_SLOTS):
        x, y, hx, hy = _rel(e, p["x"], p["y"])
        vx, vy = _to_ego(*_vel(p["speed"], p["heading"]), hx, hy)
        c, s = _rel_angle(e["heading"], p["heading"])
        on = bool(p.get("on_crossing"))
        cd = 0.0 if on else min(1.0, crossing_distance(net_path, p["x"], p["y"]) / CROSSING_RANGE_M)
        out += [1.0, x / DETECT_RADIUS_M, y / DETECT_RADIUS_M, vx / PED_SPEED, vy / PED_SPEED, c, s, float(on), cd]
    return out + [0.0] * PED_DIM * (PEDESTRIAN_SLOTS - len(out) // PED_DIM)


def obs_v2(row: dict, v2: V2Context, tracker_state: dict, prev_action=None) -> np.ndarray:
    """prev_action = 직전 행의 (가감속 라벨, 5값 차로 라벨) 또는 None(에피소드 첫 행)."""
    v = _ego(row, v2, tracker_state, prev_action) + _vehicles(row, v2.ctx.vmax) + _pedestrians(row, v2.ctx.net_path)
    out = np.asarray(v, dtype=np.float32)
    assert out.shape == (OBS_V2_DIM,) and np.isfinite(out).all()
    return out
