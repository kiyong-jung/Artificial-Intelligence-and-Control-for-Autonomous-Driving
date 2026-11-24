"""오프라인 93차원 관측. env/bus_task.py의 BusTask.observation과 같은 식을 원시 행에서 계산한다(노선은 한 바퀴 노선 기준)."""
from __future__ import annotations

import functools
import math

import numpy as np
from sumolib.geomhelper import positionAtShapeOffset
from sumolib.net import readNet

from .lap_context import LapContext, lane_usable
from .schema import DETECT_RADIUS_M, SERVICE_SECONDS

OBS_OFFSETS = (2, 1, 0, -1, -2)
VEHICLE_SLOTS, PEDESTRIAN_SLOTS = 8, 6
NAV_SCALE = 200.0


@functools.lru_cache(maxsize=4)
def _net(net_path):
    return readNet(net_path)


def _drivable(ego, ctx, offset):
    edge = ego["road_id"]
    if edge.startswith(":"):
        return offset == 0
    index = ego["lane_index"] + offset
    return 0 <= index < ego["lane_count"] and lane_usable(ctx, f"{edge}_{index}")


def obs93(row: dict, ctx: LapContext, tracker_state: dict) -> np.ndarray:
    ego = row["ego"]
    vmax, vis = ctx.vmax, DETECT_RADIUS_M
    ex, ey, ev = ego["x"], ego["y"], ego["speed"]
    h = math.radians(ego["heading"])
    hx, hy = math.sin(h), math.cos(h)

    def relative(x, y, speed=0.0, angle=0.0):
        a = math.radians(angle)
        dx, dy = x - ex, y - ey
        vx, vy = speed * math.sin(a) - ev * hx, speed * math.cos(a) - ev * hy
        return [dx * hx + dy * hy, -dx * hy + dy * hx, vx * hx + vy * hy, -vx * hy + vy * hx]

    obs = [ev / vmax] + [float(_drivable(ego, ctx, k)) for k in OBS_OFFSETS]
    for objs, count in ((row["vehicles"], VEHICLE_SLOTS), (row["pedestrians"], PEDESTRIAN_SLOTS)):
        items = []
        for o in objs:
            d2 = (o["x"] - ex) ** 2 + (o["y"] - ey) ** 2
            if d2 <= vis ** 2:
                r = relative(o["x"], o["y"], o["speed"], o["heading"])
                items.append((d2, o["id"], [1.0, r[0] / vis, r[1] / vis, r[2] / vmax, r[3] / vmax]))
        items.sort(key=lambda it: (it[0], it[1]))
        obs.extend(v for it in items[:count] for v in it[2])
        obs.extend([0.0] * 5 * max(0, count - len(items)))
    position = ego["lap_position"]
    nav = ctx.route_map.ahead(position)
    obs += [float(nav["next_turn"] == k) for k in ("left", "right", "turnaround")]
    obs += [(nav["distance_to_turn"] if nav["distance_to_turn"] is not None else NAV_SCALE) / NAV_SCALE,
            nav["distance_to_destination"] / ctx.route_map.length]
    tls = row["tls"][0] if row["tls"] else None
    if tls and tls["distance"] <= vis:
        ch = tls["state"].lower()
        obs += [1.0, tls["distance"] / vis, float(ch == "r"), float(ch == "y"), float(ch == "g"), float(ch not in "ryg")]
    else:
        obs += [0.0] * 6
    stops, idx = ctx.service_stops, tracker_state["tracker_index"]
    if idx < len(stops):
        start, end, _, lane = stops[idx]
        edge = lane.rsplit("_", 1)[0]
        shape = _net(ctx.net_path).getLane(lane).getShape()
        x, y = positionAtShapeOffset(shape, (start + end) / 2 - ctx.route_map.start[ctx.route_map.edge_ids.index(edge)])
        r = relative(x, y)
        obs += [1.0, r[0] / NAV_SCALE, r[1] / NAV_SCALE, tracker_state["dwell"] / SERVICE_SECONDS, idx / len(stops),
                (start - position) / NAV_SCALE]
    else:
        obs += [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    assert len(obs) == 93
    return np.clip(np.asarray(obs, dtype=np.float32), -1, 1)
