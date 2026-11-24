"""한 바퀴 노선(R68 + bus_loop) 문맥: 노선 거리 지도, vmax, 차로 허용 표, 횡단보도, 서는 정류장."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

from env.route_map import RouteMap

from .schema import STOP_IDS


@dataclass
class LapContext:
    lap_edges: list
    route_map: RouteMap
    vmax: float
    lane_allowed: dict
    crossing_ids: frozenset
    service_stops: list          # (start, end, stop_id, lane_id), 노선 순서
    net_path: str


def _crossings(net_path):
    return frozenset(e.get("id") for e in ET.parse(net_path).getroot().iter("edge") if e.get("function") == "crossing")


def build_lap_context(net_path: str, lap_edges: list, bus_stops: list, vmax: float, lane_allowed: dict) -> LapContext:
    rm = RouteMap(net_path, "lap", list(lap_edges), bus_stops, "bus")
    service = [s for s in rm.stops if s[2] in STOP_IDS]
    return LapContext(list(lap_edges), rm, float(vmax), dict(lane_allowed), _crossings(net_path), service, net_path)


def lap_route_index(road: str, last_road, lap_edges: list) -> int:
    """한 바퀴 노선에서 도로의 번호. 교차로 내부 차로면 직전 일반 도로(SUMO getRouteIndex와 같은 규칙)."""
    return lap_edges.index(last_road if road.startswith(":") else road)


def lane_usable(ctx: LapContext, lane_id: str, vclass: str = "bus") -> bool:
    """env/sumo_env.py의 _usable과 같은 규칙. 표에 없는 차로는 쓸 수 없다고 본다."""
    if lane_id not in ctx.lane_allowed:
        return False
    allowed = ctx.lane_allowed[lane_id]
    return not allowed or vclass in allowed
