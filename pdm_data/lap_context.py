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
    s6_crossings: list = None    # S6 대상: 노선 위 횡단보도 중 신호 없는 교차로의 것 (노선 위치, 횡단보도 id)


def _crossings(net_path):
    return frozenset(e.get("id") for e in ET.parse(net_path).getroot().iter("edge") if e.get("function") == "crossing")


def build_lap_context(net_path: str, lap_edges: list, bus_stops: list, vmax: float, lane_allowed: dict) -> LapContext:
    rm = RouteMap(net_path, "lap", list(lap_edges), bus_stops, "bus")
    service = [s for s in rm.stops if s[2] in STOP_IDS]
    return LapContext(list(lap_edges), rm, float(vmax), dict(lane_allowed), _crossings(net_path), service, net_path,
                      _unsignalized(net_path, rm.crossings))


def _unsignalized(net_path, crossings):
    """신호 교차로의 횡단보도는 보행자 신호가 순서를 정하므로 S6(신호 없는 횡단보도)이 아니다."""
    from traffic.alleys import read_net
    from traffic.signal_areas import is_signalized
    net = read_net(net_path)
    out = []
    for at, cid in crossings:
        node = cid[1:].rsplit("_c", 1)[0]
        if net.hasNode(node) and not is_signalized(net.getNode(node)):
            out.append((at, cid))
    return out


def load_lap_context(run_dir: str) -> LapContext:
    """수집 폴더의 metadata.json과 assets/network.net.xml로 문맥을 다시 만든다."""
    import json
    import os
    with open(os.path.join(run_dir, "metadata.json"), encoding="utf-8") as f:
        r = json.load(f)["route"]
    return build_lap_context(os.path.join(run_dir, "assets", "network.net.xml"), r["lap_edges"], r["bus_stops"],
                             r["vmax"], r["lane_allowed"])


def lap_route_index(road: str, last_road, lap_edges: list) -> int:
    """한 바퀴 노선에서 도로의 번호. 교차로 내부 차로면 직전 일반 도로(SUMO getRouteIndex와 같은 규칙)."""
    return lap_edges.index(last_road if road.startswith(":") else road)


def lane_usable(ctx: LapContext, lane_id: str, vclass: str = "bus") -> bool:
    """env/sumo_env.py의 _usable과 같은 규칙. 표에 없는 차로는 쓸 수 없다고 본다."""
    if lane_id not in ctx.lane_allowed:
        return False
    allowed = ctx.lane_allowed[lane_id]
    return not allowed or vclass in allowed
