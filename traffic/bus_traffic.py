"""배경 버스·택시 투입(주변 차량). 시험용 버스(`bus_test`)와는 별개다.

두 가지를 투입한다. 대수는 모두 설정(`BusTrafficCfg`)이고 값은 자리표시다. 정류장에 서지 않는 배경 버스(통과 버스)는
투입하지 않는다. 버스는 전용 차로 노선뿐이다.

1. 전용 차로 버스(`bgbus_line*`): 버스 전용 차로를 지나며 정류장에 서서 승객을 태우고(정차) 출발한다.
   - 중앙 정류장 우회 차로(`B1`, `B2`, `B3a`→`B3b`, `B4a`→`B4b`) 노선 4개: 각각 그 차로의 정류장(`stop_0`~`stop_3`)에 선다.
   - 가로변 노선 2개: 가로변 정류장 `stop_4`(`R49a`)와 `stop_5`(`R44`)에 선다.
2. 택시(`bgtaxi`): 주변 승용차와 같은 출발·도착 짝을 같은 비율로 쓰며 지나간다.
3. 정류장 근처 택시 정차(S4, `curb_taxi_stops`): 도로망의 가로변 정류장마다(택시가 들어갈 수 없는 중앙 버스 전용 차로
   정류장은 빼고) 정류장 바로 앞(안 되면 곧게 들어오는 앞 도로 끝, 그것도 안 되면 정류장 뒤)에 차로를 막는 정차 구역을
   두고, 지도 경계에서 들어와 그곳에 섰다가 경계로 나가는 택시를 포아송 도착으로 흘린다. 시험용 버스의 노선·위치·시각과
   무관하다. 신호 정지선 앞 금지 구간에는 두지 않는다.
"""
import random
from dataclasses import dataclass, field

BUS_VTYPE = ('<vType id="{id}" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" tau="1.0" sigma="0.2" '
             'guiShape="bus" color="{color}"/>')
TAXI_VTYPE = ('<vType id="bgtaxi" vClass="taxi" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0.3" '
              'guiShape="taxi" color="255,200,0"/>')

# 전용 차로 버스 노선: (이름, 지나는 도로들, 정류장 id). 앞 4개는 중앙 정류장 우회 차로, 뒤 2개는 가로변 정류장이다.
CENTRAL_GROUPS = (("B1", ["B1"], "stop_0"), ("B2", ["B2"], "stop_1"),
                  ("B3", ["B3a", "B3b"], "stop_2"), ("B4", ["B4a", "B4b"], "stop_3"),
                  ("C1", ["R49a"], "stop_4"), ("C2", ["R44"], "stop_5"))


@dataclass
class BusTrafficCfg:
    lane_buses_per_h: float = 30.0     # 전용 차로 버스(정류장에 서고 출발) 시간당 대수. 노선 6개에 같게 나눈다
    taxis_per_h: float = 40.0          # 택시. 버스는 전용 차로 노선만 투입하고 배경 버스(통과 버스)는 넣지 않는다
    dwell: tuple = (12.0, 25.0)        # 정류장 정차 시간 U(lo, hi) [s] (흐름마다 하나)
    curb_taxis_per_h: float = 12.0     # 정류장 한 곳 근처에 서는 택시 시간당 대수(포아송). 0이면 없다
    curb_dwell: tuple = (15.0, 40.0)   # 그 택시의 정차 시간 U(lo, hi) [s] (정류장마다 하나)


def _chain(net, edges, vclass="bus"):
    path = []
    for a, b in zip(edges, edges[1:]):
        seg, _ = net.getOptimalPath(net.getEdge(a), net.getEdge(b), vClass=vclass)
        if not seg:
            return None
        ids = [e.getID() for e in seg]
        path.extend(ids if not path else ids[1:])
    return path


def central_route(net, via, entries, exits):
    """via(중앙 우회 차로 도로들)를 지나는 가장 짧은 버스 경로. (지도 경계 입구 → via → 지도 경계 출구).

    배경 버스는 도로가 끊긴 곳(양화로 양 끝)에서 생기고 사라진다. 예전에는 우회 차로 바로 다음 도로(J1 뒤 R41 등)에서 끝나
    시험용 버스가 지나는 J1 근처에서 버스가 사라졌다(순간이동으로 보였다).
    """
    from .presence import ends_at_boundary, starts_at_boundary
    heads = [a for a in entries if starts_at_boundary(net, a)]
    tails = [b for b in exits if ends_at_boundary(net, b)]
    mid = _chain(net, via) if len(via) > 1 else [via[0]]
    if not mid:
        return None
    best = None
    for a in heads:
        head = _chain(net, [a, via[0]])
        if not head:
            continue
        for b in tails:
            tail = _chain(net, [via[-1], b])
            if not tail:
                continue
            path = head + mid[1:] + tail[1:]
            length = sum(net.getEdge(e).getLength() for e in path)
            if best is None or length < best[0]:
                best = (length, path)
    return best[1] if best else None


def build_bus_traffic(net, flows, cfg=None, rng=None):
    """(vType·경로·흐름 XML 조각 목록, 요약 dict). flows: map.json의 흐름(from/to/vehs_per_hour), 합정역 지도 기준."""
    cfg = cfg or BusTrafficCfg()
    rng = rng or random.Random(0)
    parts = [BUS_VTYPE.format(id="bgbus", color="0,160,90"), TAXI_VTYPE]
    summary = {"lines": [], "taxi_od": []}
    entries = sorted({f["from"] for f in flows if net.hasEdge(f["from"])})
    exits = sorted({f["to"] for f in flows if net.hasEdge(f["to"])})
    central_rate = cfg.lane_buses_per_h / len(CENTRAL_GROUPS)
    dwell = lambda: round(rng.uniform(*cfg.dwell), 1)

    for name, via, stop in CENTRAL_GROUPS:
        if not all(net.hasEdge(e) for e in via):
            continue
        route = central_route(net, via, entries, exits)
        if not route:
            continue
        rid = f"r_bgbus_{name}"
        parts.append(f'<route id="{rid}" edges="{" ".join(route)}"/>')
        parts.append(f'<flow id="bgbus_line_{name}" type="bgbus" route="{rid}" begin="0" end="100000" '
                     f'period="exp({central_rate / 3600:.6f})" departLane="best" departSpeed="max">'
                     f'<stop busStop="{stop}" duration="{dwell()}"/></flow>')
        summary["lines"].append({"id": f"bgbus_line_{name}", "stop": stop, "per_h": round(central_rate, 1),
                                 "edges": route})

    # 택시는 주변 승용차와 같은 (문 -> 나가는 곳) 짝을 같은 비율로 쓴다(지도 흐름의 대수 비율).
    od = {}
    for f in flows:
        if net.hasEdge(f["from"]) and net.hasEdge(f["to"]):
            od[(f["from"], f["to"])] = od.get((f["from"], f["to"]), 0.0) + f["vehs_per_hour"]
    total = sum(od.values())
    for i, ((src, dst), vph) in enumerate(sorted(od.items())):
        rate = cfg.taxis_per_h * vph / total
        parts.append(f'<flow id="bgtaxi_{i}" type="bgtaxi" from="{src}" to="{dst}" begin="0" end="100000" '
                     f'period="exp({rate / 3600:.6f})" departLane="best" departSpeed="max"/>')
        summary["taxi_od"].append({"from": src, "to": dst, "per_h": round(rate, 2)})
    return parts, summary


def _curb_lane(edge):
    """도로에서 택시가 달릴 수 있는 가장 바깥(번호가 가장 작은) 차로 번호. 없으면 None."""
    return next((ln.getIndex() for ln in edge.getLanes() if ln.allows("taxi")), None)


def _signal_zone(net, edge):
    """도로 끝이 신호 교차로면 정지선 앞 금지 구간 (lo, hi), 아니면 None."""
    from .signal_areas import is_signalized, no_change_m
    e = net.getEdge(edge)
    node = e.getToNode()
    if not is_signalized(node):
        return None
    return max(0.0, e.getLength() - no_change_m(node.getID())), e.getLength()


def _straight_predecessor(net, edge):
    """edge로 곧게(직진 연결) 들어오는 앞 도로. 여럿이면 id 순 첫째, 없으면 None."""
    e = net.getEdge(edge)
    preds = sorted((p for p in e.getFromNode().getIncoming() if p.getConnections(e)
                    and any(c.getDirection() == "s" for c in p.getConnections(e))), key=lambda p: p.getID())
    return preds[0] if preds else None


def _curb_spot(net, stop):
    """정류장 stop 근처의 택시 정차 구역 (도로, 차로, 시작, 끝). 정류장 시작 20~10 m 앞, 곧게 들어오는 앞 도로 끝 15~5 m,
    정류장 끝 5~15 m 뒤 순으로 고르고, 도로 끝 5 m 안과 신호 정지선 앞 금지 구간은 피한다. 없으면 None."""
    edge, lane = stop["lane"].rsplit("_", 1)
    cands = [(edge, int(lane), stop["startPos"] - 20.0, stop["startPos"] - 10.0)]
    prev = _straight_predecessor(net, edge)
    if prev is not None and _curb_lane(prev) is not None:
        L = prev.getLength()
        cands.append((prev.getID(), _curb_lane(prev), L - 15.0, L - 5.0))
    cands.append((edge, int(lane), stop["endPos"] + 5.0, stop["endPos"] + 15.0))
    for e, ln, lo, hi in cands:
        if lo < 0 or hi > net.getEdge(e).getLength() - 5.0:
            continue
        zone = _signal_zone(net, e)
        if zone and zone[0] < hi and lo < zone[1]:
            continue
        return e, ln, round(lo, 2), round(hi, 2)
    return None


def curb_taxi_stops(net, flows, bus_stops, cfg=None, rng=None):
    """S4: 가로변 버스 정류장 근처에 서는 택시 흐름. 반환: (경로 파일 조각 목록, 추가 파일 조각 목록, 요약 목록).

    bus_stops: 도로망의 모든 정류장(lane, startPos, endPos). 택시가 설 수 있는 가로변 차로의 정류장을 모두 쓴다(중앙 버스
    전용 차로 정류장은 택시가 못 들어가 빠진다). 시험용 버스 노선은 보지 않는다. 택시 vType(bgtaxi)은 build_bus_traffic이 넣는다.
    """
    from .cutin import parking_area_xml, taxi_flow_xml
    from .presence import ends_at_boundary, starts_at_boundary
    cfg = cfg or BusTrafficCfg()
    rng = rng or random.Random(0)
    if cfg.curb_taxis_per_h <= 0:
        return [], [], []
    heads = sorted({f["from"] for f in flows if net.hasEdge(f["from"]) and starts_at_boundary(net, f["from"])})
    tails = sorted({f["to"] for f in flows if net.hasEdge(f["to"]) and ends_at_boundary(net, f["to"])})
    length = lambda p: sum(net.getEdge(e).getLength() for e in p)
    parts, adds, summary = [], [], []
    for s in sorted(bus_stops, key=lambda s: s["id"]):
        edge, lane = s["lane"].rsplit("_", 1)
        if not net.hasEdge(edge) or not net.getEdge(edge).getLanes()[int(lane)].allows("taxi"):
            summary.append({"stop": s["id"], "skipped": "taxi not allowed"})
            continue
        spot = _curb_spot(net, s)
        if spot is None:
            summary.append({"stop": s["id"], "skipped": "no curb spot"})
            continue
        edge, lane, lo, hi = spot
        head = min(filter(None, (_chain(net, [a, edge], "taxi") for a in heads)), key=length, default=None)
        tail = min(filter(None, (_chain(net, [edge, b], "taxi") for b in tails)), key=length, default=None)
        if not head or not tail:
            summary.append({"stop": s["id"], "skipped": "no route"})
            continue
        pid, rid = f"pa_curb_{s['id']}", f"r_curb_{s['id']}"
        dwell = round(rng.uniform(*cfg.curb_dwell), 1)
        adds.append(parking_area_xml(pid, f"{edge}_{lane}", lo, hi, 1))
        parts.append(f'<route id="{rid}" edges="{" ".join(head + tail[1:])}"/>')
        parts.append(taxi_flow_xml(f"curbtaxi_{s['id']}", rid, pid, dwell, round(cfg.curb_taxis_per_h / 3600.0, 6),
                                   vtype="bgtaxi", end=100000))
        summary.append({"stop": s["id"], "edge": edge, "lane": lane, "start": lo, "end": hi, "dwell": dwell,
                        "per_h": cfg.curb_taxis_per_h})
    return parts, adds, summary
