"""배경 버스·택시 투입(주변 차량). 시험용 버스(`bus_test`)와는 별개다.

두 가지를 투입한다. 대수는 모두 설정(`BusTrafficCfg`)이고 값은 자리표시다. 정류장에 서지 않는 배경 버스(통과 버스)는
투입하지 않는다. 버스는 전용 차로 노선뿐이다.

1. 전용 차로 버스(`bgbus_line*`): 버스 전용 차로를 지나며 정류장에 서서 승객을 태우고(정차) 출발한다.
   - 중앙 정류장 우회 차로(`B1`, `B2`, `B3a`→`B3b`, `B4a`→`B4b`) 노선 4개: 각각 그 차로의 정류장(`stop_0`~`stop_3`)에 선다.
   - 가로변 노선 2개: 가로변 정류장 `stop_4`(`R49a`)와 `stop_5`(`R44`)에 선다.
   - **시험용 버스가 서는 정류장(지도 `bus_stops` 6곳)에는 배경 버스가 서지 않는다.** 노선이 그 정류장을 쓰게 되면 그 노선은
     만들지 않는다(`build_bus_traffic`의 `test_bus_stop_ids`).
2. 택시(`bgtaxi`): 주변 승용차처럼 지나간다. 정류장 앞 정차 알고리즘(`stop_ahead.py`)이 서게 할 수 있는 후보가 된다.
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
    n_taxi_od: int = 10


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


def build_bus_traffic(net, flows, bus_loop, test_bus_stop_ids, cfg=None, rng=None):
    """(vType·경로·흐름 XML 조각 목록, 요약 dict). flows: map.json의 흐름(from/to/vehs_per_hour), 합정역 지도 기준."""
    cfg = cfg or BusTrafficCfg()
    rng = rng or random.Random(0)
    parts = [BUS_VTYPE.format(id="bgbus", color="0,160,90"), TAXI_VTYPE]
    summary = {"lines": [], "taxi_od": []}
    entries = sorted({f["from"] for f in flows if net.hasEdge(f["from"])})
    exits = sorted({f["to"] for f in flows if net.hasEdge(f["to"])})
    central_rate = cfg.lane_buses_per_h / len(CENTRAL_GROUPS)
    dwell = lambda: round(rng.uniform(*cfg.dwell), 1)
    test_stops = set(test_bus_stop_ids)

    for name, via, stop in CENTRAL_GROUPS:
        if stop in test_stops:
            continue                                  # 시험용 버스가 서는 정류장에는 다른 버스가 서지 않는다
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

    def spread(kind, per_h, n_od, vtype):
        """주변 승용차 흐름의 (문→나가는 곳) 짝 중 많이 쓰이는 것을 골라 대수를 나눠 넣는다."""
        # 시험용 버스 노선(bus_loop)을 많이 지나는 (문→나가는 곳) 짝을 먼저 고른다. 그래야 정류장 앞에서 시험용 버스 앞에 놓인
        # 택시·버스가 실제로 있다(없으면 stop_ahead가 택시를 새로 넣는다).
        loop = set(bus_loop or [])

        def uses_loop(f):
            p = _chain(net, [f["from"], f["to"]], vclass="passenger")
            return sum(1 for e in (p or []) if e in loop)
        top = sorted((f for f in flows if not f.get("via") and net.hasEdge(f["from"]) and net.hasEdge(f["to"])),
                     key=lambda f: (-uses_loop(f), -f["vehs_per_hour"]))
        chosen, seen = [], set()
        for f in top:
            if (f["from"], f["to"]) in seen:
                continue
            seen.add((f["from"], f["to"]))
            chosen.append(f)
            if len(chosen) == n_od:
                break
        w = sum(f["vehs_per_hour"] for f in chosen)
        for i, f in enumerate(chosen):
            rate = per_h * f["vehs_per_hour"] / w
            parts.append(f'<flow id="{kind}_{i}" type="{vtype}" from="{f["from"]}" to="{f["to"]}" begin="0" '
                         f'end="100000" period="exp({rate / 3600:.6f})" departLane="best" departSpeed="max"/>')
            summary["taxi_od"].append(
                {"from": f["from"], "to": f["to"], "per_h": round(rate, 1)})

    spread("bgtaxi", cfg.taxis_per_h, cfg.n_taxi_od, "bgtaxi")
    return parts, summary
