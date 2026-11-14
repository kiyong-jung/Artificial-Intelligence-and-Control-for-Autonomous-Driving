"""배경 버스·택시 투입(주변 차량). 시험용 버스(`bus_test`)와는 별개다.

세 가지를 투입한다. 대수는 모두 설정(`BusTrafficCfg`)이고 값은 자리표시다.

1. 전용 차로 버스(`bgbus_line*`): 버스 전용 차로를 지나며 정류장에 서서 승객을 태우고(정차) 출발한다.
   - 중앙 정류장 우회 차로(`B1`, `B2`, `B3a`→`B3b`, `B4a`→`B4b`) 노선 4개: 각각 그 차로의 정류장(`stop_0`~`stop_3`)에 선다.
   - 가로변 노선: 시험용 버스와 같은 `bus_loop` 경로를 돌며 정류장 6곳에 선다. 가로변 전용 차로(맨 왼쪽 차로)는 SUMO가
     차로를 고를 때 쓸 수 있다.
2. 통과 버스(`bgbus_pass`): 정류장에 서지 않고 주변 승용차처럼 (문 → 나가는 곳) 흐름으로 지나간다. 전용 차로 버스보다 적다.
3. 택시(`bgtaxi`): 주변 승용차처럼 지나간다. 정류장 앞 정차 알고리즘(`stop_ahead.py`)이 서게 할 수 있는 후보가 된다.
"""
import random
from dataclasses import dataclass, field

BUS_VTYPE = ('<vType id="{id}" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" tau="1.0" sigma="0.2" '
             'guiShape="bus" color="{color}"/>')
TAXI_VTYPE = ('<vType id="bgtaxi" vClass="taxi" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0.3" '
              'guiShape="taxi" color="255,200,0"/>')

# 중앙 정류장 우회 차로: (이름, 지나는 도로들, 정류장 id)
CENTRAL_GROUPS = (("B1", ["B1"], "stop_0"), ("B2", ["B2"], "stop_1"),
                  ("B3", ["B3a", "B3b"], "stop_2"), ("B4", ["B4a", "B4b"], "stop_3"))


@dataclass
class BusTrafficCfg:
    lane_buses_per_h: float = 30.0     # 전용 차로 버스(정류장에 서고 출발) 시간당 대수. 중앙 노선 4개와 가로변 노선에 나눈다
    central_share: float = 0.4         # 위 대수 중 중앙 우회 차로 노선 몫(나머지는 가로변 노선)
    pass_buses_per_h: float = 12.0     # 정류장에 서지 않는 통과 버스. 전용 차로 버스보다 적어야 한다
    taxis_per_h: float = 40.0          # 택시
    dwell: tuple = (12.0, 25.0)        # 정류장 정차 시간 U(lo, hi) [s] (흐름마다 하나)
    n_pass_od: int = 6                 # 통과 버스를 나눠 넣을 (문→나가는 곳) 짝 수
    n_taxi_od: int = 10

    def __post_init__(self):
        if self.pass_buses_per_h >= self.lane_buses_per_h:
            raise ValueError("통과 버스는 전용 차로 버스보다 적어야 한다")


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
    """via(중앙 우회 차로 도로들)를 지나는 가장 짧은 버스 경로. (문 → via → 나가는 곳)."""
    # 노선은 우회 차로를 지난 뒤 그 바로 다음 도로의 끝에서 끝낸다. 더 가면 우회 차로에서 막 합류한 버스가 가장 왼쪽 전용
    # 차로(예: R55_5)에서 다음 도로로 빠지려 차선을 바꾸지 못하고 300초 서 있다 순간이동하는 일이 있었다(검증으로 확인).
    out = [o.getID() for o in net.getEdge(via[-1]).getOutgoing() if o.allows("bus")]
    if not out:
        return None
    best = None
    for a in entries:
        head = _chain(net, [a, via[0]])
        if not head:
            continue
        mid = _chain(net, via) if len(via) > 1 else [via[0]]
        path = head + mid[1:] + [out[0]]
        length = sum(net.getEdge(e).getLength() for e in path)
        if best is None or length < best[0]:
            best = (length, path)
    return best[1] if best else None


def build_bus_traffic(net, flows, bus_loop, loop_stop_ids, cfg=None, rng=None):
    """(vType·경로·흐름 XML 조각 목록, 요약 dict). flows: map.json의 흐름(from/to/vehs_per_hour), 합정역 지도 기준."""
    cfg = cfg or BusTrafficCfg()
    rng = rng or random.Random(0)
    parts = [BUS_VTYPE.format(id="bgbus", color="0,160,90"), BUS_VTYPE.format(id="bgbus_pass", color="120,200,120"),
             TAXI_VTYPE]
    summary = {"lines": [], "pass_od": [], "taxi_od": []}
    entries = sorted({f["from"] for f in flows if net.hasEdge(f["from"])})
    exits = sorted({f["to"] for f in flows if net.hasEdge(f["to"])})
    lane_total = cfg.lane_buses_per_h
    central_rate = lane_total * cfg.central_share / len(CENTRAL_GROUPS)
    loop_rate = lane_total * (1 - cfg.central_share)
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
    if bus_loop:
        stops = "".join(f'<stop busStop="{s}" duration="{dwell()}"/>' for s in loop_stop_ids)
        parts.append(f'<route id="r_bgbus_loop" edges="{" ".join(bus_loop)}"/>')
        parts.append(f'<flow id="bgbus_line_loop" type="bgbus" route="r_bgbus_loop" begin="0" end="100000" '
                     f'period="exp({loop_rate / 3600:.6f})" departLane="best" departSpeed="max">{stops}</flow>')
        summary["lines"].append({"id": "bgbus_line_loop", "stops": len(loop_stop_ids), "per_h": round(loop_rate, 1)})

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
            summary[f"{'pass' if kind == 'bgbus_pass' else 'taxi'}_od"].append(
                {"from": f["from"], "to": f["to"], "per_h": round(rate, 1)})

    spread("bgbus_pass", cfg.pass_buses_per_h, cfg.n_pass_od, "bgbus_pass")
    spread("bgtaxi", cfg.taxis_per_h, cfg.n_taxi_od, "bgtaxi")
    return parts, summary
