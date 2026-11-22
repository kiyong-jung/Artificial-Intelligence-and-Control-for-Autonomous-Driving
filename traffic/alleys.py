"""S5 일반화: 지도에서 골목 출구와 그 골목을 지나는 흐름을 자동으로 찾는다.

지도마다 골목 id를 손으로 적지 않는다. 골목은 도로 종류로 정한다(`is_alley`: 제한속도 30 km/h 이하이고 도로 등급이
연결로·간선보다 낮은 도로).

- 골목 출구(`find_alley_exits`): 도로망 전체에서 골목이 골목 아닌 도로로 이어지는 곳. 골목 운전자 분포를 줄 흐름을 고른다.
  버스 경로를 보지 않는다.
- 버스 경로 합류점(`find_merge_alleys`): 버스 경로의 연속한 두 도로(앞 도로 -> 다음 도로)가 만나는 교차로에서, 경로에 없는
  골목이 다음 도로로 연결되는 곳. S5 측정(골목에서 나온 차가 버스 앞에 들어왔나)만 쓴다.
흐름(`flow` 요소)이 합류하는지는 출발·경유·도착 도로로 최단 경로를 만들어 '골목 진입 도로 -> 경로의 다음 도로'가
연달아 나오는지로 판정한다. 그래서 골목에서 출발하는 흐름과 `via`로 골목 고리를 거쳐 나오는 흐름이 모두 잡힌다.
"""
from dataclasses import dataclass

import sumolib

# 신호 없이 우선권으로 정해지는 교차로 유형(진입 도로가 비우선이면 끼어들기 vType의 jm* 파라미터가 듣는다)
_PRIORITY_TYPES = {"priority", "priority_stop", "right_before_left", "traffic_light", "traffic_light_right_on_red"}


@dataclass(frozen=True)
class MergeAlley:
    junction: str          # 합류 교차로 id
    feeder: str            # 골목의 마지막 도로(합류점으로 들어오는 도로)
    into: str              # 합류하는 버스 경로의 도로
    junction_type: str     # net.xml의 교차로 유형
    minor: bool            # 연결이 비우선('m')인가. 아니면 골목이 우선이라 끼어들기 vType이 의미 없다
    signalized: bool

    @property
    def regulated(self):
        """충돌이 기록되는(= priority 계열이거나 신호가 있는) 합류점인가. unregulated면 False."""
        return self.junction_type in _PRIORITY_TYPES


def read_net(net_path):
    return sumolib.net.readNet(net_path, withInternal=False, withPrograms=False)


def close_loop(net, route):
    """순환 노선의 끝과 처음 사이에 시뮬레이션이 최단 경로로 잇는 도로를 노선 뒤에 붙여 돌려준다.

    노선 목록은 R42로 끝나고 R69로 시작해 둘이 이어져 있지 않다. 시뮬레이션은 시작할 때 최단 경로로 둘을 잇는다
    (scripts/run_hapjeong_scenario.py). 그 사이 도로(R68)에 있는 좌회전을 노선 목록만 보면 놓친다.
    잇지 못하거나 이미 이어져 있으면 노선을 그대로 돌려준다.
    """
    route = list(route)
    if len(route) < 2 or not (net.hasEdge(route[-1]) and net.hasEdge(route[0])):
        return route
    try:
        path = net.getShortestPath(net.getEdge(route[-1]), net.getEdge(route[0]))[0]
    except Exception:
        return route
    return route + [e.getID() for e in (path or [])[1:-1]]


ALLEY_MAX_SPEED = 8.4      # m/s, 30 km/h
ALLEY_MAX_PRIORITY = 3     # 도로망의 도로 등급. 골목(G*)은 1, 어울마당로 토막은 3, 연결로(primary_link)는 7, 간선은 10~12


def is_alley(edge, max_speed=ALLEY_MAX_SPEED, max_priority=ALLEY_MAX_PRIORITY):
    """차가 다니는 골목인가(도로 종류로만 판정한다)."""
    return edge.allows("passenger") and edge.getSpeed() <= max_speed and edge.getPriority() <= max_priority


def _is_signalized(node):
    return node.getType().startswith("traffic_light") and node.getType() != "traffic_light_unregulated"


def _alley(node, feeder, into, conns):
    sig = _is_signalized(node)
    return MergeAlley(node.getID(), feeder.getID(), into.getID(), node.getType(),
                      all(c.getState() in ("m", "s", "w") for c in conns) or sig, sig)


def find_alley_exits(net):
    """도로망의 골목 출구 전부: 골목에서 골목 아닌 도로로 나가는 연결. 반환: [MergeAlley]."""
    found = []
    for node in net.getNodes():
        for feeder in node.getIncoming():
            if not is_alley(feeder):
                continue
            for into in node.getOutgoing():
                if is_alley(into) or not into.allows("passenger") or into.getToNode() == feeder.getFromNode():
                    continue
                conns = feeder.getConnections(into)
                if conns:
                    found.append(_alley(node, feeder, into, conns))
    return found


def find_merge_alleys(net, route_edges):
    """버스 경로에 합류하는 골목 진입 도로를 모두 찾는다. 반환: [MergeAlley]."""
    route = list(route_edges)
    on_route = set(route)
    found, seen = [], set()
    for here, nxt in zip(route, route[1:]):
        if not net.hasEdge(here) or not net.hasEdge(nxt):
            continue
        node = net.getEdge(nxt).getFromNode()
        if net.getEdge(here).getToNode() != node:
            continue
        for feeder in node.getIncoming():
            fid = feeder.getID()
            if fid in on_route or (fid, nxt) in seen:
                continue
            if feeder.getFromNode() == net.getEdge(nxt).getToNode():      # 되돌아 나오는 반대 방향 도로
                continue
            if not is_alley(feeder):
                continue
            conns = feeder.getConnections(net.getEdge(nxt))
            if not conns:
                continue
            seen.add((fid, nxt))
            found.append(_alley(node, feeder, net.getEdge(nxt), conns))
    return found


def _chain(net, edges):
    """도로 id 목록을 차례로 이은 최단 경로(도로 id 목록). 이을 수 없으면 None."""
    path = []
    for a, b in zip(edges, edges[1:]):
        seg, _ = net.getOptimalPath(net.getEdge(a), net.getEdge(b))
        if not seg:
            return None
        path.extend(e.getID() for e in (seg if not path else seg[1:]))
    return path


def flow_merges(net, flow_attrs, alleys):
    """흐름이 `alleys` 중 하나로 합류하는가. 경로를 만들 수 없으면 도로 id 포함 여부로 대신한다."""
    stops = [flow_attrs.get("from"), *(flow_attrs.get("via") or "").split(), flow_attrs.get("to")]
    stops = [s for s in stops if s and net.hasEdge(s)]
    pairs = {(a.feeder, a.into) for a in alleys}
    path = _chain(net, stops) if len(stops) >= 2 else None
    path = path if path else stops
    return any((a, b) in pairs for a, b in zip(path, path[1:]))
