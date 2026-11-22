"""신호 교차로 영역: 신호가 순서를 정해 주므로 눈치 싸움이 없는 곳이다. 여기서는 A1·A2(S4·S5)를 켜지 않는다.

- 교차로 내부와 정지선 앞 `no_change_m(교차로)` m가 영역이다. 시험용 버스는 이 안에서 차선을 바꾸지 않는다
  (좌회전 차선 변경 S2는 그 전에 끝난다).
- 정지선 앞 거리는 기본 `SIGNAL_NO_CHANGE_M`이고, J1 앞은 차로를 3~4칸 옮겨야 해 빠듯하므로 10 m다(사용자가 정한 5~10 m
  범위. 창을 놓치던 원인은 정류장에서 막 출발한 버스의 과한 감속이었고 고쳤다. 다시 놓치면 5 m). **두 값은 여기에만 둔다.**
"""
from .yield_logic import Zone

SIGNAL_NO_CHANGE_M = 20.0
SIGNAL_NO_CHANGE_OVERRIDE = {"J1": 10.0}


def no_change_m(node_id):
    return SIGNAL_NO_CHANGE_OVERRIDE.get(node_id, SIGNAL_NO_CHANGE_M)


def is_signalized(node):
    t = node.getType()
    return t.startswith("traffic_light") and t != "traffic_light_unregulated"


def unsignalized_alleys(alleys):
    """S5·S4 후보 골목 중 신호 없는 교차로로 합류하는 것만."""
    return [a for a in alleys if not a.signalized]


def forbidden_intervals(net, route):
    """노선 위 신호 교차로 정지선 앞 금지 구간들: {도로: [(lo, hi), ...]}. 짧은 도로면 앞 도로로 넘어간다."""
    out = {}
    for k, eid in enumerate(route):
        if not net.hasEdge(eid):
            continue
        node = net.getEdge(eid).getToNode()
        if not is_signalized(node):
            continue
        rem, j = no_change_m(node.getID()), k
        while rem > 0 and j >= 0 and net.hasEdge(route[j]):
            e = net.getEdge(route[j])
            L = e.getLength()
            lo = max(0.0, L - rem)
            out.setdefault(route[j], []).append((lo, L))
            rem -= L - lo
            j -= 1
            if j < 0 or not net.hasEdge(route[j]) or net.getEdge(route[j]).getToNode() != e.getFromNode():
                break
    return out


def clip_zones(net, route, zones):
    """구간 목록에서 신호 교차로 정지선 앞 금지 구간을 뺀다. 남는 조각만 돌려준다(순서 유지)."""
    bad = forbidden_intervals(net, route)
    out = []
    for z in zones:
        pieces = [(z.start, z.end)]
        for lo, hi in bad.get(z.edge, []):
            nxt = []
            for a, b in pieces:
                if hi <= a or lo >= b:
                    nxt.append((a, b))
                    continue
                if a < lo:
                    nxt.append((a, lo))
                if hi < b:
                    nxt.append((hi, b))
            pieces = nxt
        out += [Zone(z.edge, a, b) for a, b in pieces if b - a > 1e-6]
    return out
