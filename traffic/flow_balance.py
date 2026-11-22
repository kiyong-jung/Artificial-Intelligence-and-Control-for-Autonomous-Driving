"""배경 차량 흐름 재배분: 버스 노선 구간을 지나는 흐름은 키우고, 지나지 않는 흐름은 줄여 **전체 대수는 그대로** 둔다.

지도의 배경 흐름 178개(합 1061대/시간) 중 시험용 버스 노선 구간을 지나는 것은 87개 402대/시간(38 %)뿐이고, 나머지 62 %(659대/시간)는 노선을
지나지 않는다. 전체 도로의 차량 수 때문에 노선 구간에 더 넣을 수 없었으므로, 노선을 지나지 않는 흐름을 줄여 그만큼을 노선을 지나는 흐름에 돌린다.

- 노선을 지나는 흐름 = 그 흐름의 (들어오는 도로, 나가는 도로) 사이 최단 경로(승용차)가 노선 도로를 하나라도 포함하는 것.
- 노선 흐름 배율 `boost`(예: 1.3), 나머지 흐름 배율은 전체 대수가 같도록 정한다. 나머지가 `min_off`배 아래로 줄지 않게 boost를 낮춘다.
- **피할 도로**(와우산로·잔다리로): 그 도로에서 시작하는 입구 흐름은 없애고(투입 없음), 그 도로를 지나거나 그 도로로 가는 흐름은 목적지를
  다른 출구로 바꿔 J82·J83 같은 교차로에서 다른 도로로 빠져나가게 한다(`redirect_flows`).
- **키우지 않을 구간**(상수역 J82 접근로): 그 구간을 지나는 흐름은 노선 흐름으로 치지 않는다(키우지 않고, 나머지로 줄어든다).
"""


def edges_named(net, names):
    """이름이 names에 속하는 도로 id 집합(반대 방향 도로 포함)."""
    names = set(names)
    return {e.getID() for e in net.getEdges() if e.getName() in names}


def path_ids(net, fr, to, vclass="passenger"):
    """fr에서 to까지 최단 경로의 도로 id 목록(경로가 없으면 빈 목록)."""
    if not (net.hasEdge(fr) and net.hasEdge(to)):
        return []
    seg, _ = net.getOptimalPath(net.getEdge(fr), net.getEdge(to), vClass=vclass)
    return [e.getID() for e in seg] if seg else []


def route_touching(net, pairs, route_edges, vclass="passenger", forbidden=frozenset()):
    """pairs [(from, to)] 중 최단 경로가 route_edges의 도로를 하나라도 지나고 forbidden의 도로는 지나지 않는 쌍의 집합."""
    route = set(route_edges)
    out = set()
    for fr, to in pairs:
        ids = path_ids(net, fr, to, vclass)
        if ids and any(e in route for e in ids) and not any(e in forbidden for e in ids):
            out.add((fr, to))
    return out


def redirect_flows(net, flows, avoid, rng, vclass="passenger"):
    """피할 도로(avoid)를 쓰지 않게 흐름을 고친다. flows는 {"id","from","to","rate"} 목록이고 제자리에서 바꾼다.

    - 입구(from)가 피할 도로 위면 투입을 없앤다(rate 0).
    - 경로가 피할 도로를 지나거나 도착지가 피할 도로면, 피할 도로를 지나지 않는 다른 출구로 목적지를 바꾼다(출구는 원래 흐름에서 그 출구로
      가던 양에 비례해 뽑는다). 그런 출구가 없으면 투입을 없앤다.
    돌려주는 값: {"no_insert", "redirected", "dropped"} 개수와 줄어든 대수(대/시간) "removed_per_h"."""
    avoid = set(avoid)
    share = {}
    for f in flows:
        share[f["to"]] = share.get(f["to"], 0.0) + f["rate"]
    cands = [x for x in sorted(share) if x not in avoid]
    cache = {}
    stats = {"no_insert": 0, "redirected": 0, "dropped": 0, "removed_per_h": 0.0}
    for f in flows:
        if f["from"] in avoid:
            stats["no_insert"] += 1
            stats["removed_per_h"] += f["rate"]
            f["rate"] = 0.0
            continue
        ids = path_ids(net, f["from"], f["to"], vclass)
        if f["to"] not in avoid and not any(e in avoid for e in ids):
            continue
        key = f["from"]
        if key not in cache:
            cache[key] = [x for x in cands if x != f["from"]
                          and (p := path_ids(net, f["from"], x, vclass)) and not any(e in avoid for e in p)]
        alts = cache[key]
        if not alts:
            stats["dropped"] += 1
            stats["removed_per_h"] += f["rate"]
            f["rate"] = 0.0
            continue
        f["to"] = rng.choices(alts, weights=[max(share[x], 1e-6) for x in alts])[0]
        stats["redirected"] += 1
    return stats


def balance_factors(rates, touching, boost, min_off=0.3, target_total=None):
    """(노선 흐름 배율, 나머지 흐름 배율). rates {키: 대/시간}, touching 노선을 지나는 키 집합. 전체 합을 target_total(기본은 지금 합)로 맞춘다.

    노선 흐름이 없거나 나머지가 없으면 (1, 1). 나머지 배율이 min_off 아래로 내려가야 하면 boost를 그만큼 낮춘다."""
    on = sum(r for k, r in rates.items() if k in touching)
    off = sum(r for k, r in rates.items() if k not in touching)
    total = on + off if target_total is None else target_total
    if on <= 0 or off <= 0 or (boost <= 1.0 and target_total is None):
        return 1.0, 1.0
    boost = min(boost, (total - min_off * off) / on)
    return boost, (total - on * boost) / off


def segment_before_node(net, route, node_id):
    """노선(도로 id 목록)에서 신호 교차로 node_id로 들어가는 구간: 앞 신호 교차로 뒤 첫 도로부터 node_id 앞 마지막 도로까지. 없으면 빈 목록."""
    from .signal_areas import is_signalized
    cur = []
    for e in route:
        if not net.hasEdge(e):
            cur = []
            continue
        cur.append(e)
        node = net.getEdge(e).getToNode()
        if is_signalized(node):
            if node.getID() == node_id:
                return cur
            cur = []
    return []
