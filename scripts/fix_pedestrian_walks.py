"""보행자 걸음을 지금 도로망에 맞춘다(3.14.0 -> 3.15.0, 도로망은 그대로).

    python scripts/fix_pedestrian_walks.py

보행자 경로 파일의 걸음(`<walk edges>`)은 도로 순서만 적고, 교차로 안에서 어느 횡단보도로 건널지는 SUMO가 걸으면서 정한다.
여기서 두 가지 순간이동이 생겼다(MSPModel_Striping, SUMO 1.27.1).

1. 이어진 두 도로 사이에 걸어갈 길이 없으면 SUMO는 경고 없이 다음 도로로 옮긴다. 3.9.0이 cluster_J52_J56_J58의
   횡단보도를 없앤 뒤 북쪽 인도(-R77과 -R73)가 끊겼는데, 그 교차로를 지나는 걸음 둘이 남아 36~41m를 건너뛰었다.
   이런 걸음은 출발 도로에서 도착 도로까지 지금 도로망에서 다시 찾는다.
2. 대각선으로 건너는 사람(교차로 하나에서 횡단보도 둘)은 갈 수 있는 횡단보도 순서가 둘이다. 첫 횡단보도 앞에서 적색을 만나면
   SUMO가 다른 순서로 바꾸는데, 대기 구역 안의 걸음 모양은 옛 횡단보도 끝에 둔 채 새 횡단보도로 옮긴다(소스의
   "@todo actually another path would be needed starting at the current position"). 두 횡단보도 끝의 거리만큼(3~11m) 뛴다.
   이런 걸음에는 짧은 쪽 횡단보도 둘을 걸음에 적어 고정한다. 그러면 SUMO가 바꿀 다른 순서가 없다.

횡단보도를 하나만 건너는 걸음은 그대로 둔다(S6 보행자 포함). 이미 맞춘 파일에 다시 돌리면 아무것도 바뀌지 않는다.
map.json의 `removed_crossing_walkers`에 적힌 횡단보도(J1_c1)는 다른 길이 있으면 고정 경로에 쓰지 않는다.
"""
from __future__ import annotations

import heapq
import json
from pathlib import Path
import xml.etree.ElementTree as ET

FOLDER = Path(__file__).resolve().parents[1] / "maps" / "hapjeong"
ROUTES = FOLDER / "source" / "pedestrians.rou.xml"
NET = FOLDER / "source" / "network.net.xml"


class WalkGraph:
    """보행자가 다닐 수 있는 차로(인도, 대기 구역, 횡단보도)와 그 사이의 연결. 방향 없이 걷는다."""

    def __init__(self, net_path: Path):
        root = ET.parse(net_path).getroot()
        self.length: dict[str, float] = {}
        self.sidewalk: dict[str, str] = {}               # 일반 도로 -> 그 인도 차로
        self.ends: dict[str, tuple[str, str]] = {}       # 일반 도로 -> (시작 노드, 끝 노드)
        for edge in root.iter("edge"):
            function = edge.get("function")
            if function not in (None, "walkingarea", "crossing"):
                continue
            for lane in edge.iter("lane"):
                if "pedestrian" not in (lane.get("allow") or "").split():
                    continue
                self.length[lane.get("id")] = float(lane.get("length"))
                if function is None:
                    self.sidewalk.setdefault(edge.get("id"), lane.get("id"))
                    self.ends[edge.get("id")] = (edge.get("from"), edge.get("to"))
        self.next: dict[str, set[str]] = {lane: set() for lane in self.length}
        for conn in root.iter("connection"):
            a, b = f"{conn.get('from')}_{conn.get('fromLane')}", f"{conn.get('to')}_{conn.get('toLane')}"
            if a in self.length and b in self.length:
                self.next[a].add(b)
                self.next[b].add(a)

    def path(self, start: str, goal: str, allowed=lambda lane: True) -> list[str] | None:
        """start 차로에서 goal 차로까지 가장 짧은 차로 열(양 끝 포함). allowed는 중간 차로를 거른다."""
        best, queue = {start: 0.0}, [(0.0, start, (start,))]
        while queue:
            cost, lane, trail = heapq.heappop(queue)
            if lane == goal:
                return list(trail)
            if cost > best[lane]:
                continue
            for other in self.next[lane]:
                if other != goal and not allowed(other):
                    continue
                total = cost + self.length[other]
                if total < best.get(other, float("inf")):
                    best[other] = total
                    heapq.heappush(queue, (total, other, trail + (other,)))
        return None

    def across(self, a: str, b: str, avoid: frozenset[str]) -> list[str] | None:
        """도로 a에서 이어지는 도로 b로 둘이 만나는 교차로 안에서만 건너는 횡단보도 열. 길이 없으면 None."""
        best = None
        for node in set(self.ends[a]) & set(self.ends[b]):
            inside = f":{node}_"
            for skip in (avoid, frozenset()):
                trail = self.path(self.sidewalk[a], self.sidewalk[b],
                                  lambda lane: lane.startswith(inside) and lane.rsplit("_", 1)[0] not in skip)
                if trail is not None:
                    middle = [lane.rsplit("_", 1)[0] for lane in trail[1:-1]]
                    crossings = [edge for edge in middle if edge[len(inside):].startswith("c")]
                    cost = sum(self.length[lane] for lane in trail[1:-1])
                    if best is None or cost < best[0]:
                        best = (cost, crossings)
                    break
        return None if best is None else best[1]

    def reroute(self, origin: str, destination: str) -> list[str] | None:
        """출발 도로에서 도착 도로까지 도로망 전체에서 다시 찾은 일반 도로 열."""
        trail = self.path(self.sidewalk[origin], self.sidewalk[destination])
        if trail is None:
            return None
        edges = []
        for lane in trail:
            edge = lane.rsplit("_", 1)[0]
            if not edge.startswith(":") and edge not in edges[-1:]:
                edges.append(edge)
        return edges


def fix_walk(graph: WalkGraph, edges: list[str], avoid: frozenset[str] = frozenset()) -> tuple[list[str], str]:
    """걸음 하나를 맞춘다. 반환: (새 도로 열, "same" | "pinned" | "rerouted")."""
    roads = [edge for edge in edges if not edge.startswith(":")]
    status = "same"
    pairs = [graph.across(a, b, avoid) for a, b in zip(roads, roads[1:])]
    if any(crossings is None for crossings in pairs):
        rerouted = graph.reroute(roads[0], roads[-1])
        if rerouted is None:
            raise ValueError(f"걸음 {' '.join(edges)}: 출발 도로에서 도착 도로까지 걸어갈 길이 없다")
        roads, status = rerouted, "rerouted"
        pairs = [graph.across(a, b, avoid) or [] for a, b in zip(roads, roads[1:])]
    fixed = roads[:1]
    for road, crossings in zip(roads[1:], pairs):
        if len(crossings) >= 2:
            fixed += crossings
        fixed.append(road)
    if status == "same" and fixed != edges:
        status = "pinned"
    return fixed, status


def fix_walks(root: ET.Element, net_path: Path, avoid: frozenset[str] = frozenset()) -> dict[str, list[str]]:
    """root의 모든 걸음을 맞춘다. 반환: 상태별 흐름 id."""
    graph = WalkGraph(net_path)
    changed: dict[str, list[str]] = {"pinned": [], "rerouted": []}
    for person in root:
        for walk in person.iter("walk"):
            fixed, status = fix_walk(graph, walk.get("edges").split(), avoid)
            if status != "same":
                walk.set("edges", " ".join(fixed))
                changed[status].append(person.get("id"))
    return changed


def main() -> None:
    definition = json.loads((FOLDER / "map.json").read_text(encoding="utf-8"))
    record = definition["network_source"]["pedestrians"]
    avoid = frozenset(f":{crossing}" for crossing in record.get("removed_crossing_walkers", {}))
    raw = ROUTES.read_bytes()
    root = ET.fromstring(raw)
    changed = fix_walks(root, NET, avoid)
    if not any(changed.values()):
        print("이미 맞췄다: 바꾸지 않는다")
        return
    ET.register_namespace("xsi", "http://www.w3.org/2001/XMLSchema-instance")
    text = ET.tostring(root, encoding="unicode")
    newline = "\r\n" if b"\r\n" in raw else "\n"   # 커밋된 파일은 CRLF다
    ROUTES.write_bytes(("<?xml version='1.0' encoding='utf-8'?>\n" + text + "\n").replace("\n", newline).encode("utf-8"))
    record["walks_fixed"] = {"rerouted": changed["rerouted"], "pinned_diagonal": len(changed["pinned"]),
                             "script": "scripts/fix_pedestrian_walks.py"}
    (FOLDER / "map.json").write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"다시 찾은 걸음 {len(changed['rerouted'])}개 {changed['rerouted']}, 횡단보도를 고정한 걸음 {len(changed['pinned'])}개")


if __name__ == "__main__":
    main()
