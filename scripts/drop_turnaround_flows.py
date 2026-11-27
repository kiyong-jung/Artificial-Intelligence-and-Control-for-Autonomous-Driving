"""되돌아가거나 크게 에돌아야만 닿는 배경 흐름을 지우고, 그 양을 같은 진입로의 다른 흐름에 나눈다(3.14.0 -> 3.15.0).

    python scripts/drop_turnaround_flows.py

배경 흐름은 출발·도착 도로만 정하고 경로는 SUMO가 가장 빠른 길로 고른다. 진입로마다 모든 진출로로 흐름을 만들었기 때문에
(scripts/build_hapjeong.py의 scaled_flows) 도착지가 출발지 뒤에 있는 흐름이 생겼다. SUMO는 그런 흐름을 지도 경계에서
유턴시키거나(R130 -> -R130, J110), 독막로6길 고리(G30 G11 G12)를 돌려 반대 차로로 보내거나, 지도를 거의 한 바퀴
돌린다. 실제 운전자라면 지도 밖에서 돌아 나갈 길이다.

각 흐름을 SUMO의 경로 탐색기(duarouter, 기본 설정)로 한 번 찾아 다음 중 하나라도 맞으면 지운다.
- 유턴: 이어진 두 도로가 한 도로의 두 방향이다(a -> -a).
- 되돌아감: 같은 도로를 두 방향으로 지나거나(R71과 -R71.55처럼 쪼갠 도로 포함) 같은 교차로를 두 번 지난다.
- 같은 문: 출발 도로의 시작점과 도착 도로의 끝점이 GATE_M 안이다(나뉜 차로의 두 방향).
- 에돎: 경로 길이가 출발점과 도착점 직선 거리의 DETOUR 배를 넘는다. 한 블록을 세 변으로 돌아가는 길은 3배다.

지운 흐름의 양은 같은 진입로에 남은 흐름에 그 양의 비율대로 나눈다. 진입로마다의 유입량과 합계는 그대로다.
이미 맞춘 map.json에 다시 돌리면 아무것도 바뀌지 않는다.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import sumolib

FOLDER = Path(__file__).resolve().parents[1] / "maps" / "hapjeong"
GATE_M = 80.0
DETOUR = 4.0


def _road(edge: str) -> str:
    """쪼갠 도로(R71.135)와 반대 방향(-R71)을 한 도로로 본다."""
    return edge.lstrip("-").split(".")[0]


def fastest_routes(net_path: Path, flows: list[dict]) -> dict[str, list[str]]:
    """흐름마다 SUMO가 고를 가장 빠른 경로. 승용차 한 대(최고 속도 13.89m/s, 속도 배율 1)로 찾는다."""
    with tempfile.TemporaryDirectory(prefix="flows_") as name:
        trips, routes = Path(name) / "trips.xml", Path(name) / "routes.xml"
        root = ET.Element("routes")
        ET.SubElement(root, "vType", id="car", vClass="passenger", maxSpeed="13.89", speedFactor="1", speedDev="0")
        for index, flow in enumerate(flows):
            ET.SubElement(root, "trip", id=flow["id"], type="car", depart=str(index), **{"from": flow["from"], "to": flow["to"]},
                          **({"via": flow["via"]} if flow.get("via") else {}))
        ET.ElementTree(root).write(trips)
        subprocess.run([sumolib.checkBinary("duarouter"), "-n", str(net_path), "-r", str(trips), "-o", str(routes),
                        "--no-step-log", "true"], check=True, capture_output=True)
        return {vehicle.get("id"): vehicle.find("route").get("edges").split()
                for vehicle in ET.parse(routes).getroot().iter("vehicle")}


def turnaround_reasons(net, edges: list[str]) -> list[str]:
    """경로가 자연스럽지 않은 이유. 비면 자연스럽다."""
    reasons = []
    if any(a == "-" + b or b == "-" + a for a, b in zip(edges, edges[1:])):
        reasons.append("uturn")
    directions: dict[str, set[bool]] = {}
    for edge in edges:
        directions.setdefault(_road(edge), set()).add(edge.startswith("-"))
    nodes = [net.getEdge(edges[0]).getFromNode().getID()] + [net.getEdge(edge).getToNode().getID() for edge in edges]
    if any(len(both) == 2 for both in directions.values()) or len(set(nodes)) != len(nodes):
        reasons.append("turns_back")
    start, end = net.getEdge(edges[0]).getFromNode().getCoord(), net.getEdge(edges[-1]).getToNode().getCoord()
    straight = math.dist(start, end)
    if straight < GATE_M:
        reasons.append("same_gate")
    if sum(net.getEdge(edge).getLength() for edge in edges) > DETOUR * max(straight, GATE_M):
        reasons.append("detour")
    return reasons


def keep_natural(net_path: Path, flows: list[dict]) -> tuple[list[dict], dict[str, list[str]]]:
    """자연스럽지 않은 흐름을 지우고 그 양을 같은 진입로의 남은 흐름에 나눈다. 반환: (남은 흐름, 지운 흐름 id -> 이유)."""
    net = sumolib.net.readNet(str(net_path))
    routes = fastest_routes(net_path, flows)
    removed = {flow["id"]: turnaround_reasons(net, routes[flow["id"]]) for flow in flows}
    removed = {flow_id: reasons for flow_id, reasons in removed.items() if reasons}
    kept = [dict(flow) for flow in flows if flow["id"] not in removed]
    for entry in {flow["from"] for flow in flows}:
        lost = sum(flow["vehs_per_hour"] for flow in flows if flow["from"] == entry and flow["id"] in removed)
        rest = [flow for flow in kept if flow["from"] == entry]
        if lost and not rest:
            raise ValueError(f"진입로 {entry}의 흐름이 모두 지워진다: {sorted(removed)}")
        total = sum(flow["vehs_per_hour"] for flow in rest)
        for flow in rest:
            flow["vehs_per_hour"] = round(flow["vehs_per_hour"] * (1 + lost / total), 3)
    return kept, removed


def main() -> None:
    path = FOLDER / "map.json"
    definition = json.loads(path.read_text(encoding="utf-8"))
    kept, removed = keep_natural(FOLDER / definition["network_source"]["path"], definition["flows"])
    if not removed:
        print("이미 맞췄다: 바꾸지 않는다")
        return
    before = sum(flow["vehs_per_hour"] for flow in definition["flows"])
    definition["flows"] = kept
    definition["network_source"]["patches"]["turnaround_flows_dropped"] = {
        "removed": removed, "gate_m": GATE_M, "detour": DETOUR, "script": "scripts/drop_turnaround_flows.py"}
    path.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    after = sum(flow["vehs_per_hour"] for flow in kept)
    print(f"흐름 {len(removed)}개를 지웠다: {removed}\n배경 차량 합계 {before:.1f} -> {after:.1f}대/시간")


if __name__ == "__main__":
    main()
