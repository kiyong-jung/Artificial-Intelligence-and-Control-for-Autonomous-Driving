"""도로망에서 이상이 나는 구간을 위치와 함께 찾는다.

    python scripts/audit_road.py --map hapjeong --out results/road_audit/hapjeong.json

validate_road.py는 충돌과 순간이동의 횟수만 센다. 이 스크립트는 어디서 났는지를 적는다.
- 구조: 차가 다니는 차로인데 다음 도로로 가는 연결이 없는 차로, 좌회전이 가장 왼쪽 차로가 아닌 곳에서
  시작하는 연결, 버스 경로에서 다음 도로로 이어지는 차로.
- 실행: 배경 교통에 SUMO가 모는 버스를 일정 간격으로 넣고(정류장 정차 포함) 충돌, 순간이동, 비상 제동,
  비상 정지를 차로별로 모은다. 버스의 주행 시간과 대기 시간도 적는다.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from sumolib import checkBinary
from sumolib.net import readNet

from env import road_builder
from env.road_config import EGO, ROAD, TRAFFIC, bus_vtype_attributes, configure_map
from maps import available_maps, load_map

BUS_PERIOD_S = 120
STOP_DURATION_S = 8
WARNING = re.compile(r"^Warning: (.*)$", re.MULTILINE)
LANE = re.compile(r"lane[= ]'([^']+)'")
KINDS = (("teleport", "Teleporting vehicle"), ("emergency_braking", "performs emergency braking"),
         ("emergency_stop", "performs emergency stop"), ("collision", "collision with"))


def drives(lane) -> bool:
    return lane.allows("passenger") or lane.allows("bus")


def structure(definition: dict) -> dict:
    """연결이 없는 차로, 바깥쪽에서 시작하는 좌회전, 버스 경로의 차로 연결을 도로망에서 읽는다."""
    net = readNet(definition["net_path"], withInternal=True)
    dead_lanes, outer_left_turns = [], []
    for edge in net.getEdges(withInternal=False):
        lanes = [lane for lane in edge.getLanes() if drives(lane)]
        # 경계로 나가는 도로(끝 노드의 이웃이 출발 노드뿐)의 차로는 끊긴 것이 아니라 도착지다.
        end = edge.getToNode()
        neighbours = ({e.getToNode() for e in end.getOutgoing() if not e.getID().startswith(":")}
                      | {e.getFromNode() for e in end.getIncoming() if not e.getID().startswith(":")})
        if neighbours == {edge.getFromNode()}:
            continue
        for lane in lanes:
            if not lane.getOutgoing():
                dead_lanes.append({"lane": lane.getID(), "junction": edge.getToNode().getID(),
                                   "length_m": round(lane.getLength(), 1)})
        leftmost = max((lane.getIndex() for lane in lanes), default=None)
        for lane in lanes:
            for connection in lane.getOutgoing():
                # 좌회전하는 차로보다 왼쪽에 직진 차로가 있으면 두 흐름이 교차로 안에서 엇갈린다.
                if connection.getDirection() in ("l", "L") and lane.getIndex() < leftmost and any(
                        other.getDirection() == "s" for inner in lanes if inner.getIndex() > lane.getIndex()
                        for other in inner.getOutgoing()):
                    outer_left_turns.append({"from": lane.getID(), "to": connection.getToLane().getID(),
                                             "junction": edge.getToNode().getID(),
                                             "signalized": edge.getToNode().getType().startswith("traffic_light")})
    route = definition["routes"][definition["ego_route"]]
    route_lanes = []
    for current, following in zip(route, route[1:]):
        edge = net.getEdge(current)
        usable = sorted({c.getFromLane().getIndex() for c in edge.getConnections(net.getEdge(following))})
        route_lanes.append({"edge": current, "next": following,
                            "drivable_lanes": [lane.getIndex() for lane in edge.getLanes() if drives(lane)],
                            "lanes_to_next": usable})
    return {"dead_lanes": dead_lanes, "outer_left_turns": outer_left_turns, "route_lanes": route_lanes}


def bus_routes(definition: dict) -> str:
    """SUMO가 모는 버스 흐름. 경로 위 정류장에 순서대로 선다."""
    route = definition["routes"][definition["ego_route"]]
    order = {edge: index for index, edge in enumerate(route)}
    stops = sorted((stop for stop in definition.get("bus_stops", []) if stop["lane"].rsplit("_", 1)[0] in order),
                   key=lambda stop: (order[stop["lane"].rsplit("_", 1)[0]], stop["startPos"]))
    root = ET.Element("routes")
    ET.SubElement(root, "vType", {"id": "audit_bus", **bus_vtype_attributes()})
    flow = ET.SubElement(root, "flow", id="audit_bus", type="audit_bus", begin="60", end="100000",
                         period=str(BUS_PERIOD_S), departLane="best", departSpeed="0")
    ET.SubElement(flow, "route", edges=" ".join(route))
    for stop in stops:
        ET.SubElement(flow, "stop", busStop=stop["id"], duration=str(STOP_DURATION_S))
    return ET.tostring(root, encoding="unicode")


def run(map_id: str, scenario: str, seed: int, seconds: float) -> dict:
    configure_map(scenario, seed, None, map_id)
    definition = ROAD["network"]
    with tempfile.TemporaryDirectory() as temporary:
        folder = Path(temporary)
        cfg = Path(road_builder.build(ROAD, EGO, TRAFFIC, str(folder)))
        (folder / "audit_bus.rou.xml").write_text(bus_routes(definition), encoding="utf-8")
        inputs = ET.parse(cfg).getroot().find("input")
        completed = subprocess.run([
            checkBinary("sumo"), "-c", str(cfg),
            "--route-files", "audit_bus.rou.xml," + inputs.find("route-files").get("value"),
            "--end", str(seconds), "--time-to-teleport", "300", "--no-step-log", "true",
            "--duration-log.disable", "true", "--collision-output", "collisions.xml",
            "--tripinfo-output", "tripinfo.xml"], cwd=folder, capture_output=True, text=True, timeout=1800)
        if completed.returncode:
            raise RuntimeError(completed.stderr[-2000:])
        warnings = WARNING.findall(completed.stderr)
        events = []
        for text in warnings:
            kind = next((name for name, marker in KINDS if marker in text), None)
            lane = LANE.search(text)
            if kind and kind != "collision":
                events.append({"kind": kind, "lane": lane.group(1) if lane else None,
                               "bus": "audit_bus" in text, "text": text[:220]})
        for collision in ET.parse(folder / "collisions.xml").getroot().iter("collision"):
            events.append({"kind": "collision", "lane": collision.get("lane"),
                           "bus": "audit_bus" in collision.get("collider") + collision.get("victim"),
                           "text": f"{collision.get('type')}: {collision.get('collider')}({collision.get('colliderType')})"
                                   f" → {collision.get('victim')}({collision.get('victimType')}) t={collision.get('time')}"})
        buses = [trip for trip in ET.parse(folder / "tripinfo.xml").getroot().iter("tripinfo")
                 if trip.get("id").startswith("audit_bus")]
    mean = lambda key: round(sum(float(bus.get(key)) for bus in buses) / len(buses), 1) if buses else None
    return {"scenario": scenario, "seed": seed, "events": events,
            "bus_trips": len(buses), "bus_duration_s": mean("duration"), "bus_waiting_s": mean("waitingTime"),
            "bus_stop_time_s": mean("stopTime")}


def main() -> int:
    parser = argparse.ArgumentParser(description="도로망의 이상 구간을 위치와 함께 찾는다")
    parser.add_argument("--map", choices=available_maps(), default="hapjeong")
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 7, 42])
    parser.add_argument("--scenarios", nargs="+", default=["quiet", "baseline", "busy"])
    parser.add_argument("--seconds", type=float, default=1800)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    report = {"map": args.map, "structure": structure(load_map(args.map)), "seconds": args.seconds}
    jobs = [(args.map, scenario, seed, args.seconds) for scenario in args.scenarios for seed in args.seeds]
    with ProcessPoolExecutor() as pool:
        report["runs"] = list(pool.map(run, *zip(*jobs)))

    found = report["structure"]
    print(f"연결 없는 차로 {len(found['dead_lanes'])}개: " + ", ".join(lane["lane"] for lane in found["dead_lanes"]))
    print(f"바깥쪽에서 시작하는 좌회전 {len(found['outer_left_turns'])}개: "
          + ", ".join(f"{turn['from']}→{turn['to']}" for turn in found["outer_left_turns"]))
    print("버스 경로에서 다음 도로로 이어지는 차로(차도 차로 중):")
    for row in found["route_lanes"]:
        mark = "" if row["lanes_to_next"] == row["drivable_lanes"] else "  ← 일부 차로만"
        print(f"  {row['edge']:>9} → {row['next']:<9} {row['lanes_to_next']} / {row['drivable_lanes']}{mark}")
    print("\n실행별 사건 수와 버스:")
    for row in report["runs"]:
        counts = Counter(event["kind"] for event in row["events"])
        print(f"  {row['scenario']:>8} seed {row['seed']:>2}: " + ", ".join(f"{name} {counts[name]}" for name, _ in KINDS)
              + f" | 버스 {row['bus_trips']}대 완주, 평균 {row['bus_duration_s']}초(대기 {row['bus_waiting_s']}초)")
    places = Counter((event["kind"], event["lane"], event["bus"]) for row in report["runs"] for event in row["events"])
    print("\n사건이 난 차로(종류, 차로, 버스 관련 여부, 횟수):")
    for (kind, lane, bus), count in places.most_common():
        print(f"  {kind:<18} {str(lane):<22} {'버스' if bus else '    '} {count}")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
