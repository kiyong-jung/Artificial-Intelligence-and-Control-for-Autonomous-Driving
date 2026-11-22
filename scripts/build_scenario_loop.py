r"""연구 시나리오 전용 합성 도로(maps/scenario_loop)를 만든다.

    python scripts/build_scenario_loop.py

실제 지도가 아니라 팀 스케치를 옮긴 도로다. 직사각형 하나와 그 가운데를 가로지르는 가로 도로 하나로,
"日" 모양이다. 좌표는 m 단위의 평면 좌표다.

    X ── TL ─────────────── ┐      위쪽 절반도 양방향 2+2차로다. X는 진출입로(경계)
         │         N        │      S5 차량: X → 위쪽 → MR에서 우회전해 가운데 도로로 끼어듦 → ML에서 우회전 → X
         │         ↓ 진입로  │      N → M: 가운데 도로 한가운데로 들어오는 짧은 일방통행 진입로(60m, 2차로)
         ML ←───── M ────── MR    가운데 도로와 아래쪽 절반은 양방향 3+3차로, 진출입로는 2+2차로다
         │  ↓    버스 경로    ↑ │
    정류장(S3·S4)              K 횡단보도(S6)
         │ (S2)               │
         └──────── B ─정류장───┘    버스(ego)는 아래쪽 절반을 반시계로 돈다(B → K → MR → M → ML → B)

- MR: 오른쪽 위에서 내려온 S5 차량이 우회전해, 좌회전으로 들어오는 버스와 같은 가운데 도로로 합류한다(신호 없음).
- M: 진입로 차량이 신호 없이 가운데 도로로 합류한다. 진입로가 양보한다.
- ML: 가운데 도로 0번 차로는 위쪽 우회전(진출)만, 1·2번 차로만 아래쪽 좌회전이다. 좌회전한 버스는 곧바로
  오른쪽 차로의 서쪽 정류장으로 옮겨야 한다(S3).
- K의 횡단보도는 신호가 없고 보행자 비우선이다(합정역 S6와 같다).

임시 폴더에서 도로망을 만들고 validate_structure로 확인한 뒤에만 maps/scenario_loop/의 파일을 바꿔 넣는다.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import sumolib

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.build_road import publish, shortest_route
from scripts.patch_network import digest
from scripts.validate_road import validate_structure

FOLDER = BASE / "maps" / "scenario_loop"
SOURCE = FOLDER / "source"
VERSION = "5.0.0"
MAIN_KMH, LINK_KMH = 50, 30
LANE_WIDTH = 3.2
CORNER_RADIUS = 30.0
EGO_ROUTE = "bus_loop"

WIDTH, LOWER, UPPER = 400, 320, 240     # 직사각형 폭, 아래쪽 절반 높이, 위쪽 절반 높이
MIDDLE, TOP = LOWER, LOWER + UPPER
# 교차로(노드) 좌표. 직사각형의 나머지 모서리(BL, BR, TR)는 도로 형상 안의 둥근 꺾임이다.
NODES = {
    "ML": (0, MIDDLE),          # 가운데 도로 서쪽 끝
    "MR": (WIDTH, MIDDLE),      # 가운데 도로 동쪽 끝. S5 끼어들기
    "K": (WIDTH, 200),          # 신호 없는 횡단보도(S6)
    "B": (WIDTH / 2, 0),        # 아래 변 가운데. 버스 경로의 시작과 끝
    "TL": (0, TOP),             # 북서 모서리. 진출입로가 서쪽으로 나간다
    "X": (-200, TOP),           # 진출입로 끝(경계). S5 차량이 나가는 곳
    "M": (WIDTH / 2, MIDDLE),   # 가운데 도로 한가운데. 진입로 합류점
    "N": (WIDTH / 2, MIDDLE + 60),   # 진입로 시작(경계)
}
# 도로: id → (시작, 끝, 중간 꺾임점, 차로 수, 제한속도 km/h, 우선순위, 양방향, 인도, 이름)
# 양방향 도로의 반대 방향 id는 시작·끝을 바꾼 것이다(A_B ↔ B_A). 꺾임점은 원호로 둥글게 만든다.
ROADS = {
    "B_K": ("B", "K", [(WIDTH, 0)], 3, MAIN_KMH, 3, True, False, "아래 변(동쪽 절반)과 오른쪽 변 아래"),
    "K_MR": ("K", "MR", [], 3, MAIN_KMH, 3, True, True, "오른쪽 변 아래(횡단보도 북쪽)"),
    "MR_M": ("MR", "M", [], 3, MAIN_KMH, 3, True, False, "가운데 가로 도로(동쪽 절반)"),
    "M_ML": ("M", "ML", [], 3, MAIN_KMH, 3, True, False, "가운데 가로 도로(서쪽 절반)"),
    "ML_B": ("ML", "B", [(0, 0)], 3, MAIN_KMH, 3, True, False, "왼쪽 변 아래와 아래 변(서쪽 절반)"),
    "TL_MR": ("TL", "MR", [(WIDTH, TOP)], 2, MAIN_KMH, 2, True, False, "위 변과 오른쪽 변 위"),
    "ML_TL": ("ML", "TL", [], 2, MAIN_KMH, 2, True, False, "왼쪽 변 위"),
    "X_TL": ("X", "TL", [], 2, LINK_KMH, 2, True, False, "진출입로"),
    "N_M": ("N", "M", [], 2, LINK_KMH, 1, False, False, "가운데 도로 진입로(일방통행)"),
}
# 인도는 횡단보도가 있는 K에 닿는 도로에만 둔다. B_K는 K 남쪽 구간이다.
SIDEWALK_EDGES = {"B_K", "K_B", "K_MR", "MR_K"}


def reverse(road_id: str) -> str:
    return "_".join(reversed(road_id.split("_")))


# ML: 가운데 도로 0번 차로는 위쪽 우회전만, 1·2번 차로는 아래쪽 좌회전만.
CONNECTIONS = [("M_ML", 0, "ML_TL", 0), ("M_ML", 0, "ML_TL", 1),
               ("M_ML", 1, "ML_B", 0), ("M_ML", 1, "ML_B", 1), ("M_ML", 2, "ML_B", 2)]
CROSSING = {"node": "K", "edges": ["B_K", "K_B"], "width": 4.0}

ROUTES = {
    EGO_ROUTE: ("[연구] 아래 변 가운데 → 남쪽 정류장(S4) → 신호 없는 횡단보도(S6) → MR 좌회전(S5 합류 지점) → "
                "가운데 도로(진입로 합류점 M) → ML 좌회전 → 서쪽 정류장 접근(S3)·정차(S4) → 남서 모서리(S2) → 아래 변 가운데",
                ["B_K", "K_MR", "MR_M", "M_ML", "ML_B"]),
}
# 정류장: (id, 이름, 도로, 정류장 끝 위치를 정할 좌표). 오른쪽 끝 차도 차로에 25m로 놓는다.
BUS_STOPS = [
    ("stop_south", "남쪽 정류장", "B_K", (285, 0)),
    ("stop_west", "서쪽 정류장", "ML_B", (0, 200)),
]
# baseline 배경 흐름(대/시간). 합성 수요다. 경계는 X 하나뿐이라 아래쪽 순환로 위 도로에서도 차를 넣고 뺀다.
# (출발, 도착, 대/시간, 경유 도로, 설명). 위쪽 절반이 양방향이라 X로 돌아가는 길은 시계·반시계 길이가 같다.
# S5 흐름은 가운데 도로 동쪽 절반(MR_M)을 경유하게 고정해 MR에서 끼어들게 한다.
FLOWS = [
    ("X_TL", "TL_X", 150, ["MR_M"], "S5: 위쪽을 돌아 MR에서 가운데 도로로 끼어든 뒤 ML에서 우회전해 진출로로 나간다"),
    ("X_TL", "B_K", 150, ["MR_M"], "가운데 도로에서 ML 좌회전, 버스와 같은 방향으로 왼쪽 변·아래 변"),
    ("B_K", "TL_X", 150, ["MR_M"], "버스와 같은 차로로 오른쪽 변·가운데 도로를 지나 ML에서 우회전해 진출"),
    ("X_TL", "B_ML", 100, ["MR_K"], "MR에서 직진해 아래쪽 절반을 반대 방향(시계)으로"),
    ("ML_M", "MR_K", 80, [], "가운데 도로 동행 → MR 우회전 → 오른쪽 변 남행"),
    ("N_M", "TL_X", 80, [], "진입로 → M에서 우회전해 버스 진행 방향으로 합류 → ML에서 우회전해 진출"),
    ("N_M", "B_K", 60, [], "진입로 → M 우회전 → ML 좌회전, 버스와 같은 방향으로 왼쪽 변·아래 변"),
    ("N_M", "MR_K", 40, [], "진입로 → M 좌회전 → MR 우회전 → 오른쪽 변 남행"),
]
PEDESTRIANS_PER_HOUR = 120   # 한 방향. S6 횡단보도를 양방향으로 건넌다.


def rounded(points: list[tuple[float, float]], radius: float = CORNER_RADIUS, steps: int = 8) -> list[tuple[float, float]]:
    """꺾임점을 원호에 가까운 곡선으로 바꾼 폴리라인. 양 끝점은 그대로 둔다."""
    result = [points[0]]
    for before, corner, after in zip(points, points[1:], points[2:]):
        a = (before[0] - corner[0], before[1] - corner[1])
        b = (after[0] - corner[0], after[1] - corner[1])
        la, lb = math.hypot(*a), math.hypot(*b)
        ua, ub = (a[0] / la, a[1] / la), (b[0] / lb, b[1] / lb)
        angle = math.acos(max(-1.0, min(1.0, ua[0] * ub[0] + ua[1] * ub[1])))
        cut = min(radius / math.tan(angle / 2), la / 2, lb / 2)
        start = (corner[0] + ua[0] * cut, corner[1] + ua[1] * cut)
        end = (corner[0] + ub[0] * cut, corner[1] + ub[1] * cut)
        for index in range(steps + 1):   # 2차 베지어. 반지름 30m에서 원호와의 차이는 수십 cm다
            t = index / steps
            result.append(((1 - t) ** 2 * start[0] + 2 * (1 - t) * t * corner[0] + t ** 2 * end[0],
                           (1 - t) ** 2 * start[1] + 2 * (1 - t) * t * corner[1] + t ** 2 * end[1]))
    result.append(points[-1])
    return result


def shape_text(points) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def write_plain(folder: Path) -> list[Path]:
    nodes = ET.Element("nodes")
    for node_id, (x, y) in NODES.items():
        ET.SubElement(nodes, "node", id=node_id, x=str(x), y=str(y), type="dead_end" if node_id in ("X", "N") else "priority")
    edges = ET.Element("edges")
    for road_id, (start, end, bends, lanes, kmh, priority, two_way, _sidewalk, name) in ROADS.items():
        points = rounded([NODES[start], *bends, NODES[end]])
        directions = [(road_id, start, end, points)]
        if two_way:
            directions.append((reverse(road_id), end, start, points[::-1]))
        for edge_id, frm, to, shape in directions:
            attributes = dict(id=edge_id, **{"from": frm, "to": to}, numLanes=str(lanes),
                              speed=f"{kmh / 3.6:.2f}", priority=str(priority), width=str(LANE_WIDTH),
                              name=name, shape=shape_text(shape), spreadType="right" if two_way else "center")
            if edge_id in SIDEWALK_EDGES:
                attributes["sidewalkWidth"] = "2.0"
            else:
                attributes["disallow"] = "pedestrian"
            ET.SubElement(edges, "edge", attributes)
    connections = ET.Element("connections")
    for frm, from_lane, to, to_lane in CONNECTIONS:
        ET.SubElement(connections, "connection", **{"from": frm, "to": to}, fromLane=str(from_lane), toLane=str(to_lane))
    ET.SubElement(connections, "crossing", node=CROSSING["node"], edges=" ".join(CROSSING["edges"]),
                  priority="false", width=str(CROSSING["width"]))
    paths = []
    for name, root in (("loop.nod.xml", nodes), ("loop.edg.xml", edges), ("loop.con.xml", connections)):
        ET.indent(root, space="    ")
        ET.ElementTree(root).write(folder / name, encoding="utf-8", xml_declaration=True)
        paths.append(folder / name)
    return paths


def build_network(folder: Path) -> Path:
    nod, edg, con = write_plain(folder)
    output = folder / "network.net.xml"
    completed = subprocess.run(
        [sumolib.checkBinary("netconvert"), "--node-files", str(nod), "--edge-files", str(edg),
         "--connection-files", str(con), "--output-file", str(output),
         "--no-turnarounds", "true", "--junctions.corner-detail", "5", "--junctions.limit-turn-speed", "5.5",
         "--walkingareas", "true", "--offset.disable-normalization", "true"],
        capture_output=True, text=True, timeout=120)
    if completed.returncode:
        raise RuntimeError("netconvert 실패: " + completed.stderr[-3000:])
    # 머리 주석의 생성 시각·임시 경로를 지워 같은 입력이면 같은 바이트가 되게 한다.
    ET.ElementTree(ET.parse(output).getroot()).write(output, encoding="utf-8", xml_declaration=True)
    return output


def check_connections(net) -> None:
    """ML의 차로 배정이 그대로 들어갔는지 본다."""
    expected = {("M_ML", "ML_TL"): {(0, 0), (0, 1)}, ("M_ML", "ML_B"): {(1, 0), (1, 1), (2, 2)}}
    for (frm, to), lanes in expected.items():
        found = {(c.getFromLane().getIndex(), c.getToLane().getIndex())
                 for c in net.getEdge(frm).getConnections(net.getEdge(to))}
        if found != lanes:
            raise ValueError(f"{frm} → {to} 차로 연결이 {sorted(found)}입니다. 기대값 {sorted(lanes)}")


def bus_stops(net) -> list[dict]:
    stops = []
    for stop_id, name, edge_id, point in BUS_STOPS:
        edge = net.getEdge(edge_id)
        lane = next(lane for lane in edge.getLanes() if lane.allows("bus"))
        offset, _distance = sumolib.geomhelper.polygonOffsetAndDistanceToPoint(point, lane.getShape())
        end = min(max(offset, 25.5), lane.getLength() - 0.5)
        stops.append({"id": stop_id, "name": name, "lane": lane.getID(), "startPos": round(end - 25, 2),
                      "endPos": round(end, 2), "lanes_on_edge": sum(l.allows("bus") for l in edge.getLanes())})
    return stops


def write_pedestrians(path: Path, net) -> None:
    """S6 횡단보도를 양쪽에서 건너는 보행자. 인도는 K에 닿는 네 도로에만 있다."""
    near, period = 15.0, 3600 / PEDESTRIANS_PER_HOUR
    routes = ET.Element("routes")
    outside, inside = net.getEdge("B_K"), net.getEdge("K_B")   # B_K의 인도는 바깥(동쪽), K_B의 인도는 안쪽(서쪽)
    for flow_id, frm, depart, to, arrive, begin in (
            ("ped_out_to_in", outside, outside.getLength() - near, inside, near, 0.0),
            ("ped_in_to_out", inside, near, outside, outside.getLength() - near, period / 2)):
        flow = ET.SubElement(routes, "personFlow", id=flow_id, begin=f"{begin:.1f}", end="36000",
                             period=f"{period:.1f}", departPos=f"{depart:.2f}")
        ET.SubElement(flow, "walk", **{"from": frm.getID(), "to": to.getID()}, arrivalPos=f"{arrive:.2f}")
    ET.indent(routes, space="    ")
    ET.ElementTree(routes).write(path, encoding="utf-8", xml_declaration=True)


def write_context(path: Path) -> None:
    """직사각형의 두 칸을 잔디로 칠한다. 보기 위한 것이다."""
    additional = ET.Element("additional")
    m = 12
    for poly_id, (y0, y1) in (("lower_green", (m, MIDDLE - m)), ("upper_green", (MIDDLE + m, TOP - m))):
        corners = [(m, y0), (m, y1), (WIDTH - m, y1), (WIDTH - m, y0), (m, y0), (m, y1)]
        ET.SubElement(additional, "poly", id=poly_id, type="grass", color="0.80,0.88,0.74", fill="true", layer="-5",
                      shape=shape_text(rounded(corners, 18)[1:-1]))
    ET.indent(additional, space="    ")
    ET.ElementTree(additional).write(path, encoding="utf-8", xml_declaration=True)


def build() -> dict:
    with tempfile.TemporaryDirectory(prefix="scenario_loop_build_") as name:
        temporary = Path(name)
        net_path = build_network(temporary)
        net = sumolib.net.readNet(str(net_path))
        check_connections(net)
        write_pedestrians(temporary / "pedestrians.rou.xml", net)
        write_context(temporary / "context.add.xml")
        routes, route_design = {}, {}
        for route_id, (description, anchors) in ROUTES.items():
            edges = routes[route_id] = shortest_route(net, anchors, route_id)
            if edges != anchors:
                raise ValueError(f"{route_id}: 경로가 {edges}로 이어집니다. 기대값 {anchors}")
            lanes = [lane for edge in edges for lane in net.getEdge(edge).getLanes() if lane.allows("passenger")]
            route_design[route_id] = {
                "description": description,
                "length_m": round(sum(net.getEdge(edge).getLength() for edge in edges), 2),
                "max_speed_mps": max(lane.getSpeed() for lane in lanes),
                "first_edge_lanes": sum(lane.allows("passenger") for lane in net.getEdge(edges[0]).getLanes()),
            }
        flows = []
        for frm, to, vehs_per_hour, via, note in FLOWS:
            stops = [frm, *via, to]
            for left, right in zip(stops, stops[1:]):
                if net.getShortestPath(net.getEdge(left), net.getEdge(right), vClass="passenger")[0] is None:
                    raise ValueError(f"배경 흐름 {frm} → {to}: {left} → {right} 경로가 없습니다.")
            flow = {"id": f"f_{frm}_to_{to}", "from": frm, "to": to, "vehs_per_hour": vehs_per_hour, "note": note}
            if via:
                flow["via"] = via
            flows.append(flow)
        xmin, ymin, xmax, ymax = net.getBoundary()
        definition = {
            "id": "scenario_loop", "name": "연구 시나리오 도로(직사각형 + 가운데 가로 도로)", "version": VERSION,
            "description": "실제 지도가 아닌 합성 도로. 직사각형과 가운데 가로 도로로 된 日 모양이다. 버스는 아래쪽 절반을 "
                           "반시계로 돌며 가운데 도로 끼어들기(S5), 가로변 정류장 2곳(S3·S4), 신호 없는 횡단보도(S6)를 지난다. "
                           "위쪽 절반은 양방향 도로이고 S5 차량이 돌아 나가는 길이다.",
            "source_status": "synthetic",
            "network_source": {"path": "source/network.net.xml", "sha256": digest(net_path),
                               "generator": "scripts/build_scenario_loop.py"},
            "extra_files": {"routes": ["source/pedestrians.rou.xml"]},
            "ego_vehicle": "bus",
            # 평면 좌표 도로라 위경도 대신 실행망 좌표로 GUI 범위를 준다.
            "focus_xy": [round(xmin, 2), round(ymin, 2), round(xmax, 2), round(ymax, 2)],
            "gui_center": {"x": round((xmin + xmax) / 2, 3), "y": round((ymin + ymax) / 2, 3)},
            "assumptions": ["도로 형상은 팀 스케치를 옮긴 것이다. 치수(폭 400m, 아래 320m, 위 240m)는 임의로 정했다.",
                            "제한속도는 50km/h, 진출입로 30km/h다. 신호는 없다.",
                            f"배경 흐름은 합성 수요다(baseline 합계 {sum(f[2] for f in FLOWS)}대/시간). quiet·baseline·busy는 0.5·1·1.25배다.",
                            f"S6 보행자는 방향마다 시간당 {PEDESTRIANS_PER_HOUR}명이다."],
            "routes": routes, "route_design": route_design, "flows": flows,
            "max_lane_speed_mps": max(lane.getSpeed() for edge in net.getEdges() for lane in edge.getLanes()
                                      if lane.allows("passenger")),
            "ego_route": EGO_ROUTE, "bus_stops": bus_stops(net),
            "design": {"measure_lanes": ["M_ML_0", "M_ML_1", "ML_B_0", "ML_B_1"],
                       "zones": {
                           "s5_cutin": {"alley": "TL_MR", "joins": "MR_M", "node": "MR",
                                        "note": "위쪽에서 내려온 차량이 우회전해, 좌회전해 들어오는 버스와 같은 가운데 도로로 신호 없이 합류"},
                           "s3_s4_west_stop": {"approach": "ML_B", "node": "ML",
                                               "note": "ML 좌회전 직후 오른쪽 차로의 서쪽 정류장으로 이동, 정차"},
                           "entry_merge": {"alley": "N_M", "joins": "M_ML", "node": "M",
                                           "note": "가운데 도로 한가운데로 들어오는 짧은 진입로. 신호 없이 진입로가 양보"},
                           "ml_lanes": {"approach": "M_ML", "left_to": "ML_B",
                                        "note": "가운데 도로 0번 차로는 위쪽 우회전(진출)만, 1·2번 차로만 좌회전"},
                           "s6_crossing": {"node": "K", "edges": ["B_K", "K_B"], "note": "신호 없는 보행자 비우선 횡단보도"},
                           "s5_exit": {"edges": ["ML_TL", "TL_X"], "note": "S5 차량이 나가는 길"},
                       }},
        }
        validate_structure(definition, net_path)
        SOURCE.mkdir(parents=True, exist_ok=True)
        publish({SOURCE / "network.net.xml": net_path.read_bytes(),
                 SOURCE / "context.add.xml": (temporary / "context.add.xml").read_bytes(),
                 SOURCE / "pedestrians.rou.xml": (temporary / "pedestrians.rou.xml").read_bytes(),
                 FOLDER / "map.json": (json.dumps(definition, ensure_ascii=False, indent=2) + "\n").encode("utf-8")})
    return definition


if __name__ == "__main__":
    result = build()
    design = result["route_design"][EGO_ROUTE]
    print(f"scenario_loop {result['version']}: 버스 경로 {design['length_m']}m, 정류장 {len(result['bus_stops'])}곳, "
          f"배경 흐름 {len(result['flows'])}개, 실행망 sha256 {result['network_source']['sha256'][:12]}")
