"""ego 경로 위의 고정 정보(차로 연결, 회전, 횡단보도, 정류장)를 경로 거리로 적어 둔다.

도로망에서 한 번만 읽는다. 주행 기록의 state.navigation이 이 값을 쓴다.
경로 거리는 경로 첫 도로의 시작이 0이고, 교차로 내부 길이를 포함한다.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from sumolib.net import readNet

TURNS = {"l": "left", "L": "left", "r": "right", "R": "right", "s": "straight", "t": "turnaround"}


def _point_at(points: list, lengths: list[float], distance: float):
    """꺾은선 위 distance(m) 지점. 시작 앞은 첫 구간, 끝 뒤는 마지막 구간을 곧게 늘인다."""
    index = 0
    while index < len(lengths) - 2 and distance > lengths[index + 1]:
        index += 1
    (x0, y0), (x1, y1) = points[index], points[index + 1]
    ratio = (distance - lengths[index]) / (lengths[index + 1] - lengths[index])
    return x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio


def _length_inside_strip(path: list, strip: list, half_width: float, step: float = 0.25) -> float:
    """꺾은선 path의 끝에서 거슬러 올라가며, 중심선 strip에서 half_width 안에 드는 가장 먼 곳까지의 길이(m)."""
    lengths = [0.0]
    for a, b in zip(path, path[1:]):
        lengths.append(lengths[-1] + math.dist(a, b))

    def inside(point):
        for (ax, ay), (bx, by) in zip(strip, strip[1:]):
            dx, dy = bx - ax, by - ay
            ratio = max(0.0, min(1.0, ((point[0] - ax) * dx + (point[1] - ay) * dy) / (dx * dx + dy * dy)))
            if math.hypot(point[0] - ax - ratio * dx, point[1] - ay - ratio * dy) <= half_width:
                return True
        return False

    back = [k * step for k in range(int(lengths[-1] / step) + 1)]
    return max((distance + step for distance in back if inside(_point_at(path, lengths, lengths[-1] - distance))),
               default=0.0)


def curve_speeds(upstream: list, shape: list, lane_length: float, vehicle_length: float, lateral_limit: float,
                 step: float = 0.5) -> list[float]:
    """차로 위치 1m마다 앞범퍼가 그 칸에 있을 때 횡가속도가 lateral_limit을 넘지 않는 속도(m/s).

    SUMO의 차체 방향은 뒤범퍼에서 앞범퍼를 본 방향이다. 차가 s만큼 나아갈 때 그 방향이 도는 비율(rad/m)이 차체가
    느끼는 곡률이고, 횡가속도는 속도² × 곡률이다. 꺾인 점 하나(각 θ)는 차체 길이 L에 걸쳐 θ/L로 퍼지므로, 굽은 차로를
    벗어난 뒤에도 뒤범퍼가 지날 때까지 제한이 이어진다. upstream은 이 차로 앞에 놓인 길의 점들(뒤범퍼가 있는 곳)이다.
    """
    points = [point for point, previous in zip(upstream + shape, [None] + upstream + shape) if point != previous]
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.dist(a, b))
    start = sum(math.dist(a, b) for a, b in zip(upstream, upstream[1:]))
    scale = (lengths[-1] - start) / lane_length   # 차로 위치 → 형상 위 거리

    def heading(distance):
        (bx, by), (fx, fy) = _point_at(points, lengths, distance - vehicle_length), _point_at(points, lengths, distance)
        return math.atan2(fy - by, fx - bx)

    count = int(math.ceil(lane_length / step)) + 1
    headings = [heading(start + (k - 1) * step * scale) for k in range(count + 2)]
    curvature = [abs((b - a + math.pi) % (2.0 * math.pi) - math.pi) / (2.0 * step * scale)
                 for a, b in zip(headings, headings[2:])]
    per_metre = round(1.0 / step)
    speeds = []
    for metre in range(int(math.ceil(lane_length))):
        worst = max(curvature[metre * per_metre:(metre + 1) * per_metre + 1])
        speeds.append(math.sqrt(lateral_limit / worst) if worst > 1e-6 else math.inf)
    return speeds


class RouteMap:
    def __init__(self, net_path: str, route_id: str, edge_ids: list[str], bus_stops: list[dict], vclass: str,
                 lateral_limit: float | None = None, vehicle_length: float = 0.0):
        net = readNet(net_path, withInternal=True)
        self.route_id = route_id
        self.signalized_nodes = {node.getID() for node in net.getNodes() if node.getType().startswith("traffic_light")}
        self.edge_ids = list(edge_ids)
        edges = [net.getEdge(edge_id) for edge_id in edge_ids]
        usable = [[lane for lane in edge.getLanes() if lane.allows(vclass)] for edge in edges]
        self.drivable_lane_ids = [[lane.getID() for lane in lanes] for lanes in usable]
        self.target_lane_ids, self.start, self.end, turns = [], [0.0], [], []
        self._inside = {}   # 교차로 내부 차로 → 그 차로 앞에 놓인 내부 차로들의 길이
        previous_lanes = {}  # 차로 → 경로에서 그 차로 바로 앞에 놓인 차로들
        for index, edge in enumerate(edges):
            self.end.append(self.start[index] + edge.getLength())
            if index + 1 == len(edges):
                self.target_lane_ids.append(self.drivable_lane_ids[index])
                break
            connections = [c for c in edge.getConnections(edges[index + 1]) if c.getFromLane().allows(vclass)]
            self.target_lane_ids.append(sorted({c.getFromLane().getID() for c in connections}))
            # 교차로 내부 길이는 차로마다 다르다. 다음 도로의 시작은 가장 긴 경로에 맞추고,
            # 짧은 경로로 지나는 동안의 거리는 position()이 그 시작을 넘지 않게 자른다.
            longest = 0.0
            for connection in connections:
                inside, via, last = 0.0, connection.getViaLaneID(), connection.getFromLane()
                while via:
                    self._inside[via] = inside
                    lane = net.getLane(via)
                    previous_lanes.setdefault(via, []).append(last)
                    last = lane
                    inside += lane.getLength()
                    via = next((c.getViaLaneID() for c in lane.getOutgoing()), "")
                previous_lanes.setdefault(connection.getToLane().getID(), []).append(last)
                longest = max(longest, inside)
            self.start.append(self.end[index] + longest)
            turns.append((self.end[index], TURNS[connections[0].getDirection()]))
        self.turns = [turn for turn in turns if turn[1] != "straight"]
        self.length = self.end[-1]
        # 경로 끝 도로의 차로 → 그 차로 끝 신호의 (신호 id, 연결 번호). SUMO는 경로 끝 너머의 신호를 알려 주지 않는다.
        self.end_signals = {lane.getID(): [(c.getTLSID(), c.getTLLinkIndex()) for c in lane.getOutgoing() if c.getTLSID()]
                            for lane in usable[-1]}

        crossed = {}
        for edge in ET.parse(net_path).getroot().iter("edge"):
            if edge.get("function") == "crossing":
                node = edge.get("id")[1:].rsplit("_c", 1)[0]
                for edge_id in edge.get("crossingEdges").split():
                    crossed[(node, edge_id)] = edge.get("id")
        crossings = []
        for index, edge in enumerate(edges):
            before = crossed.get((edge.getToNode().getID(), edge.getID()))
            if before:
                crossings.append((self.end[index], before))
            after = crossed.get((edge.getFromNode().getID(), edge.getID()))
            if after and index:
                # 나가는 쪽 횡단보도는 교차로 안에 놓여 이 도로의 시작에서 끝난다. 버스가 회전하며 비스듬히 지나면 띠가
                # 경로 위에서 9~12m다. 띠에 들어서는 곳을 적어야 버스가 횡단보도 위가 아니라 그 앞에 선다.
                strip = net.getLane(after + "_0")
                entries = []
                for lane in usable[index]:
                    for previous in previous_lanes.get(lane.getID(), []):
                        path = []
                        while previous.getID().startswith(":"):
                            path = list(previous.getShape()) + path
                            previous = previous_lanes[previous.getID()][0]
                        if len(path) > 1:
                            path_length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
                            entries.append(self.end[index - 1] + path_length
                                           - _length_inside_strip(path, list(strip.getShape()), strip.getWidth() / 2.0))
                crossings.append((min(entries + [self.start[index]]), after))
        self.crossings = sorted(crossings)
        # 보행자가 우선인 횡단보도: 보도(walkingarea)에서 횡단보도로 가는 연결이 신호 없이 우선(M)이다.
        self.pedestrian_priority = {
            connection.get("to") for connection in ET.parse(net_path).getroot().iter("connection")
            if connection.get("to", "").startswith(":") and "_c" in connection.get("to")
            and connection.get("from", "").startswith(":") and "_w" in connection.get("from")
            and connection.get("state") == "M" and connection.get("tl") is None}

        # 차로 → 차로 위치 1m마다의 곡선 속도 제한(m/s). 차로 제한속도보다 낮아지는 곳이 없는 차로는 넣지 않는다.
        self._curve_speeds = {}
        if lateral_limit:
            def upstream(lane, need):
                """lane 시작 앞 need(m)만큼의 길(앞에 놓인 차로가 여럿이면 첫째)."""
                if need <= 0.0 or not previous_lanes.get(lane.getID()):
                    return []
                previous = previous_lanes[lane.getID()][0]
                return upstream(previous, need - previous.getLength()) + list(previous.getShape())

            lanes = {lane.getID(): lane for group in usable for lane in group}
            lanes.update({lane_id: net.getLane(lane_id) for lane_id in previous_lanes if lane_id.startswith(":")})
            for lane_id, lane in lanes.items():
                profiles = [curve_speeds(upstream(previous, vehicle_length - previous.getLength()) + list(previous.getShape())
                                         if previous else [], list(lane.getShape()), lane.getLength(), vehicle_length,
                                         lateral_limit)
                            for previous in previous_lanes.get(lane_id) or [None]]
                speeds = [min(values) for values in zip(*profiles)]
                if min(speeds) < lane.getSpeed():
                    self._curve_speeds[lane_id] = speeds

        order = {edge_id: index for index, edge_id in enumerate(edge_ids)}
        stops = []
        for stop in bus_stops:
            edge_id = stop["lane"].rsplit("_", 1)[0]
            if edge_id in order:
                base = self.start[order[edge_id]]
                stops.append((base + float(stop["startPos"]), base + float(stop["endPos"]), stop["id"], stop["lane"]))
        self.stops = sorted(stops)

    def position(self, edge_index: int, lane_id: str, lane_position: float) -> float:
        """경로 거리. 교차로 안에서는 edge_index가 방금 지난 도로다(SUMO의 getRouteIndex와 같다)."""
        if not lane_id.startswith(":"):
            return self.start[edge_index] + lane_position
        inside = self.end[edge_index] + self._inside.get(lane_id, 0.0) + lane_position
        return min(inside, self.start[min(edge_index + 1, len(self.start) - 1)])

    def curve_speed(self, lane_id: str, lane_position: float) -> float:
        """앞범퍼가 이 차로의 이 위치에 있을 때의 곡선 속도 제한(m/s). 제한이 없으면 inf다."""
        speeds = self._curve_speeds.get(lane_id)
        if speeds is None:
            return math.inf
        return speeds[min(max(int(lane_position), 0), len(speeds) - 1)]

    def curve_speeds_ahead(self, lane_id: str, lane_position: float = 0.0):
        """이 차로의 lane_position부터 끝까지 (차로 위치, 곡선 속도 제한)을 1m 간격으로 낸다."""
        speeds = self._curve_speeds.get(lane_id, ())
        for metre in range(max(int(lane_position), 0), len(speeds)):
            yield max(float(metre), lane_position), speeds[metre]

    def ahead(self, position: float) -> dict:
        """position 앞에 있는 첫 회전, 횡단보도, 정류장. 없으면 None이다. 정류장 안에 있으면 거리는 0이다."""
        turn = next(((at, name) for at, name in self.turns if at >= position), None)
        crossing = next(((at, name) for at, name in self.crossings if at >= position), None)
        stop = next((item for item in self.stops if item[1] >= position), None)
        return {
            "next_turn": turn[1] if turn else None,
            "distance_to_turn": turn[0] - position if turn else None,
            "crossing_id": crossing[1] if crossing else None,
            "distance_to_crossing": crossing[0] - position if crossing else None,
            "next_stop_id": stop[2] if stop else None,
            "distance_to_stop": max(stop[0] - position, 0.0) if stop else None,
            "distance_to_destination": self.length - position,
        }
