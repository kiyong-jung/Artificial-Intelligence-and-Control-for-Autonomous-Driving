"""ego 경로 위의 고정 정보(차로 연결, 회전, 횡단보도, 정류장)를 경로 거리로 적어 둔다.

도로망에서 한 번만 읽는다. 주행 기록의 state.navigation이 이 값을 쓴다.
경로 거리는 경로 첫 도로의 시작이 0이고, 교차로 내부 길이를 포함한다.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET

from sumolib.net import readNet

TURNS = {"l": "left", "L": "left", "r": "right", "R": "right", "s": "straight", "t": "turnaround"}


class RouteMap:
    def __init__(self, net_path: str, route_id: str, edge_ids: list[str], bus_stops: list[dict], vclass: str):
        net = readNet(net_path, withInternal=True)
        self.route_id = route_id
        self.edge_ids = list(edge_ids)
        edges = [net.getEdge(edge_id) for edge_id in edge_ids]
        usable = [[lane for lane in edge.getLanes() if lane.allows(vclass)] for edge in edges]
        self.drivable_lane_ids = [[lane.getID() for lane in lanes] for lanes in usable]
        self.target_lane_ids, self.start, self.end, turns = [], [0.0], [], []
        self._inside = {}   # 교차로 내부 차로 → 그 차로 앞에 놓인 내부 차로들의 길이
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
                inside, via = 0.0, connection.getViaLaneID()
                while via:
                    self._inside[via] = inside
                    lane = net.getLane(via)
                    inside += lane.getLength()
                    via = next((c.getViaLaneID() for c in lane.getOutgoing()), "")
                longest = max(longest, inside)
            self.start.append(self.end[index] + longest)
            turns.append((self.end[index], TURNS[connections[0].getDirection()]))
        self.turns = [turn for turn in turns if turn[1] != "straight"]
        self.length = self.end[-1]

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
                crossings.append((self.start[index], after))
        self.crossings = sorted(crossings)

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
