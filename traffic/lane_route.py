"""정지선에서 옆 차로로 끝내 못 옮긴 운전자는 지금 차로가 가는 쪽으로 가고 길을 다시 찾는다.

경로의 다음 도로로 가는 연결이 지금 차로에 없으면 SUMO 차는 차로 끝(정지선)에 서서 옆 차로의 빈틈을 기다린다. 대부분은
녹색에 옆 차로가 움직일 때 들어간다. 옆 차로가 꽉 찬 대기열이면(J42 R55 좌회전 차로) 몇 분이고 서서 뒤차를 막는다. SUMO는
이 차를 순간이동으로 치웠지만 순간이동을 껐다(sumo_conn.py). 사람은 신호를 한 번 넘게 보내고도 못 들어가면 지금 차로로
직진하거나 꺾은 뒤 길을 다시 찾는다. 그 행동만 흉내 낸다.

- 정지선에 선 채 GIVE_UP_S를 기다린 차만 바꾼다. 달리는 차와 신호를 기다리는 차는 건드리지 않는다.
- 새 경로는 지금 차로에서 이어지는 도로로 시작한다(순간이동 없음). 회차는 후보가 아니다.
- 원래 목적지가 가까우면 그리로, 멀면 가장 가까운 지도 경계로 나간다.
- 시험용 버스는 건드리지 않는다. 버스 위치를 보지 않는다. update(t)를 매 스텝 부른다(runner의 extras).
"""
from .presence import nearest_boundary_exit

GIVE_UP_S = 200.0        # 가장 긴 신호 주기(J42 182초)보다 길다. 녹색을 한 번은 보고도 못 옮긴 운전자만 포기한다
AT_STOP_LINE_M = 5.0     # 차로 끝까지 이 거리 안이면 정지선 맨 앞 차다(뒤에 선 차는 앞차를 기다리는 중이다)


def reachable_next(net, edge_id, lane_index):
    """edge_id의 lane_index 차로가 연결로 닿을 수 있는 다음 도로 id 집합."""
    lane = net.getEdge(edge_id).getLanes()[lane_index]
    return {c.getTo().getID() for c in lane.getOutgoing()}


def ways_out(net, edge_id, lane_index):
    """그 차로에서 회차 없이 나갈 수 있는 다음 도로 id(정렬)."""
    lane = net.getEdge(edge_id).getLanes()[lane_index]
    return sorted({c.getTo().getID() for c in lane.getOutgoing() if c.getDirection() not in ("t", "T")})


class LaneRouteAdapter:
    def __init__(self, traci, net, bus_id, period=0.5, max_detour_m=500.0):
        self.tc, self.net, self.bus = traci, net, bus_id
        self.period, self.max_detour_m = period, max_detour_m
        self._t_next = -1e9
        self._done = {}                      # 차 id -> 진로를 못 바꾼 도로(같은 도로에서 다시 시도하지 않는다)
        self.changed = 0
        self.errors = []                     # 진로를 못 바꾼 경우(차, 도로, 원래 다음 도로, 새 경로 앞부분, 오류)
        self._len = {}

    def _L(self, edge):
        if edge not in self._len:
            self._len[edge] = self.net.getEdge(edge).getLength()
        return self._len[edge]

    def _new_route(self, route, i, nxt):
        """route[i]에서 nxt로 나간 뒤 원래 목적지로 가는 경로. 크게 돌아야 하면 가까운 지도 경계까지."""
        dest = route[-1]
        tail = None
        if self.net.hasEdge(dest):
            path, _ = self.net.getShortestPath(self.net.getEdge(nxt), self.net.getEdge(dest), vClass="passenger")
            tail = [e.getID() for e in path] if path else None
            # 목적지를 찾아 크게 돌아가지 않는다(R92로 갈 차가 좌회전했다고 1 km를 돌아 R94로 되돌아오는 경로는 만들지 않는다)
            if tail is not None and (sum(self._L(e) for e in tail) > self.max_detour_m or route[i] in tail):
                tail = None
        if tail is None:
            tail = nearest_boundary_exit(self.net, nxt) or [nxt]
        return [route[i]] + tail                 # setRoute는 지금 있는 도로에서 시작하는 경로를 받는다

    def _stuck(self, vid, road):
        v = self.tc.vehicle
        return (v.getSpeed(vid) < 0.1 and v.getWaitingTime(vid) >= GIVE_UP_S
                and self._L(road) - v.getLanePosition(vid) <= AT_STOP_LINE_M)

    def update(self, t):
        if t < self._t_next:
            return
        self._t_next = t + self.period
        tc = self.tc
        ids = set(tc.vehicle.getIDList())
        for vid in list(self._done):
            if vid not in ids:
                del self._done[vid]
        for vid in ids:
            if vid == self.bus:
                continue
            road = tc.vehicle.getRoadID(vid)
            if road.startswith(":") or not self.net.hasEdge(road) or self._done.get(vid) == road:
                continue
            if not self._stuck(vid, road):
                continue
            lane = tc.vehicle.getLaneIndex(vid)
            route, i = tc.vehicle.getRoute(vid), tc.vehicle.getRouteIndex(vid)
            if i + 1 >= len(route) or route[i + 1] in reachable_next(self.net, road, lane):
                continue
            pool = ways_out(self.net, road, lane)
            if not pool:
                continue
            new = self._new_route(route, i, pool[0])
            try:
                tc.vehicle.setRoute(vid, new)
            except Exception as exc:                      # 이어지지 않는 경로 등: 이 차만 건너뛰고 기록한다
                self.errors.append((vid, road, route[i + 1], new[:4], str(exc)[:120]))
                self._done[vid] = road
                continue
            self.changed += 1
