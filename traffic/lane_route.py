"""지금 있는 차로가 정하는 진로. 미리 정해 둔 경로의 다음 도로로 가는 연결이 지금 차로에 없고 정지선이 가까우면(차로를 바꿀 수
없으면) 그 차로가 갈 수 있는 도로로 진로를 바꾼다.

예) 잔다리로 R94(J42 진입)는 차도가 두 차로다. 차로 1은 우회전·직진·좌회전이고 차로 2는 좌회전뿐이다. 우회전할 차가 어쩔 수
없이 차로 2에 있으면 좌회전한다(우회전 차로로 못 옮겨 정지선에서 뒷차를 막던 것을 없앤다). 차로 1의 차는 경로대로 간다.

시험용 버스는 건드리지 않는다. update(t)를 매 스텝 부른다(runner의 extras).
"""
from .presence import nearest_boundary_exit


def reachable_next(net, edge_id, lane_index):
    """edge_id의 lane_index 차로가 연결로 닿을 수 있는 다음 도로 id 집합."""
    lane = net.getEdge(edge_id).getLanes()[lane_index]
    return {c.getTo().getID() for c in lane.getOutgoing()}


class LaneRouteAdapter:
    def __init__(self, traci, net, bus_id, near_m=35.0, period=0.5, max_detour_m=500.0):
        self.tc, self.net, self.bus = traci, net, bus_id
        self.near_m, self.period, self.max_detour_m = near_m, period, max_detour_m
        self._t_next = -1e9
        self._done = {}                      # 차 id -> 진로를 바꾼 도로(같은 도로에서 다시 바꾸지 않는다)
        self.changed = 0
        self.errors = []                     # 진로를 못 바꾼 경우(차, 도로, 원래 다음 도로, 새 경로 앞부분, 오류)
        self._len = {}

    def _L(self, edge):
        if edge not in self._len:
            self._len[edge] = self.net.getEdge(edge).getLength()
        return self._len[edge]

    def _new_route(self, route, i, nxt):
        """route[i]에서 nxt로 나간 뒤 원래 목적지로 가는 경로. 이어지지 않으면 가까운 지도 경계까지."""
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
            lane = tc.vehicle.getLaneIndex(vid)
            e = self.net.getEdge(road)
            if lane >= e.getLaneNumber():
                continue
            route, i = tc.vehicle.getRoute(vid), tc.vehicle.getRouteIndex(vid)
            if i + 1 >= len(route) or self._L(road) - tc.vehicle.getLanePosition(vid) > self.near_m:
                continue
            ok = reachable_next(self.net, road, lane)
            if not ok or route[i + 1] in ok:
                continue
            pool = [c for c in sorted(ok) if self.net.hasEdge(c) and c not in (route[i - 1] if i > 0 else "",)]
            if not pool:
                continue
            new = self._new_route(route, i, pool[0])
            try:
                tc.vehicle.setRoute(vid, new)
            except Exception as exc:                      # 이어지지 않는 경로 등: 이 차만 건너뛰고 기록한다
                self.errors.append((vid, road, route[i + 1], new[:4], str(exc)[:120]))
                self._done[vid] = road
                continue
            self._done[vid] = road
            self.changed += 1
