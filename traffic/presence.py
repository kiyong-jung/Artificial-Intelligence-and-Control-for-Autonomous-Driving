"""시험용 버스 반경 안에서 물체가 생기거나 사라지는지 재는 지표와 지도 경계 도우미.

- 반경 `PRESENCE_RADIUS_M`(직선거리) 안에서 차·버스·택시·보행자가 새로 생기거나 사라진 횟수를 센다(`PresenceMonitor`).
  측정만 한다. 버스 가까이에서 출발을 미루거나 차를 지우거나 목적지를 바꾸지 않는다.
- 지도 경계(도로가 끊긴 곳, 막다른 골목 끝)와 골목 안에서 생기고 사라지는 것은 세지 않는다.
- SUMO의 정체 순간이동은 켜 두고, 반경 안에서 일어나면 센다.
"""
import heapq
import math

PRESENCE_RADIUS_M = 50.0      # 학습 차량의 관측 거리(env/mdp_config.py visibility 50 m)와 같게 둔다


# ---- 지도 경계 ---------------------------------------------------------------------------------
_boundary_cache = {}


def boundary_nodes(net):
    """일반 도로(내부 도로 제외)로 이웃 노드가 하나뿐인 노드 id 집합. 지도 끊김점과 막다른 골목 끝이다."""
    key = id(net)
    if key not in _boundary_cache:
        out = set()
        for node in net.getNodes():
            nb = set()
            for e in list(node.getIncoming()) + list(node.getOutgoing()):
                if e.getFunction() == "internal":
                    continue
                nb.add(e.getFromNode().getID())
                nb.add(e.getToNode().getID())
            nb.discard(node.getID())
            if len(nb) <= 1:
                out.add(node.getID())
        _boundary_cache[key] = out
    return _boundary_cache[key]


def ends_at_boundary(net, edge_id):
    return net.hasEdge(edge_id) and net.getEdge(edge_id).getToNode().getID() in boundary_nodes(net)


def starts_at_boundary(net, edge_id):
    return net.hasEdge(edge_id) and net.getEdge(edge_id).getFromNode().getID() in boundary_nodes(net)


def touches_boundary(net, edge_id):
    """보행자는 도로를 양방향으로 걸으므로 어느 끝이든 경계면 경계 도로로 본다."""
    return ends_at_boundary(net, edge_id) or starts_at_boundary(net, edge_id)


# ---- 거리 --------------------------------------------------------------------------------------
def hidden_in_alley(road):
    """골목(G…) 안의 도로인가. 건물에 가려 학습 차량이 못 보므로 골목 안에서 생기거나 사라지는 차는 나타남·사라짐으로 보지
    않는다. 골목 입구(0 m)에 닿아서야 보이는데, 차는 골목 끝에서 출발해 입구로 달려 나오므로 입구 앞에 갑자기 생기지 않는다."""
    return road.startswith("G")


_exit_cache = {}


def nearest_boundary_exit(net, from_edge, vclass="passenger"):
    """from_edge에서 경계로 끝나는 도로까지 가장 짧은 경로(도로 id 목록, from_edge 포함). 없으면 None."""
    key = (id(net), from_edge, vclass)
    if key in _exit_cache:
        return _exit_cache[key]
    if not net.hasEdge(from_edge):
        return None
    start = net.getEdge(from_edge)
    best = {from_edge: 0.0}
    prev = {}
    heap = [(0.0, from_edge)]
    found = None
    while heap:
        d, eid = heapq.heappop(heap)
        if d > best.get(eid, math.inf):
            continue
        e = net.getEdge(eid)
        if ends_at_boundary(net, eid):
            found = eid
            break
        for nxt in e.getOutgoing():
            if nxt.getFunction() == "internal" or not nxt.allows(vclass):
                continue
            nd = d + nxt.getLength()
            if nd < best.get(nxt.getID(), math.inf):
                best[nxt.getID()] = nd
                prev[nxt.getID()] = eid
                heapq.heappush(heap, (nd, nxt.getID()))
    path = None
    if found is not None:
        path = [found]
        while path[-1] != start.getID():
            path.append(prev[path[-1]])
        path.reverse()
    _exit_cache[key] = path
    return path


# ---- 측정 --------------------------------------------------------------------------------------
class PresenceMonitor:
    """시험용 버스 반경 안의 출현·소멸·정체 순간이동을 센다. update(t)는 매 스텝 simulationStep() 앞에서 부른다."""

    def __init__(self, tc, net, bus_id, radius=PRESENCE_RADIUS_M, max_examples=12):
        self.tc, self.net, self.bus, self.radius = tc, net, bus_id, radius
        self.max_examples = max_examples
        self.counts = {"appear": 0, "vanish": 0, "teleport": 0}
        self.examples = []
        self._near = {}            # 직전 스텝에 반경 안에 있던 물체 id -> (종류, 도로, 거리)
        self._subscribed = False
        c = getattr(tc, "constants", None)
        self._k_pos = getattr(c, "VAR_POSITION", "pos")
        self._k_road = getattr(c, "VAR_ROAD_ID", "road")

    def _note(self, kind, t, oid, d, what, road):
        self.counts[kind] += 1
        if len(self.examples) < self.max_examples:
            self.examples.append((round(t, 1), oid, round(d, 1), what, kind, road))

    def _dist(self, xy, bxy):
        return math.hypot(xy[0] - bxy[0], xy[1] - bxy[1])

    def update(self, t):
        tc = self.tc
        if self.bus not in tc.vehicle.getIDList():
            self._near = {}
            return
        bxy = tc.vehicle.getPosition(self.bus)
        if not self._subscribed:
            c = getattr(tc, "constants", None)
            dom = getattr(c, "CMD_GET_VEHICLE_VARIABLE", None)
            tc.vehicle.subscribeContext(self.bus, dom, self.radius * 2, [self._k_pos, self._k_road])
            self._subscribed = True
        sim = tc.simulation
        # 소멸·순간이동: 직전 스텝에 반경 안에 있던 물체가 이번에 사라졌는가
        for vid in sim.getArrivedIDList():
            if vid in self._near:
                what, road, d = self._near[vid]
                if not ends_at_boundary(self.net, road) and not hidden_in_alley(road):
                    self._note("vanish", t, vid, d, what, road)
        for pid in sim.getArrivedPersonIDList():
            if pid in self._near:
                what, road, d = self._near[pid]
                if not touches_boundary(self.net, road):
                    self._note("vanish", t, pid, d, what, road)
        for vid in sim.getStartingTeleportIDList():
            if vid in self._near:
                what, road, d = self._near[vid]
                self._note("teleport", t, vid, d, what, road)
        # 출현
        for vid in sim.getDepartedIDList():
            if vid == self.bus or vid not in tc.vehicle.getIDList():
                continue
            d = self._dist(tc.vehicle.getPosition(vid), bxy)
            road = tc.vehicle.getRoadID(vid)
            if (d < self.radius and not starts_at_boundary(self.net, road)
                    and not hidden_in_alley(road)):
                self._note("appear", t, vid, d, "vehicle", road)
        persons = set(tc.person.getIDList())
        for pid in sim.getDepartedPersonIDList():
            if pid not in persons:
                continue
            d = self._dist(tc.person.getPosition(pid), bxy)
            road = tc.person.getRoadID(pid)
            if d < self.radius and not touches_boundary(self.net, road):
                self._note("appear", t, pid, d, "person", road)
        # 다음 스텝을 위한 반경 안 목록
        near = {}
        for vid, r in (tc.vehicle.getContextSubscriptionResults(self.bus) or {}).items():
            if vid == self.bus:
                continue
            d = self._dist(r[self._k_pos], bxy)
            if d < self.radius:
                near[vid] = ("vehicle", r[self._k_road], d)
        for pid in persons:
            xy = tc.person.getPosition(pid)
            d = self._dist(xy, bxy)
            if d < self.radius:
                near[pid] = ("person", tc.person.getRoadID(pid), d)
        self._near = near

    def summary(self):
        return {"radius": self.radius, **self.counts, "examples": list(self.examples)}
