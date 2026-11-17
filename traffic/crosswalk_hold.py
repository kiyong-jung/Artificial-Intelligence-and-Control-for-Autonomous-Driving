"""신호 교차로의 횡단보도로 가는 보행자가 교차로 대기 구역에서 떨어져 기다리게 한다.

SUMO는 교차로 경계에 보행자 대기 구역(walkingarea)과 횡단보도를 붙여 만든다. 횡단보도 앞에서 건너기를 기다리는 보행자는 차선 끝에서
2 m도 안 되는 곳에 서므로, 그 앞으로 좌회전해 나가는 시험용 버스와 보행자가 서로 기다리다 둘 다 멈췄다(J82에서 300초 정지).
시험용 버스가 그 교차로 가까이(`bus_radius`) 있는 동안은 보행자를 대기 구역에서 `hold_m` 떨어진 인도 위에서 세워 둔다. 버스가
멀어지면 풀어 준다. 한 사람을 `max_hold_s`보다 오래 세우지 않고, 풀려 난 보행자는 같은 대기 구역 앞에서 다시 세우지 않는다.
신호 없는 횡단보도(S6)는 보행자가 일부러 버스 앞으로 건너는 시나리오라서 대상이 아니다.
"""
import math

DEFAULT_HOLD_M = 3.0
DEFAULT_BUS_RADIUS_M = 40.0
DEFAULT_MAX_HOLD_S = 30.0


def _point_to_polyline(p, shape):
    best = float("inf")
    px, py = p
    for (ax, ay), (bx, by) in zip(shape, shape[1:]):
        dx, dy = bx - ax, by - ay
        n = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / n)) if n else 0.0
        best = min(best, math.hypot(px - ax - t * dx, py - ay - t * dy))
    return best


class CrosswalkHold:
    def __init__(self, tc, net, bus_id, hold_m=DEFAULT_HOLD_M, bus_radius=DEFAULT_BUS_RADIUS_M,
                 max_hold_s=DEFAULT_MAX_HOLD_S):
        self.tc, self.net, self.bus = tc, net, bus_id
        self.hold_m, self.bus_radius, self.max_hold_s = hold_m, bus_radius, max_hold_s
        self.held = {}                 # 보행자 -> (붙잡은 시각, 다음 대기 구역)
        self.done = {}                 # 풀려 난 보행자 -> 그 대기 구역(같은 구역에서는 다시 안 세운다)
        self.holds = 0
        self._shape = {}
        self._node = {}

    def _signalized_node(self, name):
        if name not in self._node:
            ok = False
            if self.net.hasNode(name):
                kind = self.net.getNode(name).getType()
                ok = kind.startswith("traffic_light") and kind != "traffic_light_unregulated"
            self._node[name] = ok
        return self._node[name]

    def _bus_near(self, node, bus_xy):
        cx, cy = self.net.getNode(node).getCoord()
        return math.hypot(bus_xy[0] - cx, bus_xy[1] - cy) <= self.bus_radius

    def update(self, t):
        tc = self.tc
        if self.bus not in tc.vehicle.getIDList():
            bus_xy = None
        else:
            bus_xy = tc.vehicle.getPosition(self.bus)
        persons = set(tc.person.getIDList())
        for pid in [p for p in self.held if p not in persons]:
            del self.held[pid]
        for pid in [p for p in self.done if p not in persons]:
            del self.done[pid]
        for pid in sorted(persons):
            road, nxt = tc.person.getRoadID(pid), tc.person.getNextEdge(pid)
            if pid in self.held:
                t0, area = self.held[pid]
                node = area[1:].rsplit("_w", 1)[0]
                if (road.startswith(":") or nxt != area or bus_xy is None or not self._bus_near(node, bus_xy)
                        or t - t0 >= self.max_hold_s):
                    tc.person.setSpeed(pid, -1)
                    self.done[pid] = area
                    del self.held[pid]
                continue
            if road.startswith(":") or not nxt or not nxt.startswith(":") or "_w" not in nxt or bus_xy is None:
                continue
            if self.done.get(pid) == nxt:
                continue
            node = nxt[1:].rsplit("_w", 1)[0]
            if not self._signalized_node(node) or not self._bus_near(node, bus_xy):
                continue
            if nxt not in self._shape:
                self._shape[nxt] = tc.lane.getShape(nxt + "_0")
            if _point_to_polyline(tc.person.getPosition(pid), self._shape[nxt]) <= self.hold_m:
                tc.person.setSpeed(pid, 0.0)
                self.held[pid] = (t, nxt)
                self.holds += 1
