"""밀도 불균형 해소: 차가 도로망 입구에서만 출발하면 신호 앞에 몰리고 신호 사이의 긴 도로는 비므로,
시험용 버스 노선의 긴 도로 위 임의의 지점에서도 차를 출발시킨다(건물·주차장에서 나오는 차와 같다).

- 입구 흐름 일부를 이쪽으로 옮긴다(`HapjeongTraffic.interior_share`). 그래서 전체 대수는 그대로다.
- **시험용 버스 가까이에서는 투입하지 않는다.** 버스에서 `min_dist`(기본 200 m, 직선거리) 안의 지점은 건너뛴다. 그래서 버스
  눈앞에 차가 갑자기 생기는 일이 없다. 차는 멀리서 출발해 달려와 버스 주변 구간으로 들어온다(정류장 앞 정차의 미리 투입과 같은 생각).
- 차는 `carDist`(배경 차량 분포)에서 뽑히므로 주행 모델 4종과 양보 성격이 그대로 붙는다.
- 가는 곳은 도로망 나가는 곳 중 무작위로 고르고, 경로는 최단 경로다. 노선 위 도로를 둘 이상 지나는 경로만 쓴다.
"""
import random

from sumolib.geomhelper import positionAtShapeOffset


class InteriorSpawner:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(runner의 extras)."""

    def __init__(self, traci, net, bus_id, route_edges, exit_edges, per_h, rng=None, min_dist=200.0, min_len=60.0,
                 dt=0.1, tries=6):
        self.tc, self.net, self.bus = traci, net, bus_id
        self.rng = rng or random.Random(0)
        self.min_dist, self.dt, self.tries = min_dist, dt, tries
        self.p_step = per_h / 3600.0 * dt
        self.route_edges = set(route_edges)
        self.roads = [e for e in route_edges if net.hasEdge(e) and net.getEdge(e).getLength() >= min_len]
        self.weights = [net.getEdge(e).getLength() for e in self.roads]
        self.exits = [e for e in exit_edges if net.hasEdge(e)]
        self._routes = {}
        self.n_spawned = self.n_skipped_near = self.n_failed = 0
        self._seq = 0

    def _route(self, src, dst):
        key = (src, dst)
        if key not in self._routes:
            seg, _ = self.net.getOptimalPath(self.net.getEdge(src), self.net.getEdge(dst), vClass="passenger")
            ids = [e.getID() for e in seg] if seg else []
            self._routes[key] = ids if sum(1 for e in ids if e in self.route_edges) >= 2 else None
        return self._routes[key]

    def _xy(self, edge, pos):
        return positionAtShapeOffset(self.net.getEdge(edge).getShape(), pos)

    def _far_from_bus(self, edge, pos):
        tc = self.tc
        if self.bus not in tc.vehicle.getIDList():
            return True
        bx, by = tc.vehicle.getPosition(self.bus)
        x, y = self._xy(edge, pos)
        return ((x - bx) ** 2 + (y - by) ** 2) ** 0.5 >= self.min_dist

    def update(self, t):
        if self.rng.random() >= self.p_step or not self.roads:
            return
        tc = self.tc
        for _ in range(self.tries):
            edge = self.rng.choices(self.roads, weights=self.weights)[0]
            length = self.net.getEdge(edge).getLength()
            pos = self.rng.uniform(5.0, length - 25.0)
            if not self._far_from_bus(edge, pos):
                self.n_skipped_near += 1
                continue
            route = self._route(edge, self.rng.choice(self.exits))
            if not route:
                continue
            self._seq += 1
            vid, rid = f"interior{self._seq}", f"r_interior{self._seq}"
            try:
                tc.route.add(rid, route)
                tc.vehicle.add(vid, rid, typeID="carDist", departLane="best", departPos=f"{pos:.1f}", departSpeed="max")
            except Exception:
                self.n_failed += 1          # 그 자리가 막혀 있으면 넣지 않는다(다음 시도)
                continue
            self.n_spawned += 1
            return

    def summary(self):
        return {"spawned": self.n_spawned, "skipped_near_bus": self.n_skipped_near, "failed_insert": self.n_failed}
