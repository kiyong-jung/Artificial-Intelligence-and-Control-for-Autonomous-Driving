"""구간별 목표 대수 유지: 양화로·독막로·잔다리로·와우산로 위 차량 수를 SUMO(calibrator)에 맡기지 않고 코드가 직접 맞춘다.

- 구간 = 시험용 버스 노선 위의 도로 조각 중 이름이 `SECTION_ROADS`이고 길이가 `min_len` 이상인 것. 목표 대수는 `밀도(대/100 m) * 길이 * 차로 비율`이다.
  차로 비율 = 승용차가 쓸 수 있는 차로 수 / 같은 도로 이름 구간 중 가장 많은 수. 버스 전용 차로가 있는 양화로 R54a(승용 3차로, R54는 5차로)는 0.6배다.
  **밀도가 0인 도로는 목표도 투입도 없다**(예산을 쓰지 않는다. 그 도로에 원래 있는 차는 그대로다).
- **투입 지점(앵커)**: 구간 끝 `stop_zone_m`에는 넣을 수 없어 짧은 구간(독막로 R71.135·R71.206 등)에는 넣을 자리가 없다. 그래서 모자란 구간에는
  그 구간 자신과, 구간 시작에서 거슬러 `upstream_m`(150 m) 안의 상류 도로(밀도가 있는 도로 이름이고 넣을 자리가 있는 것) 중에서 골라 넣는다.
  차는 노선을 따라 달려와 그 구간에 들어온다.
- 모자라면 그 구간 안 임의의 지점에 차를 넣는다. **시험용 버스 반경(`PRESENCE_RADIUS_M`) + 여유(`SPAWN_MARGIN_M`) 밖에서만** 한다.
  버스 가까이는 `near_bus_density.NearBusDensity`가 순간이동 없이 속도로 맞춘다.
- 넣은 차는 버스 노선을 따라 `ahead_m`(약 300 m) 앞까지만 가고 사라진다. 멀리 있는 나가는 곳까지 긴 경로를 주면 차가 오래 남아 쌓이고
  골목 어귀에서 막혔다(처음 설계: t=50초에 키퍼 차 164대, 끈 상태의 전체 차량은 69대).
- 밀도를 올리는 것은 곧 유량을 늘리는 일이다(밀도 x 속도). 그래서 투입 예산 `max_per_h`(시간당 대수)를 두고, **대수가 가장 많이 모자란 구간부터**
  넣는다(상대 비율이 아니다). 넓고 긴 구간(양화로)에 예산이 먼저 간다.
- 신호 앞에 차가 몰려 정체가 나지 않게: 모든 구간의 끝 `stop_zone_m`(교차로·횡단보도 앞)에는 넣지 않고(넣은 차가 서지 못해 보행자와 부딪혔다),
  구간에 멈춘 차가 `halt_max`대 이상이면 그 구간에는 넣지 않는다.
- 지우기는 `surplus_ratio`(목표의 몇 배 초과)를 주었을 때만 하고 기본은 끈다(켜 두니 투입·제거가 계속 되풀이됐다).
- 차는 `carDist`(주행 모델 4종 + 양보 성격)로 넣는다. 다른 추가 행동이 잡은 차(`protected`)는 지우지 않는다.
"""
import math
import random
from dataclasses import dataclass

from sumolib.geomhelper import positionAtShapeOffset

from .presence import PRESENCE_RADIUS_M, SPAWN_MARGIN_M, far_from_bus, nearest_boundary_exit
from .signal_areas import is_signalized

SECTION_ROADS = ("양화로", "독막로", "잔다리로", "와우산로")
# 시작값(대/100 m)이다. 양화로는 5~6차로로 넓어 가장 높이고 독막로도 올린다. 2차선인 와우산로·잔다리로는 올릴 필요가 없어서 키퍼가 채우지
# 않도록 자연 수준(끈 상태 측정: 와우산로 0.78, 잔다리로 1.93)보다 낮게 둔다: 깊게 비었을 때만 채워 이전 차량 수와 같게 유지된다.
# 잔다리로(홍대입구역~서교동사거리)와 와우산로는 차를 넣지 않는다(0): 목표도 투입도 없어 예산을 쓰지 않는다. 그 도로에 원래 있는 차는 그대로다.
# 같은 투입 예산이 넓은 도로에 먼저 가게 한다. scripts/measure_section_density.py 측정으로 조정한다(이 값은 측정하지 않았다).
DEFAULT_DENSITY_PER_100M = {"양화로": 2.0, "독막로": 1.8, "잔다리로": 0.0, "와우산로": 0.0}


@dataclass
class Section:
    name: str
    edge: str
    length: float
    signalized_end: bool
    lane_scale: float = 1.0      # 승용차가 쓸 수 있는 차로 수 / 같은 도로 이름 구간 중 가장 많은 수(버스 전용 차로가 있는 구간은 1보다 작다)


def sections_of(net, route_edges, min_len=60.0):
    out = []
    for e in route_edges:
        if not net.hasEdge(e):
            continue
        ed = net.getEdge(e)
        if ed.getName() in SECTION_ROADS and ed.getLength() >= min_len:
            out.append(Section(ed.getName(), e, ed.getLength(), is_signalized(ed.getToNode())))
    cars = {s.edge: sum(1 for ln in net.getEdge(s.edge).getLanes() if ln.allows("passenger")) for s in out}
    widest = {}
    for s in out:
        widest[s.name] = max(widest.get(s.name, 0), cars[s.edge])
    for s in out:
        s.lane_scale = cars[s.edge] / widest[s.name] if widest[s.name] else 1.0
    return out


class DensityKeeper:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(runner의 extras)."""

    def __init__(self, traci, net, bus_id, route_edges, density=DEFAULT_DENSITY_PER_100M, *, rng=None,
                 period=1.0, stop_zone_m=60.0, halt_max=3, surplus_ratio=None, protected=None, tries=6,
                 max_per_h=900.0, burst=10.0, ahead_m=300.0, upstream_m=150.0, avoid=frozenset()):
        self.tc, self.net, self.bus = traci, net, bus_id
        self.rng = rng or random.Random(0)
        self.density, self.period, self.tries = dict(density), period, tries
        self.stop_zone_m, self.halt_max, self.surplus_ratio = stop_zone_m, halt_max, surplus_ratio
        self.max_per_h, self.burst, self.ahead_m, self.upstream_m = max_per_h, burst, ahead_m, upstream_m
        self.avoid = frozenset(avoid)             # 차가 들어가면 안 되는 도로(와우산로·잔다리로): 그 앞에서 다른 도로로 빠진다
        self.protected = protected or (lambda vid: False)
        self.min_dist = PRESENCE_RADIUS_M + SPAWN_MARGIN_M
        self.route_edges = list(route_edges)
        self._index = {e: i for i, e in enumerate(self.route_edges)}
        # 밀도가 0 이하인 도로(예: 잔다리로)는 목표도 투입도 없다
        self.sections = [s for s in sections_of(net, self.route_edges) if self.density.get(s.name, 0.0) > 0.0]
        self._anchors = {s.edge: self._anchors_of(s) for s in self.sections}      # 구간 도로 -> [(투입 도로, 넣을 수 있는 위치 상한)]
        self.by_road = {}
        self._tokens = burst
        self._last = -1e9
        self._seq = 0
        self.n_spawned = self.n_removed = self.n_near = self.n_queue = self.n_failed = self.n_budget = 0

    def target(self, section):
        return max(1, round(self.density[section.name] * section.length / 100.0 * section.lane_scale))

    def summary(self):
        return {"spawned": self.n_spawned, "removed": self.n_removed, "skipped_near_bus": self.n_near,
                "skipped_queue": self.n_queue, "skipped_budget": self.n_budget, "failed_insert": self.n_failed,
                "spawned_by_road": dict(self.by_road)}

    def _anchors_of(self, s):
        """s에 차를 보낼 수 있는 투입 도로 목록 [(도로 id, 위치 상한)]. s 자신과, s 시작에서 거슬러 `upstream_m` 안의 상류 도로 중
        밀도가 있는 도로 이름이고 구간 끝 `stop_zone_m`를 뺀 투입 자리가 10 m 넘게 남는 것."""
        out, back, k = [], 0.0, self._index[s.edge]
        while k >= 0:
            ed = self.net.getEdge(self.route_edges[k]) if self.net.hasEdge(self.route_edges[k]) else None
            if (ed is not None and self.density.get(ed.getName(), 0.0) > 0.0 and ed.getLength() - self.stop_zone_m > 10.0
                    and ed.getID() not in self.avoid):
                out.append((ed.getID(), ed.getLength() - self.stop_zone_m))
            k -= 1
            if k < 0:
                break
            back += self.net.getEdge(self.route_edges[k]).getLength() if self.net.hasEdge(self.route_edges[k]) else 0.0
            if back > self.upstream_m:
                break
        return out

    def _local_route(self, edge):
        """투입 도로에서 시작해 버스 노선을 따라 약 `ahead_m` 앞까지의 도로 목록. 피할 도로 앞에서 끝나면 다른 도로로 이어 붙인다."""
        out, total = [edge], self.net.getEdge(edge).getLength()
        for e in self.route_edges[self._index[edge] + 1:]:
            if total >= self.ahead_m or not self.net.hasEdge(e):
                break
            if e in self.avoid:                  # 노선이 피할 도로로 들어가는 곳: 그 앞 교차로(J82 등)에서 다른 도로로 빠져나간다
                tail = nearest_boundary_exit(self.net, out[-1], "passenger", self.avoid)
                if tail:
                    out = out[:-1] + tail
                break
            out.append(e)
            total += self.net.getEdge(e).getLength()
        return out

    def _xy(self, edge, pos):
        return positionAtShapeOffset(self.net.getEdge(edge).getShape(), pos)

    def update(self, t):
        if t < self._last:
            self._last = -1e9                # 예열이 끝나 시각이 0으로 돌아왔다
        if t - self._last < self.period - 1e-9:
            return
        self._last = t
        self._tokens = min(self.burst, self._tokens + self.max_per_h / 3600.0 * self.period)
        short = []
        for s in self.sections:
            n = self.tc.edge.getLastStepVehicleNumber(s.edge)
            tgt = self.target(s)
            if n < tgt:
                short.append((tgt - n, s))
            elif self.surplus_ratio is not None and n > math.ceil(tgt * self.surplus_ratio):
                self._trim(s)
        for _, s in sorted(short, key=lambda x: -x[0]):      # 대수가 가장 많이 모자란 구간부터
            if self._tokens < 1.0:
                self.n_budget += 1
                break
            if self._fill(s):
                self._tokens -= 1.0

    def _fill(self, s):
        if self.tc.edge.getLastStepHaltingNumber(s.edge) >= self.halt_max:
            self.n_queue += 1
            return False
        anchors = self._anchors.get(s.edge) or []
        if not anchors:
            return False
        for _ in range(self.tries):
            edge, upper = self.rng.choices(anchors, weights=[u - 5.0 for _, u in anchors])[0]
            if edge != s.edge and self.tc.edge.getLastStepHaltingNumber(edge) >= self.halt_max:
                self.n_queue += 1
                continue
            pos = self.rng.uniform(5.0, upper)               # 신호·횡단보도 앞(구간 끝 stop_zone_m)에는 넣지 않는다
            if not far_from_bus(self.tc, self.bus, self._xy(edge, pos), self.min_dist):
                self.n_near += 1
                continue
            self._seq += 1
            vid, rid = f"keep{self._seq}", f"r_keep{self._seq}"
            try:
                self.tc.route.add(rid, self._local_route(edge))
                self.tc.vehicle.add(vid, rid, typeID="carDist", departLane="best", departPos=f"{pos:.1f}",
                                    departSpeed="max")
            except Exception:
                self.n_failed += 1          # 그 자리가 막혀 있다: 다음 시도
                continue
            self.n_spawned += 1
            name = self.net.getEdge(edge).getName()
            self.by_road[name] = self.by_road.get(name, 0) + 1
            return True
        return False

    def _trim(self, s):
        tc = self.tc
        cands = [v for v in tc.edge.getLastStepVehicleIDs(s.edge)
                 if not self.protected(v) and far_from_bus(tc, self.bus, tc.vehicle.getPosition(v), self.min_dist)]
        if not cands:
            return
        victim = min(cands, key=lambda v: (tc.vehicle.getSpeed(v) >= 0.5, self.rng.random()))   # 멈춘 차부터
        try:
            tc.vehicle.remove(victim)
        except Exception:
            return
        self.n_removed += 1
