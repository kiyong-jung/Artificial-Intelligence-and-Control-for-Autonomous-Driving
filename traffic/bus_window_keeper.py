"""버스 주변 차량 밀도 유지기: 시험용 버스를 따라다니는 앞뒤 `radius`(50 m) 창에서 **차로마다** 목표 대수를 유지한다.

- 목표 밀도는 도로 이름별 "대/100 m/차로"다(`DEFAULT_LANE_DENSITY`). 목표 2이면 창(100 m) 안에 버스 차로에 2대, 옆 차로에 2대, 차로가 더 있으면 그 차로에도
  2대씩이다. 승용차가 못 쓰는 차로(인도, 버스 전용)는 세지 않고, 창이 도로 경계를 걸치면 차로마다 창 안에서 그 차로가 있는 길이만큼만 센다.
  밀도가 없는 도로(와우산로·잔다리로)는 목표도 투입도 없다. 창 안 대수는 자연 차량을 포함해 모든 차를 센다.
- 모자라면 **버스 뒤 `radius + margin`~ +40 m에서 만든다**(구간 끝 `stop_zone_m` 앞, 빈 자리, 버스에서 직선 `radius + margin` 이상). 늘어나는 것은 허용하고,
  줄어드는 쪽으로는 지우지 않는다.
- 지우기(순간이동)는 버스 시야(`radius + margin`) 밖에서만 한다.
  - 키퍼가 넣은 차가 버스 앞 시야 밖으로 나가면 지운다(밀도 유지를 위해 앞차를 늦추지 않는다).
  - 신호 대기열이 길 때(멈춘 차가 `queue_max`대 이상) 버스 뒤 시야 밖의 멈춘 키퍼 차를 지운다.
- 노선은 순환 고리(`loop_edges`, 마지막 도로가 첫 도로로 이어짐)로 보고 버스 뒤 도로까지 계산한다(시험용 버스의 현재 경로에는 지난 도로가 없다).
- 넣은 차는 노선을 따라 `ahead_m`(약 300 m) 앞까지만 가고, 피할 도로(`avoid`: 와우산로·잔다리로) 앞에서 다른 도로로 빠져 경계 출구로 나간다.
- 시나리오 주입기(S2·S3)는 이 키퍼가 넣은 차 중에서 버스 목표 차로 뒤 가장 가까운 차를 상대로 고르고, 없으면 `spawn_car`로 하나 만들게 한다.
"""
import math
import random

from sumolib.geomhelper import positionAtShapeOffset

from .presence import PRESENCE_RADIUS_M, SPAWN_MARGIN_M, far_from_bus, nearest_boundary_exit

# 도로 이름 -> 대/100 m/차로. 양화로·독막로만 유지한다(와우산로·잔다리로는 투입 없음).
DEFAULT_LANE_DENSITY = {"양화로": 2.0, "독막로": 2.0}


class BusWindowKeeper:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(runner의 extras)."""

    def __init__(self, traci, net, bus_id, loop_edges, lane_density=DEFAULT_LANE_DENSITY, *, radius=PRESENCE_RADIUS_M,
                 margin=SPAWN_MARGIN_M, rng=None, period=0.5, ahead_m=300.0, stop_zone_m=30.0, queue_max=6,
                 max_per_h=3600.0, burst=6.0, avoid=frozenset(), protected=None, tries=6, spawn_span_m=40.0):
        self.tc, self.net, self.bus = traci, net, bus_id
        self.rng = rng or random.Random(0)
        self.density = dict(lane_density)
        self.radius, self.margin, self.period = radius, margin, period
        self.view = radius + margin                       # 버스 시야(+여유): 이 밖에서만 만들고 지운다
        self.ahead_m, self.stop_zone_m, self.queue_max, self.tries = ahead_m, stop_zone_m, queue_max, tries
        self.max_per_h, self.burst, self.spawn_span_m = max_per_h, burst, spawn_span_m
        self.avoid = frozenset(avoid)
        self.protected = protected or (lambda vid: False)
        self.loop = [e for e in loop_edges if net.hasEdge(e)]
        self.n = len(self.loop)
        self.lens = [net.getEdge(e).getLength() for e in self.loop]
        self.offset, tot = [], 0.0
        for L in self.lens:
            self.offset.append(tot)
            tot += L
        self.total = tot
        self.index = {e: i for i, e in enumerate(self.loop)}
        self.cars = set()                                  # 이 키퍼가 넣은 차 id
        self._tokens, self._last, self._seq = burst, -1e9, 0
        self.stats = {"spawned": 0, "scenario_spawned": 0, "removed_ahead": 0, "removed_queue": 0, "skipped_near": 0,
                      "skipped_budget": 0, "failed_insert": 0}
        self.by_road = {}

    # ---- 기하 -------------------------------------------------------------------------------
    def _name(self, edge):
        return self.net.getEdge(edge).getName()

    def _lane_ok(self, edge, k):
        ed = self.net.getEdge(edge)
        return 0 <= k < ed.getLaneNumber() and ed.getLane(k).allows("passenger")

    def _layout(self, i, pos, back, front):
        """버스(loop[i], pos) 기준 상대 거리 [-back, +front]와 겹치는 도로와 그 시작 상대 위치 [(도로, rel_start, 길이)]. 순환 고리다."""
        out = [(self.loop[i], -pos, self.lens[i])]
        k, s = i, -pos
        while s > -back and len(out) < self.n:
            k = (k - 1) % self.n
            s -= self.lens[k]
            out.append((self.loop[k], s, self.lens[k]))
        k, s = i, -pos + self.lens[i]
        while s < front and len(out) < self.n:
            k = (k + 1) % self.n
            out.append((self.loop[k], s, self.lens[k]))
            s += self.lens[k]
        return out

    def _rel(self, vid, edge_idx, lane_pos, i, pos):
        """차의 버스 기준 노선 상대 거리(앞이 양수). 순환 고리 위로 [-total/2, total/2]에 맞춘다."""
        delta = (self.offset[edge_idx] + lane_pos) - (self.offset[i] + pos)
        return (delta + self.total / 2.0) % self.total - self.total / 2.0

    def _xy(self, edge, pos):
        return positionAtShapeOffset(self.net.getEdge(edge).getShape(), pos)

    def _local_route(self, edge):
        """edge에서 시작해 순환 노선을 따라 약 `ahead_m` 앞까지. 피할 도로 앞에서 끝나면 다른 도로로 이어 붙인다."""
        i = self.index[edge]
        out, total = [edge], self.lens[i]
        k = i
        while total < self.ahead_m:
            k = (k + 1) % self.n
            nxt = self.loop[k]
            if nxt in self.avoid:
                tail = nearest_boundary_exit(self.net, out[-1], "passenger", self.avoid)
                if tail:
                    out = out[:-1] + tail
                break
            if nxt == edge:
                break
            out.append(nxt)
            total += self.lens[k]
        return out

    # ---- 매 스텝 ----------------------------------------------------------------------------
    def update(self, t):
        if t < self._last:
            self._last = -1e9
        if t - self._last < self.period - 1e-9:
            return
        self._last = t
        tc = self.tc
        self._tokens = min(self.burst, self._tokens + self.max_per_h / 3600.0 * self.period)
        ids = set(tc.vehicle.getIDList())
        self.cars &= ids
        if self.bus not in ids:
            return
        road = tc.vehicle.getRoadID(self.bus)
        if road not in self.index:
            return                                         # 교차로 안이거나 노선 밖
        i, pos = self.index[road], tc.vehicle.getLanePosition(self.bus)
        self._remove(i, pos, ids)
        deficits = self._deficits(i, pos)
        for k, d in sorted(deficits.items(), key=lambda kv: -kv[1]):
            if d <= 0:
                continue
            if self._tokens < 1.0:
                self.stats["skipped_budget"] += 1
                break
            if self._spawn_rear(i, pos, k):
                self._tokens -= 1.0

    def _deficits(self, i, pos):
        """차로 번호 k -> 부족한 대수(목표 - 지금)."""
        tc, R = self.tc, self.radius
        avail, count = {}, {}
        for edge, rel_start, L in self._layout(i, pos, R, R):
            overlap = max(0.0, min(rel_start + L, R) - max(rel_start, -R))
            d = self.density.get(self._name(edge), 0.0)
            if overlap > 0 and d > 0:
                for k in range(self.net.getEdge(edge).getLaneNumber()):
                    if self._lane_ok(edge, k):
                        avail[k] = avail.get(k, 0.0) + overlap * d / 100.0
            for vid in tc.edge.getLastStepVehicleIDs(edge):
                if vid == self.bus:
                    continue
                rel = rel_start + tc.vehicle.getLanePosition(vid)
                if -R <= rel <= R:
                    k = tc.vehicle.getLaneIndex(vid)
                    count[k] = count.get(k, 0) + 1
        return {k: int(round(a)) - count.get(k, 0) for k, a in avail.items() if a >= 0.5}

    def _spawn_rear(self, i, pos, k):
        """차로 k에 버스 뒤 시야 바깥쪽에서 차를 한 대 만든다. 만들었으면 True."""
        tc = self.tc
        layout = self._layout(i, pos, self.view + self.spawn_span_m + 5.0, 0.0)
        for step in range(self.tries):
            back = self.view + step * (self.spawn_span_m / max(1, self.tries - 1))
            for edge, rel_start, L in layout:
                p = -back - rel_start
                if not (0.0 <= p < L):
                    continue
                if edge in self.avoid or self.density.get(self._name(edge), 0.0) <= 0 or not self._lane_ok(edge, k):
                    break
                if p < 5.0 or p > L - self.stop_zone_m:
                    break                                   # 구간 끝 stop_zone_m(교차로·횡단보도 앞)에는 넣지 않는다
                if not far_from_bus(tc, self.bus, self._xy(edge, p), self.view):
                    self.stats["skipped_near"] += 1
                    break
                if any(abs(tc.vehicle.getLanePosition(v) - p) < 12.0 for v in tc.lane.getLastStepVehicleIDs(f"{edge}_{k}")):
                    break                                   # 그 자리가 막혀 있다: 더 뒤에서 다시 본다
                if self._add(self._local_route(edge), k, p):
                    name = self._name(edge)
                    self.by_road[name] = self.by_road.get(name, 0) + 1
                    self.stats["spawned"] += 1
                    return True
                break
        return False

    def _add(self, path, lane, pos):
        tc = self.tc
        self._seq += 1
        vid, rid = f"win{self._seq}", f"r_win{self._seq}"
        try:
            tc.route.add(rid, path)
            tc.vehicle.add(vid, rid, typeID="carDist", departLane=str(lane), departPos=f"{pos:.1f}", departSpeed="max")
        except Exception:
            self.stats["failed_insert"] += 1
            return None
        self.cars.add(vid)
        return vid

    def spawn_car(self, path, lane, pos):
        """시나리오 주입기(S2·S3)가 요청하는 상대 차 한 대. 버스 시야 안이면 None. 이 키퍼의 차로 등록되어 같은 규칙으로 관리된다."""
        edge = path[0]
        if not far_from_bus(self.tc, self.bus, self._xy(edge, pos), self.view):
            return None
        vid = self._add(path, lane, pos)
        if vid:
            self.stats["scenario_spawned"] += 1
        return vid

    def is_keeper_car(self, vid):
        return vid in self.cars

    def _remove(self, i, pos, ids):
        """키퍼 차 중 버스 앞 시야 밖의 차를 지우고, 신호 대기열이 길면 버스 뒤 시야 밖의 멈춘 차 한 대를 지운다."""
        tc, view = self.tc, self.view
        behind = None
        for vid in list(self.cars):
            road = tc.vehicle.getRoadID(vid)
            if road not in self.index or self.protected(vid):
                continue
            rel = self._rel(vid, self.index[road], tc.vehicle.getLanePosition(vid), i, pos)
            if rel > view and far_from_bus(tc, self.bus, tc.vehicle.getPosition(vid), view):
                if self._kill(vid, "removed_ahead"):
                    continue
            elif rel < -view and tc.vehicle.getSpeed(vid) < 0.5 and tc.edge.getLastStepHaltingNumber(road) >= self.queue_max:
                if behind is None or rel < behind[0]:
                    behind = (rel, vid)
        if behind is not None and far_from_bus(tc, self.bus, tc.vehicle.getPosition(behind[1]), view):
            self._kill(behind[1], "removed_queue")

    def _kill(self, vid, stat):
        try:
            self.tc.vehicle.remove(vid)
        except Exception:
            return False
        self.cars.discard(vid)
        self.stats[stat] += 1
        return True

    def summary(self):
        return {**self.stats, "spawned_by_road": dict(self.by_road)}
