"""시험용 버스 주변 밀도 맞추기: 버스 반경(`PRESENCE_RADIUS_M`) 안에서는 순간이동을 하지 못하므로, 차를 넣고 빼는 대신 **속도**로 맞춘다.

- 버스 뒤(노선 거리)에 반경 안 차가 모자라면, 반경 밖 뒤 `reach` m 안의 가장 가까운 차를 버스보다 `pull_dv` 빠르게 달리게 해 반경 안으로 들어오게 한다.
- 버스 앞에 모자라면, 반경 밖 앞 `reach` m 안의 가장 가까운 차를 도로 제한 속도보다 `slow_dv` 느리게 달리게 해 버스가 다가가 반경 안에 들게 한다.
- 차가 반경 안(`enter_gap`)에 들어오면 속도 지정을 풀고 SUMO 기본 알고리즘에 돌려준다(시간 초과·신호 구간·사라짐도 푼다).
- 필요 대수는 버스가 있는 도로의 목표 밀도로 정한다: `max(1, round(밀도 * radius / 100))`(한쪽 기준).
- 버스가 정류장에 서 있는 동안(`dwell_s`초)에는 뒤쪽만 맞춘다: 뒤차가 정차가 끝날 무렵(출발할 때)에 반경 안에 들어오도록 남은 정차 시간으로
  속도를 계산해 보낸다. 앞차는 정차 중 멀어지기만 하므로 늦추지 않고, 이미 늦추던 앞차는 놓아 준다.
- 시험용 버스에는 아무 명령도 내리지 않는다. 신호 교차로 안·정지선 앞 금지 구간에서는 새로 걸지 않는다.
- 다른 추가 행동이 잡은 차(`Ownership`)는 건드리지 않고, 잡을 때는 `"density"` 이름으로 claim한다.
"""
from .density_keeper import DEFAULT_DENSITY_PER_100M
from .presence import PRESENCE_RADIUS_M
from .signal_areas import forbidden_intervals

WHO = "density"
# 시나리오가 직접 다루는 차(주입기 상대·끼어들기 차, 정류장 앞 정차, 정차 차량, 택시)는 속도를 건드리지 않는다.
SCENARIO_PREFIXES = ("inj_", "stopahead", "stopped", "taxi")


def route_gap(route, i, pos, lengths, road, fpos, span=6):
    """버스 앞끝(노선 route의 i번째 도로 pos)에서 같은 노선 위 차(road, fpos)까지의 노선 거리. 앞이면 양수, 뒤면 음수.
    교차로 안이거나 노선 앞뒤 `span`개 도로 밖이면 None. 도로 id로만 견주면 짧은 다음 도로로 넘어간 차를 놓친다."""
    if road.startswith(":"):
        return None
    for j in range(max(0, i - span), min(len(route), i + span + 1)):
        if route[j] != road:
            continue
        if j == i:
            return fpos - pos
        if j > i:
            return lengths(route[i]) - pos + sum(lengths(e) for e in route[i + 1:j]) + fpos
        return -(pos + sum(lengths(e) for e in route[j + 1:i]) + lengths(route[j]) - fpos)
    return None


class NearBusDensity:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(runner의 extras). `period`초마다 한 번 일한다."""

    def __init__(self, traci, net, bus_id, owner, density=DEFAULT_DENSITY_PER_100M, *, radius=PRESENCE_RADIUS_M,
                 reach=150.0, enter_gap=None, pull_dv=4.0, slow_dv=4.0, min_speed=3.0, timeout=20.0, period=0.5,
                 dwell_s=8.0):
        self.tc, self.net, self.bus, self.owner = traci, net, bus_id, owner
        self.density, self.radius, self.reach = dict(density), radius, reach
        self.enter_gap = enter_gap if enter_gap is not None else radius - 10.0
        self.pull_dv, self.slow_dv, self.min_speed = pull_dv, slow_dv, min_speed
        self.timeout, self.period, self.dwell_s = timeout, period, dwell_s
        self._dwell_t0 = None                # 버스가 정류장에 서기 시작한 시각
        self._last = -1e9
        self._held = {}                      # vid -> (종류 "pull"|"slow", 시작 시각)
        self._forbidden, self._route_len = {}, -1
        self.n_pulled = self.n_slowed = self.n_in_range = self.n_timeout = 0
        self.touched = []                    # 속도를 건드린 차 id (시각, id, 종류). 진단용으로 앞 60개만 남긴다

    def summary(self):
        return {"pulled": self.n_pulled, "slowed": self.n_slowed, "released_in_range": self.n_in_range,
                "timeout": self.n_timeout, "touched": list(self.touched)}

    def _len(self, e):
        return self.net.getEdge(e).getLength()

    def _in_zone(self, road, pos):
        return any(lo <= pos <= hi for lo, hi in self._forbidden.get(road, []))

    def _release(self, vid):
        try:
            self.tc.vehicle.setSpeed(vid, -1)
        except Exception:
            pass                              # 이미 사라졌다
        self.owner.release(vid, WHO)
        self._held.pop(vid, None)

    def update(self, t):
        if t < self._last:
            self._last = -1e9
        if t - self._last < self.period - 1e-9:
            return
        self._last = t
        tc = self.tc
        ids = tc.vehicle.getIDList()
        if self.bus not in ids:
            for vid in list(self._held):
                self._release(vid)
            return
        route, i = tc.vehicle.getRoute(self.bus), tc.vehicle.getRouteIndex(self.bus)
        if len(route) != self._route_len:
            self._forbidden, self._route_len = forbidden_intervals(self.net, route), len(route)
        road, pos, v = tc.vehicle.getRoadID(self.bus), tc.vehicle.getLanePosition(self.bus), tc.vehicle.getSpeed(self.bus)
        in_zone = road.startswith(":") or self._in_zone(road, pos)
        stopped = bool(tc.vehicle.getStopState(self.bus) & 1)
        if stopped and self._dwell_t0 is None:
            self._dwell_t0 = t
        elif not stopped:
            self._dwell_t0 = None
        remaining = max(1.0, self.dwell_s - (t - self._dwell_t0)) if stopped else None   # 출발까지 남은 시간(최소 1초)

        def gap_of(vid):
            return route_gap(route, i, pos, self._len, tc.vehicle.getRoadID(vid), tc.vehicle.getLanePosition(vid))

        # 이미 걸어 둔 차: 반경 안에 들었거나 시간이 지났거나 사라졌으면 푼다. 아니면 버스 속도에 맞춰 속도를 갱신한다
        for vid, (kind, t0) in list(self._held.items()):
            g = gap_of(vid) if vid in ids else None
            if g is None or in_zone or (stopped and kind == "slow"):
                self._release(vid)
            elif (kind == "pull" and g >= -self.enter_gap) or (kind == "slow" and g <= self.enter_gap):
                self.n_in_range += 1
                self._release(vid)
            elif t - t0 > self.timeout:
                self.n_timeout += 1
                self._release(vid)
            else:
                self._drive(vid, kind, v, g, remaining)
        if in_zone:
            return
        window = set(route[max(0, i - 6):i + 7])
        cars = []
        for vid in ids:
            if vid == self.bus or tc.vehicle.getRoadID(vid) not in window:
                continue
            g = gap_of(vid)
            if g is not None:
                cars.append((g, vid))
        name = self.net.getEdge(road).getName() if self.net.hasEdge(road) else ""
        need = max(1, round(self.density.get(name, 0.0) * self.radius / 100.0))
        behind = sum(1 for g, _ in cars if -self.radius < g < 0)
        ahead = sum(1 for g, _ in cars if 0 <= g < self.radius)
        held = [k for k, _ in self._held.values()]
        for kind, have in (("pull", behind), ("slow", ahead)):
            if stopped and kind == "slow":
                continue                 # 버스가 서 있는 동안 앞차는 멀어지기만 한다: 늦추지 않는다
            deficit = need - have - held.count(kind)
            if deficit <= 0:
                continue
            if kind == "pull":
                pool = sorted((-g, vid) for g, vid in cars if -self.reach <= g <= -self.radius)
            else:
                pool = sorted((g, vid) for g, vid in cars if self.radius <= g <= self.reach)
            for _, vid in pool:
                if deficit <= 0:
                    break
                if vid in self._held or vid.startswith(SCENARIO_PREFIXES) or self._in_zone(tc.vehicle.getRoadID(vid), tc.vehicle.getLanePosition(vid)):
                    continue
                if not self.owner.claim(vid, WHO):
                    continue
                self._held[vid] = (kind, t)
                if len(self.touched) < 60:
                    self.touched.append((round(t, 1), vid, kind))
                if kind == "pull":
                    self.n_pulled += 1
                else:
                    self.n_slowed += 1
                self._drive(vid, kind, v, next(g for g, o in cars if o == vid), remaining)
                deficit -= 1

    def _drive(self, vid, kind, v_bus, gap=None, remaining=None):
        if kind == "pull":
            cap = self.tc.vehicle.getAllowedSpeed(vid) * 1.1
            if remaining is not None and gap is not None:       # 정차 중: 출발 무렵에 반경 안(enter_gap)에 닿는 속도
                v = (-gap - self.enter_gap) / remaining
                self.tc.vehicle.setSpeed(vid, min(max(v, self.min_speed), cap))
                return
            self.tc.vehicle.setSpeed(vid, min(v_bus + self.pull_dv, cap))
        else:
            # 버스 현재 속도가 아니라 도로 제한 속도 기준이다. 버스 속도 기준이면 버스가 느려진 앞차를 뒤따라 줄 때 목표가 계속 낮아져
            # 버스가 기어갔다(측정에서 2·3바퀴째가 끈 상태보다 25% 길었다)
            self.tc.vehicle.setSpeed(vid, max(self.tc.vehicle.getAllowedSpeed(vid) - self.slow_dv, self.min_speed))
