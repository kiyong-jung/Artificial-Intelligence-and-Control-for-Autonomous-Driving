"""S4. 골목에서 나온 차가 정류장 근처에 서서, 정류장으로 가는 시험용 버스의 길목을 막는다.

- 시험용 버스가 서는 정류장(s2, s4, s5, s6) 중, 정류장 앞 `max_upstream` m 안에 **신호 없는 합류 골목**이 있고 그 사이에 신호
  교차로가 없는 곳이 후보다(`s4_candidates`). s6(합정역)은 앞이 신호 교차로(J42)라 후보가 되지 않는다.
- 바퀴마다 후보 정류장 중 1~2곳(`n_range`)을 뽑는다.
- 차는 버스 눈앞에 갑자기 생기지 않는다: 그 정류장의 골목 안에서 출발해(버스 반경 밖) 큰길로 나와 정류장 근처에 선다.
  출발 시각은 골목 차가 버스보다 `min_lead_s`~`lead_s`초(15~40초) 먼저 정차 지점에 닿는 때로 맞춘다. 버스가 오기 전에 차가 먼저 서 있게 하려는 것이다.
- 그 여유가 모자라면 건너뛰지 않고, 정차 자리가 버스 반경(+여유) 밖이고 버스가 `direct_min_eta_s`초 넘게 남았고 그 자리 앞뒤 차로가 비어 있을 때
  정차 자리 가까이에 직접 세운다(`how="direct"`). 이 경우 골목에서 나오는 모습은 없다. 조건이 안 되면 건너뛴다(`none`).
- 서는 자리(`stop_position`): 정류장 시작 10~25 m 앞 또는 정류장 끝 5~20 m 뒤(정류장과 너무 가깝지 않게). 신호 정지선 앞
  금지 구간(signal_areas)과 골목 합류점보다 앞은 고르지 않는다. 정차 시간은 `dwell`. 다 서면 지도 경계로 나간다.
- **직접 배치 전용 정류장**(`direct_stops`, 예: s6 합정역): 앞이 신호 교차로라 합류 골목이 없는 정류장도 후보로 넣을 수 있다. 이 경우 골목 차는 없고, 버스가
  `min_lead_s`~`lead_s`초 안으로 다가오면 정차 자리에 직접 세운다(버스 반경 밖·자리가 빈 때만).
- `stop_ids`로 후보 정류장을 제한할 수 있다(None이면 골목이 있는 정류장 전부).
- 소유권: 세우는 동안 그 차는 `S4A`에 속한다. 끝나면 놓는다.
"""
import random
from dataclasses import dataclass

from sumolib.geomhelper import positionAtShapeOffset

from .common import Ownership
from .presence import PRESENCE_RADIUS_M, SPAWN_MARGIN_M, far_from_bus, nearest_boundary_exit
from .signal_areas import forbidden_intervals, is_signalized
from .test_bus import STOP_AT

SPAWN_TYPES = ("bgtaxi", "carDist")      # 골목에서 나올 차종(택시, 일반 승용차 분포). 같은 확률
BEFORE_M, AFTER_M = (10.0, 25.0), (5.0, 20.0)


def _gap(net, route, i, pos, j, pos2):
    """노선 위 (route[i], pos)에서 (route[j], pos2)까지 앞으로 가는 거리. j >= i."""
    if j == i:
        return pos2 - pos
    d = net.getEdge(route[i]).getLength() - pos
    for e in route[i + 1:j]:
        d += net.getEdge(e).getLength()
    return d + pos2


def _walk_back(net, route, k, pos, back):
    """(route[k], pos)에서 back m 거슬러 간 (index, 위치). 노선 앞을 넘으면 None."""
    while back > pos:
        back -= pos
        k -= 1
        if k < 0:
            return None
        pos = net.getEdge(route[k]).getLength()
    return k, pos - back


def stop_position(net, route, stop, rng, not_before):
    """정차할 (도로, 위치). 반반 확률로 정류장 시작 BEFORE_M m 앞 또는 정류장 끝 AFTER_M m 뒤. 그쪽이 안 되면 다른 쪽.
    not_before(골목이 합류하는 도로)보다 앞, 신호 정지선 앞 금지 구간, 도로 밖은 고르지 않는다. 둘 다 안 되면 None."""
    bad = forbidden_intervals(net, route)
    ks, kmin = route.index(stop["edge"]), route.index(not_before)
    L = net.getEdge(stop["edge"]).getLength()

    def ok(k, pos):
        return k >= kmin and not any(lo <= pos <= hi for lo, hi in bad.get(route[k], []))

    def before():
        for _ in range(12):
            p = _walk_back(net, route, ks, stop["startPos"], rng.uniform(*BEFORE_M))
            if p is not None and ok(*p):
                return route[p[0]], p[1]
        return None

    def after():
        for _ in range(12):
            pos = stop["endPos"] + rng.uniform(*AFTER_M)
            if pos <= L - 0.5 and ok(ks, pos):
                return stop["edge"], pos
        return None

    first, second = (before, after) if rng.random() < 0.5 else (after, before)
    return first() or second()


def s4_candidates(net, route, stops, alleys, max_upstream=300.0):
    """(정류장 id, 골목) 짝. 골목 합류점이 노선상 정류장 앞 max_upstream m 안이고 그 사이에 신호 교차로가 없으며 서는 자리가
    있는 것. 정류장은 시험용 버스가 서는 곳(STOP_AT)만."""
    by_id = {s["id"]: s for s in stops}
    out = []
    for sid in STOP_AT:
        s = by_id.get(sid)
        if s is None or s["edge"] not in route:
            continue
        ks = route.index(s["edge"])
        for a in alleys:
            if a.into not in route[:ks + 1]:
                continue
            ka = route.index(a.into)
            d = _gap(net, route, ka, 0.0, ks, s["startPos"])
            if d < 0 or d > max_upstream:
                continue
            if any(is_signalized(net.getEdge(route[k]).getToNode()) for k in range(ka, ks)):
                continue
            if stop_position(net, route, s, random.Random(0), a.into) is None:
                continue
            out.append((sid, a))
    return out


@dataclass
class _Block:
    stop: dict
    alley: object
    edge: str
    pos: float
    done: bool = False
    vid: str = None
    how: str = None            # "alley"(골목에서 나와 섰다) | "direct"(정차 자리에 직접 세웠다) | "none"(세울 수 없었다)
    commanded: bool = False
    confirmed: bool = False    # 그 차가 실제로 섰다(isStopped)
    t_done: float = None


class StopAheadManager:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(runner의 extras)."""

    OWNER = "S4A"

    def __init__(self, traci, bus_id, net, route, stops, alleys, lap_fn, rng=None, owner=None, n_range=(1, 3),
                 dwell=(15.0, 30.0), lead_s=40.0, min_lead_s=15.0, car_speed=8.0, spawn_types=SPAWN_TYPES,
                 alley_lead=60.0, direct_min_eta_s=8.0, free_behind_m=35.0, free_ahead_m=15.0, stop_ids=None, direct_stops=()):
        """stops: [{"id", "edge", "lane"(차로 번호), "startPos", "endPos"}]. route: 시험용 버스 한 바퀴 노선."""
        self.tc, self.bus, self.net, self.route = traci, bus_id, net, list(route)
        self.stops = {s["id"]: s for s in stops}
        self.lap_fn = lap_fn
        self.rng = rng or random.Random(0)
        self.owner = owner or Ownership()
        self.n_lo, self.n_hi = n_range                 # n_lo <= n < n_hi
        self.dwell, self.lead_s, self.min_lead_s, self.car_speed = dwell, lead_s, min_lead_s, car_speed
        self.spawn_types, self.alley_lead = tuple(spawn_types), alley_lead
        self.direct_min_eta_s, self.free_behind_m, self.free_ahead_m = direct_min_eta_s, free_behind_m, free_ahead_m
        self.by_stop = {}
        for sid, a in s4_candidates(net, self.route, stops, alleys):
            if stop_ids is None or sid in stop_ids:
                self.by_stop.setdefault(sid, []).append(a)
        for sid in direct_stops:                          # 골목이 없어도 정차 자리에 직접 세울 정류장
            if sid in self.stops and self.stops[sid]["edge"] in self.route and (stop_ids is None or sid in stop_ids):
                self.by_stop.setdefault(sid, [])
        self.direct_stops = set(direct_stops)
        self.plan = {}
        self.used, self.active, self.log = set(), {}, []
        self._lap_no, self._lap_t0 = None, 0.0
        self._n_spawn = 0

    # ---- 바퀴 ---------------------------------------------------------------------------------
    def _plan_block(self, sid):
        alleys = list(self.by_stop.get(sid, []))
        self.rng.shuffle(alleys)
        for a in alleys:
            p = stop_position(self.net, self.route, self.stops[sid], self.rng, a.into)
            if p is not None:
                return _Block(self.stops[sid], a, p[0], p[1])
        if not alleys and sid in self.direct_stops:        # 골목 없이 정차 자리에 직접 세울 정류장
            p = stop_position(self.net, self.route, self.stops[sid], self.rng, self.route[0])
            if p is not None:
                return _Block(self.stops[sid], None, p[0], p[1])
        return None

    def _new_lap(self, t):
        n = self.rng.randrange(self.n_lo, self.n_hi)
        ids = self.rng.sample(sorted(self.by_stop), min(n, len(self.by_stop)))
        self.plan = {sid: b for sid in ids for b in [self._plan_block(sid)] if b is not None}
        self._lap_t0 = t

    def _end_lap(self, t):
        self.log.append({"t0": self._lap_t0, "t1": t, "planned": sorted(self.plan),
                         "blocks": [{"stop": i, "vid": b.vid, "how": b.how, "t": b.t_done, "stopped": b.confirmed,
                                     "alley": b.alley.junction if b.alley else None, "edge": b.edge, "pos": round(b.pos, 1)}
                                    for i, b in self.plan.items()]})

    # ---- 거리 ---------------------------------------------------------------------------------
    def _bus_dist(self, edge, pos):
        """시험용 버스에서 (edge, pos)까지 노선 거리. 이번 바퀴 앞에 없으면 None."""
        tc = self.tc
        road = tc.vehicle.getRoadID(self.bus)
        if road.startswith(":"):
            return None
        route, i = list(tc.vehicle.getRoute(self.bus)), tc.vehicle.getRouteIndex(self.bus)
        j = next((k for k in range(i, min(len(route), i + len(self.route))) if route[k] == edge), None)
        if j is None:
            return None
        return _gap(self.net, route, i, tc.vehicle.getLanePosition(self.bus), j, pos)

    # ---- 생성·세우기 ---------------------------------------------------------------------------
    def _spawn(self, b):
        a = self.alley_lead
        feeder_len = self.net.getEdge(b.alley.feeder).getLength()
        k_into, k_edge = self.route.index(b.alley.into), self.route.index(b.edge)
        exit_tail = nearest_boundary_exit(self.net, b.edge) or [b.edge]
        path = [b.alley.feeder] + self.route[k_into:k_edge] + exit_tail
        for pos in sorted({max(1.0, feeder_len - a), 1.0}, reverse=True):
            xy = positionAtShapeOffset(self.net.getEdge(b.alley.feeder).getShape(), pos)
            if not far_from_bus(self.tc, self.bus, xy, PRESENCE_RADIUS_M + SPAWN_MARGIN_M):
                continue
            self._n_spawn += 1
            vid = f"stopahead{self._n_spawn}"
            try:
                self.tc.route.add(f"r_{vid}", path)
                self.tc.vehicle.add(vid, f"r_{vid}", typeID=self.rng.choice(self.spawn_types), departLane="best",
                                    departPos=f"{pos:.1f}", departSpeed="max")
            except Exception:
                return None
            return vid
        return None

    def _spot_free(self, b):
        """정차 자리 앞뒤 정차 차로에 차가 없는가(뒤 `free_behind_m` m는 달려오는 차를 위해)."""
        tc = self.tc
        for v in tc.lane.getLastStepVehicleIDs(f"{b.edge}_{b.stop['lane']}"):
            if b.pos - self.free_behind_m <= tc.vehicle.getLanePosition(v) <= b.pos + self.free_ahead_m:
                return False
        return True

    def _spawn_direct(self, b, d, v_bus):
        """골목 차가 시간에 못 맞출 때 정차 자리 6 m 뒤에 속도 0으로 세우고 곧 정차 명령을 준다. 못 세우면 None."""
        if d / v_bus < self.direct_min_eta_s:
            return None
        xy = positionAtShapeOffset(self.net.getEdge(b.edge).getShape(), b.pos)
        if not far_from_bus(self.tc, self.bus, xy, PRESENCE_RADIUS_M + SPAWN_MARGIN_M) or not self._spot_free(b):
            return None
        path = nearest_boundary_exit(self.net, b.edge) or [b.edge]
        self._n_spawn += 1
        vid = f"stopahead{self._n_spawn}"
        try:
            self.tc.route.add(f"r_{vid}", path)
            self.tc.vehicle.add(vid, f"r_{vid}", typeID=self.rng.choice(self.spawn_types),
                                departLane=str(b.stop["lane"]), departPos=f"{max(0.5, b.pos - 6.0):.1f}", departSpeed="0")
        except Exception:
            return None
        return vid

    def _car_eta(self, b):
        feeder_len = self.net.getEdge(b.alley.feeder).getLength()
        k_into, k_edge = self.route.index(b.alley.into), self.route.index(b.edge)
        d = min(self.alley_lead, feeder_len) + _gap(self.net, self.route, k_into, 0.0, k_edge, b.pos)
        return d / self.car_speed

    def _command_stop(self, b, t):
        tc = self.tc
        dur = round(self.rng.uniform(*self.dwell), 1)
        try:
            tc.vehicle.setStop(b.vid, b.edge, pos=b.pos, laneIndex=b.stop["lane"], duration=dur)
            tc.vehicle.setParameter(b.vid, "driver.stopAhead", b.stop["id"])
        except Exception:
            return False
        self.used.add(b.vid)
        self.active[b.vid] = t
        return True

    # ---- 매 스텝 ------------------------------------------------------------------------------
    def update(self, t):
        lap = self.lap_fn()
        if lap != self._lap_no:
            if self._lap_no is not None:
                self._end_lap(t)
            self._lap_no = lap
            self._new_lap(t)
        tc = self.tc
        ids = set(tc.vehicle.getIDList())
        for vid in list(self.active):
            if vid not in ids or (not tc.vehicle.isStopped(vid) and t - self.active[vid] > 5.0
                                  and tc.vehicle.getStopState(vid) == 0 and not tc.vehicle.getStops(vid, 1)):
                self.owner.release(vid, self.OWNER)
                del self.active[vid]
        if self.bus not in ids:
            return
        v_bus = max(tc.vehicle.getSpeed(self.bus), 5.0)
        for b in self.plan.values():
            if b.vid is not None and not b.commanded and b.vid in ids:     # 넣은 차가 나타나면 세울 자리를 준다
                if self.owner.claim(b.vid, self.OWNER):
                    b.commanded = self._command_stop(b, t)
            if b.vid in ids and b.commanded and not b.confirmed and tc.vehicle.isStopped(b.vid):
                b.confirmed = True
            if b.done:
                continue
            d = self._bus_dist(b.edge, b.pos)
            if d is None or d < 0:
                continue
            if b.alley is None:                              # 골목 없는 정류장: 버스가 가까워지면 정차 자리에 직접 세운다
                if d / v_bus > self.lead_s:
                    continue
                vid = self._spawn_direct(b, d, v_bus)
                if vid is not None:
                    b.done, b.vid, b.how, b.t_done = True, vid, "direct", t
                elif d / v_bus < self.direct_min_eta_s:
                    b.done, b.how = True, "none"            # 못 세운 채 버스가 너무 가까워졌다
                continue
            spare = d / v_bus - self._car_eta(b)             # 골목 차가 버스보다 몇 초 먼저 닿는가
            if spare > self.lead_s:
                continue                                    # 아직 이르다
            if spare < self.min_lead_s:                     # 골목 차가 시간에 못 맞춘다: 정차 자리에 직접 세운다
                vid = self._spawn_direct(b, d, v_bus)
                if vid is not None:
                    b.done, b.vid, b.how, b.t_done = True, vid, "direct", t
                else:
                    b.done, b.how = True, "none"
                continue
            vid = self._spawn(b)
            if vid is not None:
                b.done, b.vid, b.how, b.t_done = True, vid, "alley", t

    def summary(self):
        """끝난 바퀴와 진행 중인 바퀴를 합쳐 센다. blocks_per_lap은 바퀴마다 골목에서 내보내 세운 차 수다."""
        laps = list(self.log)
        if self._lap_no is not None:
            laps.append({"blocks": [{"stop": i, "vid": b.vid, "how": b.how, "stopped": b.confirmed}
                                    for i, b in self.plan.items()],
                         "planned": sorted(self.plan), "open": True})
        blocks = [b for lap in laps for b in lap["blocks"]]
        vids = [b["vid"] for b in blocks if b["vid"]]
        return {"laps": len(laps), "planned_per_lap": [len(lap["planned"]) for lap in laps],
                "blocks_per_lap": [sum(1 for b in lap["blocks"] if b["how"] in ("alley", "direct")) for lap in laps],
                "alley": sum(b["how"] == "alley" for b in blocks), "direct": sum(b["how"] == "direct" for b in blocks), "none": sum(b["how"] == "none" for b in blocks),
                "actually_stopped": sum(b["stopped"] for b in blocks),
                "repeated_vehicles": len(vids) - len(set(vids))}
