"""S4. 정류장 앞에 차가 미리 서 있어 정류장으로 가는 시험용 버스의 길목을 막는다.

- 후보 정류장은 시험용 버스가 서는 곳(`STOP_AT`)이다. `stop_ids`로 제한할 수 있다(기본 설정은 s4 극동방송·s5 서교동교회·s6 합정역이고 s2 성산중학교는 뺀다). 바퀴마다 1~2곳(`n_range`)을 뽑는다.
- **차는 골목에서 나오지 않는다.** 버스가 정차 자리에 닿기 `lead_s`(40)초 안으로 다가오면, 버스 반경(+여유) 밖의 정차 자리 가까이에 차를 **미리 생성**해
  세워 둔다(`how="direct"`). 자리가 막혀 있거나 버스가 `direct_min_eta_s`초 안으로 가까워졌으면 그 바퀴는 건너뛴다(`none`).
- 서는 자리(`stop_position`): 정류장 시작 10~25 m 앞 또는 정류장 끝 5~20 m 뒤. 신호 정지선 앞 금지 구간(signal_areas)과 도로 밖은 고르지 않는다.
- **직접 배치**: 후보 정류장(s4·s5·s6) 모두 골목 없이 정차 자리에 직접 세운다.
- **버스가 오면 일정 시간 기다렸다가 출발한다**: 차는 계속 서서 기다리다가, 버스가 정차 자리 `arrive_m`(20 m) 안으로 오면(또는 지나가면) 그 시각부터 `dwell`(기본 5초)
  뒤에 출발한다. 버스가 끝내 오지 않으면 `max_wait_s` 뒤에 출발하고, 바퀴가 끝나면 남은 차도 출발시킨다. 출발한 차는 지도 경계까지 가장 가까운 길로 나간다.
- **뒤에 온 차는 강제로 추월시킨다**: 서 있는 동안 같은 차로 바로 뒤 `follower_m`(60 m) 안에 온 차(시험용 버스 제외)는 경로가 정해져 있어도(골목으로 꺾는 경로 등)
  앞으로 가는 경로로 다시 정하고 옆 차로로 추월하게 한다. 뒤차들이 줄을 서서 버스가 정차 자리에 못 다가가 정차 차가 더 오래 기다리는 것을 막는다.
  버스가 정차 자리 `arrive_far_m`(60 m) 안에서 `still_s`(2초) 넘게 서 있어도 도착으로 본다.
- 소유권: 세워 두는 동안 그 차는 `S4A`에 속한다. 끝나면 놓는다.
"""
import random
from dataclasses import dataclass

from sumolib.geomhelper import positionAtShapeOffset

from .common import Ownership
from .presence import PRESENCE_RADIUS_M, SPAWN_MARGIN_M, far_from_bus, nearest_boundary_exit
from .signal_areas import forbidden_intervals
from .test_bus import STOP_AT

SPAWN_TYPES = ("bgtaxi", "carDist")      # 정류장 앞에 서는 차종(택시, 일반 승용차 분포). 같은 확률
BEFORE_M, AFTER_M = (10.0, 25.0), (5.0, 20.0)
HOLD_S = 100000.0                         # 버스가 올 때까지 서 있는 시간(실제 출발은 resume으로 한다)


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
    not_before(노선에서 이 도로보다 앞은 고르지 않는다), 신호 정지선 앞 금지 구간, 도로 밖은 고르지 않는다. 둘 다 안 되면 None."""
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


@dataclass
class _Block:
    stop: dict
    edge: str
    pos: float
    done: bool = False
    vid: str = None
    how: str = None            # "direct"(정차 자리에 미리 세웠다) | "none"(세울 수 없었다)
    commanded: bool = False
    confirmed: bool = False    # 그 차가 실제로 섰다(isStopped)
    t_done: float = None       # 생성 시각
    seen_ahead: bool = False   # 버스가 정차 자리 앞에 있는 것을 본 적이 있다
    still_since: float = None  # 버스가 정차 자리 가까이에서 서기 시작한 시각
    t_arrive: float = None     # 버스가 정차 자리에 닿은(또는 지나간) 시각
    release_at: float = None   # 이 시각이 지나면 출발시킨다
    released: bool = False


class StopAheadManager:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(runner의 extras)."""

    OWNER = "S4A"

    def __init__(self, traci, bus_id, net, route, stops, lap_fn, rng=None, owner=None, n_range=(1, 3),
                 dwell=(5.0, 5.0), lead_s=40.0, spawn_types=SPAWN_TYPES, direct_min_eta_s=8.0, free_behind_m=35.0,
                 free_ahead_m=15.0, stop_ids=None, arrive_m=20.0, max_wait_s=150.0, follower_m=60.0, pass_ttl_s=25.0,
                 arrive_far_m=60.0, still_s=2.0, ahead_m=300.0):
        """stops: [{"id", "edge", "lane"(차로 번호), "startPos", "endPos"}]. route: 시험용 버스 한 바퀴 노선."""
        self.tc, self.bus, self.net, self.route = traci, bus_id, net, list(route)
        self.stops = {s["id"]: s for s in stops}
        self.lap_fn = lap_fn
        self.rng = rng or random.Random(0)
        self.owner = owner or Ownership()
        self.n_lo, self.n_hi = n_range                 # n_lo <= n < n_hi
        self.dwell, self.lead_s, self.spawn_types = dwell, lead_s, tuple(spawn_types)
        self.direct_min_eta_s, self.free_behind_m, self.free_ahead_m = direct_min_eta_s, free_behind_m, free_ahead_m
        self.arrive_m, self.max_wait_s = arrive_m, max_wait_s
        self.follower_m, self.pass_ttl_s, self.arrive_far_m, self.still_s, self.ahead_m = follower_m, pass_ttl_s, arrive_far_m, still_s, ahead_m
        self.passing = {}                              # 강제로 추월시키는 차 id -> {"t0", "edge", "pos", "last"}
        self.candidates = [sid for sid in STOP_AT
                           if sid in self.stops and self.stops[sid]["edge"] in self.route and (stop_ids is None or sid in stop_ids)]
        self.plan = {}
        self.used, self.active, self.log = set(), {}, []
        self._lap_no, self._lap_t0 = None, 0.0
        self._n_spawn = 0

    # ---- 바퀴 ---------------------------------------------------------------------------------
    def _plan_block(self, sid):
        p = stop_position(self.net, self.route, self.stops[sid], self.rng, self.route[0])
        return _Block(self.stops[sid], p[0], p[1]) if p is not None else None

    def _new_lap(self, t):
        n = self.rng.randrange(self.n_lo, self.n_hi)
        ids = self.rng.sample(sorted(self.candidates), min(n, len(self.candidates)))
        self.plan = {sid: b for sid in ids for b in [self._plan_block(sid)] if b is not None}
        self._lap_t0 = t

    def _end_lap(self, t):
        for b in self.plan.values():                    # 바퀴가 끝났다: 아직 서 있는 차는 출발시킨다
            self._release(b)
        self.log.append({"t0": self._lap_t0, "t1": t, "planned": sorted(self.plan),
                         "blocks": [{"stop": i, "vid": b.vid, "how": b.how, "t": b.t_done, "stopped": b.confirmed,
                                     "t_arrive": b.t_arrive, "edge": b.edge, "pos": round(b.pos, 1)}
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

    # ---- 생성·세우기·출발 ----------------------------------------------------------------------
    def _spot_free(self, b):
        """정차 자리 앞뒤 정차 차로에 차가 없는가(뒤 `free_behind_m` m는 달려오는 차를 위해)."""
        tc = self.tc
        for v in tc.lane.getLastStepVehicleIDs(f"{b.edge}_{b.stop['lane']}"):
            if b.pos - self.free_behind_m <= tc.vehicle.getLanePosition(v) <= b.pos + self.free_ahead_m:
                return False
        return True

    def _spawn(self, b, d, v_bus):
        """정차 자리 6 m 뒤에 속도 0으로 차를 세운다. 버스 반경 안이거나 자리가 막혔거나 시간이 모자라면 None."""
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

    def _command_stop(self, b, t):
        tc = self.tc
        try:
            tc.vehicle.setStop(b.vid, b.edge, pos=b.pos, laneIndex=b.stop["lane"], duration=HOLD_S)
            tc.vehicle.setParameter(b.vid, "driver.stopAhead", b.stop["id"])
        except Exception:
            return False
        self.used.add(b.vid)
        self.active[b.vid] = t
        return True

    def _release(self, b):
        """서 있는 차를 출발시킨다(정차 해제). 아직 서지 않았거나 이미 없으면 다음 스텝에 다시 한다."""
        if b.vid is None or b.released or not b.commanded:
            return
        try:
            self.tc.vehicle.resume(b.vid)
        except Exception:
            return
        b.released = True

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
        in_junction = tc.vehicle.getRoadID(self.bus).startswith(":")
        for b in self.plan.values():
            if b.vid is not None and not b.commanded and b.vid in ids:     # 넣은 차가 나타나면 서 있을 자리를 준다
                if self.owner.claim(b.vid, self.OWNER):
                    b.commanded = self._command_stop(b, t)
            if b.vid in ids and b.commanded and not b.confirmed and tc.vehicle.isStopped(b.vid):
                b.confirmed = True
            d = self._bus_dist(b.edge, b.pos)
            if b.done:
                if not in_junction:                              # 교차로 안에서는 버스 위치를 알 수 없다
                    self._wait_for_bus(b, d, t, b.vid in ids)
                if b.confirmed and not b.released and b.vid in ids:
                    self._clear_followers(b, t, ids)
                continue
            if d is None or d < 0:
                continue
            if d / v_bus > self.lead_s:
                continue                                        # 아직 이르다
            vid = self._spawn(b, d, v_bus)
            if vid is not None:
                b.done, b.vid, b.how, b.t_done = True, vid, "direct", t
            elif d / v_bus < self.direct_min_eta_s:
                b.done, b.how = True, "none"                    # 못 세운 채 버스가 너무 가까워졌다

    def _wait_for_bus(self, b, d, t, alive):
        """서 있는 차가 버스를 기다린다. 버스가 정차 자리 `arrive_m` 안으로 오면(지나가도) 그때부터 dwell 뒤에 출발시킨다."""
        if b.vid is None or b.released or not alive:
            return
        if d is not None and d > 0:
            b.seen_ahead = True
        # 뒤에 줄 선 차들 때문에 버스가 정차 자리 arrive_m 안으로 못 들어와도, 가까이(arrive_far_m)에서 still_s 넘게 서 있으면 도착으로 본다
        if d is not None and 0 < d <= self.arrive_far_m and self.tc.vehicle.getSpeed(self.bus) < 0.5:
            b.still_since = t if b.still_since is None else b.still_since
        else:
            b.still_since = None
        waiting = b.still_since is not None and t - b.still_since >= self.still_s
        if b.t_arrive is None and ((d is not None and d <= self.arrive_m) or waiting or (d is None and b.seen_ahead)):
            b.t_arrive = t
            b.release_at = t + round(self.rng.uniform(*self.dwell), 1)
        if b.release_at is None and b.t_done is not None and t - b.t_done > self.max_wait_s:
            b.release_at = t                                     # 버스가 끝내 오지 않았다
        if b.release_at is not None and t >= b.release_at:
            self._release(b)

    # ---- 뒤에 온 차 강제 추월 --------------------------------------------------------------
    def _forward_path(self, edge):
        """edge에서 시작해 버스 노선을 따라 약 `ahead_m` 앞까지 가고 지도 경계로 나가는 경로(골목으로 꺾지 않는다). 노선 밖이면 None."""
        if edge not in self.route:
            return None
        out, total = [edge], self.net.getEdge(edge).getLength()
        for e in self.route[self.route.index(edge) + 1:]:
            if total >= self.ahead_m:
                break
            out.append(e)
            total += self.net.getEdge(e).getLength()
        tail = nearest_boundary_exit(self.net, out[-1])
        return out[:-1] + tail if tail else out

    def _pass_lane(self, edge, lane, pos):
        """추월할 옆 차로 번호(왼쪽 우선). 승용차가 못 쓰거나 그 자리에 차가 있으면 None."""
        ed = self.net.getEdge(edge)
        for k in (lane + 1, lane - 1):
            if not (0 <= k < ed.getLaneNumber() and ed.getLane(k).allows("passenger")):
                continue
            if all(abs(self.tc.vehicle.getLanePosition(v) - pos) > 12.0 for v in self.tc.lane.getLastStepVehicleIDs(f"{edge}_{k}")):
                return k
        return None

    def _clear_followers(self, b, t, ids):
        """서 있는 S4 차(b) 뒤 같은 차로에 온 차를 경로를 고쳐 강제로 추월시키고, 지나갔거나 시간이 지나면 놓는다."""
        tc = self.tc
        lane_id = f"{b.edge}_{b.stop['lane']}"
        for vid in tc.lane.getLastStepVehicleIDs(lane_id):
            if vid == self.bus or vid == b.vid or vid in self.passing or vid not in ids:
                continue
            pos = tc.vehicle.getLanePosition(vid)
            if not (b.pos - self.follower_m <= pos < b.pos) or not self.owner.claim(vid, self.OWNER):
                continue
            path = self._forward_path(b.edge)
            try:
                if path:
                    tc.vehicle.setRoute(vid, path)
            except Exception:
                pass
            self.passing[vid] = {"t0": t, "edge": b.edge, "pos": b.pos, "last": -1e9, "lane": b.stop["lane"]}
        for vid, rec in list(self.passing.items()):
            if rec["edge"] != b.edge or vid not in ids:
                continue
            road = tc.vehicle.getRoadID(vid)
            done = road != b.edge or tc.vehicle.getLanePosition(vid) > rec["pos"] + 5.0 or t - rec["t0"] > self.pass_ttl_s or b.released
            if done:
                self._stop_passing(vid)
                continue
            if t - rec["last"] >= 1.0 and tc.vehicle.getLaneIndex(vid) == rec["lane"]:
                k = self._pass_lane(b.edge, rec["lane"], tc.vehicle.getLanePosition(vid))
                if k is not None:
                    tc.vehicle.changeLane(vid, k, 5.0)
                rec["last"] = t

    def _stop_passing(self, vid):
        self.passing.pop(vid, None)
        self.owner.release(vid, self.OWNER)
        try:
            self.tc.vehicle.changeLane(vid, self.tc.vehicle.getLaneIndex(vid), 0.0)
        except Exception:
            pass

    def summary(self):
        """끝난 바퀴와 진행 중인 바퀴를 합쳐 센다. blocks_per_lap은 바퀴마다 정류장 앞에 미리 세운 차 수다."""
        laps = list(self.log)
        if self._lap_no is not None:
            laps.append({"blocks": [{"stop": i, "vid": b.vid, "how": b.how, "stopped": b.confirmed, "t_arrive": b.t_arrive}
                                    for i, b in self.plan.items()],
                         "planned": sorted(self.plan), "open": True})
        blocks = [b for lap in laps for b in lap["blocks"]]
        vids = [b["vid"] for b in blocks if b["vid"]]
        return {"laps": len(laps), "planned_per_lap": [len(lap["planned"]) for lap in laps],
                "blocks_per_lap": [sum(1 for b in lap["blocks"] if b["how"] == "direct") for lap in laps],
                "direct": sum(b["how"] == "direct" for b in blocks), "none": sum(b["how"] == "none" for b in blocks),
                "actually_stopped": sum(b["stopped"] for b in blocks),
                "bus_arrived": sum(b["t_arrive"] is not None for b in blocks),
                "repeated_vehicles": len(vids) - len(set(vids))}
