"""S4 동적판. 시험용 버스가 정류장에 들어가려 할 때, 그 앞에 있던 택시·버스가 정류장 앞에 서서 버스를 막는다.

- 시험용 버스가 한 바퀴(노선 한 번)를 도는 동안, 정류장 6곳 중 무작위로 `n`곳에서 막힌다. n은 1 이상 3 미만(1 또는 2)이고 곳은 무작위다.
- 막는 쪽은 버스가 그 정류장으로 들어가려 할 때(다음 정차 정류장이 그곳이고 `trigger_dist` m 이내) **버스 앞**에 있는 택시·버스다.
  그중 정류장에 가장 가까운 한 대를 고르고, 정류장 차로가 아닌 차로에 있으면 정류장 차로로 차선 변경을 요청한 뒤 정류장 앞에
  정차시킨다(TraCI `changeLane`, `setStop`/`setBusStop`).
- 앞에 그런 차가 없으면 택시 한 대를 **미리 투입**한다: 시험용 버스에서 `min_spawn_dist`(200 m) 이상 앞의 도로 위에 넣어
  달려와 정류장 앞에 서게 한다(버스 눈앞에 불쑥 생기지 않는다). 정류장이 너무 가까워 미리 넣을 수 없으면 그 정류장에서는
  막지 않는다(`none`으로 기록). 정류장 앞에 바로 넣는 방식은 쓰지 않는다. 그래서 한 바퀴에 최소 1회는 보장되지 않는다.
- 같은 차가 정차 행동을 반복하지 않는다. 한 번 세운 차는 `driver.stopAhead`로 표시하고 다시 고르지 않는다.
- 소유권: 세우는 동안 그 차는 `S4A`에 속한다. 끝나면 놓는다.
- 후보에서 뺀다: 노선 정류장에 원래 서는 배경 버스(`bgbus`)는 이미 정류장마다 서므로 '정차 행동 반복'이 되어 후보가 아니다.
"""
import random
from dataclasses import dataclass, field

from .common import Ownership

CANDIDATE_TYPES = ("bgtaxi", "bgbus_pass")      # 정류장에 원래 서지 않는 택시·버스
TAXI_TYPE = "bgtaxi"


@dataclass
class _Block:
    stop: dict
    done: bool = False
    t_done: float = None
    vid: str = None
    how: str = None            # "existing" | "spawned_far"(미리 투입) | "none"(세울 수 없었다)
    confirmed: bool = False    # 그 차가 실제로 섰다(isStopped)


class StopAheadManager:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(YieldManager.update와 같은 자리)."""

    OWNER = "S4A"

    def __init__(self, traci, bus_id, stops, rng=None, owner=None, n_range=(1, 3), trigger_dist=400.0,
                 dwell=(15.0, 30.0), spawn_if_missing=True, candidate_types=CANDIDATE_TYPES,
                 min_spawn_dist=200.0, min_before_stop=60.0):
        """stops: [{"id", "edge", "lane"(차로 번호), "startPos", "endPos"}] 시험용 버스가 서는 정류장들(6곳)."""
        self.tc, self.bus = traci, bus_id
        self.stops = {s["id"]: s for s in stops}
        self.rng = rng or random.Random(0)
        self.owner = owner or Ownership()
        self.n_lo, self.n_hi = n_range                 # n_lo <= n < n_hi
        self.trigger_dist, self.dwell = trigger_dist, dwell
        self.spawn_if_missing = spawn_if_missing
        self.min_spawn_dist, self.min_before_stop = min_spawn_dist, min_before_stop
        self.candidate_types = tuple(candidate_types)
        self.plan = {}                 # 이번 바퀴에 막을 정류장 id -> _Block
        self.used = set()              # 이미 정차 행동을 한 차(반복하지 않는다)
        self.active = {}               # 차 id -> 끝나야 하는 시각 이후 해제용
        self.log = []                  # 바퀴마다 한 줄
        self._lap_active = False
        self._n_spawn = 0

    # ---- 바퀴 시작·끝 ---------------------------------------------------------------------
    def _new_lap(self, t):
        n = self.rng.randrange(self.n_lo, self.n_hi)
        ids = self.rng.sample(sorted(self.stops), min(n, len(self.stops)))
        self.plan = {i: _Block(self.stops[i]) for i in ids}
        self._lap_active = True
        self._lap_t0 = t

    def _end_lap(self, t):
        self.log.append({"t0": self._lap_t0, "t1": t, "planned": sorted(self.plan),
                         "blocks": [{"stop": i, "vid": b.vid, "how": b.how, "t": b.t_done, "stopped": b.confirmed}
                                    for i, b in self.plan.items()]})
        self.plan, self._lap_active = {}, False

    # ---- 거리 (모두 시험용 버스의 경로를 기준으로 잰다) ---------------------------------------
    def _len(self, edge):
        return self.tc.lane.getLength(f"{edge}_0")

    def _gap(self, route, i, pos, j, pos2):
        """경로상 (route[i], pos)에서 (route[j], pos2)까지 앞으로 가는 거리 [m]. j >= i."""
        if j == i:
            return pos2 - pos
        d = self._len(route[i]) - pos
        for e in route[i + 1:j]:
            d += self._len(e)
        return d + pos2

    def _bus_state(self):
        tc = self.tc
        cur = tc.vehicle.getRoadID(self.bus)
        i = tc.vehicle.getRouteIndex(self.bus)
        if cur.startswith(":") or i < 0:
            return None
        return list(tc.vehicle.getRoute(self.bus)), i, tc.vehicle.getLanePosition(self.bus)

    def _dist_to_stop(self, stop):
        """시험용 버스에서 정류장 시작까지 경로상 거리 [m]. 이 경로에 정류장 도로가 없거나 지났으면 None."""
        st = self._bus_state()
        if st is None or stop["edge"] not in st[0][st[1]:]:
            return None
        route, i, pos = st
        j = route.index(stop["edge"], i)
        d = self._gap(route, i, pos, j, stop["startPos"])
        return d if d >= 0 else None

    # ---- 세우기 -----------------------------------------------------------------------------
    def _stop_pos(self, stop):
        return max(8.0, stop["startPos"] - 3.0)               # 정류장 바로 앞(그 차로를 막는다)

    def _command_stop(self, vid, stop, t):
        """vid를 정류장 앞에 세운다. 정류장 차로가 아닌 차로에 있으면 정류장 차로로 차선 변경을 요청한다."""
        tc = self.tc
        dur = round(self.rng.uniform(*self.dwell), 1)
        base = tc.vehicle.getTypeID(vid).split("@")[0]
        try:
            if base.startswith("bgbus"):
                tc.vehicle.setBusStop(vid, stop["id"], duration=dur)
            else:
                tc.vehicle.setStop(vid, stop["edge"], pos=self._stop_pos(stop), laneIndex=stop["lane"], duration=dur)
        except Exception:
            return False
        try:
            if tc.vehicle.getRoadID(vid) == stop["edge"] and tc.vehicle.getLaneIndex(vid) != stop["lane"]:
                tc.vehicle.changeLane(vid, stop["lane"], 20.0)   # 정류장 차로로 들어간다
        except Exception:
            pass
        self.used.add(vid)
        tc.vehicle.setParameter(vid, "driver.stopAhead", stop["id"])
        self.active[vid] = t
        return True

    def _find_ahead(self, stop):
        """시험용 버스 앞에 있는 택시·버스 중 정류장에 가장 가까운 한 대. 없으면 None."""
        tc = self.tc
        st = self._bus_state()
        if st is None or stop["edge"] not in st[0][st[1]:]:
            return None
        route, i, pos_b = st
        j_stop = route.index(stop["edge"], i)
        best = None
        for vid in tc.vehicle.getIDList():
            if vid == self.bus or vid in self.used or self.owner.owner(vid) not in (None, self.OWNER):
                continue
            if tc.vehicle.getTypeID(vid).split("@")[0] not in self.candidate_types:
                continue
            road = tc.vehicle.getRoadID(vid)
            if road.startswith(":") or road not in route[i:] or stop["edge"] not in tc.vehicle.getRoute(vid):
                continue
            j_v = route.index(road, i)
            pos_v = tc.vehicle.getLanePosition(vid)
            if j_v > j_stop or (j_v == j_stop and pos_v >= stop["startPos"] - 6.0):
                continue                                   # 이미 정류장 앞을 지났거나 너무 가까워 설 수 없다
            d_bus = self._gap(route, i, pos_b, j_v, pos_v)
            if d_bus < 8.0 or d_bus > self.trigger_dist + 60:
                continue                                   # 버스 뒤에 있거나 너무 멀다
            d_stop = self._gap(route, j_v, pos_v, j_stop, stop["startPos"])
            if best is None or d_stop < best[0]:
                best = (d_stop, vid)                       # 정류장에 가장 가까운 차
        return best[1] if best else None

    def _add_taxi(self, route_edges, lane, pos, speed):
        self._n_spawn += 1
        vid = f"stopahead{self._n_spawn}"
        try:
            self.tc.route.add(f"r_{vid}", route_edges)
            self.tc.vehicle.add(vid, f"r_{vid}", typeID=TAXI_TYPE, departLane=lane, departPos=f"{pos:.1f}",
                                departSpeed=speed)
        except Exception:
            return None
        return vid

    def _spawn_far(self, stop):
        """미리 투입: 시험용 버스에서 min_spawn_dist 이상 앞의 도로 입구에 택시를 넣어 달려오게 한다."""
        st = self._bus_state()
        if st is None or stop["edge"] not in st[0][st[1]:]:
            return None
        route, i, pos_b = st
        j = route.index(stop["edge"], i)
        d_stop = self._gap(route, i, pos_b, j, stop["startPos"])
        g = min(self.min_spawn_dist, d_stop - self.min_before_stop)     # 버스에서 g m 앞, 정류장 앞 min_before_stop m 이상
        if g < 0.6 * self.min_spawn_dist:
            return None                                                 # 너무 가까워 미리 넣을 수 없다
        k, pos, left = i, pos_b, g                                     # 버스에서 g m 앞 지점을 경로를 따라 찾는다
        while left > 0 and k <= j:
            room = self._len(route[k]) - pos
            if left <= room:
                pos += left
                left = 0
                break
            left -= room
            k, pos = k + 1, 0.0
        if left > 0 or k > j:
            return None
        return self._add_taxi(route[k:j + 2], "best", max(1.0, pos), "max")

    # ---- 매 스텝 ----------------------------------------------------------------------------
    def update(self, t):
        tc = self.tc
        ids = set(tc.vehicle.getIDList())
        for vid in list(self.active):
            if vid not in ids or (not tc.vehicle.isStopped(vid) and t - self.active[vid] > 5.0
                                  and tc.vehicle.getStopState(vid) == 0 and not tc.vehicle.getStops(vid, 1)):
                self.owner.release(vid, self.OWNER)
                del self.active[vid]
        if self.bus not in ids:
            if self._lap_active:
                self._end_lap(t)
            return
        if not self._lap_active:
            self._new_lap(t)
        nxt = tc.vehicle.getStops(self.bus, 1)
        next_id = nxt[0].stoppingPlaceID if nxt else None
        for b in self.plan.values():                     # 실제로 섰는지 확인한다
            if b.vid in ids and not b.confirmed and tc.vehicle.isStopped(b.vid):
                b.confirmed = True
        blk = self.plan.get(next_id)
        if blk is None or blk.done:
            return
        d = self._dist_to_stop(blk.stop)
        if d is None or d > self.trigger_dist:
            return
        vid, how = self._find_ahead(blk.stop), "existing"
        if vid is None and self.spawn_if_missing:
            vid, how = self._spawn_far(blk.stop), "spawned_far"       # 정류장 앞에 바로 넣는 방식은 쓰지 않는다
        if vid is None:
            if self._dist_to_stop(blk.stop) is not None and self._dist_to_stop(blk.stop) < 25.0:
                blk.done, blk.how = True, "none"          # 더는 세울 수 없다(기록만 한다)
            return
        if not self.owner.claim(vid, self.OWNER):
            return
        if not self._command_stop(vid, blk.stop, t):
            self.owner.release(vid, self.OWNER)
            if how != "existing":
                blk.done, blk.how = True, "none"
            return
        blk.done, blk.vid, blk.how, blk.t_done = True, vid, how, t

    def summary(self):
        """끝난 바퀴와 진행 중인 바퀴를 합쳐 센다. blocks_per_lap은 바퀴마다 실제로 정차시킨 차 수다."""
        laps = list(self.log)
        if self._lap_active:
            laps.append({"blocks": [{"stop": i, "vid": b.vid, "how": b.how, "stopped": b.confirmed}
                                    for i, b in self.plan.items()],
                         "planned": sorted(self.plan), "open": True})
        blocks = [b for lap in laps for b in lap["blocks"]]
        vids = [b["vid"] for b in blocks if b["vid"]]
        made = ("existing", "spawned_far")
        return {"laps": len(laps), "planned_per_lap": [len(lap["planned"]) for lap in laps],
                "blocks_per_lap": [sum(1 for b in lap["blocks"] if b["how"] in made) for lap in laps],
                **{k: sum(b["how"] == k for b in blocks) for k in made + ("none",)},
                "actually_stopped": sum(b["stopped"] for b in blocks),
                "repeated_vehicles": len(vids) - len(set(vids))}
