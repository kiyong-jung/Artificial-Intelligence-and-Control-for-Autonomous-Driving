"""S1 (R2). 차로 정차형 정류장에 선 버스 바로 뒤 차량의 행동: 기다림 또는 차선변경으로 추월.

대상은 정차한 버스 바로 뒤 한 대뿐이다. 정차 사건 1회에 한 번만 {기다림, 추월}을 뽑는다. 추월 확률은 그 차의 공격성 값
(`driver.aggressiveness`)에서 나온다(`common.AggressionMap.p_overtake`).

- 기다림: 차로 고정(changeLane 갱신)으로 구현한다. SUMO 기본 차선변경 모델은 정차한 버스에 막힌 차를 스스로 옆 차로로 보내
  (사실상 항상 추월) 고정하지 않으면 확률이 의미가 없다. 고정이 새는 정도는 `leaks`로 센다.
- 추월: 반응 지연(기본 1~3 s) 뒤 changeLane으로 옆 차로를 요청한다. 안전 검사는 SUMO에 맡긴다. timeout을 넘기면 '막힘'으로
  기록하고 제약을 푼다. 재합류(버스 앞 차로로 돌아오기)는 기본 모델에 맡긴다.
- 버스와 다른 차로에서 합류하는 차량은 이 모듈이 아니라 R1(`yield_logic`)이 다룬다.
- 정차 사건이 끝나면(버스 출발, 버스·차량 소멸) 제약을 푼다.
"""
import random
from dataclasses import dataclass

from .common import AGGRESSION, LaneLock, Ownership

STOPPED, PARKING, BUS_STOP = 1, 2, 16        # getStopState 비트


@dataclass
class _Event:
    follower: str
    action: str                # "wait" | "overtake"
    p: float                   # 그 차의 추월 확률
    t_decide: float
    delay: float
    lane0: int
    edge0: str = ""
    target: int = None         # 추월 목표 차로
    t_request: float = None
    outcome: str = None        # "overtook" | "blocked" | "waited" | "leaked"(기다림인데 차로를 바꿨다)
    t_end: float = None


class StopBypassManager:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(YieldManager.update와 같은 자리)."""

    OWNER = "S1"

    def __init__(self, traci, bus_id, rng=None, owner=None, aggression=None, delay=(1.0, 3.0),
                 timeout=15.0, follower_range=200.0, lock_wait=True, only_lane_blocking=True,
                 p_overtake=None, dt=0.1):
        self.traci, self.bus = traci, bus_id
        self.rng = rng or random.Random(0)
        self.owner = owner or Ownership()
        self.aggression = aggression or AGGRESSION
        self.delay, self.timeout, self.follower_range = delay, timeout, follower_range
        self.lock_wait = lock_wait                  # False면 기다림도 고정하지 않는다(누출 측정용 대조군)
        self.only_lane_blocking = only_lane_blocking
        self.p_overtake = p_overtake                # 주면 공격성 값 대신 이 확률을 쓴다(검증용)
        self.lock = LaneLock(traci)
        self.dt = dt
        self.event = None
        self._seen_stop = False                     # 이번 정차 사건에서 이미 대상을 정했는가
        self.log = []

    # ---- 정차 사건 ---------------------------------------------------------------------------
    def _bus_blocking_stop(self):
        st = self.traci.vehicle.getStopState(self.bus)
        if not (st & STOPPED and st & BUS_STOP):
            return False
        return not (self.only_lane_blocking and st & PARKING)

    def _prob(self, vid):
        if self.p_overtake is not None:
            return self.p_overtake
        a = self.traci.vehicle.getParameter(vid, "driver.aggressiveness")
        return None if a == "" else self.aggression.p_overtake(float(a))

    def _drivable(self, vid, idx):
        tc = self.traci
        edge = tc.vehicle.getRoadID(vid)
        if edge.startswith(":") or idx < 0 or idx >= tc.edge.getLaneNumber(edge):
            return False
        allowed = tc.lane.getAllowed(f"{edge}_{idx}")
        return not allowed or "passenger" in allowed

    def _begin(self, t):
        tc = self.traci
        f = tc.vehicle.getFollower(self.bus, self.follower_range)
        vid = f[0] if f else ""
        if not vid or f[1] > self.follower_range or vid not in tc.vehicle.getIDList():
            return
        p = self._prob(vid)
        if p is None or not self.owner.claim(vid, self.OWNER):
            return                                   # 성격이 없는 차량이거나 다른 추가 행동에 속한 차량
        action = "overtake" if self.rng.random() < p else "wait"
        d = self.rng.uniform(*self.delay)
        self.event = _Event(vid, action, p, t, d, tc.vehicle.getLaneIndex(vid), tc.vehicle.getRoadID(vid))
        self._seen_stop = True

    # ---- 매 스텝 -----------------------------------------------------------------------------
    def update(self, t):
        tc = self.traci
        ids = set(tc.vehicle.getIDList())
        stopped = self.bus in ids and self._bus_blocking_stop()
        ev = self.event
        if ev is not None:
            if ev.follower not in ids or not stopped:
                self._finish(t, ev.follower in ids)
            else:
                self._act(ev, t)
        if not stopped:
            self._seen_stop = False                  # 버스가 떠났다: 다음 정차는 새 사건이다
        elif self.event is None and not self._seen_stop:
            self._begin(t)

    def _act(self, ev, t):
        tc = self.traci
        vid = ev.follower
        idx = tc.vehicle.getLaneIndex(vid)
        if tc.vehicle.getRoadID(vid) != ev.edge0:       # 도로가 바뀌면 차로 번호가 달라질 수 있다: 기준을 새로 잡는다
            ev.edge0, ev.lane0 = tc.vehicle.getRoadID(vid), idx
            if ev.target is not None:
                ev.target = None
        if ev.action == "wait":
            if idx != ev.lane0:
                ev.outcome = "leaked"                # 고정이 새서 기다리지 않았다
                self._finish(t, True)
            elif self.lock_wait:
                self.lock.hold(vid, t)
            return
        if ev.outcome is not None:
            return
        if idx != ev.lane0:                          # 옆 차로로 옮겨 갔다: 추월 성공. 이후는 기본 모델에 맡긴다
            ev.outcome = "overtook"
            self._finish(t, True)
            return
        if t - ev.t_decide < ev.delay:
            self.lock.hold(vid, t)                   # 반응 지연 동안은 기본 모델이 먼저 움직이지 못하게 한다
            return
        if ev.target is None:
            ev.target = next((k for k in (idx + 1, idx - 1) if self._drivable(vid, k)), None)
            if ev.target is None:
                ev.outcome = "blocked"
                self._finish(t, True)
                return
            ev.t_request = t
        if t - ev.t_request > self.timeout:
            ev.outcome = "blocked"
            self._finish(t, True)
            return
        if int(round((t - ev.t_request) / self.dt)) % 10 == 0:
            tc.vehicle.changeLane(vid, ev.target, 3.0)

    def _finish(self, t, alive):
        ev, self.event = self.event, None
        if ev.outcome is None:
            ev.outcome = "waited" if ev.action == "wait" else "blocked"
        ev.t_end = t
        if alive:
            self.lock.release(ev.follower)
        self.owner.release(ev.follower, self.OWNER)
        self.log.append({"follower": ev.follower, "action": ev.action, "p": ev.p, "t_decide": ev.t_decide,
                         "delay": ev.delay, "t_request": ev.t_request, "t_end": t, "outcome": ev.outcome})

    def summary(self):
        n = len(self.log)
        out = {"events": n}
        for k in ("waited", "leaked", "overtook", "blocked"):
            out[k] = sum(1 for r in self.log if r["outcome"] == k)
        out["overtake_decisions"] = sum(1 for r in self.log if r["action"] == "overtake")
        return out
