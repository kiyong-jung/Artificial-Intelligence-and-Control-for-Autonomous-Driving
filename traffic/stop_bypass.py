"""S1 (R2). 차로 정차형 정류장에 선 버스 바로 뒤 차량이 한 일의 관찰: 기다렸나, 옆 차로로 추월했나.

아무것도 조종하지 않는다. 버스가 정류장에 서면(정차 사건 1회에 한 번) 같은 차로에서 버스 뒤 `follower_range` m 안에
있는 가장 가까운 차 한 대를 골라, 버스가 떠날 때까지 무엇을 했는지 기록한다. 추월할지는 그 차의 SUMO 차선 변경
모델과 운전자 성향(`yield_logic.draw_driver`)이 정한다.

- `overtook`: 그 차의 선행차가 더는 버스가 아니다(옆 차로로 옮겨 갔다).
- `waited`: 버스가 떠날 때까지 버스 뒤에 있었다.
- `gone`: 그 차가 사라졌다.
"""
from dataclasses import dataclass

STOPPED, PARKING, BUS_STOP = 1, 2, 16        # getStopState 비트


@dataclass
class _Event:
    follower: str
    t_start: float
    gap0: float                # 정차 사건을 시작할 때 버스와의 간격 [m]
    aggressiveness: float = None


class StopBypassManager:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(YieldManager.update와 같은 자리)."""

    def __init__(self, traci, bus_id, follower_range=100.0, only_lane_blocking=True):
        self.traci, self.bus = traci, bus_id
        self.follower_range = follower_range
        self.only_lane_blocking = only_lane_blocking
        self.event = None
        self._seen_stop = False                     # 이번 정차 사건에서 이미 대상을 정했는가
        self.log = []

    def _bus_blocking_stop(self):
        st = self.traci.vehicle.getStopState(self.bus)
        if not (st & STOPPED and st & BUS_STOP):
            return False
        return not (self.only_lane_blocking and st & PARKING)

    def _begin(self, t):
        tc = self.traci
        f = tc.vehicle.getFollower(self.bus, self.follower_range)
        vid = f[0] if f else ""
        if not vid or f[1] > self.follower_range:
            return
        a = tc.vehicle.getParameter(vid, "driver.aggressiveness")
        self.event = _Event(vid, t, f[1], float(a) if a else None)
        self._seen_stop = True

    def update(self, t):
        tc = self.traci
        ids = set(tc.vehicle.getIDList())
        stopped = self.bus in ids and self._bus_blocking_stop()
        ev = self.event
        if ev is not None:
            if ev.follower not in ids:
                self._finish(t, "gone")
            else:
                leader = tc.vehicle.getLeader(ev.follower, self.follower_range + 50.0)
                if not (leader and leader[0] == self.bus):
                    self._finish(t, "overtook")
                elif not stopped:
                    self._finish(t, "waited")
        if not stopped:
            self._seen_stop = False                  # 버스가 떠났다: 다음 정차는 새 사건이다
        elif self.event is None and not self._seen_stop:
            self._begin(t)

    def _finish(self, t, outcome):
        ev, self.event = self.event, None
        self.log.append({"follower": ev.follower, "t_decide": ev.t_start, "t_end": t, "gap0": ev.gap0,
                         "aggressiveness": ev.aggressiveness, "outcome": outcome})

    def summary(self):
        out = {"events": len(self.log)}
        for k in ("waited", "overtook", "gone"):
            out[k] = sum(1 for r in self.log if r["outcome"] == k)
        return out
