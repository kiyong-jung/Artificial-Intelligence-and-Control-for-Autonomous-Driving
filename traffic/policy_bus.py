"""학습한 정책이 직접 모는 시험 버스(closed-loop 평가용). 노선·바퀴·시나리오 연동은 `PDMClosedBusPilot`(= `TestBusPilot`) 그대로다.

규칙 버스와 다른 점:
  - SUMO 안전장치를 끈다: speedMode 32(안전 속도·가감속 한계·우선권·빨간불 제동을 모두 무시), laneChangeMode 0
    (SUMO 자율 차선 변경 없음, TraCI 요청은 다른 차의 간격을 보지 않고 바로 실행). 충돌·신호 위반은 그대로 일어나고 기록된다.
  - 정류장 정차 명령(setBusStop)을 걸지 않는다. 정차는 정책이 직접 한다. 정차 판정은 ServiceTracker(정류장 구역 안, 정류장 차로,
    0.1 m/s 이하로 8초)이고, 관측 v2의 dwell_remaining이 멈춘 순간 8초에서 0초까지 줄어든다.
  - 차선 변경 창(LanePlan)·도착 보장 정차·막는 차 감속·S5 감속 규칙을 쓰지 않는다. 차선은 정책 명령으로만 바꾼다.

명령(판단 0.5초마다 decide()로 받는다):
  가감속 a [m/s^2]: 0.1초마다 setSpeed(max(0, v + a*0.1)). 0.5초 뒤 속도 변화가 a*0.5가 되어 학습 라벨(구간 평균 가속도)과 같다.
  차로 5값: +-2 = 그쪽으로 차선 변경 요청(한 번) + 방향지시등, +-1 = 방향지시등만, 0 = 방향지시등을 SUMO 자동으로 되돌린다.
  +는 왼쪽(차로 번호 증가)이다.
"""
from __future__ import annotations

from .pdm_bus import PDMClosedBusPilot
from .test_bus import BLINKER_LEFT, BLINKER_RIGHT, BUS, START_EDGE

SPEED_MODE_OFF = 32      # 모든 속도 검사 끔(bit5: 교차로 안 우선권도 무시)
LC_MODE_OFF = 0          # 자율 차선 변경 없음, TraCI 요청은 안전 검사 없이
DT = 0.1
DECISION_S = 0.5


def accel_from_normalized(an: float, pos: float, neg: float) -> float:
    """정규화 가감속 [-1, 1] -> m/s^2. 학습 라벨 normalize_accel(a / pos 또는 a / neg)의 역."""
    an = max(-1.0, min(1.0, float(an)))
    return an * pos if an >= 0 else an * neg


def lane_target(cmd: int, lane_index: int, lane_count: int, internal: bool):
    """+-2 명령의 목표 차로 번호. 바꿀 수 없으면 (None, 이유)."""
    if abs(cmd) != 2:
        return None, None
    if internal:
        return None, "internal"
    target = lane_index + (1 if cmd > 0 else -1)
    if not 0 <= target < lane_count:
        return None, "no_lane"
    return target, None


def blinker_for(cmd: int) -> int:
    """방향지시등 비트. 0이면 -1(SUMO 자동)."""
    if cmd > 0:
        return BLINKER_LEFT
    if cmd < 0:
        return BLINKER_RIGHT
    return -1


class PolicyBusPilot(PDMClosedBusPilot):
    def __init__(self, tc, net, loop, stops, stop_s=8.0, cfg=None):
        super().__init__(tc, net, loop, stops, stop_s=stop_s, cfg=cfg)
        self.accel = None                  # 지금 걸린 가감속 명령 [m/s^2]. 첫 판단 전에는 None(정지 유지)
        self.lane_cmd = 0
        self._lane_pending = False
        self.policy_stats = {"decisions": 0, "lane_requests": 0, "lane_requests_sent": 0, "blink_only": 0,
                             "lane_request_rejected": {"internal": 0, "no_lane": 0}}

    # 정류장 정차 명령은 걸지 않는다(정책이 직접 선다)
    def _set_stops(self):
        return

    def _spawn(self, t):
        super()._spawn(t)
        self.tc.vehicle.setLaneChangeMode(BUS, LC_MODE_OFF)
        self.tc.vehicle.setSpeedMode(BUS, SPEED_MODE_OFF)

    def decide(self, accel_mps2: float, lane_cmd: int, t: float, info: dict = None):
        """판단 한 번. 기록기가 읽는 phase/target과 last_plan(window.pdm_plans에 남는다)도 맞춘다."""
        self.accel, self.lane_cmd = float(accel_mps2), int(lane_cmd)
        self._lane_pending = abs(self.lane_cmd) == 2
        self.policy_stats["decisions"] += 1
        self.policy_stats["lane_requests"] += int(self._lane_pending)
        self.policy_stats["blink_only"] += int(abs(self.lane_cmd) == 1)
        self.phase = {2: "request", 1: "signal"}.get(abs(self.lane_cmd), "idle")
        self.target = None
        if self.lane_cmd:
            try:
                self.target = self.tc.vehicle.getLaneIndex(BUS) + (1 if self.lane_cmd > 0 else -1)
            except Exception:
                self.target = None
        self.last_plan = {"sim_time": t, "policy": True, "accel_mps2": self.accel, "lane_cmd": self.lane_cmd,
                          **(info or {})}

    def tick(self, tc, t):
        if not self._added:
            self._spawn(t)
            return
        if BUS not in tc.vehicle.getIDList():
            return
        road = tc.vehicle.getRoadID(BUS)
        internal = road.startswith(":")
        if not internal and road != self._last_road:
            if road == START_EDGE:
                self._new_lap(t)
            self._last_road = road
        if not internal:
            self._extend()
        self._apply(tc, road, internal)

    def _apply(self, tc, road, internal):
        v = tc.vehicle
        if self.accel is None:
            v.setSpeed(BUS, 0.0)
            return
        v.setSpeed(BUS, max(0.0, v.getSpeed(BUS) + self.accel * DT))
        if self._lane_pending:
            self._lane_pending = False
            idx = v.getLaneIndex(BUS)
            n = 0 if internal else tc.edge.getLaneNumber(road)
            target, why = lane_target(self.lane_cmd, idx, n, internal)
            if target is None:
                self.policy_stats["lane_request_rejected"][why] += 1
            else:
                v.changeLane(BUS, target, DECISION_S)
                self.policy_stats["lane_requests_sent"] += 1
        v.setSignals(BUS, blinker_for(self.lane_cmd))

    def summary(self):
        s = self.policy_stats
        return (f"정책 판단 {s['decisions']}회, 차선 변경 요청 {s['lane_requests']}회(보냄 {s['lane_requests_sent']}), "
                f"방향지시등만 {s['blink_only']}회")
