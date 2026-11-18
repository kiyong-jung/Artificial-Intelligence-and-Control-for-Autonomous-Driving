"""추가 행동 알고리즘이 함께 쓰는 부품: 소유권, 공격성 값 변환, 차로 고정.

SUMO 기본 알고리즘 위에 얹는 추가 행동은 상호작용 구간에서만 개입하고, 끝나면 SUMO 설정을 원래대로 돌려놓는다.
- 한 차량은 한 순간에 하나의 추가 행동에만 속한다(`Ownership`).
- 차로 고정은 `changeLane(현재 차로, 짧은 지속)`을 갱신하는 방식이다. 갱신을 멈추면 지속 시간이 지나 자동으로 풀린다.
  laneChangeMode 자체는 바꾸지 않는다.
"""
from dataclasses import dataclass


class Ownership:
    """차량 id -> 그 차량을 지금 다루는 추가 행동 이름. 양보 중인 차가 회피를 시작하는 일을 막는다."""

    def __init__(self):
        self._owner = {}

    def claim(self, vid, who):
        cur = self._owner.get(vid)
        if cur is not None and cur != who:
            return False
        self._owner[vid] = who
        return True

    def release(self, vid, who):
        if self._owner.get(vid) == who:
            del self._owner[vid]

    def owner(self, vid):
        return self._owner.get(vid)


@dataclass
class AggressionMap:
    """공격성 a(0~1)를 시나리오별 확률로 바꾼다. 값 범위는 자리표시(현장 자료로 바꿔야 한다).

    A1 양보 확률   p_yield    = 1 - a
    S1 추월 확률   p_overtake = overtake_lo + (overtake_hi - overtake_lo) * a
    S5 무시 확률   p_ignore   = ignore_lo   + (ignore_hi   - ignore_lo)   * a   (cutin.default_bins와 같은 식)
    """
    overtake_lo: float = 0.2
    overtake_hi: float = 0.9
    ignore_lo: float = 0.2
    ignore_hi: float = 1.0

    def p_yield(self, a):
        return min(1.0, max(0.0, 1.0 - a))

    def p_overtake(self, a):
        return self.overtake_lo + (self.overtake_hi - self.overtake_lo) * a

    def p_ignore(self, a):
        return self.ignore_lo + (self.ignore_hi - self.ignore_lo) * a

    def aggressiveness_from_ignore(self, p):
        return (p - self.ignore_lo) / (self.ignore_hi - self.ignore_lo)


AGGRESSION = AggressionMap()


class LaneLock:
    """차량을 지금 차로에 붙들어 둔다(기본 차선변경 모델의 자발적 차선변경을 막는다)."""

    def __init__(self, traci, duration=1.0, renew=0.5):
        self.tc, self.duration, self.renew = traci, duration, renew
        self._last = {}          # vid -> 마지막 갱신 시각

    def hold(self, vid, t):
        if t - self._last.get(vid, -1e9) >= self.renew - 1e-9:
            self.tc.vehicle.changeLane(vid, self.tc.vehicle.getLaneIndex(vid), self.duration)
            self._last[vid] = t

    def release(self, vid):
        self._last.pop(vid, None)
        try:
            # 지속 시간 0 = 요청 해제. 차량이 이미 사라졌으면 무시한다.
            self.tc.vehicle.changeLane(vid, self.tc.vehicle.getLaneIndex(vid), 0.0)
        except Exception:
            pass

    def held(self, vid):
        return vid in self._last
