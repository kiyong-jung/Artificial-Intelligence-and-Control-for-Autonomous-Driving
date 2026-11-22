"""공격성 값 변환(S5 끼어들기 vType 구간)."""
from dataclasses import dataclass


@dataclass
class AggressionMap:
    """공격성 a(0~1)를 시나리오별 확률로 바꾼다. 값 범위는 자리표시(현장 자료로 바꿔야 한다).

    S5 무시 확률   p_ignore   = ignore_lo   + (ignore_hi   - ignore_lo)   * a   (cutin.default_bins와 같은 식)
    """
    ignore_lo: float = 0.2
    ignore_hi: float = 1.0

    def p_ignore(self, a):
        return self.ignore_lo + (self.ignore_hi - self.ignore_lo) * a

    def aggressiveness_from_ignore(self, p):
        return (p - self.ignore_lo) / (self.ignore_hi - self.ignore_lo)


AGGRESSION = AggressionMap()
