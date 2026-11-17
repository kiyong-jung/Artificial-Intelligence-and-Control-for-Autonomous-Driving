"""신호와 무관한 우회전(right_turn.py)은 합정역 사거리(J1)와 상수역 사거리(J82)에 적용하지 않는다."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.hapjeong import HapjeongTraffic  # noqa: E402


def test_j82_keeps_original_right_turn_signals(tmp_path):
    # J82는 우회전 수정을 넣은 뒤 교차로가 막히는 문제가 생겨 원래 신호 프로그램으로 되돌린다(사용자 결정)
    w = HapjeongTraffic(str(tmp_path), scenario="quiet", seed=0)
    assert {"J1", "J82", "J83"} <= set(w.right_on_red_exclude)


def test_j42_keeps_its_original_right_turn_signals(tmp_path):
    # J42는 우회전 신호가 원래 있다. 우회전 수정(신호 수정과 런타임 개방)을 모두 빼 원래 프로그램으로 되돌린다(사용자 결정)
    w = HapjeongTraffic(str(tmp_path), scenario="quiet", seed=0)
    assert "J42" in w.right_on_red_exclude
