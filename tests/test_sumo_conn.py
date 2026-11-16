"""SUMO 실행 공통 설정."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.sumo_conn import COMMON_ARGS  # noqa: E402


def test_pedestrian_jam_resolves_quickly():
    # busy 측정에서 J82 보행 구역의 보행자와 교차로 안의 시험용 버스가 서로 기다리며 약 300초(SUMO 기본 jamtime) 멈췄고,
    # 그 사이 SUMO가 버스 옆 차 4대를 순간이동시켰다. 보행자가 20초 뒤 비집고 지나가게 한다
    i = COMMON_ARGS.index("--pedestrian.striping.jamtime")
    assert float(COMMON_ARGS[i + 1]) == 20.0
