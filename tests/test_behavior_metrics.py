"""행동 지표의 횡가속도는 기록된 속도와 방향 변화에서 나온다."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "research", "av_bc_rl"))
import behavior_metrics  # noqa: E402


def _records(speeds, headings, roads):
    """판단마다의 (속도, 방향, 도로)로 지표가 읽는 최소 기록을 만든다. 마지막 값은 끝난 뒤의 상태다."""
    states = [{"ego": {"speed": v, "heading": h, "road_id": r}, "navigation": {"distance_to_destination": 100.0}}
              for v, h, r in zip(speeds, headings, roads)]
    return [{"episode": 0, "t": t, "state": state, "next_state": following, "observation": [1.0] * 43,
             "action": {"accel_raw": 0.0, "lane_change": 0, "lane_change_applied": True, "lane_change_actual": 0},
             "comfort": {"jerk_violation": False, "acceleration_violation": False},
             "event": {"arrived": False}, "truncated": True}
            for t, (state, following) in enumerate(zip(states, states[1:]))]


def test_lateral_acceleration_is_speed_times_yaw_rate():
    """6m/s로 판단(0.5초)마다 10도씩 돈다: 6 × (10도/0.5초) = 2.09m/s². 359도에서 9도로 넘어가도 10도다."""
    records = _records([6.0] * 4, [339.0, 349.0, 359.0, 9.0], ["R1"] * 4)
    assert behavior_metrics.lateral_acceleration(records) == pytest.approx([6.0 * math.radians(10.0) / 0.5] * 3)


def test_episode_reports_the_peak_and_where_it_was():
    records = _records([6.0, 6.0, 10.0, 10.0, 10.0], [0.0, 0.0, 10.0, 30.0, 30.0], ["R1", ":J82_15", ":J82_24", "R2", "R2"])
    metrics = behavior_metrics.episode_metrics(records)
    peak = 10.0 * math.radians(20.0) / 0.5
    assert metrics["lateral_accel_peak"] == pytest.approx(peak)
    assert metrics["lateral_accel_peak_by_place"] == pytest.approx({"J82": peak})
    assert metrics["lateral_accel_violation_rate"] == pytest.approx(2 / 4)      # 네 판단 중 둘이 2m/s²를 넘었다
    summary = behavior_metrics.summarize([metrics])
    assert summary["lateral_accel_peak"]["mean"] == pytest.approx(peak)
    assert summary["lateral_accel_peak_by_place_max"] == pytest.approx({"J82": peak})


def test_straight_driving_has_none():
    metrics = behavior_metrics.episode_metrics(_records([12.0] * 4, [90.0] * 4, ["R1"] * 4))
    assert metrics["lateral_accel_peak"] == 0.0 and metrics["lateral_accel_peak_by_place"] == {}


@pytest.mark.parametrize("light, stuck", [("r", 0), ("G", 1)])
def test_a_long_wait_in_a_red_light_queue_is_not_stuck(light, stuck):
    """정지선 60m 뒤에서 앞차 4m 뒤에 65초 섰다. 적색 대기열이면 갇힌 것이 아니고, 녹색인데 서 있으면 갇힌 것이다."""
    records = _records([0.0] * 131, [90.0] * 131, ["R1"] * 131)
    for record in records:
        record["state"] = {**record["state"], "traffic_light": {"state": light, "distance_to_stop_line": 60.0}}
        record["observation"] = [4.0 / 50.0 if index == 9 else value for index, value in enumerate(record["observation"])]
    assert behavior_metrics.episode_metrics(records)["stuck_events"] == stuck
