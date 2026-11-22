"""전문가 v5는 v4가 실패하거나 SUMO가 속도를 깎은 상황에서 다르게 판단하고(관측만으로), v4 판단은 그대로다."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from env import sumo_env  # noqa: E402
from env.data_schema import OBSERVATION_FEATURES  # noqa: E402
from env.route_policy import ExpertPolicy, ExpertPolicyV5  # noqa: E402

VMAX, VIS = 13.89, 50.0
EMPTY = {"left2": None, "left1": None, "current": None, "right1": None, "right2": None}


def observe(speed, **features):
    """빈 도로의 관측에 features(m, m/s 단위)를 덮어쓴다. lane_cars={차로: (앞차 간격, 앞차 속도, 뒤차 간격, 뒤차 속도)}."""
    values = {name: 0.0 for name in OBSERVATION_FEATURES}
    for lane in EMPTY:
        values.update({f"{lane}_leader_gap": 1.0, f"{lane}_leader_relative_speed": 1.0, f"{lane}_follower_gap": -1.0,
                       f"{lane}_follower_relative_speed": -1.0, f"{lane}_route_connectivity": 1.0})
    values.update(tl_distance=1.0, stop_distance=1.0, route_lane_continuation=1.0, crosswalk_distance=1.0,
                  speed_limit=1.0, next_speed_limit=1.0, next_speed_limit_distance=1.0, ego_speed=speed / VMAX)
    for lane, (lead_gap, lead_v, follow_gap, follow_v) in features.pop("lane_cars", {}).items():
        values.update({f"{lane}_leader_gap": lead_gap / VIS, f"{lane}_leader_relative_speed": (lead_v - speed) / VMAX,
                       f"{lane}_follower_gap": -follow_gap / VIS, f"{lane}_follower_relative_speed": (follow_v - speed) / VMAX})
    values.update(features)
    return np.array([values[name] for name in OBSERVATION_FEATURES], dtype=np.float32)


@pytest.fixture(params=[ExpertPolicy, ExpertPolicyV5], ids=["v4", "v5"])
def expert(request):
    return request.param(VMAX, VIS, 1.0, 2.0, sumo_env.TL_RANGE, sumo_env.STOP_RANGE, sumo_env.CONTINUATION_RANGE,
                         sumo_env.CROSSING_RANGE, sumo_env.LIMIT_RANGE)


def test_fast_platoon_alongside(expert):
    """R111(quiet 2057): 13m/s, 왼쪽 차로 15.6m 앞과 13m 뒤에 같은 속도의 차. SUMO는 40m 넘는 앞 간격을 요구한다.
    v4는 요청하고 그대로 달려 나란히 가다 차로 끝에 섰다. v5는 요청하지 않고 줄여 옆 차를 보낸다."""
    obs = observe(13.0, route_lane_offset=0.5, route_lane_continuation=120.0 / 300.0,
                  lane_cars={"left1": (15.6, 13.0, 13.0, 13.0)})
    accel, lane = expert(obs)
    if isinstance(expert, ExpertPolicyV5):
        assert (lane, accel < 0.0) == (0.0, True)
    else:
        assert (lane, accel > 0.0) == (1.0, True)


def test_slow_merge_both_request(expert):
    """느릴 때는 두 판 모두 옮긴다(3m/s, 뒤차 20m)."""
    obs = observe(3.0, route_lane_offset=0.5, route_lane_continuation=120.0 / 300.0,
                  lane_cars={"left1": (VIS, 3.0, 20.0, 3.0)})
    assert expert(obs)[1] == 1.0


def test_slower_follower_does_not_block(expert):
    """옆 차로 뒤차가 버스보다 느리면(버스 뒤로 들어오려는 차) 6.5m 뒤라도 두 판 모두 요청한다(baseline 1312).
    v5가 이 차를 기다리면 둘 다 차로 끝에 서서 62초를 섰다."""
    obs = observe(8.4, route_lane_offset=0.5, route_lane_continuation=84.0 / 300.0,
                  lane_cars={"left1": (VIS, 8.4, 6.5, 7.3)})
    assert expert(obs)[1] == 1.0


def test_faster_follower_passes_first(expert):
    """정류장 앞에서 옆 차로 뒤차가 14.5m 뒤에서 1.3m/s 빠르게 온다(busy 1123). v4는 요청했고 SUMO가 그 차에게
    버스를 따라 줄이라고 알려 그 차가 버스 뒤에 붙은 채 정류장을 지나쳤다. v5는 먼저 보낸다."""
    obs = observe(8.3, stop_distance=36.0 / 300.0, stop_lane_offset=-0.5, lane_cars={"right1": (VIS, 8.3, 14.5, 9.6)})
    assert expert(obs)[1] == (0.0 if isinstance(expert, ExpertPolicyV5) else -1.0)


def test_follow_within_sumo_safe_speed(expert):
    """같은 속도(11m/s)의 앞차가 30m 앞. IDM은 더 붙으려 하지만 SUMO의 버스 안전 속도는 10.5m/s라 속도를 깎는다
    (J82 좌회전 차로 개입). v5는 줄여서 그 속도 안에 머문다."""
    obs = observe(11.0, lane_cars={"current": (30.0, 11.0, VIS, 11.0)})
    assert (expert(obs)[0] < 0.0) == isinstance(expert, ExpertPolicyV5)


def test_stop_lane_opens_late(expert):
    """정류장 12m 앞에서 오른쪽 정류장 차로가 비었다. v4는 15m 안이라 정류장을 포기하고 가속했다(busy 1118).
    v5는 옮기면서 정류장에 선다."""
    obs = observe(6.0, stop_distance=12.0 / 300.0, stop_lane_offset=-0.5)
    accel, lane = expert(obs)
    assert lane == -1.0
    assert (accel < 0.0) == isinstance(expert, ExpertPolicyV5)
