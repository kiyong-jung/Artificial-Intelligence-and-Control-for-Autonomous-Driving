"""전문가 규칙은 관측만 보고 차로를 고르고(차로별 연결성), 틈이 있을 때만 옮기고, 설 수 있는 속도로 따라간다."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from env import sumo_env  # noqa: E402
from env.data_schema import OBSERVATION_FEATURES  # noqa: E402
from env.route_policy import ExpertPolicy, ExpertPolicyV5  # noqa: E402

VMAX, VIS, REACH = 13.89, 50.0, 200.0
LANES = ("left2", "left1", "current", "right1", "right2")


def observe(speed, **features):
    """빈 도로의 관측에 features(m, m/s 단위)를 덮어쓴다. 기본은 길가 차로(오른쪽 차로 없음)이고 왼쪽에 차로 하나가 있다.
    lane_cars={차로: (앞차 간격, 앞차 속도, 뒤차 간격, 뒤차 속도)}, reach={차로: 경로로 이어지는 거리 m, 없는 차로는 None}."""
    values = {name: 0.0 for name in OBSERVATION_FEATURES}
    for lane in LANES:
        values.update({f"{lane}_leader_gap": 1.0, f"{lane}_leader_relative_speed": 1.0, f"{lane}_follower_gap": -1.0,
                       f"{lane}_follower_relative_speed": -1.0, f"{lane}_route_connectivity": -1.0, f"{lane}_density": 1.0})
    values.update(current_route_connectivity=1.0, left1_route_connectivity=1.0, current_density=0.0, left1_density=0.0,
                  tl_distance=1.0, stop_distance=1.0, crosswalk_distance=1.0,
                  speed_limit=1.0, next_speed_limit=1.0, next_speed_limit_distance=1.0, ego_speed=speed / VMAX)
    for lane, (lead_gap, lead_v, follow_gap, follow_v) in features.pop("lane_cars", {}).items():
        values.update({f"{lane}_leader_gap": lead_gap / VIS, f"{lane}_leader_relative_speed": (lead_v - speed) / VMAX,
                       f"{lane}_follower_gap": -follow_gap / VIS, f"{lane}_follower_relative_speed": (follow_v - speed) / VMAX})
    for lane, metres in features.pop("reach", {}).items():
        values[f"{lane}_route_connectivity"] = -1.0 if metres is None else min(metres / REACH, 1.0)
    values.update(features)
    return np.array([values[name] for name in OBSERVATION_FEATURES], dtype=np.float32)


def make(policy):
    return policy(VMAX, VIS, 1.0, 2.0, sumo_env.TL_RANGE, sumo_env.STOP_RANGE, REACH, sumo_env.CROSSING_RANGE,
                  sumo_env.LIMIT_RANGE)


@pytest.fixture(params=[ExpertPolicy, ExpertPolicyV5], ids=["v4", "v5"])
def expert(request):
    return make(request.param)


@pytest.fixture
def v5():
    return make(ExpertPolicyV5)


# ---- 차로 고르기: 차로별 연결성만 본다 ----

def test_moves_to_the_lane_that_continues_when_its_own_lane_ends(expert):
    """지금 차로는 120m 뒤 경로에서 끊기고 왼쪽 차로는 이어진다(좌회전 전용 차로 앞). 왼쪽으로 옮긴다."""
    assert expert(observe(8.0, reach={"current": 120.0}))[1] == 1.0


def test_moves_right_when_the_right_lane_is_the_one_that_continues(expert):
    assert expert(observe(8.0, reach={"current": 120.0, "left1": None, "right1": 400.0}))[1] == -1.0


def test_goes_through_a_lane_that_also_ends_to_reach_one_that_continues(expert):
    """왼쪽 첫 차로도 같이 끊기지만 그 너머 차로가 이어진다. 왼쪽으로 한 칸씩 간다."""
    assert expert(observe(8.0, reach={"current": 120.0, "left1": 120.0, "left2": 400.0}))[1] == 1.0


def test_stays_when_no_lane_continues_farther(expert):
    """모든 차로가 같은 곳에서 끊기면(교차로 안 길이 차이 몇 m) 옮기지 않는다."""
    assert expert(observe(8.0, reach={"current": 120.0, "left1": 124.0}))[1] == 0.0


def test_stays_in_a_lane_that_continues(expert):
    assert expert(observe(8.0, reach={"left1": 400.0, "right1": 400.0}))[1] == 0.0


def test_stops_before_the_end_of_a_lane_it_could_not_leave(expert):
    """끊기는 차로의 끝 12m 앞에서 4m/s. 옆 차로가 보이지 않아도(교차로 안) 끝 앞에 서려고 줄인다."""
    assert expert(observe(4.0, reach={"current": 12.0, "left1": None}))[0] < 0.0


def test_moves_to_the_kerb_lane_for_a_stop(expert):
    """정류장 100m 앞, 오른쪽에 정류장까지 이어지는 차로가 있다. 정류장은 길가에 있으므로 오른쪽으로 간다."""
    assert expert(observe(8.0, stop_distance=100.0 / 300.0, reach={"right1": 400.0}))[1] == -1.0


def test_ignores_a_right_lane_that_ends_before_the_stop(expert):
    """오른쪽 차로가 정류장 전에 끊기면(우회전 전용 차로) 지금 차로가 길가 차로다. 옮기지 않고 정류장에 선다."""
    obs = observe(8.0, stop_distance=30.0 / 300.0, reach={"right1": 10.0})
    accel, lane = expert(obs)
    assert lane == 0.0 and accel < 0.0


def test_a_far_stop_does_not_pull_it_right(expert):
    assert expert(observe(8.0, stop_distance=250.0 / 300.0, reach={"right1": 400.0}))[1] == 0.0


def test_serves_the_stop_before_leaving_a_lane_that_ends_after_it(expert):
    """정류장(30m)이 지금 차로의 끝(150m)보다 가깝다. 정류장에 먼저 서고, 왼쪽으로 가려 하지 않는다."""
    accel, lane = expert(observe(8.0, stop_distance=30.0 / 300.0, reach={"current": 150.0}))
    assert lane == 0.0 and accel < 0.0


def test_leaves_the_ending_lane_before_heading_for_a_stop_beyond_it(expert):
    """지금 차로의 끝(120m)이 정류장(180m)보다 가깝다. 경로를 잇는 차선변경이 먼저다."""
    assert expert(observe(8.0, stop_distance=180.0 / 300.0, reach={"current": 120.0}))[1] == 1.0


# ---- 틈과 속도 ----

def test_fast_platoon_alongside(expert):
    """13m/s, 왼쪽 차로 15.6m 앞과 13m 뒤에 같은 속도의 차. 버스(감속 2m/s²)는 앞차가 서면 그 간격으로 못 선다.
    v4는 요청하고 그대로 달린다. v5는 요청하지 않고 줄여 옆 차를 보낸다."""
    obs = observe(13.0, reach={"current": 120.0}, lane_cars={"left1": (15.6, 13.0, 13.0, 13.0)})
    accel, lane = expert(obs)
    if isinstance(expert, ExpertPolicyV5):
        assert (lane, accel < 0.0) == (0.0, True)
    else:
        assert (lane, accel > 0.0) == (1.0, True)


def test_slow_merge_both_request(expert):
    """느릴 때는 두 판 모두 옮긴다(3m/s, 뒤차 20m)."""
    obs = observe(3.0, reach={"current": 120.0}, lane_cars={"left1": (VIS, 3.0, 20.0, 3.0)})
    assert expert(obs)[1] == 1.0


def test_slower_follower_does_not_block(expert):
    """옆 차로 뒤차가 버스보다 느리면(버스 뒤로 들어오려는 차) 6.5m 뒤라도 두 판 모두 요청한다."""
    obs = observe(8.4, reach={"current": 84.0}, lane_cars={"left1": (VIS, 8.4, 6.5, 7.3)})
    assert expert(obs)[1] == 1.0


def test_faster_follower_passes_first(expert):
    """정류장 앞에서 오른쪽 차로 뒤차가 14.5m 뒤에서 1.3m/s 빠르게 온다. v5는 먼저 보낸다."""
    obs = observe(8.3, stop_distance=36.0 / 300.0, reach={"right1": 400.0}, lane_cars={"right1": (VIS, 8.3, 14.5, 9.6)})
    assert expert(obs)[1] == (0.0 if isinstance(expert, ExpertPolicyV5) else -1.0)


def test_follows_at_a_speed_it_can_stop_from(expert):
    """같은 속도(11m/s)의 앞차가 30m 앞. IDM은 더 붙으려 한다. 버스가 앞차 급정지 뒤에 설 수 있는 속도는 10.5m/s라
    v5는 줄인다. 안전 보조가 꺼져 있어 이 판단이 추돌을 막는다."""
    obs = observe(11.0, lane_cars={"current": (30.0, 11.0, VIS, 11.0)})
    assert (expert(obs)[0] < 0.0) == isinstance(expert, ExpertPolicyV5)


def test_brakes_for_a_stopped_car_ahead(v5):
    obs = observe(8.0, lane_cars={"current": (20.0, 0.0, VIS, 8.0)})
    assert v5(obs)[0] < -0.3


def test_stop_lane_opens_late(expert):
    """정류장 12m 앞에서 오른쪽 길가 차로가 비었다. v4는 15m 안이라 정류장을 포기하고 가속한다.
    v5는 옮기면서 정류장에 선다."""
    obs = observe(6.0, stop_distance=12.0 / 300.0, reach={"right1": 400.0})
    accel, lane = expert(obs)
    assert lane == -1.0
    assert (accel < 0.0) == isinstance(expert, ExpertPolicyV5)


def test_never_requests_into_a_queue_at_the_lane_end(expert):
    """차로 끝 11m 앞에서 0.8m/s로 기어가고 왼쪽 차로 뒤차가 3.5m 뒤에서 온다. 요청은 옆 차와 무관하게 실행되므로
    (laneChangeMode 0) 틈이 없으면 서서 기다린다."""
    obs = observe(0.8, reach={"current": 11.0}, lane_cars={"left1": (VIS, 0.8, 3.5, 1.9)})
    assert expert(obs)[1] == 0.0


def test_moving_bus_waits_for_a_gap(v5):
    obs = observe(6.0, reach={"current": 30.0}, lane_cars={"left1": (VIS, 6.0, 3.5, 7.1)})
    assert v5(obs)[1] == 0.0


def test_leaves_room_for_a_car_to_merge_in_front(expert):
    """못 옮긴 채 차로 끝 9m 앞에 섰다. 옆 차로 끝의 승용차(5m)가 버스 앞으로 들어오려면 버스 최소 간격 3m까지 8.4m가
    필요하다. v4는 8m까지 더 다가가 그 차를 막고 v5는 더 다가가지 않는다."""
    obs = observe(0.0, reach={"current": 9.0}, lane_cars={"left1": (0.0, 0.0, 0.0, 0.0)})
    assert (expert(obs)[0] > 0.0) == (not isinstance(expert, ExpertPolicyV5))


def test_drops_behind_a_side_car_that_keeps_pace(v5):
    """버스는 왼쪽 차로로, 바로 옆 왼쪽 차로의 승용차는 버스 차로로 가야 한다. 옆 차를 먼저 보내려면 버스가 그 차보다
    느려야 한다."""
    obs = observe(8.0, reach={"current": 90.0}, lane_cars={"left1": (0.0, 8.0, VIS, 8.0)})
    accel, lane = v5(obs)
    assert (lane, accel < 0.0) == (0.0, True)


# ---- 신호와 횡단보도: 안전 보조 없이 스스로 선다 ----

def test_brakes_for_a_red_light(v5):
    assert v5(observe(10.0, tl_distance=40.0 / 100.0, tl_red=1.0))[0] < 0.0


def test_passes_a_green_light(v5):
    assert v5(observe(8.0, tl_distance=15.0 / 100.0))[0] > 0.0


def test_yellow_stops_only_when_it_can(v5):
    """황색에서 최대 감속으로 정지선 앞에 설 수 있으면(10m/s, 40m) 서고, 못 서면(10m/s, 15m) 지나간다."""
    assert v5(observe(10.0, tl_distance=40.0 / 100.0, tl_yellow=1.0))[0] < 0.0
    assert v5(observe(10.0, tl_distance=15.0 / 100.0, tl_yellow=1.0))[0] >= 0.0


def test_yellow_decision_does_not_flip_while_braking(v5):
    """황색에 7.8m/s, 정지선 16m. 최대 감속으로만 설 수 있다. 이미 세게 줄이는 중이면 계속 서고(선다고 정한 다음 판단),
    줄이는 중이 아니고 황색 3초 안에 지나갈 수 있으면 지나간다."""
    braking = observe(7.8, tl_distance=16.0 / 100.0, tl_yellow=1.0, ego_acceleration=-1.9 / 2.0)
    cruising = observe(7.8, tl_distance=16.0 / 100.0, tl_yellow=1.0, ego_acceleration=0.0)
    assert v5(braking)[0] < -0.8
    assert v5(cruising)[0] >= 0.0


def test_yellow_counts_on_the_braking_it_is_already_doing(v5):
    """황색에 8.6m/s, 정지선 21.8m, 회전 앞이라 0.8m/s²로 줄이는 중이다. 같은 속도로는 2.5초에 21.5m를 가지만 줄이는
    중이라 19m밖에 못 간다. 지나가지 않고 선다(train quiet 2021에서는 지나가다 적색에 정지선을 넘었다)."""
    obs = observe(8.6, tl_distance=21.8 / 100.0, tl_yellow=1.0, ego_acceleration=-0.8 / 2.0)
    assert v5(obs)[0] < -0.7


def test_yellow_stops_when_it_cannot_clear_in_time(v5):
    """황색에 13m/s, 정지선 45m. 여유 있는 감속으로는 못 서지만 3초 안에 지나갈 수도 없다. 최대 감속으로 선다."""
    assert v5(observe(13.0, tl_distance=45.0 / 100.0, tl_yellow=1.0))[0] < 0.0


def test_slows_towards_a_green_light_it_could_neither_stop_for_nor_clear(v5):
    """녹색 신호 50m 앞을 13m/s로 달리면, 지금 황색이 켜질 때 서지도 못하고 황색 안에 지나가지도 못한다. 미리 줄인다.
    20m 앞에서 8m/s면 어느 쪽이든 할 수 있으므로 줄이지 않는다."""
    assert v5(observe(13.0, tl_distance=50.0 / 100.0))[0] < 0.0
    assert v5(observe(8.0, tl_distance=20.0 / 100.0))[0] > 0.0


def test_brakes_for_a_person_on_the_crosswalk(v5):
    assert v5(observe(8.0, crosswalk_distance=30.0 / 100.0, crosswalk_occupied=1.0))[0] < 0.0
    assert v5(observe(8.0, crosswalk_distance=30.0 / 100.0))[0] > 0.0


def test_same_observation_same_action(v5):
    obs = observe(9.0, stop_distance=80.0 / 300.0, reach={"right1": 400.0}, lane_cars={"current": (25.0, 7.0, 30.0, 9.0)})
    assert np.array_equal(v5(obs), v5(obs.copy()))
