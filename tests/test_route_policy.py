"""전문가 규칙은 관측만 보고 차로를 고르고(차로별 연결성), 틈이 있을 때만 옮기고, 설 수 있는 속도로 따라간다."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from env import sumo_env  # noqa: E402
from env.data_schema import OBSERVATION_FEATURES  # noqa: E402
from env.route_policy import ExpertPolicy, ExpertPolicyV5, ExpertPolicyV6, ExpertPolicyV7  # noqa: E402

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


@pytest.fixture
def v6():
    return make(ExpertPolicyV6)


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


# ---- v6: 신호 앞에서 따라 배우기 쉽게 선다 ----

def red(speed, line, **features):
    return observe(speed, tl_distance=line / 100.0, tl_red=1.0, **features)


def test_v6_does_not_creep_once_stopped_at_a_red_light(v5, v6):
    """적색에 정지선 6m 앞에 섰다. v5는 정지선 2m 앞까지 다시 다가가고, v6은 최대 제동으로 그대로 선다."""
    assert v5(red(0.0, 6.0))[0] > 0.0
    assert v6(red(0.0, 6.0))[0] == -1.0
    assert v6(red(0.0, 11.0))[0] == -1.0


def test_v6_leaves_when_the_light_turns_green(v6):
    assert v6(observe(0.0, tl_distance=6.0 / 100.0))[0] > 0.0


def test_v6_does_not_speed_up_close_to_a_red_line(v6):
    """정지선 10m 앞을 1m/s로 굴러간다. 정지점(정지선 4m 앞)까지 가속하지 않는다."""
    assert v6(red(1.0, 10.0))[0] <= 0.0


def test_v6_still_drives_up_to_a_far_red_light(v6):
    assert v6(red(2.0, 80.0))[0] > 0.0


def test_v6_brakes_with_the_deceleration_the_stop_point_needs(v6):
    """10m/s, 정지점까지 50m면 1m/s²가 필요하다(가감속 raw -0.5). 25m면 2m/s²다(raw -1)."""
    assert v6(red(10.0, 54.0, ego_acceleration=-1.0 / 2.0))[0] == pytest.approx(-0.5, abs=0.01)
    assert v6(red(10.0, 29.0, ego_acceleration=-2.0 / 2.0))[0] == pytest.approx(-1.0, abs=0.01)


def test_v6_does_not_speed_up_towards_a_yellow_light_it_will_stop_for(v5, v6):
    """황색을 50m 앞에서 6m/s로 봤다. 설 수 있으므로 선다. v5는 정지점까지 가속했다가 줄이고, v6은 가속하지 않는다."""
    obs = observe(6.0, tl_distance=50.0 / 100.0, tl_yellow=1.0)
    assert v5(obs)[0] > 0.0
    assert v6(obs)[0] <= 0.0


@pytest.mark.parametrize("speed, line", [(13.0, 100.0), (8.0, 40.0), (3.0, 20.0), (0.5, 9.0)])
def test_v6_stops_short_of_the_red_line_and_stays(v6, speed, line):
    """v6의 행동만으로 버스를 굴린다(판단 0.5초, 가속 1m/s², 감속 2m/s²). 정지선 앞에 서고, 60초 동안 다시 움직이지 않는다."""
    acceleration, standing_at = 0.0, None
    for _ in range(240):
        raw = float(v6(red(speed, line, ego_acceleration=acceleration / 2.0))[0])
        acceleration = raw * (1.0 if raw >= 0.0 else 2.0)
        new_speed = max(speed + acceleration * 0.5, 0.0)
        line -= 0.5 * (speed + new_speed) * 0.5
        speed = new_speed
        if speed == 0.0:
            acceleration = 0.0
            standing_at = line if standing_at is None else standing_at
            assert line == standing_at
    assert standing_at is not None and 1.0 < standing_at < 13.0


def test_v6_goes_through_a_yellow_light_at_once_when_it_cannot_stop(v5, v6):
    """황색에 7.5m/s, 정지선 10m. 설 수 없으니 지나간다. 줄이던 중이었어도(-1.2m/s²) v6은 바로 가속하고,
    v5는 가속도 변화 제한 때문에 한동안 계속 줄인다."""
    obs = observe(7.5, tl_distance=10.0 / 100.0, tl_yellow=1.0, ego_acceleration=-1.2 / 2.0)
    assert v5(obs)[0] < 0.0
    assert v6(obs)[0] > 0.5


@pytest.mark.parametrize("line", [15.0, 20.0, 30.0, 45.0, 60.0, 80.0])
def test_v6_approach_speed_leaves_no_yellow_dilemma(v6, line):
    """녹색 신호에 v6이 허용하는 가장 빠른 속도로 다가가다 황색을 본다. 어느 거리에서든 여유 있는 감속(1.5m/s²)으로
    서거나, 황색 2.5초 안에 정지선을 1m 넘게 지나간다."""
    speed = min(v6._signal_approach_speed(line), VMAX)
    can_stop = speed * speed / (2.0 * (line - 0.5)) <= 1.5 + 1e-6
    can_clear = speed * 2.5 >= line + 1.0 - 1e-6
    assert can_stop or can_clear


def test_v6_slows_at_once_when_above_the_approach_speed(v5, v6):
    """녹색 신호 22m 앞에서 상한(v6 7.4m/s)보다 1.2m/s 빠르다. v6은 넘은 만큼(1.2m/s²) 줄이고, v5는 거의 줄이지 않는다."""
    obs = observe(8.6, tl_distance=22.0 / 100.0, ego_acceleration=-0.7 / 2.0)
    assert v6(obs)[0] < -0.5
    assert v5(obs)[0] > -0.2


def test_v6_does_not_outdrive_what_it_can_see(v5, v6):
    """빈 도로를 13m/s로 달린다. 보이는 거리 50m 끝에 선 차가 나타나면 못 서는 속도다. v6은 줄이고 v5는 더 낸다.
    11m/s면 설 수 있으므로 v6도 가속한다."""
    assert v5(observe(13.0))[0] > 0.0
    assert v6(observe(13.0))[0] < 0.0
    assert v6(observe(11.0))[0] > 0.0


def test_v6_can_stop_for_a_queue_that_comes_into_view(v6):
    """v6이 빈 도로에서 스스로 고른 속도로 달리다 50m 앞에 선 차를 처음 본다. 그 차 뒤에 선다."""
    speed, acceleration = 0.0, 0.0
    for _ in range(120):
        raw = float(v6(observe(speed, ego_acceleration=acceleration / 2.0))[0])
        acceleration = raw * (1.0 if raw >= 0.0 else 2.0)
        speed = max(speed + acceleration * 0.5, 0.0)
    assert speed > 11.0
    gap = 50.0
    for _ in range(60):
        raw = float(v6(observe(speed, ego_acceleration=acceleration / 2.0, lane_cars={"current": (gap, 0.0, 50.0, 0.0)}))[0])
        acceleration = raw * (1.0 if raw >= 0.0 else 2.0)
        new_speed = max(speed + acceleration * 0.5, 0.0)
        gap -= 0.5 * (speed + new_speed) * 0.5
        speed = new_speed
    assert speed == 0.0 and gap > 0.5


# ---- v7: 경로가 이어지는 차로가 보이는 범위 밖에 있을 때 ----

def test_v7_heads_for_a_lane_beyond_view_when_the_other_side_is_the_kerb(v6):
    """5차로 R54의 2번 차로. J1 좌회전 차로(5번)는 세 칸 왼쪽이라 보이지 않고, 보이는 네 차로는 모두 150m 뒤 끝나며,
    오른쪽은 1번 차로 다음이 길 끝이다. v6은 머물러 차로 끝에 서고, v7은 왼쪽으로 옮긴다. 좌우를 바꿔도 같다."""
    v7 = make(ExpertPolicyV7)
    obs = observe(8.0, reach={"current": 150.0, "left1": 150.0, "left2": 150.0, "right1": 150.0, "right2": None})
    assert v6(obs)[1] == 0.0 and v7(obs)[1] == 1.0
    assert v7(observe(8.0, reach={"current": 150.0, "left1": None, "right1": 150.0, "right2": 150.0}))[1] == -1.0


def test_v7_stays_when_the_lane_could_be_on_either_side():
    obs = observe(8.0, reach={"current": 150.0, "left1": 150.0, "left2": 150.0, "right1": 150.0, "right2": 150.0})
    assert make(ExpertPolicyV7)(obs)[1] == 0.0


@pytest.mark.parametrize("reach", [{"current": 120.0}, {"current": 120.0, "left1": 120.0, "left2": 400.0},
                                   {"current": 120.0, "left1": 124.0}, {"current": 120.0, "left1": None, "right1": 400.0}])
def test_v7_drives_like_v6_when_the_continuing_lane_is_in_view_or_there_is_none(v6, reach):
    obs = observe(8.0, reach=reach)
    assert np.array_equal(make(ExpertPolicyV7)(obs), v6(obs))


# ---- 신호만 바꾼 관측으로 학습 데이터 늘리기 ----

def test_signal_flips_ask_the_expert_what_if_the_light_changed_now(v6):
    """녹색 20m 앞 7.8m/s, 신호가 안 보이는 줄, 이미 적색인 줄. 첫 줄만 신호를 바꾼 사본이 생기고, 신호 값 말고는 그대로다.
    정답은 그 관측에서 v6이 내는 행동이다: 황색이든 적색이든 세게 줄인다."""
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "research", "av_bc_rl"))
    from augment_signals import signal_flips
    green = observe(7.8, tl_distance=20.0 / 100.0, tl_yield=1.0)
    data = np.stack([green, observe(7.8), red(7.8, 20.0)])
    for seed in range(4):
        rows, flipped, actions = signal_flips(data, v6, np.random.default_rng(seed))
        assert rows.tolist() == [0] and flipped.shape == (1, len(OBSERVATION_FEATURES))
        lights = {name: flipped[0][OBSERVATION_FEATURES.index(name)] for name in ("tl_red", "tl_yellow", "tl_yield")}
        assert lights["tl_red"] + lights["tl_yellow"] == 1.0 and lights["tl_yield"] == 0.0
        changed = {OBSERVATION_FEATURES[i] for i in np.flatnonzero(flipped[0] != green)}
        assert changed <= {"tl_red", "tl_yellow", "tl_yield"}
        assert np.array_equal(actions[0], v6(flipped[0])) and actions[0][0] < -0.7
    assert v6(observe(7.8, tl_distance=20.0 / 100.0))[0] > -0.5
