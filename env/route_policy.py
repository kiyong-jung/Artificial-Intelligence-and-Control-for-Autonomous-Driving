"""학습 없이 경로를 따라가는 규칙 정책. 데이터 수집(--policy)과 scripts/validate_road.py가 쓴다.

numpy만 쓴다. 도로만 다시 만드는 사람에게 torch를 요구하지 않는다.

- route_following_action: 수업 원본 31차원만 보는 옛 규칙. 늘 같은 가속을 내고 신호와 정류장을 보지 못해
  적색 정지선 앞에서 SUMO 강제 정지(최대 80m/s²)에 기댄다. 비교 기준으로 남긴다.
- ExpertPolicy: 44차원 관측(env/data_schema.OBSERVATION_FEATURES)만 보는 전문가 규칙. BC가 같은 관측으로
  따라 할 수 있도록 내부 상태를 두지 않는다. 같은 관측이면 같은 행동이다.
"""
import numpy as np

from env.data_schema import OBSERVATION_FEATURES

_INDEX = {name: index for index, name in enumerate(OBSERVATION_FEATURES)}


def route_following_action(observation):
    """완만하게 가속하고, 현재 차선이 경로에서 끊기면 더 멀리 이어지는 옆 차선으로 옮긴다.

    관측 [21..25]는 (좌2, 좌1, 현재, 우1, 우2) 차선의 경로 기준 연결성이다.
    """
    _left2, left1, current, right1, _right2 = (float(value) for value in observation[21:26])
    lane_change = 0.0
    if current < 1.0:
        connectivity, direction = max((left1, 1.0), (right1, -1.0))
        if connectivity > current:
            lane_change = direction
    return np.array([0.25, lane_change], dtype=np.float32)


class ExpertPolicy:
    """버스 규칙 운전자. 종방향은 IDM, 정지할 곳(적색 정지선, 정류장, 사람이 있는 횡단보도, 끝나는 차로)은
    그 자리에 선 가상의 정지 차량으로 본다. 횡방향은 정류장 차로와 경로 차로를 미리 맞추고 간격이 충분할 때만 옮긴다.

    물리 값은 환경 설정에서 받는다(정규화를 되돌리는 데 필요하다).
    """

    # 버스 운전 파라미터
    comfortable_decel = 1.5   # m/s². 승차감 한계(가속 1.5)와 같다
    headway = 1.5             # s. 버스 tau와 같다
    standstill_gap = 4.0      # m. 앞차와 선 간격
    target_gap = 0.5          # m. 가상 정지점(정지선, 정류장 목표)에 붙여 세우는 간격
    jerk_step = 0.75          # m/s². 판단 한 번(0.5초)에 바꿀 수 있는 가속도(1.5m/s³)
    yield_speed = 6.0         # m/s. 비보호 녹색 앞 접근 속도
    lane_change_ahead = 200.0 # m. 경로 차로를 이 거리 앞부터 맞춘다
    stop_lane_ahead = 250.0   # m. 정류장 차로를 이 거리 앞부터 맞춘다
    gap_search_speed = 0.6    # 옮겨야 할 차로에 틈이 없으면 제한속도의 이 비율로 줄여 옆 차를 앞으로 보낸다
    gap_search_within = 150.0 # m. 남은 거리가 이보다 짧을 때만 줄인다
    stop_give_up = 15.0       # m. 이 안까지 정류장 차로에 못 들어가면 그 정류장을 지나친다(옆 차로를 막고 기다리지 않는다)
    crosswalk_stop_gap = 3.0  # m. 사람이 있는 횡단보도 앞에 설 간격
    stop_line_gap = 2.0       # m. 적색 정지선 앞에 설 간격(SUMO도 정지선 바로 앞에서 세운다)

    def __init__(self, vmax, visibility, max_accel, max_decel,
                 tl_range=100.0, stop_range=300.0, continuation_range=300.0, crossing_range=50.0, limit_range=100.0):
        self.vmax, self.visibility = float(vmax), float(visibility)
        self.max_accel, self.max_decel = float(max_accel), float(max_decel)
        self.tl_range, self.stop_range = float(tl_range), float(stop_range)
        self.continuation_range, self.crossing_range = float(continuation_range), float(crossing_range)
        self.limit_range = float(limit_range)

    def _f(self, obs, name):
        return float(obs[_INDEX[name]])

    def _idm(self, v, v0, gap, closing, standstill):
        """IDM 가속도. gap이 None이면 자유 주행 항만."""
        a, b = self.max_accel, self.comfortable_decel
        free = 1.0 - (v / max(v0, 0.1)) ** 4
        if gap is None:
            return a * free
        desired = standstill + max(0.0, v * self.headway + v * closing / (2.0 * np.sqrt(a * b)))
        return a * (free - (desired / max(gap, 0.1)) ** 2)

    def longitudinal(self, obs):
        v = self._f(obs, "ego_speed") * self.vmax
        v0 = self._f(obs, "speed_limit") * self.vmax
        # 비보호 녹색이나 끝나 가는 차로 앞에서는 천천히 다가간다
        tl_distance = self._f(obs, "tl_distance") * self.tl_range
        if self._f(obs, "tl_yield") > 0.5 and tl_distance < 40.0:
            v0 = min(v0, self.yield_speed)
        candidates = [self._idm(v, v0, None, 0.0, 0.0)]

        leader_gap = self._f(obs, "current_leader_gap")
        if leader_gap < 1.0:
            leader_speed = v + self._f(obs, "current_leader_relative_speed") * self.vmax
            candidates.append(self._idm(v, v0, leader_gap * self.visibility, v - leader_speed, self.standstill_gap))

        # 적색이면 정지선에 선다. 황색은 최대 감속(버스 제원)으로 정지선 앞에 설 수 있으면 서고, 못 서면 지나간다.
        # 편안한 감속을 기준으로 하면 지나가기로 한 뒤 적색으로 바뀌어 SUMO가 정지선에서 강제로 세웠다.
        red, yellow = self._f(obs, "tl_red") > 0.5, self._f(obs, "tl_yellow") > 0.5
        if red or (yellow and v * v / (2.0 * self.max_decel) < tl_distance - self.stop_line_gap):
            gap = max(tl_distance - self.stop_line_gap, 0.0) + self.target_gap
            candidates.append(self._idm(v, v0, gap, v, self.target_gap))

        # 앞쪽의 낮은 허용 속도(회전 차로)에 맞춰 미리 줄인다
        limit = self._f(obs, "next_speed_limit") * self.vmax
        limit_distance = self._f(obs, "next_speed_limit_distance") * self.limit_range
        if v > limit:
            needed = (v * v - limit * limit) / (2.0 * max(limit_distance - 2.0, 1.0))
            if needed > 0.4:
                candidates.append(-min(needed * 1.1, self.max_decel))

        # 정차할 정류장: 정류장 목표에 세운다. 아직 정류장 차로가 아니면 목표 앞에서 기다렸다가 틈이 나면 옮긴다.
        # 정차 중이면 남은 시간 동안 선다.
        stop_distance = self._f(obs, "stop_distance") * self.stop_range
        dwelling = self._f(obs, "stop_dwell_remaining") > 0.0
        if dwelling:
            return -self.max_decel
        in_stop_lane = abs(self._f(obs, "stop_lane_offset")) < 0.25
        if stop_distance < self.stop_range and (in_stop_lane or stop_distance > self.stop_give_up):
            candidates.append(self._idm(v, v0, stop_distance + self.target_gap, v, self.target_gap))
        # 옮겨야 할 차로에 틈이 없고 남은 거리가 짧으면 속도를 줄여 옆 차를 먼저 보낸다
        direction, remaining = self._wanted_lane(obs)
        if direction and remaining < self.gap_search_within and not self._gap_ok(obs, "left1" if direction > 0 else "right1"):
            v_slow = self.gap_search_speed * v0
            if v > v_slow:
                candidates.append(self._idm(v, v_slow, None, 0.0, 0.0))

        # 사람이 건너거나 기다리는 횡단보도 앞에 선다
        crossing_distance = self._f(obs, "crosswalk_distance") * self.crossing_range
        if self._f(obs, "crosswalk_occupied") > 0.5 and crossing_distance < self.crossing_range:
            gap = max(crossing_distance - self.crosswalk_stop_gap, 0.0) + self.target_gap
            candidates.append(self._idm(v, v0, gap, v, self.target_gap))

        # 경로를 이으려면 옮겨야 하는데 못 옮겼으면 차로 끝 앞에 선다
        continuation = self._f(obs, "route_lane_continuation") * self.continuation_range
        if abs(self._f(obs, "route_lane_offset")) > 0.25 and continuation < 80.0:
            candidates.append(self._idm(v, v0, max(continuation - 8.0, 0.0) + self.target_gap, v, self.target_gap))

        target = min(candidates)
        # 가속도 변화 제한(승차감). 급한 감속이 필요하면 제한을 두 배로 푼다.
        current = self._f(obs, "ego_acceleration") * 2.0
        step = self.jerk_step * (2.0 if target < current - self.jerk_step and target < -self.comfortable_decel else 1.0)
        return float(np.clip(target, current - step, current + step))

    def _gap_ok(self, obs, side):
        """side='left1' 또는 'right1' 차로로 옮길 앞뒤 간격이 충분한지."""
        v = self._f(obs, "ego_speed") * self.vmax
        front = self._f(obs, f"{side}_leader_gap")
        rear = self._f(obs, f"{side}_follower_gap")
        if self._f(obs, f"{side}_route_connectivity") < 0.0:
            return False
        front_m = front * self.visibility
        rear_m = -rear * self.visibility
        closing = self._f(obs, f"{side}_follower_relative_speed") * self.vmax   # +면 뒤차가 더 빠르다
        front_ok = front >= 1.0 or front_m > 6.0 + 0.6 * v
        rear_ok = rear <= -1.0 or rear_m > 6.0 + 2.0 * max(closing, 0.0)
        return front_ok and rear_ok

    def _wanted_lane(self, obs):
        """옮겨야 할 방향(+1 왼쪽, -1 오른쪽, 0 없음)과 그 일을 끝내야 하는 곳까지 남은 거리(m)."""
        stop_distance = self._f(obs, "stop_distance") * self.stop_range
        stop_offset = self._f(obs, "stop_lane_offset")
        route_offset = self._f(obs, "route_lane_offset")
        continuation = self._f(obs, "route_lane_continuation") * self.continuation_range
        # 경로를 이으려는 차선변경이 정류장 차로보다 먼저다. 회전 차로를 놓치면 경로를 벗어난다.
        if abs(route_offset) > 0.25 and continuation < self.lane_change_ahead:
            return (1 if route_offset > 0 else -1), continuation
        if stop_distance < self.stop_lane_ahead and abs(stop_offset) > 0.25:
            return (1 if stop_offset > 0 else -1), stop_distance
        return 0, float("inf")

    def lateral(self, obs):
        direction, _remaining = self._wanted_lane(obs)
        if direction and self._f(obs, "stop_dwell_remaining") <= 0.0:
            side = "left1" if direction > 0 else "right1"
            if self._gap_ok(obs, side):
                return float(direction)
        return 0.0

    def __call__(self, obs):
        accel = self.longitudinal(obs)
        raw = accel / self.max_accel if accel >= 0.0 else accel / self.max_decel
        return np.array([np.clip(raw, -1.0, 1.0), self.lateral(obs)], dtype=np.float32)


def make_expert(env):
    """환경(SumoHighwayEnv)의 정규화 값으로 전문가 정책을 만든다."""
    from env import sumo_env
    return ExpertPolicy(env.vmax, env.visibility, env.max_accel, env.max_decel,
                        sumo_env.TL_RANGE, sumo_env.STOP_RANGE, sumo_env.CONTINUATION_RANGE, sumo_env.CROSSING_RANGE,
                        sumo_env.LIMIT_RANGE)
