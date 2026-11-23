"""학습 없이 경로를 따라가는 규칙 정책. 데이터 수집(--policy)과 scripts/validate_road.py가 쓴다.

numpy만 쓴다. 도로만 다시 만드는 사람에게 torch를 요구하지 않는다.

- route_following_action: 수업 원본 31차원만 보는 옛 규칙. 늘 같은 가속을 내고 신호와 정류장을 보지 못해
  적색 정지선 앞에서 SUMO 강제 정지(최대 80m/s²)에 기댄다. 비교 기준으로 남긴다.
- ExpertPolicy: 46차원 관측(env/data_schema.OBSERVATION_FEATURES)만 보는 전문가 규칙(v4). BC가 같은 관측으로
  따라 할 수 있도록 내부 상태를 두지 않는다. 같은 관측이면 같은 행동이다. BC 데이터와 Residual RL이 이 판으로 학습했다.
- ExpertPolicyV5: v4의 차선변경 판단과 앞차 추종을 SUMO의 안전 간격에 맞춘 판(--policy expert-v5).
"""
import numpy as np

from env.data_schema import OBSERVATION_FEATURES
from env.mdp_config import ACTION

_INDEX = {name: index for index, name in enumerate(OBSERVATION_FEATURES)}


def _side(direction):
    """차선변경 방향(+1 왼쪽, -1 오른쪽)의 관측 차로 이름."""
    return "left1" if direction > 0 else "right1"


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
    lane_end_room = 8.0       # m. 못 옮긴 채 끝나는 차로에서 차로 끝 앞에 남기는 거리

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
        candidates += self._safe_speed_candidates(obs, v)

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

        # 정차 중이면 남은 시간 동안 선다
        if self._f(obs, "stop_dwell_remaining") > 0.0:
            return -self.max_decel
        candidates += self._stop_candidates(obs, v, v0)
        candidates += self._gap_search_candidates(obs, v, v0)

        # 사람이 건너거나 기다리는 횡단보도 앞에 선다
        crossing_distance = self._f(obs, "crosswalk_distance") * self.crossing_range
        if self._f(obs, "crosswalk_occupied") > 0.5 and crossing_distance < self.crossing_range:
            gap = max(crossing_distance - self.crosswalk_stop_gap, 0.0) + self.target_gap
            candidates.append(self._idm(v, v0, gap, v, self.target_gap))

        candidates += self._lane_end_candidates(obs, v, v0)

        target = min(candidates)
        # 가속도 변화 제한(승차감). 급한 감속이 필요하면 제한을 두 배로 푼다.
        current = self._f(obs, "ego_acceleration") * 2.0
        step = self.jerk_step * (2.0 if target < current - self.jerk_step and target < -self.comfortable_decel else 1.0)
        return float(np.clip(target, current - step, current + step))

    def _safe_speed_candidates(self, obs, v):
        """v4는 앞차 추종을 IDM 하나로만 정한다."""
        return []

    def _stop_candidates(self, obs, v, v0):
        """정차할 정류장: 정류장 목표에 세운다. 아직 정류장 차로가 아니면 목표 앞에서 기다렸다가 틈이 나면 옮긴다."""
        stop_distance = self._f(obs, "stop_distance") * self.stop_range
        in_stop_lane = abs(self._f(obs, "stop_lane_offset")) < 0.25
        if stop_distance < self.stop_range and (in_stop_lane or stop_distance > self.stop_give_up):
            return [self._idm(v, v0, stop_distance + self.target_gap, v, self.target_gap)]
        return []

    def _gap_search_candidates(self, obs, v, v0):
        """옮겨야 할 차로에 틈이 없고 남은 거리가 짧으면 속도를 줄여 옆 차를 먼저 보낸다."""
        direction, remaining = self._wanted_lane(obs)
        if direction and remaining < self.gap_search_within and not self._gap_ok(obs, _side(direction)):
            v_slow = self.gap_search_speed * v0
            if v > v_slow:
                return [self._idm(v, v_slow, None, 0.0, 0.0)]
        return []

    def _lane_end_candidates(self, obs, v, v0):
        """경로를 이으려면 옮겨야 하는데 못 옮겼으면 차로 끝 앞에 선다."""
        continuation = self._f(obs, "route_lane_continuation") * self.continuation_range
        if abs(self._f(obs, "route_lane_offset")) > 0.25 and continuation < 80.0:
            return [self._idm(v, v0, max(continuation - self.lane_end_room, 0.0) + self.target_gap, v, self.target_gap)]
        return []

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
            if self._gap_ok(obs, _side(direction)):
                return float(direction)
        return 0.0

    def __call__(self, obs):
        accel = self.longitudinal(obs)
        raw = accel / self.max_accel if accel >= 0.0 else accel / self.max_decel
        return np.array([np.clip(raw, -1.0, 1.0), self.lateral(obs)], dtype=np.float32)


class ExpertPolicyV5(ExpertPolicy):
    """v4의 차선변경 판단과 앞차 추종을 SUMO가 실제로 지키는 안전 간격에 맞춘 판. 같은 관측만 보고 내부 상태가 없다.

    v4의 간격 기준(_gap_ok, 6m + 0.6v)은 SUMO가 실제로 옮겨 주는 안전 간격과 어긋났다. ego의 차선변경 모드(1536)에서
    SUMO는 TraCI 요청도 남의 안전 간격을 지킬 때만 실행하고, 버스 제원(tau 1.5초, 감속 2m/s²)으로 앞 간격
    v·tau + v²/4 - 앞차 제동거리를 요구한다. 13m/s에서는 같은 속도의 앞차와 40m가 넘게 떨어져야 옮긴다.
    v4는 15m 틈에도 요청하고 속도를 줄이지 않아 옆 차와 나란히 달리다 차로 끝에 섰다. v5는 같은
    공식(MSCFModel::getSecureGap)으로 판단해, 옮길 수 없으면 줄여서 옆 차를 보내고, 옮길 수 있을 때만 요청한다.
    틈이 없는데 요청을 걸어 두면 SUMO가 옆 차로 뒤차에게 버스를 따라 줄이라고 알려(informFollower) 그 차가 버스
    뒤에 붙은 채 끝내 앞질러 가지 않았다.
    """

    # 환경의 차선변경 안전 게이트(env/mdp_config.ACTION["lane_change_safety"]). 이것을 넘어야 요청이 SUMO에 간다.
    gate = ACTION["lane_change_safety"]
    gate_margin = 0.5        # m
    # SUMO 안전 간격을 어림할 배경차 제원. env/road_config의 추종모델 중 가장 긴 tau와 그 감속이다.
    car_decel = 4.5          # m/s²
    car_headway = 1.8        # s
    merge_floor = 4.0        # m. 옮기는 중이어도 정류장까지 이보다 가까우면 지나친다(옆 차로에 선 채 기다리지 않는다)
    creep_speed = 3.0        # m/s. 옮길 곳(정류장, 차로 끝)이 가까운데 옮길 수 없으면 이 속도로 기어간다
    creep_within = 40.0      # m
    gap_search_margin = 1.5  # m/s. 틈을 찾을 때 옆 차로 앞차보다 이만큼 느리게 간다
    safe_follow_horizon = 1.0  # s. SUMO 안전 속도까지 남은 차이를 이 시간에 걸쳐 맞춘다
    safe_follow_margin = 1.0   # m
    # 옆 차로 끝에 선 차가 버스 앞으로 들어올 자리. 승용차 5m + 버스 최소 간격 3m + 그 차가 차로 끝 앞에 남긴 0.4m는 8.4m다.
    # 8m에서는 그 차가 0.1m 차이로 못 들어와 옆 차로 끝을 막았고, 그 차로로 가야 하는 버스도 같이 섰다(final busy 9600).
    lane_end_room = 10.0       # m

    def _neighbors(self, obs, side):
        """옆 차로의 (앞차 간격 m, 앞차 속도), (뒤차 간격 m, 뒤차 속도). 차가 없으면 None."""
        v = self._f(obs, "ego_speed") * self.vmax
        front, rear = self._f(obs, f"{side}_leader_gap"), self._f(obs, f"{side}_follower_gap")
        lead = (front * self.visibility, v + self._f(obs, f"{side}_leader_relative_speed") * self.vmax) if front < 1.0 else None
        follow = (-rear * self.visibility, v + self._f(obs, f"{side}_follower_relative_speed") * self.vmax) if rear > -1.0 else None
        return lead, follow

    def _gate_ok(self, obs, side):
        """환경의 안전 게이트(env/sumo_env._lane_change_is_safe)가 요청을 받아 줄 틈인지."""
        if self._f(obs, f"{side}_route_connectivity") < 0.0:
            return False
        v, g, margin = self._f(obs, "ego_speed") * self.vmax, self.gate, self.gate_margin
        lead, follow = self._neighbors(obs, side)
        if lead is not None:
            gap, lv = lead
            if gap <= g["min_front_gap"] + g["front_tau"] * v + margin or gap <= max(v - lv, 0.0) * g["approach_horizon"] + margin:
                return False
        if follow is not None:
            gap, fv = follow
            if gap <= g["min_rear_gap"] + g["rear_tau"] * fv + margin or gap <= max(fv - v, 0.0) * g["approach_horizon"] + margin:
                return False
        return True

    def _merge_ok(self, obs, side):
        """SUMO가 협조 없이 옮겨 줄 만큼 앞뒤 안전 간격이 있는지. 버스는 감속이 약해(2m/s²) 빠를수록 앞 간격이 길어야 한다."""
        if not self._gate_ok(obs, side):
            return False
        v, a = self._f(obs, "ego_speed") * self.vmax, self._f(obs, "ego_acceleration") * 2.0
        lead, follow = self._neighbors(obs, side)
        if lead is not None:
            gap, lv = lead
            leader_brake = lv * lv / (2.0 * max(self.max_decel, self.car_decel))
            need = v * self.headway + v * v / (2.0 * self.max_decel) - leader_brake
            if a < 0.0:   # 이미 줄이는 중이면 SUMO는 반응 시간 없이 그 감속으로 본다
                need = min(need, v * v / (2.0 * min(-a, self.max_decel)) - leader_brake)
            if gap < need:
                return False
        # 버스보다 빠른 뒤차만 기다린다. 느리거나 같은 뒤차는 앞지르지 않으므로(버스 뒤로 들어오려는 차이거나 양보하는
        # 차) 기다리면 둘 다 차로 끝에 선다. 게이트만 넘으면 요청하고, 그 차의 실제 간격(추종모델마다 tau가 다르다)은 SUMO가 본다.
        if follow is not None and follow[1] > v:
            gap, fv = follow
            need = fv * self.car_headway + fv * fv / (2.0 * self.car_decel) - v * v / (2.0 * max(self.car_decel, self.max_decel))
            if gap < need:
                return False
        return True

    def _safe_speed_candidates(self, obs, v):
        """SUMO(속도 모드 31)는 버스를 Krauss 안전 속도로 묶는다. 앞차가 4.5m/s²로 서도 버스가 반응 시간 tau 뒤
        2m/s²로 그 뒤에 설 수 있는 속도다(MSCFModel::maximumSafeFollowSpeed). IDM은 이보다 가깝게 따라가 v4 개입의
        절반이 여기서 났고, v5가 J82 좌회전 차로에 일찍 들어가 앞차를 따라가면 더 늘었다. 이 속도를 넘지 않게 맞춘다."""
        gap = self._f(obs, "current_leader_gap")
        if gap >= 1.0:
            return []
        lv = max(v + self._f(obs, "current_leader_relative_speed") * self.vmax, 0.0)
        b, tau = self.max_decel, self.headway
        room = max(gap * self.visibility - self.safe_follow_margin, 0.0) + lv * lv / (2.0 * max(b, self.car_decel))
        v_safe = -b * tau + np.sqrt(b * b * tau * tau + 2.0 * b * room)
        return [(v_safe - v) / self.safe_follow_horizon]

    def _lane_end_merge(self, obs):
        """끝나는 차로에서 서 있거나 기어가는 중인지. 이때는 틈이 없어도 요청한다. 환경이 간격 게이트를 건너뛰는 조건
        (env/mdp_config.ACTION["lane_change_safety"]의 lane_end_*)과 같다. 줄 선 옆 차로에는 틈이 저절로 생기지 않고,
        요청이 SUMO에 가야 뒤차가 끼워 준다. 달리는 중에는 틈 없는 요청이 뒤차를 버스 뒤에 붙잡아 두므로 하지 않는다."""
        offset = self._f(obs, "route_lane_offset")
        if abs(offset) <= 0.25 or self._f(obs, _side(1 if offset > 0 else -1) + "_route_connectivity") < 0.0:
            return False
        return (self._f(obs, "ego_speed") * self.vmax <= self.gate["lane_end_speed"]
                and self._f(obs, "route_lane_continuation") * self.continuation_range <= self.gate["lane_end_within"])

    def lateral(self, obs):
        direction, _remaining = self._wanted_lane(obs)
        if direction and self._f(obs, "stop_dwell_remaining") <= 0.0 and (
                self._merge_ok(obs, _side(direction)) or self._lane_end_merge(obs)):
            return float(direction)
        return 0.0

    def _gap_search_candidates(self, obs, v, v0):
        """옮길 수 없으면 편안한 감속 안에서 줄여 옆 차를 먼저 보낸다. 끝이 가까우면 기어간다. 느릴수록 SUMO가
        요구하는 간격이 짧아지고, 빠른 뒤차가 버스를 빨리 지나간다. 옆 차로 앞차보다도 느리게 간다. 버스 차로로
        가야 하는 그 차는 버스를 먼저 보내려고 버스 속도에 맞춰 달리므로, 제한속도 기준으로만 줄이면 나란히 간다."""
        direction, remaining = self._wanted_lane(obs)
        if direction and remaining < self.gap_search_within and not self._merge_ok(obs, _side(direction)):
            v_slow = self.creep_speed if remaining < self.creep_within else self.gap_search_speed * v0
            lead, _follow = self._neighbors(obs, _side(direction))
            if lead is not None:
                v_slow = min(v_slow, max(lead[1] - self.gap_search_margin, self.creep_speed))
            if v > v_slow:
                return [max(self._idm(v, v_slow, None, 0.0, 0.0), -self.comfortable_decel)]
        return []

    def _stop_candidates(self, obs, v, v0):
        """옆 차로에 환경이 받아 줄 틈이 있으면(옮기는 중이거나 곧 옮긴다) 15m 안에서도 정류장을 포기하지 않는다."""
        stop_distance = self._f(obs, "stop_distance") * self.stop_range
        in_stop_lane = abs(self._f(obs, "stop_lane_offset")) < 0.25
        direction = 1 if self._f(obs, "stop_lane_offset") > 0 else -1
        merging = stop_distance > self.merge_floor and self._gate_ok(obs, _side(direction))
        if stop_distance < self.stop_range and (in_stop_lane or stop_distance > self.stop_give_up or merging):
            return [self._idm(v, v0, stop_distance + self.target_gap, v, self.target_gap)]
        return []


EXPERT_VERSIONS = {4: ExpertPolicy, 5: ExpertPolicyV5}


def make_expert(env, version=4):
    """환경(SumoHighwayEnv)의 정규화 값으로 전문가 정책을 만든다. 4는 BC 데이터와 Residual RL이 학습한 판이다."""
    from env import sumo_env
    return EXPERT_VERSIONS[version](env.vmax, env.visibility, env.max_accel, env.max_decel,
                                    sumo_env.TL_RANGE, sumo_env.STOP_RANGE, sumo_env.CONTINUATION_RANGE,
                                    sumo_env.CROSSING_RANGE, sumo_env.LIMIT_RANGE)
