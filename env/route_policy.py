"""학습 없이 경로를 따라가는 규칙 정책. 데이터 수집(--policy)과 scripts/validate_road.py가 쓴다.

numpy만 쓴다. 도로만 다시 만드는 사람에게 torch를 요구하지 않는다.

- route_following_action: 수업 원본 31차원만 보는 옛 규칙. 늘 같은 가속을 내고 신호와 정류장을 보지 못한다.
  안전 보조를 끈 지금은 첫 적색 신호나 앞차에서 끝난다. 비교 기준으로 남긴다.
- ExpertPolicy: 43차원 관측(env/data_schema.OBSERVATION_FEATURES)만 보는 전문가 규칙(v4). 내부 상태를 두지 않는다.
  같은 관측이면 같은 행동이다.
- ExpertPolicyV5: v4의 차선변경 판단과 앞차 추종을 버스 제원으로 설 수 있는 간격에 맞춘 판(--policy expert-v5).
  데이터 수집 정책이다. 무엇을 쓰고 무엇을 쓰지 않는지는 research/av_bc_rl/collection_policy.md에 있다.
- ExpertPolicyV6: v5에서 신호 앞에 서는 방법만 따라 배우기 쉽게 바꾼 판(--policy expert-v6). 신경망의 정답을 만든다.

두 전문가가 쓰는 것은 정책 입력(관측 43개)과 생성자가 받는 고정 값(차량 제원, 관측 정규화 거리, 환경 게이트 값)뿐이다.
SUMO 내부 상태, 도로 id, 위치, 차로 번호, 지난 관측은 쓰지 않는다. 환경도 버스를 돕지 않는다(speedMode 0,
laneChangeMode 0). 앞차, 적색 신호, 보행자 앞에서 서는 것은 모두 이 규칙이 한다.
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
    그 자리에 선 가상의 정지 차량으로 본다. 횡방향은 차로별 연결성으로 경로가 이어지는 차로를, 정류장 앞에서는
    가장 오른쪽 차로를 고르고 간격이 충분할 때만 옮긴다.

    차로를 고르는 근거는 둘이다. 관측의 차로별 연결성(수업 원본의 값)과, 정류장은 길가(가장 오른쪽 차로)에 있다는
    상식이다. 환경은 어느 차로로 가라고 알려 주지 않는다.

    물리 값은 환경 설정에서 받는다(정규화를 되돌리는 데 필요하다).
    """

    # 버스 운전 파라미터
    comfortable_decel = 1.5   # m/s². 승차감 한계(가속 1.5)와 같다
    headway = 1.5             # s. 버스 tau와 같다
    standstill_gap = 4.0      # m. 앞차와 선 간격
    target_gap = 0.5          # m. 가상 정지점(정지선, 정류장 목표)에 붙여 세우는 간격
    jerk_step = 0.75          # m/s². 판단 한 번(0.5초)에 바꿀 수 있는 가속도(1.5m/s³)
    yield_speed = 6.0         # m/s. 비보호 녹색 앞 접근 속도
    limit_approach_decel = 0.7  # m/s². 앞쪽 낮은 제한속도에 맞춰 줄이는 감속
    limit_approach_margin = 3.0 # m. 그 자리보다 이만큼 앞에서 그 속도가 되게 한다
    limit_approach_time = 1.0   # s. 거기에 이 시간 동안 갈 거리를 더한다(가속도를 바꾸는 데 걸리는 시간)
    stop_lane_ahead = 200.0   # m. 정류장이 이보다 가까우면 가장 오른쪽 차로로 간다
    lane_reach_margin = 10.0  # m. 옆 차로가 지금 차로보다 이만큼 넘게 더 이어져야 옮긴다(교차로 안 길이 차이를 거른다)
    lane_end_brake = 80.0     # m. 끝나는 차로의 끝이 이보다 가까우면 그 앞에 설 준비를 한다
    gap_search_speed = 0.6    # 옮겨야 할 차로에 틈이 없으면 제한속도의 이 비율로 줄여 옆 차를 앞으로 보낸다
    gap_search_within = 150.0 # m. 남은 거리가 이보다 짧을 때만 줄인다
    stop_give_up = 15.0       # m. 이 안까지 정류장 차로에 못 들어가면 그 정류장을 지나친다(옆 차로를 막고 기다리지 않는다)
    crosswalk_stop_gap = 3.0  # m. 사람이 있는 횡단보도 앞에 설 간격
    stop_line_gap = 2.0       # m. 적색 정지선 앞에 설 간격
    yellow_time = 2.5         # s. 황색을 본 뒤 정지선을 넘을 수 있는 시간. 황색 3초에서 판단 간격 0.5초를 뺀다
    yellow_decel = 0.85       # 황색에 설지 정할 때 쓰는 감속(최대 감속의 비율). 가속도가 따라오는 시간을 남긴다
    yellow_margin = 0.5       # m. 설 수 있는지는 정지선 이만큼 앞을 기준으로 본다
    yellow_clear_margin = 3.0 # m. 황색 안에 정지선을 이만큼 넘게 지나갈 수 있어야 지나간다
    lane_end_room = 8.0       # m. 못 옮긴 채 끝나는 차로에서 차로 끝 앞에 남기는 거리

    def __init__(self, vmax, visibility, max_accel, max_decel,
                 tl_range=100.0, stop_range=300.0, connectivity_range=None, crossing_range=50.0, limit_range=100.0):
        self.vmax, self.visibility = float(vmax), float(visibility)
        self.max_accel, self.max_decel = float(max_accel), float(max_decel)
        self.tl_range, self.stop_range = float(tl_range), float(stop_range)
        self.connectivity_range = float(visibility if connectivity_range is None else connectivity_range)
        self.crossing_range, self.limit_range = float(crossing_range), float(limit_range)

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
        # 비보호 녹색 앞에서는 천천히 다가간다
        tl_distance = self._f(obs, "tl_distance") * self.tl_range
        if self._f(obs, "tl_yield") > 0.5 and tl_distance < 40.0:
            v0 = min(v0, self.yield_speed)
        # 앞쪽의 낮은 허용 속도(회전 차로, 곡선)에 닿을 때 그 속도가 되도록 목표 속도를 미리 낮춘다. 그 속도를 넘은 뒤에
        # 줄이기 시작하면 늦다. 교차로 앞에 섰다 출발한 버스가 곡선 안에서 가속해 횡가속도 한계를 넘었다(J83 2.6m/s²).
        limit = self._f(obs, "next_speed_limit") * self.vmax
        limit_distance = self._f(obs, "next_speed_limit_distance") * self.limit_range
        room = limit_distance - self.limit_approach_margin - v * self.limit_approach_time
        v0 = min(v0, np.sqrt(limit * limit + 2.0 * self.limit_approach_decel * max(room, 0.0)))
        red, yellow = self._f(obs, "tl_red") > 0.5, self._f(obs, "tl_yellow") > 0.5
        current = self._f(obs, "ego_acceleration") * 2.0
        candidates = []
        if tl_distance < self.tl_range and not red:
            approach = self._signal_approach_speed(tl_distance)
            v0 = min(v0, approach)
            candidates += self._signal_approach_candidates(v, approach)
        candidates.append(self._idm(v, v0, None, 0.0, 0.0))

        leader_gap = self._f(obs, "current_leader_gap")
        if leader_gap < 1.0:
            leader_speed = v + self._f(obs, "current_leader_relative_speed") * self.vmax
            candidates.append(self._idm(v, v0, leader_gap * self.visibility, v - leader_speed, self.standstill_gap))
        candidates += self._safe_speed_candidates(obs, v)

        # 적색이면 정지선에 선다. 황색은 정지선 앞에 설 수 있으면 서고, 못 서면 지나간다.
        stopping = red or (yellow and self._stops_for_yellow(v, current, tl_distance))
        if stopping:
            if self._holds_at_signal(v, tl_distance):
                return -self.max_decel
            candidates.append(self._signal_stop(v, v0, tl_distance, yellow))

        # 이미 빠르면 그 자리까지 필요한 감속으로 줄인다
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
        step = self.jerk_step * (2.0 if target < current - self.jerk_step and target < -self.comfortable_decel else 1.0)
        rise = float("inf") if yellow and not stopping and self.clears_yellow_at_once else step
        return float(np.clip(target, current - step, current + rise))

    clears_yellow_at_once = False   # 황색을 지나가기로 했을 때 가속도 변화 제한을 푸는지

    def _signal_approach_candidates(self, v, approach):
        """신호에 다가가는 속도의 상한을 넘었을 때 더하는 감속. v4와 v5는 목표 속도만 낮춘다(IDM)."""
        return []

    def _holds_at_signal(self, v, distance):
        """신호 앞에 이미 섰으면 그대로 서 있을지. v4와 v5는 정지점까지 다시 다가간다."""
        return False

    def _signal_stop(self, v, v0, distance, yellow):
        """정지선 앞에 서는 가속도. 정지점에 선 가상의 차를 IDM으로 따라간다."""
        gap = max(distance - self.stop_line_gap, 0.0) + self.target_gap
        return self._idm(v, v0, gap, v, self.target_gap)

    def _stops_for_yellow(self, v, acceleration, distance):
        """황색에 설지. 여유 있는 감속(yellow_decel)으로 정지선 앞에 설 수 있으면 선다. 최대 감속으로만 설 수 있으면,
        이미 세게 줄이는 중이거나 황색 안에 지나갈 수 없을 때 선다. 판단마다 다시 정해도 선다와 간다가 뒤바뀌지 않게
        하려는 것이다. 설 수 있다고 줄이다가 가속도가 늦게 따라와 다음 판단에 못 선다로 바뀌면, 그사이 느려진 버스는
        적색에 정지선을 넘는다(train quiet 2010, J82). 지나갈 수 있는지는 지금 줄이는 중이면 그 감속이 이어진다고 보고
        센다. 회전 앞에서 줄이던 버스가 같은 속도로 간다고 보고 지나가다 적색에 걸렸다(train quiet 2021, J82)."""
        need = v * v / (2.0 * max(distance - self.yellow_margin, 0.1))
        if need <= self.yellow_decel * self.max_decel:
            return True
        braking = acceleration <= -0.5 * self.max_decel
        reach = v * self.yellow_time + 0.5 * min(acceleration, 0.0) * self.yellow_time ** 2
        return need <= self.max_decel and (braking or distance + self.yellow_clear_margin > reach)

    def _signal_approach_speed(self, distance):
        """적색이 아닌 신호에 다가가는 속도의 상한. 이 속도 이하면 황색이 언제 켜져도 정지선 앞에 서거나 황색 안에 지나갈 수
        있다. 감속 2m/s²인 버스가 50km/h로 달리면 황색 3초로는 정지선 35~57m 앞에서 둘 다 못 한다."""
        b = self.yellow_decel * self.max_decel
        either = b * self.yellow_time + np.sqrt((b * self.yellow_time) ** 2 - 2.0 * b * self.yellow_margin)
        beyond = distance - either * self.yellow_time
        return float("inf") if beyond <= 0.0 else float(np.sqrt(either * either + 2.0 * self.limit_approach_decel * beyond))

    def _safe_speed_candidates(self, obs, v):
        """v4는 앞차 추종을 IDM 하나로만 정한다."""
        return []

    def _reach(self, obs, lane):
        """그 차로가 경로를 따라 이어지는 거리(m). 보는 거리 끝까지 이어지면 inf, 차로가 없으면 -1."""
        connectivity = self._f(obs, f"{lane}_route_connectivity")
        if connectivity < 0.0:
            return -1.0
        return float("inf") if connectivity >= 1.0 else connectivity * self.connectivity_range

    def _stop_lane_on_right(self, obs):
        """정류장까지 이어지는 차로가 오른쪽에 있는지. 없으면 지금 차로가 정류장이 있는 길가 차로다."""
        stop_distance = self._f(obs, "stop_distance") * self.stop_range
        return self._reach(obs, "right1") >= max(stop_distance - self.lane_reach_margin, 0.0)

    def _stop_candidates(self, obs, v, v0):
        """정차할 정류장: 정류장 목표에 세운다. 아직 길가 차로가 아니면 목표 앞에서 기다렸다가 틈이 나면 옮긴다."""
        stop_distance = self._f(obs, "stop_distance") * self.stop_range
        if stop_distance < self.stop_range and (not self._stop_lane_on_right(obs) or stop_distance > self.stop_give_up):
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
        """지금 차로가 경로에서 끊기면 그 끝 앞에 선다. 그 전에 옮기면 이 정지점은 사라진다."""
        lane_end = self._reach(obs, "current")
        if lane_end < self.lane_end_brake:
            return [self._idm(v, v0, max(lane_end - self.lane_end_room, 0.0) + self.target_gap, v, self.target_gap)]
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
        """옮겨야 할 방향(+1 왼쪽, -1 오른쪽, 0 없음)과 그 일을 끝내야 하는 곳까지 남은 거리(m).

        차로별 연결성만 본다. 정류장이 지금 차로의 끝보다 가까우면 정류장이 먼저다. 오른쪽에 정류장까지 이어지는
        차로가 있으면 그리로 간다. 그렇지 않고 지금 차로가 끊기면 더 멀리 이어지는 쪽으로 간다.
        """
        stop_distance = self._f(obs, "stop_distance") * self.stop_range
        lane_end = self._reach(obs, "current")
        if stop_distance < min(self.stop_lane_ahead, lane_end):
            return (-1, stop_distance) if self._stop_lane_on_right(obs) else (0, float("inf"))
        if lane_end < float("inf"):
            left = max(self._reach(obs, "left1"), self._reach(obs, "left2")) if self._reach(obs, "left1") >= 0.0 else -1.0
            right = max(self._reach(obs, "right1"), self._reach(obs, "right2")) if self._reach(obs, "right1") >= 0.0 else -1.0
            if max(left, right) > lane_end + self.lane_reach_margin:
                return (1 if left >= right else -1), lane_end
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
    """데이터 수집 정책. v4의 차선변경 판단과 앞차 추종을 버스 제원으로 실제로 설 수 있는 간격에 맞춘 판이다.
    같은 관측만 보고 내부 상태가 없다.

    안전 보조가 꺼져 있어 SUMO는 버스를 세워 주지 않고, 차선변경 요청도 옆 차와 무관하게 실행한다. 환경의 간격
    게이트(3m + 0.15v)만 넘으면 버스가 옆으로 간다. 버스는 감속이 약해(2m/s²) 그 간격으로는 앞차가 서면 못 선다.
    v5는 앞차가 4.5m/s²로 서도 반응 시간 뒤 2m/s²로 그 뒤에 설 수 있는 속도(Krauss 안전 속도)를 넘지 않고,
    옮긴 뒤에도 그 간격이 남을 때만 요청한다. 옮길 수 없으면 줄여서 옆 차를 먼저 보낸다.
    """

    # 환경의 차선변경 안전 게이트(env/mdp_config.ACTION["lane_change_safety"]). 이것을 넘어야 요청이 실행된다.
    gate = ACTION["lane_change_safety"]
    gate_margin = 0.5        # m
    # 배경차 제원의 어림값. env/road_config의 추종모델 중 가장 긴 tau와 그 감속이다.
    car_decel = 4.5          # m/s²
    car_headway = 1.8        # s
    merge_floor = 4.0        # m. 옮기는 중이어도 정류장까지 이보다 가까우면 지나친다(옆 차로에 선 채 기다리지 않는다)
    creep_speed = 3.0        # m/s. 옮길 곳(정류장, 차로 끝)이 가까운데 옮길 수 없으면 이 속도로 기어간다
    creep_within = 40.0      # m
    gap_search_margin = 1.5  # m/s. 틈을 찾을 때 옆 차로 앞차보다 이만큼 느리게 간다
    safe_follow_horizon = 1.0  # s. 안전 속도까지 남은 차이를 이 시간에 걸쳐 맞춘다
    safe_follow_margin = 1.0   # m
    # 옆 차로 끝에 선 차가 버스 앞으로 들어올 자리. 승용차 5m + 버스 최소 간격 3m + 그 차가 차로 끝 앞에 남긴 0.4m는 8.4m다.
    # 8m에서는 그 차가 0.1m 차이로 못 들어와 옆 차로 끝을 막았고, 그 차로로 가야 하는 버스도 같이 섰다(final busy 9600).
    lane_end_room = 10.0       # m

    def _neighbors(self, obs, side):
        """옆 차로의 (앞차 간격 m, 앞차 속도), (뒤차 간격 m, 뒤차 속도). 차가 없으면 None."""
        v = self._f(obs, "ego_speed") * self.vmax
        front, rear = self._f(obs, f"{side}_leader_gap"), self._f(obs, f"{side}_follower_gap")
        lead = (front * self.visibility, max(v + self._f(obs, f"{side}_leader_relative_speed") * self.vmax, 0.0)) if front < 1.0 else None
        follow = (-rear * self.visibility, max(v + self._f(obs, f"{side}_follower_relative_speed") * self.vmax, 0.0)) if rear > -1.0 else None
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
        """옮긴 뒤에도 앞차 뒤에 설 수 있고 뒤차가 버스 뒤에 설 수 있는 간격인지. 버스는 감속이 약해(2m/s²) 빠를수록
        앞 간격이 길어야 한다."""
        if not self._gate_ok(obs, side):
            return False
        v, a = self._f(obs, "ego_speed") * self.vmax, self._f(obs, "ego_acceleration") * 2.0
        lead, follow = self._neighbors(obs, side)
        if lead is not None:
            gap, lv = lead
            leader_brake = lv * lv / (2.0 * max(self.max_decel, self.car_decel))
            need = v * self.headway + v * v / (2.0 * self.max_decel) - leader_brake
            if a < 0.0:   # 이미 줄이는 중이면 반응 시간 없이 그 감속으로 본다
                need = min(need, v * v / (2.0 * min(-a, self.max_decel)) - leader_brake)
            if gap < need:
                return False
        # 버스보다 빠른 뒤차만 기다린다. 느리거나 같은 뒤차는 앞지르지 않으므로(버스 뒤로 들어오려는 차이거나 양보하는
        # 차) 기다리면 둘 다 차로 끝에 선다. 그런 뒤차는 게이트만 넘으면 요청한다.
        if follow is not None and follow[1] > v:
            gap, fv = follow
            need = fv * self.car_headway + fv * fv / (2.0 * self.car_decel) - v * v / (2.0 * max(self.car_decel, self.max_decel))
            if gap < need:
                return False
        return True

    def _safe_speed_candidates(self, obs, v):
        """앞차가 4.5m/s²로 서도 버스가 반응 시간 tau 뒤 2m/s²로 그 뒤에 설 수 있는 속도(Krauss 안전 속도,
        SUMO의 MSCFModel::maximumSafeFollowSpeed와 같은 식)를 넘지 않는다. IDM은 이보다 가깝게 따라간다. 안전 보조가
        꺼져 있으므로 이 속도를 지키는 것이 버스의 추돌 회피다."""
        gap = self._f(obs, "current_leader_gap")
        if gap >= 1.0:
            return []
        lv = max(v + self._f(obs, "current_leader_relative_speed") * self.vmax, 0.0)
        b, tau = self.max_decel, self.headway
        room = max(gap * self.visibility - self.safe_follow_margin, 0.0) + lv * lv / (2.0 * max(b, self.car_decel))
        v_safe = -b * tau + np.sqrt(b * b * tau * tau + 2.0 * b * room)
        return [(v_safe - v) / self.safe_follow_horizon]

    def lateral(self, obs):
        direction, _remaining = self._wanted_lane(obs)
        if direction and self._f(obs, "stop_dwell_remaining") <= 0.0 and self._merge_ok(obs, _side(direction)):
            return float(direction)
        return 0.0

    def _gap_search_candidates(self, obs, v, v0):
        """옮길 수 없으면 편안한 감속 안에서 줄여 옆 차를 먼저 보낸다. 끝이 가까우면 기어간다. 느릴수록 필요한
        간격이 짧아지고, 빠른 뒤차가 버스를 빨리 지나간다. 옆 차로 앞차보다도 느리게 간다. 버스 차로로
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
        """오른쪽 차로에 환경이 받아 줄 틈이 있으면(옮기는 중이거나 곧 옮긴다) 15m 안에서도 정류장을 포기하지 않는다."""
        stop_distance = self._f(obs, "stop_distance") * self.stop_range
        merging = stop_distance > self.merge_floor and self._gate_ok(obs, "right1")
        if stop_distance < self.stop_range and (
                not self._stop_lane_on_right(obs) or stop_distance > self.stop_give_up or merging):
            return [self._idm(v, v0, stop_distance + self.target_gap, v, self.target_gap)]
        return []


class ExpertPolicyV6(ExpertPolicyV5):
    """v5에서 신호 앞에 서는 방법만 따라 배우기 쉽게 바꾼 판(--policy expert-v6). 신경망에 붙일 정답을 만드는 데 쓴다.

    v5를 따라 배운 신경망은 적색 앞에서 두 가지로 정지선을 넘었다. 하나는 정지선 4m 앞에 선 뒤 다시 다가가다 넘은
    것이다. v5가 정지점(정지선 2m 앞)까지 조금씩 다가가므로 "적색에 서 있다가 가속"이 정답에 들어 있었다. 다른 하나는
    황색에 설 때 최대 감속에 가까운 값(1.7m/s²)이 필요한 속도로 다가가서, 조금만 덜 줄여도 못 서게 된 것이다.

    v6은 정지점을 정지선 4m 앞으로 당기고, 그 앞 8m 안에서는 가속하지 않고, 서면 신호가 바뀔 때까지 그대로 선다.
    서는 감속은 정지점까지 필요한 감속(v²/2d) 그대로다. 그 값이 1m/s²에 이르면 줄이기 시작하므로 늦게 따라가도
    최대 감속(2m/s²)까지 여유가 있다. 황색에 서기로 했으면 멀리서도 가속하지 않는다. 내부 상태는 여전히 없다.

    황색에 서지도 지나가지도 못하는 속도도 막는다. v5는 다가가는 속도 상한을 "황색 2.5초 동안 정지선까지 갈 수 있다"로
    세고, 지나갈지는 "정지선을 3m 넘게 지나갈 수 있다"로 정해 둘 사이에 빈틈이 있었다. 상한도 IDM의 목표 속도로만
    써서 10%쯤 넘겨 달렸다. 신경망은 그 빈틈에서 황색을 만나 덜 줄이다가 적색에 넘었다(R111b). v6은 두 곳에 같은
    여유(1m)를 쓰고, 상한을 넘으면 넘은 만큼 바로 줄이고, 지나가기로 했으면 가속도 변화 제한 없이 바로 가속한다.

    앞이 비어 보여도 보이는 거리(50m) 끝에 설 수 있는 속도(12m/s)를 넘지 않는다.
    """

    stop_line_gap = 4.0       # m. 적색 정지선 앞에 설 간격
    hold_within = 8.0         # m. 정지점 앞 이 거리 안에서는 적색에 가속하지 않고, 섰으면 그대로 선다
    hold_speed = 0.3          # m/s. 이보다 느리면 선 것으로 본다
    signal_decel = 1.0        # m/s². 정지점까지 필요한 감속이 이 값이 되면 그 감속으로 줄인다
    signal_decel_onset = 0.5  # m/s². 필요한 감속이 이 값을 넘으면 줄이기 시작한다(사이는 비례)
    yellow_decel = 0.75       # 황색에 설지 정할 때 쓰는 감속(최대 감속의 비율). v5의 0.85보다 낮춰 여유를 둔다
    yellow_clear_margin = 1.0 # m. 황색 2.5초는 판단 한 번(0.5초, 7m/s면 3.5m)을 이미 뺀 값이라 v5의 3m보다 줄인다
    approach_horizon = 1.0    # s. 신호에 다가가는 속도 상한을 넘은 만큼을 이 시간에 줄인다
    clears_yellow_at_once = True
    sight_headway = 1.0       # s. 보이는 거리 끝에 설 수 있는 속도를 셀 때의 반응 시간(판단 0.5초 + 감속이 따라오는 시간)

    def _signal_approach_speed(self, distance):
        """황색이 언제 켜져도 여유 있는 감속으로 서거나 정지선을 yellow_clear_margin만큼 넘게 지나갈 수 있는 속도."""
        b, t = self.yellow_decel * self.max_decel, self.yellow_time
        either = b * t + np.sqrt((b * t) ** 2 - 2.0 * b * (self.yellow_margin + self.yellow_clear_margin))
        beyond = distance - (either * t - self.yellow_clear_margin)
        return float("inf") if beyond <= 0.0 else float(np.sqrt(either * either + 2.0 * self.limit_approach_decel * beyond))

    def _signal_approach_candidates(self, v, approach):
        return [(approach - v) / self.approach_horizon] if v > approach else []

    def _safe_speed_candidates(self, obs, v):
        """앞차가 안 보여도 보이는 거리 끝에 선 차가 있다고 보고 그 앞에 설 수 있는 속도를 넘지 않는다. 버스는
        13.9m/s에서 서는 데 48m가 들어, 50m 밖에 선 대기열 끝이 보이기 시작한 뒤에는 못 선다(train busy 2606, R54)."""
        if self._f(obs, "current_leader_gap") < 1.0:
            return super()._safe_speed_candidates(obs, v)
        b, tau = self.max_decel, self.sight_headway
        v_safe = -b * tau + np.sqrt(b * b * tau * tau + 2.0 * b * (self.visibility - self.safe_follow_margin))
        return [(v_safe - v) / self.safe_follow_horizon]

    def _holds_at_signal(self, v, distance):
        return v < self.hold_speed and distance - self.stop_line_gap < self.hold_within

    def _signal_stop(self, v, v0, distance, yellow):
        room = max(distance - self.stop_line_gap, 0.0)
        needed = v * v / (2.0 * max(room, 0.1))
        onset, full = self.signal_decel_onset, self.signal_decel
        brake = -min(needed, self.max_decel) * float(np.clip((needed - onset) / (full - onset), 0.0, 1.0))
        if needed > onset or yellow or room < self.hold_within:
            return brake
        # 멀리 있는 적색에는 줄이기 시작할 속도까지만 다가간다
        return self._idm(v, min(v0, np.sqrt(2.0 * onset * room)), None, 0.0, 0.0)


EXPERT_VERSIONS = {4: ExpertPolicy, 5: ExpertPolicyV5, 6: ExpertPolicyV6}


def make_expert(env, version=4):
    """환경(SumoHighwayEnv)의 정규화 값으로 전문가 정책을 만든다."""
    from env import sumo_env
    return EXPERT_VERSIONS[version](env.vmax, env.visibility, env.max_accel, env.max_decel,
                                    sumo_env.TL_RANGE, sumo_env.STOP_RANGE, env.connectivity_range,
                                    sumo_env.CROSSING_RANGE, sumo_env.LIMIT_RANGE)
