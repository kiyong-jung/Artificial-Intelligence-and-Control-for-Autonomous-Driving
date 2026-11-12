"""수집 데이터의 컬럼 표. 수집기의 metadata, CSV 내보내기, 컬럼 검사가 이 표 하나를 쓴다.

한 행은 판단 한 번의 전이다: 현재 관측·실제 상태 → 행동 → 보상·사건·종료 → 다음 관측·실제 상태.
"""
SCHEMA_VERSION = "hapjeong_bus_v1"

SCHOOL, TEAM = "학교 기본", "팀 추가"
# (컬럼, 형태, 구분, 의미). 점(.)은 JSONL에서 한 단계 안쪽 키다.
COLUMNS = [
    ("episode", "Integer", SCHOOL, "Episode 번호"),
    ("t", "Integer", SCHOOL, "Episode 내부 timestep (판단 한 번 = decision_interval초)"),
    ("sim_time", "Float", TEAM, "관측 시점의 SUMO 시뮬레이션 시각(초). t와 실제 시간을 구분한다"),
    ("observation", "Vector (31-dim)", SCHOOL, "시점 t에서 AV Policy가 실제로 입력받는 부분 관측 oₜ"),
    ("action_raw", "Vector (2-dim)", SCHOOL, "Policy가 출력한 원본 Action [가감속, 차선변경]"),
    ("action.accel_raw", "Float", SCHOOL, "환경에 적용된 가감속 Action 값"),
    ("action.lane_change_raw", "Float", SCHOOL, "환경에 적용된 차선변경 raw 값"),
    ("action.lane_change", "Integer", SCHOOL, "실제 명령으로 변환된 차선변경 값 {-1, 0, +1}"),
    ("action.lane_change_applied", "Boolean", SCHOOL, "차선변경 명령이 실제 적용되었는지 여부"),
    ("reward", "Float", SCHOOL, "시점 t에서 받은 Reward rₜ"),
    ("terminated", "Boolean", SCHOOL, "충돌 / 도착 등 환경 자체의 종료 여부"),
    ("truncated", "Boolean", SCHOOL, "최대 Episode Step 등 시간 제한에 의한 종료 여부"),
    ("state", "Dictionary", SCHOOL, "SUMO 내부의 privileged simulator state sₜ (ego, vehicles, pedestrians, navigation, traffic_light)"),
    ("detected_vehicles", "List", SCHOOL, "Observation 시점에 감지된 주변 차량 정보 (관측 차선의 차량과 반경 50m 안의 차량. ego 기준 상대 위치·속도, 방향지시등)"),
    ("detected_pedestrians", "List", TEAM, "Observation 시점에 감지된 보행자 (ego 기준 상대 위치·속도)"),
    ("event.collided", "Boolean", SCHOOL, "해당 step에서 충돌 발생 여부"),
    ("event.arrived", "Boolean", SCHOOL, "목적지 도착 여부"),
    ("event.lane_before", "Integer / String", SCHOOL, "Action 적용 전 AV 차선"),
    ("event.lane_after", "Integer / String", SCHOOL, "Action 적용 후 AV 차선"),
    ("event.termination_reason", "String", TEAM, "종료 원인 (arrived, collision, time_limit, teleport, missing). 종료가 아니면 빈 값"),
    ("comfort.peak_acceleration", "Float", TEAM, "이 step 동안 버스 가속도 크기의 최댓값 (m/s²). 승객 승차감"),
    ("comfort.peak_jerk", "Float", TEAM, "이 step 동안 버스 jerk(가속도 변화율) 크기의 최댓값 (m/s³). 승객 승차감"),
    ("comfort.acceleration_violation", "Boolean", TEAM, "가속도가 승차감 한계를 넘었는지"),
    ("comfort.jerk_violation", "Boolean", TEAM, "jerk가 승차감 한계를 넘었는지"),
    ("next_observation", "Vector (31-dim)", SCHOOL, "다음 시점의 부분 관측 oₜ₊₁"),
    ("next_state", "Dictionary", SCHOOL, "다음 시점의 privileged simulator state sₜ₊₁"),
]

# state 안쪽 키. ego가 사라진 뒤(충돌·도착)의 state는 ego.id와 ego.alive만 있다.
STATE_KEYS = {
    "ego": ["id", "alive", "x", "y", "speed", "acceleration", "jerk", "heading", "road_id", "lane_id",
            "lane_index", "lane_position", "lateral_position", "lateral_speed", "signals"],
    "vehicles": ["id", "x", "y", "speed", "heading", "road_id", "lane_id", "lane_index", "lane_position",
                 "lateral_position", "lateral_speed", "signals"],
    "pedestrians": ["id", "x", "y", "speed", "heading", "road_id"],
    "navigation": ["route_id", "route_edge_index", "drivable_lane_ids", "target_lane_ids", "next_turn",
                   "distance_to_turn", "crossing_id", "distance_to_crossing", "next_stop_id", "distance_to_stop",
                   "distance_to_destination"],
    "traffic_light": ["id", "link_index", "state", "distance_to_stop_line"],
}
DETECTED_VEHICLE_KEYS = ["id", "x", "y", "relative_longitudinal_position", "relative_lateral_position",
                         "relative_longitudinal_speed", "relative_lateral_speed", "relative_heading",
                         "route_distance", "lane_relation", "left_signal", "right_signal"]
DETECTED_PEDESTRIAN_KEYS = ["id", "relative_longitudinal_position", "relative_lateral_position",
                            "relative_longitudinal_speed", "relative_lateral_speed"]

_LANES = ("left2", "left1", "current", "right1", "right2")
OBSERVATION_FEATURES = (
    ["ego_speed"]
    + [f"{lane}_{name}" for lane in _LANES for name in
       ("leader_gap", "leader_relative_speed", "follower_gap", "follower_relative_speed")]
    + [f"{lane}_route_connectivity" for lane in _LANES]
    + [f"{lane}_density" for lane in _LANES])

CONVENTIONS = {
    "coordinate": "x, y는 SUMO 도로망 좌표(m). heading은 북쪽 0도에서 시계 방향(도). "
                  "relative_*는 ego 차체 좌표로, 종방향은 진행 방향이 +, 횡방향은 ego의 왼쪽이 +다. "
                  "relative_heading은 상대가 ego보다 왼쪽(반시계)으로 돌아 있으면 +다.",
    "lane_index": "SUMO 차로 번호. 오른쪽이 0이고 왼쪽으로 갈수록 커진다. 합정역은 0번이 인도라 차도는 1번부터다.",
    "signals": "SUMO getSignals 비트마스크. 1 오른쪽 방향지시등, 2 왼쪽 방향지시등, 8 제동등.",
    "observation": "모든 값은 [-1, 1]. 거리는 visibility, 속도는 경로 제한속도로 나눈다. "
                   "차량이 없으면 선행 간격 1, 후행 간격 -1이고, 차로가 없으면 연결성 -1, 밀도 1이다.",
    "missing": "값이 없으면 null이다(앞에 회전·횡단보도·신호가 없을 때, ego가 사라진 뒤의 state).",
    "jerk": "SUMO 계산 간격(step_length)마다 가속도 차이를 그 간격으로 나눈 값이다. state.ego.jerk는 마지막 "
            "계산의 값, comfort.peak_jerk는 판단 한 번 동안의 절댓값 최대다.",
}


def metadata() -> dict:
    """실행마다 한 번 남기는 데이터 설명."""
    return {"schema_version": SCHEMA_VERSION,
            "columns": [{"name": name, "type": kind, "origin": origin, "meaning": meaning}
                        for name, kind, origin, meaning in COLUMNS],
            "state_keys": STATE_KEYS, "detected_vehicle_keys": DETECTED_VEHICLE_KEYS,
            "detected_pedestrian_keys": DETECTED_PEDESTRIAN_KEYS,
            "observation_feature_names": OBSERVATION_FEATURES, "conventions": CONVENTIONS}


def lookup(record: dict, column: str):
    """점으로 이은 컬럼 이름을 중첩 dict에서 찾는다. 없으면 KeyError다."""
    value = record
    for key in column.split("."):
        value = value[key]
    return value
