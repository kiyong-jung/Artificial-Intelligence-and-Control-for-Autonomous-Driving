"""도로의 차량·교통 설정.

도로 형상과 경로·수요는 maps/<지도 id>/map.json, 차량 특성은 이 파일,
교통량 배율은 scenario가 맡는다. 실행 진입점이 configure_map()을 호출해
세 dict를 제자리 갱신하므로 `from env.road_config import ROAD` 참조가 유지된다.
단위: 거리 m, 속도 m/s, 가속도 m/s², 교통량 대/시간.
"""
from copy import deepcopy

from maps import DEFAULT_MAP, load_map

# baseline 자체가 혼잡한 수요다. busy는 내부 차량이 더 늘어나는 범위에서 1.25배를 쓴다.
# heavy(1.5배)는 상호작용을 늘리려고 나중 단계에 더하는 교통량이다. 규칙 운전사 v7이 10바퀴를 모두 완주했고
# 1.75배부터는 서교동사거리 앞 끼어들기에서 멈춰 선다(reports/progress_log.md 30번).
SCENARIOS = {"quiet": 0.5, "baseline": 1.0, "busy": 1.25, "heavy": 1.5}

# ego 차량 제원. 지도마다 map.json의 ego_vehicle이 고른다(없으면 car).
# 가감속 한계는 행동 a=±1의 크기다. 실제 한 스텝의 가감속은 mdp_config의 행동 정의가 결정한다.
_EGO_COMMON = {
    # 제한속도 배율을 seed로 뽑지 않는다. ego의 최고속도는 seed와 무관하다. SUMO는 speedFactor 하나만
    # 주면 그 값을 평균으로 쓰고 기본 편차 0.1로 여전히 뽑으므로 speedDev=0이 있어야 고정된다.
    "speed_factor": 1, "speed_dev": 0,
    "depart_lane": "best", # 진입부 차선 수가 경로마다 다르므로 SUMO가 연결 차선을 고른다.
    "depart_speed": 0.0,   # 정지 출발. 혼잡한 진입부에서 빈틈이 작아도 투입된다.
    # 수업 원본(upstream env/sumo_env.py)과 같다. speed_mode 0은 SUMO의 안전 속도, 가감속 한계, 교차로 우선권,
    # 적색 정지를 모두 끈다. 켜 두면 SUMO가 대신 브레이크를 밟아 충돌 회피를 배울 신호가 사라진다.
    # lane_change_mode 0은 SUMO의 자율 차선변경을 끄고 TraCI 요청을 옆 차 간격과 무관하게 실행한다. 측면충돌은
    # 수업 원본의 간격 게이트(mdp_config.ACTION["lane_change_safety"])와 정책이 막는다. 이 모드에서는 차선변경이
    # 차로 가운데 0.35m 앞에서 끝나므로 환경이 가운데까지 옮긴다(env/sumo_env.py _steer).
    "speed_mode": 0, "lane_change_mode": 0,
}
_EGO_VEHICLES = {
    # 수업 원본의 승용차.
    "car": {"vclass": "passenger", "gui_shape": "passenger/sedan", "accel": 5.4, "decel": 5.4,
            "emergency_decel": 9.0,   # SUMO의 승용차 기본값. 안전 보조의 제동 한계다.
            "max_speed": 33.33},
    # 시내버스. 10/29 회의에서 정한 버스 특징(긴 차체, 제한된 가속, 승객 편안함)을 수치로 둔다.
    # 길이·폭은 대형 시내버스, 가감속은 서 있는 승객이 견디는 범위다. comfort의 한계를 넘으면
    # 기록에 위반으로 남는다(행동을 막지는 않는다). 현장 실측값은 아니다.
    # lateral_acceleration_limit에서는 곡선 속도 제한이 나온다. 차로 형상의 곡률에서 만들어 관측(speed_limit,
    # next_speed_limit*)에 넣는다(env/route_map.curve_speeds). 환경은 이 속도로 자르지 않는다. 지키는 것은 정책이다.
    # 2.0m/s²(약 0.2g)는 이 표의 감속 한계(decel 2.0)와 같은 크기다. 도로망의
    # 교차로 속도는 승용차 기준 5.5m/s²(netconvert junctions.limit-turn-speed 기본값)이고, 여러 내부 차로로 나뉜
    # 회전(J42)에는 그 제한도 걸리지 않아 버스가 47km/h로 좌회전했다.
    "bus": {"vclass": "bus", "gui_shape": "bus", "length": 12.0, "width": 2.5,
            "accel": 1.0, "decel": 2.0, "emergency_decel": 9.0, "min_gap": 3.0, "tau": 1.5,
            "person_capacity": 40, "boarding_duration": 2.0, "max_speed": 33.33,
            "comfort": {"acceleration_limit": 1.5, "jerk_limit": 2.0, "hard_braking": 3.0,
                        "lateral_acceleration_limit": 2.0}},
}
BUS_COMFORT = _EGO_VEHICLES["bus"]["comfort"]   # 행동 지표(research/av_bc_rl/behavior_metrics.py)가 같은 한계를 쓴다
# 지도별 버스 운행 설정. 정차할 정류장(map.json의 bus_stops id)과 한 번 서는 시간(초).
# 합정역은 팀이 정한 s2 성산중학교, s4 극동방송, s5 서교동교회, s6 합정역에 8초씩 선다(traffic/test_bus.py와 같다).
_SERVICE = {
    "hapjeong": {"stops": ("osm_stop_357920024", "osm_stop_5930628870",
                           "osm_stop_3446267493", "osm_stop_3424857165"), "dwell_s": 8.0},
}
_VTYPE_ATTRIBUTES = (("vclass", "vClass"), ("gui_shape", "guiShape"), ("length", "length"), ("width", "width"),
                     ("accel", "accel"), ("decel", "decel"), ("emergency_decel", "emergencyDecel"),
                     ("min_gap", "minGap"), ("tau", "tau"), ("person_capacity", "personCapacity"),
                     ("boarding_duration", "boardingDuration"), ("max_speed", "maxSpeed"))


def vtype_attributes(vehicle: dict) -> dict:
    """제원 dict를 SUMO vType 속성으로 바꾼다. 없는 값은 SUMO 기본값을 쓴다."""
    return {attribute: str(vehicle[key]) for key, attribute in _VTYPE_ATTRIBUTES if key in vehicle}


def bus_vtype_attributes() -> dict:
    """ego가 아닌 버스(점검용, 배경 버스)도 같은 제원을 쓴다."""
    return vtype_attributes(_EGO_VEHICLES["bus"])


# 배경차는 추종모델 네 종류를 섞어 특정 모델의 버릇에 과적합되지 않게 한다.
# IDM/EIDM은 선호 최소간격을 수 mm 침범할 수 있는 근사 모델이라 0.1초 간격과
# tau=1.8초를 쓴다. 현장 운전행태 실측값은 아니다.
_CONTROLLERS = {
    "krauss": {"probability": .35, "carFollowModel": "Krauss",
               "accel": 2.0, "decel": 4.5, "tau": 1.2,
               "min_gap": 2.5, "sigma": .3, "speed_dev": .08, "color": "1,1,0"},
    "idm": {"probability": .30, "carFollowModel": "IDM",
            "accel": 1.8, "decel": 4.5, "tau": 1.8,
            "min_gap": 2.5, "speed_dev": .08, "color": "0,1,1"},
    "eidm": {"probability": .20, "carFollowModel": "EIDM",
             "accel": 2.0, "decel": 4.5, "tau": 1.8,
             "min_gap": 2.5, "speed_dev": .08, "color": "1,.6,0"},
    "acc": {"probability": .15, "carFollowModel": "ACC",
            "accel": 2.0, "decel": 4.5, "tau": 1.5,
            # 기본 +2m/s 여유 대신 Krauss의 안전 추종속도를 상한으로 써서
            # 짧은 합류부의 ACC 후미 추돌을 막는다.
            "collision_avoidance_override": 0.0,
            "min_gap": 2.5, "speed_dev": .05, "color": "1,0,1"},
}

ROAD, EGO, TRAFFIC = {}, {}, {}


def configure_map(scenario="baseline", seed=0, ego_route=None, map_id=DEFAULT_MAP):
    """지도, 교통 조건, ego 경로를 고르고 ROAD, EGO, TRAFFIC을 제자리 갱신한다.

    ego_route가 None이면 map.json의 ego_route다. 잘못된 값이면 아무것도 바꾸지 않는다.
    seed는 SUMO와 환경의 초기 난수에 전달한다. 지도 자체는 바뀌지 않으므로
    같은 도로·같은 교통 조건에서 PPO와 BC를 비교할 수 있다.
    """
    if scenario not in SCENARIOS:
        raise ValueError(f"알 수 없는 scenario: {scenario}; 선택: {tuple(SCENARIOS)}")
    # SUMO의 CLI seed는 signed int이다.
    if not isinstance(seed, int) or seed < 0 or seed >= 2**31:
        raise ValueError("seed는 0 이상 2**31 미만의 정수여야 합니다.")
    network = load_map(map_id)
    route_id = network["ego_route"] if ego_route is None else ego_route
    if route_id not in network["routes"]:
        raise ValueError(f"알 수 없는 ego 경로: {route_id!r}; 선택: {tuple(network['routes'])}")
    facts = network["route_design"][route_id]
    ROAD.clear()
    ROAD.update(map_id=network["id"], map_version=network["version"], name=network["name"],
                network=network, length=facts["length_m"], num_lanes=facts["first_edge_lanes"],
                speed_limit=facts["max_speed_mps"], ego_route=route_id, scenario=scenario, seed=seed)
    EGO.clear()
    vehicle = network.get("ego_vehicle", "car")
    EGO.update({**deepcopy(_EGO_COMMON), **deepcopy(_EGO_VEHICLES[vehicle])}, vehicle=vehicle, route_id=route_id)
    EGO["max_speed"] = min(facts["max_speed_mps"], EGO["max_speed"])
    EGO["service"] = deepcopy(_SERVICE.get(network["id"], {"stops": (), "dwell_s": 0.0}))
    TRAFFIC.clear()
    TRAFFIC.update(
        # 배경차의 물리적 최고속도. 차로 제한속도는 SUMO가 따로 지킨다.
        # 출발 속도는 그 자리에서 안전한 최대값. 고정값이면 교차로 바로 앞 골목 진입로에서 투입하지 못한다.
        max_speed=network["max_lane_speed_mps"], depart_speed="max",
        # 횡방향 속도뿐 아니라 가속도도 제한하는 SL2015를 사용한다.
        # 원형 도로의 순간 횡점프와 저속에서 옆으로 미끄러지는 현상을 줄인다.
        lateral_resolution=0.8,
        # standing_speed: 선 채로 옆으로 움직이는 속도(m/s, lcMaxSpeedLatStanding). 0.3이면 대기열에 선 배경 차가 옆으로
        # 밀리며 30~40도 돌아 뒤차와 겹친다(busy seed 2600에서 15도 넘게 돈 정지 차 26대, 0이면 0대). 버스도 0이다.
        # 버스가 끝나는 차로에서 요청한 차선변경은 SUMO가 급한 변경으로 다뤄 이 값과 무관하게 움직인다(0과 0.3의 결과가 같았다).
        lateral_model={"accel": 0.8, "max_speed": 1.0, "standing_speed": 0.0,
                       "speed_factor": 0.2, "roundabout_cooperation": 0.2},
        # 배경 차는 앞차의 왼쪽으로만 앞지르고(도로교통법 제21조) 오른쪽 차로로 돌아가야 할 의무(keepRight)도 없다.
        # 오른쪽으로 가려는 이 두 동기가 있으면, 대기열 앞에서 왼쪽으로 옮기던 차가 차로 경계를 넘자마자 오른쪽이 낫다고
        # 보고 되돌아온다. 옆 속도는 속도의 0.2배라 그 사이 차가 서면 두 차로에 걸친 채 다음 녹색까지 섰다(R50 66초,
        # 깜빡이 좌우 번갈아). 하나만 끄면 다른 하나가 되돌린다. 측정은 research/av_bc_rl/decisions.tsv.
        background_lane_change={"lcSpeedGainRight": 0.0, "lcKeepRight": 0.0},
        # -1은 SUMO의 모델별 기본값(Krauss=1, IDM/EIDM/ACC=0.1)이다.
        # minGap 전체를 강제하면 몸체가 2.49m 떨어진 정지 차량도 삭제된다.
        collision_min_gap_factor=-1.0,
        # 부딪힌 차가 서 있다가 치워지기까지의 시간(초). stuck_timeout(200초)보다 짧아, 뒤에 선 ego가 배경 사고
        # 하나만으로 stuck이 되지는 않는다.
        collision_stop_s=120.0,
        demand_scale=SCENARIOS[scenario],
        vehs_per_hour=sum(float(f["vehs_per_hour"]) for f in network["flows"]) * SCENARIOS[scenario],
        controllers=deepcopy(_CONTROLLERS),
    )
