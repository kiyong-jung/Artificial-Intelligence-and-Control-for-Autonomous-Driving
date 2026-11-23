"""SUMO 도시 도로 RL 환경 — Gymnasium 인터페이스 + TraCI.

┌─────────────────────────────────────────────────────────────┐
│  강화학습 루프에서 이 클래스의 역할                              │
│                                                             │
│    obs, _ = env.reset()          # 에피소드 시작, 초기 관측     │
│    obs, r, term, trunc, _ = env.step(action)  # 0.5초 진행    │
│                                                             │
│  내부적으로는 TraCI(Traffic Control Interface)라는 소켓 API로  │
│  SUMO 시뮬레이터를 원격 조종한다:                               │
│    traci.simulationStep()  → 시뮬레이션을 1스텝 진행            │
│    traci.vehicle.setSpeed() → 차량 속도 명령                   │
│    traci.vehicle.getSpeed() 등 → 상태 조회                     │
└─────────────────────────────────────────────────────────────┘

관측/행동/보상 설정은 env/mdp_config.py 에 있다(31차원 관측 + 경로 정보, 2차원 행동).
교차 접근 차량은 아직 관측에 없으므로 도시 주행의 완전한 MDP는 아니다.

ego는 수업 원본처럼 SUMO 안전 보조 없이 달린다(speedMode 0, laneChangeMode 0). 앞차, 적색 신호, 보행자를
SUMO가 대신 피해 주지 않는다. 충돌(차, 보행자)과 적색 신호 통과는 환경이 찾아 에피소드를 끝낸다.
"""
import os
import sys

import numpy as np
import gymnasium as gym
from gymnasium import spaces

# SUMO 설치 폴더의 tools/ 안에 traci, sumolib 파이썬 모듈이 들어있다.
# pip로도 설치했지만, 버전 불일치를 피하려면 SUMO_HOME 쪽을 우선 경로에 추가.
#
# SUMO_HOME이 설정 안 돼 있으면 OS별 기본 설치 경로를 순서대로 탐색해서
# 자동으로 잡아준다 (환경변수 설정을 깜빡해도 동작하도록).
if "SUMO_HOME" not in os.environ:
    _candidates = [
        # macOS (Homebrew) — Apple Silicon / Intel
        "/opt/homebrew/opt/sumo/share/sumo",
        "/usr/local/opt/sumo/share/sumo",
        # Linux (apt)
        "/usr/share/sumo",
        # Windows (기본 인스톨러 경로)
        r"C:\Program Files (x86)\Eclipse\Sumo",
        r"C:\Program Files\Eclipse\Sumo",
    ]
    for _p in _candidates:
        if os.path.isdir(_p):
            os.environ["SUMO_HOME"] = _p
            print(f"[sumo_env] SUMO_HOME 자동 감지: {_p}")
            break
    else:
        raise RuntimeError(
            "SUMO_HOME 환경변수가 없고 기본 경로에서도 SUMO를 찾지 못했습니다.\n"
            "  Windows: 인스톨러 재설치(SUMO_HOME 체크) 또는 환경변수 수동 설정\n"
            "  macOS  : README.md의 '설치' 절 참고\n"
            "  Linux  : sudo apt install sumo sumo-tools 후 "
            "export SUMO_HOME=/usr/share/sumo")

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import traci
from sumolib import checkBinary

from env.route_map import RouteMap
from env.data_schema import ROUTE_FEATURES

SIGNAL_RIGHT, SIGNAL_LEFT = 1, 2   # traci.vehicle.getSignals의 방향지시등 비트
LANE_RELATIONS = {2: "left2", 1: "left", 0: "same", -1: "right", -2: "right2", None: "other"}


# 경로 정보 관측(ROUTE_FEATURES)의 정규화 거리(m)
TL_RANGE = 100.0           # 이보다 먼 신호는 없는 것으로 본다
STOP_RANGE = 300.0         # 정차할 정류장까지 거리. 정류장 차로로 미리 옮길 수 있게 멀리 본다
STOP_MARGIN = 1.0          # 버스 앞범퍼를 정류장 끝 이만큼 앞에 세운다
CROSSING_RANGE = 100.0     # 경로 위 다음 횡단보도까지 거리. 버스(감속 2m/s²)가 50km/h에서 서려면 약 60m가 필요하다
LIMIT_RANGE = 100.0        # 앞쪽 허용 속도를 보는 거리
RED_STATES = "rRu"         # 정지해야 하는 신호 문자(SUMO 신호 상태). 관측 tl_red와 적색 통과 판정이 같이 쓴다
IN_PATH_CLEARANCE = 0.3    # m. 옆 차로 차의 차체가 버스 차체에 옆으로 이보다 가까우면 버스 앞길에 있는 차로 본다


def _unit(angle_deg):
    """SUMO 각도(북쪽 0도, 시계 방향)를 진행 방향 단위 벡터로 바꾼다."""
    radians = np.deg2rad(angle_deg)
    return float(np.sin(radians)), float(np.cos(radians))


def _relative(ego, other):
    """ego 기준 상대 위치·속도. 종방향은 ego 진행 방향이 +, 횡방향은 ego의 왼쪽이 +다.
    relative_heading도 상대가 ego의 왼쪽으로 돌아 있으면 +다.

    ego와 other는 (x, y, 속도, 각도)다.
    """
    hx, hy = _unit(ego[3])
    ox, oy = _unit(other[3])
    dx, dy = other[0] - ego[0], other[1] - ego[1]
    dvx, dvy = other[2] * ox - ego[2] * hx, other[2] * oy - ego[2] * hy
    return {"relative_longitudinal_position": dx * hx + dy * hy,
            "relative_lateral_position": -dx * hy + dy * hx,
            "relative_longitudinal_speed": dvx * hx + dvy * hy,
            "relative_lateral_speed": -dvx * hy + dvy * hx,
            "relative_heading": (ego[3] - other[3] + 180.0) % 360.0 - 180.0}


class SumoHighwayEnv(gym.Env):
    # 클래스 이름은 수업 원본과 같다. 생성자는 원본과 달리 traffic=을 받지 않는다.
    # 배경 교통은 road_builder가 만든 SUMO 경로 파일이 정한다.
    metadata = {"render_modes": ["human"]}

    def __init__(self, cfg_path, road, ego, mdp_sim, mdp_obs, action, reward,
                 gui=False, gui_autostart=True):
        """
        Args:
            cfg_path: road_builder.build()가 만든 .sumocfg 경로
            road:     env.road_config.ROAD
            ego:      env.road_config.EGO (가속/감속 한계가 여기서 옴)
            mdp_sim:  env.mdp_config.SIMULATION
            mdp_obs:  env.mdp_config.OBSERVATION
            action:   env.mdp_config.ACTION
            reward:   env.mdp_config.REWARD
            gui:      True면 sumo-gui(화면 있는 버전)로 실행
            gui_autostart: (gui=True일 때만 의미)
                True  → 창이 뜨면 시뮬레이션이 자동으로 시작
                False → 사용자가 ▶(플레이) 버튼을 누를 때까지 대기.
                        sumo-gui는 --start 옵션이 없으면 TraCI의
                        simulationStep() 호출도 플레이 전까지 블록되므로,
                        에피소드 전체가 "▶를 눌러야 시작"되는 효과가 난다.
        """
        super().__init__()
        self.cfg_path = cfg_path
        self.gui = gui
        self.gui_autostart = gui_autostart
        self.ego_route = ego["route_id"]
        self.base_seed = int(road["seed"])
        self._seed_initialized = False
        self.ego_speed_mode = int(ego["speed_mode"])
        self.ego_lane_change_mode = int(ego["lane_change_mode"])
        self._lane_geometry = {}
        self._segment_cache = {}

        # ----- 도로/차량 설정에서 필요한 값들 -----
        self.vmax = float(road["speed_limit"])   # 속도 정규화 기준
        self.ego_depart_lane = str(ego["depart_lane"])
        self.ego_depart_speed = str(ego["depart_speed"])
        # ---- 차선변경 안전 게이트 설정 (mdp_config.ACTION["lane_change_safety"]) ----
        lcs = action["lane_change_safety"]
        self.lc_safety_enabled = bool(lcs["enabled"])
        self.lc_front_min = float(lcs["min_front_gap"])     # m
        self.lc_rear_min = float(lcs["min_rear_gap"])       # m
        self.lc_front_tau = float(lcs["front_tau"])         # s
        self.lc_rear_tau = float(lcs["rear_tau"])           # s
        self.lc_spawn_protect = float(lcs["spawn_protect"]) # m
        self.lc_horizon = float(lcs["approach_horizon"])    # s
        # 행동 a ∈ [-1,1] 을 실제 가속도로 변환할 때 쓰는 스케일:
        #   a = +1 → +max_accel (풀 액셀),  a = -1 → -max_decel (풀 브레이크)
        self.max_accel = float(ego["accel"])   # m/s^2
        self.max_decel = float(ego["decel"])   # m/s^2 (양수로 저장, 부호는 계산 시)
        self.ego_vclass = str(ego["vclass"])
        # 승차감 한계(버스). 없는 제원이면 위반 여부를 기록하지 않는다.
        self.comfort_limits = ego.get("comfort")
        self._lane_usable = {}
        network = road["network"]
        self.route_map = RouteMap(os.path.join(os.path.dirname(cfg_path), "network.net.xml"), self.ego_route,
                                  network["routes"][self.ego_route], network.get("bus_stops", []), self.ego_vclass,
                                  (self.comfort_limits or {}).get("lateral_acceleration_limit"),
                                  float(ego.get("length", 5.0)))

        # ----- MDP 설정 -----
        self.step_length = float(mdp_sim["step_length"])
        self.decision_interval = float(mdp_sim["decision_interval"])
        # 행동 한 번을 SUMO 계산 몇 번 동안 유지하는지. 결정 시각이 계산 시각과 맞아야 한다.
        self.substeps = round(self.decision_interval / self.step_length)
        if self.substeps < 1 or abs(self.substeps * self.step_length
                                    - self.decision_interval) > 1e-9:
            raise ValueError(
                f"decision_interval({self.decision_interval:g}초)은 "
                f"step_length({self.step_length:g}초)의 정수배여야 합니다.")
        self.max_steps = int(mdp_sim["max_steps"])
        self.warmup_steps = int(mdp_sim["warmup_steps"])
        self.warmup_jitter_steps = int(mdp_sim.get("warmup_jitter_steps", 0))
        self.state_radius = float(mdp_sim["state_radius"])
        self.ego_insert_timeout = float(mdp_sim["ego_insert_timeout"])
        self.stuck_timeout = float(mdp_sim.get("stuck_timeout", 200.0))
        self.stuck_progress = float(mdp_sim.get("stuck_progress_m", 5.0))
        self.visibility = float(mdp_obs["visibility"])   # 관측 거리 W
        # 차로 연결성을 보는 거리. 없으면 수업 원본처럼 W다.
        self.connectivity_range = float(mdp_obs.get("connectivity_range", self.visibility))
        # 밀도 정규화용 "차량 1대가 점유하는 길이" (차체 + 최소 간격)
        self.density_veh_len = float(mdp_obs.get("density_veh_length", 7.5))
        self.act_dim = int(action["dim"])
        self.accel_index = int(action.get("accel_index", 0))
        self.lane_change_index = int(action.get("lane_change_index", 1))
        self.lane_change_threshold = float(action.get("lane_change_threshold", 0.5))
        self.lane_change_duration = float(action.get("lane_change_duration", 1.0))
        self.rw = reward

        # GUI는 ego를 따라가며 가까이 보이도록 기본값을 둔다.
        self.gui_zoom = float(mdp_sim["gui_zoom"])
        self.gui_delay = float(mdp_sim["gui_delay"])
        self.gui_track_ego = bool(mdp_sim["gui_track_ego"])

        self.ego_id = "ego"        # SUMO 안에서 ego 차량을 식별하는 ID
        self.step_count = 0
        self._started = False      # SUMO 프로세스가 떠 있는지 여부

        # Gymnasium 행동 공간:
        #   action[0] = 종방향 가감속 raw ∈ [-1,1]
        #   action[1] = 차선변경 raw ∈ [-1,1], 환경 내부에서 {-1,0,+1}로 양자화
        # PPO의 Gaussian 정책과 호환하기 위해 Box를 유지한다. 요청한 차선변경은
        # info["lane_change"], 그 명령의 실행 여부는 info["lane_change_applied"](수업 원본과
        # 같은 뜻), 같은 edge에서 확인된 실제 이동은 info["lane_change_actual"]로 구분한다.
        self.action_space = spaces.Box(low=-1.0, high=1.0,
                                       shape=(self.act_dim,), dtype=np.float32)
        # 관측: 31차원, 범위 [-1,1]. 레이아웃은 env/mdp_config.py 주석 참고.
        #   [0] 내 속도, [1..20] 5개 차선 × (선행/후행 상대거리·상대속도),
        #   [21..25] 경로 기준 연결성, [26..30] 전방 밀도.
        #   시야 차선: 좌2, 좌1, 현재, 우1, 우2 (5개) — 3차선 도로에서
        #   맨 왼쪽에 있어도 오프셋 -2로 맨 오른쪽 차선까지 보인다.
        self.obs_offsets = (+2, +1, 0, -1, -2)
        n_lanes_view = len(self.obs_offsets)
        self.obs_dim = 1 + n_lanes_view * 4 + n_lanes_view + n_lanes_view
        # 실제 버스가 알 수 있는 경로 정보(신호, 정류장, 횡단보도, 제한속도, 가속도)를 뒤에 붙인다.
        # 31차원만으로는 적색 신호와 정류장을 볼 수 없다. 어느 차로로 가라는 값은 넣지 않는다(수업 규칙).
        self.route_features = bool(mdp_obs.get("route_features", True))
        if self.route_features:
            self.obs_dim += len(ROUTE_FEATURES)
        service = ego.get("service") or {}
        wanted = set(service.get("stops", ()))
        self.service_stops = [stop for stop in self.route_map.stops if stop[2] in wanted]
        self.dwell_s = float(service.get("dwell_s", 0.0))
        self.ego_length = float(ego.get("length", 5.0))
        self._crossing_ends = {}
        # 행동의 가속도는 결정 구간 동안 지금 가속도에서 명령 가속도로 곧게 바뀐다(구동계의 반응).
        # 결정마다 가속도가 계단처럼 뛰면 0.1초 사이 변화가 jerk 한계(2m/s³)를 늘 넘는다.
        self.accel_ramp = bool(action.get("accel_ramp", True))
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(self.obs_dim,),
                                            dtype=np.float32)

    # ==================================================================
    # SUMO 프로세스 시작
    # ==================================================================
    def _start_sumo(self):
        # gui=True면 화면이 있는 sumo-gui, 아니면 화면 없는 sumo (학습용, 훨씬 빠름)
        binary = checkBinary("sumo-gui" if self.gui else "sumo")
        # 계산 간격과 충돌 처리(warn과 정지, 교차로 포함)는 scenario.sumocfg에 있다.
        # 경고(비상 제동, 충돌)는 콘솔 대신 실행 폴더의 sumo_warnings/<sumo_seed>.log에 남긴다.
        # 수집 워커들이 실행 폴더를 함께 쓰므로 에피소드마다 다른 파일이다. reset의 info와 수집 기록에 sumo_seed가 있다.
        warnings_dir = os.path.join(os.path.dirname(self.cfg_path), "sumo_warnings")
        os.makedirs(warnings_dir, exist_ok=True)
        cmd = [
            binary, "-c", self.cfg_path,
            "--no-warnings", "true",   # 콘솔 경고 끄기
            "--error-log", os.path.join(warnings_dir, f"{self.sumo_seed}.log"),
            "--no-step-log", "true",   # 스텝 로그 끄기
            # Gymnasium의 RNG에서 뽑아 reset(seed)의 재현성과
            # reset(None)의 연속적인 새 교통 패턴을 함께 보장한다.
            "--seed", str(self.sumo_seed),
        ]
        if self.gui:
            cmd += ["--quit-on-end"]      # 에피소드 종료 시 창 정리
            cmd += ["--delay", str(self.gui_delay)]
            if self.gui_autostart:
                cmd += ["--start"]        # 자동 재생
            # --start가 없으면: 창이 뜬 뒤 ▶(플레이)를 눌러야 진행된다.
            #   매 에피소드가 reset()에서 SUMO를 새로 띄우므로,
            #   에피소드마다 ▶를 눌러 시작하게 된다.
        traci.start(cmd)   # SUMO를 자식 프로세스로 띄우고 소켓 연결
        self._started = True

    # ==================================================================
    # 에피소드 초기화
    # ==================================================================
    def reset(self, seed=None, options=None):
        # 첫 reset()만 지도 설정의 seed를 사용한다. 이후 reset(None)은
        # RNG를 되감지 않아 학습 에피소드마다 같은 교통이 반복되지 않는다.
        if seed is None and not self._seed_initialized:
            seed = self.base_seed
        super().reset(seed=seed)
        self._seed_initialized = True
        self.sumo_seed = int(self.np_random.integers(0, 2**31 - 1))
        self._lane_geometry.clear()
        self._segment_cache.clear()

        # SUMO 상태를 완전히 초기화하는 가장 확실한 방법은 껐다 켜는 것.
        if self._started:
            traci.close()
            self._started = False
        self._start_sumo()

        # 배경 flow를 먼저 흘려 교차로 대기열이 생긴 뒤에 ego를 투입한다.
        warmup_steps = self.warmup_steps + int(self.np_random.integers(0, self.warmup_jitter_steps + 1))
        for _ in range(warmup_steps):
            traci.simulationStep()

        # ego 차량 투입 — 반드시 "도로 시작점(0m)"에서 출발
        # 순환 노선이라 경로 끝에서 멈추지 않는다. 기본값이면 SUMO가 마지막 도로에서 정지하려고 감속한다.
        traci.vehicle.add(self.ego_id, self.ego_route, typeID="ego_type",
                          departLane=self.ego_depart_lane,
                          departPos="0",
                          departSpeed=self.ego_depart_speed,
                          arrivalSpeed="current")

        # 수업 원본과 같다. speedMode 0은 SUMO의 안전 속도·우선권·적색 정지를 모두 끄고, laneChangeMode 0은
        # SUMO의 자율 차선변경을 끄고 TraCI 요청을 남의 간격과 무관하게 실행한다(env/road_config.py).
        traci.vehicle.setSpeedMode(self.ego_id, self.ego_speed_mode)
        traci.vehicle.setLaneChangeMode(self.ego_id, self.ego_lane_change_mode)

        traci.simulationStep()  # ego가 실제로 도로에 나타나도록 1스텝 진행

        # ── ego 삽입 보증 ──
        # 도로 입구는 flow가 차를 스폰하는 자리라, 시작 차선 입구가 잠시
        # 막혀 있으면 SUMO가 삽입을 "지연"시킨다. 그 경우 ego 없이
        # 에피소드가 진행되어 즉시 종료되는 버그가 되므로, 삽입될 때까지
        # 설정한 물리 시간만큼 기다린다. 계산 간격을 바꿔도 대기 한도는 같다.
        # 혼잡하다는 이유로 차량을 강제 삽입하거나 주변 차량을 삭제하지 않는다.
        _wait = 0
        wait_limit = int(np.ceil(self.ego_insert_timeout / self.step_length))
        while self.ego_id not in traci.vehicle.getIDList() and _wait < wait_limit:
            traci.simulationStep()
            _wait += 1
        if self.ego_id not in traci.vehicle.getIDList():
            raise RuntimeError(
                f"ego 삽입 실패: 도로 입구가 {self.ego_insert_timeout:g}초 이상 막혀 있습니다. "
                "진입부 대기열과 수요 경로를 확인하세요.")

        # GUI 기본 시점: ego를 화면 중앙에서 추적하고 충분히 가까이 확대한다.
        if self.gui and self.gui_track_ego:
            try:
                view_ids = traci.gui.getIDList()
                if view_ids:
                    # tracking을 먼저 켜고 zoom을 마지막에 적용해야 SUMO가
                    # 차량 추적을 시작하면서 viewport를 다시 덮어쓰지 않는다.
                    traci.gui.trackVehicle(view_ids[0], self.ego_id)
                    traci.gui.setZoom(view_ids[0], self.gui_zoom)
            except traci.TraCIException:
                pass

        self.step_count = 0
        self._lane_change, self._lane_request_until, self._centring = None, -1.0, False
        self._acceleration, self._jerk = 0.0, 0.0
        self._service_next, self._dwell_left = 0, None
        self.stops_served, self.stops_missed = 0, 0
        self._speed = float(traci.vehicle.getSpeed(self.ego_id)) if self.ego_id in traci.vehicle.getIDList() else 0.0
        self._progress_mark = (0, float(traci.vehicle.getDistance(self.ego_id)))
        return self._get_obs(), {"sumo_seed": self.sumo_seed,
                                 "ego_insert_wait_seconds": (1 + _wait) * self.step_length,
                                 "warmup_seconds": warmup_steps * self.step_length,
                                 "background_vehicles_at_reset": len(traci.vehicle.getIDList()) - 1}

    # ==================================================================
    # 관측 만들기 (레이아웃은 env/mdp_config.py 주석 참고)
    #
    # 핵심 설계: 특정 도로에 대한 하드코딩 없이, 매 스텝 SUMO 네트워크를
    # 실시간 조회해서 "지금 내 주변"을 계산한다. 지도별 도로 이름을 이
    # 코드에 넣지 않지만, 교차 접근 교통을 모두 관측한다는 뜻은 아니다.
    #
    # 사용하는 TraCI API:
    #   getLeader / getFollower : 같은 차선의 앞/뒤 차량 (edge 경계도 넘어 탐색)
    #   getNeighbors(mode)      : 옆 차선의 앞/뒤 차량.
    #       mode 비트: bit0 = 0:왼쪽 / 1:오른쪽,  bit1 = 0:후행 / 1:선행
    #   lane.getLinks           : 차선이 다음 edge로 이어지는지 (연결성 판단)
    # ==================================================================

    # (차선 오프셋, 선행차 getNeighbors mode, 후행차 mode)
    #   오프셋 +1 = 왼쪽 차선 (SUMO는 차선 index가 왼쪽으로 갈수록 커짐)
    _NEIGHBOR_MODES = {+1: (0b10, 0b00),   # 왼쪽: 선행=2, 후행=0
                       -1: (0b11, 0b01)}   # 오른쪽: 선행=3, 후행=1

    def _offset_lane(self, offset):
        """현재 위치에서 비교 가능한 차선만 반환한다.

        교차로 내부의 서로 다른 연결 차로는 '옆 차선'이 아닐 수 있다.
        이때 현재 진행 차로만 유효하게 두고, 교차 접근 교통은 후속 관측
        버전에서 별도로 추가한다. 내부 구간을 빈 도로로 처리하지는 않는다.
        """
        edge = traci.vehicle.getRoadID(self.ego_id)
        if edge.startswith(":"):
            return traci.vehicle.getLaneID(self.ego_id) if offset == 0 else None
        index = traci.vehicle.getLaneIndex(self.ego_id) + offset
        if 0 <= index < traci.edge.getLaneNumber(edge) and self._usable(f"{edge}_{index}"):
            return f"{edge}_{index}"
        return None

    def _usable(self, lane):
        """ego가 다닐 수 있는 차로인가. 인도와 다른 차종 전용 차로는 없는 차로로 본다."""
        if lane not in self._lane_usable:
            allowed = traci.lane.getAllowed(lane)
            self._lane_usable[lane] = not allowed or self.ego_vclass in allowed
        return self._lane_usable[lane]

    def _geometry(self, lane):
        """변하지 않는 길이·연결만 캐시한다."""
        if lane not in self._lane_geometry:
            # getLinks: (다음 일반 lane, 우선권, 개방, 충돌차, 내부 lane, ...)
            links = tuple((link[0], link[4]) for link in traci.lane.getLinks(lane))
            self._lane_geometry[lane] = (traci.lane.getLength(lane), links)
        return self._lane_geometry[lane]

    def _forward_segments(self, offset, reach=None):
        """시야 안 경로를 (lane, 시작위치, 끝위치, ego로부터 거리)로 분해.

        '첫 번째 연결'은 좌회전·직진이 섞인 도시 교차로에서 잘못된 길을
        가리킬 수 있다. ego route의 다음 edge로 가는 연결만 따라간다.
        내부 차로도 거리와 차량 수에 포함한다. 여러 유효 연결이 있으면
        현재 차선 번호와 가까운 연결을 고른다(완전한 경로 계획은 아님).
        reach는 보는 거리(m)이고 기본은 관측 거리다.
        """
        reach = self.visibility if reach is None else reach
        if (offset, reach) in self._segment_cache:
            return self._segment_cache[offset, reach]
        lane = self._offset_lane(offset)
        segments = []
        if lane is None:
            return segments
        route = traci.vehicle.getRoute(self.ego_id)
        route_index = traci.vehicle.getRouteIndex(self.ego_id)
        position = traci.vehicle.getLanePosition(self.ego_id)
        distance = 0.0
        visited = set()
        while lane and distance < reach and (lane, route_index) not in visited:
            visited.add((lane, route_index))
            length, links = self._geometry(lane)
            start = min(max(position, 0.0), length)
            end = min(length, start + reach - distance)
            segments.append((lane, start, end, distance))
            distance += end - start
            if distance >= reach or route_index + 1 >= len(route):
                break
            target_edge = route[route_index + 1]
            candidates = [link for link in links
                          if traci.lane.getEdgeID(link[0]) == target_edge]
            if not candidates:
                break
            current_index = int(lane.rsplit("_", 1)[1])
            next_lane, via = min(candidates, key=lambda link:
                                 (abs(int(link[0].rsplit("_", 1)[1]) - current_index),
                                  link[0]))
            lane = via or next_lane
            if not lane.startswith(":"):
                route_index += 1
            position = 0.0
        self._segment_cache[offset, reach] = segments
        return segments

    def _neighbor(self, offset: int, leader: bool):
        """선행/후행의 (간격 m, 속도)를 반환. 현재 차선은 내부 연결도 탐색.

        SUMO getFollower는 합류부에서 가장 위험한 후행차를 선택한다.
        옆 차선은 SUMO 이웃 캐시와 직접 스캔 중 가까운 차량을 사용한다.
        """
        if offset == 0:
            res = (traci.vehicle.getLeader(self.ego_id, self.visibility) if leader
                   else traci.vehicle.getFollower(self.ego_id, self.visibility))
            side = self._in_path_side_leader() if leader else None
            if not res or not res[0]:
                return side
            vid, gap = res
            if side is not None and side[0] < gap:
                return side
        else:
            if self._offset_lane(offset) is None:
                return None
            front, rear = self._scan_lane_neighbors(offset)
            scan = front if leader else rear
            pairs = ()
            if offset in self._NEIGHBOR_MODES:
                modes = self._NEIGHBOR_MODES[offset]
                pairs = traci.vehicle.getNeighbors(self.ego_id, modes[0 if leader else 1])
            if not pairs:
                if scan is None or scan[0] > self.visibility:
                    return None
                return max(0.0, float(scan[0])), float(scan[1])
            vid, gap = min(pairs, key=lambda pair: pair[1])
            if scan is not None and scan[0] < gap and scan[0] <= self.visibility:
                return max(0.0, float(scan[0])), float(scan[1])
        if gap > self.visibility:
            return None
        # 음수 gap을 '차 없음'으로 바꾸면 겹침/매우 가까운 차량이 사라진다.
        return max(0.0, float(gap)), traci.vehicle.getSpeed(vid)

    def _in_path_side_leader(self):
        """옆 차로에 속하지만 차체가 버스 앞길에 걸친 가장 가까운 앞차의 (간격 m, 속도). 없으면 None.

        SUMO의 getLeader는 차로 소속으로만 앞차를 고른다. 옆 차로에서 넘어오는 중인 차, 두 차로에 걸쳐 선 차, 버스가
        차선변경으로 넘어가는 중인 차로의 차는 버스 차로 소속이 아니라 빠진다. 안전 보조를 끈 버스는 이 차를 스스로
        피해야 하므로 차체가 옆으로 겹치면(IN_PATH_CLEARANCE 안) 현재 차로 앞차로 관측한다. 간격은 getLeader처럼
        버스의 최소 간격(minGap)을 뺀 값이다.
        """
        ego_lane = traci.vehicle.getLaneID(self.ego_id)
        min_gap = traci.vehicle.getMinGap(self.ego_id)
        ego_lateral = traci.vehicle.getLateralLanePosition(self.ego_id)
        ego_half = traci.vehicle.getWidth(self.ego_id) / 2.0
        ego_lane_half = traci.lane.getWidth(ego_lane) / 2.0
        best = None
        for direction in (1, -1):
            # 차로 경계까지 남은 폭. 음수면 그만큼 경계를 넘어가 있다.
            ego_room = ego_lane_half - direction * ego_lateral - ego_half
            for lane, start, end, distance in self._forward_segments(direction):
                lane_half = traci.lane.getWidth(lane) / 2.0
                for vid in traci.lane.getLastStepVehicleIDs(lane):
                    position = traci.vehicle.getLanePosition(vid)
                    if vid == self.ego_id or not start <= position <= end:
                        continue
                    room = (lane_half + direction * traci.vehicle.getLateralLanePosition(vid)
                            - traci.vehicle.getWidth(vid) / 2.0)
                    if room + ego_room < IN_PATH_CLEARANCE:
                        gap = max(0.0, distance + position - start - traci.vehicle.getLength(vid) - min_gap)
                        if gap <= self.visibility and (best is None or gap < best[0]):
                            best = (float(gap), float(traci.vehicle.getSpeed(vid)))
        return best

    def _lane_connectivity(self, offset: int) -> float:
        """경로 기준 연결성: 없음=-1, 보는 거리 끝까지 이어짐=1, 나머지는 거리/connectivity_range.

        도시 교차로에서는 '어딘가로 연결됨'보다 '지정 경로로 이어짐'이
        중요하다. 다른 방향으로만 나가는 차선은 연결된 차선으로 세지 않는다.
        경로의 마지막 도로 끝(목적지)까지 가는 차로는 끊기는 차로가 아니라 1이다.
        """
        try:
            if self._offset_lane(offset) is None:
                return -1.0
            segments = self._forward_segments(offset, self.connectivity_range)
            last_lane, _start, last_end, _distance = segments[-1]
            if (traci.lane.getEdgeID(last_lane) == traci.vehicle.getRoute(self.ego_id)[-1]
                    and last_end >= self._geometry(last_lane)[0] - 1e-6):
                return 1.0
            usable = sum(end - start for _, start, end, _ in segments)
            return float(np.clip(usable / self.connectivity_range, 0.0, 1.0))
        except traci.TraCIException:
            return -1.0  # 조회 불가를 개방된 차선으로 표시하지 않음

    def _lane_density(self, offset: int) -> float:
        """경로상 시야 내 차량 수 / (사용 가능 거리 / 차량 점유 길이).

        내부 차로와 여러 짧은 edge를 함께 센다. 차선이 없거나 지정 경로로
        이어지지 않아 사용 가능한 길이가 0이면 1(진입 불가)로 표시한다.
        """
        try:
            if self._offset_lane(offset) is None:
                return 1.0
            segments = self._forward_segments(offset)
            usable = sum(end - start for _, start, end, _ in segments)
            if usable <= 0:
                return 1.0
            vehicles = set()
            for lane, start, end, _ in segments:
                for vid in traci.lane.getLastStepVehicleIDs(lane):
                    if vid != self.ego_id and start <= traci.vehicle.getLanePosition(vid) <= end:
                        vehicles.add(vid)
            max_count = usable / self.density_veh_len
            return float(np.clip(len(vehicles) / max_count, 0.0, 1.0))
        except traci.TraCIException:
            return 1.0

    def _pose(self, vid):
        x, y = traci.vehicle.getPosition(vid)
        return float(x), float(y), float(traci.vehicle.getSpeed(vid)), float(traci.vehicle.getAngle(vid))

    def get_detected_vehicles(self):
        """ego가 감지한 차량 목록(분석/데이터셋용)을 반환한다.

        두 가지를 합친다. 관측 대상 차선의 차량은 관측과 같은 경로 구간을 따라 찾고 lane_relation과
        route_distance(ego 앞끝 기준 경로상 앞끝 거리 m, 전방 +)를 적는다. 그 밖에 ego에서 직선 거리로
        관측 거리(W) 안에 있는 차량(교차 도로, 반대 방향)은 lane_relation "other", route_distance None이다.
        관측 벡터는 앞의 것만 쓴다. relative_*는 ego 차체 좌표다.
        """
        detected = {}
        if self.ego_id not in traci.vehicle.getIDList():
            return []
        ego = self._pose(self.ego_id)

        def record(vid, distance, offset):
            if vid == self.ego_id or vid in detected or (distance is not None and abs(distance) > self.visibility):
                return
            pose = self._pose(vid)
            signals = traci.vehicle.getSignals(vid)
            detected[vid] = {"id": vid, "x": pose[0], "y": pose[1], **_relative(ego, pose),
                             "route_distance": None if distance is None else float(distance),
                             "lane_relation": LANE_RELATIONS[offset],
                             "left_signal": bool(signals & SIGNAL_LEFT), "right_signal": bool(signals & SIGNAL_RIGHT)}

        try:
            ego_pos = traci.vehicle.getLanePosition(self.ego_id)
            for offset in self.obs_offsets:
                lane = self._offset_lane(offset)
                if lane is None:
                    continue
                for vid in traci.lane.getLastStepVehicleIDs(lane):
                    rel = traci.vehicle.getLanePosition(vid) - ego_pos
                    if rel < 0:
                        record(vid, rel, offset)
                for segment_lane, start, end, distance in self._forward_segments(offset):
                    for vid in traci.lane.getLastStepVehicleIDs(segment_lane):
                        position = traci.vehicle.getLanePosition(vid)
                        if start <= position <= end:
                            record(vid, distance + position - start, offset)
            follower = traci.vehicle.getFollower(self.ego_id, self.visibility)
            if follower and follower[0]:
                vid, gap = follower
                rel = -(gap + traci.vehicle.getMinGap(vid)
                        + traci.vehicle.getLength(self.ego_id))
                record(vid, rel, 0)
            for vid in traci.vehicle.getIDList():
                x, y = traci.vehicle.getPosition(vid)
                if (x - ego[0]) ** 2 + (y - ego[1]) ** 2 <= self.visibility ** 2:
                    record(vid, None, None)
        except traci.TraCIException:
            pass
        return list(detected.values())

    def get_detected_pedestrians(self):
        """ego에서 관측 거리(W) 안에 있는 보행자. 차로와 무관하게 직선 거리로 고른다."""
        if self.ego_id not in traci.vehicle.getIDList():
            return []
        ego = self._pose(self.ego_id)
        detected = []
        for pid in traci.person.getIDList():
            x, y = traci.person.getPosition(pid)
            if (x - ego[0]) ** 2 + (y - ego[1]) ** 2 > self.visibility ** 2:
                continue
            relative = _relative(ego, (float(x), float(y), float(traci.person.getSpeed(pid)),
                                       float(traci.person.getAngle(pid))))
            relative.pop("relative_heading")
            detected.append({"id": pid, **relative})
        return detected

    def _get_obs(self):
        self._segment_cache.clear()
        # ego가 이미 사라졌다면(충돌/도착) 0벡터 반환 — 종료 직전 마지막 관측용
        if self.ego_id not in traci.vehicle.getIDList():
            return np.zeros(self.obs_dim, dtype=np.float32)

        v = traci.vehicle.getSpeed(self.ego_id)
        obs = [v / self.vmax]                      # [0] 내 절대속도

        # ----- 5개 차선(좌2→좌1→현재→우1→우2)의 선행/후행 상대거리·상대속도 -----
        for offset in self.obs_offsets:
            lead = self._neighbor(offset, leader=True)
            if lead is None:
                obs += [1.0, 1.0]                  # 없음: 멀고 + 위협 없음
            else:
                gap, lv = lead
                obs += [gap / self.visibility,
                        float(np.clip((lv - v) / self.vmax, -1.0, 1.0))]

            foll = self._neighbor(offset, leader=False)
            if foll is None:
                obs += [-1.0, -1.0]                # 없음: 멀고 + 위협 없음
            else:
                gap, fv = foll
                obs += [-gap / self.visibility,    # 후행은 음수 부호
                        float(np.clip((fv - v) / self.vmax, -1.0, 1.0))]

        # ----- 5개 차선의 연결성 (같은 순서) -----
        for offset in self.obs_offsets:
            obs.append(self._lane_connectivity(offset))

        # ----- 5개 차선의 전방 차량 밀도 (같은 순서) -----
        for offset in self.obs_offsets:
            obs.append(self._lane_density(offset))

        if self.route_features:
            obs += self._route_observation()
        return np.clip(np.array(obs, dtype=np.float32), -1.0, 1.0)

    # ==================================================================
    # 경로 정보: 신호, 정류장, 횡단보도, 제한속도 (ROUTE_FEATURES 순서)
    # ==================================================================
    def _route_position(self):
        """경로 거리(m). 버스 앞범퍼 기준이다."""
        return self.route_map.position(int(traci.vehicle.getRouteIndex(self.ego_id)),
                                       traci.vehicle.getLaneID(self.ego_id),
                                       float(traci.vehicle.getLanePosition(self.ego_id)))

    def _crossing_occupied(self, crossing_edge):
        """횡단보도 위에 사람이 있으면 참이다. 신호 없는 횡단보도는 양 끝 2.5m 안에서 멈춰 기다리는 사람도 센다.
        신호 있는 횡단보도에서 기다리는 사람은 보행 신호를 기다리는 것이라 차의 진행과 무관하다."""
        if traci.edge.getLastStepPersonIDs(crossing_edge):
            return True
        node = crossing_edge[1:].rsplit("_c", 1)[0]
        if node in self.route_map.signalized_nodes or crossing_edge not in self.route_map.pedestrian_priority:
            return False
        ends = self._crossing_ends.get(crossing_edge)
        if ends is None:
            shape = traci.lane.getShape(crossing_edge + "_0")
            ends = self._crossing_ends[crossing_edge] = (shape[0], shape[-1])
        for pid in traci.person.getIDList():
            if traci.person.getSpeed(pid) > 0.5:
                continue
            px, py = traci.person.getPosition(pid)
            if any((px - ex) ** 2 + (py - ey) ** 2 < 2.5 ** 2 for ex, ey in ends):
                return True
        return False

    def _route_observation(self):
        lane_id = traci.vehicle.getLaneID(self.ego_id)
        position = self._route_position()

        tl = [1.0, 0.0, 0.0, 0.0]
        signals = self._next_tls(lane_id, position)
        if signals and signals[0][2] <= TL_RANGE:
            light = signals[0][3]
            tl = [signals[0][2] / TL_RANGE, float(light in RED_STATES), float(light in "yY"), float(light in "gs")]

        stop = [1.0, 0.0]
        if self._service_next < len(self.service_stops):
            _start, end, _stop_id, _lane = self.service_stops[self._service_next]
            distance = end - STOP_MARGIN - position
            stop[0] = float(np.clip(distance / STOP_RANGE, 0.0, 1.0))
            if self._dwell_left is not None:
                stop[1] = self._dwell_left / self.dwell_s

        crossing = [1.0, 0.0]
        ahead = next(((at, edge) for at, edge in self.route_map.crossings if at >= position), None)
        if ahead and ahead[0] - position <= CROSSING_RANGE:
            crossing = [(ahead[0] - position) / CROSSING_RANGE, float(self._crossing_occupied(ahead[1]))]

        allowed = self._allowed_speed(lane_id)
        limit, limit_distance = self._next_speed_limit(lane_id, allowed)
        return tl + stop + crossing + [allowed / self.vmax, limit / self.vmax, limit_distance / LIMIT_RANGE,
                                       float(np.clip(self._acceleration / 2.0, -1.0, 1.0))]

    def _next_tls(self, lane_id, position):
        """getNextTLS 형식의 다음 신호들. getNextTLS는 경로 끝을 넘어 보지 않아, 순환 노선이 끝나는 마지막 도로 끝
        신호(합정역 J1)가 빠졌다. 버스는 경로 끝에서 서지 않고 그 신호를 지나 계속 달리므로(arrivalSpeed=current)
        경로 끝 차로의 신호를 직접 읽는다."""
        signals = traci.vehicle.getNextTLS(self.ego_id)
        if signals:
            return signals
        links = traci.vehicle.getNextLinks(self.ego_id)
        lane = links[-1][0] if links else lane_id
        while lane.startswith(":"):
            lane = traci.lane.getLinks(lane)[0][0]
        distance = self.route_map.length - position
        return tuple((tls, index, distance, traci.trafficlight.getRedYellowGreenState(tls)[index])
                     for tls, index in self.route_map.end_signals.get(lane, ()))

    def _allowed_speed(self, lane_id):
        """지금 자리의 허용 속도: 차로 제한, 차량 제한, 곡선 제한(횡가속도) 중 가장 낮은 값."""
        return min(traci.lane.getMaxSpeed(lane_id), traci.vehicle.getAllowedSpeed(self.ego_id),
                   self.route_map.curve_speed(lane_id, float(traci.vehicle.getLanePosition(self.ego_id))))

    def _next_speed_limit(self, lane_id, allowed):
        """경로를 따라 LIMIT_RANGE 안에서 가장 낮은 허용 속도와 그 거리. 회전 차로의 속도와 곡선 제한이 여기서 보인다.
        교차로 안에서도 경로의 다음 차로를 따라가 본다(getNextLinks는 교차로 안에서 앞 차로를 주지 않았다)."""
        best, best_distance = allowed, LIMIT_RANGE
        for lane, start, end, distance in self._forward_segments(0, LIMIT_RANGE):
            limits = list(self.route_map.curve_speeds_ahead(lane, start))
            if lane != lane_id:
                limits.insert(0, (start, float(traci.lane.getMaxSpeed(lane))))
            for at, speed in limits:
                if at <= end and speed < best - 1e-6:
                    best, best_distance = speed, distance + at - start
        return best, best_distance

    def _update_service(self):
        """정류장 정차를 진행한다. 버스 몸 전체가 정류장 차로의 정류장 안에서 멈춰 dwell_s초를 채우면 'served',
        채우기 전에 떠나거나 정류장을 지나치면 'missed'다. 정류장이 있는 도로를 벗어나도 지나친 것이다.
        s6처럼 정류장 끝이 도로 끝에 붙어 있으면 교차로 안의 위치가 정류장 끝 + 1m를 넘지 않는다."""
        if self._service_next >= len(self.service_stops):
            return None
        start, end, _stop_id, stop_lane = self.service_stops[self._service_next]
        position = self._route_position()
        lane_id = traci.vehicle.getLaneID(self.ego_id)
        in_zone = lane_id == stop_lane and start + self.ego_length - 1.0 <= position <= end + 1.0
        left_stop_road = position > start and traci.vehicle.getRoadID(self.ego_id) != stop_lane.rsplit("_", 1)[0]
        stopped = self._speed < 0.1
        event = None
        if self._dwell_left is not None:
            if in_zone and stopped:
                self._dwell_left -= self.decision_interval
                if self._dwell_left <= 1e-9:
                    event = "served"
            else:
                event = "missed"
        elif in_zone and stopped:
            self._dwell_left = self.dwell_s
        elif position > end + 1.0 or left_stop_road:
            event = "missed"
        if event:
            self._service_next += 1
            self._dwell_left = None
            if event == "served":
                self.stops_served += 1
            else:
                self.stops_missed += 1
        return event

    # ==================================================================
    # 1 스텝 진행: 행동 적용 → SUMO를 decision_interval만큼 계산 → 보상/종료 판정
    # ==================================================================
    def _quantize_lane_change(self, raw: float) -> int:
        """연속 policy 출력을 실제 차선 명령 {-1,0,+1}로 변환한다.

        +1 = 왼쪽, 0 = 유지, -1 = 오른쪽. SUMO 차선 index가 왼쪽으로
        갈수록 증가하는 규약과 동일해서 해석이 직관적이다.
        """
        raw = float(np.clip(raw, -1.0, 1.0))
        if raw >= self.lane_change_threshold:
            return 1
        if raw <= -self.lane_change_threshold:
            return -1
        return 0

    def _scan_lane_neighbors(self, direction: int):
        """목표 차선의 앞/뒤 차량을 직접 조회해 이웃 캐시의 빈틈을 보완.

        전방은 지정 경로의 내부 차로와 다음 edge까지 함께 살핀다.
        후방 스캔은 현재 edge 범위이며, 인접 차선의 SUMO 이웃 조회와 합친다.
        """
        try:
            lane = self._offset_lane(direction)
            if lane is None:
                return None, None
            my_pos = traci.vehicle.getLanePosition(self.ego_id)
            front, rear = None, None
            for vid in traci.lane.getLastStepVehicleIDs(lane):
                if vid == self.ego_id:
                    continue
                distance = traci.vehicle.getLanePosition(vid) - my_pos
                if distance < 0:
                    gap = -distance - traci.vehicle.getLength(self.ego_id)
                    if rear is None or gap < rear[0]:
                        rear = (gap, traci.vehicle.getSpeed(vid))
            for segment_lane, start, end, distance in self._forward_segments(direction):
                for vid in traci.lane.getLastStepVehicleIDs(segment_lane):
                    if vid == self.ego_id:
                        continue
                    position = traci.vehicle.getLanePosition(vid)
                    if start <= position <= end:
                        gap = distance + position - start - traci.vehicle.getLength(vid)
                        if front is None or gap < front[0]:
                            front = (gap, traci.vehicle.getSpeed(vid))
            return front, rear
        except traci.TraCIException:
            return None, None

    def _lane_change_is_safe(self, direction: int) -> bool:
        """목표 차선으로 지금 이동해도 측면충돌이 나지 않는지 검사한다.

        3중 안전 규칙:
          (0) 입구 보호구간: ego가 도로 시작 spawn_protect(m) 이내면 변경 금지.
              flow가 입구(0m)에서 차를 새로 만드는데, 생성 "직전" 차량은
              어떤 API로도 미리 볼 수 없어 원천적으로 사각지대이기 때문.
          (1) 앞: gap > min_front_gap + front_tau × 내속도
              + 접근 여유: gap > (내속도 − 앞차속도)⁺ × approach_horizon   (내가 더 빠를 때)
          (2) 뒤: gap > min_rear_gap + rear_tau × 뒤차속도
              + 접근 여유: gap > (뒤차속도 − 내속도)⁺ × approach_horizon   (뒤가 더 빠를 때)
        이웃은 getNeighbors 결과와 차선 직접 스캔 결과 중 "더 가까운 쪽"을
        쓴다 (스폰 직후 차량 사각지대 보완).
        """
        # 차선 위치는 edge마다 0으로 돌아간다. 도시의 짧은 edge마다
        # 보호구간을 다시 적용하지 않도록 누적 주행 거리를 사용한다.
        traveled = traci.vehicle.getDistance(self.ego_id)
        if traveled < self.lc_spawn_protect:
            return False

        v = traci.vehicle.getSpeed(self.ego_id)
        scan_front, scan_rear = self._scan_lane_neighbors(direction)

        def closer(a, b):
            if a is None:
                return b
            if b is None:
                return a
            return a if a[0] <= b[0] else b

        lead = closer(self._neighbor(direction, leader=True), scan_front)
        if lead is not None:
            gap, lv = lead
            if gap <= self.lc_front_min + self.lc_front_tau * v:
                return False
            if gap <= max(v - lv, 0.0) * self.lc_horizon:  # 시간여유 내 접촉 위험
                return False

        foll = closer(self._neighbor(direction, leader=False), scan_rear)
        if foll is not None:
            gap, fv = foll
            if gap <= self.lc_rear_min + self.lc_rear_tau * fv:
                return False
            if gap <= max(fv - v, 0.0) * self.lc_horizon:  # 시간여유 내 접촉 위험
                return False
        return True

    def _apply_lane_change(self, lane_change: int) -> bool:
        """가능하고 안전한 차선변경 명령을 보내고, 명령 접수 여부를 반환.

        안전 게이트(_lane_change_is_safe)를 통과하지 못한 명령은 실행하지
        않고 False를 반환한다 → step()에서 "불가능한 명령"과 동일하게
        invalid_action_penalty가 부과된다. 즉 에이전트 입장에서
        "위험한 변경 = 실행도 안 되고 감점"이라, 측면충돌로 즉사하며
        배우는 대신 안전한 타이밍을 고르는 쪽으로 학습이 유도된다.
        게이트를 지난 명령은 SUMO가 옆 차와 무관하게 실행한다(laneChangeMode 0). 옆으로 옮기는 데 몇 초가
        걸리므로 실제 변경 여부는 simulationStep 이후에 별도로 확인하고, 옮기는 동안 게이트를 계속 본다(_steer).
        """
        if self.ego_id not in traci.vehicle.getIDList():
            return False
        # 유지 행동은 새 횡이동 명령이 아니다. 매 결정마다 현재 차로로 다시
        # 명령하면 몇 초에 걸쳐 진행 중인 차선변경이 중간에 취소될 수 있다.
        if lane_change == 0:
            return True
        try:
            edge = traci.vehicle.getRoadID(self.ego_id)
            if edge.startswith(":"):
                return False
            current = traci.vehicle.getLaneIndex(self.ego_id)
            n_lanes = traci.edge.getLaneNumber(edge)
            target = current + lane_change
            if target < 0 or target >= n_lanes or not self._usable(f"{edge}_{target}"):
                return False
            # ★ 측면충돌 방지: 실제 이동 명령(0이 아닌 변경)에만 안전 검사
            if self.lc_safety_enabled and not self._lane_change_is_safe(lane_change):
                return False
            traci.vehicle.changeLane(
                self.ego_id, target, self.lane_change_duration)
            self._lane_change = (traci.vehicle.getLaneID(self.ego_id), lane_change)
            self._lane_request_until = float(traci.simulation.getTime()) + self.lane_change_duration
            self._centring = False
            return True
        except traci.TraCIException:
            return False

    def _steer(self) -> bool:
        """조향. SUMO 계산마다 부른다. 진행 중인 차선변경은 간격 게이트가 깨지면 되돌리고, 차선변경이 끝났는데 차로
        가운데가 아니면 가운데로 돌아간다. 이번 계산에서 차선변경을 되돌렸으면 True다.

        self._lane_change는 게이트를 지났지만 아직 목표 차로에 들어서지 못한 차선변경의 (출발 차로, 방향)이다.
        수업 원본의 차선변경은 한 계산에 끝나 게이트를 명령 때 한 번만 본다. 여기서는 차로 폭 3.2m를 넘는 데 약 1.6초가
        걸리고 SUMO는 그동안 옆 차를 보지 않는다(laneChangeMode 0). 그 사이 옆 차로에 차가 들어오면 그대로 부딪힌다
        (train baseline 2300). 같은 게이트를 넘어가는 동안 계속 보고, 깨지면 제 차로 가운데로 돌아간다.

        laneChangeMode 0에서 SUMO는 차선변경을 차체가 목표 차로에 다 들어간 곳(가운데에서 (차로 폭 - 차 폭)/2
        못 미친 곳)에서 끝내고, 가운데로 맞추는 움직임(LCA_SUBLANE)은 버린다(MSVehicle::Influencer::influenceChangeDecision,
        mySublaneLC == LC_NEVER). 교차로에 들어서며 끊긴 차선변경도 비뚤어진 채 남는다. 맞추는 비트(1024)를 켜면
        SUMO가 옆 차와의 옆 간격을 지키려는 움직임(MSLCM_SL2015::keepLatGap)까지 함께 살아나므로 켜지 않고,
        환경이 changeSublane으로 차로 가운데까지만 옮긴다. 옆 속도와 가속도 한계는 차량 타입(SL2015)의 값이다.
        """
        aborted = False
        moving = abs(traci.vehicle.getLateralSpeed(self.ego_id)) >= 1e-3
        requested = float(traci.simulation.getTime()) < self._lane_request_until
        if self._lane_change is not None:
            origin, direction = self._lane_change
            self._segment_cache.clear()
            if traci.vehicle.getLaneID(self.ego_id) != origin or not (moving or requested):
                self._lane_change = None
            elif self.lc_safety_enabled and not self._lane_change_is_safe(direction):
                self._lane_change, self._centring, aborted = None, True, True
            else:
                return False
            if requested:
                # SUMO에 남은 요청을 지운다. 남겨 두면 가운데로 돌아온 뒤나 다음 도로에서 SUMO가 다시 옮기기 시작하고,
                # 그 차선변경은 게이트가 보지 않는다(train baseline 2300, 되돌린 0.5초 뒤 다시 넘어가 부딪혔다).
                traci.vehicle.changeLane(self.ego_id, traci.vehicle.getLaneIndex(self.ego_id), 0.0)
                self._lane_request_until = -1.0
        lateral = float(traci.vehicle.getLateralLanePosition(self.ego_id))
        if abs(lateral) < 0.01:
            self._centring = False
        elif self._centring or not moving:
            self._centring = True
            traci.vehicle.changeSublane(self.ego_id, -lateral)
        return aborted

    def _stop_line_ahead(self):
        """경로 위 다음 신호의 (신호 id, 연결 번호, 정지선의 경로 거리). 없으면 None."""
        position = self._route_position()
        signals = self._next_tls(traci.vehicle.getLaneID(self.ego_id), position)
        if not signals:
            return None
        tls, index, distance, _light = signals[0]
        return tls, int(index), position + float(distance)

    def _passed_red(self, stop_line, alive: bool, arrived: bool) -> bool:
        """이번 SUMO 계산에서 앞범퍼가 적색 정지선을 넘었는지. stop_line은 계산 직전의 _stop_line_ahead()다.
        경로 끝은 마지막 도로의 정지선이라, 적색에 도착한 것도 적색 통과다."""
        if stop_line is None:
            return False
        tls, index, at = stop_line
        if alive:
            passed = self._route_position() > at + 1e-6
        else:
            passed = arrived and at <= self.route_map.length + 1e-6
        return passed and traci.trafficlight.getRedYellowGreenState(tls)[index] in RED_STATES

    def get_privileged_state(self) -> dict:
        """POMDP 데이터셋용 simulator-side state.

        agent가 실제로 받는 입력은 _get_obs()의 부분관측이고, 이 메서드는
        분석/지도신호용 privileged state만 수집한다. 정책 입력으로 쓰지 않는다.
        주변 차량과 보행자는 ego에서 state_radius 안에 있는 것만 적는다.
        """
        state = {"sim_time": float(traci.simulation.getTime()),
                 "ego": {"id": self.ego_id, "alive": False}, "vehicles": [], "pedestrians": [],
                 "navigation": None, "traffic_light": None}
        if self.ego_id not in traci.vehicle.getIDList():
            return state
        x, y, speed, heading = self._pose(self.ego_id)
        road_id = traci.vehicle.getRoadID(self.ego_id)
        lane_id = traci.vehicle.getLaneID(self.ego_id)
        lane_position = float(traci.vehicle.getLanePosition(self.ego_id))
        state["ego"] = {
            "id": self.ego_id, "alive": True, "x": x, "y": y, "speed": speed,
            "acceleration": self._acceleration, "jerk": self._jerk, "heading": heading,
            "road_id": road_id, "lane_id": lane_id, "lane_index": int(traci.vehicle.getLaneIndex(self.ego_id)),
            "lane_position": lane_position,
            "lateral_position": float(traci.vehicle.getLateralLanePosition(self.ego_id)),
            "lateral_speed": float(traci.vehicle.getLateralSpeed(self.ego_id)),
            "signals": int(traci.vehicle.getSignals(self.ego_id)),
        }
        radius2 = self.state_radius ** 2
        for vid in traci.vehicle.getIDList():
            if vid == self.ego_id:
                continue
            vx, vy = traci.vehicle.getPosition(vid)
            if (vx - x) ** 2 + (vy - y) ** 2 > radius2:
                continue
            vx, vy, vspeed, vheading = self._pose(vid)
            state["vehicles"].append({
                "id": vid, "x": vx, "y": vy, "speed": vspeed, "heading": vheading,
                "road_id": traci.vehicle.getRoadID(vid), "lane_id": traci.vehicle.getLaneID(vid),
                "lane_index": int(traci.vehicle.getLaneIndex(vid)),
                "lane_position": float(traci.vehicle.getLanePosition(vid)),
                "lateral_position": float(traci.vehicle.getLateralLanePosition(vid)),
                "lateral_speed": float(traci.vehicle.getLateralSpeed(vid)),
                "signals": int(traci.vehicle.getSignals(vid)),
            })
        for pid in traci.person.getIDList():
            px, py = traci.person.getPosition(pid)
            if (px - x) ** 2 + (py - y) ** 2 > radius2:
                continue
            state["pedestrians"].append({
                "id": pid, "x": float(px), "y": float(py), "speed": float(traci.person.getSpeed(pid)),
                "heading": float(traci.person.getAngle(pid)), "road_id": traci.person.getRoadID(pid)})

        inside = road_id.startswith(":")
        edge_index = int(traci.vehicle.getRouteIndex(self.ego_id))
        route = self.route_map
        position = route.position(edge_index, lane_id, lane_position)
        ahead = route.ahead(position)
        state["navigation"] = {
            "route_id": route.route_id, "route_edge_index": edge_index,
            "drivable_lane_ids": [lane_id] if inside else route.drivable_lane_ids[edge_index],
            "target_lane_ids": [lane_id] if inside else route.target_lane_ids[edge_index],
            **ahead,
        }
        signals = self._next_tls(lane_id, position)
        if signals:
            tls_id, link_index, distance, light = signals[0]
            state["traffic_light"] = {"id": tls_id, "link_index": int(link_index), "state": light,
                                      "distance_to_stop_line": float(distance)}
        return state

    def step(self, action):
        # action = [가감속 raw, 차선변경 raw]
        action_arr = np.asarray(action, dtype=np.float32).reshape(-1)
        if action_arr.size < self.act_dim:
            raise ValueError(
                f"action은 {self.act_dim}차원이어야 합니다. 받은 shape={np.asarray(action).shape}")

        accel_raw = float(np.clip(action_arr[self.accel_index], -1.0, 1.0))
        lane_raw = float(np.clip(action_arr[self.lane_change_index], -1.0, 1.0))
        lane_change = self._quantize_lane_change(lane_raw)
        lane_change_applied = False
        lane_before = None
        edge_before = None
        requested_speed = None

        # ----- (1) 종방향 + 횡방향 행동 적용 -----
        if self.ego_id in traci.vehicle.getIDList():
            v = traci.vehicle.getSpeed(self.ego_id)
            lane_before = int(traci.vehicle.getLaneIndex(self.ego_id))
            edge_before = traci.vehicle.getRoadID(self.ego_id)

            accel_cmd = (accel_raw * self.max_accel
                         if accel_raw >= 0.0 else accel_raw * self.max_decel)
            start = self._acceleration if self.accel_ramp else accel_cmd
            ramp = [start + (accel_cmd - start) * (k + 1) / self.substeps for k in range(self.substeps)]
            targets = np.cumsum(ramp) * self.step_length + v
            # 수업 원본처럼 차량·도로의 최고 속도(vmax)로만 자른다. 차로 제한속도와 곡선 제한은 관측(speed_limit,
            # next_speed_limit*)에 있고, 지키는 것은 정책의 일이다.
            allowed = self.vmax
            requested_speed = float(np.clip(targets[-1], 0.0, allowed))
            lane_change_applied = self._apply_lane_change(lane_change)

        # ----- (2) 행동을 decision_interval 동안 유지: SUMO를 substeps번 계산 -----
        # 사건은 계산마다 모은다. ego 사건이 나면 남은 계산을 하지 않는다.
        collision_ids, teleporting_ids = (), ()
        background_collisions = 0
        vehicle_collision = pedestrian_collision = red_light = lane_change_aborted = False
        peak_acceleration = peak_jerk = 0.0
        for substep in range(self.substeps):
            stop_line = None
            if requested_speed is not None:
                # 가감속 행동은 결정 구간 내내 같은 가속도다. 목표 속도를 한 번에 주면 SUMO가 첫 계산에서
                # 최대 가속으로 따라붙고 나머지는 등속이라, 가속도가 톱니처럼 튀어 jerk가 행동과 무관해진다.
                traci.vehicle.setSpeed(self.ego_id, float(np.clip(targets[substep], 0.0, allowed)))
                lane_change_aborted = self._steer() or lane_change_aborted
                stop_line = self._stop_line_ahead()
            traci.simulationStep()
            if self.ego_id in traci.vehicle.getIDList():
                # 가속도는 속도 변화로 잰다. SUMO가 적색 정지선 앞에서 속도를 한 계산 만에 0으로 만들면
                # getAcceleration()은 0을 돌려줘, 승객이 겪는 급정지가 기록에서 빠진다.
                speed = float(traci.vehicle.getSpeed(self.ego_id))
                acceleration = (speed - self._speed) / self.step_length
                self._speed = speed
                self._jerk = (acceleration - self._acceleration) / self.step_length
                self._acceleration = acceleration
                peak_acceleration = max(peak_acceleration, abs(acceleration))
                peak_jerk = max(peak_jerk, abs(self._jerk))
            collision_ids += tuple(traci.simulation.getCollidingVehiclesIDList())
            persons = set(traci.person.getIDList())
            for collision in traci.simulation.getCollisions():
                pair = (collision.collider, collision.victim)
                if self.ego_id not in pair:
                    background_collisions += 1
                elif persons.intersection(pair):
                    pedestrian_collision = True
                else:
                    vehicle_collision = True
            teleporting_ids += tuple(traci.simulation.getStartingTeleportIDList())
            arrived = self.ego_id in traci.simulation.getArrivedIDList()
            alive = self.ego_id in traci.vehicle.getIDList()
            red_light = red_light or self._passed_red(stop_line, alive, arrived)
            if (arrived or not alive or vehicle_collision or pedestrian_collision or red_light
                    or self.ego_id in teleporting_ids):
                break
        self.step_count += 1

        # ----- (3) 이벤트/관측 (결정 끝에서 한 번) -----
        # 보행자를 친 것도 ego 충돌이다. SUMO의 충돌 차량 목록은 차끼리의 충돌만 담아 getCollisions로 센다.
        collided = vehicle_collision or pedestrian_collision
        teleported = self.ego_id in teleporting_ids
        lane_after = int(traci.vehicle.getLaneIndex(self.ego_id)) if alive else None
        edge_after = traci.vehicle.getRoadID(self.ego_id) if alive else None
        # edge 전환은 차선 번호 자체가 바뀔 수 있어 변경 횟수로 세면 안 된다.
        # 경계를 넘는 스텝은 확인 불가로 남기고 같은 edge의 실제 이동만 센다.
        lane_change_observable = bool(alive and edge_before == edge_after
                                      and not edge_after.startswith(":"))
        actual_lane_change = ((lane_after - lane_before) if lane_change_observable else 0)

        service_event = self._update_service() if alive and not collided and self.route_features else None
        obs = self._get_obs()
        terminated, truncated = False, False
        ego_speed, ego_gap = None, None
        termination_reason = None

        # ----- (4) 기존 보상 유지 -----
        if collided:
            reward = -float(self.rw["collision_penalty"])
            terminated = True
            termination_reason = "pedestrian_collision" if pedestrian_collision else "collision"
        elif red_light:
            reward = -float(self.rw.get("red_light_penalty", self.rw["collision_penalty"]))
            terminated = True
            termination_reason = "red_light"
        elif arrived:
            reward = float(self.rw["arrival_bonus"])
            terminated = True
            termination_reason = "arrived"
        elif teleported:
            reward = 0.0
            terminated = True
            termination_reason = "teleport"
        elif not alive:
            reward = 0.0
            terminated = True
            termination_reason = "missing"
        elif self._stalled():
            reward = 0.0
            terminated = True
            termination_reason = "stuck"
        else:
            ego_speed = traci.vehicle.getSpeed(self.ego_id)
            same_lane_leader = self._neighbor(0, leader=True)
            if same_lane_leader is not None:
                ego_gap = same_lane_leader[0]
            reward = float(self.rw["speed_weight"]) * ego_speed / self.vmax
            if (ego_gap is not None and
                    ego_gap < float(self.rw["close_gap_threshold"])):
                reward -= float(self.rw["close_gap_penalty"])

            # 도시 기본값은 0. 교차로 대기도 '느린 앞차'로 보이므로 정당한
            # 대기를 구분하기 전까지는 정체 페널티를 끈다.
            bp = float(self.rw.get("blocked_penalty", 0.0))
            if (bp > 0.0 and ego_gap is not None
                    and ego_gap < float(self.rw.get("blocked_gap", 20.0))
                    and ego_speed < float(self.rw.get("blocked_speed_frac", 0.7))
                        * self.vmax):
                reward -= bp

            # ---- 차선변경 보상 항 ----
            # lane_change        : 이번 스텝에 policy가 낸 양자화 명령 (-1/0/+1)
            # lane_change_applied: 그 명령이 실행됐는지 (유지 명령은 항상 True)
            # actual_lane_change : 같은 edge에서 확인된 실제 차선 이동. SL2015의 차선변경은
            #                      여러 스텝에 걸치므로 감점은 명령이 아니라 실제 이동마다 준다.
            if actual_lane_change != 0:
                # 실행된 변경마다 소액 감점 → 이득 없는 지그재그 억제.
                # 변경 덕에 속도 보상이 그 이상 늘면 순이득이므로,
                # "필요할 때만 바꾸는" 정책으로 수렴하게 만드는 항이다.
                reward -= float(self.rw.get("lane_change_penalty", 0.0))
            elif lane_change != 0 and not lane_change_applied:
                # 명령은 냈는데 실행 불가(차선 없음/교차로)였던 경우 감점
                # → 의미 없는 행동 출력을 줄이는 정칙화 (rl_action_penalty 대응)
                reward -= float(self.rw.get("invalid_action_penalty", 0.0))

            # ---- 버스 운행 항(기본값 0) ----
            if service_event == "served":
                reward += float(self.rw.get("stop_served_bonus", 0.0))
            elif service_event == "missed":
                reward -= float(self.rw.get("stop_missed_penalty", 0.0))
            if abs(traci.vehicle.getSpeed(self.ego_id) - requested_speed) > 1e-3:
                reward -= float(self.rw.get("intervention_penalty", 0.0))
            limits = self.comfort_limits
            if limits and (peak_acceleration > limits["acceleration_limit"] or peak_jerk > limits["jerk_limit"]):
                reward -= float(self.rw.get("comfort_penalty", 0.0))

        if self.step_count >= self.max_steps and not terminated:
            truncated = True
            termination_reason = "time_limit"

        limits = self.comfort_limits
        comfort = {
            # 이 결정 구간(decision_interval)의 SUMO 계산마다 잰 가속도와 jerk의 최댓값(절댓값)
            "peak_acceleration": peak_acceleration, "peak_jerk": peak_jerk,
            "acceleration_violation": peak_acceleration > limits["acceleration_limit"] if limits else None,
            "jerk_violation": peak_jerk > limits["jerk_limit"] if limits else None,
        }
        info = {
            "comfort": comfort,
            "collided": collided,
            "pedestrian_collision": pedestrian_collision,
            "red_light_violation": red_light,
            "simulation_collided": bool(collision_ids),
            # ego가 끼지 않은 새 충돌 쌍(차·보행자 포함). 부딪힌 차는 지워지지 않고 그 자리에 선다.
            "background_collisions": background_collisions,
            "collision_ids": collision_ids,
            "teleported": teleported,
            "teleporting_ids": teleporting_ids,
            "arrived": arrived,
            "termination_reason": termination_reason,
            "sumo_seed": self.sumo_seed,
            "speed": ego_speed,
            "gap": ego_gap,
            "accel_raw": accel_raw,
            "lane_change_raw": lane_raw,
            "lane_change": lane_change,
            "lane_change_applied": lane_change_applied,
            # 게이트를 지나 시작한 차선변경을 이 판단 동안 환경이 되돌렸는지(옮기는 사이 간격이 깨졌다)
            "lane_change_aborted": lane_change_aborted,
            "lane_change_actual": actual_lane_change,
            "lane_change_observable": lane_change_observable,
            "lane_before": lane_before,
            "lane_after": lane_after,
            "requested_speed": requested_speed,
            "service_event": service_event,
            "stops_served": self.stops_served,
            "stops_missed": self.stops_missed,
            "speed_intervened": bool(alive and requested_speed is not None and
                                      abs(traci.vehicle.getSpeed(self.ego_id)
                                          - requested_speed) > 1e-3),
        }
        return obs, float(reward), terminated, truncated, info

    def _stalled(self):
        """stuck_progress_m를 가는 데 stuck_timeout보다 오래 걸렸는지. 서 있는 것과 기어가는 것을 함께 잡는다.

        SUMO의 대기 시간은 0.1m/s를 넘으면 0으로 돌아가, 기어가는 정책은 시간 제한까지 stuck이 되지 않았다.
        """
        distance = float(traci.vehicle.getDistance(self.ego_id))
        if distance - self._progress_mark[1] >= self.stuck_progress:
            self._progress_mark = (self.step_count, distance)
        return (self.step_count - self._progress_mark[0]) * self.decision_interval >= self.stuck_timeout

    # ==================================================================
    def close(self):
        """환경 정리. 학습이 끝나면 반드시 호출해서 SUMO 프로세스를 종료할 것."""
        if self._started:
            traci.close()
            self._started = False
