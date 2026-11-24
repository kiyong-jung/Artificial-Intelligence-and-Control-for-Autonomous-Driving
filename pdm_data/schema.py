"""PDM 버스 원시 데이터의 공통 상수(설계: docs/superpowers/specs/2026-11-24-pdm-bus-data-collection-design.md)."""
from __future__ import annotations

from traffic.test_bus import STOP_AT

RAW_SCHEMA_VERSION = "pdm_bus_raw_v1"
BUS = "bus0"
DT = 0.1                      # SUMO 계산 간격 [s]
DECISION_STEPS = 5            # 판단 간격 0.5 s
STATE_RADIUS_M = 150.0        # 특권 state 반경
DETECT_RADIUS_M = 50.0        # 감지(관측 visibility) 반경
LEADER_LOOKAHEAD_M = 160.0
FOLLOWER_LOOKBACK_M = 80.0
TLS_LOOKAHEAD_M = 200.0
FOES_LOOKAHEAD_M = 100.0
STOPS_AHEAD = 4
INDUCED_BRAKE_MPS2 = 3.0
S5_WINDOW_S = 15.0
NEAR_M = 50.0
DEFAULT_MAX_LAP_S = 1800.0
STOP_IDS = tuple(STOP_AT)     # 서는 정류장 s2, s4, s5, s6
SERVICE_SECONDS = 8.0

COLUMN_DOC = {
    "meta": "run_id, episode(바퀴), t(바퀴 안 판단 번호), sim_time, sim_step, final(에피소드 마지막 스냅숏), end(끝 원인)",
    "ego": "버스 상태 스냅숏: 위치·속도·가속도·횡속도, SUMO 안전 속도, 제한·허용·최대 속도(PDM v0), 도로·차로, 한 바퀴 노선 번호·위치, "
           "방향지시등·정차 비트, 대기 시간, 차선 변경 상태·가능 여부",
    "expert": "전문가 판단: 규칙(phase, why, 창 종류, 목표 차로, 요청 방향, 강제 정차, 속도 제한, 막는 차)과 PDM 마지막 계획(속도 비율, v0, 제안 점수)",
    "route": "getBestLanes, getNextLinks 원시 값, 한 바퀴 노선 위치·길이, 다음 회전",
    "tls": "노선 200 m 안 신호: id, link 번호, 거리, ego link 문자, 전환까지 시간, 경과 시간, phase, program",
    "stops": "getStops 원시 값(StopData), 각 정류장까지 노선 거리, 정차 상태 비트",
    "neighbors": "현재·좌·우 차로의 선행·후행 슬롯: id, TraCI 간격(gap_raw, minGap 제외), secure_gap, follow_speed, stop_speed",
    "junction_foes": "getJunctionFoes(100 m) 원시 튜플",
    "vehicles": "반경 150 m 차량: 위치·속도·가속도·횡속도·차로·신호·치수, 운전 성격, 주입 여부, 역할, 노선 거리",
    "pedestrians": "반경 150 m 보행자: 위치·방향·속도·도로·다음 도로·대기 시간·단계, 횡단보도 위 여부, 주입 여부, 역할, A3 판정",
    "scenario": "지금 진행 중인 S1~S6·A1 표시와 이번 바퀴 주입 계획",
    "stopped_since": "버스가 지금 정차를 시작한 시각(정차 중이 아니면 null)",
    "window": "판단 구간 [t, t+0.5) 동안: 0.1 s substep 5개, 승차감 최댓값, 전문가 명령 호출, 차로 전후, 충돌·순간이동·비상정지, 바퀴 완료",
    "events.jsonl": "사건 한 줄: kind, lap, t_start, t_end, ids, injected, detail",
    "ssm.xml": "SUMO SSM 장치 출력(시험 버스만, TTC·DRAC·PET·BR·SGAP·TGAP)",
}
