# PDM 버스 전문가 데이터 수집 설계

작성 2026-11-24. 대상 브랜치 `feat/pdm-closed-bus`.

## 1. 의도

`scripts/run_hapjeong_pdm_bus.py`와 같은 도로·교통·시나리오에서 PDM-Closed 버스(전문가, `traffic/pdm_bus.py`)를 달리게 해 원시 데이터를 모은다. 이 데이터 하나로 세 가지 일을 한다.

1. **모델 학습용 transition**: `scripts/data_colum.md`의 필수 열 전부를 만든다. 정책 입력은 93차원 관측(`env/bus_task.py`의 `bus_service_v1`)이다.
2. **입력 설계 실험**: `SUMO_AV_input_설계.md`가 제안한 100~130차원 특징을 원시 값에서 계산할 수 있어야 한다. 관측 정의를 바꾸더라도 다시 수집하지 않고 전처리만 다시 돌린다.
3. **평가**: 차량·보행자와 버스의 상호작용(간격, TTC, DRAC, PET, 양보 여부, 유발 급제동)과 시나리오 S1~S6·A1~A3의 발생과 결과를 잰다. 메타데이터로 어떤 실행에서 나온 데이터인지 재현할 수 있어야 한다.

## 2. 목표, 비목표, 반목표

**목표**
- 수집기는 원시 값만 남기고, 학습용 열·특징·지표는 모두 전처리에서 계산한다.
- `data_colum.md` 필수 열 23개 전부를 전처리로 만들 수 있다.
- 입력 설계 표(9개 특징 묶음)의 모든 항목을 원시 값이나 도로망 파일에서 계산할 수 있다.
- 상호작용과 시나리오는 행 단위(지금 진행 중인 것)와 사건 단위(시작·끝·결과) 둘 다로 남긴다.

**비목표**
- 정책 학습, 학습 코드 수정.
- 시나리오·교통·전문가 알고리즘의 동작 변경.
- 가림(occlusion)이 있는 센서 모델. 감지는 지금 관측과 같은 이상적 반경 감지다.
- `collect_bus_lap.py`, `collect_pomdp_data.py` 교체. 둘 다 그대로 둔다.

**반목표(기술적으로 돌아가도 실패인 것)**
- 기록 때문에 전문가나 교통의 움직임이 바뀌는 것. 기록 코드는 TraCI 조회만 하고 set 계열 호출을 하지 않는다. 특히 7절의 구독 함정에 주의한다.
- 다시 계산할 수 없는 값을 빠뜨리는 것. 신호 전환 시각, `getBestLanes`, 정차 남은 시간처럼 나중에 위치로 복원할 수 없는 값은 반드시 수집 때 남긴다.
- 값이 없는 것과 0을 구분하지 못하는 것. 없으면 `null`이다.
- 오프라인에서 계산한 93차원 관측이 `BusTask.observation`과 다른 값을 내는 것.

## 3. 제약

- SUMO 1.27.1(`D:/Users/user/Sumo`). 필요한 TraCI 함수(`getJunctionFoes`, `getStops`, `getNextLinks`, `getBestLanes`, `getNeighbors`, `getFollower`, `getLaneChangeState`, `couldChangeLane`, `getSecureGap`, `getFollowSpeed`, `getStopSpeed`, `getSpeedWithoutTraCI`, `getLateralSpeed`, `trafficlight.getNextSwitch`, `getSpentDuration`, `person.getNextEdge`, `getWaitingTime`)가 모두 있음을 확인했다.
- 기본 python은 3.9다. 3.10 이상이 필요하면 `D:\conda_envs\faster`를 쓴다. C 드라이브가 부족하므로 출력은 저장소의 `data/` 아래(D 드라이브)에 둔다.
- `scripts/run_hapjeong_scenario.py`와 `scripts/run_hapjeong_pdm_bus.py`는 고치지 않는다. `traffic/pdm_bus.py`에는 동작이 바뀌지 않는 덧붙임만 허용한다(마지막 점수 결과를 속성에 남기기).
- 시뮬레이션 계산 간격은 0.1초, 판단 간격은 0.5초(5스텝)다. `env/mdp_config.py`, `collect_bus_lap.py`와 같다.
- 월드 설정 기본값은 `run_hapjeong_pdm_bus.py`(곧 `run_hapjeong_scenario.py`)의 기본값과 같다. 주입기 켬, `triggers="scenario"`, 정류장 앞 정차 켬, A3 보행자 판정 켬, S1 판정 끔, 밀도 유지기 켬, `route_boost` 1.3, 배경 버스 30대/h, 택시 40대/h.

## 4. 접근

| 방식 | 내용 | 장단점 |
|---|---|---|
| **A. 원시 값 수집 + 오프라인 전처리 (채택)** | 수집기는 원시 값·사건·메타데이터만 쓴다. 관측·행동·보상·특징·지표는 전처리 스크립트가 계산한다 | 관측이나 보상 정의를 바꿔도 다시 수집할 필요가 없다. "수집 뒤 전처리로 추출"이라는 사용자 흐름과 맞다. 대신 93차원 관측을 다시 만들 때 값이 어긋날 위험이 있어 시험으로 막는다 |
| B. 필수 열을 수집 중에 계산 + 원시 값 병행 | `collect_pomdp_data.py`처럼 관측·보상을 즉시 계산한다 | 재현할 때 어긋날 걱정은 없다. 대신 정의를 바꾸면 결국 전처리가 따로 필요하고, 같은 값을 두 곳에서 계산한다 |
| C. SUMO 출력 파일 중심 | FCD·SSM·tripinfo 파일에 TraCI 값을 조금 보탠다 | 도로망 전체를 기록해 파일이 크다. ego 기준 값(신호 전환 시각, best lanes, 정차 남은 시간)은 결국 TraCI로 받아야 한다 |

**사용자 결정**: A에 C의 SSM 장치만 더한다. SUMO SSM 장치를 시험 버스 유형에만 붙여 차량 간 TTC·DRAC·PET를 SUMO가 직접 계산하게 한다.

## 5. 결정된 사항

| 항목 | 결정 | 출처 |
|---|---|---|
| 행동 라벨 | **A안: 실제로 일어난 결과 기준.** 가감속은 판단 구간 동안의 실제 가속도를 정규화한 값, 차선 변경은 전문가가 요청한 방향이다. 전문가 명령(속도 비율, 규칙의 요청 단계)은 별도 열로 남긴다 | 사용자 |
| 에피소드 | 버스 한 바퀴(R68 → R68). `t`는 바퀴마다 0부터 센다 | 제안 승인 |
| 에피소드 노선 | **B안: R68 + `bus_loop`(= `TestBusPilot.lap_route`)를 한 바퀴 노선으로 쓴다.** 노선 위치·진행률·남은 노선·진행 보상의 분모·arrived는 모두 이 노선 기준이다. 배포 환경의 `BusTask`는 지도 `bus_loop`(R69 → R42, 1949.44 m, R68 없음)를 쓰므로 지금은 정의가 다르다. 사용자가 나중에 배포 환경을 이 노선에 맞게 고친다 | 사용자 (검토 뒤) |
| 보상 | `env/bus_task.py`의 `REWARDS`(진행, 정차 완료, 충돌, 신호 위반, 정류장 놓침)로 전처리에서 계산한다. 항목별 값을 따로 남긴다 | 제안 승인 |
| terminated / truncated / arrived | 충돌이면 terminated, 시간 한도를 넘으면 truncated다. arrived는 한 바퀴를 마치고 정류장 4곳을 모두 들렀을 때만 참이다 | 제안 승인 |
| 시간 한도 | 배포 환경의 1000스텝(500초)을 쓰지 않는다. 실측 바퀴가 600~800초라 대부분 잘리기 때문이다. 기본은 바퀴를 1800초(수집기 `--max-seconds`) 안에 못 마치면 truncated다. 전처리 인자로 바꿀 수 있다 | 빌더 검토 뒤 Claude 결정 |
| 버스 순간이동 | ego가 순간이동하면 그 바퀴는 순간이동 직전 행에서 truncated로 끊고 `termination_reason="teleport"`를 남긴다. 뒤의 행은 transition에서 뺀다(진행 보상이 튀므로). 원시 행은 그대로 남는다 | 빌더 검토 뒤 Claude 결정 |
| 범위 | 특권 state는 반경 150 m(`mdp_config.state_radius`). 감지(detected)는 반경 50 m(관측 visibility). 선행차·신호·정류장은 반경이 아니라 노선을 따라 160 m 앞까지 찾는다 | 제안 승인 |
| 정적 정보 | 도로망 해시와 도로망·추가 파일 사본을 메타데이터 폴더에 둔다. 차로 연결 같은 정적 값은 전처리에서 `net.xml`로 계산한다 | 제안 승인 |
| 상호작용 | 행마다 상대별 원시 값을 남기고, 사건 표(A1, A3, S5, S6 PET, 유발 급제동)와 SSM 출력도 남긴다 | 제안 승인 |
| 시나리오 | `scripts/measure_s1_s6.py`의 사건 정의를 그대로 쓴다. 주입 상대와 자연 발생 상대를 구분한다. 행마다 진행 중인 시나리오 표시를 붙인다 | 제안 승인 |

## 6. 산출물

실행 하나(시나리오 × 시드 × 바퀴 수)가 폴더 하나다. 기본 위치는 `data/pdm_bus/<run_name>/`이다.

```
data/pdm_bus/<run_name>/
  steps.jsonl.gz       판단 한 번(0.5초)마다 한 줄. 7절
  events.jsonl         사건 하나에 한 줄. 8절
  ssm.xml              SUMO SSM 장치 출력(시험 버스만). 9절
  metadata.json        실행 설정·결과 요약·컬럼 설명. 10절. 마지막에 쓴다: 이 파일이 있으면 수집이 끝까지 된 것이다
  assets/              network.net.xml과 실행에 쓴 추가 파일 사본(전처리가 도로망 계산에 쓴다)
```

전처리 결과는 `data/pdm_bus/<run_name>/processed/`에 둔다(11절).

## 7. 원시 행 (`steps.jsonl.gz`)

**시점 규칙**: 행 `t`는 판단 구간 [t, t+0.5초)의 시작에서 찍은 스냅숏과, 그 구간이 끝날 때 채운 `window` 블록으로 이루어진다. 행동 라벨은 이 구간에서 일어난 일이므로 행 하나 안에서 짝이 맞는다. 바퀴의 마지막 구간 뒤에는 `window`가 없는 마지막 스냅숏 행(`final: true`)을 하나 더 써서 `next_state`와 `next_observation`을 만들 수 있게 한다.

모든 좌표·방향·차로 번호 규칙은 `env/data_schema.py`의 `CONVENTIONS`를 따른다. 정리하면 좌표는 SUMO 도로망 좌표, 방향은 북쪽 0도에서 시계 방향, 차로 0번은 오른쪽(합정역은 0번이 인도), 방향지시등 비트는 1 오른쪽·2 왼쪽·8 제동이다.

### 7.1 `meta`
`run_id`, `episode`(바퀴 번호), `t`(바퀴 안 판단 번호), `sim_time`, `sim_step`, `final`.

### 7.2 `ego` (스냅숏)
| 키 | TraCI |
|---|---|
| x, y, heading | getPosition, getAngle |
| speed, acceleration, lateral_speed | getSpeed, getAcceleration, getLateralSpeed |
| speed_without_traci | getSpeedWithoutTraCI (SUMO 모델이 판단한 안전 속도. 입력 설계의 기준선·마스크용) |
| allowed_speed, lane_max_speed, max_speed | getAllowedSpeed, lane.getMaxSpeed, getMaxSpeed (PDM이 넣은 v0) |
| road_id, lane_id, lane_index, lane_count | getRoadID, getLaneID, getLaneIndex, edge.getLaneNumber |
| lane_position, lateral_position | getLanePosition, getLateralLanePosition |
| route_index, lap_route_index, distance | getRouteIndex, 한 바퀴 노선 안 번호, getDistance |
| signals, stop_state, waiting_time, accumulated_waiting | getSignals, getStopState(비트), getWaitingTime, getAccumulatedWaitingTime |
| lc_state_left, lc_state_right | getLaneChangeState(±1) 원시 비트 쌍 |
| could_change_left, could_change_right | couldChangeLane(±1) |
| length, width, min_gap | 고정값. 행마다 쓰지 않고 메타데이터에 둔다 |

### 7.3 `expert` (전문가가 이 시점에 가진 판단)
- 규칙(`TestBusPilot`): `phase`, `why`, `plan_kind`(S3/S2/STOP), `target_lane`, `request_direction`(+1 왼쪽, −1 오른쪽, 0), `forced_stop`, `speed_limited`, `blocker_active`.
- PDM(`PDMClosedBusPilot`): 마지막 계획의 `fraction`, `v0`, `scores[5]`, `mult[5]`, `progress[5]`, `ttc[5]`, `comfort[5]`, `collide_t`, `emergency_brake`, 계획 시각. 이를 위해 `pdm_bus.py`에 `self.last_plan = res` 같은 동작 불변 덧붙임을 한다.

### 7.4 `route` (노선·경로)
- `best_lanes`: 현재 도로의 getBestLanes 전체. 차로마다 laneID, length, occupation, offset, allowsContinuation, nextLanes를 남긴다.
- `next_links`: getNextLinks 전체. lane, via, priority, opened, foe, state, direction, length를 남긴다.
- `lap_progress`: 한 바퀴 노선(R68 + `bus_loop`) 위 위치와 바퀴 길이. 정의는 `env/route_map.RouteMap.position`과 같지만, RouteMap은 이 한 바퀴 노선으로 만든다. SUMO 안의 버스 노선은 바퀴마다 이어 붙어 길어지므로(`TestBusPilot._extend`) `getRouteIndex`를 그대로 쓰지 않는다. 현재 도로가 한 바퀴 노선의 몇 번째인지(`lap_route_index`)로 바꿔 쓴다.
- `next_turn`: RouteMap.ahead 결과(다음 회전 종류와 거리, 남은 거리). 전처리에서 다시 계산할 수 있지만 검산용으로 남긴다.

### 7.5 `tls` (신호)
노선을 따라 200 m 안의 getNextTLS 전체 목록. 항목마다 다음을 남긴다.
`id`, `link_index`, `distance`, `state`(ego가 지날 link의 문자), `time_to_switch`(getNextSwitch − 지금), `spent`(getSpentDuration), `phase`, `program`. 신호 프로그램 자체는 메타데이터에 둔다. 전처리는 프로그램으로 time-to-green과 time-to-red를 계산한다.

### 7.6 `stops` (정류장)
- getStops(BUS, 4)의 StopData 원시 값: lane, startPos, endPos, stoppingPlaceID, stopFlags, duration, until, arrival, actualArrival, depart, intendedArrival.
- 각 정류장까지의 노선 거리.
- getStopState 비트. getStopArrivalDelay는 arrival이 지정된 경우에만 남긴다.
- 정차 중이면 남은 정차 시간.

### 7.7 `neighbors` (차선 기준 슬롯)
여섯 슬롯이다: 현재 차로 선행(getLeader, 160 m), 현재 차로 후행(getFollower, 80 m), 좌·우 선행과 좌·우 후행(getNeighbors 모드 비트). 슬롯마다 다음을 남긴다.
- `id`, `gap_raw`(TraCI가 준 값. 선행 간격은 ego minGap을 뺀 값이라는 규칙을 메타데이터에 적는다)
- 선행 슬롯: `secure_gap`(getSecureGap, ego 기준), `follow_speed`(getFollowSpeed), `stop_speed`(getStopSpeed(gap))
- 후행 슬롯: 후행차 기준 `secure_gap`
슬롯 상대의 속도·가속도·횡속도·방향지시등은 7.9의 차량 표에서 id로 찾는다. 빈 슬롯은 `null`이다.

### 7.8 `junction_foes`
getJunctionFoes(BUS, 100 m)의 원시 튜플 전체: foeId, egoDist, foeDist, egoExitDist, foeExitDist, egoLane, foeLane, egoResponse, foeResponse. 입력 설계의 골목·교차 차량 슬롯과 우선권 비트에 쓴다.

### 7.9 `vehicles` (특권, 반경 150 m)
차량마다 남기는 값:
- 기본: `id`, `type`, `vclass`, `model`(Krauss/IDM/EIDM/ACC/bus/taxi 등), `x`, `y`, `heading`, `speed`, `acceleration`, `lateral_speed`, `road_id`, `lane_id`, `lane_index`, `lane_position`, `lateral_position`, `signals`, `length`, `width`, `min_gap`, `waiting_time`.
- 성격·역할: `driver_type`(yielder/blocker 등 파라미터), `p_yield`, `injected`(주입기가 넣었나), `role`(S2/S3 후행 상대, S4 정차, S5 끼어들기, A1 상호작용 중이면 그 판정).
- 상호작용 계산용: 같은 노선 차로면 ego와의 노선 거리.

정적인 값(`type`, `length` 등)은 처음 본 때 한 번만 쓰고 이후 행에서는 생략할 수 있다. 다만 그러면 읽기 함수가 채워 줘야 한다. 이 선택은 빌더에게 맡긴다.

### 7.10 `pedestrians` (특권, 반경 150 m)
보행자마다 남기는 값: `id`, `x`, `y`, `heading`, `speed`, `road_id`, `lane_position`, `next_edge`, `waiting_time`, `stage`, `on_crossing`(내부 crossing 도로 위인가. 이름 규칙은 수집 전에 도로망에서 확인한다), `injected`, `role`(S6 주입, s6ped 배경), `a3_decision`(A3 판정이 있으면).

### 7.11 `scenario` (지금 진행 중인 것)
- `s1`: 정류장 정차 중이고 같은 차로 후행차가 있다.
- `s2` / `s3`: 차선 변경 창 안. 창 종류, 창 끝까지 남은 거리, 상대 id를 남긴다.
- `s4`: 노선 앞 50 m 안에 정차 차량이 있다. 그 id를 남긴다.
- `s5`: 주입된 끼어들기 차가 골목 끝에 대기 중이거나 버스 50 m 안에 있다. id와 상태를 남긴다.
- `s6`: 신호 없는 횡단보도 50 m 앞에 건너는 중이거나 대기 중인 보행자가 있다. id를 남긴다.
- `a1`: 상호작용 중인 차량 id와 판정.
- `injector_plan`: 이번 바퀴의 주입 계획.

### 7.12 `window` (구간 [t, t+0.5초)이 끝날 때 채움)
- `substeps`: 0.1초 스텝 5개마다 ego의 `edge`, `lane_index`, `lane_position`, `distance`, `speed`, `acceleration`, 첫 신호 튜플. 전처리가 `BusTask`의 정차 판정(`ServiceTracker`)과 신호 위반 판정(`crossed_red`)을 원래와 같은 0.1초 간격으로 다시 돌리는 데 쓴다.
- `peak_acceleration`, `peak_jerk`.
- 구간 안의 전문가 명령: setMaxSpeed 값 변화, 긴급 제동 setSpeed, changeLane 요청과 그 방향.
- `lane_before`, `lane_after`, `edge_before`, `edge_after`.
- `collisions`: 구간 안 충돌 전부(관련 id, ego 관련 여부, 종류).
- `teleports`, `emergency_stops`(ego 50 m 안 여부 포함).
- `lap_done`.

### 7.13 구독 함정
`PDMClosedBusPilot`은 버스(`bus0`)에 반경 50 m 차량·보행자 context 구독을 걸어 자기 물체 목록을 만든다. 같은 버스·같은 도메인에 수집기가 다른 반경이나 변수로 context 구독을 다시 걸면 PDM의 구독을 덮어써 전문가 동작이 바뀐다. 수집기는 버스에 context 구독을 걸지 않는다. 대신 getIDList와 위치 필터를 쓰거나, 차량별 변수 구독이나 다른 기준 물체의 구독을 쓴다. 구체적인 방법은 빌더가 정한다. 12절의 "동작 불변" 시험으로 확인한다.

## 8. 사건 표 (`events.jsonl`)

한 줄이 사건 하나다. 공통 키는 `kind`, `lap`, `t_start`, `t_end`, `ids`, `injected`, `detail`이다.

| kind | 원천 | detail |
|---|---|---|
| `lane_window` | 시험 버스 LanePlan | 창 종류(S2/S3/STOP), 시작·끝, 성공·실패, missed, 강제 정차 여부, 실제 변경 시각 |
| `a1` | `YieldManager.log` | 후행차, 양보/비양보, action, 시작·끝, 주행 모델 |
| `a3` | PedDefianceManager | 보행자, 건너기 판정, 선두 차량 급감속·비상 감속 |
| `s1_stop` | 정차 판정 + 후행차 | 정류장, 정차 시작·끝, 후행차와 그 반응(대기/추월) |
| `s4_block` | StopAheadManager 기록 | 정차 차량, 정류장, 버스가 막혔는지 |
| `s5_cutin` | 주입기 기록 + `measure_s1_s6`의 정의 | 골목, 풀린 시각, 15초 안에 버스 앞 50 m에 들어왔나, 버스 최소 간격, 버스 감속 |
| `s6_pass` | 횡단보도 통과 | 보행자, 보행자·버스 점유 구간, PET, 버스 최소 속도 |
| `injection` | `ScenarioInjector.log` | 계획 종류, 성공 여부, `natural_present` |
| `induced_brake` | 50 m 안 차량의 감속 3.0 m/s² 이상 | 차량, 최대 감속, 버스 후행차였나 |
| `collision` / `teleport` / `emergency_stop` | SUMO | 관련 id, 위치, ego 관련 여부 |
| `service` / `missed_stop` / `red_violation` | 수집 중 기록하지 않음 | 전처리가 `window.substeps`로 만들어 `processed/events.jsonl`에 더한다 |

행 단위 표시(7.11)와 사건 표는 id와 시각으로 이어 볼 수 있어야 한다.

## 9. SSM

- 수집기는 `hapjeong.BUS_VTYPE`을 PDM 유형(`BUS_PDM_VTYPE`)에 SSM 파라미터를 더한 문자열로 바꿔 넣는다. 붙는 파라미터는 `has.ssm.device=true`, `device.ssm.measures="TTC DRAC PET BR SGAP TGAP"`, 임계값 TTC 4.0 s·DRAC 3.0 m/s²·PET 2.0 s, `device.ssm.range` 50 m, `device.ssm.file=<run>/ssm.xml`이다. `pdm_bus.py`와 실행 스크립트는 고치지 않는다.
- SSM은 동역학을 바꾸지 않는다. 12절 시험으로 확인한다.
- 보행자 상충을 SUMO 1.27.1 SSM이 지원하는지는 빌더가 확인한다. 지원하면 켜고, 아니면 보행자 PET·TTC는 8절의 `s6_pass`와 전처리 계산으로 대신한다.
- 전처리는 ssm.xml의 상충(foe, 종류, 최소 TTC·DRAC·PET와 그 시각)을 `processed/events.jsonl`의 `ssm_conflict`로 옮긴다.

## 10. 메타데이터 (`metadata.json`)

- **재현**: schema 버전(`pdm_bus_raw_v1`), 스크립트, git 커밋과 수정 여부, SUMO·python 버전, 실행 명령.
- **월드 설정**: 시나리오(quiet/baseline/busy), 시드, warmup, 배경 버스·택시, route_boost, 밀도, 정차 차량 사양, triggers, 주입기·A3·S1·정류장 앞 정차 켬/끔.
- **전문가 설정**: `PDMCfg` 전체, `TestBusPilot` 상수, 버스 vType 문자열(SSM 포함), 정차 시간.
- **시간·범위**: step_length, 판단 간격, state 반경, detected 반경, 노선 검색 거리.
- **노선**: 한 바퀴 도로 목록, 바퀴 길이, 정류장 정의(서는 4곳 표시), 차선 변경 창(LanePlan), 신호 앞 금지 구간, 노선 위 신호 프로그램 전체(getAllProgramLogics).
- **도로망**: `assets/network.net.xml`의 sha256.
- **결과 요약**: 바퀴 완주 여부, 바퀴 시간, 행 수, 차선 변경 요청 수, missed, 강제 정차, 충돌·순간이동 수, PDM 요약(속도 비율 분포, 긴급 제동 수), 시나리오별 사건 수.
- **열 설명**: 7~9절의 키와 단위.

## 11. 전처리 (`processed/`)

전처리 스크립트 하나가 실행 폴더를 읽어 다음을 만든다. 보상 가중치와 행동 정규화 상수 같은 매개변수는 인자로 받고, 쓴 값을 `processed/metadata.json`에 남긴다.

### 11.1 `data_colum.md` 필수 열

| 열 | 만드는 법 |
|---|---|
| episode, t | `meta.episode`, `meta.t` |
| observation | 행 t 스냅숏으로 `BusTask.observation`과 같은 식의 93차원을 계산한다. 노선 관련 값(navigation 블록, 남은 노선 obs[80], 정류장 블록)은 한 바퀴 노선(R68 + `bus_loop`)으로 만든 RouteMap 기준이다. vmax는 지도 `route_design.bus_loop.max_speed_mps`(13.89)다(12절 시험) |
| action_raw | [action.accel_raw, action.lane_change_raw] |
| action.accel_raw | a = (v(t+0.5) − v(t)) / 0.5이다. a ≥ 0이면 a / A⁺, 아니면 a / A⁻이고 [−1, 1]로 자른다. **A⁺/A⁻는 수집 뒤 전처리에서 정한다(사용자 결정).** 수집기는 정규화하지 않고 원시 값(`window.substeps`의 속도·가속도, 전문가 vType의 accel·decel·emergencyDecel)만 남긴다. 전처리는 A⁺/A⁻를 반드시 인자로 받으며 기본값이 없다. 배포 환경 값을 묵시적으로 쓰지 않는다. 정하는 데 도움이 되도록 전처리에는 a의 분포 보고 모드를 둔다(백분위, 최댓값, 후보 상수별 잘림 비율). 쓴 상수와 잘린 비율은 `processed/metadata.json`에 남기고, 정규화 전의 a는 `action_extra.accel_mps2`에 남긴다 |
| action.lane_change_raw | 아래 lane_change와 같은 값의 float |
| action.lane_change | 전문가가 요청 단계(`phase == "request"`)에 있는 구간에서는 요청 방향, 아니면 0이다(+1 왼쪽). `TestBusPilot`은 changeLane을 요청 단계 처음과 그 뒤 2초(도착 보장 정차 중에는 1초)마다만 부른다. 하지만 배포 환경은 정책이 매 판단마다 ±1을 내야 변경이 유지되므로, 요청 단계 전체를 ±1로 둔다. 도착 보장 정차 중의 요청과 `_emergency` 요청도 포함한다. 방향지시등만 켠 `signal` 단계는 0이다. 실제 changeLane 호출 시각은 `window`에 남아 다른 규칙으로도 다시 만들 수 있다 |
| action.lane_change_applied | lane_change가 0이면 참이다. 아니면 구간 안에 같은 도로에서 그 방향으로 차로가 실제로 바뀌었을 때 참이다 |
| reward | `BusTask.outcome`과 같은 식이다: 진행 × 10 / 바퀴 길이 − 0.001 + 정차 완료 2 + 종료 보상. 항목별 값은 `reward_terms.*`에 남긴다 |
| terminated | ego 충돌, 또는 `BusTask`가 끝내는 사건(신호 위반·정류장 놓침), 또는 도착이면 참이다. 기본은 `BusTask`를 따라 첫 종료 사건에서 에피소드를 자르고, 인자로 자르지 않게 할 수 있다 |
| truncated | 바퀴를 시간 한도 안에 못 마쳤다 |
| state | 행 t 스냅숏(ego, vehicles, pedestrians, route, tls, stops, neighbors, junction_foes) |
| detected_vehicles | vehicles 중 반경 50 m. 키는 `data_schema.DETECTED_VEHICLE_KEYS`를 따른다 |
| event.collided | `window.collisions`에 ego가 있나 |
| event.arrived | 바퀴 완료 그리고 4곳 모두 정차 완료 |
| event.lane_before / lane_after | `window.lane_before` / `lane_after` |
| next_observation / next_state | 행 t+1(또는 `final` 행)의 관측·스냅숏 |

여기에 더하는 열은 다음과 같다: `detected_pedestrians`, `event.termination_reason`, `comfort.*`(peak, 위반 여부), `expert.*`(B안 라벨: 속도 비율, 요청 단계), `reward_terms.*`, `scenario.*`, `interaction.*`(11.3).

### 11.2 입력 설계 특징 (`SUMO_AV_input_설계.md`)

| 특징 묶음 | 원천 |
|---|---|
| Ego 운동 | ego.speed·acceleration, 직전 행 action, ego.lc_state, lane_index, lane_count |
| 차선 변경 가능 | ego.could_change_left/right |
| 경로 | route.best_lanes(offset, continuation), route.next_links(direction), route.next_turn, lap_progress |
| 정류장 | stops(거리, 차로, 남은 정차 시간, stopFlags), lanes_to_stop = 정류장 차로 − 현재 차로, 단계 one-hot(순항/접근/정차/출발. 전처리 규칙) |
| 신호 | tls[0] 거리·link 문자(G/g 구분)·time_to_switch, 메타데이터의 신호 프로그램으로 time-to-green/red, dilemma 특징 |
| 주변 차량 lane-slot | neighbors 6슬롯 + vehicles(속도, 가속도, 횡속도) → Δs, Δv, 상대 가속도, v_y, inverse TTC |
| 전후방 안전 여유 | time headway, gap − secure_gap(minGap 보정), 후행 inverse TTC, 후행 요구 감속도, 앞쪽 차량 수(vehicles) |
| 골목·교차 차량 | junction_foes + next_links(priority, foe) → conflict 거리, 도착 시간차, has_priority, phantom 슬롯 |
| 보행자 | pedestrians(위치, 방향, 속도, road_id, next_edge, waiting_time, on_crossing) + 도로망의 crossing 위치 → 횡오프셋, 의도 proxy, t_ped/t_ego to conflict |

이 표가 원시 값만으로 채워지지 않는 항목이 생기면 그 항목은 수집기에 원시 값을 더하는 쪽으로 고친다.

### 11.3 상호작용 열 (행마다)
- 선행·후행 슬롯마다 THW, TTC, inverse TTC, DRAC, gap − secure_gap.
- 50 m 안 보행자마다 conflict까지 버스·보행자 도달 시간과 그 차.
- 지금 상호작용 중인 상대 수(A1, S5, S6).

### 11.4 요약
실행마다 바퀴별 요약표(`processed/summary.json`)를 만든다: 시나리오별 사건 수와 결과, 상호작용 지표 분포, SSM 상충 수, 승차감 위반 수, 정류장 정차 완료 수, 보상 합.

## 12. 검증

- **93차원 일치**: 수집기에 확인 모드를 둔다. 같은 스냅숏에서 `BusTask.observation`을 부르고, 오프라인 전처리 값과 비교한다. 이때 얇은 어댑터를 쓰는데, 어댑터의 `route_map`은 한 바퀴 노선(R68 + `bus_loop`)으로 만든 RouteMap이다. 행 200개 이상에서 최대 절대 오차가 1e-4 미만이어야 한다. 이 시험은 "같은 식"을 확인할 뿐이다. 노선이 달라 지금의 배포 환경 관측과는 값이 다르며, 이는 의도한 차이다(5절).
- **동작 불변**: 같은 시드로 `run_hapjeong_pdm_bus.py --nogui`(기록 없음)와 수집기(기록·SSM 켬)를 같은 시간만큼 돌린다. 버스의 속도·위치 궤적과 PDM 요약이 같아야 한다.
- **필수 열 검사**: 전처리 결과에 `data_colum.md`의 23개 열이 모두 있고, 형태가 맞고, `episode`와 `t`가 이어지는지 확인한다(`data_schema.lookup` 활용).
- **SUMO 없는 단위 시험**: 행동 라벨, 보상 재계산, 특징 계산은 손으로 만든 스냅숏으로 시험한다.
- 긴 실행 대신 짧은 실행(바퀴 일부)으로 확인하고, 돌리지 않은 것은 보고에 적는다.

## 13. 빌더에게 맡기는 것

- 파일·함수 이름과 모듈 배치(예: `scripts/collect_pdm_bus.py`, `scripts/preprocess_pdm_bus.py`, 공용 계산은 `traffic/` 또는 `env/` 아래).
- 7.13의 구독 함정을 피하는 구체적 방법과 수집 속도 최적화.
- 정적 값을 처음 한 번만 쓸지 여부와 그에 맞는 읽기 함수.
- 전처리 결과 형식. 기본은 JSONL.gz와 NPZ이고 `collect_pomdp_data.py`의 관례를 따른다.
- 여러 실행을 돌리는 방법(시나리오 × 시드 반복, 병렬 작업 수). 기본 명령 예시는 문서화한다.
- 단계 one-hot(순항/접근/정차/출발)의 구체적 규칙과 dilemma 특징 식. 입력 설계 문서의 정의를 따르되, 정해지지 않은 부분은 빌더가 정하고 메타데이터에 적는다.
