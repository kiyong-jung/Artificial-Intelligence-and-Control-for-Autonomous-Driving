# 주변 교통 알고리즘 구현·검증 결과

계획: `Other_vehicle_algorithm_implementation_plan.md`. SUMO 1.27.1, Python 3.9 (conda `sumo`), TraCI. 모든 수치는 `scripts/verify_a1.py`, `verify_a2.py`, `verify_a3.py`를 실제로 돌려 얻었고 원본은 `results/traffic_verification/*.json`(git 제외)에 있다. 빠른 회귀 검증은 `pytest tests/test_traffic.py`(11개 통과).

검증 도로는 합성 도로다(2차로 직선, 우선 교차로 합류, 횡단보도 있는 T자로). 합정역 지도에서의 검증은 하지 않았다.

## 구현

| 파일 | 내용 |
| --- | --- |
| `traffic/yield_logic.py` | A1. 진입 의사(방향지시등 + 출구 거리 + 직전 2 s 전진) → 후행차 조회 → 1회 결정 → 반응 지연 → `openGap(referenceVeh=버스)` → 종료 시 해제 |
| `traffic/cutin.py` | A2. 구간별 vType + `vTypeDistribution`(jmIgnoreFoeProb/Speed, jmTimegapMinor, impatience), S4 parkingArea onRoad·택시 flow |
| `traffic/pedestrian.py` | A3. P1(jmIgnoreFoeProb 1, jmIgnoreFoeSpeed = v_low, impatience 0), P2(기본 간격 수용 + impatience), 비우선 횡단보도 |
| `traffic/scenarios*.py` | 판단 없는 시험용 버스와 합성 도로 |
| `traffic/sumo_conn.py` | 0.1 s 스텝, ballistic, `collision.action warn`, `check-junctions`, `intermodal-collision.action warn` |

## 계획과 다르게 한 것 (실측으로 확인)

1. **양보형도 `lcCooperative`를 끈다.** 계획은 양보형 차선변경 파라미터를 기본값으로 뒀다. 그대로 돌리니 양보 결정이 "아니오"인 차량도 SUMO가 간격을 내줘 실측 양보율이 설정 0.82가 아니라 **1.00**이었다. 양보형도 `lcCooperative=-1`, `lcCooperativeSpeed=0`으로 두자 일치했다.
2. **`lcCooperativeHelpTime`은 설정하지 않는다.** SUMO 1.27.1의 LC2013에서 `laneChangeModel.lcCooperativeHelpTime` 설정이 오류로 거부된다.
3. **`openGap`의 `newSpaceHeadway`는 0 이상이어야 한다.** -1은 오류. 0을 쓰고 시간 차간(`newTau`)만 지정한다.
4. **openGap duration 12 s.** 8 s로는 목표 차간에 도달하지 못해(아래) 12 s로 했다.
5. **SUMO_HOME이 이 PC에서 잘못 설정돼 있다**(`C:`, 실제 `D:`). `sumo_conn.py`가 PATH의 `sumo`로 보정한다.

## 관문 1 항목별 결과

| 항목 | 결과 |
| --- | --- |
| A1 양보율 (유형별 200회, 설정 확률 ±0.05) | 아래 표. 두 유형 모두 통과 |
| A1 유발 감속이 유형 간 다름 | 평균 최대 감속 양보형 ≈1.47 m/s², 비양보형 ≈0.29 m/s². 평균 차이 순열 검정 p < 0.001 |
| A1 누출 (양보 확률 0, 유형별 200회, 기준 5%) | 양보형 0%, 비양보형 0% (수정 전 양보형은 100%) |
| openGap 시계열 | t=6.0 s에 발행, 간격 16.9 m → 약 24.5 m. 목표(`newTau`×9 m/s = 24.9 m)의 ±0.5 m 안에 발행 후 8.6 s에 도달 (duration 12 s 안) |
| A2 S5 진입률 | 아래 표 |
| 충돌 기록: 차로 위 | 정지한 선행차에 speedMode 0 차량이 추돌 → 기록됨 (pytest) |
| 충돌 기록: priority 교차로 | 버스 speedMode 0 + 끼어들기 p=1에서 60회 중 18%에서 기록됨 |
| 충돌 기록: 차량-보행자 | `ped_force`(우선권 무시)와 speedMode 0 버스가 T=6 s 간격에서 충돌 → 기록됨 (`--intermodal-collision.action warn`) |
| 동일 시드 재현 | TraCI로 같은 시드 두 번 돌려 결과 동일 (pytest) |
| libsumo = TraCI 일치 | **검증하지 못함.** conda `sumo` 환경(Python 3.9.0)에서 pip `libsumo` 1.27.1이 `DLL load failed`로 import되지 않는다. 시험 설치를 되돌렸다. 환경 문제이며 알고리즘 문제가 아니다 |
| SUMO 버전 고정, sumocfg 옵션 일괄 반영 | 코드에서는 `traffic/sumo_conn.py` 인자로 고정(1.27.1). 합정역 sumocfg 파일 자체는 수정하지 않았다 |

### A1 양보율 (200회, seed 0~199)

| 유형 | 설정 확률 평균 | 결정 비율 | 실측 양보율 | 결정과 결과 일치 | 평균 최대 감속 | ±0.05 |
|---|---|---|---|---|---|---|
| 양보형 | 0.821 | 0.785 | 0.785 | 1.000 | 1.47 | 통과 |
| 비양보형 | 0.171 | 0.155 | 0.155 | 1.000 | 0.29 | 통과 |


### A2 S5 끼어들기 진입률

진입 = 버스가 교차로까지 28 m(약 2.5 s) 이내일 때 골목 차량이 교차로에 들어옴. 골목 차량이 정지선에 도착할 때 버스가 15~40 m 앞이 되도록 투입 시각을 흩뜨렸다(60회/점).

| jmIgnoreFoeProb | 0.0 | 0.05 | 0.1 | 0.2 | 0.3 | 0.5 | 1.0 |
|---|---|---|---|---|---|---|---|
| 진입률 | 0.00 | 0.13 | 0.23 | 0.33 | 0.45 | 0.47 | 0.47 |

6구간 vTypeDistribution 전체 진입률 0.42, p=1이지만 jmIgnoreFoeSpeed=0일 때 0.00.

- `jmIgnoreFoeProb`가 커질수록 진입률이 단조 증가한다. 0.5 이상에서는 약 0.47로 포화되는데, 투입 시각 분포 중 도착 시 버스가 28 m 안에 있는 비율이 그 정도이기 때문이다(구조적 상한).
- **`jmIgnoreFoeProb`는 진입 확률 그대로가 아니다.** 설정 0.05가 0.13으로 나왔다. 스텝마다 다시 뽑는 것으로 추정된다(확인하지 않음). 그래서 구간별 p는 이 곡선으로 보정해야 한다.
- `jmIgnoreFoeSpeed` 기본값(0)이면 p=1이어도 달리는 버스를 무시하지 않는다(진입 0%). 공식 문서와 일치.
- S4: 정차 택시(onRoad parkingArea)가 40 s 서 있는 동안 버스는 뒤에서 31.5 s 멈췄고(최소 간격 2.5 m), 추월하지 않았으며, 택시가 떠난 뒤 다시 달렸다. **1차로에서는 정차 차량이 버스를 막는다.** 다차로에서의 회피는 확인하지 않았다.

### A3 보행자

버스가 횡단보도에 닿기까지 남은 시간 T와 버스 속도 v를 바꿔 "버스가 지나가기 전에 보행자가 건너기 시작했는가"를 봤다(v_low = 1.5 m/s, 보행자는 횡단보도 3.8 m 앞에서 출발).

| 유형 | 버스 속도 | T=2s | T=3s | T=4s | T=5s | T=6s | T=8s | T=10s |
|---|---|---|---|---|---|---|---|---|
| ped_p1 | v=1.0 | 기다림 | 기다림 | 기다림 | 기다림 | 건넘 | 건넘 | 건넘 |
| ped_p1 | v=2.0 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 |
| ped_p1 | v=5.0 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 |
| ped_p1 | v=10.0 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 |
| ped_p2 | v=1.0 | 기다림 | 기다림 | 기다림 | 기다림 | 건넘 | 건넘 | 건넘 |
| ped_p2 | v=2.0 | 기다림 | 기다림 | 기다림 | 기다림 | 건넘 | 건넘 | 건넘 |
| ped_p2 | v=5.0 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 | 건넘 | 건넘 |
| ped_p2 | v=10.0 | 기다림 | 기다림 | 기다림 | 기다림 | 기다림 | 건넘 | 건넘 |


- P1은 v = 1.0(≤ v_low)일 때만 버스 앞에서 건넜고 v ≥ 2.0에서는 T = 10 s(20 m 이상 여유)에서도 기다렸다. P2는 v = 2.0에서도 T ≥ 6 s에서 건넜다. 유형 차이가 의도대로 나온다.
- 정지한 버스(v=0) 앞에서는 P1·P2 모두 건넜다.
- 한계: v = 1.0에서 T ≤ 5 s이면 P1도 건너지 않았다. 보행자가 정지선에 닿기 전에 버스가 먼저 지나가서 "버스 통과 전 진입"으로 세지 않은 것이 주된 이유로 보이나 확인하지 않았다. P2의 수용 시간 간격(약 6~8 s)을 inD 수용 간격 분포와 대조하는 일은 하지 않았다(자료 없음).

## 하지 않은 것

- 버스 제어 측과의 연동(관문 2), 버스 laneChangeMode 값으로 누출 재검사, 지표 코드, 합정역 지도 위 배치.
- 배이형/차로 정차형 정류장 구분, 승객 가중 지연, 보행자 PET.
- A1 비양보형의 `setAcceleration` 간격 좁히기는 코드(`p_close_gap`)만 있고 검증하지 않았다(기본 0).
- S5 `speedMode 55` 강제 진입 경로는 넣지 않았다(진입률이 목표에 못 미칠 때만 필요하다고 계획에 적혀 있는데 목표값이 정해지지 않았다).
- 양보 확률·반응 지연 분포는 계획의 초깃값이며 실제 자료로 보정하지 않았다.
