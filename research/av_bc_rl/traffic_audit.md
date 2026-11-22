# traffic/ 억지 시나리오 코드 감사

2026-11-22, main `2ca0f86` 기준. 읽기만 했고 고친 것은 없다. 행 번호는 그 커밋의 파일이다.

## 요약

시나리오를 만들려고 주변 차량과 보행자를 버스 위치에 맞춰 조작하는 코드가 다섯 곳에 몰려 있다.

- `traffic/scenario_injector.py`, `traffic/stop_ahead.py`: 버스 위치와 도착 시각에 맞춰 차를 넣는다.
- `traffic/yield_logic.py`, `traffic/stop_bypass.py`: 운전자 모델 대신 `setSpeed`와 차로 고정으로 움직인다.
- `traffic/ped_defiance.py`: 보행자가 시험용 버스에게만 무시하고 건넌다.
- `traffic/presence.py`, `traffic/person_spawner.py`, `traffic/interior_spawner.py`: 버스 근처에서 차와 사람을 숨기거나 미루거나 지운다.
- `traffic/test_bus.py`: 버스가 각본상의 상대를 기다리며 천천히 가거나, 달리는 차로에서 멈춘다.

학습 환경(`env/sumo_env.py`)은 `traffic/`을 쓰지 않는다. 학습과 데이터 수집의 배경 교통은 map.json의 흐름과 네 가지 차량 추종 모델이라 자연스럽다. 억지 코드는 `scripts/run_hapjeong_scenario.py`, `scripts/measure_s1_s6.py`로 보는 시나리오 화면에서만 돈다.

SUMO 1.27.1 문서 기준 확인: 기본 speedMode 31은 bit6이 0이라 제한속도를 지킨다. 그래서 "제한속도 + 8m/s" 따라잡기는 실제로 과속하지는 않지만, 차가 버스를 향해 목표 지향적으로 움직인다.

## 찾은 것

| 위치 | 하는 일 | 심각도 | 자연스러운 대안 |
|---|---|---|---|
| `scenario_injector.py:215-242` | S1: 매 바퀴 정류장 하나에서 버스 뒤 220m에 차를 넣는다 | 중 | 삭제. 수요에서 생기게 둔다 |
| `scenario_injector.py:296-350, 253-262` | S2/S3: 창 300m 전에 버스 뒤 65m에 차를 넣거나 근처 차를 골라 `setLaneChangeMode(0)`으로 묶는다 | 상 | 삭제 |
| `scenario_injector.py:441-483` | 상대 차를 버스 속도 +2~8m/s로 따라붙게 하고, 0~5.5m 뒤에서는 버스 속도에 맞춰 붙어 다니게 한다 | 상 | 삭제 |
| `scenario_injector.py:361-394` | 버스가 적색에 서 있으면 목표 차로 60, 74, 88m 뒤에 세 대를 넣는다 | 중상 | 삭제. 적색 대기열은 수요로 생긴다 |
| `scenario_injector.py:503-545, 159-188` | S5: 골목 두 곳 중 1~2곳에서, 버스보다 4초 먼저 합류점에 닿을 위치를 계산해 차를 넣고 버스만 무시(`ignoreIDs=bus0`)하게 한다 | 상 | 모든 골목 흐름에 `impatience`, `jmTimegapMinor` 분포를 준다(`jmIgnoreFoeProb`=0) |
| `scenario_injector.py:548-576` | S6: 버스가 오면 횡단보도 8m 앞에 모든 차를 무시하는 보행자를 넣는다 | 상 | 삭제. 시간당 240명의 S6 보행자가 이미 있다 |
| `stop_ahead.py:215-251` | S4: 버스보다 3~25초 먼저 정류장에 닿도록 골목에서 택시를 만들어 정류장 앞에 15~30초 세운다 | 상 | 버스와 무관한 포아송 택시 흐름에 정류장 근처 `<stop>`(`cutin.py:55-63`의 쓰지 않는 도우미) |
| `yield_logic.py:214-257` | A1: 목표 차로 후행차를 차로에 묶고 양보형은 버스-2m/s, 비양보형은 버스+2m/s로 속도를 지정한다 | 상 | 속도 지정 삭제. 운전자마다 `lcCooperative`, `lcSpeedGain`, `lcAssertive`, `speedFactor`를 연속 분포로 뽑는다 |
| `yield_logic.py:41-52` | 모든 차의 협조 차선 변경을 끈다(`lcCooperative=-1`) | 중 | 위 분포로 바꾼다 |
| `stop_bypass.py:79-145` | S1: 정차한 버스 뒤 200m 안의 차를 기다리기/추월로 정해 묶거나 1초마다 `changeLane` | 중 | 삭제. SUMO 차선 변경 모델이 스스로 추월한다 |
| `ped_defiance.py:138, 248-275` | 보행자가 시험용 버스에게만 무시하고 건넌다 | 중상 | 모든 보행자에 `impatience`, `jmIgnoreFoeProb` 분포 |
| `presence.py:130-176` | 버스 50m 안에서 출발하는 차를 지우고 목적지를 바꾼다 | 하 | 삭제(측정은 남긴다) |
| `person_spawner.py:50-110` | 버스 55m 안 보행자의 출발을 미루고, 걷기가 끝나면 버스가 멀어질 때까지 세워 둔다. 원래 보행자 경로는 지운다(`hapjeong.py:156-160`) | 중 | SUMO가 원래 보행자 경로대로 만들게 둔다 |
| `interior_spawner.py:43-61` | 입구 수요 30%를 버스 200m 밖의 경로 위 임의 지점에 넣는다 | 하 | 버스와 무관한 `departPos="random_free"` 흐름 |
| `hapjeong.py:49, 210-236` | 골목 흐름의 끼어들기 유형 지정을 끄고 하드코딩한 두 골목만 주입한다 | 중 | 신호 없는 합류 골목 전부에 위의 운전자 분포 |
| `test_bus.py:295-302` | 버스가 각본 상대를 기다리며 제한속도의 45%로 줄인다 | 중상 | 주입기를 지우면 꺼진다 |
| `test_bus.py:326-356` | 창 끝에서 달리는 차로에 `setSpeed(0)`으로 멈춰 차선 변경을 1초마다 다시 요청한다 | 중상 | 창을 앞당기거나 감속만 한다 |
| `network.net.xml` S6 횡단보도 | 보행자 링크가 비우선(`m`). 한국 법규(차량 일시정지)와 반대 | 중 | 우선 횡단보도로 두고 운전자 `jmCrossingGap` 분포 |
| `right_turn.py:78-123` | 적색 중 우회전을 `g`로 연다 | 하 | 정지 후 진행 `s` |

## 남길 것(자연스러움)

- `env/`의 `carDist`: Krauss, IDM, EIDM, ACC 혼합, speedDev, SL2015 횡방향 모델, 포아송 도착.
- `bus_traffic.py`의 전용 차로 버스 노선과 포아송 택시.
- 보행자 경로 파일의 일반 보행자 600명/시와 S6 보행자 240명/시, P1/P2 유형의 SUMO 기본 파라미터.
- `alleys.py`의 합류 골목 탐지, `cutin.py`의 도우미, `metrics.py`, `PresenceMonitor`(측정용).
- map.json의 정적 수요 배율(`alley_inflow_x3`, `s6_walkers_x4`, `sangsu_exits_x2`). 보고서에 밝힌다.

## 최소 수정안

1. 주입기와 S4 주입을 기본에서 끄고 지운다(테스트 포함). 버스의 상대 대기(H1)도 함께 꺼진다.
2. A1과 S1은 관찰만 한다. 속도 지정과 차로 고정을 지우고, 출발할 때 운전자 파라미터를 연속 분포로 뽑는다.
3. S5는 모든 신호 없는 합류 골목 흐름에 운전자 분포를 준다. 버스만 무시하기와 두 골목 목록을 지운다.
4. S4는 버스와 무관한 포아송 택시 정차로 만든다.
5. 보행자는 버스만 무시하기를 지우고 모든 보행자에 impatience 분포를 준다.
6. 버스 근처 생성·삭제를 지운다. 측정만 남긴다.
7. `measure_s1_s6.py`의 S1~S4를 행위자와 무관한 기하 판정으로 바꾸고 seed 여러 개의 바퀴당 비율과 구간으로 보고한다.

자연 발생률은 보장이 아니라 측정값이다. 주입기 전 측정(`PROBLEM_SOLVING_REPORT.md:125-134`)과 지도 검증(busy 80회, `maps/hapjeong/README.md:244-247`)에서 S1, S5, S6은 자주, S2/S3의 차로 변경 상대는 드물게 나왔다.
