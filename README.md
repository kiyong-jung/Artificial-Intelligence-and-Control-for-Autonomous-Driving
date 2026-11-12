# 합정역 자율주행버스 도로 (SUMO)

서울 합정역 일대 실제 도로를 SUMO로 옮긴 팀 프로젝트용 도로다. 연구 주제는 자율주행버스가 일반 차로의 가로변 정류장을 오가며 겪는 상호작용(정류장 출발 후 재합류, 정류장 직후 좌회전 차로 변경, 신호 없는 횡단보도 등)이다. 도로망은 서예인이 만들었고, 실제 지도와 대조해 고친 내용은 [maps/hapjeong/README.md](maps/hapjeong/README.md)에 있다.

![합정역 일대 지도(왼쪽)와 같은 범위에 겹친 SUMO 도로망(오른쪽, 붉은색)](maps/hapjeong/references/hapjeong_road.png)

연구 경로는 합정역에서 독막로, 상수역, 와우산로, 잔다리로, 서교동사거리를 돌아 합정역으로 오는 순환 경로다(`bus_loop`, 1,970m). 경로 위에 실제 위치의 가로변 정류장 6곳과 신호 없는 횡단보도 1곳이 있다. 독막로 양쪽 골목은 이어져 있고, 골목이 버스 경로에 신호 없이 합류하는 곳이 네 곳이다(독막로 두 곳, 잔다리로 두 곳). 3.3.0에서 골목 합류점은 여덟 곳이다(독막로 두 곳, 와우산로 한 곳, 잔다리로 다섯 곳). 모두 실제 골목이고 측정으로 골랐다.

![연구 경로(파랑), 가로변 정류장(노랑), 신호 없는 횡단보도(초록)](maps/hapjeong/references/hapjeong_scenario.png)

## 설치

Python 3.10과 SUMO 1.27.1에서 확인했다. SUMO는 따로 설치하고 `SUMO_HOME`을 설정한다. 패키지는 SUMO를 쓸 가상환경 하나에 설치한다.

```bash
git clone https://github.com/Twave1717/Artificial-Intelligence-and-Control-for-Autonomous-Driving.git
cd Artificial-Intelligence-and-Control-for-Autonomous-Driving
conda create -n sumo-rl python=3.10
conda activate sumo-rl
python -m pip install -r requirements.txt
```

`ModuleNotFoundError: No module named 'sumolib'`가 나오면 패키지를 설치하지 않은 환경(예: conda `base`)에서 실행한 것이다. `conda activate sumo-rl` 뒤에 다시 실행한다.

도로를 읽을 때 `network.net.xml`의 sha256을 검사한다. `.gitattributes`가 XML의 줄바꿈을 바꾸지 않게 하므로 Windows에서도 git으로 받으면 된다.

## 도로 보기

```bash
python view_road.py --map hapjeong                    # sumo-gui로 연다. ▶를 눌러 재생한다
python view_road.py --map hapjeong --scenario busy    # 교통량: quiet, baseline(기본), busy
python view_road.py --map hapjeong --netedit          # netedit로 도로 구조만 본다
```

`--map`을 생략하면 팔달문 도로가 열린다(부록).

## 도로 고치기와 검증

합정역 실행 도로망은 서예인의 원본(`maps/hapjeong/source/network.base.net.xml`)에 `scripts/build_hapjeong.py`가 수정을 적용해 만든다. 고친 뒤 다시 만들고 검증한다.

```bash
python scripts/build_hapjeong.py
python scripts/validate_road.py --map hapjeong --out maps/hapjeong/references/stability_validation.json
```

같은 입력이면 같은 파일이 나온다. 가장 최근 검증(seed 0·7·42 × quiet·baseline·busy, 1,800초)에서 충돌과 순간이동은 0이고, 버스 ego는 9회 모두 `bus_loop`를 완주했다.

충돌이나 정체가 어디서 났는지 보려면 `scripts/audit_road.py`를 쓴다. 배경 교통에 버스를 120초마다 넣고 충돌, 순간이동, 비상 제동, 비상 정지를 차로별로 모은다. 연결이 없는 차로와 버스 경로의 차로 연결도 적는다.

```bash
python scripts/audit_road.py --map hapjeong --out maps/hapjeong/references/road_audit.json
```

## 학습, 평가, 데이터 수집

합정역의 ego는 버스다(길이 12m, 폭 2.5m, 가속 1.0m/s², 감속 2.0m/s², 정원 40명). 제원은 `env/road_config.py`의 `_EGO_VEHICLES`에 있고 `map.json`의 `ego_vehicle`이 고른다. 환경은 버스가 다닐 수 없는 차로(인도)를 없는 차로로 본다.

```bash
python collect_pomdp_data.py --map hapjeong --policy route-follow --episodes 10 --max-steps 1200 --out-dir data/hapjeong --name run1
python scripts/export_csv.py data/hapjeong/run1.jsonl.gz --rows 15 --out data/submission/00조_데이터예시.csv
python scripts/fill_column_slide.py 00조_칼럼설명.pptx data/submission/00조_칼럼설명.pptx
```

- 컬럼은 `env/data_schema.py`의 표 하나에 있다. 수집기의 metadata, CSV, 수업 양식이 이 표에서 나온다. 수업의 필수 항목 19개에 `sim_time`, `detected_pedestrians`, `event.termination_reason`, `comfort.*` 4개를 더했다.
- `state`는 `ego`, `vehicles`, `pedestrians`, `navigation`, `traffic_light`로 나뉜다. 주변 차량과 보행자는 ego에서 150m 안만 적는다(`mdp_config.py`의 `state_radius`).
- `detected_vehicles`는 관측 차선의 차량과 ego에서 50m 안의 차량이다. 관측 차선 밖의 차량(교차 도로, 반대 방향)은 `lane_relation`이 `other`다. `detected_pedestrians`는 50m 안의 보행자다.
- 버스 승차감은 `state.ego.acceleration`, `state.ego.jerk`와 `comfort.peak_acceleration`, `comfort.peak_jerk`로 남는다. 한계(가속 1.5m/s², jerk 2.0m/s³)를 넘은 스텝은 `comfort.*_violation`이 참이다.
- `export_csv.py`는 모든 행에 표의 컬럼이 있는지, 가속도가 제원의 한계 안인지 검사한 뒤에만 CSV를 쓴다.

아직 없는 것은 다음과 같다. 버스는 정류장에 서지 않는다(`state.navigation.distance_to_stop`으로 거리만 적는다). 관측은 수업 원본의 31차원 그대로라 보행자, 신호, 방향지시등은 정책 입력에 없고 `state`와 감지 목록에만 있다. 3.3.0 도로망의 busy 80에피소드에서 골목 차량이 버스 앞에 끼어든 것은 36번이고, 버스가 왼쪽 차로로 옮길 때 그 차로에 차가 있던 것은 27번이다(3.0.0에서는 버스가 다른 차와 거의 마주치지 않았다). 구간별 수치는 `python3 scripts/scenario_sections.py <jsonl.gz>`로 본다. 버스는 서교동사거리 좌회전 신호를 길게 기다려, 제한 시간 500초를 넘기는 에피소드가 열에 한둘 나온다([maps/hapjeong/README.md](maps/hapjeong/README.md)의 알려진 현상). 시나리오별 발생은 `python3 scripts/scenario_occurrence.py <jsonl.gz>`로 세고, 시나리오의 재료가 도로에 얼마나 공급되는지는 `python scripts/scenario_supply.py`로 센다. 학습 쪽 사용법은 [부록 B](#부록-b-학습-평가-데이터-수집)에 있다.

## 주변 교통(Traffic)

자율주행버스 주변의 다른 차량과 보행자가 버스를 마주했을 때 어떻게 반응하는지를 SUMO에서 구현하고, 설정한 대로 나오는지 숫자로 확인한다. 버스 주행 정책은 범위 밖이다. 구조, 사용법, 검증 수치, 이슈는 [traffic/README.md](traffic/README.md)에 있다.

- 구현: 뒤차의 양보/비양보(A1), 골목 끼어들기(S5), 정류장 앞 택시(S4), 횡단보도 보행자(S6), 평가 지표. 배경 차량의 주행 모델(Krauss, IDM, EIDM, ACC)과 양보 성격을 독립으로 붙인다.
- 확인된 것: 합정역 배경 교통 속에서 양보율이 설정 확률과 맞는다(양보형 0.785, 비양보형 0.155). 합정역 배경 교통에 켜고 돌린 짧은 실행에서 충돌과 순간이동은 없었다.
- 눈으로 보기: `python scripts/run_hapjeong_scenario.py`. 빨간 버스가 `bus_loop`를 돌고, 버스를 마주친 뒤차에 양보(초록)/비양보(빨강) 고리가 생긴다. 빠른 확인은 `pytest tests`다.
- 한계: `traffic/`은 학습 환경에 연결되어 있지 않고 버스 정책도 없다(`bus_hook`과 규칙 버스로만 확인). S4~S6가 시나리오에서 실제로 발생하는지는 확인하지 못했고 S1은 구현하지 않았다. 검증 수치는 2.0.0 도로망에서 잰 값이다. 3.2.0 도로망에서는 `pytest tests` 통과만 확인했다.

### Todo

- [ ] 시나리오 발생 계측, S5·S6·S4 주입기
- [ ] 정류장 형태 확정 후 S1·S2 구현
- [ ] 도로 수정 후 합정역 검증 재실행
- [ ] `SumoHighwayEnv`에 주변 교통 연결
- [ ] 버스 ego의 정류장 정차, 관측에 보행자·신호 추가
- [ ] 실제 버스 정책과 `bus_hook` 연결

## 함께 작업하기

`main`에 바로 push하지 않는다. 브랜치를 만들어 작업하고 GitHub에서 PR을 열어 합친다.

```bash
git switch -c <브랜치 이름>
git push -u origin <브랜치 이름>
```

도로를 고쳤다면 PR을 열기 전에 `scripts/validate_road.py`를 돌려 `통과`를 확인한다.

## 출처

도로 자료와 README의 지도 그림은 © OpenStreetMap contributors, ODbL 1.0이다.

## 부록 A. 팔달문 도로

수원 팔달문 원형 차도를 중심으로 정조로, 행궁로, 수원천로 양쪽, 창룡대로, 중부대로와 그 사이 골목 중 팀이 지도에 표시한 도로만 남겼다. 신호는 없다. 지역 간 일관성 확인용 2차 대상지이고, 지금 학습 환경의 기본 지도다(`--map` 생략).

- 도로 ID, 경로, 정류장: [maps/paldalmun/README.md](maps/paldalmun/README.md)
- 도로를 이렇게 고른 이유와 측정 결과: [maps/paldalmun/DESIGN.md](maps/paldalmun/DESIGN.md)

![팔달문 일대 지도(왼쪽)와 같은 범위에 겹친 SUMO 도로망(오른쪽, 붉은색)](maps/paldalmun/references/paldalmun_road.png)

남길 도로는 `maps/paldalmun/source/drawn_edges.json`의 목록이고, ego 경로는 `scripts/build_road.py`의 `ROUTES`다.

```bash
python view_road.py
python scripts/build_road.py
python scripts/validate_road.py --seeds 0 7 42 --scenarios quiet baseline busy --seconds 1800 --out maps/paldalmun/references/stability_validation.json
```

가장 최근 검증에서는 45회 모두 통과했고, 배경 차량의 충돌과 순간이동은 0이었다.

## 부록 B. 학습, 평가, 데이터 수집

```bash
python train.py --scenario baseline --seed 42
python test.py MODEL --nogui --episodes 10
python collect_pomdp_data.py --policy route-follow --episodes 10 --out-dir data/paldalmun --name run1
python train_bc.py --data data/paldalmun/run1.npz --epochs 50
```

- ego의 기본 경로는 팔달문 `r0`다. 다른 경로는 `--ego-route s_left_turn`처럼 고른다.
- ego는 0.5초마다 행동을 정하고 SUMO는 0.1초 간격으로 계산한다. 에피소드는 최대 1,000번 결정(500초)이다.
- `test.py`는 같은 폴더에 `run_config.json`이 있는 모델만 평가한다. 옵션을 주지 않으면 학습 때의 설정으로 평가한다.

수업 원본 코드와 다르게 동작하는 부분은 다음과 같다.

- SUMO 계산 간격 0.1초. 0.5초로 계산하면 교차로와 합류부의 대기열이 달라진다.
- ego 안전 보조가 켜져 있다(`speed_mode=31`, `lane_change_mode=512`). 끌지는 팀이 정한다. 값은 `env/road_config.py`의 `_EGO`에 있다.
- ego를 넣기 전에 배경 교통을 180초 흘린다.
- `blocked_penalty`가 0이다. 교차로에서 양보하며 기다리는 것까지 벌점을 받지 않게 했다.
- ego 투입 시각을 에피소드마다 0~180초 흩뜨린다. 배경 차량이 일정한 간격으로 나와, 투입 시각이 같으면 ego가 매번 같은 차를 같은 자리에서 만난다.
- 가감속 행동은 결정 구간(0.5초) 내내 같은 가속도로 적용한다. 목표 속도를 한 번에 주면 첫 계산에서 최대 가속으로 따라붙어 jerk가 행동과 무관하게 튄다.
- 차로 유지 행동은 SUMO에 명령을 보내지 않는다. 차선 변경 벌점과 `lane_changes`는 실제 차선 이동을 센다.
- 관측의 차로 연결성은 ego 경로 방향으로 이어지는 거리만 센다.
- PPO의 GAE는 시간 초과에서도 끊는다. ego가 300초 서 있다 순간이동하면 에피소드는 `teleport`로 끝난다.
