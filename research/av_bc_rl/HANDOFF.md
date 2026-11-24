# 인수인계: 합정역 버스 AV 연구 (BC, PPO, Residual RL)

이 문서는 다른 컴퓨터의 에이전트가 `git clone` 한 번으로 이 연구를 이어받기 위한 것이다. 브랜치는
`research/av-bc-rl`이고, 모든 결과와 결정은 이 폴더(`research/av_bc_rl/`)에 있다.


## 2026-11-24 상태(먼저 읽기)

아래 본문은 11/22까지의 기록이라 지금과 다른 곳이 많다. 지금 상태와 과정은 `reports/progress_log.md`에 있다.

- 환경은 수업 기준으로 바뀌었다. ego 안전장치 끔(speedMode 0, laneChangeMode 0), 관측 43개, 지도 3.20.0.
- 제출할 버스는 규칙 없는 신경망이다. 규칙 운전사(`expert-v6`)는 정답을 만들 때만 쓴다. 학습 방법과 명령은 `research/av_bc_rl/bc_redlight.md`.
- 지금 가장 좋은 모델: `research/av_bc_rl/models/v6_bins_dagger_s{0,1,2}/`. 최종 seed 180번 중 176번 완주(충돌 1, 신호 위반 3). 새 seed 60번 직접 확인에서는 56번.
- 끊긴 추가 학습을 잇는 명령: `bash research/av_bc_rl/resume_dagger.sh`.
- 예전 46차원 모델(bc_s*, dagger_s*, ppo_s*, residual_s*)은 더는 불러올 수 없다. Residual RL은 수업 규칙에 맞지 않아 결과에서 뺀다.

## 지금까지 한 일

1. **측정부터.** 버스 행동 지표(`behavior_metrics.py`)를 만들고 옛 규칙 정책을 쟀다. 적색 정지선 앞에서
   SUMO가 버스를 최대 80m/s²로 강제 정지시키고(바퀴당 1.8회), SUMO가 31% 스텝에서 속도를 바꿨으며,
   기록된 행동과 실제 움직임이 8% 어긋났다. 정류장에는 한 번도 서지 않았다. 원인은 관측(31차원)에 신호와
   정류장이 없고 규칙이 늘 같은 가속(0.25)을 내는 것이었다.
2. **환경.** 관측에 버스가 실제로 알 수 있는 경로 정보 15개를 붙였다(46차원). 정류장 정차(s2, s4, s5, s6에
   8초)를 환경이 센다. 가속도는 구동계처럼 결정 구간 동안 곧게 바뀐다. 차선변경 모드를 512에서 1536으로
   바꿔 버스가 차로 가운데로 다시 정렬한다(512에서는 인도에 걸쳐 멈추고 순간이동했다).
3. **전문가 규칙(`env/route_policy.ExpertPolicy`, v4).** 관측만 쓰는 IDM + 가상 정지점. busy dev 30회에서
   급제동 0, SUMO 개입 1.3%, 불일치 0.21%, 정류장 118/120. 관측만 쓰는지 48,176 샘플로 검증(차이 0).
   `ExpertPolicyV5`는 차선변경과 앞차 추종을 SUMO가 실제로 지키는 안전 간격에 맞춰 정류장 120/120, 개입 0.89%다.
   BC와 Residual은 v4로 학습했으므로 `make_expert`의 기본은 4다.
4. **억지 교통 코드와 순간이동.** 버스 위치에 맞춰 배경 차와 보행자를 만들고 세우던 코드를 지웠다(`traffic_audit.md`).
   순간이동은 모든 경로에서 끈다(`time-to-teleport -1`). 막히면 감추지 않고 `stuck`으로 센다.
5. **검사 루프(사용자 요청, 2회 이상).** `scripts/audit_naturalness.py`가 정적 검사(배경 행위자에게 가는 TraCI 명령,
   위험한 운전자·교차로 속성, 고정 시각 수요)와 동적 검사(순간이동, 충돌, 비상 제동, 오래 선 차, 버스 가까이 출현)를
   한다. 1회차(`naturalness_audit_round1.md`) 뒤 환경, 주변 교통, 지도 수요를 고쳤고 2회차는 다른 모델이 다시 봤다
   (`naturalness_audit_round2.md`). 지도는 3.14.0이 됐다(보행자 포아송 도착, 골목 고리 경유 흐름 제거).
6. **학습.** BC(전문가 시연), DAgger, PPO(처음부터), Residual RL(전문가 가감속 + 학습한 보정)을 같은 관측과
   보상으로 학습하고, `evaluate.py`로 final seed에서 닫힌 루프 평가했다. 결과는 `report.md`. 모델은 `models/`.

## 다른 컴퓨터에서 시작하기

```bash
git clone https://github.com/kiyong-jung/Artificial-Intelligence-and-Control-for-Autonomous-Driving.git
cd Artificial-Intelligence-and-Control-for-Autonomous-Driving
git switch research/av-bc-rl
conda create -n sumo-rl python=3.10 && conda activate sumo-rl
python -m pip install -r requirements.txt tensorboard
# SUMO 1.27.1을 설치하고 SUMO_HOME을 설정한다(README의 설치 절). 또는 pip install eclipse-sumo==1.27.1
python -m pytest -q                       # 이 브랜치에서 통과해야 한다
python research/av_bc_rl/agsci_task/evaluate.py --results_dir /tmp/eval_expert   # 전문가 점수, 약 3분
```

데이터(`data/`)와 학습 결과(`results/`)는 git에 없다. 아래 명령으로 다시 만든다. 학습한 모델 파일은
`research/av_bc_rl/models/`에 넣어 두었다.

## 다시 만드는 명령

```bash
PY=python
# 전문가 시연(train seed): 교통량마다 60에피소드
for sc in quiet:2000 baseline:2300 busy:2600; do
  $PY collect_pomdp_data.py --map hapjeong --scenario ${sc%%:*} --seed ${sc##*:} --policy expert \
      --episodes 60 --workers 6 --max-steps 1400 --out-dir data/research --name train_expert_${sc%%:*}
done
$PY research/av_bc_rl/merge_datasets.py data/research/bc_train.npz data/research/train_expert_{quiet,baseline,busy}.npz
# BC
$PY train_bc.py --data data/research/bc_train.npz --epochs 50 --seed 0 --device cpu --out-dir results/research/bc_s0
# DAgger(BC가 달린 관측에 전문가 라벨, 2번 반복). 전문가 v5로 다시 모으려면 위 수집에서 --policy expert-v5
$PY research/av_bc_rl/dagger.py --data data/research/bc_train.npz --model results/research/bc_s0/model.pt \
    --iterations 2 --episodes 20 --out results/research/dagger_s0 --seed 0
# PPO, Residual (busy, 학습 seed 셋)
$PY train.py --map hapjeong --scenario busy --seed 2101 --timesteps 600000 --eval-interval 0 --max-steps 1400 \
    --results-dir results/research --run-label ppo_s2101
$PY train.py --map hapjeong --scenario busy --seed 2201 --timesteps 300000 --eval-interval 0 --max-steps 1400 \
    --residual-scale 0.3 --results-dir results/research --run-label residual_s2201
# 평가(dev로 고르고, final은 마지막에 한 번)
bash research/av_bc_rl/run_final_eval.sh          # 모든 정책 final 평가와 표(models/의 모델)
$PY research/av_bc_rl/failure_cases.py --split final --out research/av_bc_rl/report/final_failures.md
```

## AI-Scientist-v3로 이어가기

과제 폴더는 `research/av_bc_rl/agsci_task/`다(task.md, evaluate.py, program_0, rubrics.json, requirements.txt).
평가기는 `--program_path --results_dir`를 받아 `metrics.json`(combined_score)과 `correct.json`을 쓴다.
후보 프로그램은 `make_policy(env)`를 정의한다. 기준(program_0)은 전문가다.

1. AI-Scientist-v3를 받고(`git clone https://github.com/SakanaAI/AI-Scientist-v3`) README대로 Docker 이미지와
   `uv sync`를 준비한다.
2. 과제 폴더를 `examples/hapjeong_bus/`로 복사한다. 평가기는 이 저장소를 `HAPJEONG_REPO` 환경 변수로 찾는다.
   컨테이너 안에서 돌리면 저장소를 마운트하고 그 경로를 `HAPJEONG_REPO`로 넘긴다. SUMO는 컨테이너에
   `pip install eclipse-sumo==1.27.1`로 넣는다(requirements.txt). 리눅스 ARM에서 이 바퀴(wheel)가 되는지는
   확인하지 못했다.
3. `agsci doctor examples/hapjeong_bus`와 `agsci run examples/hapjeong_bus --dry-run`으로 먼저 점검한다.
   평가기는 결정적이어야 한다(같은 seed, 같은 점수). 이 Mac에서 마지막 환경(지도 3.14.0, 검사 루프 뒤)으로 전문가를
   두 번 돌려 97.22111290315144로 같았고 metrics.json 전체가 같았다(검사 루프 전에는 96.88373764527032). 다른 컴퓨터(리눅스, 다른 SUMO 빌드)에서는 값이 조금 다를 수 있으니
   거기서 한 번 더 두 번 돌려 본다.
4. CPU 과제다(`container.gpu_count=0`, `investigation.execution.gpu_evaluation=false`). 평가 한 번에 약 3분이다.

## final 평가 뒤에 바꾼 환경 (11/23)

`report.md`와 `metrics/*_final.json`은 이 변경 전 환경에서 잰 값이다. 관측의 뜻이 바뀌었으므로(아래 2, 3) 학습 정책은
다시 모으고 다시 학습한 뒤 비교한다. 앞뒤 수치는 `metrics/curve_merge_{before,after,after_mode38}_dev.json`과
`decisions.tsv`에 있다(전문가 v5, dev 교통량마다 20회).

1. **선 차의 옆 이동을 다시 0으로, 버스는 차로 끝에서 요청으로 끼어든다.** `lcMaxSpeedLatStanding` 0.3은 대기열의 배경 차를
   옆으로 밀어 30~40도 돌렸다(busy seed 2600에서 도로 위 26대, 지금 0대). R111b 끝 갇힘의 실제 원인은 버스가 못 움직인 것이
   아니라 서로 자리를 바꿔야 하는 두 차의 교착이었다. 왼쪽 차로 끝의 승용차는 버스 차로로, 버스는 그 차의 차로로 가야 했는데
   전문가가 차로 끝 8m 앞에 서서 그 차가 들어올 자리(8.4m)를 0.1m 차이로 막았다. v5는 10m를 남기고(`lane_end_room`),
   끝나는 차로에서 1m/s 이하로 기어가면 틈이 없어도 요청한다. 환경 게이트도 그 조건에서는 요청을 SUMO에 넘긴다
   (`mdp_config.ACTION["lane_change_safety"]`의 `lane_end_*`). 달리는 중의 요청은 그대로 막는다.
2. **곡선 속도 제한.** 버스 횡가속도 한계 2.0m/s²(`road_config` comfort)로 차로 형상에서 위치별 속도 제한을 만들고
   (`route_map.curve_speeds`), 환경의 허용 속도와 관측 `speed_limit`, `next_speed_limit*`에 넣었다. 앞쪽 제한은 이제
   교차로 안에서도 경로를 따라 100m를 본다. J82·J83·J42 좌회전과 -R96.64의 꺾인 곳이 16~21km/h가 된다. 횡가속도
   최댓값은 16.6 → 2.08m/s², 바퀴 시간은 +6~11초다. `behavior_metrics.py`가 횡가속도(속도 × 요 레이트)를 센다.
   AI-Scientist 평가기의 전문가 점수는 97.22에서 96.55836138096521이 됐다(두 번 돌려 metrics.json 전체가 같다).
3. **나가는 쪽 횡단보도 위치.** 교차로를 돌아 나가는 길의 횡단보도는 경로 위에서 9~12m에 걸치는데 그 끝(다음 도로의 시작)을
   위치로 적고 있었다. 그 3m 앞에 서는 버스는 횡단보도 위에 섰다. 느리게 돌면서 드러났다(dev busy 1601, 보행자 충돌).
   이제 띠에 들어서는 곳을 적는다. 관측 `crosswalk_distance`와 기록의 `distance_to_crossing`이 그만큼 앞당겨진다.
4. **틈 찾는 v5는 옆 차로 앞차보다 느리게 간다(`gap_search_margin` 1.5m/s).** dev busy 1603에서 버스와 옆 차가 서로 상대 차로로
   가야 했다. SUMO는 그 차가 버스를 먼저 보내도록 버스 속도에 맞춰 달리게 해, 제한속도의 0.6배로만 줄이던 버스와 115m를
   나란히 가다 R111b 끝에서 교착했다. 고친 뒤 dev 60회에서 바뀐 에피소드는 1603(완주)과 1611(1초 일찍 옮김, 바퀴 시간 같음)이다.
   앞뒤 수치는 `metrics/gap_pace_{before,after}_*.json`.

## 수업 규칙에 맞춘 환경과 수집 정책 (11/23, 브랜치 `feat/course-compliant-collection`)

이 절 위의 기록은 SUMO 안전 보조를 켠 환경(속도 모드 31, 차선 모드 1536, 46차원 관측)에서 나왔다. 수업 규칙
(`project_materials/7_Project_Consulting.md` 4절)은 자율주행 차량을 돕는 보조 규칙과 어느 차로로 가라는 정보를
금지한다. 강화학습은 뒤로 미루고(사용자 결정), 환경을 규칙에 맞춘 뒤 그 환경에서 잘 달리는 수집 정책을 만들었다.
무엇을 쓰고 쓰지 않는지와 측정 표는 `collection_policy.md`에 있다.

1. **안전 보조 끔.** ego는 수업 원본처럼 `speed_mode` 0, `lane_change_mode` 0이다(`env/road_config.py`). 환경은
   속도를 경로 최고 속도로만 자른다. 차로 끝에서 간격 게이트를 건너뛰던 예외는 없앴다. SUMO가 요청 속도를 바꾼
   스텝은 0이다(`speed_intervened`).
2. **조향.** 차선 모드 0에서 SUMO는 차선변경을 차로 가운데 0.35m 앞에서 끝낸다. 환경이 `changeSublane`으로 가운데까지
   옮긴다(`env/sumo_env.py`의 `_steer`). 차로 경계를 넘는 약 1.6초 동안 간격 게이트를 계속 보고, 깨지면 되돌린다
   (`lane_change_aborted`). 수업 원본의 차선변경은 한 계산에 끝나므로 게이트를 한 번만 본다.
3. **종료.** 충돌(차, 보행자)과 적색 정지선 통과에서 에피소드가 끝난다(`collision`, `pedestrian_collision`,
   `red_light`). 경로 끝(J1 정지선)에 적색으로 도착한 것도 적색 통과다.
4. **관측 43차원(`hapjeong_bus_v4`).** `route_lane_offset`, `route_lane_continuation`, `stop_lane_offset`을 뺐다.
   차로별 연결성은 200m 앞까지 본다(`OBSERVATION["connectivity_range"]`). 목적지까지 가는 차로는 1이다. 횡단보도는
   100m 앞까지 본다. 현재 차로 앞차에는 옆 차로에서 넘어와 버스 앞길에 걸친 차도 들어간다.
5. **수집 정책 `expert-v5`.** 차로별 연결성으로 차로를 고르고, 정류장 앞에서는 가장 오른쪽 차로로 간다. 앞쪽 낮은
   제한속도와 녹색 신호에 미리 속도를 맞춘다. 버스 감속 한계(2m/s²)로는 50km/h에서 황색 3초 안에 서지도 지나가지도
   못하는 구간(정지선 35~57m 앞)이 있어, 신호에 30km/h 안팎으로 다가간다. 바퀴 시간이 그만큼 늘었다.
6. **옛 모델.** `models/`의 BC, DAgger, PPO, Residual은 46차원 관측으로 학습했다. `collect_pomdp_data.py`가 차원이
   다르다고 알리고 멈춘다(`tests/test_model_observation.py`). 지금 환경에서 `--policy expert-v5`로 데이터를 다시 모아
   다시 학습한다. `report.md`와 `metrics/*_final.json`의 옛 수치는 지금 환경과 비교할 수 없다.

이 변경에서 열린 일이다.

- 조교님 확인 여섯 가지(`collection_policy.md`): 규칙 기반 시연 데이터, 연결성 200m(50m로는 dev 60회 중 4회가
  R111b 끝에 갇힌다), 경로 기준 연결성, 관측 거리 100m, 적색 통과와 보행자 충돌 종료, 차선변경 중 게이트.
- 수집 속도는 워커 12개에 103 전이/초다. 10^7 전이에 약 27시간이 걸린다. 한계는 파이썬의 TraCI 호출이고 절반은
  기록용 상태 읽기다. libsumo나 TraCI 구독으로 바꾸면 줄어들 것이다(재지 않았다).
- 비보호 좌회전(J83, J42의 `g`)에서 버스는 마주 오는 차를 보지 못한다. 관측에 교차 방향 차량이 없다. 속도를
  6m/s로 줄여 들어가고, 측정에서는 상대 차가 버스에 양보했다. 교차 방향 차량을 관측에 넣는 것이 다음 순서다.
- 횡단보도는 사람이 올라선 뒤에야 보인다(신호 있는 횡단보도). 버스가 가까이 왔을 때 들어서는 보행자는 못 피한다.
- AI-Scientist 평가기(`agsci_task/`)의 전문가 점수는 이 환경에서 다시 재지 않았다. 과제 글만 43차원과 안전 보조
  끔으로 고쳤다.

## 열린 일

연구 쪽에서 이어갈 일이다.

- 곡선 제한으로 버스가 J82를 20km/h로 돌면 교차로를 비우는 데 6.5초가 걸리고, J82의 황색 3초 + 전적색 3초보다 길다.
  황색에 못 서고 들어간 버스는 보행 신호가 켜질 때 아직 교차로 안에 있다. 횡단보도 위치를 고친 뒤 dev 60회와 속도 모드
  38의 30회에서 충돌은 0이지만 신호 시간과 버스 통과 시간이 맞지 않는 것은 그대로다(신호는 팀이 정한다).
- 속도 모드 38에서는 J83에서 횡가속도가 2.23m/s²까지 나왔다(모드 31은 2.08). 환경은 판단을 시작할 때의 자리로 허용 속도를
  정하고, 버스는 0.5초 동안 2~3m를 더 간다. 그 0.5초에 지날 자리의 가장 낮은 제한을 쓰면 줄어든다.
- 관측 `next_speed_limit`은 100m 안의 가장 낮은 제한 하나다. 높은 제한 뒤에 더 낮은 제한이 이어지는 길에서는 가까운 쪽을
  못 본다. 합정역 노선에는 그런 곳이 없다.
- 교차로 안에서는 관측 `route_lane_offset`이 0, `route_lane_continuation`이 300m다(`_route_observation`의 `if not inside`).
  J107 내부 차로(20m) 바로 뒤의 R111b(27m) 끝을 버스가 못 본다. dev busy 1603(고치기 전)에서 버스는 J107 안에서 4.6에서
  7.2m/s로 가속해 차로 끝 10m 앞에 서지 못하고 6.4m 앞에 섰다. 교차로 안에서도 `getBestLanes`는 다음 도로 차로(R111b_1 26.7m,
  offset 1)를 주므로, 내부 차로가 이어지는 차로를 찾아 그 값에 남은 내부 차로 길이를 더하면 된다. 그렇게 고친 시험 사본은
  그것만으로 1603을 완주했다(1052스텝, 차로 끝 11.5m 앞에서 교대). 틈 찾기를 고친 뒤 dev 60회, train 30회, 모드 38 30회에서 차로를
  옮기지 못한 채 R111b에 들어간 에피소드가 없어 넣지 않았다. 관측의 뜻이 바뀌므로 데이터를 다시 모을 때 같이 넣는다.
- 속도 모드 38 dev busy 1608은 이 변경 전부터 충돌한다(R73_1, 2.4m/s). 오른쪽으로 옮기다 선 승용차(f_R114_to_R128.1)가
  선 차의 옆 이동 0 때문에 R73_2에서 -1.19m, 93도로 멈춰 버스 차로로 튀어나와 있다. 관측은 그 차를 왼쪽 차로 차로 보고,
  모드 31에서는 SUMO 안전 속도가 버스를 세운다.
- `metrics/curve_merge_after_*_dev.json`은 e8a8cae로 다시 돌리면 교통량마다 20회 중 19회가 다르다(모드 38은 30회 중 29회).
  e8a8cae에서는 dev busy 1603이 갇히고 모드 38 1608이 충돌한다. 비교는 `gap_pace_before_*`를 기준으로 한다.

- BC 데이터와 Residual RL은 전문가 v4와 지도 3.13.0 수요(보행자 고정 시각, 골목 고리 경유 흐름)로 만들었다. final 평가는
  지금 환경(지도 3.14.0, 충돌 차 정지, 진행 기반 stuck)에서 했다. 전문가 v5와 3.14.0으로 데이터를 다시 모으고
  BC, DAgger, Residual을 다시 학습하는 것이 다음 순서다(`--policy expert-v5`, `dagger.py`는 metadata로 판을 고른다).
- PPO는 60만 걸음에서 멈췄다. 처음부터 배우는 PPO가 전문가 수준에 닿는지는 더 긴 학습이 필요하다. final에서 74번 갇혔고
  50번이 서교동교회 정류장 앞이다(`report/final_failures.md`). 전문가 시연으로 시작하는 PPO(BC 초기화)도 후보다.
- final `residual_s2203` busy 3번(seed 9603)에서 ego가 R77→R111 경계에서 끊긴 차선변경으로 1.25m 오른쪽에 밀린 채 J82를
  좌회전했고, 나가는 R112 오른쪽이 인도라 SUMO가 출구에서 세웠다(`MSVehicle::brakeForOverlap`). 시험용 버스와 같은
  원인인데 ego는 이미 차선 모드 1536이다. 25초 동안 가운데로 돌아오지 않은 이유를 찾는다. 재현:
  `python collect_pomdp_data.py --map hapjeong --scenario busy --seed 9603 --policy ppo --model research/av_bc_rl/models/residual_s2203/model.pt --episodes 1 --workers 1 --max-steps 1400 --out-dir /tmp/r --name r`.
- BC와 DAgger는 R111b 끝(J82 좌회전 직전 27m)에서 가끔 갇힌다. 좌회전 차로에서 오른쪽으로 옮긴 뒤 다시 움직이지 않는 것처럼
  전문가가 보여 준 적 없는 상태다. DAgger 반복을 늘리면 줄어들 실패다. final busy 9619의 `bc_s0`은 선 차의 옆 이동을
  0으로 하면 차로 끝에 선 채 200초 동안 요청을 내지 않았다(SUMO는 그때 변경이 막히지 않았다고 본다). 11/23 변경 뒤에는
  완주하지만(정차 3/4) 곡선 제한으로 접근이 달라져 미리 좌회전 차로에 들어간 덕이고, 그 정책의 버릇은 그대로다.
- 검사 도구 `scripts/audit_naturalness.py`를 바꿀 때마다 돌린다(정적 검사는 pytest에 들어 있다). 남은 OPEN 줄과 팀
  결정은 `naturalness_audit_round2.md`, `naturalness_audit_round3.md`에 있다.
- `metrics/*_dev.json`은 환경을 여러 번 바꾸는 동안 잰 것이라 판이 섞여 있다(예: `bc_s0_dev`에는 순간이동이 있다).
  비교는 final만 쓴다. `evaluate.py`는 이제 정책, 모델 해시, 에피소드 수, 주행 코드 트리가 다르면 다시 돌리므로 dev를
  다시 돌리면 지금 환경으로 새로 잰다.
- 독립 검토가 찾은 작은 것 넷은 final 평가 도중이라 고치지 않았다. 재학습 전에 고친다.
  - `env/sumo_env.py` 가속 램프가 측정 가속도를 자르지 않고 시작한다. SUMO 비상 제동(-9) 다음 스텝에 전진 명령을 줘도
    감속한다. `np.clip(self._acceleration, -self.max_decel, self.max_accel)`로 시작한다. 전문가에게는 0회, 규칙 정책 busy에서 41회.
  - `env/route_policy.py` v5 `_neighbors`의 이웃 속도가 0 아래로 갈 수 있다(`max(..., 0.0)`).
  - 정류장 차로 차이(`stop_lane_offset`)는 정류장이 그 도로의 가장 오른쪽 버스 차로라고 가정한다. 합정역에서는 맞다.
  - `ExpertPolicyV5.gate`는 import할 때 `mdp_config.ACTION`에서 읽는다. 다른 행동 설정의 환경이면 `make_expert`로 넘긴다.

팀이 정할 일이다(근거 문서를 같이 적는다).

- S6 횡단보도 보행자 우선 전환(`proposal_s6_priority.md`). 바꾸면 `_crossing_occupied`가 걸어 들어오는 사람도 보게 같이 바꾼다.
- 수요 배수(골목 3배, S6 보행자 4배, 상수역 2배). 실측 근거가 없다. 배수별 시나리오 발생 표는 `maps/hapjeong/README.md` 3.14.0 절.
- 독막로6길 고리는 경유 흐름을 없앤 뒤 나오는 차가 없다. 출발지를 둘지 고리를 정리할지.
- 황색 신호에서 배경 차가 9m/s²로 비상 제동한다(에피소드당 약 3번, ego는 0). 원인은 IDM 계열 정지 규칙(T 1.8초,
  b 4.5)이고 신호 시간이 아니다. tau 1.2면 35건이 14건이 되지만 배경 교통 전체가 바뀐다(`naturalness_audit_round1.md` E4).
- J42 R55 좌회전 차로가 하나뿐이고 주기가 182초라 좌회전 대기가 길다. 배경 버스 노선 B1, B4가 차로를 보지 않고 골라져
  있다(`traffic/README.md`).
- PR #8(3.7.0~3.13.0 패치 흡수)은 리뷰 대기다. 이 브랜치의 지도는 3.14.0(도로망은 3.13.0과 같고 수요만 다르다).

## 파일

| 파일 | 내용 |
|---|---|
| `task.md`, `rubrics.json` | 연구 과제와 완료 조건 |
| `decisions.tsv` | 모든 결정과 근거(시간순, 덧붙이기만) |
| `behavior_metrics.py` | 수집 JSONL에서 버스 행동 지표 |
| `evaluate.py` | 정책을 dev/train/final seed에서 주행시키고 지표 JSON |
| `compare.py` | 여러 평가 결과를 표로 |
| `merge_datasets.py`, `dagger.py` | BC 데이터 합치기, DAgger |
| `training_progress.py` | 학습 중 TensorBoard 지표 읽기 |
| `traffic_audit.md` | 억지 교통 코드 감사 |
| `agsci_task/` | AI-Scientist-v3 과제 폴더 |
| `metrics/` | 모든 평가 결과 JSON |
| `report.md` | 결과 보고서 |
