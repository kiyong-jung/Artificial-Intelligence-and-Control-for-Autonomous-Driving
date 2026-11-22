# 인수인계: 합정역 버스 AV 연구 (BC, PPO, Residual RL)

이 문서는 다른 컴퓨터의 에이전트가 `git clone` 한 번으로 이 연구를 이어받기 위한 것이다. 브랜치는
`research/av-bc-rl`이고, 모든 결과와 결정은 이 폴더(`research/av_bc_rl/`)에 있다.

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

## 열린 일

연구 쪽에서 이어갈 일이다.

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
  전문가가 보여 준 적 없는 상태다. DAgger 반복을 늘리면 줄어들 실패다.
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
