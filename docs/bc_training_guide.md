# PDM 버스 데이터 전처리, MLP 행동 복제(BC) 학습, closed-loop 평가 명령 정리

관측 33차원과 3값 차로 라벨로 MLP 정책을 행동 복제 학습하고 평가하는 순서다.
- 관측 33차원: 31차원에 정지 지점 거리와 신호를 더했다. 설계는 [specs/2026-11-26-obs33-mlp-bc-design.md](superpowers/specs/2026-11-26-obs33-mlp-bc-design.md)에 있다.
- 3값 차로 라벨: 실제 옆 이동 구간만 ±1이다.

데이터는 `data/pdm_bus_sumo_lc`의 `busy_s*`만 쓴다. 다른 데이터는 쓰지 않는다.
- 전처리와 closed-loop 평가는 로컬(Windows, Git Bash)에서 한다. 평가에는 SUMO가 필요하다.
- 학습은 GPU 서버(Linux)에서 한다.

```
[로컬] 수집 ─ 전처리 ─ 점검 ─ 묶기(tar) ──scp──▶ [서버] 풀기 ─ 학습 실행 ─ 결과 표 / TensorBoard
[로컬] closed-loop 평가(SUMO-LC 월드) ◀──scp── [서버] 학습된 모델(model.pt, run_config.json, audit.json)
```

## 1. 로컬: 전처리

저장소 루트에서 Git Bash로 실행한다. 일괄 스크립트의 기본 폴더는 예전 데이터(`data/pdm_bus`, `baseline`)다.
그래서 **`OUT_DIR`과 `SCENARIO`를 꼭 준다**.

```bash
OUT_DIR=data/pdm_bus_sumo_lc SCENARIO=busy bash scripts/preprocess_pdm_bus_batch.sh count   # 상태만 센다
OUT_DIR=data/pdm_bus_sumo_lc SCENARIO=busy ACCEL_POS=1.5 ACCEL_NEG=3.0 bash scripts/preprocess_pdm_bus_batch.sh 6   # 6개씩 동시
```

- 건너뛰는 것:
  - 이미 33차원까지 전처리한 seed. `processed/metadata.json`에 `obs33_names`가 있으면 끝난 것으로 본다.
  - 수집 중인 `.partial` 폴더.
- 중간에 멈추면 같은 명령을 다시 치면 된다.
- count 출력에 "v2 전처리 N개"라고 나오지만, 실제로는 33차원 기준으로 센다.
- seed 하나에 약 7분 걸린다. 관측 계산이 행당 약 34 ms다.
- 로그는 `data/pdm_bus_sumo_lc/_preprocess_logs/<seed>.log`, 실패한 seed는 `_preprocess_logs/failed.txt`에 남는다.
- 가감속 라벨 정규화 상수는 가속 1.5, 감속 3.0이다. 모든 seed가 같은 값이어야 한다.

seed 하나만 전처리할 때:

```bash
D:/conda_envs/sumo/python.exe scripts/preprocess_pdm_bus.py data/pdm_bus_sumo_lc/busy_s0 --accel-pos 1.5 --accel-neg 3.0
```

출력은 `<seed>/processed/`에 생긴다. 학습에는 `transitions.npz`의 다음 열만 쓴다.

| 열 | 내용 |
|---|---|
| `observation_33`, `next_observation_33` | 관측 33차원 (N, 33). 모든 값이 [-1, 1] |
| `action_33` | [정규화 가감속, 3값 차로 {+1 왼쪽, 0 유지, -1 오른쪽}] |
| `episode`, `t` | 에피소드 단위 train/validation/test 분할에 쓴다 |

`observation_v2`, `action_raw` 같은 예전 열도 함께 들어 있지만 MLP 학습은 읽지 않는다.

### 전처리 점검(선택)

```bash
D:/conda_envs/sumo/python.exe scripts/check_obs33.py data/pdm_bus_sumo_lc/busy_s0 data/pdm_bus_sumo_lc/busy_s1
```

run마다 `processed/obs33_check.json`을 남긴다. 다음 셋을 확인한다.
- `범위 안 True`: 33개 값이 모두 [-1, 1] 안에 있다.
- `라벨 없는 변경 0`: 차로가 바뀌었는데 라벨이 0인 행이 없다.
- `정차 완료 N, 그때 적색 -> 다음 행 녹색 N`: 두 수가 같다.

## 2. 로컬: 새 데이터만 묶기

```bash
bash scripts/pack_bc_data.sh D:/transfer/bc_obs33.tar
scp D:/transfer/bc_obs33.tar <서버>:~/
```

기본으로 `data/pdm_bus_sumo_lc`의 `busy_s*` 중 33차원까지 전처리한 seed만 넣는다. 묶음에 들어가는 것(저장소 루트 기준 상대 경로):

- `data/pdm_bus_sumo_lc/<seed>/processed/transitions.npz`, `metadata.json`
- `data/pdm_bus_sumo_lc/bc_data_files.txt`: 학습에 쓸 npz 목록이다. seed 번호 순으로 고정한다(분할과 resume이 파일 순서에 의존).

seed 38개가 약 154 MB였다.

학습 스크립트는 MLP만 쓰더라도 시작할 때 `utils/ego_attention.py`를 import한다. 그래서 서버에 `ref/rl-agents/rl_agents`가 있어야 한다.
- 예전 묶음(`bc_data.tar`)을 서버에서 이미 풀었다면 그대로 있으니 데이터만 옮기면 된다.
- 새 서버라면 한 번만 같이 넣는다: `WITH_RL_AGENTS=1 bash scripts/pack_bc_data.sh D:/transfer/bc_obs33.tar`

**seed를 더 수집·전처리한 뒤 다시 묶으면 `bc_data_files.txt`가 바뀐다.**
- 이어 하기(resume)는 데이터의 sha256과 순서가 같아야 한다. 그래서 바뀐 목록으로는 새 `--out-root`에서 학습한다.
- 비교할 실험끼리는 같은 목록을 써야 분할이 같다.

## 3. 서버: 처음 준비

```bash
git clone -b feat/pdm-closed-bus https://github.com/kiyong-jung/Artificial-Intelligence-and-Control-for-Autonomous-Driving.git
cd Artificial-Intelligence-and-Control-for-Autonomous-Driving
tar -xf ~/bc_obs33.tar          # data/pdm_bus_sumo_lc/... 가 생긴다(예전 data/pdm_bus는 그대로)
pip install torch numpy gymnasium tensorboard     # torch는 서버 CUDA 버전에 맞게
mkdir -p results
```

이미 저장소가 있으면 4절대로 브랜치만 업데이트하고 `tar -xf ~/bc_obs33.tar`만 한다.

## 4. 서버: 브랜치 업데이트 (병합 없이)

```bash
git fetch origin
git switch feat/pdm-closed-bus
git pull --ff-only origin feat/pdm-closed-bus
```

- 서버에서는 추적되는 파일을 고치지 않는다. 설정을 바꿀 때는 로컬에서 고쳐 push하고 서버에서 pull한다.
- 서버에서 고친 것이 있으면 `git stash` 후 pull한다.
- 서버 변경을 버리고 원격과 맞추려면 `git reset --hard origin/feat/pdm-closed-bus`를 쓴다. 서버 수정 내용이 지워진다.
- `data/`, `results/`는 git이 무시하므로 pull해도 그대로다.
- 실험이 도는 중에는 pull하지 않는다. 남은 실행과 재시도가 새 코드로 돌기 때문이다.
  필요하면 실험용 폴더를 따로 둔다: `git worktree add ../bc_run feat/pdm-closed-bus`

## 5. 서버: MLP 실험 실행

설정 파일은 [experiments/bc_obs33_mlp.json](../experiments/bc_obs33_mlp.json)이다.
- 공통 설정: `--input obs33 --policy mlp --hidden-layers 256,256`, 가감속 구간 21개 CE, 200 epoch, cosine, patience 40, 차로 가중 effective.
- 실험 6개를 seed 3개씩, 모두 18회 돌린다. 분할은 `split_seed` 38001로 고정한다.

| 실험 | base와 다른 점 |
|---|---|
| `base` | 없음. MLP 256×256(파라미터 80,665개), 차로 손실은 effective 가중 CE |
| `lane_inverse` | 차로 가중 inverse(상한 50) |
| `lane_focal` | 차로 손실을 focal loss로 바꾼다(`--lane-loss focal --focal-gamma 2`). 같은 effective 가중을 alpha로 쓴다 |
| `small128` | MLP 128×128(파라미터 23,961개) |
| `accel_mse` | 가감속을 MSE 회귀로(`--accel-bins 0`), 제동 표본 가중 4. 차로는 effective 가중 CE |
| `accel_mse_focal` | 가감속 MSE + 차로 focal loss(gamma 2) |

```bash
# 무엇을 돌릴지만 확인
python scripts/run_bc_experiments.py --config experiments/bc_obs33_mlp.json \
  --data-list data/pdm_bus_sumo_lc/bc_data_files.txt --out-root results/bc_obs33_mlp --gpus 3 --jobs-per-gpu 6 --dry-run

# 실행 (GPU 3 한 장에 6개 동시, 접속이 끊겨도 돈다)
nohup python scripts/run_bc_experiments.py --config experiments/bc_obs33_mlp.json \
  --data-list data/pdm_bus_sumo_lc/bc_data_files.txt --out-root results/bc_obs33_mlp \
  --gpus 3 --jobs-per-gpu 6 > results/bc_obs33_mlp_runner.log 2>&1 &
```

- `--gpus 3`: 학습 프로세스마다 `CUDA_VISIBLE_DEVICES=3`을 준다. 여러 장이면 `--gpus 0,1`.
- `--jobs-per-gpu`: 한 장에 동시에 돌릴 수다. MLP는 파라미터가 약 8만 개라 GPU 메모리를 거의 쓰지 않는다. 한계는 CPU와 데이터 적재다.
- 일부만 돌리려면 `--only base --seeds 1`. 실패한 실행은 자동으로 2번까지(`--retries`) 이어서 다시 시도한다.
- 같은 `--out-root`에 다른 설정을 쓰면 멈춘다. 실험을 더하려면 설정의 `experiments`에 항목을 추가하고 `--allow-config-change`를 준다.
  예: `"wide512": ["--hidden-layers", "512,512"]`.

결과 위치:

```
results/bc_obs33_mlp/
├── experiment_config.json        처음 실행 때의 설정 사본
├── run_status.json               실행별 성공 여부·시간·장치
├── logs/<실험>_seed<n>.log       실행별 로그
├── summary.md, summary.csv       summarize 결과
└── <실험>/seed<n>/
    ├── model.pt                  선택된(validation bc_loss 최소) 모델
    ├── checkpoint.pt             매 epoch 체크포인트(resume용)
    ├── history.jsonl             epoch별 지표
    ├── tb/                       TensorBoard 기록
    ├── run_args.json, run_config.json   (run_config: input "obs33", network.policy_type "mlp")
    └── audit.json                최종 지표(이 파일이 있으면 끝난 실행)
```

## 6. 서버: 진행 확인

```bash
python scripts/summarize_bc_experiments.py results/bc_obs33_mlp     # 실험별 seed 평균±표준편차 표(진행 중인 실행 포함)
tail -f results/bc_obs33_mlp_runner.log                           # 실행기 로그
tail -f results/bc_obs33_mlp/logs/base_seed1.log                  # 실행 하나
nvidia-smi
```

결과 표는 validation 지표로 정렬한다. test 열은 마지막 확인용이다(test로 실험을 고르지 않는다).
3값 차로에서는 ±1이 곧 실제 이동이라 열의 뜻이 아래와 같다.

| 열 | 뜻 |
|---|---|
| `lane_bal` | 차로 3개 클래스(왼쪽, 유지, 오른쪽) recall 평균 |
| `move_recall`, `move_prec` | 실제 차로 이동(±1)의 recall, precision |
| `keep_to_move` | 유지해야 할 때 옮기라고 예측한 수(위험) |
| `blink_to_move` | 3값에는 깜빡이 클래스가 없어 항상 `-` |
| `accel_mae` | 가감속 오차(정규화 값, 0.06이면 가속 약 0.09 m/s², 감속 약 0.19 m/s²) |

반대 방향 오예측 수(`wrong_side`)는 표에 없다. `audit.json`의 `final.validation.wrong_side`와 TensorBoard에서 본다.

손실 값은 둘로 나눠 남긴다.
- `lane_loss`: 그 실험이 실제로 학습한 차로 손실이다(focal이면 focal 값).
  `bc_loss` = `accel_loss` + `lane_loss`이고, 모델 선택(validation 최소)도 이 값으로 한다.
- `lane_ce`: 모든 실험에서 같은 기준(가중 CE)으로 잰 값이다.
- focal loss와 가감속 MSE는 값의 크기가 CE와 달라서 `bc_loss`, `accel_loss`로는 실험끼리 비교하지 않는다.
  실험끼리는 표의 지표(`accel_mae`, `lane_bal` 등)나 `lane_ce`로 비교한다.
  MSE 실험에는 구간이 없어 `accel_bin_acc`가 `-`다.

### TensorBoard

```bash
tensorboard --logdir results/bc_obs33_mlp --port 6006     # 서버에서
ssh -L 6006:localhost:6006 <서버>                         # 로컬 PC에서, 그다음 브라우저로 http://localhost:6006
```

그래프 이름은 `지표/분할` 형식이다(예: `lane_balanced_acc/validation`). 그래서 train과 validation이 한 그래프에 겹쳐 보인다.
그 밖에 다음 그래프도 있다.
- `lane_recall_<클래스>`, `keep_to_move`, `wrong_side`, `lr`, `time_s`
- 차로 손실 `lane_loss/*`(학습한 손실)와 `lane_ce/*`(공통 기준)
- 선택한 모델의 `final_validation/*`, `final_test/*`

## 7. 중단과 이어 하기(resume)

**같은 명령을 다시 치면 된다.** 끝난 실행(`audit.json` 있음)은 건너뛰고, 멈춘 실행은 마지막 `checkpoint.pt`에서 이어 간다.

- 체크포인트는 매 epoch 원자적으로 저장한다. 담는 것: 모델, target, optimizer, scheduler, 난수 상태, best 모델, early stop 카운트.
  멈췄다 이어 간 결과는 한 번에 끝낸 것과 같다(테스트로 확인).
- 이어 갈 때 인자와 데이터(sha256과 순서)가 같은지 확인한다. 경로는 바뀌어도 된다.
- 바꿔도 되는 인자는 `--epochs`, `--patience`, `--device`, 경로다. 단 `--lr-schedule cosine`은 `--epochs`가 같아야 한다.
- TensorBoard 기록도 체크포인트 뒤에 쓰인 부분을 지우고 이어 쓴다.
- 실행기를 멈추려면 `kill <실행기 PID>`(SIGTERM)를 보낸다. 돌던 학습을 정리하고 끝난다.

## 8. 학습 한 번만 직접 돌릴 때

```bash
python scripts/train_ego_attention_bc.py --data-list data/pdm_bus_sumo_lc/bc_data_files.txt \
  --input obs33 --policy mlp --hidden-layers 256,256 \
  --out results/bc_obs33_single --device cuda --resume \
  --epochs 200 --batch-size 1024 --lr 1e-3 --lr-schedule cosine --patience 40 --split-seed 38001
```

스크립트 이름에 ego_attention이 들어 있지만 `--policy mlp`면 EgoAttention을 쓰지 않는다.
`--input obs33`과 `--policy mlp`는 반드시 함께 준다. 한쪽만 주면 오류로 멈춘다.

주요 인자(기본값):

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `--input` | observation | `obs33`(관측 33차원, `observation_33`/`action_33` 열) |
| `--policy` | ego_attention | `mlp`. 차로 3클래스 전용 |
| `--hidden-layers` | 256,256 | MLP 은닉층 크기(ReLU) |
| `--accel-bins` | 21 | 가감속을 [-1, 1]의 구간 K개로 나눈 CE. 0이면 MSE 회귀 |
| `--lane-weighting` | effective | 차로 클래스 가중: none, inverse, sqrt, effective |
| `--cb-beta` | 0.9999 | effective 가중의 beta |
| `--lane-weight-cap` | 100 | 가장 많은 클래스 대비 최대 배수 |
| `--lane-loss` | ce | `ce`(클래스 가중 CE) 또는 `focal`(클래스 가중을 alpha로 쓰는 focal loss) |
| `--focal-gamma` | 2.0 | focal loss의 gamma. 0이면 CE와 같다 |
| `--lane-weight` | 1.0 | 전체 손실에서 차로 손실의 배수 |
| `--accel-weighting` | none | 가감속 구간 클래스 가중 |
| `--brake-weight` | 4 | 제동 표본(정규화 가감속 < -0.05) 가중 |
| `--lane-keep-bias` | 0.5 | 차로 출력층의 "유지" 클래스 초기 bias |
| `--lr-schedule` | none | none, cosine, plateau |
| `--weight-decay` | 0 | 0보다 크면 AdamW |
| `--patience` | 0 | validation bc_loss가 이 epoch 동안 안 좋아지면 멈춤(0이면 끝까지) |
| `--seed`, `--split-seed` | 38001, 없음 | 초기화·섞기 난수, 분할 난수(없으면 `--seed`) |
| `--device` | cpu | cpu, cuda, cuda:1, auto |
| `--resume` | 꺼짐 | `--out`의 체크포인트에서 이어 감 |
| `--tensorboard` | 켜짐 | `<out>/tb`에 기록. `--no-tensorboard`로 끈다 |

## 9. 로컬: closed-loop 평가

학습된 정책을 SUMO-LC 월드에서 한 바퀴씩 직접 몰게 하고, PDM 전문가와 비교한다.
- 월드: busy 교통, 좌회전 차로 보정 도로망. 양보형·비양보형 시나리오(S2·S3)는 끈다.
- 평가 seed: 기본 1000~1019. 학습 데이터 seed와 겹치지 않는다.
- 정책 버스는 SUMO 안전장치를 끈다. 0.5 s마다 가감속과 차로 명령(+1, 0, -1)을 직접 낸다.
- 차로 변경은 ±1 명령 한 번이면 끝까지 옮긴다. 옮기는 중에 같은 방향 명령이 또 와도 한 차로만 옮긴다.

**`--world sumo_lc`는 꼭 준다.** 33차원 정책을 다른 월드로 평가하려 하면 오류로 멈춘다.

### 서버에서 모델 가져오기

평가에는 실행마다 `model.pt`, `run_config.json`, `audit.json`만 있으면 된다.

```bash
# 서버
cd Artificial-Intelligence-and-Control-for-Autonomous-Driving
tar -cf ~/bc_obs33_models.tar results/bc_obs33_mlp/*/seed*/{model.pt,run_config.json,audit.json}
# 로컬(저장소 루트, Git Bash)
scp <서버>:~/bc_obs33_models.tar D:/transfer/
tar --force-local -xf D:/transfer/bc_obs33_models.tar      # results/bc_obs33_mlp/... 가 생긴다
```

### 평가 명령

```bash
# 한 에피소드만(확인용)
D:/conda_envs/sumo/python.exe scripts/eval_bc_closed_loop.py one --world sumo_lc \
  --agent results/bc_obs33_mlp/base/seed1 --seed 1000 --out results/cl_obs33_smoke
D:/conda_envs/sumo/python.exe scripts/eval_bc_closed_loop.py one --world sumo_lc \
  --agent expert --seed 1000 --out results/cl_obs33_smoke

# 무엇을 돌릴지만 확인
D:/conda_envs/sumo/python.exe scripts/eval_bc_closed_loop.py run --world sumo_lc \
  --models results/bc_obs33_mlp --out results/closed_loop_obs33_mlp --jobs 8 --dry-run

# 전부(모델 18개 + 전문가) x 평가 seed 20개, 동시에 8개
D:/conda_envs/sumo/python.exe scripts/eval_bc_closed_loop.py run --world sumo_lc \
  --models results/bc_obs33_mlp --out results/closed_loop_obs33_mlp --jobs 8

# 표만 다시 / 기록에서 지표 다시 계산
D:/conda_envs/sumo/python.exe scripts/eval_bc_closed_loop.py summarize results/closed_loop_obs33_mlp
D:/conda_envs/sumo/python.exe scripts/eval_bc_closed_loop.py reanalyze results/closed_loop_obs33_mlp
```

- `--seeds 1000-1004`: 평가 seed를 줄인다.
- `--no-expert`: 전문가를 빼고 돌린다.
- `--allow-unfinished`: `audit.json`이 없는(학습 중인) 모델도 평가한다.
- `--stall-seconds`(기본 180): 이 시간 동안 5 m보다 덜 움직이면 멈춰 섬(stuck)으로 끝낸다.
- 끝난 에피소드(`result.json` 있음)는 다시 돌리지 않는다.
  같은 `--out`에 다른 조건으로 평가하면 예전 결과가 그대로 쓰이므로, 조건을 바꾸면 `--out`도 바꾼다.

출력 `results/closed_loop_obs33_mlp/`:

```
<agent>/seed<S>/run/          원시 기록(학습 데이터와 같은 형식)
<agent>/seed<S>/result.json   지표
episodes.csv  stops.csv  agents.csv  failures.csv  highlights.json  summary.md
```

`summary.md`에 들어가는 것:
- 지표 표: 학습 seed 평균 ± 표준편차.
- 결과 분포: 도착, 충돌, 신호 위반, 정류장 놓침, 멈춰 섬. 95% 신뢰구간을 붙인다.
- 실패 위치.
- 영상 후보.

## 10. 로컬 테스트

torch가 있는 파이썬은 `D:/conda_envs/sumo/python.exe`다.

```bash
D:/conda_envs/sumo/python.exe -m pytest -q tests/test_mlp_policy.py tests/test_pdm_obs33.py tests/test_labels3.py \
  tests/test_lane_paths.py tests/test_closed_loop_eval.py tests/test_pdm_preprocess.py
```

## 참고: 지금까지의 결정

- 입력은 관측 33차원, 모델은 MLP만 쓴다. EgoAttention과 관측 v2(166차원)는 쓰지 않는다.
- 차로 라벨은 A안이다. 실제 옆 이동이 시작된 행부터 차로 번호가 바뀐 행까지만 ±1이다. 의도나 방향지시등만 있는 행은 0이다.
- 차로 가중 effective와 inverse를 비교하는 이유는 예전 v2 실험 결과다(seed 29개, 40 epoch).
  - effective: 시험 클래스 평균 recall이 가장 높았고, 유지 → 변경 오예측이 0건이었다.
  - inverse: 이동 precision이 가장 높았다.
  - 3값 라벨에서도 같은지 다시 본다.
- focal loss(`lane_focal`)는 차로 이동 행이 전체의 약 6%로 드물다는 점 때문에 넣었다.
  클래스 가중만으로는 쉬운 "유지" 행이 손실을 채우는데, focal loss는 이미 맞히는 표본의 손실을 줄여 헷갈리는 이동 직전·직후 행에 집중시킨다.
- `small128`은 입력이 33개뿐이라 256×256이 과한지 보려고 넣었다. closed-loop 성적이 비슷하면 작은 모델이 낫다.
- `accel_mse`, `accel_mse_focal`은 가감속을 구간 CE 대신 MSE로 학습하면 어떤지 보려고 넣었다.
  - 두 손실 항의 크기가 달라진다. busy_s0~s1, 3 epoch로 재 보니 validation 값이 다음과 같았다.
    - 가감속 MSE: 약 0.14, 차로 CE: 약 0.80
    - 가감속 구간 CE: 약 2.39, 차로 CE: 약 0.97
  - 그래서 MSE 실험에서는 차로 쪽이 기울기를 더 많이 차지한다. 가감속 MAE가 나쁘게 나오면 `--lane-weight`를 낮춘 실험을 더한다.
- 가감속은 구간 21개 CE, 가중 없음으로 한다. v2 실험에서 구간 11개나 가감속 가중보다 MAE가 낮았다.
- 미룬 일:
  - 뒤차 탐색이 두 조각 교차로(R111b→R112, R112.221→R113, R42→R68)의 두 번째 조각을 건너뛴다. 고치면 전처리를 다시 해야 한다.
  - 온라인 PPO 환경의 관측 33차원.
