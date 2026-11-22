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
3. **전문가 규칙(`env/route_policy.ExpertPolicy`).** 관측만 쓰는 IDM + 가상 정지점. busy dev 30회에서
   급제동 0, SUMO 개입 1.3%, 불일치 0.21%, 정류장 118/120. 관측만 쓰는지 48,176 샘플로 검증(차이 0).
4. **억지 교통 코드.** `traffic/`의 버스 상대 조작 코드를 감사했다(`traffic_audit.md`). 학습 환경은 이 코드를
   쓰지 않는다. 정리는 별도 브랜치에서 진행했다(아래 "열린 일").
5. **학습.** BC(전문가 시연), PPO(처음부터), Residual RL(전문가 가감속 + 학습한 보정)을 같은 관측과
   보상으로 학습하고, `evaluate.py`로 같은 seed에서 닫힌 루프 평가했다. 결과는 `report.md`.

## 다른 컴퓨터에서 시작하기

```bash
git clone https://github.com/kiyong-jung/Artificial-Intelligence-and-Control-for-Autonomous-Driving.git
cd Artificial-Intelligence-and-Control-for-Autonomous-Driving
git switch research/av-bc-rl
conda create -n sumo-rl python=3.10 && conda activate sumo-rl
python -m pip install -r requirements.txt tensorboard
# SUMO 1.27.1을 설치하고 SUMO_HOME을 설정한다(README의 설치 절). 또는 pip install eclipse-sumo==1.27.1
python -m pytest -q                       # 이 브랜치에서 통과해야 한다
python research/av_bc_rl/agsci_task/evaluate.py --results_dir /tmp/eval_expert   # 전문가 96.9점, 3분
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
# PPO, Residual (busy, 학습 seed 셋)
$PY train.py --map hapjeong --scenario busy --seed 2101 --timesteps 600000 --eval-interval 0 --max-steps 1400 \
    --results-dir results/research --run-label ppo_s2101
$PY train.py --map hapjeong --scenario busy --seed 2201 --timesteps 300000 --eval-interval 0 --max-steps 1400 \
    --residual-scale 0.3 --results-dir results/research --run-label residual_s2201
# 평가(dev로 고르고, final은 마지막에 한 번)
$PY research/av_bc_rl/evaluate.py --policy expert --split final --label expert
$PY research/av_bc_rl/evaluate.py --policy bc --model <model.pt> --split final --label bc_s0
$PY research/av_bc_rl/evaluate.py --policy ppo --model <model.pt> --split final --label ppo_s2101
$PY research/av_bc_rl/compare.py --split final --groups expert=expert bc=bc_s0,... --out research/av_bc_rl/report/final_table.md
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
   평가기는 결정적이어야 한다(같은 seed, 같은 점수). 이 Mac에서 전문가를 두 번 돌려 96.88373764527032로
   같았고 에피소드별 결과도 모두 같았다. 다른 컴퓨터(리눅스, 다른 SUMO 빌드)에서는 값이 조금 다를 수 있으니
   거기서 한 번 더 두 번 돌려 본다.
4. CPU 과제다(`container.gpu_count=0`, `investigation.execution.gpu_evaluation=false`). 평가 한 번에 약 3분이다.

## 열린 일

- `traffic/` 억지 코드 정리 브랜치의 결과 검토와 병합(진행 상황은 decisions.tsv).
- PR #8(3.7.0~3.13.0 패치 흡수)은 아직 리뷰 대기다. 이 연구는 main의 도로망(3.13.0)을 그대로 쓴다.
- S6 횡단보도는 망에서 차량 우선(보행자 비우선)이다. 한국 법규(보행자 우선)와 맞출지는 팀이 정한다.
- 전문가가 busy에서 120번 중 2번 정류장을 놓친다. 정류장 차로에 빠른 차가 계속 지나가 틈이 없을 때다.

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
