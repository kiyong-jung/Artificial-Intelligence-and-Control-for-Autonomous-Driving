# 합정 전체 노선: 네 가지 학습 방식

학교 `algorithms/ppo.py`와 `HybridActorCritic`, 팀 `env/road_builder.py` 및 `SumoHighwayEnv`를 사용한다. 별도 시연 버스의 강제 정차·차선 변경 코드를 정책에 넣지 않는다. 기존 CLI는 기본 `legacy` 동작을 유지하며 아래 실행만 버스 학습 프로필을 선택한다.

## 설치와 입력

저장소 루트에서 `pip install -r requirements.txt`. SUMO 1.27.1 실행 파일을 별도로 설치하고 `SUMO_HOME`을 지정한다. 실험 측정 환경은 Python 3.12.10이며 CPU 실행이다. 개인 PC의 절대 경로는 필요하지 않다.

BC 데이터는 별도 전달하는 `data/hapjeong_bc/`에 둔다. Git에 대용량 데이터·체크포인트·학습 로그는 포함하지 않는다.

- `training_view.npz`: observation `(N,93)`, action_raw `(N,2)`, episode `(N,)`, action_valid `(N,)`.
- `split.json`: `split.train`, `split.validation`, `split.test`에 중복 없는 episode ID 목록.
- `run_config.json`: 데이터 생성 당시 도로·관측·행동·보상·모델 설정. 임의의 93차원 데이터를 쓰면 안 된다.
- 보존된 실험 데이터는 규칙 교사 100회 중 충돌한 2개 에피소드를 제외한 98개/80,182전이, 분할 68/15/15이다. 원래 사전 분할에서 실패 에피소드만 제외했으며 성공 위주 선택 편향이 있다. 우측 차선 변경이 학습됐다고 주장하지 않는다. 규칙 시범 학습의 과제 허용 여부는 별도 확인이 필요하다.

## 실행

출력 폴더가 이미 있으면 덮어쓰지 않고 실패한다. BC→PPO와 Residual의 `--base`에는 BC 출력 `model.pt`를 지정한다. 옆의 `run_config.json`도 필요하다.

```sh
# 1. BC 단독: PPO와 같은 hybrid actor를 지도 학습
python -m training.hapjeong.train bc --data data/hapjeong_bc/training_view.npz --split-file data/hapjeong_bc/split.json --config data/hapjeong_bc/run_config.json --out results/hapjeong_bc

# 2. BC→PPO: 전체 정책 갱신. 32768 + 65536 두 단계
python -m training.hapjeong.train bc-ppo --base results/hapjeong_bc/model.pt --out results/hapjeong_bc_ppo

# 3. 순수 PPO: 무작위 초기화, 모델 로드 없음
python -m training.hapjeong.train ppo --out results/hapjeong_ppo

# 4. Residual PPO: BC 고정, 별도 PPO가 보정 행동 학습
python -m training.hapjeong.train residual --base results/hapjeong_bc/model.pt --out results/hapjeong_residual
```

PPO 명령에 `--steps 64`를 붙이면 짧은 작동 검사다. BC→PPO에는 **단계마다** 64스텝이 적용된다. 이 검사는 성능 검증이 아니다. 일반 예산은 rollout 크기의 배수 단위로 처리된다.

## 설정과 선정 근거

정확한 값은 `presets.json`에 저장한다. 모두 실험한 범위에서 선정한 값이며 전역 최적값이 아니다.

| 방식 | 설정 | 선정 근거 |
|---|---|---|
| BC | 128×128 tanh, Adam 3e-4, 100epoch, batch256, 제동 가중치4, 가속 MSE+차선 CE | train에서 클래스 가중치 산출, episode validation loss로 epoch 선택 |
| BC→PPO 1단계 | lr1e-4, clip0.2, epoch10, 32768step, seed74001 | 기존 BC→PPO 기준 모델 |
| BC→PPO 2단계 | lr3e-5, clip0.1, epoch5, 추가65536step, seed75001 | 개발50회에서 30완주로 선정, 별도 최종100회 49완주 |
| 순수 PPO | 2단계 PPO 설정, 무작위 초기화, 65536step | **동일 과제에서 선정된 우승 설정 없음. 참고 설정이며 49% 성능을 주장하지 않음** |
| Residual PPO | lr1e-4, clip0.2, epoch10, 32768step, seed74001, 보정 크기0.5 | 같은100회에서 범위2.75보다 완주29 대13 |

공통 PPO: rollout1024, minibatch64, gamma0.999, GAE0.95, value coefficient0.005, entropy/lane entropy0, gradient clip0.5. 단계 전환은 **가중치만 로드하고 Adam을 새로 생성**한다. optimizer/RNG까지 보존한 정확한 재개가 아니다. BC critic은 직접 지도 학습되지 않지만 actor와 공유하는 trunk는 갱신된다.

Residual은 93D 관측에 고정 BC 행동2개를 붙인 95D이다. 가속은 `clip(base + 0.5*tanh(residual), -1, 1)`. 차선은 순환하는 범주 보정이며 단순한 좌우 물리 이동량 합산이 아니다. rollout buffer에는 합성 행동이 아닌 PPO가 샘플링한 보정 행동과 그 log probability를 저장한다. 평가에는 base_model.pt와 base_run_config.json도 함께 있어야 한다.

## 공통 과제와 평가

필수 정류장 성산중학교·극동방송·서교동교회·합정역 각각 연속8초, 이후 bus_loop 도착이 완주다. 정류장 구간·차로에 앞범퍼가 있고 속도0.1m/s 이하일 때 시간을 센다. 2400결정/1200초 제한. 93D의 차로 가용성 인덱스1~5는 0으로 고정한다. 차량8대/보행자6명은 반경 내 이상적 감지로, 가림 모델은 없다. 정류장 상대 위치는 기존 Cartesian 표현을 유지한다.

주행 보상은 첫 정류장 기준거리로 고정한 약0.0282972/m, 정류장5, 도착10, 충돌·적색진입·정류장미수행·미완료 종료 -10, step cost0.001, 정차 potential shaping2(gamma0.999)다. 전체 경로 길이가 늘었다고 metre당 보상을 줄이지 않는다. 적색 접근 거리별 보상은 이 모델에 없다. 이전 step 보상을 충돌 시 소급해서 지우지 않는다.

SpeedMode38로 물리 가감속·속도 제한은 유지하며 신호 자동제동·우선권 보조는 사용하지 않는다. LaneChangeMode0, 별도 차선 변경 안전 게이트 비활성. 배경 정차 차량은 버스 위치와 무관하게 사전 시간표로 생성한다. 지도, 배경 차량 모델, SUMO 충돌 판정 및 teleport 설정은 변경하지 않는다.

평가할 seed를 직접 지정한 JSON(`{"validation_seeds": [80001, 80002]}`)을 만든다. 이 숫자는 예시이며 기존 사용 여부를 확인한 뒤 개발/시험 용도를 사전 고정한다.

```sh
python -m training.hapjeong.validate results/hapjeong_bc_ppo/refine/model.pt results/check_bc_ppo protocol.json --compact
python -m training.hapjeong.validate_residual results/hapjeong_residual/model.pt results/check_residual protocol.json --compact
```

`--compact`를 생략하면 진단 trace도 저장한다. trace의 observation은 행동 전, state는 행동 후다. `summary.json`의 성공·충돌·신호위반·정류장미수행·시간초과를 비교한다. 보상 계수가 다른 실험의 return만으로 순위를 정하지 않는다. 10회 개발 평가로 선별하고 애매하거나 유망하면 추가20회만 진행한다. 최종시험 seed를 다시 튜닝에 사용하지 않는다.

## 기존 측정 결과의 범위

| 동일 seed 76101~76200, 각100회 | 완주 | 충돌 | 신호위반 | 미수행 | 기타 |
|---|---:|---:|---:|---:|---:|
| BC |17|42|34|7|0|
| BC→PPO 1단계 |33|41|20|6|0|
| Residual 0.5 |29|23|32|14|2|
| Residual 2.75 |13|36|28|23|0|

위 비교는 방법 선정에 사용했다. 별도 최종시험 **78101~78200**에서는 BC→PPO 1단계 **31/100**, 선정된 2단계 모델 **49/100**(충돌24, 신호위반24, 미수행3)이다. BC 단독17과 개선49는 같은 seed 비교가 아니다. 위 시험 seed는 이미 사용했으므로 새 후보 튜닝에 사용하지 않는다. 충돌률5% 미만 목표는 달성하지 못했다.

## PR #8과의 관계

기준은 main `2ca0f86`. 이 변경은 `maps/hapjeong/*`, `scripts/build_hapjeong.py`, 루트 README를 수정하지 않는다. #8은 지도 생성과 배경 흐름을 바꾸므로 텍스트 충돌이 없어도 합친 뒤 동일 모델 작동 확인이 필요하다. 기존 49/100을 #8의 재생성 도로에서 측정한 성능으로 표시하지 않는다. 학습 프로필은 기본 옵션과 분리돼 있다.
