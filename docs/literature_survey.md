# 선행 연구 조사: 버스 입력 설계, 신경망 구조, 모방학습 뒤 강화학습

2026-11-24 · 출처는 모두 조사 에이전트가 직접 열어 본 것이다. "확인 못 함"은 출처에서 확인하지 못했다는 뜻이다.

## 결론

- **가까운 차를 순서대로 나열한 목록을 단순 신경망(MLP)에 넣는 방식이 비교 실험에서 가장 약했다.** 팀의 93개 방식이 이 조합이다.
- **차로별 앞뒤 차로 짠 고정 벡터는 단순 신경망으로도 잘 된다.** 더 복잡한 구조(Deep Sets)보다 5~9% 낮은 데 그쳤다. 그 연구(Huegle 2019)의 입력도 43개였다.
- **단순 신경망(MLP)은 표준이다.** 정책망과 가치망을 나눠 각각 256칸 두 층으로 두는 설정이 흔하다.
- **모방학습 뒤 강화학습은 그냥 이어 붙이면 무너지기 쉽다.** 가치망을 먼저 예열하고, 탐색 폭을 작게 시작하고, 모방 항을 조금 남겨 두어야 한다.
- **모방학습 데이터는 15만~30만 건이 흔하다.** 양보다 장면의 다양성이 중요했다. 1,000만 건은 학습이 아니라 수업 제출 요건 때문에 필요하다.
- **규칙 운전사를 따라 배우게 하는 것은 연구에서 흔한 방법이다**(Learning by Cheating, PlanT).

## 1. 다른 연구는 무엇을 입력으로 주나

| 출처 | 입력 | 크기 | 신호 / 경로 / 보행자 |
|---|---|---|---|
| highway-env | 차마다 있음, 위치, 속도. 거리순 목록, 빈 칸은 0, [-1,1] 정규화 | 5대×5값(교차로는 15대×7값) | 없음 / 없음 / 없음 |
| Flow | 내 속도, 앞차와 속도 차, 앞차 거리 | 3 | 없음 |
| Huegle 2019(SUMO) | 차로별 앞 2대, 뒤 2대 | 43 | 없음 |
| MetaDrive | 라이다 240줄 + 내 상태 + 다음 경유점 | 259 | 경유점 |
| PDM-Open | 차로 중심선 120점 + 과거 2초 내 상태 | — | 중심선 |
| Learning by Cheating, Roach, ChauffeurNet | 위에서 본 지도 그림(차, 보행자, 신호가 각각 한 겹) | 192×192 이상 | 모두 그림의 겹으로 |
| PlanT | 차마다 한 묶음(위치, 속도, 방향, 크기) + 경로 묶음 | 차 수에 따라 변함 | 신호 1값 / 경로 묶음 / 없음 |

공통점은 셋이다. 내 차 기준 상대 좌표, 정한 범위로 정규화, 신호와 경로는 따로 넣기.

## 2. 목록 방식과 차로 구조 방식

| 연구 | 비교 | 결과 |
|---|---|---|
| Leurent & Mercat 2019 | 목록+단순 신경망 / 격자 / 주의 구조 | 목록+단순 신경망이 꼴찌(충돌 잦음), 주의 구조가 1등 |
| Huegle 2019 | 차로별 고정 43개 / 격자 / Deep Sets | Deep Sets가 5~9% 높음. 고정 43개도 근접 |
| MetaDrive | 근처 4대를 이어 붙이기 / 평균 내기 | 이어 붙이기가 모든 지도에서 낮음(교차로 성공률 62.7 대 71.6) |
| PlanT 2022 | 차마다 묶음+트랜스포머 / 그림 | 절반의 데이터로 같은 성능. 속도를 빼면 크게 떨어짐 |

## 3. 신경망 구조

| 출처 | 설정 |
|---|---|
| Stable-Baselines3 PPO 기본 | 64칸 두 층, 탐색 폭(표준편차) 1에서 시작 |
| RL Zoo 연속 제어 PPO, highway-env 예제 | 정책망과 가치망 각각 256칸 두 층 |
| Andrychowicz 2021 | 정책망과 가치망 분리, 두 층, tanh, 처음 탐색 폭 0.5, 입력 정규화 |
| 과거가 필요할 때 | Roach는 순환망 대신 과거 4장면을 입력에 넣음 |

## 4. 모방학습 뒤 강화학습에서 알려진 함정

| 함정 | 내용 | 대응 |
|---|---|---|
| 실수 누적 | 작은 실수가 쌓여 못 본 상황에 들어감(DAgger) | 학습한 정책이 지나간 상황에 정답을 다시 붙여 재학습 |
| 원인 착각 | 입력이 많을수록 엉뚱한 단서를 따라 함. 과거 행동과 이력이 주범(de Haan) | 과거 행동을 입력에서 빼기 |
| 관성 | 서 있으면 계속 서 있음. 데이터가 늘수록 심해짐(Codevilla) | 오래 선 구간을 줄이기 |
| 이어 붙일 때 무너짐 | 배우지 않은 가치망이 잘 배운 정책을 망침(Jump-Start RL) | 정책을 고정하고 가치망만 수만 걸음 예열 |
| 탐색 폭 | 처음 폭이 크면 따라 배운 행동이 흩어짐 | 작게 시작 |
| 잊어버림 | 강화학습 중 모방한 행동을 잃음 | 모방 항이나 원래 정책과의 차이 벌점을 조금씩 줄이며 유지(DAPG, VPT) |

## 5. 데이터 양

| 연구 | 양 | 관찰 |
|---|---|---|
| Learning by Cheating | 15.7만 | — |
| PDM-Open | 17.7만 | — |
| PlanT | 22.8만, 3배로 늘리면 선생 수준 | — |
| MetaDrive | 30만 | 모방학습 성공률 13% |
| Codevilla | 10시간과 100시간 | 많다고 좋지 않음. 시드에 따라 크게 흔들림 |

## 우리 팀에 주는 뜻

1. **입력은 차로별 앞뒤 차 구조를 유지한다.** 93개 목록 방식으로 바꾸지 않는다.
2. **보행자는 "가까운 몇 명" 목록 대신 횡단보도 칸으로 준다**(거리, 사람 있음, 들어오기까지 시간). 조사 에이전트의 추론이고 직접 비교한 연구는 없다.
3. **단순 신경망을 쓴다.** 정책망과 가치망을 나누고 각각 256칸 두 층, 입력 정규화.
4. **모방학습 뒤 강화학습은 세 가지를 지킨다.** 가치망 예열, 작은 탐색 폭, 모방 항 유지. 팀의 BC→PPO가 불안정했다면 여기부터 확인한다.
5. **모방학습 데이터는 20만 건쯤에서 시작한다.** 오래 선 구간은 줄이고 끼어들기, 양보, 정류장 장면을 늘린다.
6. **규칙 운전사가 프로그램이라 DAgger를 싸게 쓸 수 있다.**

## 출처

- highway-env 문서: https://highway-env.farama.org/observations/ , https://highway-env.farama.org/environments/intersection/
- Leurent, Mercat. Social Attention for Autonomous Decision-Making in Dense Traffic. 2019. https://arxiv.org/abs/1911.12250
- Huegle, Kalweit, Mirchevska, Werling, Boedecker. Dynamic Input for Deep Reinforcement Learning in Autonomous Driving. IROS 2019. https://arxiv.org/abs/1907.10994
- Wu, Kreidieh, Parvate, Vinitsky, Bayen. Flow. IEEE T-RO 2021. https://arxiv.org/abs/1710.05465
- Li, Peng, Feng, Zhang, Xue, Zhou. MetaDrive. https://arxiv.org/abs/2109.12674
- Dauner, Hallgarten, Geiger, Chitta. Parting with Misconceptions about Learning-based Vehicle Motion Planning. CoRL 2023. https://arxiv.org/abs/2306.07962
- Chen, Zhou, Koltun, Krähenbühl. Learning by Cheating. CoRL 2019. https://arxiv.org/abs/1912.12294
- Zhang, Liniger, Dai, Yu, Van Gool. End-to-End Urban Driving by Imitating a Reinforcement Learning Coach (Roach). ICCV 2021. https://arxiv.org/abs/2108.08265
- Bansal, Krizhevsky, Ogale. ChauffeurNet. RSS 2019. https://arxiv.org/abs/1812.03079
- Renz, Chitta, Mercea, Koepke, Akata, Geiger. PlanT. CoRL 2022. https://arxiv.org/abs/2210.14222
- Stable-Baselines3 PPO 문서: https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html
- RL Zoo: https://github.com/DLR-RM/rl-baselines3-zoo
- Andrychowicz et al. What Matters for On-Policy Deep Actor-Critic Methods? ICLR 2021. https://arxiv.org/abs/2006.05990
- Ross, Gordon, Bagnell. DAgger. AISTATS 2011. https://arxiv.org/abs/1011.0686
- de Haan, Jayaraman, Levine. Causal Confusion in Imitation Learning. NeurIPS 2019. https://arxiv.org/abs/1905.11979
- Codevilla, Santana, López, Gaidon. Exploring the Limitations of Behavior Cloning for Autonomous Driving. 2019. https://arxiv.org/abs/1904.08980
- Rajeswaran et al. DAPG. RSS 2018. https://arxiv.org/abs/1709.10087
- Uchendu et al. Jump-Start Reinforcement Learning. ICML 2023. https://arxiv.org/abs/2204.02372
- Wołczyk et al. Fine-tuning Reinforcement Learning Models is Secretly a Forgetting Mitigation Problem. ICML 2024. https://arxiv.org/abs/2402.02868
- Baker et al. Video PreTraining (VPT). 2022. https://arxiv.org/abs/2206.11795

---

# 2부. 최신 연구(2023~2026)

2026-11-24 · 조사 에이전트가 원문을 열어 가져온 내용이다. 내가 원문을 다시 읽지는 않았다.

## 결론

1. **보상은 단순할수록 낫다.** CaRL(2025)의 보상은 "이번에 나아간 거리 × 벌점 계수"뿐이고, 충돌과 신호 위반은 에피소드를 끝낸다. 여러 항을 더한 보상은 차가 안 움직이는 쪽으로 빠졌다(점수 2 대 38).
2. **단순 신경망에는 목록이 아니라 뜻이 고정된 칸을 준다.** V-Max(2025)에서 물체 목록을 단순 신경망에 넣으면 0.68, 주의 구조는 0.87이었다. 칸마다 "있음/없음" 표시를 함께 넣는다.
3. **모방학습은 선생이 좌우한다.** 선생이 학생에게 없는 정보로 운전하면 학생은 따라 할 수 없다(LEAD 2025: 선생 정보를 학생에 맞추자 22.5에서 34.1). 드문 행동에 가중치를 주는 것은 오히려 해로웠다(Zimmerlin 2024).
4. **모방학습 뒤 강화학습은 순서가 있다.** 가치망 먼저, 정책은 천천히 풀고, 모방 항은 유지한다. PIRLNav(2023)에서 그냥 이어 붙이면 +1.6%p, 이 순서를 지키면 +9.9%p였다.
5. **SUMO의 배경 차는 버스를 피해 준다.** 강화학습이 이를 악용할 수 있다. Waymax(2023)에서 피해 주는 차를 상대로 배운 정책은 피해 주지 않는 차 앞에서 충돌률이 6.5%에서 25.2%로 올랐다. 평가 교통은 학습 교통보다 거칠어야 한다.

SUMO에서 버스 정차와 신호 없는 횡단보도를 직접 다룬 2024~2026 강화학습 논문은 찾지 못했다.

## 눈여겨볼 방법

| 주제 | 방법 | 출처 |
|---|---|---|
| 적색 신호 | "정지선에 선 길이 0의 가상 차"로 표현해 앞차 따라가기와 같은 틀로 다룬다. 입력 5개로 충돌 0 | Kumar 2025 |
| 가감속 출력 | 숫자 하나를 맞히는 대신 몇 칸 중 고르게 하고, 쓸 때는 확률로 평균을 낸다(40 → 50점) | Jaeger 2023 |
| 오래 선 구간 | 많이 남기면 "계속 서 있기"만 배운다. 다만 변한 장면만 남기는 필터는 효과가 작았다 | Zimmerlin 2024 |
| 내 속도 입력 | 넣으면 엉뚱한 단서가 되어 성능이 떨어진 사례가 있다 | PlanT 2.0 2025 |
| 가치망 예열 | 정책을 고정하고 가치망만 먼저 학습, 가치망 마지막 층은 0 근처로 시작 | PIRLNav 2023 |
| 모방 항 유지 | 강화학습 4번에 모방학습 1번을 섞어 충돌률 0.229 → 0.089 | RAD 2025 |
| 개입형 DAgger | 학생이 운전하다 위험하면 선생이 2초 넘겨받는다. 4회쯤에서 효과가 멈춘다 | TakeAD 2025 |
| 보상 점검 | 학습 전에 "완주 > 느린 완주 > 가만히 있기 > 중간에 충돌" 순서로 점수가 나오는지 확인한다. 검토한 보상 9개 중 7개가 이 순서를 어겼다 | Knox 2021 |

## 수업 규칙과 부딪히는 권고(쓰지 않는다)

- 조사 에이전트가 제안한 입력 가운데 "목표 차로까지 차로 수 차이"는 수업이 금지한 차로 지시다.
- "멈춰야 할 가장 가까운 지점" 같은 통합 값은 환경이 판단을 대신해 주는 것이라 보조로 볼 수 있다. 조교 확인 없이는 넣지 않는다.

## 출처(2부)

- Jaeger, Dauner, Beißwenger, Gerstenecker, Chitta, Geiger. CaRL. CoRL 2025. https://arxiv.org/html/2504.17838v3
- Cusumano-Towner et al. Robust Autonomy Emerges from Self-Play. 2025. https://arxiv.org/html/2502.03349
- Li, Jia, Wang, Yan. Think2Drive. 2024. https://arxiv.org/html/2402.16720
- Gulino et al. Waymax. 2023. https://arxiv.org/html/2310.08710
- Charraut, Doulazmi, Tournaire, Buhet. V-Max. 2025. https://arxiv.org/html/2503.08388v3
- Kazemkhani et al. GPUDrive. 2024. https://arxiv.org/html/2408.01584v3
- Zhang et al. CarPlanner. CVPR 2025. https://arxiv.org/html/2502.19908v3
- Kumar, Mishra, Chakraborty, Peruru. DRL longitudinal control at signalised intersections. 2025. https://arxiv.org/html/2505.08896v1
- Knox, Allievi, Banzhaf, Schmitt, Stone. Reward (Mis)design for Autonomous Driving. 2021. https://ar5iv.labs.arxiv.org/html/2104.13906
- Abouelazm et al. Balancing Progress and Safety. 2025. https://arxiv.org/html/2505.06737v3
- Ramrakhya, Batra, Wijmans, Das. PIRLNav. 2023. https://arxiv.org/pdf/2301.07302
- Lu et al. Imitation Is Not Enough. 2023. https://arxiv.org/pdf/2212.11419v2
- Gao et al. RAD. 2025. https://arxiv.org/html/2502.13144v2
- Zhou et al. WSRL. ICLR 2025. https://arxiv.org/abs/2412.07762
- Nakamoto et al. Cal-QL. NeurIPS 2023. https://arxiv.org/abs/2303.05479
- Ball et al. RLPD. 2023. https://arxiv.org/html/2302.02948v4
- Liu et al. TakeAD. 2025. https://arxiv.org/html/2512.17370v2
- Jaeger, Chitta, Geiger. Hidden Biases of End-to-End Driving Models. ICCV 2023. https://arxiv.org/html/2306.07957v1
- Zimmerlin et al. Hidden Biases of End-to-End Driving Datasets. 2024. https://arxiv.org/html/2412.09602v2
- Gerstenecker, Geiger, Renz. PlanT 2.0. 2025. https://arxiv.org/html/2511.07292v1
- Nguyen et al. LEAD. 2025. https://arxiv.org/html/2512.20563v1
- Trumpp et al. Efficient Learning of Urban Driving Policies Using BEV State Representations. ITSC 2023. https://arxiv.org/abs/2305.19904

---

# 3부. 권민혜 교수 연구실(BMIL)의 주행 연구

2026-11-24 · 조사 에이전트가 논문과 공개 코드를 열어 가져온 내용이다. 내가 원문을 다시 읽지는 않았다.

## 결론

- **연구실의 입력 설계는 논문마다 거의 같고, 수업 기본 코드가 바로 그 설계다.** 내 속도, 차로별 가장 가까운 앞차와 뒤차의 거리와 속도 차, 차로별 앞쪽 밀도, 차로가 있는지와 끊기기까지 거리. 값은 시야 거리(30~60m)로 나눠 [-1, 1]로 맞춘다.
- **이유는 한결같다.** "자율주행 차는 전체 상태를 볼 수 없다." 전체 상태는 따로 저장하되 정책 입력에는 넣지 않는다.
- **안전장치 없이 배운다.** 자율주행 차는 SUMO 안전 검사를 끄고(speed_mode 0), 안전은 보상으로 배운다.
- **데이터는 품질 등급을 나눠 모은다.** 다 배운 정책, 절반쯤 배운 정책, 무작위 정책의 데이터를 따로 약 100만~120만 건씩. 데이터 품질과 행동의 다양성이 결과를 좌우했고, 모방학습이 품질 저하에 가장 약했다.
- **PPO는 이 연구실 실험에서 가장 약했다.** 합류 실험 충돌률이 PPO 12%, DDPG 2%, TD3 0%였다.
- **신호, 버스 정류장, 횡단보도를 다룬 논문은 없었다.** 연구 도로는 고속도로, 원형 도로, 병목, 합류다. 우리가 넣는 신호, 정류장, 횡단보도 입력은 선례 없이 같은 방식을 넓힌 것이다.

## 연구실 설계와 그 이유

| 항목 | 연구실 방식 | 논문이 든 이유 |
|---|---|---|
| 주변 차 | 차로마다 가장 가까운 앞차 1대, 뒤차 1대. 가까운 순 목록이 아님 | 차는 주변만 인지한다 |
| 보는 차로 수 | 내 차로 포함 3개(AD4RL 등). 수업 코드는 5개 | — |
| 빈 칸 | 앞차 없음은 +1, 뒤차 없음은 -1 | 값의 뜻을 고정 |
| 도로 구조 | 차로별 "없음 / 계속 이어짐 / 몇 m 뒤 끊김"과 앞쪽 밀도만 준다 | 같은 입력을 여러 도로에 쓴다(O3RL은 도로 세 종류에 같은 19개 입력) |
| 전체 상태 | 모든 차의 속도, 위치, 차로를 따로 저장 | 데이터 검증, 표현 학습용. 정책 입력에는 쓰지 않음 |
| 행동 | 연속 가속 + 차로 변경(연속 출력을 왼쪽, 유지, 오른쪽으로 나눔) | — |
| 보상 | 속도, 앞차 안전거리, 차로를 바꿀 때만 뒤차 안전거리, 의미 없는 차로 변경 벌점, 급가감속 벌점, 충돌 벌점(가장 큼) | 사고는 "최악의 주행 행동". 뒤차 거리는 "뒤차의 의무"라 바꿀 때만 본다 |
| 신경망 | 단순 신경망. 64-32 또는 256-256 | — |
| 데이터 | (관측, 행동, 보상, 다음 관측, 종료) 단위. 품질 등급별 100만~120만 건 | 오프라인 강화학습 |

## 수업 FAQ에서 다시 확인한 것

- 글자보다 숫자를 쓴다. 신호는 차원을 쪼갠다.
- 겹치거나 시나리오별로 따로 둔 컬럼은 합친다.
- 전체 상태(State)만 보고 도로 구성을 짐작할 수 있으면 충분하다.
- 보상 항은 꼭 필요한 것만 둔다.
- Hugging Face에 올릴 때 파일마다 형식이 다르면 미리보기가 깨진다(수업 예시 데이터셋에서 실제로 그랬다). 형식을 하나로 맞춘다.

## 우리 계획에 주는 뜻

1. **입력은 연구실 방식(차로별 앞뒤 차, 밀도, 차로 끊김)을 유지한다.** 밀도를 빼려던 계획은 거둔다. 연구실이 일관되게 넣는 값이다.
2. **차로 정보는 "노선으로 이어지는가"가 아니라 "차로가 물리적으로 몇 m 뒤에 끊기는가"로 준다.** 연구실 방식이고, 차로 지시가 아니다.
3. **1,000만 건은 "다 배운 모델의 완벽한 주행"만이 아닐 수 있다.** 수업 소개 자료는 "학습 과정에서 생성된 데이터를 저장"이라고 적었고, 연구실은 품질 등급을 나눠 모은다. 조교에게 물을 항목에 넣는다.
4. **PPO만 고집하지 않는다.** 연구실 실험에서 PPO가 가장 약했다. 모방학습으로 시작하는 것이 더 중요해진다.
5. **모방학습 데이터는 충돌 없는 주행만 쓴다.** 연구실 결과에서 모방학습이 데이터 품질에 가장 민감했다.

## 출처(3부)

- 연구실 페이지: https://brainmil.wordpress.com/
- Lee, Eom, Kwon. AD4RL. ICRA 2024. https://arxiv.org/html/2404.02429v1 , 코드 https://github.com/DongsuLeeTech/AD4RL
- Lee, Kwon. Scenario-Free Autonomous Driving with Multi-Task Offline-to-Online RL. IEEE T-ITS 2025. 코드 https://github.com/DongsuLeeTech/O3RL
- Lee, Kwon. Episodic Future Thinking Mechanism for Multi-Agent RL. NeurIPS 2024. https://arxiv.org/html/2410.17373v1
- Lee, Kwon. Combating Stop-and-Go Wave at a Ring Road. KICS 2021. https://journal.kics.or.kr/digital-library/manuscript/file/37529/NODE10617399.pdf
- 국내 학회 논문(KSAE 2024·2025, KICS 2023·2024, JCCI 2024)은 연구실 페이지의 파일 링크로 열었다. 제목은 조사 보고에 있다.
- 수업 저장소: https://github.com/bmil-ssu/Artificial-Intelligence-and-Control-for-Autonomous-Driving
