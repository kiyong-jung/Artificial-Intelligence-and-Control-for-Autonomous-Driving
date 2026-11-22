# S6 횡단보도를 보행자 우선으로 바꾸자는 제안

와우산로 극동방송 정류장 바로 뒤의 신호 없는 횡단보도(S6, 교차로 `S6_wausanro`)를 보행자 우선으로 바꾸자는 제안이다.
지금 도로망에서는 보행자가 차의 틈을 기다린다. 도로교통법 제27조 제1항은 반대로 보행자가 횡단보도를 건너고 있거나 건너려고
할 때 운전자가 횡단보도 앞에서 일시정지하게 한다. 임시 지도에서 우선으로 바꿔 재 보니 버스 한 바퀴 시간과 교통 흐름은 거의
그대로였고, 버스가 S6에서 보행자에게 양보하는 장면이 20회 중 13회 생겼다. 대신 SUMO가 버스 속도를 대신 줄이는 일이 S6 앞에서
새로 생긴다. 그래서 우선으로 바꾸되, 전문가 규칙이 걸어 들어오는 보행자를 보게 관측을 같이 고치기를 권한다.

이 문서는 측정 결과와 팀이 동의할 때 바꿀 내용을 적는다. 커밋된 `maps/hapjeong`은 바꾸지 않았다.

## 지금 상태

- 커밋된 `maps/hapjeong/source/network.net.xml`(3.13.0)에서 보도에서 횡단보도로 가는 연결 `:S6_wausanro_w0 → :S6_wausanro_c0`은
  비우선(`m`)이고, 와우산로 차량 연결 4개는 우선(`M`)이다.
- 비우선으로 바꾼 커밋은 `ac6571e`다. `scripts/build_hapjeong.py`의 주석에 따르면 우선이면 주변 교통(`traffic/`)의 보행자 유형
  P1·P2(`jmIgnoreFoe*`, `impatience`)가 아무 효과가 없어서다.
- 학습 환경은 P1·P2를 쓰지 않는다. `env/`와 `collect_pomdp_data.py`는 `traffic/`을 읽지 않고, `source/pedestrians.rou.xml`의
  보행자에는 유형이 없어 SUMO 기본 보행자로 걷는다(코드와 파일에서 확인). P1·P2가 의미를 갖는 곳은 `traffic/` 스크립트뿐이다.
- 전문가 규칙(`env/route_policy.py`)은 관측 `crosswalk_occupied`를 보고 선다. `env/sumo_env.py`의 `_crossing_occupied`는 비우선
  횡단보도에서는 횡단보도 위의 사람만 센다. 보행자 우선 횡단보도에서는 양끝 2.5 m 안에 멈춰 선(0.5 m/s 이하) 사람도 센다.
  비우선에서 기다리는 사람까지 세면 사람은 차를, 버스는 사람을 기다리는 교착이 났다(`decisions.tsv` 2026-11-22 diagnose 행).

## 시험한 것

임시 지도 `maps/hapjeong_trial_s6prio`(gitignore 대상)를 `research/av_bc_rl/s6_priority_trial.py make`로 만들었다. 커밋된 지도
폴더를 복사하고 도로망 10줄과 `map.json`만 바꾼다.

- 바꾸는 방법은 netconvert의 횡단보도 정의 `<crossing node="S6_wausanro" edges="-R112.84 R112.112" priority="true"/>`다.
  `build_hapjeong.py`의 `MIDBLOCK_PRIORITY`가 넘기는 값과 같은 속성이다.
- 커밋된 도로망을 netconvert로 그냥 다시 변환하면 S6 밖도 바뀐다. 3.12.0에서 좌표를 직접 고친 J1 횡단보도 c2가 원래대로
  돌아가고, 횡단보도 다섯 곳(`C3`, `C4`, `J1_c2`, `J8_c0`, `J96_c4`)의 `outlineShape`와 `origBoundary` 자릿수가 달라진다.
  그래서 스크립트는 아무것도 안 바꾼 변환과 우선으로 바꾼 변환을 둘 다 만들고, 둘이 다른 줄만 커밋된 파일에 옮긴다. 바뀐 줄이
  모두 `S6_wausanro` 요소 안에 있는지, 신호 교차로의 연결 번호가 그대로인지, 보도→횡단보도가 `M`인지 검사한다. 두 번 만들어
  sha256이 같았다(`d7875ed6…`).
- 바뀐 10줄은 S6 교차로의 `request` 5줄, 차량 연결 4개(`M` → `m`), 보도→횡단보도 연결 1개(`m` → `M`)다.
- 전문가 규칙과 관측 코드는 그대로다. `RouteMap.pedestrian_priority`를 도로망에서 읽으므로, 우선 지도에서는 S6에서 멈춰 기다리는
  사람도 자동으로 `crosswalk_occupied`에 들어간다.

세 가지를 같은 조건으로 견줬다.

1. 지금(비우선). 커밋된 `hapjeong`이다.
2. 우선. 임시 지도다.
3. 우선 + 걸어 들어오는 사람도 봄. 임시 지도에서 `_crossing_occupied`가 S6 보도 위에서 다음 도로가 횡단보도인 사람도 세게
   이 실행 안에서만 바꿨다(`collect --lookahead`, `env/` 파일은 그대로).

## 측정 결과

### 버스(전문가 규칙)

`collect_pomdp_data.py --policy expert --scenario busy`로 지도마다 20 에피소드를 달렸다. 에피소드 seed는 900~919로 세 경우가
같고, 제한은 1,200 스텝(600초)이다. "S6 접근"은 버스 앞범퍼에서 횡단보도까지 50 m 안이다. 극동방송 정류장 정차 위치가 횡단보도
19.8 m 앞이라 8초 정차가 이 구간에 들어간다. "S6 앞 정지"는 횡단보도 15 m 안에서 정차 중이 아닐 때 0.1 m/s 아래로 선 구간이다.

| 항목 | 지금(비우선) | 우선 | 우선 + 걸어 들어오는 사람도 봄 |
|---|---|---|---|
| 완주 | 20 / 20 | 20 / 20 | 20 / 20 |
| 버스 충돌, 막힘(stuck), 놓친 정류장 | 0, 0, 0 | 0, 0, 0 | 0, 0, 0 |
| 한 바퀴 평균 | 389.2초 | 390.4초 | 390.2초 |
| 같은 seed끼리 한 바퀴 차이, 평균(최대) | | +1.2초(+13.5초) | +1.1초(+13.5초) |
| S6 접근 중 보행자를 만난 통과 | 20 / 20 | 20 / 20 | 20 / 20 |
| 그중 횡단보도 위에 사람 | 16 | 20 | 20 |
| 그중 보도에 멈춰 기다리는 사람 | 20 | 0 | 0 |
| S6 앞 정지 횟수(정지가 있던 에피소드) | 3회(3) | 19회(13) | 19회(13) |
| S6 앞 정지 시간, 평균 / 최대 | 1.0초 / 1.0초 | 9.8초 / 43.0초 | 9.5초 / 43.0초 |
| 1 m/s 아래로 기어간 시간, 에피소드 평균 / 최대 | 0.95초 / 5.5초 | 11.3초 / 44.5초 | 11.3초 / 44.5초 |
| 정류장 출발부터 횡단보도 통과까지, 평균 / 최대 | 8.8초 / 15.0초 | 20.1초 / 52.5초 | 20.2초 / 52.5초 |
| S6 접근 중 가속도 한계 위반 스텝 | 0 | 46 | 34 |
| S6 접근 중 jerk 한계 위반 스텝 | 0 | 34 | 31 |
| S6 접근 중 SUMO가 요청 속도를 바꾼 스텝 | 0 | 40 | 18 |
| 에피소드 중 배경 차 충돌이 난 스텝 | 1 | 0 | 0 |

### 배경 교통(버스 없음)

버스 에피소드는 S6를 한 번씩만 지나므로, 보행자와 배경 차는 따로 쟀다. busy 교통을 300초 흘린 뒤 3,600초 동안 TraCI로
S6를 봤다(`background`, seed 0~4, 지도마다 5회). 보행자 대기는 S6 양끝 보도에서 0.1 m/s 아래로 서 있던 시간을 건넌 사람마다
더한 값이다. 값은 5회 평균이고, 최대는 5회 중 최댓값이다.

| 항목 | 지금(비우선) | 우선 |
|---|---|---|
| 건넌 보행자(시간당) | 243.2명 | 242.0명 |
| 보행자 대기, 평균 / 90번째 백분위 / 최대 | 8.6초 / 23.3초 / 48.4초 | 0.0초 / 0.0초 / 0.5초 |
| 기다린 보행자 비율 | 66% | 0% |
| 차 한 대가 S6 앞에서 선 시간, 남행 `-R112` / 북행 `R112.112` | 0.00초 / 0.01초 | 2.67초 / 2.82초 |
| 한 번에 서 있던 차의 최대 대수, `-R112` / `R112.112` | 1 / 1 | 4 / 3 |
| 차 한 대가 이어서 선 최대 시간 | 3.7초 | 33.5초 |
| 지나간 차, `-R112` / `R112.112` | 216.6대 / 157.8대 | 207.0대 / 153.0대 |
| 도로망 전체 출발 / 도착 / 끝에 남은 투입 대기 | 1,285.2 / 1,211.6 / 26.6 | 1,285.2 / 1,212.6 / 26.6 |
| 충돌, S6 / 그 밖 | 0 / 0 | 0 / 1 |

### 숫자를 읽을 때

- 한 바퀴 시간이 거의 그대로인 까닭은 S6 바로 다음 신호다. 같은 seed끼리 버스가 `R112.221`(S6 뒤)에 들어선 시각은 최대
  46.5초 늦었다(seed 918). 그런데 `R113`(삼거리포차 `J83` 신호 뒤)에 들어선 시각은 12초, 서교동사거리 앞 `R54a`에서는 0초
  차이였다. S6에서 늦어진 시간을 다음 적색 대기가 흡수한다. 신호 주기가 바뀌면 이 흡수도 달라진다.
- 비우선에서도 버스는 20회 모두 S6에서 보행자를 만났다. 차이는 만나는 방식이다. 비우선에서는 20회 모두 보도에서 기다리는
  사람이 있었다. seed 900에서는 그 사람이 버스가 정류장에 서 있는 동안 건넜다. 우선에서는 보도에서 기다린 사람이 0번이다.
  사람이 서지 않고 바로 들어서고, 버스가 횡단보도 앞에서 기다린다.
- 우선에서 43초 정지는 보행자 네 명이 이어서 들어선 경우다(seed 918). 앞사람이 다 건너기 전에 다음 사람이 들어섰다. S6 보행자
  시간당 240명(`map.json`의 `s6_walkers_per_hour`, 3.3.0에서 60명의 4배로 늘린 가정값)이 이 줄을 만든다.
- SUMO 개입 40 스텝은 우선 횡단보도의 SUMO 보행자가 걷는 속도 그대로 들어서서 생긴다. 지금 관측은 멈춰 선 사람만 세므로
  우선 지도에서 "기다리는 사람"은 0번 잡혔다. 버스는 사람이 횡단보도에 올라선 뒤에야 알고, 그 전에 SUMO의 교차로 규칙이
  버스를 세운다. 전문가 행동과 실제 움직임이 달라지는 스텝이라 BC 데이터로는 나쁘다. 걸어 들어오는 사람도 보게 하면 18 스텝으로
  준다. 남은 18 스텝은 대부분 그 순간 관측에 사람이 없었고, 근처 보행자는 보도 차로(`-R112`, `R112.112`)나 횡단보도를 떠나는
  보도 위에 있었다. SUMO가 보도에서 다가오는 사람에게 먼저 감속한 것으로 본다(추정).
- 와우산로를 지나간 차가 우선 지도에서 평균 3~4% 적다(5회 모두 같은 방향). 도로망 전체 출발·도착은 같아서, 차가 줄었다기보다 다른
  길로 갔다. SUMO는 경로를 고를 때 비우선 연결에 1.5초 벌점(`--weights.minor-penalty`)을 준다. seed 0에서 벌점을 0으로 두면
  북행 차이는 없어졌지만(144대 = 144대) 남행 차이는 남았다(220대, 211대). 남행 쪽 원인은 찾지 못했다.
- 우선 지도의 충돌 1건은 S6가 아니라 양화로 `R50_3`(서교동사거리 앞 대기열)의 차 대 차 측면 충돌이다(seed 1, 479.3초).
  `maps/hapjeong/README.md`의 "알려진 현상"에 있는 자리다. 지도 변경이 차들의 시각을 조금 바꿔 드러난 것으로 본다(추정).
- 표본은 지도마다 버스 20회, 배경 5시간이다. 정지 횟수와 보행자 대기의 차이는 이 표본에서 뚜렷하다. 한 바퀴 시간 차이 1초는
  seed 사이 퍼짐(316~500초)보다 훨씬 작아서 "차이 없음"으로 읽는다.

## 좋은 점과 나쁜 점

좋은 점.

- 법규와 맞는다. `maps/hapjeong/README.md`의 S6 설명("차가 보행자에게 양보한다")도 그제야 사실이 된다.
- 버스가 보행자에게 양보하는 장면이 생긴다. 비우선에서는 20회 중 3회, 1초씩 섰다. 우선에서는 13회 섰고 평균 9.8초다. S6가
  학습 데이터에서 실제 양보 상황이 된다.
- 보행자 대기가 평균 8.6초(최대 48초)에서 0초가 된다.
- 비용이 작다. 버스 한 바퀴 +1초, 배경 차는 S6에서 한 대당 약 2.7초 선다. 줄은 최대 4대라 `J83`에서 S6까지 84 m인 두 차로 안에 든다.
  막힌 에피소드와 S6 충돌은 0이다.

나쁜 점.

- 관측을 고치지 않으면 S6 앞에서 SUMO가 버스를 대신 세운다(40 스텝, 가속도 위반 46 스텝). 전문가 행동과 실제 움직임이 어긋난
  데이터가 쌓인다.
- `traffic/`의 P1·P2가 S6에서 효과를 잃는다. 다만 학습 환경은 P1·P2를 원래 쓰지 않는다. `measure_s1_s6.py`의 S6 차량 대기
  (`S6_vehicle_waiting`)도 뜻이 바뀌어 예전 값과 바로 견줄 수 없다. 배경 측정처럼 늘 것으로 본다(추정).
- 우선 횡단보도의 SUMO 보행자는 다가오는 차 앞에서 서지 않는다. seed 918에서는 버스가 횡단보도 1.6 m 앞까지 와 있을 때 걷는
  속도로 들어섰다. 실제 보행자보다 대담하다.
- 버스가 S6에서 최대 43초 선다. 시간당 240명이라는 가정값 때문이다.
- 도로망 버전이 바뀌므로 3.13.0에서 모은 전문가·BC 데이터는 다시 모아야 한다.

## 권고

우선으로 바꾸기를 권한다. 단, `_crossing_occupied`가 걸어 들어오는 보행자를 보도록 같이 고친다. 법규와 맞고, S6가 실제로 버스가
양보하는 장면이 되며, 시간과 흐름에 드는 비용이 작다. 관측을 그대로 두면 S6 양보를 SUMO가 대신 해 BC 데이터가 나빠진다.

관측 수정은 아직 절반만 확인했다. 보도 위에서 다음 도로가 횡단보도인 사람까지 세면 개입이 40에서 18로 준다. 보도 차로에서
횡단보도로 다가오는 사람(몇 m 안, 다음 도로가 S6 보도)까지 세는 판을 더 재고 고르기를 권한다. 이 판은 재지 않았다.

다음 둘은 팀이 고를 일이다. 재지 않았다.

- S6 보행자 시간당 240명을 줄일지. 줄이면 긴 정지가 줄지만 S6를 만나는 에피소드도 준다. 240명은 비우선일 때 만남을 늘리려고
  정한 값이다.
- 한국 운전자의 실제 양보율을 넣을지. 우선 횡단보도에서 배경 차 일부가 양보하지 않게 하는 방법(`jmIgnoreFoeProb`)이 있다.
  이 경우 다양성은 보행자 대신 운전자 쪽에 생긴다.

## 팀이 동의하면 바꿀 것

1. `scripts/build_hapjeong.py`. 다음에 원본에서 다시 빌드해도 우선이 되게 한다.

   ```diff
   -MIDBLOCK_PRIORITY = False      # S6 횡단보도를 비우선으로 둔다(주변 교통 A3)
   +MIDBLOCK_PRIORITY = True       # S6 횡단보도는 보행자 우선이다(도로교통법 제27조 제1항, 운전자 일시정지)
   ```

   ```diff
   -        # 주변 교통 A3는 비우선 횡단보도(priority="false")를 전제한다. 우선이면 보행자 P1·P2의
   -        # jmIgnoreFoe*·impatience가 아무 영향이 없고 보행자가 차 속도와 무관하게 건넌다. 한국 법규상
   -        # 운전자 일시정지 의무와는 다르며 한계에 적는다.
   +        # 보행자 우선이다. 보행자 P1·P2의 jmIgnoreFoe*·impatience는 여기서 효과가 없고, 보행자는 차 속도와
   +        # 무관하게 건넌다. P1·P2는 scripts/verify_hapjeong.py처럼 비우선으로 다시 만든 복사본에서 본다.
    ```

2. 커밋된 도로망. 지금 도로망에는 빌드 스크립트가 모르는 패치(3.7.0~3.13.0)가 있어 다시 빌드하면 안 된다(루트 README의
   "알려진 한계"). 시험한 것과 같은 10줄을 옮긴다.

   ```bash
   python research/av_bc_rl/s6_priority_trial.py make
   cp maps/hapjeong_trial_s6prio/source/network.net.xml maps/hapjeong/source/network.net.xml
   cp maps/hapjeong_trial_s6prio/map.json maps/hapjeong/map.json
   ```

   그다음 `maps/hapjeong/map.json`의 `"version"`을 `"3.13.0+s6prio"`에서 `"3.14.0"`으로 고친다. sha256
   `d7875ed69ac3749ade2c81dc1a01daa06324f0f903e5779d40fe48ec562f0dec`와 `patches.s6_priority_crossing`은 `make`가 이미 적었다.
   도로망에서 바뀌는 줄은 다음과 같다(줄 번호는 3.13.0 기준).

   ```diff
    2386: <junction id="S6_wausanro" type="priority" ...>
   -2387:     <request index="0" response="00000" foes="10000" cont="0" />
   -2388:     <request index="1" response="00000" foes="10000" cont="0" />
   -2389:     <request index="2" response="00000" foes="10000" cont="0" />
   -2390:     <request index="3" response="00000" foes="10000" cont="0" />
   -2391:     <request index="4" response="01111" foes="01111" cont="0" />
   +2387:     <request index="0" response="10000" foes="10000" cont="0" />
   +2388:     <request index="1" response="10000" foes="10000" cont="0" />
   +2389:     <request index="2" response="10000" foes="10000" cont="0" />
   +2390:     <request index="3" response="10000" foes="10000" cont="0" />
   +2391:     <request index="4" response="00000" foes="01111" cont="0" />
   -2547: <connection from="-R112" to="-R112.84" fromLane="1" toLane="1" via=":S6_wausanro_0_0" dir="s" state="M" />
   -2548: <connection from="-R112" to="-R112.84" fromLane="2" toLane="2" via=":S6_wausanro_0_1" dir="s" state="M" />
   +2547: <connection from="-R112" to="-R112.84" fromLane="1" toLane="1" via=":S6_wausanro_0_0" dir="s" state="m" />
   +2548: <connection from="-R112" to="-R112.84" fromLane="2" toLane="2" via=":S6_wausanro_0_1" dir="s" state="m" />
   -2678: <connection from="R112.112" to="R112.221" fromLane="1" toLane="1" via=":S6_wausanro_2_0" dir="s" state="M" />
   -2679: <connection from="R112.112" to="R112.221" fromLane="2" toLane="2" via=":S6_wausanro_2_1" dir="s" state="M" />
   +2678: <connection from="R112.112" to="R112.221" fromLane="1" toLane="1" via=":S6_wausanro_2_0" dir="s" state="m" />
   +2679: <connection from="R112.112" to="R112.221" fromLane="2" toLane="2" via=":S6_wausanro_2_1" dir="s" state="m" />
   -3441: <connection from=":S6_wausanro_w0" to=":S6_wausanro_c0" fromLane="0" toLane="0" dir="s" state="m" />
   +3441: <connection from=":S6_wausanro_w0" to=":S6_wausanro_c0" fromLane="0" toLane="0" dir="s" state="M" />
   ```

   다른 패치처럼 `scripts/` 아래 패치 스크립트로 남기려면 `s6_priority_trial.py`의 `make()`를 대상 폴더만 `maps/hapjeong`으로
   바꿔 옮기면 된다.

3. `env/sumo_env.py`의 `_crossing_occupied`. 시험한 판(`s6_priority_trial.py`의 `patch_lookahead`)은 기존 판정 뒤에 다음을 더한다.
   보행자 우선 횡단보도에서 그 교차로의 보도(`:<교차로>_w*`) 위에 있고 `traci.person.getNextEdge`가 그 횡단보도인 사람이 있으면
   참이다. 위의 권고대로 보도 차로까지 넓힌 판을 먼저 잰다.

4. 문서와 검증 스크립트.
   - `traffic/README.md` 170행 "S6 횡단보도는 … 비우선으로 바꿨다"와 21·48행의 S6 설명.
   - `maps/hapjeong/README.md`에 3.14.0 절, 루트 README "알려진 한계"의 패치 목록에 3.14.0.
   - `scripts/verify_hapjeong.py`의 `nonpriority_s6_net()`. 횡단보도 도로를 `-R112.88 R112`로 적었는데, 지금 도로망에는
     `-R112.88`이 없다(검색으로 확인). netconvert가 모르는 도로라며 실패할 것이다(코드로 본 추정, 실행하지 않았다).
     `-R112.84 R112.112`로 고친다.

5. 3.14.0 도로망에서 전문가·BC 데이터를 다시 모으고 `pytest tests`를 돌린다.

## 다시 재는 방법

저장소 맨 위에서 실행한다. 지도마다 버스 20회는 4 작업자로 약 8~9분, 배경 5시간은 약 6분 걸렸다.

```bash
python research/av_bc_rl/s6_priority_trial.py make                    # 임시 지도 maps/hapjeong_trial_s6prio
python research/av_bc_rl/s6_priority_trial.py collect --episodes 20 --workers 4 --seed 900
python research/av_bc_rl/s6_priority_trial.py collect --lookahead --episodes 20 --workers 4 --seed 900
python research/av_bc_rl/s6_priority_trial.py background --seeds 0 1 2 3 4 --seconds 3600 --warmup 300 --workers 4
python research/av_bc_rl/s6_priority_trial.py report                  # 위 표의 값, results/s6_priority/summary.json
python research/av_bc_rl/s6_priority_trial.py clean                   # 임시 지도 지우기
```

결과 파일(`results/s6_priority/`)은 gitignore 대상이라 커밋하지 않았다. 측정은 SUMO 1.27.1, 2026-11-23, `research/av-bc-rl`의
`5be672a` 위에서 했다.
