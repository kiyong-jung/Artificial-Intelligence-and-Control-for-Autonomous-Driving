# 주행 영상

도로망 3.22.0(좌회전은 가장 왼쪽 차로에서만)에서 찍은 영상이다. `model_`로 시작하는 영상은 규칙 없이 학습한 신경망만 운전했다. 제출 데이터를 모으는 최종 모델(입력 31개, DAgger 16회, 교통량 1.5배까지 학습, `results/n_dagger_s1/bc_it16/model.pt`)이다. `rule_driver_`로 시작하는 영상은 학습 정답을 만드는 규칙 운전사 v7이고 비교용이다.

화면 오른쪽 글은 시각, 속도, 차로, 신호, 정류장, 그리고 버스가 서 있을 때 그 이유(정류장 정차, 앞차 뒤 대기, 신호 대기, 이유 없음)를 보인다.

## 최종 신경망

| 파일 | 장면 | 교통량, seed | 길이 |
|---|---|---|---|
| `model_best_busy_lap.mp4` | 정류장 4곳을 모두 서고 완주한 한 바퀴. 같은 평가에서 차로 변경이 가장 많은 바퀴(10번)다. | 1.5배, 7901 | 66초(750초를 배속) |
| `model_worst_collision.mp4` | 차와 충돌하기 전 마지막 120초 | 1.25배, 7616 | 34초 |
| `model_worst_red_light.mp4` | 적색 신호를 지나기 전 마지막 120초 | 1.5배, 7909 | 34초 |
| `model_worst_stuck_seogyo.mp4` | 서교동사거리 앞(R94)에서 좌회전 차로로 못 끼어들고 200초 멈춰 선 마지막 300초. 제출 수집 `heavy_s0_000` 12번 바퀴 | 1.5배, 40000012 | 43초 |
| `model_worst_pedestrian.mp4` | 신호 없는 횡단보도에서 보행자와 닿기 전 마지막 60초. 제출 수집 `heavy_s0_004` 177번 바퀴 | 1.5배, 40000977 | 24초 |

seed는 모두 학습에 쓰지 않은 평가용(7000번대)이다. 위 세 영상의 평가(네 교통량 80바퀴)에서 76바퀴 완주, 충돌 2, 적색 통과 2였다. 멈춰 섬과 보행자 충돌은 그 평가에 없어 제출 수집 12,000바퀴에서 골랐다.

## 규칙 운전사(비교용)

| 파일 | 장면 |
|---|---|
| `rule_driver_best_busy_lap.mp4` | 혼잡 교통량에서 완주한 한 바퀴 |
| `rule_driver_worst_stuck.mp4` | 서교동사거리 앞(R94)에서 좌회전 차로로 끼어들지 못하고 멈춰 선 마지막 120초 |
| `rule_driver_worst_time_limit.mp4` | 시간 초과로 끝난 바퀴 |

## 장면의 원인 실험(exp/snail-traffic)

규칙 운전사 v7이 운전한다. 교통량 정의는 `env/road_config.py`의 `CAUSE_TRAFFIC`, 결정 기록은 `research/snail/decisions.tsv`다.

| 파일 | 장면 | 기록 |
|---|---|---|
| `taxi25_v7_taxi_in_stop.mp4` | 극동방송 정류장 입구에 25초 서는 택시(주황) 뒤에서 기다렸다가, 택시가 떠난 뒤 정류장에 8초 선다(멈춘 때부터 정차를 마칠 때까지 35초) | `v7_s60000_taxi25` 8번 바퀴, 118~172초 |
| `cut10a_v7_alley_pull_out.mp4` | 성지1길 골목 차(보라)가 G12에서 나와, 성산중학교 정류장을 떠나는 버스 13m 앞으로 들어온다. 버스는 -1.4m/s²로 줄인다 | `v7_s60000_cut10a` 9번 바퀴, 70~100초 |
| `rich2_it16_vs_rd5_taxi.mp4` | 같은 교통(같은 seed)에서 두 신경망을 나란히 본다. 성산중학교 정류장 입구에 택시(주황)가 서 있다. 왼쪽 it16은 20km/h로 다가가 택시를 들이받고 끝난다. 오른쪽 rd5는 택시 뒤에 서서 기다렸다가, 택시가 떠난 뒤 정류장에 서고 출발한다. 3배속이다 | `it16_s61000_rich2`, `rd5_s61000_rich2` 21번 바퀴, 26~88초 |

## 다시 만들기

평가 폴더 하나에서 베스트와 실패 유형별 워스트를 골라 만든다.

```bash
python research/av_bc_rl/render_best_worst.py data/research/verify_map322_heavy --out reports/videos --prefix model_ --label "신경망"
```

한 바퀴만 만들 때는 `research/av_bc_rl/make_lap_video.py <jsonl.gz> --episode N --out out.mp4`를 쓴다.

## PR 미리보기

`preview_best_lap.gif`는 `model_best_busy_lap.mp4`를 3배 더 빠르게 줄인 22초 미리보기다. `preview_worst_*.gif`는 워스트 영상 네 개의 마지막 부분이다(GitHub는 저장소 안의 mp4를 본문에서 바로 틀지 못한다).
