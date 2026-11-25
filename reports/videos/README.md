# 주행 영상

도로망 3.22.0(좌회전은 가장 왼쪽 차로에서만)에서 찍은 영상이다. `model_`로 시작하는 영상은 규칙 없이 학습한 신경망만 운전했다. 제출 데이터를 모으는 최종 모델(입력 31개, DAgger 16회, 교통량 1.5배까지 학습, `results/n_dagger_s1/bc_it16/model.pt`)이다. `rule_driver_`로 시작하는 영상은 학습 정답을 만드는 규칙 운전사 v7이고 비교용이다.

화면 오른쪽 글은 시각, 속도, 차로, 신호, 정류장, 그리고 버스가 서 있을 때 그 이유(정류장 정차, 앞차 뒤 대기, 신호 대기, 이유 없음)를 보인다.

## 최종 신경망

| 파일 | 장면 | 교통량, seed | 길이 |
|---|---|---|---|
| `model_best_busy_lap.mp4` | 정류장 4곳을 모두 서고 완주한 한 바퀴. 같은 평가에서 차로 변경이 가장 많은 바퀴(10번)다. | 1.5배, 7901 | 66초(750초를 배속) |
| `model_worst_collision.mp4` | 차와 충돌하기 전 마지막 120초 | 1.25배, 7616 | 34초 |
| `model_worst_red_light.mp4` | 적색 신호를 지나기 전 마지막 120초 | 1.5배, 7909 | 34초 |

seed는 모두 학습에 쓰지 않은 평가용(7000번대)이다. 이 평가(네 교통량 80바퀴)에서 76바퀴 완주, 충돌 2, 적색 통과 2였다. 멈춰 섬과 시간 초과가 없어 그 워스트 영상은 없다.

## 규칙 운전사(비교용)

| 파일 | 장면 |
|---|---|
| `rule_driver_best_busy_lap.mp4` | 혼잡 교통량에서 완주한 한 바퀴 |
| `rule_driver_worst_stuck.mp4` | 서교동사거리 앞(R94)에서 좌회전 차로로 끼어들지 못하고 멈춰 선 마지막 120초 |
| `rule_driver_worst_time_limit.mp4` | 시간 초과로 끝난 바퀴 |

## 다시 만들기

평가 폴더 하나에서 베스트와 실패 유형별 워스트를 골라 만든다.

```bash
python research/av_bc_rl/render_best_worst.py data/research/verify_map322_heavy --out reports/videos --prefix model_ --label "신경망"
```

한 바퀴만 만들 때는 `research/av_bc_rl/make_lap_video.py <jsonl.gz> --episode N --out out.mp4`를 쓴다.

## PR 미리보기

`preview_best_lap.gif`는 `model_best_busy_lap.mp4`를 3배 더 빠르게 줄인 22초 미리보기다(GitHub는 저장소 안의 mp4를 본문에서 바로 틀지 못한다).
