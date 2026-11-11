"""A2 검증: S5 끼어들기 진입률(p 보정 곡선), vTypeDistribution 구간별 진입률,
priority 교차로 충돌 기록, S4 정차 차량이 버스를 막는지.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.scenarios_a2 import (build_a2_net, run_cutin_episode, build_s4_net,  # noqa: E402
                                  run_s4, traci)
from traffic.cutin import default_bins  # noqa: E402

N = int(os.environ.get("A2_N", 60))
THRESH = 28.0      # 버스가 교차로까지 28 m(약 2.5 s) 이내일 때 진입하면 우선권 위반으로 본다
OUT = os.path.join(os.path.dirname(__file__), "..", "results", "traffic_verification")


def rate(rows, key="entered"):
    return float(np.mean([r[key] for r in rows]))


def main():
    res = {"threshold_m": THRESH, "n_per_point": N}
    # 1) p 보정 곡선 (timegap 0.5, impatience 0.5 고정)
    curve = {}
    for p in (0.0, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0):
        cfg = build_a2_net(single_bin=(p, 0.5, 0.5))
        rows = [run_cutin_episode(cfg, "cutin_0", s, threshold_m=THRESH) for s in range(N)]
        curve[str(p)] = rate(rows)
    res["entry_rate_vs_jmIgnoreFoeProb"] = curve
    # 1b) v_ign 기본값 0이면 달리는 버스를 무시하지 못한다는 공식 문서 항목 확인
    cfg = build_a2_net(single_bin=(1.0, 0.5, 0.5), v_ign=0.0)
    res["entry_rate_p1_vign0"] = rate([run_cutin_episode(cfg, "cutin_0", s, threshold_m=THRESH)
                                       for s in range(N)])
    # 2) 분포(6개 구간)
    cfg = build_a2_net(bins=default_bins(6))
    rows = [run_cutin_episode(cfg, "cutin", s, threshold_m=THRESH) for s in range(2 * N)]
    res["distribution_entry_rate"] = rate(rows)
    # 3) priority 교차로 충돌 기록 (버스 speedMode 0 = 버스가 회피 안 함, p=1)
    cfg = build_a2_net(single_bin=(1.0, 0.1, 1.0))
    rows = [run_cutin_episode(cfg, "cutin_0", s, bus_speedmode=0, threshold_m=THRESH)
            for s in range(N)]
    res["junction_collision_rate_bus_speedMode0"] = rate(rows, "collision")
    # 4) S4
    s4 = run_s4(build_s4_net(stop_duration=40))
    parked = [r for r in s4 if r[5]]
    bus_stopped_behind = [r for r in parked if r[4] < 0.1 and r[1] is not None]
    gap = min((r[1] - 4.5 - r[3]) for r in bus_stopped_behind) if bus_stopped_behind else None
    res["S4"] = {"taxi_parked_s": len(parked) * 0.1,
                 "bus_stopped_behind_taxi_s": len(bus_stopped_behind) * 0.1,
                 "min_gap_to_taxi_m": gap,
                 "bus_passed_taxi_while_parked": any(r[3] > r[1] for r in parked),
                 "bus_resumed_after_taxi_left": s4[-1][4] > 5.0}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "a2.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(json.dumps(res, ensure_ascii=False, indent=1))
    traci.close()


if __name__ == "__main__":
    main()
