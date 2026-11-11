"""A3 검증: P1 대기형(버스가 v_low 이하일 때만 건넘), P2 진입형(시간 간격 수용), 차량-보행자 충돌 기록."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.pedestrian import build_a3_net, run_crossing_episode, traci  # noqa: E402

OUT = os.path.join(os.path.dirname(__file__), "..", "results", "traffic_verification")
V_LOW = 1.5
TS = (2, 3, 4, 5, 6, 8, 10)          # 버스가 횡단보도에 닿기까지 남은 시간 [s]
SPEEDS = (1.0, 2.0, 5.0, 10.0)


def main():
    cfg = build_a3_net()
    res = {"v_low": V_LOW, "table": {}}
    for pt in ("ped_p1", "ped_p2"):
        tab = {}
        for v in SPEEDS:
            tab[str(v)] = {str(T): bool(run_crossing_episode(cfg, pt, v, bus_dist=v * T, v_low=V_LOW)
                                         ["crossed_before_bus"]) for T in TS}
        res["table"][pt] = tab
    # 충돌 기록: 보행자는 버스를 무시(ped_force), 버스는 회피 안 함(speedMode 0)
    col = []
    for T in TS:
        r = run_crossing_episode(cfg, "ped_force", 10.0, bus_dist=10.0 * T, bus_speedmode=0)
        col.append({"T": T, "collision": r["collision"], "crossed_before_bus": r["crossed_before_bus"]})
    res["vehicle_pedestrian_collision"] = col
    res["stopped_bus_crossed"] = {pt: bool(run_crossing_episode(cfg, pt, 0.0, bus_dist=8, max_t=40)
                                           ["crossed_before_bus"]) for pt in ("ped_p1", "ped_p2")}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "a3.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(json.dumps(res, ensure_ascii=False, indent=1))
    traci.close()


if __name__ == "__main__":
    main()
