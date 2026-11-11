"""A1 검증: 양보율, 유발 감속 차이, 누출, openGap 시계열.

결과는 results/traffic_verification/a1.json 에 쓰고 요약을 출력한다.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.scenarios import build_a1_net, run_a1_episode, traci  # noqa: E402

N = int(os.environ.get("A1_N", 200))
OUT = os.path.join(os.path.dirname(__file__), "..", "results", "traffic_verification")


def perm_test(a, b, n=20000, seed=0):
    """평균 차이의 양측 순열 검정 p값 (scipy 없이)."""
    rng = np.random.default_rng(seed)
    a, b = np.asarray(a), np.asarray(b)
    obs = abs(a.mean() - b.mean())
    pool = np.concatenate([a, b])
    cnt = 0
    for _ in range(n):
        rng.shuffle(pool)
        cnt += abs(pool[:len(a)].mean() - pool[len(a):].mean()) >= obs
    return (cnt + 1) / (n + 1)


def main():
    cfg = build_a1_net()
    res = {}
    # 1) 양보율 + 유발 감속
    eps = {t: [run_a1_episode(cfg, t, s) for s in range(N)] for t in ("yielder", "nonyielder")}
    for t, rows in eps.items():
        p_mean = float(np.mean([r["p_yield"] for r in rows]))
        dec = float(np.mean([bool(r["decision"]) for r in rows]))
        real = float(np.mean([r["changed"] for r in rows]))
        agree = float(np.mean([bool(r["decision"]) == r["changed"] for r in rows]))
        res[t] = {"n": len(rows), "mean_set_p": p_mean, "decision_rate": dec,
                  "realized_rate": real, "outcome_agrees_with_decision": agree,
                  "mean_max_decel": float(np.mean([r["max_decel"] for r in rows])),
                  "pass_rate_within_0.05": abs(real - p_mean) <= 0.05}
    d_y = [r["max_decel"] for r in eps["yielder"]]
    d_n = [r["max_decel"] for r in eps["nonyielder"]]
    res["decel_diff_perm_p"] = perm_test(d_y, d_n)
    # 2) 누출: 양보 확률 0 (결정은 항상 비양보), SUMO가 스스로 간격을 내주는 비율
    leak = {}
    for t in ("yielder", "nonyielder"):
        rows = [run_a1_episode(cfg, t, 10_000 + s, p_override=0.0) for s in range(N)]
        leak[t] = float(np.mean([r["changed"] for r in rows]))
    res["leak_rate_p0"] = leak
    # 3) openGap 시계열: 차선변경 요청을 안 하고 간격 변화만 본다
    ser = run_a1_episode(cfg, "yielder", 1, record=True, t_end=30.0,
                         bus_kwargs={"t_request": 1e9})
    tau = ser["series"]
    tvals = [s for s in tau]
    t_open = ser["t_open"]
    gap_open = next(s["gap"] for s in tvals if s["t"] >= t_open)
    target = ser["new_tau"] * 9.0                  # 목표 시간차간 x 속도 (getLanePosition 기준 범퍼 간격)
    reach = next((s["t"] for s in tvals if s["t"] >= t_open and s["gap"] >= target - 0.5), None)
    after = [s for s in tvals if s["t"] >= t_open + 12.0][:1]
    res["openGap"] = {"t_open": t_open, "gap_at_open": gap_open,
                      "gap_at_open_plus_duration": after[0]["gap"] if after else None,
                      "target_gap": target, "reach_time_after_open": None if reach is None else reach - t_open,
                      "duration": 12.0,
                      "gap_max": max(s["gap"] for s in tvals),
                      "f_speed_min": min(s["f_speed"] for s in tvals),
                      "series_every_1s": [(s["t"], round(s["gap"], 2), round(s["f_speed"], 2))
                                          for s in tvals if abs(s["t"] - round(s["t"])) < 1e-6]}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "a1.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "openGap"}, ensure_ascii=False, indent=1))
    print(json.dumps(res["openGap"], ensure_ascii=False))
    traci.close()


if __name__ == "__main__":
    main()
