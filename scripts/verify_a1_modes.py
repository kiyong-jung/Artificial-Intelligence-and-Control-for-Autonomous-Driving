"""버스 laneChangeMode 값별 A1 누출(p=0) 재검사와 양보율 유지 확인."""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.scenarios import build_a1_net, run_a1_episode, traci  # noqa: E402

N = int(os.environ.get("A1_N", 60))
cfg = build_a1_net()
res = {}
for mode in (1621, 1109, 512, 0):
    leak = {t: float(np.mean([run_a1_episode(cfg, t, 20_000 + s, p_override=0.0, bus_lc_mode=mode)["changed"]
                              for s in range(N)])) for t in ("yielder", "nonyielder")}
    rows = [run_a1_episode(cfg, "yielder", s, bus_lc_mode=mode) for s in range(N)]
    res[str(mode)] = {"leak_p0": leak,
                      "yield_realized": float(np.mean([r["changed"] for r in rows])),
                      "yield_set_p": float(np.mean([r["p_yield"] for r in rows])),
                      "agree": float(np.mean([bool(r["decision"]) == r["changed"] for r in rows]))}
out = os.path.join(os.path.dirname(__file__), "..", "results", "traffic_verification", "a1_modes.json")
json.dump(res, open(out, "w", encoding="utf-8"), indent=1)
print(json.dumps(res, indent=1))
traci.close()
