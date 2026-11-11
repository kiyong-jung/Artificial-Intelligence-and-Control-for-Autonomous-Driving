"""같은 시드로 A1 에피소드를 돌려 결과를 JSON으로 저장한다. 백엔드는 TRAFFIC_BACKEND로 고른다.
사용: compare_backends.py out.json  (traci와 libsumo로 각각 실행한 뒤 두 파일을 비교)"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.scenarios import build_a1_net, run_a1_episode  # noqa: E402

cfg = build_a1_net()
out = [run_a1_episode(cfg, t, s, record=(s < 3)) for t in ("yielder", "nonyielder") for s in range(30)]
from traffic.scenarios_a2 import build_a2_net, run_cutin_episode, build_s4_net, run_s4  # noqa: E402
from traffic.pedestrian import build_a3_net, run_crossing_episode  # noqa: E402
c2 = build_a2_net(single_bin=(0.3, 0.5, 0.5))
out += [run_cutin_episode(c2, "cutin_0", s) for s in range(15)]
out.append(run_s4(build_s4_net()))
c3 = build_a3_net()
out += [run_crossing_episode(c3, pt, v, bus_dist=v * T) for pt in ("ped_p1", "ped_p2")
        for v in (1.0, 5.0) for T in (4, 8)]
json.dump(out, open(sys.argv[1], "w"), sort_keys=True)
print("backend", os.environ.get("TRAFFIC_BACKEND", "traci"), len(out))
