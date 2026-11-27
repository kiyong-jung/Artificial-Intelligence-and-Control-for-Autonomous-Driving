"""충돌로 끝난 바퀴마다 끝 기록의 event와 버스에 가장 가까운 차 셋(id, 거리, 차로, 속도), 버스의 마지막 3초 속도를 적는다.

    python research/snail/collision_partner.py data/research/eval/rd5_s63000_rich2.jsonl.gz ...
"""
import gzip
import json
import math
import sys

for path in sys.argv[1:]:
    episodes = {}
    with gzip.open(path, "rt") as f:
        for line in f:
            r = json.loads(line)
            episodes.setdefault(r["episode"], []).append(r)
    for episode, rows in episodes.items():
        end = rows[-1]
        reason = end["event"]["termination_reason"]
        if reason not in ("collision", "pedestrian_collision", "red_light"):
            continue
        last = next(r for r in reversed(rows) if r["state"]["ego"].get("x") is not None)
        ego = last["state"]["ego"]
        near = sorted(((math.dist((ego["x"], ego["y"]), (v["x"], v["y"])), v) for v in last["state"]["vehicles"]), key=lambda p: p[0])[:3]
        speeds = [round(r["state"]["ego"].get("speed") or 0, 1) for r in rows[-7:]]
        extra = {k: v for k, v in end["event"].items() if k != "termination_reason" and v not in (None, False, [], {}, 0)}
        print(path.split("/")[-1].replace(".jsonl.gz", ""), episode, reason, end["sim_time"], ego.get("lane_id"), "ego v", speeds)
        print("   event", json.dumps(extra, ensure_ascii=False)[:300])
        for d, v in near:
            print(f"   {d:5.1f} m  {v['id'][:44]:44s} {v['lane_id']:16s} v={v['speed']:.1f}")
