"""실패한 바퀴의 끝 상황: 끝난 이유, 시각, 버스 차로와 위치, 마지막으로 움직인 시각, 같은 차로 앞차(id, 거리, 그 차의 차로·속도).

    python research/snail/failures.py data/research/eval/v7_s60000_cut15a.jsonl.gz
"""
import gzip
import json
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
        if reason == "arrived":
            continue
        ego = end["state"]["ego"]
        moved = next((r["sim_time"] for r in reversed(rows) if r["state"]["ego"].get("speed", 0) > 0.1), None)
        leaders = [d for d in end["detected_vehicles"] if d["lane_relation"] == "same" and d["relative_longitudinal_position"] > 0]
        lead = min(leaders, key=lambda d: d["relative_longitudinal_position"]) if leaders else None
        state = next((v for v in end["state"]["vehicles"] if lead and v["id"] == lead["id"]), None)
        print(path.split("/")[-1], episode, reason, end["sim_time"], ego.get("lane_id"), round(ego.get("lane_position", 0), 1),
              "moved", moved, "lead", lead and lead["id"][:48], lead and round(lead["relative_longitudinal_position"], 1),
              state and state["lane_id"], state and state["speed"])
