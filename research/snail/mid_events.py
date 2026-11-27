"""골목 출입 차(mid_in_, mid_out_)가 버스 가까이에서 나타나거나 사라진 횟수를 바퀴당으로 센다.

    PYTHONPATH=. python research/snail/mid_events.py data/research/eval/v7_s600[02][04]_mid40i0o.jsonl.gz

기록의 state.vehicles는 버스 150m 안의 차다. 그래서 mid_in_ 차가 처음 보인 거리가 150m보다 한참 짧으면 그 자리에 투입된 것이고,
mid_out_ 차가 마지막으로 보인 거리가 짧으면 그 자리에서 도착해 사라진 것이다(버스가 끝난 뒤 사라진 것은 빼려고 바퀴 마지막
스텝은 세지 않는다). same은 detected_vehicles 기준 같은 차로, ahead는 버스 앞이다.
"""
import collections
import gzip
import json
import math
import sys

RINGS = (10.0, 20.0, 30.0)


def scan(path):
    laps, seen, previous, lap = [], {}, set(), None
    with gzip.open(path, "rt") as f:
        for line in f:
            r = json.loads(line)
            if lap is None or r["episode"] != lap["episode"]:
                lap = collections.Counter(episode=r["episode"])
                laps.append(lap)
                seen, previous = {}, set()
            ego = r["state"]["ego"]
            if "x" not in ego:
                continue
            relation = {d["id"]: d for d in r["detected_vehicles"]}
            now = {}
            for v in r["state"]["vehicles"]:
                d = math.hypot(v["x"] - ego["x"], v["y"] - ego["y"])
                now[v["id"]] = d
                if v["id"].startswith("mid_in_") and v["id"] not in seen:
                    seen[v["id"]] = d
                    lap["in_seen"] += 1
                    near = relation.get(v["id"])
                    for ring in RINGS:
                        if d < ring:
                            lap[f"in_{ring:.0f}"] += 1
                            if near and near["lane_relation"] == "same":
                                lap[f"in_{ring:.0f}_same_{'ahead' if near['relative_longitudinal_position'] > 0 else 'behind'}"] += 1
            if not r["terminated"] and not r["truncated"]:
                for vid in previous - set(now):
                    if vid.startswith("mid_out_"):
                        for ring in RINGS:
                            if last[vid] < ring:
                                lap[f"out_{ring:.0f}"] += 1
            previous, last = set(now), now
    return laps


if __name__ == "__main__":
    rows = [lap for path in sys.argv[1:] for lap in scan(path)]
    keys = sorted({k for lap in rows for k in lap if k != "episode"})
    print(f"laps {len(rows)}", json.dumps({k: round(sum(lap[k] for lap in rows) / len(rows), 3) for k in keys}))
