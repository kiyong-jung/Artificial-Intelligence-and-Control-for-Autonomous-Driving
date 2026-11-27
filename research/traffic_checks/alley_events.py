"""골목 합류(alley*) 기록에서 바퀴별로 차가 버스 가까이 나타나거나 사라진 수, 끼어들기한 차가 어디서 왔는지, 장면 횟수를 센다.

    PYTHONPATH=. python research/traffic_checks/alley_events.py data/research/eval/v7_s600[02][04]_alley20.jsonl.gz

기록의 state.vehicles는 버스 150m 안의 차다. 처음 보인 거리가 50m보다 짧으면 그 차는 그 자리에서 투입된 것이고(150m 원 밖에서
0.5초에 100m를 올 수 없다), 바퀴 첫 스텝에 이미 있던 차는 빼고 센다. 마지막으로 보인 거리가 50m보다 짧은데 바퀴가 이어지면
그 자리에서 도착하거나 치워져 사라진 것이다. alley_는 골목에서 출발한 차, other는 나머지 모든 차다.
  appear{10,20,50}_{alley,other}   버스에서 그 거리 안에 처음 나타난 차
  vanish50_{alley,other}           버스 50m 안에서 사라진 차
  teleport                         event.teleporting_ids에 나온 서로 다른 차
  cut_in, cut_in_alley             traffic_around_bus.py와 같은 끼어들기 정의, 그때 같은 차로 가장 가까운 앞차가 골목 차인 수
  lc_left30, lc_right30            왼쪽·오른쪽 차로 변경 요청 시작 때 옮길 차로 앞뒤 30m 안에 차(관측 기준)
  lc30_alley                       그중 옮길 차로 30m 안 차 가운데 골목 차가 있던 수
  stop_follower50                  정류장 정차 시작 때 같은 차로 뒤 50m 안에 차
cut_in_where에는 골목 차 끼어들기 때 버스가 있던 도로와 그 차의 출발 도로를 센다.
"""
import collections
import gzip
import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "av_bc_rl")))
from traffic_around_bus import GAP, SIDE, SPEED, ahead, behind  # noqa: E402
from env.data_schema import OBSERVATION_FEATURES as F  # noqa: E402

DWELL, FOLLOWER = F.index("stop_dwell_remaining"), F.index("current_follower_gap")
RINGS = (10.0, 20.0, 50.0)


def origin(vid):
    return vid.split("_to_")[0][len("alley_"):]


def scan(path):
    laps, lap = [], None
    with gzip.open(path, "rt") as f:
        for line in f:
            r = json.loads(line)
            if lap is None or r["episode"] != lap["episode"]:
                lap = collections.Counter(episode=r["episode"])
                lap["where"], lap["teleported"] = collections.Counter(), set()
                laps.append(lap)
                seen, last, previous = None, {}, None
            ego = r["state"]["ego"]
            if "x" not in ego:
                continue
            now = {v["id"]: math.hypot(v["x"] - ego["x"], v["y"] - ego["y"]) for v in r["state"]["vehicles"]}
            if seen is None:
                seen = set(now)
            for vid in set(now) - seen:
                kind = "alley" if vid.startswith("alley_") else "other"
                for ring in RINGS:
                    lap[f"appear{ring:.0f}_{kind}"] += now[vid] < ring
            seen |= set(now)
            if not r["terminated"] and not r["truncated"]:
                for vid in set(last) - set(now):
                    lap[f"vanish50_{'alley' if vid.startswith('alley_') else 'other'}"] += last[vid] < 50.0
            lap["teleported"] |= set(r["event"].get("teleporting_ids") or ())
            o, request = r["observation"], r["action"]["lane_change"]
            if previous is not None and not request and not previous["action"]["lane_change"] and o[SPEED] > 0.05 \
                    and (previous["observation"][GAP] - o[GAP]) * 50.0 > 10:
                lap["cut_in"] += 1
                leaders = [d for d in r["detected_vehicles"]
                           if d["lane_relation"] == "same" and d["relative_longitudinal_position"] > 0]
                lead = min(leaders, key=lambda d: d["relative_longitudinal_position"]) if leaders else None
                if lead and lead["id"].startswith("alley_"):
                    lap["cut_in_alley"] += 1
                    lap["where"][f"{ego.get('road_id')}<-{origin(lead['id'])}"] += 1
            if request and (previous is None or previous["action"]["lane_change"] != request):
                side = SIDE[request]
                if ahead(o, f"{side}1") or behind(o, f"{side}1"):
                    lap[f"lc_{side}30"] += 1
                    lap["lc30_alley"] += any(d["lane_relation"] == side and abs(d["relative_longitudinal_position"]) <= 30
                                             and d["id"].startswith("alley_") for d in r["detected_vehicles"])
            if o[DWELL] > 0 and (previous is None or previous["observation"][DWELL] <= 0):
                lap["stop_follower50"] += o[FOLLOWER] > -0.999
            last, previous = now, r
    for lap in laps:
        lap["teleport"] = len(lap.pop("teleported"))
    return laps


if __name__ == "__main__":
    rows = [lap for path in sys.argv[1:] for lap in scan(path)]
    keys = sorted({k for lap in rows for k in lap if k not in ("episode", "where")})
    print(f"laps {len(rows)}", json.dumps({k: round(sum(lap[k] for lap in rows) / len(rows), 3) for k in keys}))
    print(dict(sum((lap["where"] for lap in rows), collections.Counter())))
