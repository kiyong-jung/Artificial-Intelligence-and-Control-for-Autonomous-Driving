"""JSONL 주행 기록에서 버스 주변에 차가 어디에 얼마나 있는지, 버스가 차와 얽힌 장면이 바퀴당 몇 번인지 센다.

    python research/av_bc_rl/traffic_around_bus.py heavy='data/final/heavy_s0_00[0-4].jsonl.gz' --out x.json

구역은 다음 신호 정지선까지 거리로 나눈다. near는 60m 안, mid(블록 중간)는 그 밖이거나 앞에 신호가 없을 때다.
  n50            버스 50m 안 배경 차 수(구역별 시간 평균)
  leader30       같은 차로 앞 30m 안에 차가 있는 시간 비율(관측 기준, count_interactions.py와 같은 이웃 정의)
  side30         바로 옆 차로 앞뒤 30m 안에 차가 있는 시간 비율(관측 기준)
  lc             차로 변경 요청 시작 수. lc_target30은 그때 옮길 차로 앞뒤 30m 안에 차가 있던 수, lc_follower30은 뒤 30m
  stop           정류장 정차 시작 수. stop_follower50은 그때 같은 차로 뒤 50m 안에 차가 있던 수
  cut_in         요청 없이 달리는 중 같은 차로 앞차 간격이 한 판단(0.5초)에 10m 넘게 준 수(count_interactions.py와 같은 정의)
  slow_mid_s     블록 중간에서 같은 차로 30m 안 앞차가 1~9m/s로 달리는 동안 버스가 그 뒤에 있던 시간(초)
  snail_lead_s   느린 차(id가 snail_)가 같은 차로 30m 안 앞차였던 시간(초)
  snail_met      같은 차로 앞차로 만난 느린 차 수. snail_passed는 그중 나중에 버스 뒤(상대 위치 < 0)로 간 수
  snail_bus_left_lane  느린 차가 30m 안 앞차일 때 버스가 차로를 옮긴 수. snail_moved_away는 버스는 그대로인데 느린 차가 옆 차로로 간 수
  passed         같은 차로 앞차였다가 나중에 버스 뒤로 간 모든 차 수(버스가 앞지른 차)
"""
import argparse
import collections
import glob
import gzip
import json
import math
import multiprocessing
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from env.data_schema import OBSERVATION_FEATURES as F  # noqa: E402

NEAR = 60.0
SIDE = {1: "left", -1: "right"}
GAP, SPEED, DWELL = F.index("current_leader_gap"), F.index("ego_speed"), F.index("stop_dwell_remaining")
NEIGHBOR = 30.0 / 50.0   # 관측의 간격은 50m로 나눈 값이다. 앞차 없음 1, 뒤차 없음 -1


def ahead(o, lane):
    return o[F.index(f"{lane}_leader_gap")] < NEIGHBOR


def behind(o, lane):
    return o[F.index(f"{lane}_follower_gap")] > -NEIGHBOR


def laps(path):
    """파일 하나의 바퀴별 집계."""
    out, lap, previous = [], None, None
    with gzip.open(path, "rt") as f:
        for line in f:
            r = json.loads(line)
            if lap is None or r["episode"] != lap["episode"]:
                if lap is not None:
                    out.append(finish(lap, previous))
                lap = collections.Counter(episode=r["episode"])
                lap["start"], lap["met"], lap["behind"] = r["sim_time"], set(), set()
                previous = None
            step(lap, r, previous)
            previous = r
    if lap is not None:
        out.append(finish(lap, previous))
    return out


def step(lap, r, previous):
    dt = r["sim_time"] - previous["sim_time"] if previous else 0.5
    state, ego = r["state"], r["state"]["ego"]
    light = state.get("traffic_light")
    zone = "near" if light and light.get("distance_to_stop_line") is not None \
        and light["distance_to_stop_line"] < NEAR else "mid"
    speed = {v["id"]: v["speed"] for v in state["vehicles"]}
    detected = r["detected_vehicles"]
    lap[f"{zone}_s"] += dt
    lap[f"{zone}_n50"] += dt * sum(1 for v in state["vehicles"]
                                   if math.hypot(v["x"] - ego["x"], v["y"] - ego["y"]) <= 50.0)
    o = r["observation"]
    leaders = [d for d in detected if d["lane_relation"] == "same" and 0 < d["relative_longitudinal_position"] <= 30]
    lap[f"{zone}_leader30"] += dt * ahead(o, "current")
    lap[f"{zone}_side30"] += dt * any(ahead(o, lane) or behind(o, lane) for lane in ("left1", "right1"))
    if leaders:
        lead = min(leaders, key=lambda d: d["relative_longitudinal_position"])
        lead_speed = speed.get(lead["id"], ego["speed"] + lead["relative_longitudinal_speed"])
        if zone == "mid" and 1.0 < lead_speed < 9.0:
            lap["slow_mid_s"] += dt
        if lead["id"].startswith("snail_"):
            lap["snail_lead_s"] += dt
    snail = previous and previous.get("_snail")
    if snail:
        now = next((d for d in detected if d["id"] == snail), None)
        if r["action"].get("lane_change_actual"):
            lap["snail_bus_left_lane"] += 1
        elif now is not None and now["lane_relation"] in ("left", "right", "left2", "right2"):
            lap["snail_moved_away"] += 1
    r["_snail"] = lead["id"] if leaders and lead["id"].startswith("snail_") else None
    for d in detected:
        if d["lane_relation"] == "same" and d["relative_longitudinal_position"] > 0:
            lap["met"].add(d["id"])
        elif d["lane_relation"] != "other" and d["relative_longitudinal_position"] < -2 and d["id"] in lap["met"]:
            lap["behind"].add(d["id"])
    request = r["action"]["lane_change"]
    if request and (previous is None or previous["action"]["lane_change"] != request):
        lane = SIDE[request] + "1"
        lap["lc"] += 1
        lap[f"lc_{zone}"] += 1
        lap["lc_target30"] += ahead(o, lane) or behind(o, lane)
        lap["lc_follower30"] += behind(o, lane)
    if previous is not None and not request and not previous["action"]["lane_change"] \
            and r["observation"][SPEED] > 0.05 and (previous["observation"][GAP] - r["observation"][GAP]) * 50.0 > 10:
        lap["cut_in"] += 1
    if o[DWELL] > 0 and (previous is None or previous["observation"][DWELL] <= 0):
        lap["stop"] += 1
        lap["stop_follower50"] += o[F.index("current_follower_gap")] > -0.999   # count_interactions.py와 같다
    lap["background_collisions"] += r["event"].get("background_collisions", 0)
    lap["end"] = r["sim_time"]


def finish(lap, last):
    met, behind = lap.pop("met"), lap.pop("behind")
    lap["passed"] = len(behind)
    lap["snail_met"] = sum(1 for v in met if v.startswith("snail_"))
    lap["snail_passed"] = sum(1 for v in behind if v.startswith("snail_"))
    lap["duration_s"] = lap.pop("end") - lap.pop("start")
    result = dict(lap)
    result["reason"] = last["event"].get("termination_reason") or "unfinished"
    result["end_road"] = last["state"]["ego"].get("road_id")
    return result


def summarize(rows):
    n = len(rows)
    total = collections.Counter()
    for row in rows:
        total.update({k: v for k, v in row.items() if isinstance(v, (int, float)) and k != "episode"})
    s = {"laps": n, "reasons": dict(collections.Counter(r["reason"] for r in rows)),
         "stuck_roads": dict(collections.Counter(r["end_road"] for r in rows if r["reason"] in ("stuck", "time_limit")))}
    for zone in ("near", "mid"):
        s[f"{zone}_time_share"] = total[f"{zone}_s"] / max(total["near_s"] + total["mid_s"], 1e-9)
        for key in ("n50", "leader30", "side30"):
            s[f"{zone}_{key}"] = total[f"{zone}_{key}"] / max(total[f"{zone}_s"], 1e-9)
    for key in ("stop", "stop_follower50", "lc", "lc_near", "lc_mid", "lc_target30", "lc_follower30", "cut_in", "slow_mid_s", "snail_lead_s",
                "snail_met", "snail_passed", "snail_bus_left_lane", "snail_moved_away", "passed", "background_collisions"):
        s[key] = total[key] / max(n, 1)
    arrived = [r["duration_s"] for r in rows if r["reason"] == "arrived"]
    s["lap_s"] = float(np.mean(arrived)) if arrived else None
    return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in s.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("groups", nargs="+", help="이름=glob")
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    files = {name: sorted(glob.glob(pattern)) for name, pattern in (g.split("=", 1) for g in args.groups)}
    jobs = [path for paths in files.values() for path in paths]
    with multiprocessing.Pool(min(args.workers, len(jobs))) as pool:
        per_file = dict(zip(jobs, pool.map(laps, jobs)))
    table = {name: summarize([lap for path in paths for lap in per_file[path]]) for name, paths in files.items()}
    for name, s in table.items():
        print(name, json.dumps(s, ensure_ascii=False), flush=True)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump({"files": files, "table": table}, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
