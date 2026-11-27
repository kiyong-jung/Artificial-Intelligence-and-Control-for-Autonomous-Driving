"""수집한 JSONL에서 에피소드마다 버스(ego)가 '이상하게' 움직인 정도를 숫자로 센다.

    python research/av_bc_rl/behavior_metrics.py data/research/baseline_rule_v0.jsonl.gz [--json out.json]

판단 한 번이 0.5초다. 지표는 모두 기록에 남은 값만 쓴다(정책 내부 값은 보지 않는다).
"""
import argparse
import functools
import json
import os
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from collect_pomdp_data import iter_transitions  # noqa: E402
from env.road_config import BUS_COMFORT  # noqa: E402
from maps import load_map  # noqa: E402

MAP_ID = "hapjeong"        # 연구 주행 지도(evaluate.py --map hapjeong). 교차로 안에 서서 막는지 가를 때 쓴다
DT = 0.5
VIS = 50.0                 # 관측 거리 W(m). obs[9] = 현재 차선 선행차 거리 / W
STOPPED = 0.1              # m/s 미만이면 멈춤
MIN_STOP_S = 2.0           # 이만큼 이상 멈춰야 정지로 센다
STUCK_S = 30.0             # 이만큼 넘게 멈추면 갇힘
HARD_BRAKE = BUS_COMFORT["hard_braking"]                    # m/s². 버스 승차감 한계
LATERAL_LIMIT = BUS_COMFORT["lateral_acceleration_limit"]   # m/s². 버스 횡가속도 한계(곡선 속도 제한과 같은 값)
PINGPONG_S = 10.0          # 이 시간 안에 반대 방향으로 다시 바꾸면 왕복
QUEUE_GAP = 15.0           # 선행차가 이보다 가까우면 줄 서기
RED_NEAR = 35.0            # 정지선이 이보다 가까운 적·황색이면 신호 대기
RED_QUEUE = 150.0          # 정지선이 이보다 가까운 적·황색 뒤에 줄을 섰으면 신호 대기열
STOP_NEAR = 30.0           # 정류장이 이보다 가까우면 정류장 정차


def _near(distance, limit):
    """거리 0.0도 유효한 값이다. None만 '없음'으로 본다."""
    return distance is not None and distance < limit


def stop_reason(record):
    """정지 시작 시점의 기록으로 멈춘 이유를 고른다. 어느 것도 아니면 'unexplained'."""
    state, obs = record["state"], record["observation"]
    light = state.get("traffic_light") or {}
    red = bool(light.get("state")) and light["state"][:1] in "rRyYu"
    if red and _near(light.get("distance_to_stop_line"), RED_NEAR):
        return "signal"
    if obs[9] * VIS < QUEUE_GAP:
        return "signal_queue" if red and _near(light.get("distance_to_stop_line"), RED_QUEUE) else "queue"
    nav = state.get("navigation") or {}
    if _near(nav.get("distance_to_stop"), STOP_NEAR):
        return "bus_stop"
    if _near(nav.get("distance_to_crossing"), 15.0):
        return "crosswalk"
    if state["ego"].get("road_id", "").startswith(":"):
        return "in_junction"
    return "unexplained"


def lateral_acceleration(records):
    """판단마다 횡가속도 크기(m/s²) = 속도 × 요 레이트. 요 레이트는 기록된 heading(도)의 판단 사이 변화다.
    SUMO의 heading은 뒤범퍼에서 앞범퍼를 본 방향이라 차체가 실제로 도는 속도다. 도착해 ego가 사라진 마지막 판단은 0이다."""
    egos = [r["state"]["ego"] for r in records] + [(records[-1].get("next_state") or {}).get("ego", {})]
    out = np.zeros(len(records))
    for i, (a, b) in enumerate(zip(egos, egos[1:])):
        if "heading" in a and "heading" in b:
            turn = (b["heading"] - a["heading"] + 180.0) % 360.0 - 180.0
            out[i] = abs(np.radians(turn)) / DT * 0.5 * (a["speed"] + b["speed"])
    return out


def _place(road_id):
    """교차로 안 도로(:J82_15)는 교차로 이름(J82)으로 묶는다."""
    return road_id[1:].rsplit("_", 1)[0] if road_id.startswith(":") else road_id


@functools.lru_cache(maxsize=None)
def conflict_free_lanes(map_id=MAP_ID):
    """교차로 안 차로 가운데 다른 어느 연결(차, 횡단보도)과도 겹치지 않는 차로. 도로가 갈라지거나 차로 수만 바뀌는
    곳이다(합정 J2는 R54가 R42 직진과 R65 우회전으로 갈라질 뿐이다). 여기 선 차는 아무도 막지 않는다. SUMO 배경 차도
    겹치는 연결이 있을 때만 교차로를 비운다(MSVehicle::keepClear)."""
    root = ET.parse(load_map(map_id)["net_path"]).getroot()
    free = set()
    for junction in root.iter("junction"):
        lanes = (junction.get("intLanes") or "").split()
        free.update(lanes[int(request.get("index"))] for request in junction.iter("request") if "1" not in request.get("foes"))
    return frozenset(free)


def junction_stops(records):
    """교차로 안 차로(:J2_1_0)에 멈춰 있던 구간마다 (교차로, 시작 t, 길이 초, 그때 신호 상태, 막는지). 같은 교차로
    안에서 멈춤이 이어진 판단을 하나로 센다. 겹치는 연결이 있는 차로에 섰으면 막는 것(blocking)이다. 지도에 없는 차로는
    막는 것으로 본다."""
    free = conflict_free_lanes()
    out, run = [], None
    for record in records:
        ego = record["state"]["ego"]
        road = ego.get("road_id", "")
        inside = road.startswith(":") and ego["speed"] < STOPPED
        place = _place(road) if inside else None
        if run is not None and place == run["junction"]:
            run["duration_s"] += DT
            run["blocking"] = run["blocking"] or ego.get("lane_id") not in free
            continue
        if run is not None and run["duration_s"] >= MIN_STOP_S:
            out.append(run)
        light = (record["state"].get("traffic_light") or {}).get("state")
        run = ({"junction": place, "t": record["t"], "duration_s": DT, "signal": light,
                "blocking": ego.get("lane_id") not in free} if inside else None)
    if run is not None and run["duration_s"] >= MIN_STOP_S:
        out.append(run)
    return out


def episode_metrics(records):
    steps = len(records)
    speeds = [r["state"]["ego"]["speed"] for r in records]
    final = (records[-1].get("next_state") or {}).get("ego", {}).get("speed", speeds[-1])  # 도착하면 ego가 없다
    speed = np.array(speeds + [final])
    accel = np.diff(speed) / DT
    act = [r["action"] for r in records]
    last = records[-1]["event"]

    stops, i = [], 0
    while i < steps:
        if speed[i] < STOPPED:
            j = i
            while j < steps and speed[j] < STOPPED:
                j += 1
            if (j - i) * DT >= MIN_STOP_S:
                stops.append({"t": records[i]["t"], "duration_s": (j - i) * DT, "reason": stop_reason(records[i]),
                              "road": records[i]["state"]["ego"]["road_id"]})
            i = j
        else:
            i += 1

    changes = [(r["t"], r["action"]["lane_change_actual"]) for r in records if r["action"].get("lane_change_actual")]
    pingpong = sum(1 for (t0, d0), (t1, d1) in zip(changes, changes[1:]) if d0 == -d1 and (t1 - t0) * DT <= PINGPONG_S)
    requested = sum(1 for a in act if a["lane_change"] != 0)
    refused = sum(1 for a in act if a["lane_change"] != 0 and not a["lane_change_applied"])
    intervened = sum(1 for a in act if a.get("speed_intervened"))
    accel_cmd_pos = np.array([a["accel_raw"] > 0.05 for a in act])
    mismatch = int(np.sum(accel_cmd_pos & (accel < -0.5)))
    service = [r["event"].get("service_event") for r in records]
    recorded = any("service_event" in r["event"] for r in records)
    # 정차 사건이 기록된 데이터(스키마 v2)는 그 사건을 센다. 옛 데이터는 정류장 근처 5초 이상 정지로 어림한다.
    served = (service.count("served") if recorded
              else len({s["t"] for s in stops if s["reason"] == "bus_stop" and s["duration_s"] >= 5.0}))
    missed = service.count("missed") if recorded else None
    nav0, navN = records[0]["state"]["navigation"], records[-1]["state"]["navigation"]
    reasons = defaultdict(float)
    for s in stops:
        reasons[s["reason"]] += s["duration_s"]
    lateral = lateral_acceleration(records)
    lateral_by_place = defaultdict(float)
    for record, value in zip(records, lateral):
        place = _place(record["state"]["ego"]["road_id"])
        lateral_by_place[place] = max(lateral_by_place[place], float(value))
    return {
        "episode": records[0]["episode"],
        "steps": steps,
        "termination": last.get("termination_reason") or ("truncated" if records[-1]["truncated"] else "open"),
        "arrived": bool(last.get("arrived")),
        "vehicle_collision": bool(last.get("collided")) and not last.get("pedestrian_collision"),
        "pedestrian_collision": bool(last.get("pedestrian_collision")),
        "red_light_violation": bool(last.get("red_light_violation")),
        "teleports": sum(1 for r in records if "ego" in r["event"].get("teleporting_ids", ())),
        "lap_time_s": steps * DT if last.get("arrived") else None,
        "progress": 1.0 - (navN.get("distance_to_destination") or 0.0) / max(nav0.get("distance_to_destination") or 1.0, 1.0),
        "mean_speed": float(np.mean(speed)),
        "hard_brakes": int(np.sum(accel < -HARD_BRAKE)),
        "max_decel": float(-np.min(accel)) if accel.size else 0.0,
        "peak_acceleration": float(max(r["comfort"].get("peak_acceleration") or 0.0 for r in records)),
        "jerk_violation_rate": float(np.mean([bool(r["comfort"]["jerk_violation"]) for r in records])),
        "accel_violation_rate": float(np.mean([bool(r["comfort"]["acceleration_violation"]) for r in records])),
        "lateral_accel_peak": float(lateral.max()),
        "lateral_accel_violation_rate": float(np.mean(lateral > LATERAL_LIMIT)),
        "lateral_accel_peak_by_place": {k: v for k, v in lateral_by_place.items() if v > 0.5 * LATERAL_LIMIT},
        "speed_intervened_rate": intervened / steps,
        "label_mismatch_rate": mismatch / steps,
        "stops": len(stops),
        "stop_time_by_reason": dict(reasons),
        "unexplained_stops": sum(1 for s in stops if s["reason"] in ("unexplained", "in_junction")),
        "stuck_events": sum(1 for s in stops if s["duration_s"] >= STUCK_S and s["reason"] not in ("signal", "signal_queue")),
        "lane_changes": len(changes),
        "lane_change_pingpong": pingpong,
        "lane_change_requests": requested,
        "lane_change_refused": refused,
        "bus_stops_served": served,
        "bus_stops_missed": missed,
        "stop_events": stops,
        "junction_stops": junction_stops(records),
    }


def load_episodes(path):
    episodes = defaultdict(list)
    for record in iter_transitions(path):
        episodes[record["episode"]].append(record)
    return [sorted(v, key=lambda r: r["t"]) for _, v in sorted(episodes.items())]


SUMMARY_KEYS = ("arrived", "vehicle_collision", "pedestrian_collision", "red_light_violation", "teleports",
                "lap_time_s", "progress", "mean_speed", "hard_brakes", "max_decel", "peak_acceleration",
                "jerk_violation_rate",
                "lateral_accel_peak", "lateral_accel_violation_rate",
                "speed_intervened_rate", "label_mismatch_rate", "unexplained_stops", "stuck_events", "lane_changes",
                "lane_change_pingpong", "lane_change_refused", "bus_stops_served", "bus_stops_missed")


def summarize(per_episode):
    out = {"episodes": len(per_episode),
           "terminations": dict(sorted(defaultdict(int, {}).items()))}
    counts = defaultdict(int)
    for e in per_episode:
        counts[e["termination"]] += 1
    out["terminations"] = dict(counts)
    for key in SUMMARY_KEYS:
        values = [float(e[key]) for e in per_episode if e[key] is not None]
        out[key] = {"mean": float(np.mean(values)) if values else None,
                    "std": float(np.std(values)) if values else None,
                    "max": float(np.max(values)) if values else None,
                    "n": len(values)}
    reasons = defaultdict(float)
    for e in per_episode:
        for k, v in e["stop_time_by_reason"].items():
            reasons[k] += v
    out["stop_time_by_reason_total_s"] = dict(reasons)
    places = defaultdict(float)
    for e in per_episode:
        for k, v in e["lateral_accel_peak_by_place"].items():
            places[k] = max(places[k], v)
    out["lateral_accel_peak_by_place_max"] = dict(sorted(places.items(), key=lambda item: -item[1]))
    # 교차로 안에 10초, 30초 넘게 선 횟수. free_는 겹치는 연결이 없는 곳(아무도 막지 않음)이다.
    junctions = defaultdict(lambda: {"10s": 0, "30s": 0, "free_10s": 0, "free_30s": 0, "max_s": 0.0})
    for e in per_episode:
        for stop in e["junction_stops"]:
            counts, prefix = junctions[stop["junction"]], "" if stop["blocking"] else "free_"
            counts[prefix + "10s"] += stop["duration_s"] >= 10.0
            counts[prefix + "30s"] += stop["duration_s"] >= 30.0
            counts["max_s"] = max(counts["max_s"], stop["duration_s"])
    out["junction_stops_by_place"] = dict(sorted(junctions.items()))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("jsonl")
    parser.add_argument("--json", default=None, help="에피소드별 지표와 요약을 이 파일에 쓴다")
    args = parser.parse_args()
    per_episode = [episode_metrics(records) for records in load_episodes(args.jsonl)]
    summary = summarize(per_episode)
    for e in per_episode:
        print(f"ep{e['episode']:>3} {e['termination']:<10} t={e['steps'] * DT:6.1f}s v={e['mean_speed']:4.1f} "
              f"hard={e['hard_brakes']:2d} jerk%={e['jerk_violation_rate'] * 100:4.1f} interv%={e['speed_intervened_rate'] * 100:4.1f} "
              f"mismatch%={e['label_mismatch_rate'] * 100:4.1f} unexpl={e['unexplained_stops']} stuck={e['stuck_events']} "
              f"lc={e['lane_changes']} pp={e['lane_change_pingpong']} refused={e['lane_change_refused']} served={e['bus_stops_served']} missed={e['bus_stops_missed']}")
    print(json.dumps({k: v for k, v in summary.items()}, ensure_ascii=False, indent=1))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({"summary": summary, "episodes": per_episode}, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
