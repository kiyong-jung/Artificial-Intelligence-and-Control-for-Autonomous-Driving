"""장면의 원인(cause: taxi*, cut*, ped*, rich) 기록에서 팀이 정한 장면 S4·S5·S6과 A1을 바퀴마다 센다. alley_events.py의
출현·소멸·순간이동·끼어들기·차로 변경·정류장 뒤차 수와 같은 바퀴 순서로 짝지어 쓴다.

    PYTHONPATH=. python research/snail/cause_events.py data/research/eval/v7_s600[02][04]_taxi25.jsonl.gz

  s4_seen      정류장 차로의 정류장 안 또는 그 15m 앞에 선(0.1m/s 미만) 택시(taxi_)가 버스 50m 안에 있었다(택시마다 한 번)
  s4_block     그 택시가 같은 차로 20m 안 앞차일 때 버스가 섰다(택시마다 한 번)
  taxi_cut_in  끼어들기(traffic_around_bus.py 정의)가 일어날 때 선 택시가 버스 50m 안에 있었다
  s5_30        골목 도로(G로 시작하고 신호 없는 교차로로 끝나는 도로. 지도 원래 흐름의 골목 차도, alley_ 차도 포함)에 마지막으로 보인 차가 15초 안에 버스와 같은 차로 앞 30m 안에 들어왔다
  s5_team      같은 조건에서 같은 도로 앞 50m 안(팀 정의, scripts/measure_s1_s6.py S5)
  s5_brake     s5_30 뒤 3초 안에 1m/s 넘게 달리던 버스의 가속도가 -1m/s² 아래로 내려갔다(골목 차 때문에 버스가 줄였다)
  s6_any       횡단보도 접근 중 보행자가 건넘(count_interactions.py와 같은 정의: crosswalk_distance 관측 0.5 미만=25m 안)
  s6_team      신호 없는 횡단보도 S6_wausanro 50m 안에서 그 횡단보도 위에 보행자가 있었다(통과마다 한 번, 팀 정의)
  a1_left/right  차로 변경 요청 시작 때 옮길 차로 뒤 30m 안에 차(관측). 그 차가 양보하는지 아닌지는 나누지 않는다
  served, missed 바퀴 끝의 정류장 정차 성공·놓침 수
"""
import collections
import gzip
import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "av_bc_rl")))
from traffic_around_bus import GAP, SIDE, SPEED, behind  # noqa: E402
from env.data_schema import OBSERVATION_FEATURES as F  # noqa: E402
from maps import load_map  # noqa: E402
from sumolib.net import readNet  # noqa: E402

CROSS_D, CROSS_BUSY = F.index("crosswalk_distance"), F.index("crosswalk_occupied")
MAP = load_map("hapjeong")
STOP_LANES = {s["lane"]: (float(s["startPos"]) - 15.0, float(s["endPos"]) + 1.0) for s in MAP["bus_stops"]}
# 골목 도로: 이름이 G로 시작하고 끝이 신호 교차로가 아닌 도로(팀 정의처럼 신호 교차로 합류 G19·G22는 S5에서 뺀다)
ALLEYS = {e.getID() for e in readNet(MAP["net_path"]).getEdges()
          if e.getID().lstrip("-").startswith("G") and not e.getToNode().getType().startswith("traffic_light")}
MERGE_WINDOW, BRAKE_WINDOW = 15.0, 3.0


def scan(path):
    laps, lap = [], None
    with gzip.open(path, "rt") as f:
        for line in f:
            r = json.loads(line)
            if lap is None or r["episode"] != lap["episode"]:
                lap = collections.Counter(episode=r["episode"])
                laps.append(lap)
                left_alley, s4_seen, s4_block, s5_30, s5_team = {}, set(), set(), set(), set()
                brake_until, s6_near, s6_busy, team_near, previous = None, False, False, False, None
            ego, t = r["state"]["ego"], r["sim_time"]
            if "x" not in ego:
                continue
            o = r["observation"]
            standing_taxis = []
            for v in r["state"]["vehicles"]:
                vid = v["id"]
                if v["road_id"] in ALLEYS:
                    left_alley[vid] = t
                window = STOP_LANES.get(v["lane_id"])
                if vid.startswith("taxi_") and window and v["speed"] < 0.1 and window[0] <= v["lane_position"] <= window[1] \
                        and math.hypot(v["x"] - ego["x"], v["y"] - ego["y"]) <= 50.0:
                    standing_taxis.append(vid)
                    s4_seen.add(vid)
                if vid in left_alley and t - left_alley[vid] <= MERGE_WINDOW \
                        and v["road_id"] == ego["road_id"] and 0 < v["lane_position"] - ego["lane_position"] <= 50.0:
                    s5_team.add(vid)
            for d in r["detected_vehicles"]:
                gap = d["relative_longitudinal_position"]
                if d["lane_relation"] != "same" or gap <= 0:
                    continue
                if d["id"] in standing_taxis and gap <= 20.0 and ego["speed"] < 0.1:
                    s4_block.add(d["id"])
                if d["id"] not in s5_30 and d["id"] in left_alley \
                        and t - left_alley[d["id"]] <= MERGE_WINDOW and gap <= 30.0:
                    s5_30.add(d["id"])
                    brake_until = t + BRAKE_WINDOW
            if brake_until is not None and t <= brake_until and ego["acceleration"] < -1.0 and ego["speed"] > 1.0:
                lap["s5_brake"] += 1
                brake_until = None
            request = r["action"]["lane_change"]
            if previous is not None and not request and not previous["action"]["lane_change"] and o[SPEED] > 0.05 \
                    and (previous["observation"][GAP] - o[GAP]) * 50.0 > 10:
                lap["taxi_cut_in"] += bool(standing_taxis)
            if request and (previous is None or previous["action"]["lane_change"] != request):
                lap[f"a1_{SIDE[request]}"] += behind(o, f"{SIDE[request]}1")
            near = o[CROSS_D] < 0.5
            if near and not s6_near:
                s6_busy = False
            if near and o[CROSS_BUSY] > 0.5 and not s6_busy:
                lap["s6_any"] += 1
                s6_busy = True
            s6_near = near
            nav = r["state"]["navigation"]
            here = (nav.get("crossing_id") or "").startswith(":S6_wausanro") and nav.get("distance_to_crossing", 1e9) <= 50.0
            if here and not team_near and any((p.get("road_id") or "").startswith(":S6_wausanro_c")
                                              for p in r["state"]["pedestrians"]):
                lap["s6_team"] += 1
                team_near = True
            if not here:
                team_near = False
            lap["s4_seen"], lap["s4_block"] = len(s4_seen), len(s4_block)
            lap["s5_30"], lap["s5_team"] = len(s5_30), len(s5_team)
            lap["served"], lap["missed"] = r["event"]["stops_served"], r["event"]["stops_missed"]
            previous = r
    return laps


if __name__ == "__main__":
    rows = [lap for path in sys.argv[1:] for lap in scan(path)]
    keys = sorted({k for lap in rows for k in lap if k != "episode"})
    print(f"laps {len(rows)}", json.dumps({k: round(sum(lap[k] for lap in rows) / len(rows), 3) for k in keys}))
    print(json.dumps({f"{k}>=1": sum(lap[k] >= 1 for lap in rows) for k in keys}))
