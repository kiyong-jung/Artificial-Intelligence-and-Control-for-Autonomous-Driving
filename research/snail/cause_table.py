"""장면의 원인(taxi*, cut*, ped*, rich) 교통량을 heavy와 한 표로 묶는다. 장면마다 바퀴당 평균±SE와 1번·3번 넘게 나온 바퀴 수.

    PYTHONPATH=. python research/snail/cause_table.py [--labels v7 it16] [--seeds 60000 60024] heavy taxi25 ... > research/snail/cause_table.txt

건강 열은 버스 없는 측정(sim_health_cause*.json, 6 seed), 나머지는 그 seed들의 기록이다. 장면 정의는 alley_events.py(cut_in,
lc_left30, lc_right30, stop_follower50, 출현·소멸)와 cause_events.py(S4·S5·S6·A1)에 있다. 실패한 바퀴도 센다.
"""
import argparse
import collections
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, "research/av_bc_rl")
sys.path.insert(0, "research/snail")
from alley_events import scan as scan_alley  # noqa: E402
from cause_events import scan as scan_cause  # noqa: E402
from filter_episodes import ARTIFACT, episode_flags, read_episodes  # noqa: E402
from traffic_around_bus import laps  # noqa: E402

E = "data/research/eval/"
SAFETY = ("collision", "pedestrian_collision", "red_light")
SCENES = (("S1 stop_follower50", "stop_follower50"), ("S2 lc_left30", "lc_left30"), ("S3 lc_right30", "lc_right30"),
          ("A1 a1_left", "a1_left"), ("A1 a1_right", "a1_right"), ("S4 s4_seen", "s4_seen"), ("S4 s4_block", "s4_block"),
          ("S5 s5_30", "s5_30"), ("S5 s5_team", "s5_team"), ("S5 s5_brake", "s5_brake"), ("cut_in", "cut_in"),
          ("cut_in with a standing taxi within 50 m", "taxi_cut_in"),
          ("S6 s6_any", "s6_any"), ("S6 s6_team", "s6_team"))


def read(path):
    flags = [episode_flags(rows) for rows in read_episodes(path)]
    events = [collections.Counter({**a, **c}) for a, c in zip(scan_alley(path), scan_cause(path))]
    return laps(path), events, flags


def ms(values):
    v = np.array(values, float)
    return f"{v.mean():.2f}±{v.std(ddof=1) / np.sqrt(len(v)):.2f}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenarios", nargs="+")
    parser.add_argument("--labels", nargs="+", default=["v7", "it16"])
    parser.add_argument("--seeds", nargs="+", default=["60000", "60024"])
    parser.add_argument("--health", nargs="+", default=[f"research/av_bc_rl/metrics/sim_health_cause{k}.json" for k in ("", "2", "3")])
    args = parser.parse_args()
    files = [f"{E}{p}_s{seed}_{s}.jsonl.gz" for p in args.labels for s in args.scenarios for seed in args.seeds]
    files = [f for f in files if os.path.exists(f)]
    with Pool(12) as pool:
        per = dict(zip(files, pool.map(read, files)))
    health = {s: h for path in args.health if os.path.exists(path) for s, h in json.load(open(path))["table"].items()}
    if health:
        print("bus-free health (6 seeds): scenario arrivals/h inserted_share backlog_end standing120 collision_pairs "
              "person_pairs teleports taxi_stops/h route_near route_mid")
        for s in args.scenarios:
            h = health.get(s)
            if h:
                print(s, f"{h['arrived_per_h']:.0f}", f"{h['inserted_share']:.3f}", f"{h['backlog_end']:.0f}",
                      f"{h['standing_120s']:.2f}", f"{h['collision_pairs']:.2f}", h.get("person_collision_pairs", "n/a"),
                      h["teleports"], f"{h.get('taxi_stops_per_h', 0):.0f}", f"{h['route_near']:.2f}", f"{h['route_mid']:.2f}")
        print()
    print("driver: label scenario n arrived collision pedestrian red stuck/time stuck_roads served missed lap_s "
          "appear50(alley/other) vanish50(alley/other) teleport jump_laps artifact_laps")
    groups = {}
    for p in args.labels:
        for s in args.scenarios:
            mine = [f for f in files if f.startswith(f"{E}{p}_s") and f.endswith(f"_{s}.jsonl.gz")]
            if not mine:
                continue
            rows = [lap for f in mine for lap in per[f][0]]
            events = [e for f in mine for e in per[f][1]]
            flags = [fl for f in mine for fl in per[f][2]]
            groups[(p, s)] = events
            reasons = [r["reason"] for r in rows]
            arrived = [r for r in rows if r["reason"] == "arrived"]
            stuck = collections.Counter(r["end_road"] for r in rows if r["reason"] in ("stuck", "time_limit"))
            kinds = {k: sum(1 for fl in flags if fl.get(k)) for k in ARTIFACT if any(fl.get(k) for fl in flags)}

            def mean(k):
                return np.mean([e.get(k, 0) for e in events])

            print(p, s, len(rows), len(arrived), *(reasons.count(k) for k in SAFETY),
                  sum(r in ("stuck", "time_limit") for r in reasons), json.dumps(dict(stuck), separators=(",", ":")),
                  f"{mean('served'):.2f}", f"{mean('missed'):.2f}",
                  f"{np.mean([r['duration_s'] for r in arrived]):.0f}" if arrived else "-",
                  f"{mean('appear50_alley'):.2f}/{mean('appear50_other'):.2f}",
                  f"{mean('vanish50_alley'):.2f}/{mean('vanish50_other'):.2f}", sum(e["teleport"] for e in events),
                  sum(1 for fl in flags if fl.get("vehicle_jump") or fl.get("pedestrian_jump")),
                  f"{sum(1 for fl in flags if any(fl.get(k) for k in ARTIFACT))}{kinds or ''}".replace(" ", ""))
    print()
    print("scenes per lap (mean±SE) and laps with >=1 / >=3 (all laps, failed ones included)")
    for title, key in SCENES:
        print(title)
        for (p, s), events in groups.items():
            n = len(events)
            print(f"  {p} {s} {ms([e.get(key, 0) for e in events])} "
                  f">=1 {sum(e.get(key, 0) >= 1 for e in events)}/{n} >=3 {sum(e.get(key, 0) >= 3 for e in events)}/{n}")


if __name__ == "__main__":
    main()
