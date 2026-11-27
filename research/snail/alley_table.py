"""골목 합류(alley*) 교통량을 heavy·mid40i0o·x2와 한 표로 묶고, 장면이 바퀴마다 1번·3번 넘게 나온 바퀴 수를 센다.

    PYTHONPATH=. python research/snail/alley_table.py > research/snail/alley_table.txt

건강 열은 버스 없는 측정(sim_health_alley.json, 6 seed), 나머지는 seed 60000·60024 기록 48바퀴다. 값은 바퀴당 평균±SE다.
appear50은 버스 50m 안에 처음 나타난 차(골목 차/나머지), vanish50은 50m 안에서 사라진 차, teleport는 순간이동한 차 수,
jump는 filter_episodes.py의 vehicle_jump·pedestrian_jump 표시가 붙은 바퀴 수다(alley_events.py).
"""
import collections
import glob
import json
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, "research/av_bc_rl")
sys.path.insert(0, "research/snail")
from alley_events import scan  # noqa: E402
from filter_episodes import ARTIFACT, episode_flags, read_episodes  # noqa: E402
from traffic_around_bus import laps  # noqa: E402

E = "data/research/eval/"
SC = ("heavy", "mid40i0o", "x2", "alley20", "alley40", "x2alley20")
SAFETY = ("collision", "pedestrian_collision", "red_light")
SCENES = ("cut_in", "lc_left30", "lc_right30", "stop_follower50")


def both(path):
    return laps(path), scan(path), [episode_flags(rows) for rows in read_episodes(path)]


if __name__ == "__main__":
    health = {**json.load(open("research/av_bc_rl/metrics/sim_health_mid.json"))["table"],
              **json.load(open("research/av_bc_rl/metrics/sim_health_alley.json"))["table"]}
    names = [(p, s) for p in ("v7", "it16") for s in SC]
    files = sorted({f for p, s in names for f in glob.glob(f"{E}{p}_s600[02][04]_{s}.jsonl.gz")})
    with Pool(10) as pool:
        per = dict(zip(files, pool.map(both, files)))
    print("bus-free health (6 seeds): scenario arrivals/h inserted_share backlog_end standing120 collision_pairs teleports route_near route_mid")
    for s in SC:
        h = health[s]
        print(s, f"{h['arrived_per_h']:.0f}", f"{h['inserted_share']:.3f}", f"{h['backlog_end']:.0f}", f"{h['standing_120s']:.2f}",
              f"{h['collision_pairs']:.2f}", h.get("teleports", "n/a"), f"{h['route_near']:.2f}", f"{h['route_mid']:.2f}")
    print()
    print("policy scenario n arrived safety stuck/time stuck_roads midLeader30 lc_target30 lc_follower30 cut_in cut_in_alley lc30_alley "
          "appear50(alley/other) appear20 vanish50(alley/other) teleport jump_laps lap_s artifact")
    frequency = {}
    for p, s in names:
        mine = [f for f in files if f.startswith(f"{E}{p}_s600") and f.endswith(f"_{s}.jsonl.gz")]
        rows = [lap for f in mine for lap in per[f][0]]
        events = [lap for f in mine for lap in per[f][1]]
        flags = [fl for f in mine for fl in per[f][2]]
        if not rows:
            continue
        kinds = {k: sum(1 for fl in flags if fl.get(k)) for k in ARTIFACT if any(fl.get(k) for fl in flags)}

        def ms(values):
            v = np.array(values, float)
            return f"{v.mean():.2f}±{v.std(ddof=1) / np.sqrt(len(v)):.2f}"

        def ev(k):
            return np.mean([e.get(k, 0) for e in events])

        reasons = [r["reason"] for r in rows]
        arr = [r for r in rows if r["reason"] == "arrived"]
        stuck = {}
        for r in rows:
            if r["reason"] in ("stuck", "time_limit"):
                stuck[r["end_road"]] = stuck.get(r["end_road"], 0) + 1
        print(p, s, len(rows), len(arr), sum(r in SAFETY for r in reasons), sum(r in ("stuck", "time_limit") for r in reasons),
              json.dumps(stuck, separators=(",", ":")),
              f"{sum(r.get('mid_leader30', 0) for r in rows) / sum(r.get('mid_s', 0) for r in rows):.3f}",
              ms([r.get("lc_target30", 0) for r in rows]), ms([r.get("lc_follower30", 0) for r in rows]),
              ms([r.get("cut_in", 0) for r in rows]), ms([e.get("cut_in_alley", 0) for e in events]), f"{ev('lc30_alley'):.2f}",
              f"{ev('appear50_alley'):.2f}/{ev('appear50_other'):.2f}", f"{ev('appear20_alley'):.2f}/{ev('appear20_other'):.2f}",
              f"{ev('vanish50_alley'):.2f}/{ev('vanish50_other'):.2f}", sum(e["teleport"] for e in events),
              sum(1 for fl in flags if fl.get("vehicle_jump") or fl.get("pedestrian_jump")),
              f"{np.mean([r['duration_s'] for r in arr]):.0f}",
              f"{sum(1 for fl in flags if any(fl.get(k) for k in ARTIFACT))}{kinds or ''}".replace(" ", ""))
        frequency[(p, s)] = events
    print()
    print("laps with >=1 and >=3 of each scene (count of 48, all laps including failed ones)")
    print("policy scenario " + " ".join(f"{k}>=1 {k}>=3" for k in SCENES))
    for (p, s), events in frequency.items():
        print(p, s, " ".join(f"{sum(e.get(k, 0) >= 1 for e in events)}/{len(events)} {sum(e.get(k, 0) >= 3 for e in events)}/{len(events)}"
                             for k in SCENES))
    print()
    print("alley cut-ins by (bus road <- alley origin)")
    for (p, s), events in frequency.items():
        where = sum((e["where"] for e in events), collections.Counter())
        if where:
            print(p, s, sum(where.values()), dict(where.most_common()))
