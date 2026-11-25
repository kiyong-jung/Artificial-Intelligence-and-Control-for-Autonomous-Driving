"""골목 출입(mid*) 교통량의 바퀴당 상호작용과 표준오차(평균±SE), 건강, 버스 가까이 나타나거나 사라진 차를 한 표로 묶는다.

    PYTHONPATH=. python research/snail/mid_table.py > research/snail/mid_table.txt

arrivals/h·mid/near 밀도는 버스 없는 건강 측정(sim_health_mid.json, 6 seed), 나머지는 seed 60000·60024 기록이다.
safety는 충돌·보행자 충돌·신호 위반으로 끝난 바퀴 수, in<20m은 바퀴당 버스 20m 안에 투입된 골목 차, out<20m은 20m 안에서
사라진 골목 차다(mid_events.py). artifact는 filter_episodes.py의 ARTIFACT 표시가 붙은 바퀴 수와 그 종류다.
"""
import glob
import json
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, "research/av_bc_rl")
sys.path.insert(0, "research/snail")
from filter_episodes import ARTIFACT, episode_flags, read_episodes  # noqa: E402
from mid_events import scan  # noqa: E402
from traffic_around_bus import laps  # noqa: E402

E = "data/research/eval/"
SC = ("heavy", "mid40i0o", "mid20i20o", "x2", "x2mid40i0o")
SAFETY = ("collision", "pedestrian_collision", "red_light")


def both(path):
    return laps(path), scan(path), [episode_flags(rows) for rows in read_episodes(path)]


if __name__ == "__main__":
    health = json.load(open("research/av_bc_rl/metrics/sim_health_mid.json"))["table"]
    names = [(p, s) for p in ("v7", "it16") for s in SC]
    files = sorted({f for p, s in names for f in glob.glob(f"{E}{p}_s600[02][04]_{s}.jsonl.gz")})
    with Pool(10) as pool:
        per = dict(zip(files, pool.map(both, files)))
    print("bus-free health (6 seeds): scenario arrivals/h inserted_share standing120 collision_pairs route_near route_mid mid/near")
    for s in SC:
        h = health[s]
        print(s, f"{h['arrived_per_h']:.0f}", f"{h['inserted_share']:.3f}", f"{h['standing_120s']:.2f}", f"{h['collision_pairs']:.2f}",
              f"{h['route_near']:.2f}", f"{h['route_mid']:.2f}", f"{h['route_mid'] / h['route_near']:.3f}")
    print()
    print("policy scenario n arrived safety stuck/time midLeader30 midSide30 mid_n50 lc_target30 lc_follower30 cut_in in<10m/in<20m out<10m/out<20m lap_s artifact")
    for p, s in names:
        mine = [f for f in files if f.startswith(f"{E}{p}_s600") and f.endswith(f"_{s}.jsonl.gz")]
        rows = [lap for f in mine for lap in per[f][0]]
        events = [lap for f in mine for lap in per[f][1]]
        flags = [fl for f in mine for fl in per[f][2]]
        kinds = {k: sum(1 for fl in flags if fl.get(k)) for k in ARTIFACT if any(fl.get(k) for fl in flags)}
        if not rows:
            continue

        def ms(k):
            v = np.array([r.get(k, 0) for r in rows], float)
            return f"{v.mean():.2f}±{v.std(ddof=1) / np.sqrt(len(v)):.2f}"

        def share(k):
            return sum(r.get(f"mid_{k}", 0) for r in rows) / sum(r.get("mid_s", 0) for r in rows)

        def ev(k):
            return np.mean([e.get(k, 0) for e in events])

        reasons = [r["reason"] for r in rows]
        arr = [r for r in rows if r["reason"] == "arrived"]
        print(p, s, len(rows), len(arr), sum(r in SAFETY for r in reasons), sum(r in ("stuck", "time_limit") for r in reasons),
              f"{share('leader30'):.3f}", f"{share('side30'):.3f}", f"{share('n50'):.2f}",
              ms("lc_target30"), ms("lc_follower30"), ms("cut_in"),
              f"{ev('in_10'):.2f}/{ev('in_20'):.2f}", f"{ev('out_10'):.2f}/{ev('out_20'):.2f}",
              f"{np.mean([r['duration_s'] for r in arr]):.0f}",
              f"{sum(1 for fl in flags if any(fl.get(k) for k in ARTIFACT))}{kinds or ''}".replace(" ", ""))
