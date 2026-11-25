"""느린 차 교통량별 바퀴당 상호작용과 표준오차(평균±SE): PYTHONPATH=. python research/snail/se_table.py > research/snail/snail_table.txt"""
import glob, sys, numpy as np
from multiprocessing import Pool
sys.path.insert(0, "research/av_bc_rl")
from traffic_around_bus import laps
E = "data/research/eval/"
SC = ("heavy", "snail05v5", "snail10v5", "snail10v8", "snail20v8", "snail20v5", "x2", "x2snail10v5", "x2snail20v8")
if __name__ == "__main__":
    names = [(p, s) for p in ("v7", "it16") for s in SC]
    files = sorted({f for p, s in names for f in glob.glob(f"{E}{p}_s600[02][04]_{s}.jsonl.gz")})
    with Pool(8) as pool:
        per = dict(zip(files, pool.map(laps, files)))
    print("policy scenario n arrived midLeader30 lc_target30 lc_follower30 cut_in behind_snail_s met/passed(bus_lc/snail_left) lap_s")
    for p, s in names:
        rows = [l for f in files if f.startswith(f"{E}{p}_s600") and f.endswith(f"_{s}.jsonl.gz") for l in per[f]]
        def ms(k):
            v = np.array([r.get(k, 0) for r in rows], float)
            return f"{v.mean():.2f}±{v.std(ddof=1) / np.sqrt(len(v)):.2f}"
        arr = [r for r in rows if r["reason"] == "arrived"]
        mid = sum(r.get("mid_leader30", 0) for r in rows) / sum(r.get("mid_s", 0) for r in rows)
        m = lambda k: np.mean([r.get(k, 0) for r in rows])
        print(p, s, len(rows), len(arr), f"{mid:.3f}", ms("lc_target30"), ms("lc_follower30"), ms("cut_in"), ms("snail_lead_s"),
              f"{m('snail_met'):.2f}/{m('snail_passed'):.2f}({m('snail_bus_left_lane'):.2f}/{m('snail_moved_away'):.2f})",
              f"{np.mean([r['duration_s'] for r in arr]):.0f}")
