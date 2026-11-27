"""차로 변경을 신호 60m 안(near)과 블록 중간(mid)으로 나누고, 끼어들기 중 방금 투입된 차가 새 앞차가 된 것을 센다(평균±SE).
    PYTHONPATH=. python research/snail/mid_split.py
cut_in_new는 끼어들기 순간의 같은 차로 앞차가 1초 안에 처음 보인 차인 수다(그 자리에 투입된 골목 차)."""
import sys, glob, collections, json, numpy as np
from multiprocessing import Pool
sys.path.insert(0, "research/av_bc_rl")
from filter_episodes import read_episodes
from env.data_schema import OBSERVATION_FEATURES as F
from traffic_around_bus import ahead, behind, SIDE, NEAR
GAP, SPEED = F.index("current_leader_gap"), F.index("ego_speed")
def one(path):
    out = []
    for rows in read_episodes(path):
        c = collections.Counter(); first = {}; prev = None
        num = den = 0.0
        for r in rows:
            for v in r["state"]["vehicles"]:
                first.setdefault(v["id"], r["sim_time"])
            light = r["state"].get("traffic_light")
            zone = "near" if light and light.get("distance_to_stop_line") is not None and light["distance_to_stop_line"] < NEAR else "mid"
            o = r["observation"]
            if zone == "mid":
                den += 1; num += ahead(o, "current")
            req = r["action"]["lane_change"]
            if req and (prev is None or prev["action"]["lane_change"] != req):
                c["lc"] += 1; c[f"lc_{zone}"] += 1
                hit = ahead(o, SIDE[req] + "1") or behind(o, SIDE[req] + "1")
                c[f"lc_target30_{zone}"] += hit
            if prev is not None and not req and not prev["action"]["lane_change"] and o[SPEED] > 0.05 and (prev["observation"][GAP] - o[GAP]) * 50 > 10:
                c["cut_in"] += 1
                lead = [d for d in r["detected_vehicles"] if d["lane_relation"] == "same" and d["relative_longitudinal_position"] > 0]
                if lead:
                    l = min(lead, key=lambda d: d["relative_longitudinal_position"])
                    if r["sim_time"] - first.get(l["id"], -99) <= 1.0:
                        c["cut_in_new_car"] += 1
            prev = r
        c["mid_leader30_lap"] = num / den if den else 0.0
        out.append(c)
    return out
if __name__ == "__main__":
    E = "data/research/eval/"
    groups = {f"{p}:{s}": sorted(glob.glob(f"{E}{p}_s600[02][04]_{s}.jsonl.gz")) for p in ("v7", "it16") for s in ("heavy", "mid40i0o", "mid20i20o", "x2", "x2mid40i0o")}
    files = sorted({f for fs in groups.values() for f in fs})
    with Pool(10) as pool:
        per = dict(zip(files, pool.map(one, files)))
    for g, fs in groups.items():
        laps = [l for f in fs for l in per[f]]
        def ms(k):
            v = np.array([l.get(k, 0) for l in laps], float); return f"{v.mean():.3f}±{v.std(ddof=1)/np.sqrt(len(v)):.3f}"
        print(g, len(laps), "lc", ms("lc"), "lc_mid", ms("lc_mid"), "lcT30_mid", ms("lc_target30_mid"), "lcT30_near", ms("lc_target30_near"), "cut_in", ms("cut_in"), "cut_in_new", ms("cut_in_new_car"), "midL30_lap", ms("mid_leader30_lap"))
