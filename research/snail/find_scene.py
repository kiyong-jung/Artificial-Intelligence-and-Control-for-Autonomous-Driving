"""영상용 장면 찾기: 차로 변경 요청 순간 옮길 차로 앞뒤 30m 차 수, 50m 안 차 수, 느린 차 수로 정렬한다.
    PYTHONPATH=. python research/snail/find_scene.py data/research/eval/it16_s60000_x2.jsonl.gz"""
import gzip, json, sys
sys.path.insert(0, ".")
from env.data_schema import OBSERVATION_FEATURES as F
eps = {}
for line in gzip.open(sys.argv[1], "rt"):
    r = json.loads(line); eps.setdefault(r["episode"], []).append(r)
hits = []
for ep, rows in eps.items():
    ok = rows[-1]["event"]["termination_reason"] == "arrived"
    start = rows[0]["sim_time"]
    for prev, r in zip(rows, rows[1:]):
        req = r["action"]["lane_change"]
        if not req or prev["action"]["lane_change"] == req:
            continue
        o = r["observation"]; side = "left1" if req > 0 else "right1"
        lead, fol = o[F.index(f"{side}_leader_gap")] * 50, -o[F.index(f"{side}_follower_gap")] * 50
        n_target = (lead < 30) + (fol < 30)
        snails = sum(1 for v in r["state"]["vehicles"] if v["id"].startswith("snail_") and abs(v["x"] - r["state"]["ego"]["x"]) + abs(v["y"] - r["state"]["ego"]["y"]) < 60)
        cars = sum(1 for v in r["state"]["vehicles"] if abs(v["x"] - r["state"]["ego"]["x"]) + abs(v["y"] - r["state"]["ego"]["y"]) < 50)
        tl = (r["state"]["traffic_light"] or {}).get("distance_to_stop_line") or 999
        hits.append((n_target, cars, snails, ok, tl >= 60, ep, round(r["sim_time"] - start, 1), round(lead, 1), round(fol, 1)))
hits.sort(reverse=True)
for h in hits[:12]:
    print(h)
