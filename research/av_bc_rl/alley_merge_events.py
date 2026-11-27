"""골목 출구(G24 → 와우산로 R112.112)에서 버스와 끼어들려는 골목 차(id가 alley_G27)가 만난 장면을 수집 기록에서 찾아 센다.

만남은 골목 차가 골목 끝(정지선 8m 안)에 있을 때 버스가 와우산로(R112)에서 교차로 70m 안으로 다가온 경우다.
만남마다 누가 먼저 교차로를 지났는지, 골목 차가 선 채 기다린 시간, 버스가 지나기 전에 줄인 속도(가장 빠를 때와
그 뒤 가장 느릴 때의 차), 두 차 앞범퍼 사이 최소 간격을 잰다.

    python research/av_bc_rl/alley_merge_events.py data/final_v2/rich2_s0_*.jsonl.gz --out events.jsonl

웹페이지와 README의 한 바퀴 영상은 버스가 줄여서 먼저 보낸 유일한 만남이 든 바퀴(rich2_s0_011 30번)다. 같은 출구에서
바로 뒤 골목 차는 기다리게 해서, 양보와 비양보가 한 바퀴에 함께 나온다.

    python research/av_bc_rl/make_lap_video.py data/final_v2/rich2_s0_011.jsonl.gz --episode 30 --fps 15 \
        --out web_page/assets/videos/full_lap.mp4 --title "최종 모델 · 복합 교통 한 바퀴 완주" --slow 151 171 \
        --note 157 162 "골목 차 ① 양보|버스가 35→18 km/h로 줄여 먼저 보냄" \
        --note 162 168 "골목 차 ② 비양보|골목 차가 3초 기다리고 버스가 먼저 지나감"
"""
import argparse
import gzip
import json
import math
import re
import statistics
from collections import defaultdict
from multiprocessing import Pool

NODE_Y = -64.942            # 교차로(X_wausan11). 버스 y가 이보다 크면 지난 것
STOP_LINE = (814.8, -61.8)  # 골목(G24) 끝
APPROACH = {"R112", ":X_wausan11_2", ":X_wausan11_3"}
END = re.compile(r'"termination_reason":\s*"([a-z_]+)"')


def scan(path):
    rows, ends = defaultdict(list), {}
    with gzip.open(path, "rt", encoding="utf-8") as f:
        for line in f:
            end = END.search(line)
            if end:
                ends[int(line[len('{"episode":'):line.index(",")])] = end.group(1)
            if "R112" not in line and "G24" not in line and ":X_wausan11" not in line:
                continue
            row = json.loads(line)
            cars = {v["id"]: v for v in row["state"]["vehicles"] if v["id"].startswith("alley_G27")}
            rows[row["episode"]].append((row["t"], row["state"]["ego"], cars))
    events = []
    for episode, steps in rows.items():
        bus = {t: ego for t, ego, _ in steps}
        for car in {c for _, _, cars in steps for c in cars}:
            track = [(t, ego, cars[car]) for t, ego, cars in steps if car in cars]
            at_exit = [(t, ego, v) for t, ego, v in track
                       if v["road_id"] == "G24" and math.dist((v["x"], v["y"]), STOP_LINE) < 8]
            near = [(t, ego) for t, ego, v in at_exit if ego["road_id"] in APPROACH and 0 < NODE_Y - ego["y"] < 70]
            if not near:
                continue
            t0 = near[0][0]
            left = next((t for t, ego, v in track if t > t0 and v["road_id"] != "G24"), None)
            passed = next((t for t in sorted(bus) if t >= t0 and bus[t]["y"] > NODE_Y), None)
            if left is None or passed is None or any(t not in bus for t in range(t0, passed + 1)):
                continue
            speeds = [bus[t]["speed"] for t in range(t0, passed + 1)]
            peak = max(range(len(speeds)), key=speeds.__getitem__)
            events.append(dict(
                file=path.split("/")[-1], episode=episode, car=car, end=ends.get(episode),
                first="car" if left < passed else "bus", step_near=t0, step_car_left=left, step_bus_passed=passed,
                bus_distance=round(NODE_Y - near[0][1]["y"], 1),
                car_wait_s=0.5 * sum(1 for _, _, v in at_exit if v["speed"] < 0.3),
                bus_slowdown_kmh=round(3.6 * (speeds[peak] - min(speeds[peak:])), 1),
                min_gap_m=round(min(math.dist((ego["x"], ego["y"]), (v["x"], v["y"]))
                                    for t, ego, v in track if t0 <= t <= max(left, passed) + 6), 1)))
    return len(ends), events


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("jsonl", nargs="+")
    parser.add_argument("--out", help="만남 하나에 한 줄(JSONL)")
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args()
    with Pool(args.workers) as pool:
        found = pool.map(scan, sorted(args.jsonl))
    laps = sum(n for n, _ in found)
    events = [e for _, chunk in found for e in chunk]
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.writelines(json.dumps(e, ensure_ascii=False) + "\n" for e in events)
    print(f"{laps}바퀴 중 {len({(e['file'], e['episode']) for e in events})}바퀴에서 만남 {len(events)}번")
    for first, name in (("bus", "버스가 먼저 지남"), ("car", "골목 차가 먼저 들어감")):
        group = [e for e in events if e["first"] == first]
        print(f"{name}: {len(group)}번 ({100 * len(group) / len(events):.1f}%), "
              f"버스 거리 중앙값 {statistics.median(e['bus_distance'] for e in group):.1f} m, "
              f"골목 차 대기 중앙값 {statistics.median(e['car_wait_s'] for e in group):.1f}초, "
              f"최소 간격 중앙값 {statistics.median(e['min_gap_m'] for e in group):.1f} m")
    braked = [e for e in events if e["bus_slowdown_kmh"] >= 3.6]
    print(f"버스가 지나기 전 3.6 km/h(1 m/s) 넘게 줄인 만남: {len(braked)}번")
    for e in braked:
        print("  ", json.dumps(e, ensure_ascii=False))


if __name__ == "__main__":
    main()
