"""버스 옆이나 뒤에 붙어 다닌 배경 차를 찾는다. 버스가 3 m/s 넘게 달리는 동안만 센다.

    python research/snail/companions.py data/research/eval/r8_s63000_rich2.jsonl.gz [--episode 0]

옆: 버스 몸통(앞범퍼에서 뒤로 12 m)과 나란하고 옆으로 1.5~5 m. 뒤: 같은 줄(옆으로 1.5 m 안)에서 뒤범퍼 뒤 0~30 m.
차마다 가장 길게 이어진 구간(초)과 그동안의 버스 속도, 차 속도를 적는다. --episode가 없으면 바퀴 전체를 합쳐 분포만 낸다.
"""
import argparse
import collections
import gzip
import json
import math

BUS, DT, MOVING = 12.0, 0.5, 3.0


def relation(ego, v):
    h = math.radians(ego["heading"])
    dx, dy = v["x"] - ego["x"], v["y"] - ego["y"]
    lon, lat = dx * math.sin(h) + dy * math.cos(h), dx * math.cos(h) - dy * math.sin(h)
    if -BUS - 4 < lon < 4 and 1.5 < abs(lat) < 5:
        return "옆"
    if -BUS - 30 < lon < -BUS and abs(lat) < 1.5:
        return "뒤"
    return None


def streaks(rows):
    """(종류, 차) → 이어진 구간들의 [시작 t, 스텝 수, 버스 속도 합, 차 속도 합]."""
    open_, done = {}, []
    for r in rows:
        ego = r["state"]["ego"]
        seen = set()
        if ego.get("x") is not None and (ego.get("speed") or 0) > MOVING:
            for v in r["state"]["vehicles"]:
                kind = relation(ego, v)
                if kind:
                    key = (kind, v["id"])
                    seen.add(key)
                    s = open_.setdefault(key, [r["t"], 0, 0.0, 0.0, ego["road_id"]])
                    s[1] += 1; s[2] += ego["speed"]; s[3] += v["speed"]
        for key in [k for k in open_ if k not in seen]:
            done.append((key, open_.pop(key)))
    return done + list(open_.items())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("jsonl")
    parser.add_argument("--episode", type=int, default=None)
    args = parser.parse_args()
    episodes = collections.defaultdict(list)
    with gzip.open(args.jsonl, "rt") as f:
        for line in f:
            r = json.loads(line)
            if args.episode is None or r["episode"] == args.episode:
                episodes[r["episode"]].append(r)
    hist = {"옆": collections.Counter(), "뒤": collections.Counter()}
    moving = total = 0
    for episode, rows in episodes.items():
        total += len(rows)
        moving += sum(1 for r in rows if (r["state"]["ego"].get("speed") or 0) > MOVING)
        found = streaks(rows)
        for (kind, vid), (t, n, es, vs, road) in found:
            hist[kind][min(int(n * DT // 5) * 5, 30)] += 1
        if args.episode is not None:
            for (kind, vid), (t, n, es, vs, road) in sorted(found, key=lambda p: -p[1][1])[:15]:
                print(f"{kind} {n * DT:5.1f}초  바퀴 {t * DT:5.0f}초부터  {road:10s} 버스 {es / n * 3.6:4.1f} km/h  차 {vs / n * 3.6:4.1f} km/h  {vid}")
    print(f"바퀴 {len(episodes)}개, 달린 시간 비율 {moving / total:.2f}")
    for kind in hist:
        n = sum(hist[kind].values())
        print(kind, f"구간 {n}개(바퀴당 {n / len(episodes):.1f})", "  ".join(f"{b}~{b + 5}초 {hist[kind][b]}" if b < 30 else f"30초 이상 {hist[kind][b]}" for b in sorted(hist[kind])))


if __name__ == "__main__":
    main()
