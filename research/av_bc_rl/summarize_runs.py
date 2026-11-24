"""evaluate.py가 쓴 지표 파일 여럿을 한 줄씩 표로 낸다: 완주, 적색 통과, 충돌(차·보행자), 정차, 순간이동, 실패한 곳.

    python research/av_bc_rl/summarize_runs.py research/av_bc_rl/metrics/v6_*_dev.json
    python research/av_bc_rl/summarize_runs.py --first 10 research/av_bc_rl/metrics/v6_bins_s0_dev.json

--first N은 교통량마다 앞 N개 에피소드만 센다(같은 seed로 에피소드 수가 다른 실행과 견줄 때).
"""
import argparse
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from behavior_metrics import load_episodes  # noqa: E402


def failure_places(jsonl, first):
    """완주하지 못한 에피소드가 끝난 도로와 그때 속도."""
    places = []
    for index, episode in enumerate(load_episodes(jsonl)):
        reason = episode[-1]["event"].get("termination_reason")
        if (first is None or index < first) and reason != "arrived":
            ego = episode[-1]["state"]["ego"]
            places.append(f"{reason}@{ego.get('road_id', '?')}({ego.get('speed', 0.0):.1f}m/s)")
    return places


def row(path, first, places):
    with open(path, encoding="utf-8") as f:
        result = json.load(f)
    episodes, where = [], []
    for block in result["scenarios"]:
        episodes += block["episodes"][:first]
        if places:
            where += failure_places(os.path.join(REPO, block["jsonl"]), first)
    n = len(episodes)
    count = lambda key: sum(bool(e[key]) for e in episodes)  # noqa: E731
    ends = collections.Counter(e["termination"] for e in episodes if not e["arrived"])
    return (f"| {result['label']} | {count('arrived')}/{n} | {count('red_light_violation')} | "
            f"{count('vehicle_collision')}+{count('pedestrian_collision')} | "
            f"{sum(e['bus_stops_served'] for e in episodes) / n:.2f} | {sum(e['bus_stops_missed'] for e in episodes)} | "
            f"{sum(e['teleports'] for e in episodes)} | {dict(ends) or ''} {' '.join(where)} |")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("metrics", nargs="+")
    parser.add_argument("--first", type=int, default=None)
    parser.add_argument("--places", action="store_true", help="실패한 도로와 속도를 주행 기록에서 읽어 붙인다")
    args = parser.parse_args()
    print("| 실행 | 완주 | 적색 통과 | 충돌(차+보행자) | 정차(4곳 중) | 지나친 정류장 | 순간이동 | 못 끝낸 이유 |")
    print("|---|---|---|---|---|---|---|---|")
    for path in args.metrics:
        print(row(path, args.first, args.places))


if __name__ == "__main__":
    main()
