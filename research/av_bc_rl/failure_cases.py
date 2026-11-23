"""평가 기록(data/research/eval/<label>_<split>_<교통량>.jsonl.gz)에서 완주하지 못한 에피소드를 골라 어디서 왜 끝났는지 적는다.

stuck이면 버스가 마지막으로 5m 넘게 움직인 뒤 머문 곳과 그때의 관측(정류장 거리, 경로 차로 차이, 신호)을, 충돌이면 충돌한
곳을 적는다. 배경 차끼리의 충돌(ego가 끼지 않은 충돌) 스텝 수도 센다.

    python research/av_bc_rl/failure_cases.py --split final
    python research/av_bc_rl/failure_cases.py --split final --labels ppo_s2101 ppo_s2102 --out research/av_bc_rl/report/failures.md
"""
import argparse
import collections
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, REPO)
from behavior_metrics import load_episodes  # noqa: E402
from env import data_schema  # noqa: E402

FEATURES = list(data_schema.OBSERVATION_FEATURES)


def stop_place(episode):
    """마지막으로 5m 넘게 움직인 뒤의 첫 기록."""
    x0, y0 = episode[-1]["state"]["ego"]["x"], episode[-1]["state"]["ego"]["y"]
    k = len(episode) - 1
    while k > 0:
        e = episode[k - 1]["state"]["ego"]
        if (e["x"] - x0) ** 2 + (e["y"] - y0) ** 2 >= 25.0:
            break
        k -= 1
    return episode[k]


def describe(path):
    rows, places = [], collections.Counter()
    background = 0
    for index, episode in enumerate(load_episodes(path)):
        background += sum(1 for r in episode if r["event"].get("simulation_collided") and not r["event"].get("collided"))
        reason = episode[-1]["event"].get("termination_reason")
        if reason == "arrived":
            continue
        record = stop_place(episode) if reason == "stuck" else episode[-1]
        ego = record["state"]["ego"]
        observed = dict(zip(FEATURES, record["observation"]))
        light = (record["state"].get("traffic_light") or {}).get("state")
        places[f"{reason}@{ego['lane_id']}"] += 1
        rows.append(f"| {index} | {reason} | {record['sim_time']:.0f} | `{ego['lane_id']}` {ego['lane_position']:.1f}m | "
                    f"{ego['speed']:.2f} | {light} | {observed['stop_distance']:.2f} | "
                    f"{observed['left1_route_connectivity']:+.2f} / {observed['current_route_connectivity']:+.2f} / "
                    f"{observed['right1_route_connectivity']:+.2f} |")
    return rows, places, background


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--split", default="final")
    parser.add_argument("--labels", nargs="*", default=None, help="생략하면 그 split의 기록 전부")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    pattern = os.path.join(REPO, "data", "research", "eval", f"*_{args.split}_*.jsonl.gz")
    paths = sorted(glob.glob(pattern))
    if args.labels:
        paths = [p for p in paths if os.path.basename(p).split(f"_{args.split}_")[0] in args.labels]
    lines = []
    for path in paths:
        name = os.path.basename(path).removesuffix(".jsonl.gz")
        rows, places, background = describe(path)
        lines.append(f"\n### {name}: 못 끝낸 에피소드 {len(rows)}개, 배경 충돌 스텝 {background}\n")
        if rows:
            lines.append("| 에피소드 | 끝 | 시각(s) | 머문 곳 | 속도 | 신호 | 정류장 거리 | 연결성(왼쪽 / 지금 / 오른쪽) |")
            lines.append("|---|---|---|---|---|---|---|---|")
            lines.extend(rows)
            lines.append("\n곳별: " + ", ".join(f"{k} {v}" for k, v in places.most_common()))
    text = "\n".join(lines) + "\n"
    print(text)
    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)


if __name__ == "__main__":
    main()
