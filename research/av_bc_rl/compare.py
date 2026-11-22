"""evaluate.py 결과(metrics/<label>_<split>.json) 여러 개를 표로 묶는다.

학습 seed가 여럿인 그룹은 모든 seed의 에피소드를 한데 모아 평균을 낸다(seed마다 평균을 낸 뒤 다시 평균하지 않는다).
seed 사이 차이는 완주율과 정차 수를 seed별로 따로 적어 보인다. 바퀴 시간은 완주한 에피소드만으로 내고 그 수를 같이 적는다.
횟수 지표(급제동, 30초 넘는 정지, 차선 왕복)는 에피소드당이다. 일찍 끝난 에피소드도 그대로 센다.

    python research/av_bc_rl/compare.py --split final --groups expert=expert rule=rule \
        bc=bc_s0,bc_s1,bc_s2 ppo=ppo_s2101,ppo_s2102,ppo_s2103 --out research/av_bc_rl/report/final_table.md
"""
import argparse
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
COLUMNS = [  # (표 머리, 에피소드 키, 배율, 형식)
    ("급제동/에피소드", "hard_brakes", 1, "{:.2f}"),
    ("최대 감속(m/s²)", "max_decel", 1, "{:.1f}"),
    ("SUMO 개입", "speed_intervened_rate", 100, "{:.1f}%"),
    ("행동-움직임 불일치", "label_mismatch_rate", 100, "{:.2f}%"),
    ("jerk 위반", "jerk_violation_rate", 100, "{:.1f}%"),
    ("신호 아닌 30초+ 정지/에피소드", "stuck_events", 1, "{:.2f}"),
    ("차선 왕복/에피소드", "lane_change_pingpong", 1, "{:.2f}"),
]


def load(label, split):
    return json.loads((HERE / "metrics" / f"{label}_{split}.json").read_text(encoding="utf-8"))


def mean(values, scale, fmt):
    values = [float(v) for v in values if v is not None]
    return fmt.format(float(np.mean(values)) * scale) if values else "-"


def row(name, blocks):
    episodes = [e for block in blocks for e in block["episodes"]]
    if not episodes:
        return f"| {name} | 0 | - |" + " - |" * (len(COLUMNS) + 3)
    ends = {}
    for e in episodes:
        ends[e["termination"]] = ends.get(e["termination"], 0) + 1
    arrived = [e for e in episodes if e["arrived"]]
    per_seed_arrived = "/".join(f"{100 * sum(e['arrived'] for e in b['episodes']) / len(b['episodes']):.0f}" for b in blocks)
    per_seed_served = "/".join(f"{np.mean([e['bus_stops_served'] for e in b['episodes']]):.2f}" for b in blocks)
    lap = f"{np.mean([e['lap_time_s'] for e in arrived]):.0f} ({len(arrived)})" if arrived else "-"
    served = f"{np.mean([e['bus_stops_served'] for e in episodes]):.2f}"
    if len(blocks) > 1:
        completion = f"{100 * len(arrived) / len(episodes):.0f}% ({per_seed_arrived})"
        served += f" ({per_seed_served})"
    else:
        completion = f"{100 * len(arrived) / len(episodes):.0f}%"
    cells = [mean([e[key] for e in episodes], scale, fmt) for _, key, scale, fmt in COLUMNS]
    label = f"{name} ({len(blocks)} seed)" if len(blocks) > 1 else name
    return (f"| {label} | {len(episodes)} | {', '.join(f'{k} {v}' for k, v in sorted(ends.items()))} | {completion} | "
            f"{lap} | {served} | " + " | ".join(cells) + " |")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--split", default="final")
    parser.add_argument("--groups", nargs="+", required=True, help="이름=label1,label2 ...")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    groups, scenarios = [], []
    for item in args.groups:
        name, labels = item.split("=", 1)
        runs = [load(label, args.split) for label in labels.split(",")]
        groups.append((name, runs))
        for run in runs:
            scenarios += [b["scenario"] for b in run["scenarios"] if b["scenario"] not in scenarios]
    lines = []
    for scenario in scenarios:
        lines.append(f"\n### {scenario} ({args.split} seed)\n")
        lines.append("| 정책 | 에피소드 | 종료 | 완주율(seed별) | 바퀴 시간 s(완주 수) | 정차/4(seed별) | "
                     + " | ".join(c[0] for c in COLUMNS) + " |")
        lines.append("|---" * (len(COLUMNS) + 6) + "|")
        for name, runs in groups:
            blocks = [b for run in runs for b in run["scenarios"] if b["scenario"] == scenario]
            lines.append(row(name, blocks))
    text = "\n".join(lines) + "\n"
    print(text)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
