"""evaluate.py 결과(metrics/<label>_<split>.json) 여러 개를 표로 묶는다. 학습 seed가 여럿이면 seed 사이 평균±표준편차.

    python research/av_bc_rl/compare.py --split final --groups expert=expert rule=route-follow \
        bc=bc_s0,bc_s1,bc_s2 ppo=ppo_s2101,ppo_s2102,ppo_s2103 --out research/av_bc_rl/report/final_table.md
"""
import argparse
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
COLUMNS = [  # (표 머리, 요약 키, 배율, 형식)
    ("완주율", "arrived", 100, "{:.0f}%"),
    ("바퀴 시간(s)", "lap_time_s", 1, "{:.0f}"),
    ("급제동/바퀴", "hard_brakes", 1, "{:.2f}"),
    ("최대 감속(m/s²)", "max_decel", 1, "{:.1f}"),
    ("SUMO 개입", "speed_intervened_rate", 100, "{:.1f}%"),
    ("행동-움직임 불일치", "label_mismatch_rate", 100, "{:.2f}%"),
    ("jerk 위반", "jerk_violation_rate", 100, "{:.1f}%"),
    ("갇힘/바퀴", "stuck_events", 1, "{:.2f}"),
    ("차선 왕복/바퀴", "lane_change_pingpong", 1, "{:.2f}"),
    ("정차/4", "bus_stops_served", 1, "{:.2f}"),
]


def load(label, split):
    return json.loads((HERE / "metrics" / f"{label}_{split}.json").read_text(encoding="utf-8"))


def cell(values, scale, fmt):
    values = [v for v in values if v is not None]
    if not values:
        return "-"
    mean = float(np.mean(values)) * scale
    if len(values) == 1:
        return fmt.format(mean)
    return fmt.format(mean) + " ± " + fmt.format(float(np.std(values)) * scale).rstrip("%")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--split", default="final")
    parser.add_argument("--groups", nargs="+", required=True, help="이름=label1,label2 ...")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    lines = []
    scenarios = None
    groups = []
    for item in args.groups:
        name, labels = item.split("=", 1)
        runs = [load(label, args.split) for label in labels.split(",")]
        groups.append((name, runs))
        scenarios = scenarios or [block["scenario"] for block in runs[0]["scenarios"]]
    for scenario in scenarios:
        lines.append(f"\n### {scenario} ({args.split} seed)\n")
        lines.append("| 정책 | 에피소드 | 종료 | " + " | ".join(c[0] for c in COLUMNS) + " |")
        lines.append("|---" * (len(COLUMNS) + 3) + "|")
        for name, runs in groups:
            blocks = [next(b for b in run["scenarios"] if b["scenario"] == scenario) for run in runs]
            terminations = {}
            for block in blocks:
                for key, count in block["summary"]["terminations"].items():
                    terminations[key] = terminations.get(key, 0) + count
            episodes = sum(block["summary"]["episodes"] for block in blocks)
            cells = [cell([block["summary"][key]["mean"] for block in blocks], scale, fmt) for _, key, scale, fmt in COLUMNS]
            ends = ", ".join(f"{k} {v}" for k, v in sorted(terminations.items()))
            lines.append(f"| {name}{' (' + str(len(runs)) + ' seed)' if len(runs) > 1 else ''} | {episodes} | {ends} | " + " | ".join(cells) + " |")
    text = "\n".join(lines) + "\n"
    print(text)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
