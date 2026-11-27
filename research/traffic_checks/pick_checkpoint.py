"""같은 seed로 평가한 체크포인트 중 수집에 쓸 하나를 고른다. 기준은 평가 전에 정했다.

1. 다섯 교통량을 합친 안전 사건(차 충돌 + 보행자 충돌 + 적색 통과)이 가장 적은 것
2. 같으면 완주가 가장 많은 것
3. 그래도 같으면 앞에 적은 것(먼저 학습한 것)

    python research/traffic_checks/pick_checkpoint.py --seed 63000 rd5=results/rich_dagger_s1/bc_it5/model.pt r6=... > out.txt

마지막 줄에 고른 모델 경로를 쓴다.
"""
import argparse
import collections
import gzip
import json
import re

SCENARIOS = ("quiet", "baseline", "busy", "heavy", "rich2")
SAFETY = ("collision", "pedestrian_collision", "red_light")
END = re.compile(r'"(?:terminated|truncated)":\s*true')


def endings(label, seed, scenario):
    counts = collections.Counter()
    with gzip.open(f"data/research/eval/{label}_s{seed}_{scenario}.jsonl.gz", "rt") as f:
        for line in f:
            if END.search(line):
                counts[json.loads(line)["event"]["termination_reason"]] += 1
    return counts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", required=True)
    parser.add_argument("candidates", nargs="+", help="label=model.pt")
    args = parser.parse_args()
    rows = []
    for order, item in enumerate(args.candidates):
        label, model = item.split("=", 1)
        per = {s: endings(label, args.seed, s) for s in SCENARIOS}
        safety = sum(per[s][k] for s in SCENARIOS for k in SAFETY)
        arrived = sum(per[s]["arrived"] for s in SCENARIOS)
        rows.append((safety, -arrived, order, label, model, per))
        cells = "  ".join(f"{s} {per[s]['arrived']}/{sum(per[s].values())} 안전{sum(per[s][k] for k in SAFETY)}" for s in SCENARIOS)
        print(f"{label}: 안전 사건 {safety}, 완주 {arrived}  |  {cells}")
    best = min(rows)
    print(f"고름: {best[3]}")
    print(best[4])


if __name__ == "__main__":
    main()
