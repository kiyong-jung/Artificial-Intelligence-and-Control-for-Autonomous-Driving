"""평가 기록 폴더에서 베스트 한 바퀴와 실패 유형마다 워스트 하나를 골라 영상으로 만든다.

베스트는 혼잡 교통량에서 정류장을 모두 서고 완주한 바퀴 중 차로 변경이 가장 많은 것이다. 워스트는 실패 유형마다
가장 붐비는 교통량의 첫 바퀴이고, 마지막 --last초만 담는다.

    python research/av_bc_rl/render_best_worst.py data/research/verify_map322 --out videos --label "신경망"
"""
import argparse
import collections
import gzip
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = {"quiet": "한산", "baseline": "보통", "busy": "혼잡", "heavy": "매우 혼잡(1.5배)"}
ENDS = {"collision": "차와 충돌", "pedestrian_collision": "보행자 충돌", "red_light": "적색 통과", "stuck": "멈춰 섬",
        "time_limit": "시간 초과"}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder")
    parser.add_argument("--out", required=True)
    parser.add_argument("--label", default="신경망")
    parser.add_argument("--prefix", default="")
    parser.add_argument("--last", type=float, default=120.0)
    args = parser.parse_args()
    best, worst = None, {}
    busiest = next((s for s in ("heavy", "busy") if os.path.exists(os.path.join(args.folder, f"{s}.jsonl.gz"))), "busy")
    for scenario in ("heavy", "busy", "baseline", "quiet"):
        path = os.path.join(args.folder, f"{scenario}.jsonl.gz")
        if not os.path.exists(path):
            continue
        changes = collections.Counter()
        with gzip.open(path, "rt", encoding="utf-8") as f:
            for line in f:
                row = json.loads(line)
                if row["action"].get("lane_change_actual"):
                    changes[row["episode"]] += 1
                reason = row["event"].get("termination_reason")
                if reason == "arrived" and row["event"].get("stops_served") == 4:
                    if scenario == busiest and (best is None or changes[row["episode"]] > best[2]):
                        best = (path, row["episode"], changes[row["episode"]])
                elif reason and reason != "arrived":
                    worst.setdefault(reason, (path, row["episode"], scenario))
    jobs = []
    if best:
        jobs.append((best[0], best[1], f"{args.prefix}best_busy_lap.mp4", f"{NAMES[busiest]}, 완주 ({args.label})", []))
    for reason, (path, episode, scenario) in worst.items():
        jobs.append((path, episode, f"{args.prefix}worst_{reason}.mp4", f"{NAMES[scenario]}, {ENDS.get(reason, reason)} ({args.label})",
                     ["--last", str(args.last), "--fps", "8", "--hold", "4"]))
    for path, episode, name, title, extra in jobs:
        subprocess.run([sys.executable, os.path.join(HERE, "make_lap_video.py"), path, "--episode", str(episode),
                        "--out", os.path.join(args.out, name), "--title", title, *extra], check=True)
    if not worst:
        print("실패한 바퀴가 없어 워스트 영상은 없다")


if __name__ == "__main__":
    main()
