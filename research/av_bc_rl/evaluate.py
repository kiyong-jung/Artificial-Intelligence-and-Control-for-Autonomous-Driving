"""정책 하나를 고정 seed 집합에서 닫힌 루프로 주행시키고 버스 행동 지표를 JSON으로 남긴다.

AI-Scientist-v3 과제의 평가기 자리다. 주행은 collect_pomdp_data.py가 하고(같은 환경, 같은 기록), 지표는
behavior_metrics.py가 센다. seed 집합은 task.md의 dev(1000번대), train(2000번대), final(9000번대)이다.

    python research/av_bc_rl/evaluate.py --policy expert --split dev --scenarios busy --episodes 30
    python research/av_bc_rl/evaluate.py --policy bc --model results/bc_x/model.pt --split final --label bc_seed0
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from behavior_metrics import episode_metrics, load_episodes, summarize  # noqa: E402

SPLITS = {"dev": 1000, "train": 2000, "final": 9000}
SCENARIO_OFFSET = {"quiet": 0, "baseline": 300, "busy": 600}   # 교통량마다 다른 seed를 쓴다
MAX_STEPS = 1400   # 700초. 전문가의 busy 한 바퀴(정차 포함)는 330~480초다


def run(args, scenario):
    seed = SPLITS[args.split] + SCENARIO_OFFSET[scenario]
    name = f"{args.label}_{args.split}_{scenario}"
    out_dir = os.path.join(REPO, "data", "research", "eval")
    jsonl = os.path.join(out_dir, name + ".jsonl.gz")
    if not os.path.exists(jsonl):
        command = [sys.executable, os.path.join(REPO, "collect_pomdp_data.py"), "--map", "hapjeong",
                   "--scenario", scenario, "--seed", str(seed), "--policy", args.policy,
                   "--episodes", str(args.episodes), "--workers", str(args.workers),
                   "--max-steps", str(MAX_STEPS), "--out-dir", out_dir, "--name", name]
        if args.model:
            command += ["--model", args.model]
        started = time.time()
        subprocess.run(command, cwd=REPO, check=True, stdout=subprocess.DEVNULL)
        print(f"  {name}: {time.time() - started:.0f}s")
    per_episode = [episode_metrics(records) for records in load_episodes(jsonl)]
    return {"scenario": scenario, "seed_start": seed, "jsonl": os.path.relpath(jsonl, REPO),
            "summary": summarize(per_episode),
            "episodes": [{k: v for k, v in e.items() if k != "stop_events"} for e in per_episode]}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", required=True, choices=["route-follow", "expert", "bc", "ppo"])
    parser.add_argument("--model", default=None, help="bc 또는 ppo(Residual 포함) model.pt")
    parser.add_argument("--split", choices=tuple(SPLITS), default="dev")
    parser.add_argument("--scenarios", nargs="+", default=["quiet", "baseline", "busy"], choices=tuple(SCENARIO_OFFSET))
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--label", default=None, help="결과 이름. 기본은 정책 이름")
    args = parser.parse_args()
    if args.policy in ("bc", "ppo") and not args.model:
        parser.error("--policy bc/ppo는 --model이 필요합니다.")
    args.label = args.label or args.policy
    results = {"label": args.label, "policy": args.policy, "model": args.model, "split": args.split,
               "episodes_per_scenario": args.episodes, "max_steps": MAX_STEPS,
               "scenarios": [run(args, scenario) for scenario in args.scenarios]}
    out = os.path.join(HERE, "metrics", f"{args.label}_{args.split}.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=1)
    for block in results["scenarios"]:
        s = block["summary"]
        print(f"{args.label:>16} {block['scenario']:<8} n={s['episodes']} {s['terminations']} "
              f"lap={s['lap_time_s']['mean'] or 0:.0f}s hard={s['hard_brakes']['mean']:.2f} "
              f"interv={s['speed_intervened_rate']['mean'] * 100:.1f}% mismatch={s['label_mismatch_rate']['mean'] * 100:.2f}% "
              f"jerk={s['jerk_violation_rate']['mean'] * 100:.1f}% stuck={s['stuck_events']['mean']:.2f} "
              f"served={s['bus_stops_served']['mean']:.2f} missed={(s['bus_stops_missed']['mean'] or 0):.2f}")
    print(out)


if __name__ == "__main__":
    main()
