"""정책 하나를 고정 seed 집합에서 닫힌 루프로 주행시키고 버스 행동 지표를 JSON으로 남긴다.

AI-Scientist-v3 과제의 평가기 자리다. 주행은 collect_pomdp_data.py가 하고(같은 환경, 같은 기록), 지표는
behavior_metrics.py가 센다. seed 집합은 dev(1000번대), train(2000번대), final(9000번대)이고, --seed로 따로 줄 수도 있다. 요약은 results/metrics/에 쓴다.

    python research/av_bc_rl/evaluate.py --policy expert --split dev --scenarios busy --episodes 30
    python research/av_bc_rl/evaluate.py --policy bc --model results/bc_x/model.pt --split final --label bc_seed0
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from behavior_metrics import episode_metrics, load_episodes, summarize  # noqa: E402
sys.path.insert(0, REPO)
from env.road_config import SCENARIOS  # noqa: E402

SPLITS = {"dev": 1000, "train": 2000, "final": 9000}
SCENARIO_OFFSET = {"quiet": 0, "baseline": 300, "busy": 600, "heavy": 900}   # 교통량마다 다른 seed를 쓴다
MAX_STEPS = 1400   # 700초. 전문가의 busy 한 바퀴(정차 포함)는 330~480초다


CODE_PATHS = ("env", "algorithms", "utils", "train.py", "collect_pomdp_data.py", "maps/hapjeong")   # 주행 결과를 바꾸는 코드와
# 지도. collect_pomdp_data.py가 train.py의 HPARAMS·REWARD를, algorithms/ppo.py가 utils/의 신경망을 읽는다.


def run_key(args):
    """이미 있는 주행 기록을 다시 써도 되는지 가르는 값: 정책, 모델 파일 해시, 에피소드 수, 주행 코드와 지도의 git 트리.
    커밋하지 않은 변경(추적하지 않는 새 파일 포함)이 있으면 그 내용의 해시를 '+dirty:' 뒤에 붙여, 고칠 때마다 다시 돌린다."""
    def tree(path):
        def git(*command):
            return subprocess.run(["git", *command], cwd=REPO, capture_output=True).stdout
        rev = git("rev-parse", f"HEAD:{path}").decode().strip()
        diff = git("diff", "HEAD", "--", path)
        untracked = sorted(git("ls-files", "--others", "--exclude-standard", "--", path).decode().split())
        if not diff and not untracked:
            return rev
        digest = hashlib.sha256(diff)
        for name in untracked:
            digest.update(name.encode())
            with open(os.path.join(REPO, name), "rb") as f:
                digest.update(f.read())
        return f"{rev}+dirty:{digest.hexdigest()[:12]}"
    key = {"policy": args.policy, "episodes": args.episodes, "max_steps": MAX_STEPS,
           "code": {path: tree(path) for path in CODE_PATHS}}
    if args.model:
        with open(args.model, "rb") as f:
            key["model_sha256"] = hashlib.sha256(f.read()).hexdigest()
    return key


def run(args, scenario):
    seed = SPLITS[args.split] + SCENARIO_OFFSET[scenario] if args.seed is None else args.seed
    name = f"{args.label}_{args.split if args.seed is None else f's{args.seed}'}_{scenario}"
    out_dir = os.path.join(REPO, "data", "research", "eval")
    jsonl = os.path.join(out_dir, name + ".jsonl.gz")
    key_path = os.path.join(out_dir, name + ".key.json")
    key = run_key(args)
    if os.path.exists(jsonl):
        old = None
        if os.path.exists(key_path):
            with open(key_path, encoding="utf-8") as f:
                old = json.load(f)
        if old != key:
            print(f"  {name}: 기록이 지금 코드·모델·에피소드 수와 달라 다시 돌린다")
            for suffix in (".jsonl.gz", ".npz", ".metadata.json"):
                if os.path.exists(os.path.join(out_dir, name + suffix)):
                    os.remove(os.path.join(out_dir, name + suffix))
            shutil.rmtree(os.path.join(out_dir, name + ".assets"), ignore_errors=True)
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
        with open(key_path, "w", encoding="utf-8") as f:
            json.dump(key, f, ensure_ascii=False, indent=1)
    per_episode = [episode_metrics(records) for records in load_episodes(jsonl)]
    return {"scenario": scenario, "seed_start": seed, "jsonl": os.path.relpath(jsonl, REPO),
            "summary": summarize(per_episode),
            "episodes": [{k: v for k, v in e.items() if k != "stop_events"} for e in per_episode]}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", required=True, choices=["route-follow", "expert", "expert-v5", "expert-v6", "expert-v7", "bc", "ppo"])
    parser.add_argument("--model", default=None, help="bc 또는 ppo(Residual 포함) model.pt")
    parser.add_argument("--split", choices=tuple(SPLITS), default="dev")
    parser.add_argument("--scenarios", nargs="+", default=["quiet", "baseline", "busy"], choices=tuple(SCENARIOS))
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--label", default=None, help="결과 이름. 기본은 정책 이름")
    parser.add_argument("--seed", type=int, default=None,
                        help="모든 교통량에 같은 첫 seed를 쓴다(교통량끼리 비교). 기본은 split과 교통량마다 다른 seed")
    args = parser.parse_args()
    if args.seed is None and set(args.scenarios) - set(SCENARIO_OFFSET):
        parser.error("실험용 교통량은 --seed가 필요합니다.")
    if args.policy in ("bc", "ppo") and not args.model:
        parser.error("--policy bc/ppo는 --model이 필요합니다.")
    args.label = args.label or args.policy
    results = {"label": args.label, "policy": args.policy, "model": args.model, "split": args.split,
               "episodes_per_scenario": args.episodes, "max_steps": MAX_STEPS,
               "scenarios": [run(args, scenario) for scenario in args.scenarios]}
    out = os.path.join(HERE, "..", "..", "results", "metrics", f"{args.label}_{args.split if args.seed is None else f's{args.seed}'}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=1)
    for block in results["scenarios"]:
        s = block["summary"]
        print(f"{args.label:>16} {block['scenario']:<8} n={s['episodes']} {s['terminations']} "
              f"collision={s['vehicle_collision']['mean'] * 100:.0f}% pedestrian={s['pedestrian_collision']['mean'] * 100:.0f}% "
              f"red={s['red_light_violation']['mean'] * 100:.0f}% teleports={s['teleports']['mean'] * s['episodes']:.0f} "
              f"lap={s['lap_time_s']['mean'] or 0:.0f}s hard={s['hard_brakes']['mean']:.2f} "
              f"interv={s['speed_intervened_rate']['mean'] * 100:.1f}% mismatch={s['label_mismatch_rate']['mean'] * 100:.2f}% "
              f"jerk={s['jerk_violation_rate']['mean'] * 100:.1f}% lat={s['lateral_accel_peak']['max']:.2f}m/s2 "
              f"stuck={s['stuck_events']['mean']:.2f} "
              f"served={s['bus_stops_served']['mean']:.2f} missed={(s['bus_stops_missed']['mean'] or 0):.2f}")
    print(out)


if __name__ == "__main__":
    main()
