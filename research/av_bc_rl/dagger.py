"""DAgger: BC 정책이 직접 달린 관측에 전문가 행동을 다시 붙여 데이터에 더하고 BC를 다시 학습한다.

전문가(env/route_policy.ExpertPolicy)는 관측만으로 행동을 정하므로, BC가 간 상태의 관측을 그대로 넣으면
"그 상태에서 전문가가 했을 행동"이 나온다. BC가 전문가 궤적에서 조금씩 벗어나며 쌓이는 오차를 줄인다.

    python research/av_bc_rl/dagger.py --data data/research/bc_train.npz --model results/research/bc_s0/model.pt \
        --iterations 2 --episodes 30 --out results/research/dagger_s0 --seed 0
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))
from env import sumo_env  # noqa: E402
from env.route_policy import EXPERT_VERSIONS  # noqa: E402

PY = sys.executable
SCENARIO_SEEDS = {"quiet": 2100, "baseline": 2400, "busy": 2700}   # train 집합 안, 전문가 수집 seed와 겹치지 않는다


def expert_from_metadata(meta):
    """데이터를 모은 판의 전문가. metadata의 policy가 expert-v5면 v5, 아니면 v4다."""
    expert = EXPERT_VERSIONS[5 if meta.get("policy") == "expert-v5" else 4]
    return expert(meta["road"]["speed_limit"], meta["observation"]["visibility"], meta["ego"]["accel"],
                  meta["ego"]["decel"], sumo_env.TL_RANGE, sumo_env.STOP_RANGE, sumo_env.CONTINUATION_RANGE,
                  sumo_env.CROSSING_RANGE, sumo_env.LIMIT_RANGE)


def relabel(npz_path, expert):
    with np.load(npz_path, allow_pickle=False) as data:
        arrays = {key: data[key] for key in data.files}
    arrays["action_raw"] = np.stack([expert(obs) for obs in arrays["observation"]]).astype(np.float32)
    arrays["lane_change"] = np.where(arrays["action_raw"][:, 1] >= 0.5, 1, np.where(arrays["action_raw"][:, 1] <= -0.5, -1, 0)).astype(np.int8)
    return arrays


def main():
    import json
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", required=True, help="전문가 시연 NPZ(merge_datasets.py 출력)")
    parser.add_argument("--model", required=True, help="처음 BC model.pt")
    parser.add_argument("--iterations", type=int, default=2)
    parser.add_argument("--episodes", type=int, default=30, help="반복마다 교통량별 롤아웃 수")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--workers", type=int, default=5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    data_path = Path(args.data)
    meta = json.loads(data_path.with_name(data_path.name.removesuffix(".npz") + ".metadata.json").read_text(encoding="utf-8"))
    expert = expert_from_metadata(meta)
    model = Path(args.model)
    for iteration in range(1, args.iterations + 1):
        rollouts = []
        for scenario, base in SCENARIO_SEEDS.items():
            name = f"dagger_s{args.seed}_it{iteration}_{scenario}"
            # 반복마다 rollout seed가 겹치지 않게 episodes만큼 띄운다. 학습 seed(args.seed)는 100씩 띄운다.
            seed = base + 100 * args.seed + args.episodes * (iteration - 1)
            if not (out / f"{name}.npz").exists():
                subprocess.run([PY, str(REPO / "collect_pomdp_data.py"), "--map", "hapjeong", "--scenario", scenario,
                                "--seed", str(seed), "--policy", "bc", "--model", str(model), "--episodes",
                                str(args.episodes), "--workers", str(args.workers), "--max-steps", "1400",
                                "--out-dir", str(out), "--name", name], cwd=REPO, check=True, stdout=subprocess.DEVNULL)
            rollouts.append(out / f"{name}.npz")
        merged = {}
        with np.load(data_path, allow_pickle=False) as data:
            base_arrays = {key: data[key] for key in data.files}
        # 반복 2부터는 base가 앞 반복의 합본이라 이미 100000·200000·300000번대 에피소드가 있다. 새 rollout 번호가 겹치지 않게
        # 반복마다 교통량 수만큼 칸을 더 띄운다.
        parts = [base_arrays] + [relabel(path, expert) for path in rollouts]
        for index, part in enumerate(parts[1:], start=1):
            part["episode"] = part["episode"].astype(np.int64) + 100000 * (len(SCENARIO_SEEDS) * (iteration - 1) + index)
        for key in base_arrays:
            merged[key] = np.concatenate([part[key] for part in parts if key in part])
        data_path = out / f"dagger_s{args.seed}_it{iteration}.npz"
        np.savez_compressed(data_path, **merged)
        data_path.with_name(data_path.name.removesuffix(".npz") + ".metadata.json").write_text(
            json.dumps({**meta, "dagger_iteration": iteration, "dagger_rollouts": [str(p) for p in rollouts]},
                       ensure_ascii=False, indent=1), encoding="utf-8")
        run_dir = out / f"bc_it{iteration}"
        subprocess.run([PY, str(REPO / "train_bc.py"), "--data", str(data_path), "--epochs", str(args.epochs),
                        "--seed", str(args.seed), "--device", "cpu", "--out-dir", str(run_dir)],
                       cwd=REPO, check=True, stdout=subprocess.DEVNULL)
        model = run_dir / "model.pt"
        print(f"iteration {iteration}: {len(merged['observation'])} samples -> {model}")
    print(model)


if __name__ == "__main__":
    main()
