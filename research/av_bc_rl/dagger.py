"""DAgger: BC 정책이 직접 달린 관측에 전문가 행동을 다시 붙여 데이터에 더하고 BC를 다시 학습한다.

전문가(env/route_policy.ExpertPolicy)는 관측만으로 행동을 정하므로, BC가 간 상태의 관측을 그대로 넣으면
"그 상태에서 전문가가 했을 행동"이 나온다. BC가 전문가 궤적에서 조금씩 벗어나며 쌓이는 오차를 줄인다.

    python research/av_bc_rl/dagger.py --data data/research/bc_train.npz --model results/research/bc_s0/model.pt \
        --iterations 2 --episodes 30 --out results/research/dagger_s0 --seed 0

같은 --out으로 다시 돌리면 끝난 반복(합본과 모델이 다 있는 것)은 건너뛰고 이어서 돈다.

--overlap을 주면 수집과 학습을 겹쳐 돌린다. 반복 k를 학습하는 동안 반복 k+1의 롤아웃을 그때 가진 모델(반복 k-1의 것)로
미리 모은다. 롤아웃이 한 반복 뒤진 모델의 것이 되는 대신 CPU가 쉬지 않아, 반복 하나가 수집+학습 시간이 아니라 둘 중 긴
쪽만큼 걸린다.

모르는 인자는 train_bc.py로 넘긴다(예: --loss bins --decode local). 처음 BC를 학습한 인자와 같게 준다.

정답을 다른 판의 전문가로 다시 붙이려면(관측은 그대로다) 먼저 한 번 실행한다. metadata의 policy도 그 판으로 바뀐다.

    python research/av_bc_rl/dagger.py --data data/research/v4/demo_clean.npz --relabel-to data/research/v6/demo_v6.npz \
        --expert-version 6
"""
import argparse
import shutil
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(HERE))
from env import sumo_env  # noqa: E402
from env.route_policy import EXPERT_VERSIONS  # noqa: E402

PY = sys.executable
SCENARIO_SEEDS = {"quiet": 2100, "baseline": 2400, "busy": 2700, "heavy": 3500}   # train 집합 안, 전문가 수집 seed와 겹치지 않는다
# 실험용 교통량(exp/snail-traffic). SCENARIO_SEEDS와 따로 두어 기존 실행의 episode 번호 규칙(칸 4개)이 그대로다.
EXTRA_SEEDS = {"x2": 4500}


def expert_from_metadata(meta):
    """데이터에 정답을 붙인 판의 전문가. metadata의 policy가 expert-vN이면 그 판, 아니면 v4다."""
    policy = str(meta.get("policy"))
    expert = EXPERT_VERSIONS[int(policy.removeprefix("expert-v")) if policy.startswith("expert-v") else 4]
    return expert(meta["road"]["speed_limit"], meta["observation"]["visibility"], meta["ego"]["accel"],
                  meta["ego"]["decel"], sumo_env.TL_RANGE, sumo_env.STOP_RANGE, meta["observation"]["connectivity_range"],
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
    parser.add_argument("--model", help="처음 BC model.pt")
    parser.add_argument("--relabel-to", help="롤아웃 없이 --data의 정답만 --expert-version 전문가로 다시 붙여 이 NPZ로 쓴다")
    parser.add_argument("--expert-version", type=int, choices=sorted(EXPERT_VERSIONS), help="--relabel-to에 쓸 전문가 판")
    parser.add_argument("--iterations", type=int, default=2)
    parser.add_argument("--scenarios", nargs="+", default=["quiet", "baseline", "busy"], choices=(*SCENARIO_SEEDS, *EXTRA_SEEDS),
                        help="롤아웃을 모을 교통량. 나중 반복에 heavy를 더해 점점 붐비게 할 수 있다")
    parser.add_argument("--episodes", type=int, default=30, help="반복마다 교통량별 롤아웃 수")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--workers", type=int, default=5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--augment-signals", action="store_true",
                        help="반복마다 합본에 신호만 황색·적색으로 바꾼 관측을 더해 학습한다(augment_signals.py). "
                             "합본 자체에는 넣지 않는다. 처음 BC도 그렇게 늘린 데이터로 학습한 것이어야 한다")
    parser.add_argument("--overlap", action="store_true",
                        help="학습하는 동안 다음 반복의 롤아웃을 그때 가진 모델로 미리 모은다")
    parser.add_argument("--episode-base", type=int, default=0,
                        help="새 롤아웃 episode 번호에 더할 값. 다른 실행의 합본(--data)에서 이어 갈 때 번호가 겹치지 않게 그 최대값보다 크게 준다")
    parser.add_argument("--out")
    args, train_args = parser.parse_known_args()

    data_path = Path(args.data)
    meta = json.loads(data_path.with_name(data_path.name.removesuffix(".npz") + ".metadata.json").read_text(encoding="utf-8"))
    if args.relabel_to:
        if args.expert_version is None or train_args:
            parser.error("--relabel-to에는 --expert-version이 필요하고 다른 인자는 받지 않습니다.")
        target = Path(args.relabel_to)
        target.parent.mkdir(parents=True, exist_ok=True)
        meta = {**meta, "policy": f"expert-v{args.expert_version}", "relabelled_from": str(data_path)}
        np.savez_compressed(target, **relabel(data_path, expert_from_metadata(meta)))
        target.with_name(target.name.removesuffix(".npz") + ".metadata.json").write_text(
            json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
        print(target)
        return
    if not args.model or not args.out:
        parser.error("--model과 --out이 필요합니다.")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    expert = expert_from_metadata(meta)
    model = Path(args.model)
    def collect(iteration, model):
        """반복 하나의 롤아웃을 교통량마다 모은다. 이미 있는 것은 건너뛴다."""
        paths = []
        for scenario in args.scenarios:
            base = {**SCENARIO_SEEDS, **EXTRA_SEEDS}[scenario]
            name = f"dagger_s{args.seed}_it{iteration}_{scenario}"
            # 반복마다 rollout seed가 겹치지 않게 episodes만큼 띄운다. 학습 seed(args.seed)는 100씩 띄운다.
            seed = base + 100 * args.seed + args.episodes * (iteration - 1)
            if not (out / f"{name}.npz").exists():
                subprocess.run([PY, str(REPO / "collect_pomdp_data.py"), "--map", "hapjeong", "--scenario", scenario,
                                "--seed", str(seed), "--policy", "bc", "--model", str(model), "--episodes",
                                str(args.episodes), "--workers", str(args.workers), "--max-steps", "1400",
                                "--out-dir", str(out), "--name", name], cwd=REPO, check=True, stdout=subprocess.DEVNULL)
            paths.append(out / f"{name}.npz")
        return paths

    ahead = None   # --overlap: 다음 반복의 롤아웃을 미리 모으는 스레드와 그 실패
    for iteration in range(1, args.iterations + 1):
        done_data, done_model = out / f"dagger_s{args.seed}_it{iteration}.npz", out / f"bc_it{iteration}" / "model.pt"
        if done_data.exists() and done_model.exists():   # 끊긴 실행을 이어 돌릴 때 끝난 반복은 건너뛴다
            data_path, model = done_data, done_model
            continue
        shutil.rmtree(out / f"bc_it{iteration}", ignore_errors=True)   # 학습 도중 끊긴 폴더
        if ahead is not None:
            thread, failure = ahead
            thread.join()
            if failure:
                raise failure[0]
        rollouts = collect(iteration, model)
        merged = {}
        with np.load(data_path, allow_pickle=False) as data:
            base_arrays = {key: data[key] for key in data.files}
        # 반복 2부터는 base가 앞 반복의 합본이라 이미 100000·200000·300000번대 에피소드가 있다. 새 rollout 번호가 겹치지 않게
        # 반복마다 교통량 수만큼 칸을 더 띄운다.
        parts = [base_arrays] + [relabel(path, expert) for path in rollouts]
        for index, part in enumerate(parts[1:], start=1):
            slots = max(len(SCENARIO_SEEDS), len(args.scenarios))
            part["episode"] = part["episode"].astype(np.int64) + 100000 * (slots * (iteration - 1) + index) + args.episode_base
        for key in base_arrays:
            merged[key] = np.concatenate([part[key] for part in parts if key in part])
        data_path = out / f"dagger_s{args.seed}_it{iteration}.npz"
        np.savez_compressed(data_path, **merged)
        data_path.with_name(data_path.name.removesuffix(".npz") + ".metadata.json").write_text(
            json.dumps({**meta, "dagger_iteration": iteration, "dagger_rollouts": [str(p) for p in rollouts]},
                       ensure_ascii=False, indent=1), encoding="utf-8")
        train_path = data_path
        if args.augment_signals:
            from augment_signals import KEYS, augment
            train_path = out / f"dagger_s{args.seed}_it{iteration}_train.npz"
            np.savez_compressed(train_path, **augment({key: merged[key] for key in KEYS}, expert, args.seed))
            shutil.copyfile(data_path.with_name(data_path.name.removesuffix(".npz") + ".metadata.json"),
                            train_path.with_name(train_path.name.removesuffix(".npz") + ".metadata.json"))
        run_dir = out / f"bc_it{iteration}"
        ahead = None
        if args.overlap and iteration < args.iterations:
            failure = []

            def collect_ahead(next_iteration=iteration + 1, current=model, failure=failure):
                try:
                    collect(next_iteration, current)
                except Exception as error:   # 스레드 안의 실패를 다음 반복 머리에서 다시 던진다
                    failure.append(error)
            thread = threading.Thread(target=collect_ahead)
            thread.start()
            ahead = (thread, failure)
        subprocess.run([PY, str(REPO / "train_bc.py"), "--data", str(train_path), "--epochs", str(args.epochs),
                        "--seed", str(args.seed), "--device", "cpu", "--out-dir", str(run_dir), *train_args],
                       cwd=REPO, check=True, stdout=subprocess.DEVNULL)
        model = run_dir / "model.pt"
        print(f"iteration {iteration}: {len(merged['observation'])} samples -> {model}")
    print(model)


if __name__ == "__main__":
    main()
