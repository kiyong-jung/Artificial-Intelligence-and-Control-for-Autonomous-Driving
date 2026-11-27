"""collect_pomdp_data.py로 수집한 NPZ/JSONL 데이터로 Behavior Cloning 학습.

사용 예시:
    python train_bc.py --data data/pomdp_example.npz
    tensorboard --logdir results
    브라우저에서 http://localhost:6006 접속

학습 대상:
    observation -> action_raw

BCPolicy는 algorithms/bc.py에 정의된
state_dim -> 256 -> 256 -> 출력 MLP를 사용한다.
--loss mse(기본)는 전체 action vector를 MSE loss로 학습한다.
--loss bins는 가감속을 칸 중 고르기, 차선변경을 셋 중 고르기로 보고 cross-entropy로 학습한다.

TensorBoard 이벤트 파일은 model.pt와 같은 run 폴더(results/bc_월일_시분/)에
바로 저장되므로, train.py의 run_* 결과와 나란히 비교할 수 있다.
"""

import argparse
import csv
import gzip
import hashlib
import json
from contextlib import closing
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

from algorithms.bc import BCPolicy
from env.experiment import write_run_config


BASE = Path(__file__).resolve().parent


def load_data(path, action_key='action_raw', valid_key=None):
    """observation/action_raw/episode를 불러온다."""
    path = Path(path)
    if action_key != 'action_raw' and not valid_key:
        raise ValueError('Explicit labels require an explicit validity mask')

    if path.suffix == ".npz":
        with np.load(path, allow_pickle=False) as data:
            required = ("observation", action_key, "episode") + ((valid_key,) if valid_key else ())
            missing = [key for key in required if key not in data]

            if missing:
                raise ValueError(
                    f"필수 데이터가 없습니다: {missing}. "
                    f"필요한 key = {required}"
                )

            obs = data["observation"]
            actions = data[action_key]
            episodes = data["episode"]
            if valid_key:
                mask = data[valid_key]
                if mask.dtype != np.bool_ or mask.shape != (len(obs),):
                    raise ValueError('Validity mask must be boolean shape(N,)')
                obs, actions, episodes = obs[mask], actions[mask], episodes[mask]

    elif path.suffix == ".jsonl" or path.name.endswith(".jsonl.gz"):
        if action_key != 'action_raw' or valid_key:
            raise ValueError('Explicit labels/masks currently require NPZ')
        obs, actions, episodes = [], [], []

        opener = gzip.open if path.name.endswith(".gz") else open
        with opener(path, "rt", encoding="utf-8") as stream:
            for line_no, line in enumerate(stream, 1):
                if not line.strip():
                    continue

                try:
                    row = json.loads(line)
                    obs.append(row["observation"])
                    actions.append(row["action_raw"])
                    episodes.append(row["episode"])
                except (ValueError, KeyError, TypeError) as exc:
                    raise ValueError(
                        f"{path}:{line_no}: 잘못된 transition"
                    ) from exc

    else:
        raise ValueError("데이터는 .npz, .jsonl 또는 .jsonl.gz이어야 합니다.")

    obs = np.asarray(obs, dtype=np.float32)
    actions = np.asarray(actions, dtype=np.float32)
    episodes = np.asarray(episodes)

    n = len(obs)

    if n == 0:
        raise ValueError("데이터가 비어 있습니다.")

    if obs.ndim != 2 or obs.shape[1] == 0:
        raise ValueError(
            f"observation shape은 (N, state_dim)이어야 합니다. 현재 shape={obs.shape}"
        )

    if actions.ndim != 2 or actions.shape[1] != 2:
        raise ValueError(
            f"action_raw shape은 (N, 2)이어야 합니다. 현재 shape={actions.shape}"
        )

    if len(actions) != n or episodes.shape != (n,):
        raise ValueError(
            "observation, action_raw, episode의 sample 수가 서로 다릅니다."
        )

    if not np.isfinite(obs).all():
        raise ValueError("observation에 NaN 또는 Inf가 있습니다.")

    if not np.isfinite(actions).all():
        raise ValueError("action_raw에 NaN 또는 Inf가 있습니다.")

    if not np.issubdtype(episodes.dtype, np.number) or not np.isfinite(episodes).all():
        raise ValueError("episode은 유한한 숫자 ID여야 합니다.")
    if not np.equal(episodes, np.floor(episodes)).all():
        raise ValueError("episode ID는 정수여야 합니다.")
    if (np.abs(actions) > 1).any():
        raise ValueError("action_raw는 환경에 전달된 [-1, 1] 범위의 행동이어야 합니다.")

    return obs, actions, episodes


def split_episodes(episodes, val_fraction, seed):
    """같은 episode의 transition이 train/validation에 동시에 들어가지 않도록 분리함."""
    unique_episodes = np.unique(episodes)

    if len(unique_episodes) < 2:
        raise ValueError(
            "episode 단위 train/validation 분리를 위해 최소 2개 episode가 필요합니다."
        )

    rng = np.random.default_rng(seed)
    shuffled = rng.permutation(unique_episodes)

    n_val = max(1, round(len(unique_episodes) * val_fraction))
    n_val = min(n_val, len(unique_episodes) - 1)

    val_episodes = shuffled[:n_val]
    val_mask = np.isin(episodes, val_episodes)

    train_idx = np.flatnonzero(~val_mask)
    val_idx = np.flatnonzero(val_mask)

    return train_idx, val_idx


def standstill_middles(obs, episodes, keep):
    """에피소드 안에서 이어서 서 있는 구간마다 처음 keep개와 마지막 keep개 판단을 뺀 가운데를 True로 표시한다.
    관측 첫 값이 속도(제한속도로 나눈 값)다. 신호와 정류장에서 서 있는 판단은 서로 거의 같아 수만 많다."""
    standing = obs[:, 0] < 1e-3
    starts = np.r_[True, (standing[1:] != standing[:-1]) | (episodes[1:] != episodes[:-1])]
    run = np.cumsum(starts) - 1
    first = np.flatnonzero(starts)
    position = np.arange(len(obs)) - first[run]
    remaining = np.diff(np.r_[first, len(obs)])[run] - 1 - position
    return standing & (position >= keep) & (remaining >= keep)


def make_loader(obs, actions, indices, batch_size, shuffle):
    """Numpy array를 PyTorch DataLoader로 변환함."""
    dataset = TensorDataset(
        torch.from_numpy(obs[indices]),
        torch.from_numpy(actions[indices]),
    )

    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        drop_last=False,
    )


def run_epoch(model, loader, device, optimizer=None):
    """한 epoch 학습 또는 validation을 수행함."""
    is_train = optimizer is not None
    model.train(is_train)

    total_loss = 0.0
    total_samples = 0

    with torch.set_grad_enabled(is_train):
        for state, target_action in loader:
            state = state.to(device)
            target_action = target_action.to(device)

            loss = model.loss(state, target_action)

            if not torch.isfinite(loss):
                raise ValueError(
                    "학습 loss가 NaN 또는 Inf입니다. "
                    "데이터 값 또는 learning rate를 확인하세요."
                )

            if is_train:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            batch_size = state.shape[0]
            total_loss += loss.item() * batch_size
            total_samples += batch_size

    return total_loss / total_samples


def save_model(model, path):
    """BCPolicy.load()와 호환되도록 구조 정보와 정규화 통계도 저장한다."""
    model.save(path)


def file_sha256(path):
    """데이터셋은 메모리보다 클 수 있으므로 1MB씩 읽어 해시한다."""
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        "--data",
        required=True,
        type=Path,
        help="collect_pomdp_data.py로 생성한 .npz 또는 .jsonl 파일",
    )

    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--hidden", type=int, default=256, help="은닉층의 폭")
    parser.add_argument("--depth", type=int, default=2, help="은닉층 수")
    parser.add_argument("--blocks", type=int, default=0, help="1 이상이면 가운데가 넓은 잔차 블록(FFN)을 이만큼 쌓는다")
    parser.add_argument("--expand", type=int, default=4, help="FFN 블록 가운데 폭의 배율")
    parser.add_argument("--loss", choices=["mse", "bins"], default="mse",
                        help="mse: 행동 값 회귀(수업 원본). bins: 가감속 칸과 차선변경 셋 중 고르기")
    parser.add_argument("--accel-bins", type=int, default=11, help="--loss bins의 가감속 칸 수([-1, 1]을 같은 간격으로)")
    parser.add_argument("--decode", choices=["mean", "mode", "local"], default="mean",
                        help="--loss bins에서 칸 점수를 가감속 값으로 바꾸는 방법. 모델 파일에 적힌다")

    parser.add_argument("--thin-standstill", type=int, default=None, metavar="STEPS",
                        help="학습 데이터에서 이어서 서 있는 구간마다 처음과 마지막 STEPS개 판단만 남긴다. 검증 데이터는 그대로다")

    parser.add_argument("--blind", nargs="+", default=[], metavar="FEATURE",
                        help="신경망이 보지 않을 관측 항목 이름(env/data_schema.OBSERVATION_FEATURES). 그 항목의 정규화 "
                             "표준편차를 무한대로 두어 입력이 늘 0이 되게 한다. 관측 차원은 그대로다. "
                             "묶음 이름도 받는다: compact는 좌2·우2 차로 전부(12개)와 내 차로 뒤차 간격·상대속도, tl_yield(15개)")

    parser.add_argument(
        "--device",
        choices=["auto", "cpu", "cuda", "mps"],
        default="auto",
    )

    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="학습 결과 저장 폴더",
    )

    args = parser.parse_args()

    if args.epochs < 1:
        parser.error("--epochs는 1 이상이어야 합니다.")

    if args.batch_size < 1:
        parser.error("--batch-size는 1 이상이어야 합니다.")

    if not np.isfinite(args.lr) or args.lr <= 0:
        parser.error("--lr은 양수여야 합니다.")

    if not 0 < args.val_fraction < 1:
        parser.error("--val-fraction은 0과 1 사이여야 합니다.")

    # --help와 데이터 유틸 함수는 TensorBoard 없이도 쓸 수 있도록 여기서 import한다.
    try:
        from torch.utils.tensorboard import SummaryWriter
    except ImportError as exc:
        parser.error(f"TensorBoard를 불러올 수 없습니다: {exc}. "
                     "현재 Python 환경에서 python -m pip install tensorboard를 실행하세요.")

    # Reproducibility
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)

    # Device 선택
    if args.device == "auto":
        if torch.cuda.is_available():
            device = "cuda"
        elif getattr(torch.backends, "mps", None) is not None and torch.backends.mps.is_available():
            device = "mps"
        else:
            device = "cpu"
    else:
        device = args.device

    # Dataset
    obs, actions, episodes = load_data(args.data)

    state_dim = obs.shape[1]
    action_dim = actions.shape[1]

    train_idx, val_idx = split_episodes(
        episodes,
        args.val_fraction,
        args.seed,
    )
    if args.thin_standstill is not None:
        train_idx = train_idx[~standstill_middles(obs, episodes, args.thin_standstill)[train_idx]]

    train_loader = make_loader(
        obs,
        actions,
        train_idx,
        args.batch_size,
        shuffle=True,
    )

    val_loader = make_loader(
        obs,
        actions,
        val_idx,
        args.batch_size,
        shuffle=False,
    )

    # Model
    model = BCPolicy(
        state_dim=state_dim,
        action_dim=action_dim,
        head=args.loss,
        accel_bins=args.accel_bins,
        decode=args.decode,
        hidden=args.hidden,
        depth=args.depth,
        blocks=args.blocks,
        expand=args.expand,
    ).to(device)

    # 검증 데이터가 통계에 섞이지 않도록 학습 관측만으로 정규화한다.
    # 상수 항목은 표준편차를 1로 두어 검증/추론 입력의 폭증을 방지한다.
    obs_mean = obs[train_idx].mean(axis=0)
    obs_std = obs[train_idx].std(axis=0)
    obs_std = np.where(obs_std < 1e-6, 1.0, obs_std)
    if args.blind:
        from env.data_schema import BLIND_BUNDLES, OBSERVATION_FEATURES
        args.blind = [name for item in args.blind for name in BLIND_BUNDLES.get(item, [item])]
        unknown = sorted(set(args.blind) - set(OBSERVATION_FEATURES))
        if unknown or state_dim != len(OBSERVATION_FEATURES):
            parser.error(f"--blind: 지금 관측에 없는 항목입니다: {unknown or state_dim}")
        obs_std[[OBSERVATION_FEATURES.index(name) for name in args.blind]] = np.inf
    model.obs_mean.copy_(torch.as_tensor(obs_mean, device=device))
    model.obs_std.copy_(torch.as_tensor(obs_std, device=device))

    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=args.lr,
    )

    # 결과 폴더 — 짧고 읽기 쉬운 이름: bc_월일_시분 (예: bc_0922_1145)
    # 같은 분(minute)에 여러 번 실행하면 _2, _3 … 접미사가 자동으로 붙는다.
    if args.out_dir is None:
        stem = datetime.now().strftime("bc_%m%d_%H%M")
        out_dir = BASE / "results" / stem
        k = 2
        while out_dir.exists():
            out_dir = BASE / "results" / f"{stem}_{k}"
            k += 1
    else:
        out_dir = args.out_dir

    out_dir.mkdir(parents=True, exist_ok=False)

    # 설정 저장
    config = {
        "data": str(args.data),
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "lr": args.lr,
        "val_fraction": args.val_fraction,
        "seed": args.seed,
        "loss": args.loss,
        "accel_bins": args.accel_bins,
        "decode": args.decode,
        "thin_standstill": args.thin_standstill,
        "blind": args.blind,
        "device": device,
        "state_dim": state_dim,
        "action_dim": action_dim,
        "train_samples": len(train_idx),
        "val_samples": len(val_idx),
        "train_episodes": np.unique(episodes[train_idx]).tolist(),
        "val_episodes": np.unique(episodes[val_idx]).tolist(),
    }

    (out_dir / "config.json").write_text(
        json.dumps(config, indent=2),
        encoding="utf-8",
    )

    # train.py와 같은 run_config.json. 도로·시뮬레이션 설정은 데이터셋을 모은 값이다.
    stem = args.data.name.removesuffix(".gz").removesuffix(".jsonl").removesuffix(".npz")
    dataset_metadata = args.data.with_name(stem + ".metadata.json")
    if dataset_metadata.exists():
        collected = json.loads(dataset_metadata.read_text(encoding="utf-8"))
        write_run_config(
            out_dir, hparams=config, reward=collected["reward"], kind="bc_training",
            settings={key: collected[key] for key in
                      ("road", "ego", "traffic", "simulation", "observation", "action")},
            dataset={"path": str(args.data.resolve()), "sha256": file_sha256(args.data)})
    else:
        print(f"※ {dataset_metadata}이 없어 데이터를 모은 도로를 알 수 없습니다. "
              "run_config.json을 쓰지 않으므로 test.py는 이 모델을 평가하지 않습니다.")

    print("=" * 70)
    print("Behavior Cloning Training")
    print("=" * 70)
    print(f"data          : {args.data}")
    print(f"device        : {device}")
    print(f"state_dim     : {state_dim}")
    print(f"action_dim    : {action_dim}")
    print(f"train samples : {len(train_idx)}")
    print(f"val samples   : {len(val_idx)}")
    print(f"output        : {out_dir}")
    print(f"tensorboard   : tensorboard --logdir \"{out_dir.parent}\"  → http://localhost:6006")
    print("=" * 70)

    best_val_loss = float("inf")

    log_path = out_dir / "training_log.csv"

    # TensorBoard 이벤트는 model.pt와 같은 폴더(out_dir)에 바로 쓴다 (train.py와 동일한 구조).
    # closing(): 오류나 Ctrl+C로 종료되어도 대기 중인 이벤트를 파일에 저장한다.
    with closing(SummaryWriter(log_dir=str(out_dir))) as tb_writer, log_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        tb_writer.add_text("config", "```json\n" + json.dumps(config, indent=2) + "\n```", 0)

        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "epoch",
                "train_loss",
                "val_loss",
            ],
        )

        writer.writeheader()

        for epoch in range(1, args.epochs + 1):
            train_loss = run_epoch(
                model,
                train_loader,
                device,
                optimizer=optimizer,
            )

            val_loss = run_epoch(
                model,
                val_loader,
                device,
                optimizer=None,
            )

            writer.writerow(
                {
                    "epoch": epoch,
                    "train_loss": train_loss,
                    "val_loss": val_loss,
                }
            )
            stream.flush()

            # validation loss가 가장 낮은 모델 저장
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                save_model(
                    model,
                    out_dir / "model.pt",
                )

            # x축은 epoch. CSV와 동일한 값을 기록한다.
            tb_writer.add_scalar("Loss/train", train_loss, epoch)
            tb_writer.add_scalar("Loss/validation", val_loss, epoch)
            tb_writer.add_scalar("Loss/best_validation", best_val_loss, epoch)
            tb_writer.add_scalar("Optimization/learning_rate", optimizer.param_groups[0]["lr"], epoch)
            # 학습 중에도 브라우저에서 완료된 epoch 결과를 바로 볼 수 있게 한다.
            tb_writer.flush()

            print(
                f"Epoch {epoch:3d}/{args.epochs} | "
                f"train_loss={train_loss:.6f} | "
                f"val_loss={val_loss:.6f}"
            )

    # 마지막 epoch 모델 저장
    save_model(
        model,
        out_dir / "last_model.pt",
    )

    print("=" * 70)
    print("Training finished")
    print(f"Best validation loss : {best_val_loss:.6f}")
    print(f"Best model           : {out_dir / 'model.pt'}")
    print(f"Last model           : {out_dir / 'last_model.pt'}")
    print(f"Training log         : {log_path}")
    print(f"Evaluation           : python test.py {out_dir / 'model.pt'} --algorithm bc --nogui")
    print(f"TensorBoard          : tensorboard --logdir \"{out_dir.parent}\"  → http://localhost:6006")
    print("=" * 70)


if __name__ == "__main__":
    main()
