"""녹색 신호에 다가가는 관측마다 "지금 황색(또는 적색)으로 바뀌었다면"을 한 줄씩 더해 학습 데이터를 늘린다.

버스가 정지선 15~25m 앞에 있을 때 황색이 켜지는 일은 한 바퀴에 한 번도 드물다. 그런데 그때 0.5초 안에 세게 줄여야 선다.
시범과 DAgger 롤아웃 136만 줄 가운데 황색에 세게 줄이는 정답은 0.6%뿐이라, 신경망이 그 자리에서 덜 줄이다 적색에 넘었다.
신호 색은 관측의 다른 값과 무관하게 바뀌므로, 녹색 관측에서 신호 값만 바꾼 것도 실제로 나올 수 있는 관측이다. 정답은
규칙 운전사가 그 관측에 내는 행동이다(관측만 보는 함수라 그대로 물을 수 있다). 주행할 때 쓰는 규칙은 없다.

더한 줄은 원래 줄과 같은 에피소드 번호를 쓴다. train_bc.py는 에피소드 단위로 학습과 검증을 나누므로 둘이 섞이지 않는다.

    python research/av_bc_rl/augment_signals.py --data data/research/v6/demo_v6.npz --out data/research/v6/demo_v6_aug.npz
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dagger import expert_from_metadata  # noqa: E402
from env.data_schema import OBSERVATION_FEATURES  # noqa: E402

DISTANCE, RED, YELLOW, YIELD = (OBSERVATION_FEATURES.index(name) for name in ("tl_distance", "tl_red", "tl_yellow", "tl_yield"))
KEYS = ("observation", "action_raw", "lane_change", "episode")


def signal_flips(observation, expert, rng, yellow_share=2 / 3):
    """신호가 보이고(거리 < 1) 적색도 황색도 아닌 줄마다 신호만 황색 또는 적색으로 바꾼 관측과 그 정답.
    돌려주는 것은 (원래 줄 번호, 바꾼 관측, 전문가 행동)이다."""
    rows = np.flatnonzero((observation[:, DISTANCE] < 1.0) & (observation[:, RED] < 0.5) & (observation[:, YELLOW] < 0.5))
    flipped = observation[rows].copy()
    to_yellow = rng.random(len(rows)) < yellow_share
    flipped[:, YELLOW], flipped[:, RED], flipped[:, YIELD] = to_yellow, ~to_yellow, 0.0
    actions = np.stack([expert(obs) for obs in flipped]).astype(np.float32) if len(rows) else np.zeros((0, 2), np.float32)
    return rows, flipped, actions


def augment(arrays, expert, seed=0):
    rows, flipped, actions = signal_flips(arrays["observation"], expert, np.random.default_rng(seed))
    lane = np.where(actions[:, 1] >= 0.5, 1, np.where(actions[:, 1] <= -0.5, -1, 0)).astype(np.int8)
    extra = {"observation": flipped, "action_raw": actions, "lane_change": lane, "episode": arrays["episode"][rows]}
    return {key: np.concatenate([arrays[key], extra[key]]) for key in KEYS}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", required=True, help="정답이 붙은 NPZ. 옆의 metadata.json에 적힌 판의 전문가로 정답을 낸다")
    parser.add_argument("--out", required=True)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    source, target = Path(args.data), Path(args.out)
    meta = json.loads(source.with_name(source.name.removesuffix(".npz") + ".metadata.json").read_text(encoding="utf-8"))
    with np.load(source, allow_pickle=False) as data:
        arrays = {key: data[key] for key in KEYS}
    merged = augment(arrays, expert_from_metadata(meta), args.seed)
    target.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(target, **merged)
    target.with_name(target.name.removesuffix(".npz") + ".metadata.json").write_text(
        json.dumps({**meta, "signal_flips_from": str(source)}, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"{target}: {len(arrays['observation'])} + {len(merged['observation']) - len(arrays['observation'])}줄")


if __name__ == "__main__":
    main()
