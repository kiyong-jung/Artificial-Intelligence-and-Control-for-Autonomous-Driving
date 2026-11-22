"""학습 중인 run 폴더들의 TensorBoard 기록에서 마지막 학습 지표를 읽는다(학습이 끝나야 CSV가 생긴다).

    python research/av_bc_rl/training_progress.py results/research/run_*
"""
import glob
import os
import sys

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

KEYS = ("rollout/ep_rew_mean", "train/ep_rew_mean", "rollout/ep_len_mean", "train/std", "train/approx_kl")


def main(paths):
    for path in sorted(paths):
        events = glob.glob(os.path.join(path, "tb", "events.*")) + glob.glob(os.path.join(path, "events.*"))
        if not events:
            continue
        accumulator = EventAccumulator(os.path.dirname(events[0]))
        accumulator.Reload()
        tags = accumulator.Tags().get("scalars", [])
        shown = []
        for tag in tags:
            if any(key.split("/")[-1] in tag for key in KEYS):
                last = accumulator.Scalars(tag)[-1]
                shown.append(f"{tag}={last.value:.3f}@{last.step}")
        print(os.path.basename(path), " ".join(shown) or f"(tags: {tags[:6]})")


if __name__ == "__main__":
    main(sys.argv[1:])
