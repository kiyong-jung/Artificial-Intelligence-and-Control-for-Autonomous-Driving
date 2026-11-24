"""bins로 학습한 BC 모델의 가감속 고르는 방법(decode)만 바꾼 사본을 쓴다. 가중치는 그대로다.

같은 모델을 평균(mean), 가장 높은 칸(mode), 그 칸과 양옆 평균(local)으로 몰아 보고 견줄 때 쓴다.

    python research/av_bc_rl/set_decode.py results/v6_bins_s0/model.pt mode results/v6_bins_s0/model_mode.pt
"""
import sys

import torch

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from algorithms.bc import DECODES  # noqa: E402


def main():
    source, decode, target = sys.argv[1:4]
    checkpoint = torch.load(source, map_location="cpu", weights_only=True)
    if checkpoint.get("head") != "bins" or decode not in DECODES:
        raise SystemExit(f"bins 모델과 {DECODES} 중 하나가 필요합니다.")
    torch.save({**checkpoint, "decode": decode}, target)


if __name__ == "__main__":
    main()
