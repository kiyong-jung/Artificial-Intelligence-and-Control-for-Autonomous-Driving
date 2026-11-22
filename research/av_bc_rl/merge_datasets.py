"""collect_pomdp_data.py가 만든 NPZ 여러 개를 BC 학습용 하나로 합친다(교통량별로 따로 모은 데이터).

에피소드 번호는 파일마다 10000씩 띄워 겹치지 않게 하고, 첫 파일의 metadata.json을 복사해 출처 목록을 덧붙인다.
train_bc.py는 이 metadata로 run_config.json을 써서 test.py와 평가기가 모델을 다시 돌릴 수 있다.

    python research/av_bc_rl/merge_datasets.py data/research/bc_train.npz data/research/train_*.npz
"""
import json
import sys
from pathlib import Path

import numpy as np

KEYS = ("observation", "action_raw", "lane_change", "reward", "next_observation", "terminated", "truncated", "t")


def main(out, sources):
    parts = {key: [] for key in KEYS + ("episode",)}
    for index, source in enumerate(sources):
        with np.load(source, allow_pickle=False) as data:
            for key in KEYS:
                parts[key].append(data[key])
            parts["episode"].append(data["episode"].astype(np.int64) + 10000 * index)
    merged = {key: np.concatenate(values) for key, values in parts.items()}
    np.savez_compressed(out, **merged)
    first = Path(sources[0])
    meta = json.loads(first.with_name(first.name.removesuffix(".npz") + ".metadata.json").read_text(encoding="utf-8"))
    meta["merged_from"] = [str(Path(s)) for s in sources]
    meta["merged_episode_offset"] = 10000
    Path(out).with_name(Path(out).name.removesuffix(".npz") + ".metadata.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    print(out, {key: value.shape for key, value in merged.items()})


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
