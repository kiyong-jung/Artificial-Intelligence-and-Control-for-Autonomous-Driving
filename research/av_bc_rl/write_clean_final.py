"""filter_episodes.py의 보고서대로 묶음 하나에서 남길 에피소드만 골라 NPZ, JSONL, metadata를 새 폴더에 쓴다.

원본 묶음은 그대로 둔다. metadata에는 줄어든 transitions와 뺀 에피소드 번호, 사유를 적는다.

    python research/av_bc_rl/write_clean_final.py data/final_filter/heavy_s0_000.json --src data/final --out data/final_clean
"""
import argparse
import gzip
import json
import sys
from pathlib import Path

import numpy as np

ROW_KEYS = ("observation", "action_raw", "lane_change", "reward", "next_observation", "terminated", "truncated", "episode", "t")


def write_npz(src, dst, keep):
    z = dict(np.load(src))
    mask = np.isin(z["episode"], np.fromiter(keep, dtype=np.int64))
    out = {key: z[key][mask] for key in ROW_KEYS}
    offsets = z["detected_offsets"]
    counts = np.diff(offsets)
    out["detected_xy"] = z["detected_xy"][np.repeat(mask, counts)]
    out["detected_offsets"] = np.concatenate([[0], np.cumsum(counts[mask])]).astype(offsets.dtype)
    np.savez_compressed(dst, **out)
    return int(mask.sum())


def write_jsonl(src, dst, keep):
    n = 0
    with gzip.open(src, "rt", encoding="utf-8") as f, gzip.open(dst, "wt", encoding="utf-8") as out:
        for line in f:
            if json.loads(line)["episode"] in keep:
                out.write(line)
                n += 1
    return n


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("report")
    parser.add_argument("--src", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    (path, episodes), = json.loads(Path(args.report).read_text(encoding="utf-8")).items()
    name = Path(path).name.removesuffix(".jsonl.gz")
    src, out = Path(args.src), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    keep = {int(e) for e, info in episodes.items() if info["keep"]}
    npz_rows = write_npz(src / f"{name}.npz", out / f"{name}.npz", keep)
    jsonl_rows = write_jsonl(src / f"{name}.jsonl.gz", out / f"{name}.jsonl.gz", keep)
    if npz_rows != jsonl_rows:
        sys.exit(f"{name}: NPZ {npz_rows}행과 JSONL {jsonl_rows}행이 다르다")
    meta = json.loads((src / f"{name}.metadata.json").read_text(encoding="utf-8"))
    meta["filtered"] = {
        "tool": "research/av_bc_rl/filter_episodes.py (기본 기준: 버스 실패와 시뮬레이션 이상)",
        "source_transitions": meta["transitions"],
        "dropped_episodes": {e: info["flags"] for e, info in episodes.items() if not info["keep"]},
    }
    meta["transitions"] = npz_rows
    meta["episodes_kept"] = len(keep)
    (out / f"{name}.metadata.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"{name}: 에피소드 {len(keep)}/{len(episodes)}, {npz_rows}행")
    return 0


if __name__ == "__main__":
    sys.exit(main())
