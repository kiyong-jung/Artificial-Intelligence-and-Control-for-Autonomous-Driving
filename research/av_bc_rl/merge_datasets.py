"""collect_pomdp_data.py가 만든 NPZ 여러 개를 BC 학습용 하나로 합친다(교통량별로 따로 모은 데이터).

에피소드 번호는 파일마다 10000씩 띄워 겹치지 않게 하고, 첫 파일의 metadata.json을 복사해 출처 목록을 덧붙인다.
train_bc.py는 이 metadata로 run_config.json을 써서 test.py와 평가기가 모델을 다시 돌릴 수 있다.

완주하지 못한 에피소드(stuck, 충돌, 시간 초과)는 기본으로 뺀다. 갇힌 전문가의 '그대로 서 있기' 라벨이 수백 스텝
들어가면 BC가 그 상태를 따라 한다. 같은 이름의 .jsonl.gz에서 에피소드마다 끝난 사유를 읽는다.

    python research/av_bc_rl/merge_datasets.py data/research/bc_train.npz data/research/train_*.npz
    python research/av_bc_rl/merge_datasets.py out.npz a.npz b.npz --keep-failed
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from behavior_metrics import load_episodes  # noqa: E402

KEYS = ("observation", "action_raw", "lane_change", "reward", "next_observation", "terminated", "truncated", "t")


def arrived_episodes(npz_path):
    jsonl = Path(npz_path).with_name(Path(npz_path).name.removesuffix(".npz") + ".jsonl.gz")
    return {ep[0]["episode"] for ep in load_episodes(str(jsonl))
            if ep[-1]["event"].get("termination_reason") == "arrived"}


def main(out, sources, keep_failed=False):
    parts = {key: [] for key in KEYS + ("episode",)}
    dropped = {}
    for index, source in enumerate(sources):
        with np.load(source, allow_pickle=False) as data:
            keep = np.ones(len(data["episode"]), dtype=bool)
            if not keep_failed:
                keep = np.isin(data["episode"], sorted(arrived_episodes(source)))
                dropped[str(source)] = sorted({int(e) for e in data["episode"][~keep]})
            for key in KEYS:
                parts[key].append(data[key][keep])
            parts["episode"].append(data["episode"][keep].astype(np.int64) + 10000 * index)
    metas = [json.loads(Path(s).with_name(Path(s).name.removesuffix(".npz") + ".metadata.json").read_text(encoding="utf-8"))
             for s in sources]
    # dagger.py는 metadata의 policy로 라벨을 붙일 전문가를 고른다. 다른 정책이나 다른 지도의 데이터를 섞으면 그 값이 틀린다.
    for key in ("policy",):
        values = {m.get(key) for m in metas}
        if len(values) > 1:
            raise SystemExit(f"{key}가 다른 파일을 합칠 수 없다: {sorted(map(str, values))}")
    maps = {((m.get("road") or {}).get("map_id"), (m.get("road") or {}).get("map_version")) for m in metas}
    if len(maps) > 1:
        raise SystemExit(f"지도가 다른 파일을 합칠 수 없다: {sorted(map(str, maps))}")
    merged = {key: np.concatenate(values) for key, values in parts.items()}
    np.savez_compressed(out, **merged)
    meta = metas[0]
    meta["merged_from"] = [str(Path(s)) for s in sources]
    meta["merged_episode_offset"] = 10000
    meta["dropped_failed_episodes"] = dropped
    Path(out).with_name(Path(out).name.removesuffix(".npz") + ".metadata.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    print(out, {key: value.shape for key, value in merged.items()}, "뺀 에피소드:", dropped)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out")
    parser.add_argument("sources", nargs="+")
    parser.add_argument("--keep-failed", action="store_true", help="완주하지 못한 에피소드도 넣는다")
    args = parser.parse_args()
    main(args.out, args.sources, args.keep_failed)
