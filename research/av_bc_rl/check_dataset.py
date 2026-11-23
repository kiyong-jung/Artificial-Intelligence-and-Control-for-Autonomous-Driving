"""수집한 데이터셋 하나를 검사한다. 두 개를 주면 같은 seed로 다시 모은 결과가 같은지도 본다.

    python research/av_bc_rl/check_dataset.py data/research/run_a.jsonl.gz
    python research/av_bc_rl/check_dataset.py data/research/run_a.jsonl.gz --same-as data/research/run_b.jsonl.gz

검사: 모든 행에 컬럼 표(env/data_schema.py)의 컬럼이 있는지와 안쪽 키(scripts/export_csv.py의 check),
NaN과 무한대가 없는지, 관측이 [-1, 1] 안인지, 행동이 [-1, 1] 안인지, SUMO가 요청 속도를 바꾼 스텝이 없는지,
JSONL과 NPZ의 행 수와 관측이 같은지. 하나라도 어긋나면 종료 코드 1이다.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "scripts"))
from collect_pomdp_data import iter_transitions  # noqa: E402
from env.data_schema import OBSERVATION_FEATURES, SCHEMA_VERSION  # noqa: E402
from export_csv import check as check_columns  # noqa: E402


def _non_finite(value, path=""):
    """중첩 값에서 NaN이나 무한대인 숫자의 위치를 낸다."""
    if isinstance(value, float) and not math.isfinite(value):
        yield path
    elif isinstance(value, dict):
        for key, item in value.items():
            yield from _non_finite(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            yield from _non_finite(item, f"{path}[{index}]")


def check(jsonl_path):
    base = jsonl_path.removesuffix(".jsonl.gz")
    with open(base + ".metadata.json", encoding="utf-8") as f:
        metadata = json.load(f)
    records = list(iter_transitions(jsonl_path))
    errors = list(check_columns(records, metadata["ego"], metadata["simulation"]["step_length"])["errors"])
    if metadata["data"]["schema_version"] != SCHEMA_VERSION:
        errors.append(f"스키마가 {metadata['data']['schema_version']}입니다. 지금 코드는 {SCHEMA_VERSION}입니다.")
    intervened = 0
    for record in records:
        where = f"episode {record['episode']} t {record['t']}"
        errors += [f"{where}: {path}가 유한한 수가 아닙니다." for path in _non_finite(record)]
        for name in ("observation", "next_observation"):
            values = np.asarray(record[name], dtype=float)
            if values.shape != (len(OBSERVATION_FEATURES),) or np.abs(values).max() > 1.0:
                errors.append(f"{where}: {name}이 {len(OBSERVATION_FEATURES)}차원 [-1, 1]이 아닙니다.")
        if np.abs(np.asarray(record["action_raw"], dtype=float)).max() > 1.0:
            errors.append(f"{where}: action_raw가 [-1, 1] 밖입니다.")
        intervened += bool(record["action"]["speed_intervened"])
    if intervened:
        errors.append(f"SUMO가 요청 속도를 바꾼 스텝이 {intervened}개입니다. 안전 보조를 끈 환경에서는 0이어야 합니다.")
    with np.load(base + ".npz") as arrays:
        observation = arrays["observation"]
        for key in arrays.files:
            if arrays[key].dtype.kind == "f" and not np.isfinite(arrays[key]).all():
                errors.append(f"NPZ {key}에 유한하지 않은 값이 있습니다.")
    if len(observation) != len(records) or metadata["transitions"] != len(records):
        errors.append(f"행 수가 다릅니다. JSONL {len(records)}, NPZ {len(observation)}, metadata {metadata['transitions']}")
    elif not np.allclose(observation, np.asarray([r["observation"] for r in records]), atol=1e-4):
        errors.append("JSONL과 NPZ의 관측이 다릅니다.")
    ends = {}
    for record in records:
        reason = record["event"]["termination_reason"]
        if reason:
            ends[reason] = ends.get(reason, 0) + 1
    return records, errors, {"transitions": len(records), "episodes": len({r["episode"] for r in records}),
                             "terminations": ends, "observation_min": float(observation.min()),
                             "observation_max": float(observation.max())}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("jsonl")
    parser.add_argument("--same-as", default=None, help="같은 seed와 설정으로 다시 모은 JSONL. 모든 행이 같아야 한다")
    args = parser.parse_args()
    records, errors, summary = check(args.jsonl)
    if args.same_as:
        other = list(iter_transitions(args.same_as))
        different = sum(1 for a, b in zip(records, other) if a != b) + abs(len(records) - len(other))
        summary["rows_different_from_rerun"] = different
        if different:
            errors.append(f"다시 모은 데이터와 {different}행이 다릅니다.")
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    for error in errors[:20]:
        print("오류:", error)
    print(f"오류 {len(errors)}개")
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
