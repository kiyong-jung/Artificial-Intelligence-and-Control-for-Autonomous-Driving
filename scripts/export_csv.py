"""수집한 JSONL을 컬럼 표(env/data_schema.py)대로 검사하고, 앞의 몇 행을 CSV로 내보낸다.

    python scripts/export_csv.py data/hapjeong/run1.jsonl.gz --rows 15 --out 00조_데이터예시.csv

검사: 모든 행에 표의 컬럼이 있는지, state와 감지 목록의 안쪽 키가 표와 같은지, 기록된 가속도가 ego 제원의
한계 안인지 본다. 하나라도 어긋나면 CSV를 쓰지 않고 끝난다.
CSV에서 벡터, dict, list는 JSON 문자열 한 칸이다.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from collect_pomdp_data import iter_transitions
from env.data_schema import (COLUMNS, DETECTED_PEDESTRIAN_KEYS, DETECTED_VEHICLE_KEYS, OBSERVATION_FEATURES,
                             STATE_KEYS, lookup)


def check(records: list[dict], ego: dict, step_length: float) -> dict:
    """어긋난 곳의 목록(errors)과 데이터가 실제로 담은 값의 요약(summary)."""
    errors = []
    seen = {"vehicles": 0, "pedestrians": 0, "detected_vehicles": 0, "detected_pedestrians": 0,
            "traffic_light": 0, "next_turn": 0, "crossing": 0, "stop": 0, "signals_on": 0}
    acceleration, jerk, peak_jerk = [], [], []

    def keys(where, row, expected):
        if list(row) != expected:
            errors.append(f"{where}: 키가 표와 다릅니다. 없는 것 {sorted(set(expected) - set(row))}, "
                          f"남는 것 {sorted(set(row) - set(expected))}")

    for record in records:
        where = f"episode {record.get('episode')} t {record.get('t')}"
        before = len(errors)
        for column, *_ in COLUMNS:
            try:
                lookup(record, column)
            except (KeyError, TypeError):
                errors.append(f"{where}: 컬럼 {column}이 없습니다.")
        if len(errors) > before:
            continue
        for name in ("observation", "next_observation"):
            if len(record[name]) != len(OBSERVATION_FEATURES):
                errors.append(f"{where}: {name}이 {len(record[name])}차원입니다.")
        for name in ("state", "next_state"):
            state = record[name]
            if not state["ego"]["alive"]:
                continue
            keys(f"{where} {name}.ego", state["ego"], STATE_KEYS["ego"])
            keys(f"{where} {name}.navigation", state["navigation"], STATE_KEYS["navigation"])
            for group in ("vehicles", "pedestrians"):
                for row in state[group]:
                    keys(f"{where} {name}.{group}", row, STATE_KEYS[group])
            if state["traffic_light"] is not None:
                keys(f"{where} {name}.traffic_light", state["traffic_light"], STATE_KEYS["traffic_light"])
        for row in record["detected_vehicles"]:
            keys(f"{where} detected_vehicles", row, DETECTED_VEHICLE_KEYS)
        for row in record["detected_pedestrians"]:
            keys(f"{where} detected_pedestrians", row, DETECTED_PEDESTRIAN_KEYS)

        state = record["state"]
        seen["vehicles"] += bool(state["vehicles"])
        seen["pedestrians"] += bool(state["pedestrians"])
        seen["detected_vehicles"] += bool(record["detected_vehicles"])
        seen["detected_pedestrians"] += bool(record["detected_pedestrians"])
        seen["traffic_light"] += state["traffic_light"] is not None
        seen["next_turn"] += state["navigation"]["next_turn"] is not None
        seen["crossing"] += state["navigation"]["distance_to_crossing"] is not None
        seen["stop"] += state["navigation"]["next_stop_id"] is not None
        seen["signals_on"] += bool(state["ego"]["signals"] & 3)
        after = record["next_state"]["ego"]
        if after["alive"]:
            acceleration.append(after["acceleration"])
            jerk.append(after["jerk"])
        peak_jerk.append(record["comfort"]["peak_jerk"])
        # SUMO는 speedMode의 안전 보조로 제원의 감속 한계보다 세게 멈출 수 있다. 가속은 넘으면 안 된다.
        if record["comfort"]["peak_acceleration"] > max(ego["accel"], ego["emergency_decel"]) + 1e-3:
            errors.append(f"{where}: 가속도 {record['comfort']['peak_acceleration']}이 제원의 한계를 넘습니다.")
    if acceleration and max(acceleration) > ego["accel"] + 1e-3:
        errors.append(f"가속 최댓값 {max(acceleration):.3f}m/s²이 제원의 가속 한계 {ego['accel']}를 넘습니다.")
    violations = sum(bool(record["comfort"]["jerk_violation"]) for record in records)
    summary = {
        "rows": len(records), "episodes": len({record["episode"] for record in records}),
        "rows_with": seen,
        "acceleration_mps2": {"min": min(acceleration, default=None), "max": max(acceleration, default=None)},
        "jerk_mps3": {"min": min(jerk, default=None), "max": max(jerk, default=None)},
        "peak_jerk_mps3_max": max(peak_jerk, default=None), "jerk_violation_rows": violations,
        "termination": sorted({str(record["event"]["termination_reason"]) for record in records
                               if record["terminated"] or record["truncated"]}),
    }
    return {"errors": errors, "summary": summary}


def cell(value):
    if isinstance(value, bool):
        return "TRUE" if value else "FALSE"
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    return "" if value is None else value


def main() -> int:
    parser = argparse.ArgumentParser(description="수집 데이터의 컬럼을 검사하고 CSV 예시를 만든다")
    parser.add_argument("data", type=Path, help="collect_pomdp_data.py가 만든 .jsonl.gz")
    parser.add_argument("--rows", type=int, default=15, help="CSV로 내보낼 행 수")
    parser.add_argument("--skip", type=int, default=0, help="CSV에서 건너뛸 앞쪽 행 수")
    parser.add_argument("--out", type=Path, default=None, help="CSV 경로. 없으면 검사만 한다")
    args = parser.parse_args()

    meta = json.loads(Path(str(args.data).replace(".jsonl.gz", ".metadata.json")).read_text(encoding="utf-8"))
    records = list(iter_transitions(str(args.data)))
    result = check(records, meta["ego"], meta["simulation"]["step_length"])
    print(json.dumps(result["summary"], ensure_ascii=False, indent=2))
    if result["errors"]:
        print(f"\n어긋난 곳 {len(result['errors'])}개:")
        print("\n".join(result["errors"][:20]))
        return 1
    print(f"\n컬럼 {len(COLUMNS)}개가 {len(records)}행 모두에 있습니다.")
    if args.out:
        with args.out.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow([column for column, *_ in COLUMNS])
            for record in records[args.skip:args.skip + args.rows]:
                writer.writerow([cell(lookup(record, column)) for column, *_ in COLUMNS])
        print(f"CSV {min(args.rows, len(records) - args.skip)}행: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
