"""지름길 후보를 임시 지도로 만들어, 버스가 겪는 시나리오와 도로의 안전이 어떻게 달라지는지 잰다.

    python scripts/shortcut_experiment.py --try "" dongmak7 jandari3 dongmak7+jandari3

후보는 scripts/build_hapjeong.py의 SHORTCUTS에 있다. ""는 지름길 없는 기준선이다.
조합마다 maps/hapjeong_trial_<이름>에 지도를 만들고 다음을 잰 뒤 지운다.
- 재료: scripts/scenario_supply.py로 배경 교통만 한 시간씩 세 번 돌려, 골목에서 버스 방향으로 합류하는 차와
  버스 경로의 교통량을 센다.
- 시나리오: 경로 추종 규칙 버스로 busy 교통에서 --episodes번 달려 scripts/scenario_occurrence.py로 센다.
- 안전: scripts/audit_road.py를 baseline·busy × seed 0·7·42로 돌려 충돌, 순간이동, 비상 정지를 센다.
결과는 results/shortcut_experiment/<이름>.json에 쓰고, 한 줄 요약을 --log(TSV)에 덧붙인다.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts import build_hapjeong
from scripts.scenario_occurrence import main as count_scenarios
from scripts.scenario_supply import measure as measure_supply

COLUMNS = ["shortcuts", "S5_merges_per_h", "volume_min_per_h", "volume_at_stops_per_h", "episodes", "arrived", "S5_cut_ins", "S5_episodes", "S5_waiting_episodes", "S2+_episodes",
           "stop_passes_shared", "in_lane_share", "collisions", "teleports", "emergency_stops", "collision_lanes"]


def measure(names: list[str], episodes: int, seed: int, workers: int, safety: bool, keep: Path | None = None) -> dict:
    label = "+".join(names) or "none"
    map_id = f"hapjeong_trial_{label.replace('+', '_')}"
    folder = BASE / "maps" / map_id
    shutil.rmtree(folder, ignore_errors=True)
    (folder / "source").mkdir(parents=True)
    for path in (BASE / "maps" / "hapjeong" / "source").iterdir():
        shutil.copy2(path, folder / "source" / path.name)
    try:
        build_hapjeong.FOLDER, build_hapjeong.SOURCE = folder, folder / "source"
        build_hapjeong.use_shortcuts(names)
        definition = build_hapjeong.build()
        supply = measure_supply(map_id)
        with tempfile.TemporaryDirectory() as temporary:
            if keep:   # 주행 기록을 남겨 두면 측정기를 고친 뒤 다시 달리지 않고 다시 셀 수 있다.
                temporary = keep / f"{label}_seed{seed}"
                temporary.mkdir(parents=True, exist_ok=True)
            subprocess.run([sys.executable, "collect_pomdp_data.py", "--map", map_id, "--policy", "route-follow",
                            "--scenario", "busy", "--episodes", str(episodes), "--workers", str(workers),
                            "--max-steps", "1200", "--seed", str(seed), "--out-dir", temporary, "--name", "run"],
                           cwd=BASE, check=True, capture_output=True, text=True)
            scenarios = count_scenarios(str(Path(temporary) / "run.jsonl.gz"))
            reasons = Counter()
            import gzip
            with gzip.open(Path(temporary) / "run.jsonl.gz", "rt", encoding="utf-8") as lines:
                for line in lines:
                    row = json.loads(line)
                    if row["terminated"] or row["truncated"]:
                        reasons[row["event"]["termination_reason"]] += 1
            audit = {"runs": [], "structure": {"dead_lanes": []}}
            if safety:
                audit_path = Path(temporary) / "audit.json"
                subprocess.run([sys.executable, "scripts/audit_road.py", "--map", map_id, "--scenarios", "baseline", "busy",
                                "--out", str(audit_path)], cwd=BASE, check=True, capture_output=True, text=True)
                audit = json.loads(audit_path.read_text(encoding="utf-8"))
    finally:
        shutil.rmtree(folder, ignore_errors=True)
    events = [event for run in audit["runs"] for event in run["events"]]
    kinds = Counter(event["kind"] for event in events)
    by_id = {row["id"]: row for row in scenarios["scenarios"]}
    return {
        "shortcuts": label, "episodes": scenarios["episodes"], "seed": seed, "termination": dict(reasons),
        "flows": len(definition["flows"]),
        "alley_origin_vehs_per_hour": round(sum(flow["vehs_per_hour"] for flow in definition["flows"]
                                                if flow["from"] in build_hapjeong.ALLEY_ORIGINS), 1),
        "supply": {key: value for key, value in supply.items() if key != "runs"},
        "scenarios": scenarios,
        "safety": {"runs": len(audit["runs"]), "collisions": kinds["collision"], "teleports": kinds["teleport"],
                   "emergency_stops": kinds["emergency_stop"], "emergency_brakings": kinds["emergency_braking"],
                   "collision_lanes": dict(Counter(e["lane"] for e in events if e["kind"] == "collision")),
                   "dead_lanes": [lane["lane"] for lane in audit["structure"]["dead_lanes"]],
                   "bus_trips": sum(run["bus_trips"] for run in audit["runs"])},
        "row": {
            "shortcuts": label, "S5_merges_per_h": supply["s5_merges_per_hour_total"],
            "volume_min_per_h": supply["volume_per_hour_min"], "volume_at_stops_per_h": supply["volume_per_hour_at_stops"],
            "episodes": scenarios["episodes"], "arrived": reasons["arrived"],
            "S5_cut_ins": by_id["S5"]["count"], "S5_episodes": by_id["S5"]["episodes"],
            "S5_waiting_episodes": by_id["S5~"]["episodes"], "S2+_episodes": by_id["S2+"]["episodes"],
            "stop_passes_shared": f"{scenarios['stop_passes_with_vehicle_in_lane']}/{scenarios['stop_passes']}",
            "in_lane_share": scenarios["in_lane_step_share"],
            **({"collisions": kinds["collision"], "teleports": kinds["teleport"], "emergency_stops": kinds["emergency_stop"],
                "collision_lanes": ",".join(sorted({e["lane"] for e in events if e["kind"] == "collision"})) or "-"}
               if safety else dict.fromkeys(("collisions", "teleports", "emergency_stops", "collision_lanes"), "안 잼")),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="지름길 후보별 시나리오 발생과 안전을 잰다")
    parser.add_argument("--try", dest="trials", nargs="+", required=True, help='후보 이름을 +로 이은 조합. ""는 기준선')
    parser.add_argument("--episodes", type=int, default=40)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--workers", type=int, default=10)
    parser.add_argument("--skip-safety", action="store_true", help="audit_road.py를 돌리지 않는다(시나리오만 빨리 볼 때)")
    parser.add_argument("--keep", type=Path, default=None, help="주행 기록을 지우지 않고 남길 폴더")
    parser.add_argument("--log", type=Path, default=BASE / ".audit" / "hapjeong-shortcuts.tsv")
    parser.add_argument("--out", type=Path, default=BASE / "results" / "shortcut_experiment")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    if not args.log.exists():
        args.log.write_text("\t".join(["seed", *COLUMNS]) + "\n", encoding="utf-8")
    for trial in args.trials:
        names = [name for name in trial.split("+") if name]
        result = measure(names, args.episodes, args.seed, args.workers, not args.skip_safety, args.keep)
        print({node: counts for node, counts in result["scenarios"]["alley_merges"].items() if any(counts.values())})
        (args.out / f"{result['shortcuts']}_seed{args.seed}.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        line = "\t".join(str(value) for value in [args.seed, *(result["row"][column] for column in COLUMNS)])
        with args.log.open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")
        print(line, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
