"""배경 교통만 돌려, 시나리오의 재료가 버스 경로에 얼마나 공급되는지 센다.

    python scripts/scenario_supply.py --map hapjeong

버스가 있든 없든 도로와 수요가 정하는 값이라, 버스 에피소드를 세는 것보다 표본이 크다.
- S5 재료: 골목(id가 G로 시작)에서 신호 없이 버스 경로 도로로, 버스와 같은 방향으로 들어오는 차(대/시간).
- S1~S3 재료: 버스 경로 도로를 버스와 같은 방향으로 지나는 차(대/시간). 정류장이 있는 도로와 좌회전 전 도로를 따로 적는다.
- S6 재료: 신호 없는 횡단보도를 건너는 보행자(명/시간).
SUMO의 vehroute 출력에서 차량이 실제로 지난 도로를 읽는다.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from sumolib import checkBinary

from env import road_builder
from env.road_config import EGO, ROAD, TRAFFIC, configure_map
from maps import available_maps
from scripts.scenario_occurrence import ALLEY, S6_CROSSING, alley_merges


def run(map_id: str, scenario: str, seed: int, seconds: float) -> dict:
    configure_map(scenario, seed, None, map_id)
    definition = ROAD["network"]
    route = definition["routes"][definition["ego_route"]]
    with tempfile.TemporaryDirectory() as temporary:
        cfg = road_builder.build(ROAD, EGO, TRAFFIC, temporary)
        subprocess.run([checkBinary("sumo"), "-c", cfg, "--end", str(seconds), "--no-step-log", "true", "--no-warnings", "true",
                        "--duration-log.disable", "true", "--vehroute-output", "vehroutes.xml",
                        "--vehroute-output.write-unfinished", "true", "--vehroute-output.exit-times", "true"],
                       cwd=temporary, check=True, capture_output=True, text=True)
        alley_end, _ = alley_merges(Path(temporary) / "network.net.xml")
        merges, volume, walkers = Counter(), Counter(), 0
        root = ET.parse(Path(temporary) / "vehroutes.xml").getroot()
        for vehicle in root.iter("vehicle"):
            # 끝까지 가지 못한 차량은 실제로 지난 도로까지만 센다(exitTimes의 개수).
            element = vehicle.find("route")
            edges = element.get("edges").split()
            edges = edges[:len(element.get("exitTimes", "").split()) or len(edges)]
            for edge in set(edges) & set(route):
                volume[edge] += 1
            for before, after in zip(edges, edges[1:]):
                if before in alley_end and after in route:
                    merges[f"{alley_end[before]} → {after}"] += 1
        for person in root.iter("person"):
            walkers += any(S6_CROSSING[1:].rsplit("_c", 1)[0] in (walk.get("edges") or "") for walk in person.iter("walk"))
    hours = seconds / 3600
    stops = {stop["lane"].rsplit("_", 1)[0] for stop in definition.get("bus_stops", [])}
    return {"seed": seed,
            "alley_merges_into_bus_direction_per_hour": {key: round(value / hours, 1) for key, value in sorted(merges.items())},
            "bus_route_volume_per_hour": {edge: round(volume[edge] / hours, 1) for edge in route},
            "stop_edges": sorted(stops & set(route))}


def summarise(runs: list[dict], route: list[str]) -> dict:
    mean = lambda values: round(sum(values) / len(values), 1)
    merges = sorted({key for row in runs for key in row["alley_merges_into_bus_direction_per_hour"]})
    volume = {edge: mean([row["bus_route_volume_per_hour"][edge] for row in runs]) for edge in route}
    stops = runs[0]["stop_edges"]
    return {
        "s5_merges_per_hour": {key: mean([row["alley_merges_into_bus_direction_per_hour"].get(key, 0) for row in runs]) for key in merges},
        "s5_merges_per_hour_total": mean([sum(row["alley_merges_into_bus_direction_per_hour"].values()) for row in runs]),
        "volume_per_hour": volume,
        "volume_per_hour_at_stops": mean([sum(row["bus_route_volume_per_hour"][edge] for edge in stops) / len(stops) for row in runs]),
        "volume_per_hour_min": min(volume.values()), "volume_per_hour_mean": mean(list(volume.values())),
    }


def measure(map_id: str, scenario: str = "busy", seeds=(0, 7, 42), seconds: float = 3600) -> dict:
    with ProcessPoolExecutor() as pool:
        runs = list(pool.map(run, [map_id] * len(seeds), [scenario] * len(seeds), seeds, [seconds] * len(seeds)))
    configure_map(scenario, seeds[0], None, map_id)
    route = ROAD["network"]["routes"][ROAD["network"]["ego_route"]]
    return {"map": map_id, "scenario": scenario, "seconds": seconds, "seeds": list(seeds), **summarise(runs, route), "runs": runs}


def main() -> int:
    parser = argparse.ArgumentParser(description="시나리오의 재료가 버스 경로에 얼마나 공급되는지 센다")
    parser.add_argument("--map", choices=available_maps(), default="hapjeong")
    parser.add_argument("--scenario", default="busy")
    parser.add_argument("--seconds", type=float, default=3600)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    result = measure(args.map, args.scenario, seconds=args.seconds)
    print(json.dumps({key: value for key, value in result.items() if key != "runs"}, ensure_ascii=False, indent=1))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
