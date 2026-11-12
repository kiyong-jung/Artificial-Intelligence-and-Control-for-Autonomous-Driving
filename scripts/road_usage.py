"""배경 교통을 흘려 도로마다 지나간 차의 수를 센다. 차가 거의 다니지 않는 도로를 찾는 데 쓴다.

    python scripts/road_usage.py --map hapjeong --out maps/hapjeong/references/road_usage.json

배경 교통만 --seconds 동안 seed 세 개로 돌리고, 차량 경로 기록에서 도로별 통과 대수(대/시간)의 평균을 낸다.
끝까지 가지 못한 차량은 실제로 지난 도로까지만 센다.
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
from sumolib.net import readNet

from env import road_builder
from env.road_config import EGO, ROAD, TRAFFIC, configure_map
from maps import available_maps

SEEDS = (0, 7, 42)


def run(map_id: str, scenario: str, seed: int, seconds: float) -> dict[str, float]:
    configure_map(scenario, seed, None, map_id)
    with tempfile.TemporaryDirectory() as temporary:
        cfg = road_builder.build(ROAD, EGO, TRAFFIC, temporary)
        subprocess.run([checkBinary("sumo"), "-c", cfg, "--end", str(seconds), "--no-step-log", "true", "--no-warnings", "true",
                        "--duration-log.disable", "true", "--vehroute-output", "vehroutes.xml",
                        "--vehroute-output.write-unfinished", "true", "--vehroute-output.exit-times", "true"],
                       cwd=temporary, check=True, capture_output=True, text=True)
        passed = Counter()
        for vehicle in ET.parse(Path(temporary) / "vehroutes.xml").getroot().iter("vehicle"):
            # 아직 출발하지 못한 차량은 경로가 없다. 정체가 심하면 생긴다.
            element = vehicle.find("route")
            if element is None:
                continue
            edges = element.get("edges").split()
            passed.update(set(edges[:len(element.get("exitTimes", "").split()) or len(edges)]))
    return {edge: count * 3600 / seconds for edge, count in passed.items()}


def measure(map_id: str, scenario: str, seconds: float) -> dict:
    with ProcessPoolExecutor() as pool:
        runs = list(pool.map(run, [map_id] * len(SEEDS), [scenario] * len(SEEDS), SEEDS, [seconds] * len(SEEDS)))
    configure_map(scenario, SEEDS[0], None, map_id)
    definition = ROAD["network"]
    net = readNet(definition["net_path"])
    route = set(definition["routes"][definition["ego_route"]])
    roads = {}
    for edge in net.getEdges(withInternal=False):
        if not any(lane.allows("passenger") or lane.allows("bus") for lane in edge.getLanes()):
            continue
        roads[edge.getID()] = {"name": edge.getName(), "length_m": round(edge.getLength(), 1),
                               "vehicles_per_hour": round(sum(row.get(edge.getID(), 0.0) for row in runs) / len(runs), 1),
                               "on_bus_route": edge.getID() in route}
    return {"map": map_id, "version": definition.get("version"), "scenario": scenario, "seconds": seconds, "seeds": list(SEEDS),
            "roads": dict(sorted(roads.items(), key=lambda item: item[1]["vehicles_per_hour"]))}


def main() -> int:
    parser = argparse.ArgumentParser(description="도로마다 지나간 차의 수를 센다")
    parser.add_argument("--map", choices=available_maps(), default="hapjeong")
    parser.add_argument("--scenario", default="busy")
    parser.add_argument("--seconds", type=float, default=3600)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    result = measure(args.map, args.scenario, args.seconds)
    for edge, row in result["roads"].items():
        print(f"{row['vehicles_per_hour']:>7.1f}  {edge:<12} {row['length_m']:>7.1f}m  {row['name']}{'  (버스 경로)' if row['on_bus_route'] else ''}")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
