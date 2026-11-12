"""버스 경로를 구간으로 나눠, 구간마다 교통량이 충분한지와 시나리오가 얼마나 나오는지 본다.

    python3 scripts/scenario_sections.py data/pomdp/run.jsonl.gz [배경 교통 측정 JSON]

주행 기록(JSONL.gz)을 scripts/scenario_occurrence.py의 Counter로 한 줄씩 세면서, 늘어난 횟수를 그 순간 버스가 있던
구간에 적는다. 둘째 인자로 scripts/shortcut_experiment.py의 결과 JSON을 주면 구간의 버스 방향 교통량(대/시간)도 적는다.
"""
from __future__ import annotations

import gzip
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scenario_occurrence import Counter, alley_merges  # noqa: E402

# (구간 이름, 그 구간의 도로들). 도로를 나눈 조각(R71.135)은 원래 id(R71)로 본다.
SECTIONS = [("독막로 서쪽", ["R69", "R69a", "R71"]), ("독막로 동쪽", ["R73", "R77", "R111", "R111b"]),
            ("와우산로", ["R112"]), ("잔다리로 동쪽", ["R113"]), ("잔다리로", ["-R96"]),
            ("좌회전 대기", ["R94"]), ("양화로", ["R54a", "R54b", "R54", "R42"])]
KEYS = ["S2", "S2+", "S5", "S5~", "S6", "S6~"]


def base(road: str) -> str:
    return road.split(".")[0]


def measure(path: str, volume: dict[str, float] | None = None) -> list[dict]:
    counter = Counter(*alley_merges(Path(path.replace(".jsonl.gz", ".assets")) / "sumo" / "network.net.xml"))
    section_of = {road: name for name, roads in SECTIONS for road in roads}
    rows = {name: {"steps": 0, "in_lane_steps": 0, "stops": 0, "stops_shared": 0, **dict.fromkeys(KEYS, 0)} for name, _ in SECTIONS}
    seen: dict[str, set] = {key: set() for key in KEYS}
    with gzip.open(path, "rt", encoding="utf-8") as lines:
        for line in lines:
            row = json.loads(line)
            before = (len(counter.stop_passes), len(counter.stop_passes_shared), counter.in_lane_steps,
                      {key: len(counter.hit[key]) for key in KEYS}, counter.count["S5"])
            counter.add(row)
            name = section_of.get(base(row["state"]["ego"]["road_id"]))
            if name is None:      # 교차로 안
                continue
            target = rows[name]
            target["steps"] += 1
            target["in_lane_steps"] += counter.in_lane_steps - before[2]
            target["stops"] += len(counter.stop_passes) - before[0]
            target["stops_shared"] += len(counter.stop_passes_shared) - before[1]
            for key in KEYS:      # S5는 끼어든 차의 수, 나머지는 그 일이 있었던 에피소드 수
                target[key] += (counter.count["S5"] - before[4]) if key == "S5" else len(counter.hit[key]) - before[3][key]
    episodes = len(counter.episodes)
    table = []
    for name, roads in SECTIONS:
        row = rows[name]
        table.append({"section": name, "vehicles_per_hour": min((volume[road] for road in volume if base(road) in roads), default=None)
                      if volume else None,
                      "in_lane_share": round(row["in_lane_steps"] / row["steps"], 3) if row["steps"] else None,
                      "seconds_per_episode": round(row["steps"] * 0.5 / episodes, 1),
                      "stops_shared": f"{row['stops_shared']}/{row['stops']}", **{key: row[key] for key in KEYS}})
    return table


def main() -> int:
    volume = None
    if len(sys.argv) > 2:
        volume = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))["supply"]["volume_per_hour"]
    table = measure(sys.argv[1], volume)
    print(f"{'구간':<10} {'대/시간':>7} {'옆·같은 차로에 차':>10} {'머문 시간':>8} {'정류장 공유':>9} " + " ".join(f"{key:>4}" for key in KEYS))
    for row in table:
        print(f"{row['section']:<10} {row['vehicles_per_hour'] if row['vehicles_per_hour'] is not None else '-':>8} "
              f"{row['in_lane_share']:>12.0%} {row['seconds_per_episode']:>9.1f}초 {row['stops_shared']:>10} "
              + " ".join(f"{row[key]:>4}" for key in KEYS))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
