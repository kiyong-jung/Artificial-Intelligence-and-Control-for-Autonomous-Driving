"""J82(상수역 사거리)의 횡단보도를 건너는 보행자를 보행자 경로 파일에서 뺀다(도로망은 그대로).

    python scripts/remove_j82_pedestrians.py

quiet 3바퀴째(t≈933~1250초)에 시험용 버스가 J82 교차로 안(:J82_24)에서 약 300초 서 있었다. 같은 시드에서 배경 보행자를 모두 뺀
측정은 이 정체가 없었다. 보행자(ped129)가 R112 횡단보도(c0) 앞에서 속도 0으로 서 있고 버스와 차량이 함께 서 있었다.
보행자를 없애기로 했다(사용자 결정). 남은 보행자는 그대로 두고, 빠진 만큼 다른 횡단보도로 옮겨 가지 않게 보행자 수만 줄인다.

SUMO를 보행자만 넣고 돌려(차량 없음) 횡단보도 :J82_c* 위를 지나는 보행자를 찾고, 그 보행자를 pedestrians.rou.xml에서 뺀다.
이미 뺐다면 아무것도 바꾸지 않는다.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import sumolib
import traci

BASE = Path(__file__).resolve().parents[1]
SOURCE = BASE / "maps" / "hapjeong" / "source"
MAP_JSON = BASE / "maps" / "hapjeong" / "map.json"
NODE = "J82"


def pedestrians_on_crossings(net: Path, rou: Path, signals: Path) -> set[str]:
    """보행자만 넣고 끝까지 돌려 횡단보도 :J82_c* 위에 선 적이 있는 보행자 id."""
    end = int(max(float(x) for x in re.findall(r'depart="([0-9.]+)"', rou.read_text(encoding="utf-8")))) + 400
    traci.start([sumolib.checkBinary("sumo"), "-n", str(net), "-r", str(rou), "-a", str(signals), "--end", str(end),
                 "--no-step-log", "true", "--no-warnings", "true", "--ignore-route-errors", "true"])
    found: set[str] = set()
    try:
        while traci.simulation.getTime() < end:
            traci.simulationStep()
            for pid in traci.person.getIDList():
                if traci.person.getRoadID(pid).startswith(f":{NODE}_c"):
                    found.add(pid)
    finally:
        traci.close()
    return found


def main() -> None:
    rou = SOURCE / "pedestrians.rou.xml"
    gone = pedestrians_on_crossings(SOURCE / "network.net.xml", rou, SOURCE / "signals.add.xml")
    text = rou.read_text(encoding="utf-8")
    before = len(re.findall(r"<person ", text))
    for pid in sorted(gone):
        text = re.sub(rf'<person id="{re.escape(pid)}"[^>]*>.*?</person>', "", text, flags=re.S)
    rou.write_text(text, encoding="utf-8")
    after = len(re.findall(r"<person ", text))
    definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    definition["network_source"]["pedestrians"]["removed_j82_crossing_walkers"] = {
        "removed": sorted(gone), "persons_before": before, "persons_after": after,
        "script": "scripts/remove_j82_pedestrians.py"}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"J82 횡단보도 보행자 {len(gone)}명 제거: {before} -> {after}명")


if __name__ == "__main__":
    sys.exit(main())
