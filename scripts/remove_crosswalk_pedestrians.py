"""지정한 횡단보도를 건너는 보행자를 보행자 경로 파일에서 뺀다(도로망은 그대로).

    python scripts/remove_crosswalk_pedestrians.py J1 c0

J1 대기 구역 :J1_w0에 보행자 세 명(ped18, ped38, ped56)이 속도 0으로 오래 서 있는 동안 시험용 버스가 J1 좌회전 끝(:J1_23)에서
900초 넘게 섰다(busy 2바퀴째). 그 보행자들은 w0에서 이어지는 횡단보도 :J1_c0을 건너려고 기다렸다. 사용자가 그 횡단보도를 건너는
보행자를 빼라고 했다.

SUMO를 보행자만 넣고 돌려(차량 없음) 횡단보도 :노드_c번호 위를 지나는 보행자를 찾아 pedestrians.rou.xml에서 뺀다. 보행자를 빼면
나머지 보행자의 경로가 조금씩 달라질 수 있어(대기 시간을 경로 선택에 쓴다) 더 찾을 보행자가 없을 때까지 되풀이한다. 이미 뺐다면
아무것도 바꾸지 않는다. map.json에 뺀 수를 기록한다.
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


def pedestrians_on(crossings: tuple[str, ...]) -> set[str]:
    """보행자만 넣고 끝까지 돌려 지정한 횡단보도 위에 선 적이 있는 보행자 id."""
    rou = SOURCE / "pedestrians.rou.xml"
    end = int(max(float(x) for x in re.findall(r'depart="([0-9.]+)"', rou.read_text(encoding="utf-8")))) + 400
    traci.start([sumolib.checkBinary("sumo"), "-n", str(SOURCE / "network.net.xml"), "-r", str(rou),
                 "-a", str(SOURCE / "signals.add.xml"), "--end", str(end), "--no-step-log", "true", "--no-warnings", "true",
                 "--ignore-route-errors", "true"])
    found: set[str] = set()
    try:
        while traci.simulation.getTime() < end:
            traci.simulationStep()
            for pid in traci.person.getIDList():
                if traci.person.getRoadID(pid) in crossings:
                    found.add(pid)
    finally:
        traci.close()
    return found


def main() -> None:
    node, suffixes = sys.argv[1], sys.argv[2:]
    crossings = tuple(f":{node}_{s}" for s in suffixes)
    rou = SOURCE / "pedestrians.rou.xml"
    before = len(re.findall(r"<person ", rou.read_text(encoding="utf-8")))
    removed: list[str] = []
    while True:
        gone = pedestrians_on(crossings)
        if not gone:
            break
        text = rou.read_text(encoding="utf-8")
        for pid in sorted(gone):
            text = re.sub(rf'<person id="{re.escape(pid)}"[^>]*>.*?</person>', "", text, flags=re.S)
        rou.write_text(text, encoding="utf-8")
        removed += sorted(gone)
        print(f"  {len(gone)}명 제거")
    after = len(re.findall(r"<person ", rou.read_text(encoding="utf-8")))
    if removed:
        definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
        rec = definition["network_source"]["pedestrians"].setdefault("removed_crossing_walkers", {})
        rec[f"{node}_{'_'.join(suffixes)}"] = {"removed": len(removed), "persons_before": before, "persons_after": after,
                                              "script": "scripts/remove_crosswalk_pedestrians.py"}
        MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{node} {suffixes} 횡단보도 보행자 {len(removed)}명 제거: {before} -> {after}명")


if __name__ == "__main__":
    main()
