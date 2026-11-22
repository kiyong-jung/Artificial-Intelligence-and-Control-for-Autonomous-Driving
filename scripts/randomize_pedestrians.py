"""보행자 경로 파일의 고정 출발 시각을 무작위 도착으로 바꾼다.

    python scripts/randomize_pedestrians.py

`<person depart="...">`는 seed가 달라도 같은 사람이 같은 시각에 나온다. 같은 길을 걷는 사람들을 `<personFlow>` 하나로
묶고 `period="exp(시간당 인원 / 3600)"`을 적는다. 그러면 SUMO가 seed에 따라 도착 간격을 지수 분포에서 뽑는다(포아송 도착).
걷는 길과 길마다의 시간당 평균 인원은 그대로다. scripts/build_hapjeong.py의 build_pedestrians도 이 함수를 쓴다.
"""
from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

FOLDER = Path(__file__).resolve().parents[1] / "maps" / "hapjeong"
ROUTES = FOLDER / "source" / "pedestrians.rou.xml"
ET.register_namespace("xsi", "http://www.w3.org/2001/XMLSchema-instance")


def person_flows(persons: list[ET.Element], end: float) -> list[ET.Element]:
    """같은 길을 걷는 사람들을 personFlow 하나로 묶는다. 흐름 id는 그 길을 처음 걷는 사람의 id다."""
    groups: dict[str, list[ET.Element]] = {}
    for person in persons:
        walks = person.findall("walk")
        if len(walks) != 1 or len(person) != 1 or set(person.attrib) != {"id", "depart"}:
            raise ValueError(f"보행자 {person.get('id')}는 걷기 한 번만 있는 사람이 아니다")
        groups.setdefault(walks[0].get("edges"), []).append(person)
    flows = []
    for edges, people in groups.items():
        flow = ET.Element("personFlow", id=people[0].get("id"), begin="0", end=f"{end:g}",
                          period=f"exp({len(people) / end:.8f})")
        ET.SubElement(flow, "walk", edges=edges)
        flows.append(flow)
    return flows


def randomize(root: ET.Element, end: float) -> int:
    """root의 person을 모두 personFlow로 바꾼다. 반환: 바꾼 사람 수."""
    persons = root.findall("person")
    for person in persons:
        root.remove(person)
    root.extend(person_flows(persons, end))
    ET.indent(root, space="    ")
    return len(persons)


def main() -> None:
    definition = json.loads((FOLDER / "map.json").read_text(encoding="utf-8"))
    record = definition["network_source"]["pedestrians"]
    raw = ROUTES.read_bytes()
    root = ET.fromstring(raw)
    count = randomize(root, float(record["end"]))
    if not count:
        raise SystemExit("고정 출발 시각 보행자가 없다(이미 바꿨다)")
    text = ET.tostring(root, encoding="unicode")
    newline = "\r\n" if b"\r\n" in raw else "\n"   # 커밋된 파일은 CRLF다
    ROUTES.write_bytes(("<?xml version='1.0' encoding='utf-8'?>\n" + text + "\n").replace("\n", newline).encode("utf-8"))
    record["arrivals"] = "poisson"
    (FOLDER / "map.json").write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"사람 {count}명 → personFlow {len(root.findall('personFlow'))}개")


if __name__ == "__main__":
    main()
