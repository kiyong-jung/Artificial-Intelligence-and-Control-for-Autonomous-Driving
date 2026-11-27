"""지도 경계 교차로 다섯 곳의 유턴 연결을 없앤다(3.15.0 -> 3.16.0).

    python scripts/remove_fringe_turnarounds.py

경계 J99, J100, J108, J109, J110은 지도 밖으로 나가는 도로(R126~R130)가 끝나는 점이다. 이 점에는 나가는 도로를 같은 도로의
반대 방향으로 되돌리는 유턴 연결 하나만 있었다. 배경 차는 들어갈 때 최근 교통으로 고친 도로별 시간으로 길을 고르므로, 안쪽 길이
막히면 지도 끝까지 가서 유턴해 돌아오는 길을 골랐다(3.15.0의 일곱 실행에서 J110 두 대). 실제 운전자는 지도 밖에서 돈다.
연결을 없애면 이 다섯 점은 다른 경계처럼 dead_end가 되고, 차는 여기서 도착만 한다.

netconvert로 연결을 지우면 교차로를 다시 계산하며 횡단보도 여섯 개의 모양이 바뀌고 3.12.0의 J1 c2 시작점 수정이 되돌아간다.
그래서 도로망 파일에서 직접 지운다. 지우는 것은 교차로마다 연결 하나, 교차로 안 도로 하나와 그 연결 하나다. 교차로는
type="dead_end", intLanes=""로 바꾸고 request를 지운다. 이 결과는 같은 연결을 netconvert로 지웠을 때의 다섯 교차로와 같다.
신호 교차로가 아니라 signals.add.xml은 그대로다. 줄바꿈(CRLF)은 원래 파일을 따른다.
"""
from __future__ import annotations

import io
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.patch_network import digest  # noqa: E402

SOURCE = BASE / "maps" / "hapjeong" / "source"
MAP_JSON = BASE / "maps" / "hapjeong" / "map.json"
NEW_VERSION = "3.16.0"
FRINGE = {"J99": "R126", "J100": "R127", "J108": "R128", "J109": "R129", "J110": "R130"}   # 경계 점 -> 그 점에서 끝나는 도로


def remove_turnarounds(root: ET.Element) -> list[str]:
    """경계 점의 유턴 연결을 지우고 지운 점 id를 돌려준다. 이미 없으면 건너뛴다."""
    junctions = {j.get("id"): j for j in root.iter("junction")}
    edges = {e.get("id"): e for e in root.iter("edge")}
    connections = list(root.iter("connection"))
    removed = []
    for node, edge in FRINGE.items():
        turn = [c for c in connections if c.get("from") == edge and c.get("to") == "-" + edge]
        if not turn:
            continue
        (turn,) = turn
        internal = turn.get("via").rsplit("_", 1)[0]
        junction = junctions[node]
        others = [c for c in connections if c.get("from") == edge and c is not turn and not c.get("to").startswith(":")]
        if others or junction.get("intLanes") != turn.get("via") or len(junction.findall("request")) != 1:
            raise ValueError(f"{node}에 유턴 말고 다른 연결이 있다: 경계 점이 아니다")
        root.remove(turn)
        root.remove(edges[internal])
        for c in connections:
            if c.get("from") == internal:
                root.remove(c)
        for request in junction.findall("request"):
            junction.remove(request)
        junction.text = None
        junction.set("type", "dead_end")
        junction.set("intLanes", "")
        removed.append(node)
    return removed


def main() -> None:
    net_path = SOURCE / "network.net.xml"
    original = net_path.read_bytes()
    tree = ET.parse(net_path)
    removed = remove_turnarounds(tree.getroot())
    if not removed:
        print("이미 없다: 바꾸지 않는다")
        return
    buffer = io.BytesIO()
    tree.write(buffer, encoding="utf-8", xml_declaration=True)
    data = buffer.getvalue()
    if b"\r\n" in original:
        data = data.replace(b"\n", b"\r\n")
    net_path.write_bytes(data)
    definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    definition["version"] = NEW_VERSION
    definition["network_source"]["sha256"] = digest(net_path)
    definition["network_source"]["patches"]["fringe_no_turnarounds"] = {
        "removed": [f"{FRINGE[node]} -> -{FRINGE[node]} ({node})" for node in removed],
        "script": "scripts/remove_fringe_turnarounds.py"}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"경계 유턴 {len(removed)}개 제거: {removed}. map {NEW_VERSION}")


if __name__ == "__main__":
    main()
