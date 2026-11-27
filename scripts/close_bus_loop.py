"""버스 경로 bus_loop를 합정역 사거리(J1) 좌회전 뒤 독막로 R68까지 늘려 순환을 닫는다(3.18.0 -> 3.19.0).

    python scripts/close_bus_loop.py

지금까지 bus_loop는 양화로 R42(16 m)의 J1 정지선에서 끝났다. 버스는 정지선 앞에서 에피소드를 마쳐, J1 좌회전과 그 앞의
연속 차로 변경(합정역 정류장 R54a_1에서 R42 3·4번 좌회전 차로로)을 하지 않았다. R68(13 m)을 붙이면 경로가 출발 도로 R69 바로
앞 J4(신호 없음)에서 끝나 한 바퀴가 닫힌다. 경유 도로는 scripts/build_hapjeong.py의 ROUTES이고, 경로와 길이는 빌드와 같은
방법(build_road.shortest_route)으로 다시 계산한다. 다시 돌리면 아무것도 바꾸지 않는다.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import sumolib

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.build_hapjeong import EGO_ROUTE, ROUTES  # noqa: E402
from scripts.build_road import shortest_route  # noqa: E402

FOLDER = BASE / "maps" / "hapjeong"
NEW_VERSION = "3.19.0"


def main() -> None:
    path = FOLDER / "map.json"
    definition = json.loads(path.read_text(encoding="utf-8"))
    net = sumolib.net.readNet(str(FOLDER / definition["network_source"]["path"]))
    description, anchors = ROUTES[EGO_ROUTE]
    edges = shortest_route(net, anchors, EGO_ROUTE)
    if definition["routes"][EGO_ROUTE] == edges:
        print("이미 닫혔다: 바꾸지 않는다")
        return
    before = definition["routes"][EGO_ROUTE]
    definition["routes"][EGO_ROUTE] = edges
    definition["route_design"][EGO_ROUTE].update(
        description=description, length_m=round(sum(net.getEdge(edge).getLength() for edge in edges), 2))
    definition["version"] = NEW_VERSION
    path.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{EGO_ROUTE}: {before[-1]}에서 끝나던 경로를 {edges[len(before) - 1:]}로, "
          f"길이 {definition['route_design'][EGO_ROUTE]['length_m']} m. map {NEW_VERSION}")


if __name__ == "__main__":
    main()
