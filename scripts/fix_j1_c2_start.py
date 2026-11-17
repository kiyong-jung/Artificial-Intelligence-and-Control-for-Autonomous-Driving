"""J1 횡단보도 c2의 시작점을 w2 보행자 섬에 붙인다(3.11.0 -> 3.12.0).

    python scripts/fix_j1_c2_start.py

c2(R40 B1 R45, 길이 29 m)는 R40과 R45가 갈라지는 방향이 달라 자동으로 놓인 위치에서 w2 쪽 시작선이 w2 가장자리에 모서리 한 점으로만
닿았다(중심선 시작점이 w2에서 2.0 m 떨어짐. 다른 횡단보도의 끝은 0.1~0.35 m). 보행자 섬 w2의 도로 쪽 가장자리(w2 모양의 마지막
점 -> 첫 점) 가운데로 c2의 시작점을 옮겨 시작선이 w2 가장자리에 놓이게 한다. 끝점(w1)은 그대로다.

도로망 파일의 좌표만 고친다(연결 구조, 신호, 보행자 경로는 그대로). 고치는 것: c2 차로 모양(shape)·길이(length)·윤곽(outlineShape).
이미 붙어 있으면(시작점이 w2에서 0.5 m 안) 아무것도 바꾸지 않는다.
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.patch_network import digest  # noqa: E402

SOURCE = BASE / "maps" / "hapjeong" / "source"
MAP_JSON = BASE / "maps" / "hapjeong" / "map.json"
NEW_VERSION = "3.12.0"
CROSSING, AREA = ":J1_c2", ":J1_w2"


def _pts(text: str) -> list[tuple[float, float]]:
    return [tuple(map(float, p.split(","))) for p in text.split()]


def _fmt(points) -> str:
    return " ".join(f"{x:.3f},{y:.3f}" for x, y in points)


def _dist_to_polygon(p, poly) -> float:
    best = float("inf")
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        dx, dy = bx - ax, by - ay
        n = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / n)) if n else 0.0
        best = min(best, math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy))
    return best


def main() -> None:
    net_path = SOURCE / "network.net.xml"
    tree = ET.parse(net_path)
    edges = {e.get("id"): e for e in tree.getroot().iter("edge")}
    lane = edges[CROSSING].find("lane")
    area = _pts(edges[AREA].find("lane").get("shape"))
    start, end = _pts(lane.get("shape"))
    if _dist_to_polygon(start, area) <= 0.5:
        print("이미 붙어 있다: 바꾸지 않는다")
        return
    # w2 모양의 마지막 점과 첫 점을 잇는 가장자리가 도로 쪽이다. 그 가운데로 시작점을 옮긴다.
    (qx0, qy0), (qx1, qy1) = area[-1], area[0]
    new_start = ((qx0 + qx1) / 2.0, (qy0 + qy1) / 2.0)
    length = math.hypot(end[0] - new_start[0], end[1] - new_start[1])
    ux, uy = (end[0] - new_start[0]) / length, (end[1] - new_start[1]) / length
    half = float(lane.get("width")) / 2.0
    nx, ny = -uy * half, ux * half
    outline = [(new_start[0] + nx, new_start[1] + ny), (end[0] + nx, end[1] + ny),
               (end[0] - nx, end[1] - ny), (new_start[0] - nx, new_start[1] - ny)]
    lane.set("shape", _fmt([new_start, end]))
    lane.set("length", f"{length:.3f}")
    lane.set("outlineShape", _fmt(outline))
    tree.write(net_path, encoding="utf-8", xml_declaration=True)
    definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    definition["version"] = NEW_VERSION
    definition["network_source"]["sha256"] = digest(net_path)
    definition["network_source"]["patches"]["j1_c2_start_on_w2"] = {
        "moved_start": [round(new_start[0], 3), round(new_start[1], 3)], "script": "scripts/fix_j1_c2_start.py"}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"c2 시작점 {start} -> {new_start}, 길이 {length:.1f} m. map {NEW_VERSION}")


if __name__ == "__main__":
    main()
