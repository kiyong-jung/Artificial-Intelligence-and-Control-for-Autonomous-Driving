"""서교동사거리(J42) 좌회전 다섯 연결을 앞뒤 차로에 접하는 곡선으로 바꾼다(3.16.0 -> 3.17.0).

    python scripts/smooth_j42_left_turns.py

잔다리로 R94 -> 양화로 R54a(버스 경로)와 맞은편 -R92.163 -> R53 좌회전은 교차로 안 차로가 회전을 다 마치기 전에 다음 도로에
닿아, 이음매에서 방향이 25~30도 꺾였다. 차는 한 계산 스텝에 그만큼 돌고, 12 m 버스는 R54a 시작에서 차체가 옆 차로로 쓸렸다.
연결마다 앞 차로 끝과 뒤 차로 시작을 잇고 두 끝 방향에 모두 접하는 2차 베지어 곡선을 연결 모양(shape)으로 준다. 곡선이
급한 곳의 반지름으로 속도도 정한다: sqrt(A_LAT * 최소 반지름). netconvert는 평균 반지름으로 속도를 정해 이 회전을
12.8~13.9 m/s로 두었다.

netconvert는 교차로를 다시 계산하며 J42와 무관한 횡단보도 여섯 개의 윤곽과 3.12.0의 J1 c2 수정까지 바꾼다
(scripts/remove_fringe_turnarounds.py). 그래서 netconvert 결과에서 J42의 교차로·교차로 안 도로·연결만 가져와 원래 도로망에
끼운다. 신호 연결 번호가 하나라도 바뀌면 멈춘다(signals.add.xml은 그대로 쓴다). 줄바꿈(CRLF)은 원래 파일을 따른다.
"""
from __future__ import annotations

import io
import json
import math
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import sumolib

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.patch_network import tls_links  # noqa: E402
from scripts.patch_network import digest  # noqa: E402
from scripts.prune_network import OPTIONS  # noqa: E402

SOURCE = BASE / "maps" / "hapjeong" / "source"
MAP_JSON = BASE / "maps" / "hapjeong" / "map.json"
NEW_VERSION = "3.17.0"
NODE = "J42"
TURNS = [("R94", "1", "R54a", "1"), ("R94", "2", "R54a", "2"), ("R94", "2", "R54a", "4"),
         ("-R92.163", "2", "R53", "2"), ("-R92.163", "2", "R53", "4")]
A_LAT = 5.5      # netconvert --junctions.limit-turn-speed 기본값(m/s²)
POINTS = 13      # --junctions.internal-link-detail 12와 같은 점 수


def _pts(text: str) -> list[tuple[float, float]]:
    return [tuple(map(float, p.split(",")[:2])) for p in text.split()]


def _unit(p, q):
    d = math.dist(p, q)
    return (q[0] - p[0]) / d, (q[1] - p[1]) / d


def tangent_curve(a, a_dir, b, b_dir):
    """a에서 a_dir로 나가 b에 b_dir로 닿는 2차 베지어 점들과 가장 작은 곡률 반지름(m)."""
    det = a_dir[0] * b_dir[1] - a_dir[1] * b_dir[0]
    rx, ry = b[0] - a[0], b[1] - a[1]
    s = (rx * b_dir[1] - ry * b_dir[0]) / det
    u = (a_dir[0] * ry - a_dir[1] * rx) / det
    if s <= 0 or u <= 0:
        raise ValueError(f"두 접선이 곡선 앞뒤에서 만나지 않는다(s={s:.1f}, u={u:.1f})")
    c = (a[0] + s * a_dir[0], a[1] + s * a_dir[1])

    def at(t):
        return ((1 - t) ** 2 * a[0] + 2 * t * (1 - t) * c[0] + t * t * b[0],
                (1 - t) ** 2 * a[1] + 2 * t * (1 - t) * c[1] + t * t * b[1])
    d2 = (2 * (a[0] - 2 * c[0] + b[0]), 2 * (a[1] - 2 * c[1] + b[1]))
    radius = math.inf
    for i in range(201):
        t = i / 200
        d1 = (2 * (1 - t) * (c[0] - a[0]) + 2 * t * (b[0] - c[0]), 2 * (1 - t) * (c[1] - a[1]) + 2 * t * (b[1] - c[1]))
        cross = abs(d1[0] * d2[1] - d1[1] * d2[0])
        if cross:
            radius = min(radius, math.hypot(*d1) ** 3 / cross)
    return [at(i / (POINTS - 1)) for i in range(POINTS)], radius


def _belongs(el: ET.Element) -> bool:
    """J42의 교차로, 교차로 안 도로·교차로, 그리고 J42를 지나는 연결."""
    inner = f":{NODE}_"
    if el.tag in ("edge", "junction"):
        return el.get("id") == NODE or el.get("id").startswith(inner)
    if el.tag == "connection":
        return any((el.get(k) or "").startswith(inner) for k in ("from", "to", "via")) or el.get("tl") == NODE
    return el.tag == "tlLogic" and el.get("id") == NODE


def _key(el: ET.Element) -> tuple:
    if el.tag == "connection":
        return el.tag, el.get("from"), el.get("to"), el.get("fromLane"), el.get("toLane")
    return el.tag, el.get("id") or ""


def splice(old: ET.Element, new: ET.Element) -> int:
    """old에서 J42 요소만 new의 것으로 바꾼다. 바꾼 요소 수를 돌려준다."""
    fresh = {_key(e): e for e in new if _belongs(e)}
    stale = [e for e in old if _belongs(e)]
    if set(fresh) != {_key(e) for e in stale}:
        raise ValueError("netconvert가 J42 요소를 더하거나 뺐다")
    changed = 0
    for i, el in enumerate(list(old)):
        if _belongs(el):
            repl = fresh[_key(el)]
            repl.tail = el.tail
            changed += ET.tostring(repl) != ET.tostring(el)
            old.remove(el)
            old.insert(i, repl)
    return changed


def main() -> None:
    # 이미 적용한 지도에 다시 돌리면 안 된다. 3.18.0이 J42 정지선을 옮긴 뒤로는 아래의 "모양이 같은가" 비교가 어긋나,
    # 다시 돌리면 곡선을 새로 만들고 지도 판을 3.17.0으로 되돌렸다(sha256도 바뀜).
    applied = json.loads((SOURCE.parent / "map.json").read_text(encoding="utf-8"))["network_source"].get("patches", {})
    if "j42_tangent_left_turns" in applied:
        print("이미 적용했다(map.json의 patches.j42_tangent_left_turns): 바꾸지 않는다")
        return
    net_path = SOURCE / "network.net.xml"
    original = net_path.read_bytes()
    tree = ET.parse(net_path)
    root = tree.getroot()
    shapes = {lane.get("id"): _pts(lane.get("shape")) for lane in root.iter("lane")}
    con = ET.Element("connections")
    plan = []
    for f, fl, t, tl in TURNS:
        a, b = shapes[f"{f}_{fl}"], shapes[f"{t}_{tl}"]
        curve, radius = tangent_curve(a[-1], _unit(a[-2], a[-1]), b[0], _unit(b[0], b[1]))
        speed = math.floor(math.sqrt(A_LAT * radius) * 10) / 10
        shape = " ".join(f"{x:.3f},{y:.3f}" for x, y in curve)
        current = next(c for c in root.iter("connection") if (c.get("from"), c.get("fromLane"), c.get("to"),
                                                               c.get("toLane")) == (f, fl, t, tl))
        if current.get("shape") == shape:
            continue
        ET.SubElement(con, "connection", {"from": f, "fromLane": fl, "to": t, "toLane": tl, "shape": shape,
                                          "speed": f"{speed:.1f}"})
        plan.append((f"{f}_{fl} -> {t}_{tl}", round(radius, 1), speed))
    if not plan:
        print("이미 곡선이다: 바꾸지 않는다")
        return
    with tempfile.TemporaryDirectory(prefix="hapjeong_j42_turns_") as name:
        tmp = Path(name)
        ET.ElementTree(con).write(tmp / "turns.con.xml", encoding="utf-8")
        out = tmp / "network.net.xml"
        done = subprocess.run([sumolib.checkBinary("netconvert"), "--sumo-net-file", str(net_path),
                               "--connection-files", str(tmp / "turns.con.xml"), *OPTIONS, "--output-file", str(out)],
                              capture_output=True, text=True, timeout=300)
        if done.returncode:
            raise RuntimeError("netconvert 실패: " + done.stderr[-3000:])
        new_root = ET.parse(out).getroot()
        if tls_links(out) != tls_links(net_path):
            raise ValueError("신호 연결 번호가 바뀌었다")
    changed = splice(root, new_root)
    buffer = io.BytesIO()
    tree.write(buffer, encoding="utf-8", xml_declaration=True)
    data = buffer.getvalue()
    if b"\r\n" in original:
        data = data.replace(b"\n", b"\r\n")
    net_path.write_bytes(data)
    definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    definition["version"] = NEW_VERSION
    definition["network_source"]["sha256"] = digest(net_path)
    definition["network_source"]["patches"]["j42_tangent_left_turns"] = {
        "turns": [{"connection": c, "min_radius_m": r, "speed_mps": v} for c, r, v in plan],
        "a_lat": A_LAT, "script": "scripts/smooth_j42_left_turns.py"}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"J42 좌회전 {len(plan)}개를 곡선으로: {plan}. J42 요소 {changed}개 바뀜. map {NEW_VERSION}")


if __name__ == "__main__":
    main()
