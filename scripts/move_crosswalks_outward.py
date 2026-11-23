"""서교동사거리·상수역 사거리 우회전 모서리의 횡단보도 네 개를 바깥쪽으로 0.5 m 옮기고 그 앞 정지선을 1 m 물린다
(3.17.0 -> 3.18.0).

    python scripts/move_crosswalks_outward.py

서교동사거리(J42), 상수역 사거리(J82), 홍대입구역사거리(J96)의 다섯 우회전(`:J42_11_0`, `:J42_18_0`, `:J82_0_0`,
`:J82_4_0`, `:J96_7_0`)은 모서리에서 횡단보도 끝(보행자가 신호를 기다리는 자리)과 0.17~0.66 m 떨어져 지났다. 승용차 반폭이
0.9 m라 기다리는 사람이 도는 차체 안에 들었다. SUMO는 대기 구역의 사람과 교차로 안 차를 서로 피하게 하지 않으므로 겉모양
문제다. 사람이 차체 안에 든 자리는 모두 우회전이 들어오는 쪽 횡단보도의 끝이었다(J42 c2·c3, J82 c0·c1). 그 네 개를 바깥쪽
(교차로 중심에서 먼 쪽)으로 옮겨 기다리는 자리를 차 경로에서 떼어 낸다.

옮기는 거리와 고른 횡단보도는 잰 값이다(maps/hapjeong/README.md 3.18.0). 한 횡단보도는 한 모서리에서는 들어오는 쪽이고
옆 모서리 우회전에는 나가는 쪽이다. 나가는 쪽 횡단보도를 바깥으로 옮기면 netconvert가 그 우회전의 대기 위치(교차로 안
교차점)를 곡선 뒤쪽으로 옮겨, 기다리는 차의 앞 모서리가 횡단보도 시작 부분에 걸리고 SUMO가 보행자 충돌로 센다(1 m에서
:J42_11_0, :J82_8_0, :J42_0_0). 0.5 m에서는 이 충돌이 나오지 않았다. 버스 경로가 들어오는 R94(J42 c1)와 R111b(J82 c3)는
건드리지 않는다. 정지선을 물리면 J42 좌회전의 대기 위치가 바뀌어 차끼리 충돌하거나(:J42_8_1), J82 좌회전 안쪽 차로 앞에
짧은 토막이 붙어 버스 횡가속도가 올랐다. J96 c2는 옮기면 R127로 합류하는 우회전(:J96_14_0)의 충돌이 늘어 그대로 둔다.

횡단보도만 옮기면 바깥쪽이 들어오는 차로의 끝에 걸려, 정지선에 선 차가 횡단보도 위에 선다. 그래서 그 횡단보도가 건너는
들어오는 도로에 endOffset을 주어 정지선을 물린다. 정지선은 횡단보도 바깥 끝에서 STOP_LINE_GAP_M 더 뒤에 둔다(실제 정지선도
횡단보도 앞에 떨어져 있다). 나가는 도로는 그대로라, 차는 옮긴 횡단보도의 바깥쪽을 나가는 도로 위에서 지난다.

netconvert는 이미 있는 횡단보도의 모양을 바꾸지 못한다(같은 도로의 횡단보도를 다시 정의하면 오류). 그래서 한 번 지우고
(`discard`) 옮긴 모양으로 다시 만든다. netconvert는 교차로를 다시 계산하며 이 교차로들과 무관한 횡단보도까지 바꾸므로
(scripts/remove_fringe_turnarounds.py), 결과에서 옮긴 횡단보도의 교차로·교차로 안 도로·연결과 endOffset을 준 도로만 가져와
원래 도로망에 끼운다. 신호 연결 번호가 하나라도 바뀌면 멈춘다(signals.add.xml은 그대로 쓴다). 줄바꿈(CRLF)은 원래 파일을
따른다. 다시 돌리면 아무것도 바꾸지 않는다.
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

from scripts.add_j1_left_lane import tls_links  # noqa: E402
from scripts.patch_network import digest  # noqa: E402
from scripts.prune_network import OPTIONS  # noqa: E402

SOURCE = BASE / "maps" / "hapjeong" / "source"
MAP_JSON = BASE / "maps" / "hapjeong" / "map.json"
NEW_VERSION = "3.18.0"
# 횡단보도 -> 바깥쪽으로 옮길 거리(m). 우회전이 들어오는 쪽만 옮긴다. 더 옮기면 옆 우회전 차가 기다리는 자리에서 보행자와 부딪힌다.
SHIFT_M = {":J42_c2": 0.5, ":J42_c3": 0.5, ":J82_c0": 0.5, ":J82_c1": 0.5}
STOP_LINE_GAP_M = 0.5   # 옮긴 횡단보도 바깥 끝과 정지선 사이


def _node(crossing: str) -> str:
    return crossing[1:].rsplit("_c", 1)[0]


def plan(root: ET.Element, net) -> tuple[list[dict], dict[str, float]]:
    """옮긴 횡단보도 정의와, 정지선을 물릴 들어오는 도로 -> 거리."""
    crossings, offsets = [], {}
    for edge in root.iter("edge"):
        cid = edge.get("id")
        if cid not in SHIFT_M:
            continue
        lane = edge.find("lane")
        pts = [tuple(map(float, p.split(","))) for p in lane.get("shape").split()]
        a, b = pts[0], pts[-1]
        node = net.getNode(_node(cid))
        cx, cy = node.getCoord()
        nx, ny = -(b[1] - a[1]) / math.dist(a, b), (b[0] - a[0]) / math.dist(a, b)
        if ((a[0] + b[0]) / 2 - cx) * nx + ((a[1] + b[1]) / 2 - cy) * ny < 0:
            nx, ny = -nx, -ny
        shift = SHIFT_M[cid]
        crossed = edge.get("crossingEdges").split()
        crossings.append({"node": node.getID(), "edges": " ".join(crossed), "width": lane.get("width"),
                          "shape": " ".join(f"{x + shift * nx:.2f},{y + shift * ny:.2f}" for x, y in pts)})
        for eid in crossed:
            if net.getEdge(eid).getToNode() == node:
                offsets[eid] = max(offsets.get(eid, 0.0), shift + STOP_LINE_GAP_M)
    return crossings, offsets


def _belongs(el: ET.Element, nodes: set[str], edges: set[str]) -> bool:
    """세 교차로의 교차로·교차로 안 도로·교차로 안 교차로·연결, 그리고 정지선을 물린 도로.

    신호 프로그램(tlLogic)은 가져오지 않는다. 횡단보도를 지웠다 다시 만들면 netconvert가 J42의 기본 프로그램을 새로 짓는데,
    연결 번호가 같으니 원래 프로그램이 그대로 맞는다."""
    inner = tuple(f":{n}_" for n in nodes)
    if el.tag == "edge":
        return el.get("id") in edges or el.get("id").startswith(inner)
    if el.tag == "junction":
        return el.get("id") in nodes or el.get("id").startswith(inner)
    if el.tag == "connection":
        return any((el.get(k) or "").startswith(inner) for k in ("from", "to", "via")) or el.get("tl") in nodes
    return False


def _key(el: ET.Element) -> tuple:
    if el.tag == "connection":
        return el.tag, el.get("from"), el.get("to"), el.get("fromLane"), el.get("toLane")
    return el.tag, el.get("id") or ""


def splice(old: ET.Element, new: ET.Element, nodes: set[str], edges: set[str]) -> int:
    """old에서 고칠 요소만 new의 것으로 바꾼다. 바꾼 요소 수를 돌려준다."""
    fresh = {_key(e): e for e in new if _belongs(e, nodes, edges)}
    stale = [e for e in old if _belongs(e, nodes, edges)]
    if set(fresh) != {_key(e) for e in stale}:
        raise ValueError(f"netconvert가 요소를 더하거나 뺐다: {sorted(set(fresh) ^ {_key(e) for e in stale})[:6]}")
    changed = 0
    for i, el in enumerate(list(old)):
        if _belongs(el, nodes, edges):
            repl = fresh[_key(el)]
            repl.tail = el.tail
            changed += ET.tostring(repl) != ET.tostring(el)
            old.remove(el)
            old.insert(i, repl)
    return changed


def _netconvert(source: Path, out: Path, *extra: str) -> None:
    done = subprocess.run([sumolib.checkBinary("netconvert"), "--sumo-net-file", str(source), *OPTIONS, *extra,
                           "--output-file", str(out)], capture_output=True, text=True, timeout=300)
    if done.returncode:
        raise RuntimeError("netconvert 실패: " + done.stderr[-3000:])


def main() -> None:
    net_path = SOURCE / "network.net.xml"
    original = net_path.read_bytes()
    tree = ET.parse(net_path)
    root = tree.getroot()
    net = sumolib.net.readNet(str(net_path), withInternal=True, withPedConnections=True)
    crossings, offsets = plan(root, net)
    done_already = {e.get("id"): float(e.find("lane").get("endOffset") or 0.0)
                    for e in root.iter("edge") if e.get("id") in offsets}
    if all(abs(done_already[e] - d) < 1e-6 for e, d in offsets.items()):
        print("이미 옮겼다: 바꾸지 않는다")
        return
    with tempfile.TemporaryDirectory(prefix="hapjeong_crosswalks_") as name:
        tmp = Path(name)
        drop, add, edg = ET.Element("connections"), ET.Element("connections"), ET.Element("edges")
        for c in crossings:
            ET.SubElement(drop, "crossing", node=c["node"], edges=c["edges"], discard="true")
            ET.SubElement(add, "crossing", **c)
        for eid, d in sorted(offsets.items()):
            ET.SubElement(edg, "edge", id=eid, endOffset=f"{d:.2f}")
        for tag, el in (("drop.con.xml", drop), ("add.con.xml", add), ("offset.edg.xml", edg)):
            ET.ElementTree(el).write(tmp / tag, encoding="utf-8")
        _netconvert(net_path, tmp / "dropped.net.xml", "--connection-files", str(tmp / "drop.con.xml"))
        out = tmp / "network.net.xml"
        _netconvert(tmp / "dropped.net.xml", out, "--connection-files", str(tmp / "add.con.xml"),
                    "--edge-files", str(tmp / "offset.edg.xml"))
        new_root = ET.parse(out).getroot()
        if tls_links(out) != tls_links(net_path):
            raise ValueError("신호 연결 번호가 바뀌었다")
    nodes = {c["node"] for c in crossings}
    changed = splice(root, new_root, nodes, set(offsets))
    buffer = io.BytesIO()
    tree.write(buffer, encoding="utf-8", xml_declaration=True)
    data = buffer.getvalue()
    if b"\r\n" in original:
        data = data.replace(b"\n", b"\r\n")
    net_path.write_bytes(data)
    definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    definition["version"] = NEW_VERSION
    definition["network_source"]["sha256"] = digest(net_path)
    # 정지선을 물린 도로는 그만큼 짧아진다(버스 경로의 R94, R111b).
    new_net = sumolib.net.readNet(str(net_path))
    for route_id, edges in definition["routes"].items():
        definition["route_design"][route_id]["length_m"] = round(sum(new_net.getEdge(e).getLength() for e in edges), 2)
    definition["network_source"]["patches"]["crosswalks_outward"] = {
        "shift_m": SHIFT_M, "stop_line_offset_m": dict(sorted(offsets.items())),
        "script": "scripts/move_crosswalks_outward.py"}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"횡단보도 {len(crossings)}개를 바깥쪽으로, 정지선 {len(offsets)}곳을 물림. 요소 {changed}개 바뀜. map {NEW_VERSION}")


if __name__ == "__main__":
    main()
