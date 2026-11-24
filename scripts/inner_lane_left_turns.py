"""버스 경로의 좌회전을 가장 왼쪽 차로에서만 하게 한다(3.20.0 -> 3.21.0 J82, 3.21.0 -> 3.22.0 J83·J42·J1).

    python scripts/inner_lane_left_turns.py

원본 OSM에는 차로별 방향(turn:lanes)이 없다. 사용자 결정으로 실제 도로처럼 좌회전은 가장 왼쪽 차로에서만 하고, 그 차로는
좌회전 뒤 안쪽 차로로 나온다. TURNS의 연결은 가로변 쪽 차로의 좌회전이고, 그것을 지우면 남은 좌회전은 가장 왼쪽 차로의
것뿐이다.

- 상수역 사거리(J82, 3.21.0). 독막로 R111b(차도 1~3번 차로)의 2번 차로 좌회전(R112 1번 차로로)을 지운다. 3번 차로만
  R112 2번 차로로 좌회전한다. R111b만 녹색인 단계(35초)에 돌아 다른 방향과 겹치지 않는다.
- 삼거리포차 앞(J83, 3.22.0). 와우산로 R112.221(차도 1~2번 차로)의 1번 차로 좌회전(R113 1번 차로로)을 지운다. 1번 차로는
  직진(R130)만, 2번 차로는 R113 2번 차로로 좌회전한다.
- 서교동사거리(J42, 3.22.0). 잔다리로 R94(차도 1~2번 차로)의 1번 차로 좌회전(R54a 1번 차로로)을 지운다. 1번 차로는
  직진(R92)만, 2번 차로는 R54a 2번·4번 차로로 좌회전한다.
- 합정역 사거리(J1, 3.22.0). 양화로 R42(차도 1~4번 차로)의 3번 차로 좌회전(R68 1번 차로로, 3.7.0에서 더한 것)을 지운다.
  3번 차로는 직진(R40)만, 4번 차로는 R68 2번 차로로 좌회전한다. 안쪽에서 안쪽으로 나가고, R68 2번 차로는 13 m 뒤 J4에서
  R69 3번 차로로 이어져 버스 경로 끝(R68)까지 다시 옮길 일이 없다. 지운 18번 뒤의 신호 번호만 당겨지고, 양보 녹색 두
  연결(0번 B2 -> R40, 10번 B1 -> R41)은 번호가 그대로다.

3.4.0이 J83·J42 좌회전을 가로변 차로에서도 되게 한 것은 가로변 차로의 규칙 버스가 좌회전 줄에 끼어들지 못하고 300초를
서 있었기 때문이다(build_hapjeong.py REPLACED_CONNECTIONS). 지금 버스는 정책이 차로를 고르고, 틈이 없으면 줄여 옆 차를
먼저 보낸다.

커밋된 도로망에 연결 하나만 빼는 방식은 remove_j42_right_turn.py와 같고, netconvert가 다른 교차로까지 다시 계산하므로
smooth_j42_left_turns.py처럼 결과에서 그 교차로의 교차로·교차로 안 도로·연결만 가져와 원래 도로망에 끼운다. 연결 하나가
빠지면 그 교차로의 교차로 안 도로와 신호 연결 번호가 당겨지므로 그 집합을 통째로 바꾸고, 그 신호 프로그램(도로망 안과
signals.add.xml)의 상태 문자열에서 지운 연결의 글자를 뺀다. 다른 신호 교차로의 연결 번호가 바뀌면 멈춘다.

연결을 지우면 netconvert가 남은 연결의 모양을 새로 그릴 수 있다(J82에서는 버스가 도는 안쪽 좌회전이 더 급해져 곡선
속도 제한 최솟값이 5.1 -> 4.6 m/s가 됐다). 바꾸려는 것은 연결 하나뿐이므로, 모양이 바뀐 연결에는 옛 모양과 속도를 그대로
주어 netconvert를 한 번 더 돌리고, 남은 모든 연결의 모양·속도가 옛 것과 같은지 확인한다.
줄바꿈(CRLF)은 원래 파일을 따른다. 다시 돌리면 아무것도 바꾸지 않는다.
"""
from __future__ import annotations

import io
import json
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import sumolib
from sumolib import geomhelper

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.add_j1_left_lane import tls_links  # noqa: E402
from scripts.patch_network import digest  # noqa: E402
from scripts.prune_network import OPTIONS  # noqa: E402

SOURCE = BASE / "maps" / "hapjeong" / "source"
MAP_JSON = BASE / "maps" / "hapjeong" / "map.json"
KEYS = ("from", "fromLane", "to", "toLane")
# (교차로, 지우는 연결(from, fromLane, to, toLane), map.json 패치 이름, 지운 도로망 판). 적힌 순서대로 지운다.
TURNS = (
    ("J82", ("R111b", "2", "R112", "1"), "j82_inner_lane_left_turn", "3.21.0"),
    ("J83", ("R112.221", "1", "R113", "1"), "j83_inner_lane_left_turn", "3.22.0"),
    ("J42", ("R94", "1", "R54a", "1"), "j42_inner_lane_left_turn", "3.22.0"),
    ("J1", ("R42", "3", "R68", "1"), "j1_inner_lane_left_turn", "3.22.0"),
)


def _belongs(el: ET.Element, node: str) -> bool:
    """node의 교차로, 교차로 안 도로·교차로, 그리고 node를 지나는 연결."""
    inner = f":{node}_"
    if el.tag in ("edge", "junction"):
        return el.get("id") == node or el.get("id").startswith(inner)
    return el.tag == "connection" and (
        any((el.get(k) or "").startswith(inner) for k in ("from", "to", "via")) or el.get("tl") == node)


def turn_paths(root: ET.Element, node: str) -> dict[tuple, tuple[list[str], str]]:
    """node를 지나는 차 연결마다 (교차로 안 차로들을 이은 점들, 첫 차로의 속도)."""
    inner = f":{node}_"
    lanes = {lane.get("id"): lane for edge in root.iter("edge") if edge.get("id").startswith(inner) for lane in edge}
    onward = {f"{c.get('from')}_{c.get('fromLane')}": c.get("via") for c in root.iter("connection")
              if c.get("from").startswith(inner) and c.get("via")}
    paths = {}
    for c in root.iter("connection"):
        via = c.get("via") or ""
        if c.get("from").startswith(":") or not via.startswith(inner):
            continue
        points: list[str] = []
        while via:
            points += [p for p in lanes[via].get("shape").split() if not points or p != points[-1]]
            via = onward.get(via)
        paths[tuple(c.get(k) for k in KEYS)] = (points, lanes[c.get("via")].get("speed"))
    return paths


def same_path(old: tuple[list[str], str], new: tuple[list[str], str]) -> bool:
    """속도가 같고 같은 선인지. 대기 위치(교차로 안 교차점)가 옮겨지면 netconvert가 선 위에 나누는 점을 하나 끼운다."""
    (old_points, old_speed), (new_points, new_speed) = old, new
    line = [tuple(map(float, p.split(","))) for p in old_points]
    extra = [tuple(map(float, p.split(","))) for p in new_points if p not in old_points]
    return (old_speed == new_speed and [p for p in new_points if p in old_points] == old_points
            and all(geomhelper.distancePointToPolygon(p, line) < 0.01 for p in extra))


def _netconvert(net_path: Path, tmp: Path, removed: tuple | None, pins: dict[tuple, tuple[list[str], str]]
                ) -> tuple[ET.Element, list[str]]:
    """removed를 지우고(None이면 그대로) pins의 연결에 모양·속도를 준 netconvert 결과와 경고."""
    con = ET.Element("connections")
    if removed:
        ET.SubElement(con, "delete", dict(zip(KEYS, removed)))
    for key, (points, speed) in pins.items():
        ET.SubElement(con, "connection", {**dict(zip(KEYS, key)), "shape": " ".join(points), "speed": speed})
    ET.ElementTree(con).write(tmp / "turn.con.xml", encoding="utf-8")
    out = tmp / "network.net.xml"
    done = subprocess.run([sumolib.checkBinary("netconvert"), "--sumo-net-file", str(net_path),
                           "--connection-files", str(tmp / "turn.con.xml"), *OPTIONS, "--output-file", str(out)],
                          capture_output=True, text=True, timeout=300)
    if done.returncode:
        raise RuntimeError("netconvert 실패: " + done.stderr[-3000:])
    return ET.parse(out).getroot(), [line for line in done.stderr.splitlines() if line.startswith("Warning")]


def splice(old: ET.Element, new: ET.Element, node: str) -> tuple[int, int]:
    """old의 node 요소를 태그마다 new의 것으로 통째로 바꾼다(신호 프로그램은 그대로 둔다). 연결이 빠져 교차로 안 도로
    id가 당겨지므로 하나씩 맞바꿀 수 없다. 새 요소는 같은 태그의 첫 옛 요소 자리에 netconvert 순서대로 넣는다."""
    removed = added = 0
    for tag in ("edge", "junction", "connection"):
        stale = [e for e in old if e.tag == tag and _belongs(e, node)]
        fresh = [e for e in new if e.tag == tag and _belongs(e, node)]
        at, tail = list(old).index(stale[0]), stale[0].tail
        for el in stale:
            old.remove(el)
        for k, el in enumerate(fresh):
            el.tail = tail
            old.insert(at + k, el)
        removed, added = removed + len(stale), added + len(fresh)
    return removed, added


def drop_state_char(text: str, node: str, index: int, length: int) -> str:
    """node 신호 프로그램 블록 안의 길이 length인 상태 문자열에서 index번 글자를 뺀다(서식과 줄바꿈은 그대로)."""
    def fix_state(match: re.Match) -> str:
        state = match.group(1)
        if len(state) != length:
            raise ValueError(f"{node} 상태 길이 {len(state)}가 연결 수 {length}와 다르다")
        return f'state="{state[:index]}{state[index + 1:]}"'
    fixed, count = re.subn(rf'<tlLogic id="{node}".*?</tlLogic>',
                           lambda m: re.sub(r'state="([^"]*)"', fix_state, m.group(0)), text, flags=re.S)
    if count != 1:
        raise ValueError(f"{node} 신호 프로그램이 {count}개다")
    return fixed


def remove_turn(node: str, removed: tuple, patch: str, version: str) -> None:
    """removed 연결 하나를 지우고 도로망, 신호, map.json을 고친다. 이미 없으면 아무것도 하지 않는다."""
    net_path, signals_path = SOURCE / "network.net.xml", SOURCE / "signals.add.xml"
    old_links = tls_links(net_path)
    if removed not in old_links[node].values():
        print(f"{node}: 이미 없다({'·'.join(removed)})")
        return
    removed_index = next(i for i, key in old_links[node].items() if key == removed)
    original = net_path.read_bytes()
    tree = ET.parse(net_path)
    root = tree.getroot()
    kept = {key: path for key, path in turn_paths(root, node).items() if key != removed}
    with tempfile.TemporaryDirectory(prefix=f"hapjeong_{node.lower()}_") as name:
        tmp = Path(name)
        _, baseline_warnings = _netconvert(net_path, tmp, None, {})
        new_root, _ = _netconvert(net_path, tmp, removed, {})
        redrawn = sorted(key for key, path in turn_paths(new_root, node).items() if not same_path(kept[key], path))
        new_root, warnings = _netconvert(net_path, tmp, removed, {key: kept[key] for key in redrawn})
        new_links = tls_links(tmp / "network.net.xml")
    paths = turn_paths(new_root, node)
    if set(paths) != set(kept) or not all(same_path(kept[key], paths[key]) for key in kept):
        raise ValueError(f"{node} 남은 연결의 모양이나 속도가 옛 것과 다르다")
    for tls in old_links:
        if tls != node and old_links[tls] != new_links.get(tls):
            raise ValueError(f"{tls}의 신호 연결이 바뀌었다")
    if [key for _i, key in sorted(new_links[node].items())] != [key for i, key in sorted(old_links[node].items())
                                                                 if i != removed_index]:
        raise ValueError(f"{node} 연결이 {removed} 하나만 빠진 순서가 아니다")
    new_warnings = sorted(set(warnings) - set(baseline_warnings))

    count, added = splice(root, new_root, node)
    buffer = io.BytesIO()
    tree.write(buffer, encoding="utf-8", xml_declaration=True)
    text = drop_state_char(buffer.getvalue().decode("utf-8"), node, removed_index, len(old_links[node]))
    if b"\r\n" in original:
        text = text.replace("\n", "\r\n")
    net_path.write_bytes(text.encode("utf-8"))
    signals = signals_path.read_bytes().decode("utf-8")
    signals_path.write_bytes(drop_state_char(signals, node, removed_index, len(old_links[node])).encode("utf-8"))

    definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    definition["version"] = version
    definition["network_source"]["sha256"] = digest(net_path)
    definition["network_source"]["patches"][patch] = {
        "removed": list(removed), "link_index": removed_index, "kept_shapes": [list(key) for key in redrawn],
        "script": "scripts/inner_lane_left_turns.py"}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{node} 좌회전 연결 {'·'.join(removed)}(신호 {removed_index}번) 제거, 옛 모양을 준 연결 {redrawn}. "
          f"{node} 요소 {count}개 -> {added}개. netconvert 경고 {len(warnings)}개(손대기 전 {len(baseline_warnings)}개, "
          f"새 경고 {new_warnings}). map {version}")


def main() -> None:
    for node, removed, patch, version in TURNS:
        remove_turn(node, removed, patch, version)


if __name__ == "__main__":
    main()
