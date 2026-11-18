"""J96의 우회전 녹색이 충돌하는 횡단보도 녹색과 겹치지 않게 신호 프로그램을 고친다(3.12.0 -> 3.13.0).

    python scripts/separate_right_turns_from_crosswalks.py

원래 프로그램은 우회전 링크의 녹색 중 85%가 충돌하는 횡단보도의 보행자 녹색과 겹쳤다(주기의 31%). 우회전 차가 건너는
보행자에게 양보하지 못해 우회전 차량과 보행자가 부딪혔다(quiet·baseline·busy 측정 각 1건). 보행자 녹색과 겹치는 위상에서는 그 우회전
링크를 적색(r)으로 바꾼다(사용자 결정). 직진·좌회전 등 다른 차량 링크, 보행자 링크, 위상 길이는 그대로다.
우회전 링크는 도로망의 연결 방향(dir="r") 중 신호(tl)가 있고 횡단보도가 아닌 것이다. 충돌하는 횡단보도 링크는 도로망의 교차로
요청(request) foes 정보로 찾는다.

J42는 우회전 신호가 원래 있어 이 수정에서 뺐다(사용자 결정). J42의 원래 프로그램을 그대로 둔다.

고치는 곳: signals.add.xml(있으면)과 도로망 안의 같은 교차로 신호 프로그램. 연결 수는 바뀌지 않는다. 다시 돌려도 결과가 같다.
"""
from __future__ import annotations

import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.patch_network import digest  # noqa: E402

SOURCE = BASE / "maps" / "hapjeong" / "source"
MAP_JSON = BASE / "maps" / "hapjeong" / "map.json"
NEW_VERSION = "3.13.0"
TLS_IDS = ("J96",)


def right_turns_and_conflicts(net_path: Path, tls: str) -> tuple[list[int], dict[int, list[int]]]:
    """우회전 링크 번호 목록과 {우회전 링크: 충돌하는 횡단보도 링크 번호들}."""
    text = net_path.read_text(encoding="utf-8")
    root = ET.fromstring(text.encode("utf-8"))
    crossing = {e.get("id") for e in root.iter("edge") if e.get("function") == "crossing"}
    links = {}
    for c in root.iter("connection"):
        if c.get("tl") == tls:
            links[int(c.get("linkIndex"))] = (c.get("dir"), c.get("to") in crossing)
    cross_links = {i for i, (_, is_cross) in links.items() if is_cross}
    rights = sorted(i for i, (d, is_cross) in links.items() if d == "r" and not is_cross)
    junction = re.search(rf'<junction id="{re.escape(tls)}".*?</junction>', text, re.S).group(0)
    foes: dict[int, list[int]] = {}
    for i, _resp, f in re.findall(r'<request index="(\d+)" response="([01]+)" foes="([01]+)"', junction):
        n = len(f)
        foes[int(i)] = [k for k in range(n) if f[n - 1 - k] == "1" and k in cross_links]
    return rights, {i: foes.get(i, []) for i in rights}


def fix_programs(path: Path, tls: str, rights: list[int], conflicts: dict[int, list[int]], n_links: int) -> int:
    """path의 tls 신호 프로그램에서 보행자 녹색과 겹치는 우회전 녹색을 적색으로 바꾼다. 바꾼 위상 칸 수를 돌려준다."""
    tree = ET.parse(path)
    changed = 0
    for logic in tree.getroot().iter("tlLogic"):
        if logic.get("id") != tls:
            continue
        for phase in logic.iter("phase"):
            state = list(phase.get("state"))
            if len(state) != n_links:
                continue
            for i in rights:
                if state[i] in "Gg" and any(state[x] in "Gg" for x in conflicts[i]):
                    state[i] = "r"
                    changed += 1
            phase.set("state", "".join(state))
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return changed


def main() -> None:
    net_path, signals_path = SOURCE / "network.net.xml", SOURCE / "signals.add.xml"
    total = {}
    for tls in TLS_IDS:
        rights, conflicts = right_turns_and_conflicts(net_path, tls)
        root = ET.parse(net_path).getroot()
        n_links = sum(1 for c in root.iter("connection") if c.get("tl") == tls)
        changed = 0
        for path in (net_path, signals_path):
            changed += fix_programs(path, tls, rights, conflicts, n_links)
        total[tls] = {"right_turn_links": rights, "changed_phase_cells": changed}
        print(f"{tls}: 우회전 링크 {rights}, 바꾼 위상 칸 {changed}")
    if not any(v["changed_phase_cells"] for v in total.values()):
        print("이미 고쳤다: 바꾸지 않는다")
        return
    definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    if definition["version"] != NEW_VERSION:
        definition["version"] = NEW_VERSION
    definition["network_source"]["sha256"] = digest(net_path)
    definition["network_source"]["patches"]["right_turns_not_with_crosswalk_green"] = {
        "tls": total, "script": "scripts/separate_right_turns_from_crosswalks.py"}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"map {NEW_VERSION}")


if __name__ == "__main__":
    main()
