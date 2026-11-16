"""서교동사거리(J42) 잔다리로 R94의 우회전을 없앤다(3.7.0 -> 3.8.0).

    python scripts/remove_j42_right_turn.py

R94는 차도가 두 차로다. 차로 1은 우회전(R53)·직진(R92)·좌회전(R54a)이 한 차로를 같이 써서, 우회전 차가 횡단보도 보행자를
기다리면 직진·좌회전 뒷차까지 막혔다. 우회전 연결(R94 차로 1 -> R53 차로 1)을 없애 차로 1은 직진·좌회전만 한다(사용자 결정).
다른 시나리오(S1~S6)는 이 우회전과 무관하다. 우회전하려던 배경 차는 경로가 바뀐다(흐름은 최단 경로로 다시 찾는다).

커밋된 도로망에 연결 하나만 빼는 방식은 add_j1_left_lane.py와 같다(전체를 다시 만들면 보행자 경로·신호가 달라진다).
J42 신호 프로그램은 연결 번호 순서로 적혀 있어, 도로망 안의 프로그램과 signals.add.xml 프로그램을 새 번호에 맞춰 다시 적는다.
"""
from __future__ import annotations

import json
import shutil
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
NEW_VERSION = "3.8.0"
TLS = "J42"
REMOVED = ("R94", "1", "R53", "1")


def main() -> None:
    net_path, signals_path = SOURCE / "network.net.xml", SOURCE / "signals.add.xml"
    old = tls_links(net_path)
    if REMOVED not in old[TLS].values():
        print("이미 없다: 바꾸지 않는다")
        return
    with tempfile.TemporaryDirectory(prefix="hapjeong_j42_") as name:
        tmp = Path(name)
        con = tmp / "j42.con.xml"
        root = ET.Element("connections")
        ET.SubElement(root, "delete", {"from": REMOVED[0], "fromLane": REMOVED[1], "to": REMOVED[2], "toLane": REMOVED[3]})
        ET.ElementTree(root).write(con, encoding="utf-8")
        out = tmp / "network.net.xml"
        done = subprocess.run([sumolib.checkBinary("netconvert"), "--sumo-net-file", str(net_path),
                               "--connection-files", str(con), *OPTIONS, "--output-file", str(out)],
                              capture_output=True, text=True, timeout=300)
        if done.returncode:
            raise RuntimeError("netconvert 실패: " + done.stderr[-3000:])
        ET.ElementTree(ET.parse(out).getroot()).write(out, encoding="utf-8", xml_declaration=True)
        new = tls_links(out)
        for tls in old:
            if tls != TLS and old[tls] != new.get(tls):
                raise ValueError(f"{tls}의 신호 연결이 바뀌었다")
        index_of = {key: i for i, key in old[TLS].items()}
        order = []
        for i in sorted(new[TLS]):
            if new[TLS][i] not in index_of:
                raise ValueError(f"{TLS}에 원본에 없는 연결이 생겼다: {new[TLS][i]}")
            order.append(index_of[new[TLS][i]])
        if len(order) != len(old[TLS]) - 1:
            raise ValueError("J42 연결 수가 하나 줄지 않았다")
        for path in (out, signals_path):
            tree = ET.parse(path)
            for logic in tree.getroot().iter("tlLogic"):
                if logic.get("id") != TLS:
                    continue
                for phase in logic.iter("phase"):
                    state = phase.get("state")
                    if len(state) == len(old[TLS]):
                        phase.set("state", "".join(state[i] for i in order))
            tree.write(path, encoding="utf-8", xml_declaration=True)
        shutil.copyfile(out, net_path)
    definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    definition["version"] = NEW_VERSION
    definition["network_source"]["sha256"] = digest(net_path)
    definition["network_source"]["patches"]["j42_r94_no_right_turn"] = {
        "removed": list(REMOVED), "script": "scripts/remove_j42_right_turn.py"}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"J42 우회전 제거: {REMOVED}. map {NEW_VERSION}")


if __name__ == "__main__":
    main()
