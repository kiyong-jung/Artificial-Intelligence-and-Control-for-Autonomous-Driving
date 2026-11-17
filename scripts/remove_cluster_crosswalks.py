"""cluster_J52_J56_J58의 횡단보도 두 개를 없앤다(3.8.0 -> 3.9.0).

    python scripts/remove_cluster_crosswalks.py

이 교차로의 신호 프로그램은 보행자 녹색(링크 12·13)과 차량 녹색을 겹쳐 켜서(양보 녹색 `g`), 차가 보행자에게 양보하지 못하면
부딪혔다(quiet 측정 충돌). 보행자 시간을 줄여 겹침을 없애려 했지만 어느 위상에도 충돌 없는 보행자 녹색이 없었다. 차량 녹색을
줄이지 않기로 했으므로(사용자 결정) 횡단보도와 보행자 신호를 없앤다. 이 횡단보도를 건너던 보행자는 경로 파일에서 뺀다.

1. netconvert의 `<crossing discard="true"/>`로 횡단보도 두 개(c0: R77 -R77, c1: -R73 R73)를 없앤다.
2. 신호 프로그램에서 보행자 링크 두 칸을 뺀다(도로망 안과 signals.add.xml 모두).
3. 새 도로망에서 경로를 못 찾는 보행자를 pedestrians.rou.xml에서 뺀다.
4. map.json의 도로망 sha256·버전을 바꾼다.
커밋된 도로망에 고치는 방식은 remove_j42_right_turn.py와 같다(전체를 다시 만들면 보행자 경로·신호가 달라진다).
"""
from __future__ import annotations

import json
import re
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
NEW_VERSION = "3.9.0"
TLS = "cluster_J52_J56_J58"
CROSSINGS = ("R77 -R77", "-R73 R73")


def unroutable_persons(net: Path, rou: Path, extra: list[Path]) -> list[str]:
    """새 도로망에서 SUMO가 경로를 못 만드는 보행자 id. 경로는 출발할 때 만들므로 마지막 출발 뒤까지 돌린다."""
    end = int(max(float(x) for x in re.findall(r'depart="([0-9.]+)"', rou.read_text(encoding="utf-8")))) + 60
    done = subprocess.run([sumolib.checkBinary("sumo"), "-n", str(net), "-r", str(rou), "--end", str(end),
                           "--ignore-route-errors", "true", "--no-step-log", "true", "--no-warnings", "false",
                           *(["-a", ",".join(map(str, extra))] if extra else [])],
                          capture_output=True, text=True, timeout=300)
    text = done.stdout + done.stderr
    return sorted(set(re.findall(r"[Pp]erson '([^']+)'", text)))


def prune_persons(net: Path, rou: Path) -> list[str]:
    gone = unroutable_persons(net, rou, [])
    if gone:
        text = rou.read_text(encoding="utf-8")
        for pid in gone:
            text = re.sub(rf'<person id="{re.escape(pid)}"[^>]*>.*?</person>', "", text, flags=re.S)
        rou.write_text(text, encoding="utf-8")
    return gone


def has_crossing(net_path: Path, node: str, edges: str) -> bool:
    """node 교차로에 crossingEdges가 edges인 횡단보도가 있는가."""
    for e in ET.parse(net_path).getroot().iter("edge"):
        if e.get("function") == "crossing" and e.get("crossingEdges") == edges and e.get("id").startswith(f":{node}_c"):
            return True
    return False


def main() -> None:
    remove_crosswalks(TLS, CROSSINGS, NEW_VERSION, "cluster_no_crosswalks", "scripts/remove_cluster_crosswalks.py")


def remove_crosswalks(tls_id: str, crossings: tuple, version: str, patch_key: str, script: str) -> None:
    """tls_id 교차로의 횡단보도(crossings)를 없애고 신호 프로그램·보행자 경로·map.json을 맞춘다. 다른 스크립트도 쓴다."""
    TLS = tls_id
    net_path, signals_path = SOURCE / "network.net.xml", SOURCE / "signals.add.xml"
    rou_path = SOURCE / "pedestrians.rou.xml"
    old = tls_links(net_path)
    if not any(has_crossing(net_path, TLS, edges) for edges in crossings):
        gone = prune_persons(net_path, rou_path)         # 횡단보도는 이미 없다: 보행자 정리만 다시 한다
        print(f"횡단보도는 이미 없다. 경로 없는 보행자 {len(gone)}명 제거")
        return
    with tempfile.TemporaryDirectory(prefix="hapjeong_cluster_") as name:
        tmp = Path(name)
        con = tmp / "cluster.con.xml"
        root = ET.Element("connections")
        for edges in crossings:
            ET.SubElement(root, "crossing", {"node": TLS, "edges": edges, "discard": "true"})
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
        if len(order) != len(old[TLS]) - len(crossings):
            raise ValueError("연결 수가 횡단보도 수만큼 줄지 않았다")
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
        # 경로를 못 만드는 보행자를 뺀다
        shutil.copyfile(out, net_path)
        gone = prune_persons(net_path, rou_path)
    definition = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    definition["version"] = version
    definition["network_source"]["sha256"] = digest(net_path)
    definition["network_source"]["patches"][patch_key] = {
        "removed_crossings": list(crossings), "removed_persons": len(gone), "script": script}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{TLS} 횡단보도 {len(crossings)}개 제거, 보행자 {len(gone)}명 제거. map {version}")


if __name__ == "__main__":
    main()
