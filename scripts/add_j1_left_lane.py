"""합정역 사거리(J1)에 양화로(R42) 좌회전 차로를 하나 더 둔다(3.6.0 -> 3.7.0).

    python scripts/add_j1_left_lane.py

양화로 R42(차로 0 인도, 1~4 차도)는 차로 4만 독막로 R68로 좌회전했다. 합정역 정류장(s6)에서 출발한 시험용 버스가 J1 앞에서
차로를 여러 칸 옮겨야 해 좌회전 차로 변경(S2)이 빠듯했다. 차로 3도 직진 또는 좌회전으로 하고, 차로 3의 좌회전은 독막로
R68 차로 1(오른쪽에서 2번째)로 잇는다. 차로 4의 좌회전은 그대로 R68 차로 2로 간다.

커밋된 3.6.0 도로망은 지금의 build_hapjeong.py로 다시 만들면 보행자 경로·신호 등이 크게 달라진다. 그래서 전체를
다시 만들지 않고, 커밋된 도로망에 연결 하나만 더한다.
1. netconvert로 R42_3 -> R68_1 연결을 더한다.
2. 새 연결은 J1 신호 프로그램(signals.add.xml)에서 기존 좌회전(R42_4 -> R68_2)과 같은 신호를 받는다.
3. J1 말고 다른 신호 교차로의 연결 번호가 바뀌었으면 멈춘다.
4. map.json의 도로망 sha256과 버전을 바꾼다.
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

from scripts.patch_network import digest  # noqa: E402
from scripts.prune_network import OPTIONS  # noqa: E402

SOURCE = BASE / "maps" / "hapjeong" / "source"
MAP_JSON = BASE / "maps" / "hapjeong" / "map.json"
NEW_VERSION = "3.7.0"
TLS = "J1"
NEW_LINK = ("R42", "3", "R68", "1")
SIGNAL_FROM = ("R42", "4", "R68", "2")      # 새 연결이 신호를 받는 기존 연결


def tls_links(net_path: Path) -> dict[str, dict[int, tuple]]:
    """신호 교차로 -> {연결 번호: (from, fromLane, to, toLane) 또는 ("crossing", to)}."""
    root = ET.parse(net_path).getroot()
    crossing = {e.get("id") for e in root.iter("edge") if e.get("function") == "crossing"}
    out: dict[str, dict[int, tuple]] = {}
    for c in root.iter("connection"):
        if c.get("tl") is None:
            continue
        key = ("crossing", c.get("to")) if c.get("to") in crossing else \
            (c.get("from"), c.get("fromLane"), c.get("to"), c.get("toLane"))
        out.setdefault(c.get("tl"), {})[int(c.get("linkIndex"))] = key
    return out


def main() -> None:
    net_path, signals_path = SOURCE / "network.net.xml", SOURCE / "signals.add.xml"
    old = tls_links(net_path)
    if NEW_LINK in old[TLS].values():
        print("이미 있다: 바꾸지 않는다")
        return
    with tempfile.TemporaryDirectory(prefix="hapjeong_j1_") as name:
        tmp = Path(name)
        con = tmp / "j1.con.xml"
        root = ET.Element("connections")
        ET.SubElement(root, "connection", {"from": NEW_LINK[0], "fromLane": NEW_LINK[1],
                                            "to": NEW_LINK[2], "toLane": NEW_LINK[3]})
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
            key = new[TLS][i]
            key = SIGNAL_FROM if key == NEW_LINK else key
            if key not in index_of:
                raise ValueError(f"{TLS}에 원본에 없는 연결이 생겼다: {key}")
            order.append(index_of[key])
        if len(order) != len(old[TLS]) + 1:
            raise ValueError("J1 연결 수가 하나 늘지 않았다")
        # netconvert는 연결 번호만 새로 매기고 도로망 안의 신호 프로그램은 원래 길이로 둔다(SUMO가 linkIndex 오류로 멈췄다).
        # 도로망 안 프로그램과 signals.add.xml 프로그램을 모두 새 번호에 맞춰 다시 적는다.
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
    definition["network_source"]["patches"]["j1_second_left_lane"] = {
        "added": list(NEW_LINK), "signal_from": list(SIGNAL_FROM), "script": "scripts/add_j1_left_lane.py"}
    MAP_JSON.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"J1 좌회전 차로 추가: {NEW_LINK}, 신호는 {SIGNAL_FROM}과 같다. map {NEW_VERSION}")


if __name__ == "__main__":
    main()
