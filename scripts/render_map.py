"""합정역 실행 도로망 전체를 그린다. 버스 경로, 정류장, 신호 교차로, 신호 없는 횡단보도, 골목(빨강), 골목 합류점, 자른 점을 표시한다.

    python3 scripts/render_map.py
    python3 scripts/render_map.py maps/hapjeong_trial_x "후보 포함, 측정 중"   # 다른 지도 폴더를 그릴 때(제목에 덧붙일 말)
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_fix_figures import BASE, FONT, Net

FOLDER = BASE / "maps" / "hapjeong"
ROUTE, STOP, OTHER_STOP, SIGNAL, CUT, ALLEY = "#1565c0", "#f5b400", "#9bb8e0", "#ffd400", "#212121", "#d50000"
ALLEY_MPS = 30 / 3.6
# 이름표가 정류장이나 다른 골목 이름과 겹치는 길은 옮겨 적는다(pt).
LABEL_OFFSETS = {"독막로6길": (-78, -30), "와우산로11길": (22, -26), "독막로15길": (12, 18), "와우산로13길": (30, -8)}


def alleys(net_path: Path) -> tuple[dict[str, list[str]], list[str]]:
    """대로가 아닌 길(이름 → 도로 id들)과, 그 길이 시작하는 점 가운데 들어오는 차도가 없는 노드.

    대로가 아닌 길은 제한속도 30km/h 이하이고 이름이 있는 차도다. 합정역 사거리의 이름 없는 30km/h 연결로는
    대로의 일부라 뺀다. 들어오는 차도가 없는 시작점은 건물과 주차장에서 차가 나오는 출발점이다.
    """
    found: dict[str, list[str]] = {}
    starts, entered = set(), set()
    for edge in ET.parse(net_path).getroot().iter("edge"):
        lanes = [lane for lane in edge.findall("lane") if lane.get("allow") != "pedestrian"]
        if edge.get("function") is not None or not lanes:
            continue
        entered.add(edge.get("to"))
        if edge.get("name") and max(float(lane.get("speed")) for lane in lanes) <= ALLEY_MPS + 0.01:
            found.setdefault(edge.get("name"), []).append(edge.get("id"))
            starts.add(edge.get("from"))
    return found, sorted(starts - entered)


def main(folder: Path = FOLDER, note: str = ""):
    font_manager.fontManager.addfont(FONT)
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
    net = Net((folder / "source" / "network.net.xml").read_text(encoding="utf-8"))
    definition = json.loads((folder / "map.json").read_text(encoding="utf-8"))
    route = definition["routes"][definition["ego_route"]]

    fig = plt.figure(figsize=(16, 10.4), dpi=150)
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.91])
    ax.set_facecolor("#e8eee3")
    ax.set_aspect("equal")
    ax.set_xticks([]), ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)
    fig.text(0.02, 0.955, f"합정역 도로망 {definition['version']}" + (f" ({note})" if note else ""), fontsize=24, fontweight="bold")
    fig.text(0.02, 0.928, f"버스 경로 {definition['ego_route']} {definition['route_design'][definition['ego_route']]['length_m']:,.0f}m, "
             f"경로 가로변 정류장 {len(definition['bus_stops'])}곳, 신호 없는 횡단보도 1곳. 빨강은 대로가 아닌 길(골목), 검은 X는 자른 점",
             fontsize=14, color="#555555")

    for junction in net.junctions.values():
        if len(junction["shape"]) > 2:
            ax.fill(*zip(*junction["shape"]), color="#4a4a4a", zorder=1, lw=0)
    for lane_id, lane in net.lanes.items():
        if lane["internal"]:
            continue
        color = "#bdbdbd" if lane["allow"] == "pedestrian" else "#7aa0d6" if lane["allow"] == "bus" else "#4a4a4a"
        ax.plot(*zip(*lane["points"]), color=color, lw=0.9 if lane["allow"] == "pedestrian" else 1.3,
                solid_capstyle="butt", zorder=2)
    for edge_id in route:
        lanes = [lane for lane_id, lane in net.lanes.items()
                 if lane_id.rsplit("_", 1)[0] == edge_id and lane["allow"] != "pedestrian"]
        if lanes:
            middle = lanes[len(lanes) // 2]
            ax.plot(*zip(*middle["points"]), color=ROUTE, lw=5, alpha=0.75, solid_capstyle="round", zorder=3)
    for index in range(len(route) - 1):
        points = next((net.path(route[index], str(lane), route[index + 1], str(target))
                       for lane in range(1, 7) for target in range(1, 7)
                       if net.path(route[index], str(lane), route[index + 1], str(target))), None)
        if points:
            ax.plot(*zip(*points), color=ROUTE, lw=5, alpha=0.75, solid_capstyle="round", zorder=3)
    start = net.lanes[f"{route[0]}_1"]["points"]
    ax.annotate("", xy=start[1], xytext=start[0], zorder=4,
                arrowprops=dict(arrowstyle="-|>", color=ROUTE, lw=3, mutation_scale=30))

    for junction in net.junctions.values():
        if junction["type"].startswith("traffic_light"):
            ax.plot(junction["x"], junction["y"], "o", ms=7, color=SIGNAL, mec="black", mew=0.8, zorder=5)
    # 경로 밖 정류장(양화로 중앙버스차로와 가로변)은 원본의 stops.add.xml에 있다.
    for stop in ET.parse(folder / "source" / "stops.add.xml").getroot().iter("busStop"):
        points = net.lanes[stop.get("lane")]["points"]
        ax.plot(*points[len(points) // 2], "s", ms=9, color=OTHER_STOP, mec="black", mew=1.0, zorder=6)
    for stop in definition["bus_stops"]:
        points = net.lanes[stop["lane"]]["points"]
        x, y = points[len(points) // 2]
        ax.plot(x, y, "s", ms=13, color=STOP, mec="black", mew=1.2, zorder=6)
        ax.annotate(stop["name"], (x, y), xytext=(9, -14), textcoords="offset points", fontsize=12.5,
                    fontweight="bold", zorder=7, bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=STOP, alpha=0.92))
    for lane_id, crossed in net.crossings.items():
        if lane_id.startswith(":S6_"):
            x, y = net.lanes[lane_id]["points"][0]
            ax.plot(x, y, "D", ms=12, color="#2e9d48", mec="black", zorder=6)
            ax.annotate("신호 없는 횡단보도", (x, y), xytext=(10, 6), textcoords="offset points", fontsize=12,
                        color="#1b5e20", fontweight="bold", zorder=7)
    ax.annotate("골목 합류점", (net.junctions["X_dongmak3"]["x"], net.junctions["X_dongmak3"]["y"]),
                xytext=(-10, 16), textcoords="offset points", fontsize=12, color="#6c3483", fontweight="bold", zorder=7)

    named, origins = alleys(folder / "source" / "network.net.xml")
    for name, edge_ids in named.items():
        lines = [net.lanes[f"{edge_id}_0"] for edge_id in edge_ids]
        for line in lines:
            ax.plot(*zip(*line["points"]), color=ALLEY, lw=3.5, solid_capstyle="round", zorder=5)
        # 이름은 그 길에서 가장 긴 도로의 가운데에 한 번만 적는다.
        points = max(lines, key=lambda line: line["length"])["points"]
        ax.annotate(name, points[len(points) // 2], xytext=LABEL_OFFSETS.get(name, (8, 6)), textcoords="offset points", fontsize=11.5,
                    color=ALLEY, fontweight="bold", zorder=8,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec=ALLEY, alpha=0.9))
    for node in origins:
        ax.plot(net.junctions[node]["x"], net.junctions[node]["y"], "o", ms=10, color="white", mec=ALLEY, mew=2.5, zorder=6)
    merges = ["X_dongmak3", "X_dongmak6"] + [node for shortcut in definition["network_source"]["patches"]["shortcuts"]
                                             for node in shortcut["merges"]]
    for node in merges:
        junction = net.junctions[node]
        ax.plot(junction["x"], junction["y"], "^", ms=12, color="#8e44ad", mec="black", zorder=6)
    for node in definition["network_source"]["patches"]["cuts"]:
        junction = net.junctions[node]
        ax.plot(junction["x"], junction["y"], "X", ms=20, color=CUT, mec="white", mew=1.5, zorder=9)
    for name, node, offset in (("합정역 사거리", "J1", (14, 24)), ("서교동사거리", "J42", (14, -28)),
                               ("홍대입구역사거리", "J96", (-44, -36)), ("상수역 사거리", "J82", (-118, 22)),
                               ("삼거리포차 앞", "J83", (-104, 20))):
        junction = net.junctions[node]
        ax.annotate(name, (junction["x"], junction["y"]), xytext=offset, textcoords="offset points", fontsize=13.5,
                    color="#222222", zorder=8, bbox=dict(boxstyle="round,pad=0.2", fc="#fff8e1", ec="#999999", alpha=0.9))

    handles = [plt.Line2D([], [], color=ROUTE, lw=5, label="버스 경로"),
               plt.Line2D([], [], color="#4a4a4a", lw=2, label="차도"),
               plt.Line2D([], [], color="#7aa0d6", lw=2, label="버스 전용 차로"),
               plt.Line2D([], [], marker="o", ls="", ms=8, color=SIGNAL, mec="black", label="신호 교차로"),
               plt.Line2D([], [], marker="s", ls="", ms=11, color=STOP, mec="black", label="경로 가로변 정류장"),
               plt.Line2D([], [], marker="s", ls="", ms=9, color=OTHER_STOP, mec="black", label="경로 밖 정류장(중앙 4, 가로변 2)"),
               plt.Line2D([], [], marker="D", ls="", ms=10, color="#2e9d48", mec="black", label="신호 없는 횡단보도"),
               plt.Line2D([], [], marker="^", ls="", ms=11, color="#8e44ad", mec="black", label="골목 합류점(신호 없음)"),
               plt.Line2D([], [], color=ALLEY, lw=3.5, label="대로가 아닌 길(골목, 30km/h 이하)"),
               plt.Line2D([], [], marker="o", ls="", ms=9, color="white", mec=ALLEY, mew=2.5,
                          label="골목이 시작하는 곳(건물·주차장에서 차가 나옴)"),
               plt.Line2D([], [], marker="X", ls="", ms=13, color=CUT, mec="white", label="자른 점(새 경계)")]
    ax.legend(handles=handles, loc="upper left", fontsize=12.5, framealpha=0.95)
    xs = [x for lane in net.lanes.values() if not lane["internal"] for x, _ in lane["points"]]
    ys = [y for lane in net.lanes.values() if not lane["internal"] for _, y in lane["points"]]
    ax.set_xlim(min(xs) - 20, max(xs) + 20)
    ax.set_ylim(min(ys) - 20, max(ys) + 20)
    out = folder / "references" / "hapjeong_map.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out)
    print(out)


if __name__ == "__main__":
    main(*(Path(sys.argv[1]),) if len(sys.argv) > 1 else (), *sys.argv[2:3])
