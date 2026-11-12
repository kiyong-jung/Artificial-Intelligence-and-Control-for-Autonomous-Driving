"""합정역 실행 도로망 전체를 그린다. 버스 경로, 정류장, 신호 없는 횡단보도, 골목 합류점, 지름길, 자른 점을 표시한다.

    python3 scripts/render_map.py
"""
from __future__ import annotations

import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_fix_figures import BASE, FONT, Net, load

FOLDER = BASE / "maps" / "hapjeong"
ROUTE, STOP, CUT, SHORTCUT = "#1565c0", "#f5b400", "#c62828", "#e65100"


def main():
    font_manager.fontManager.addfont(FONT)
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
    net = Net(load(None))
    definition = json.loads((FOLDER / "map.json").read_text(encoding="utf-8"))
    route = definition["routes"][definition["ego_route"]]

    fig = plt.figure(figsize=(16, 10.4), dpi=150)
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.91])
    ax.set_facecolor("#e8eee3")
    ax.set_aspect("equal")
    ax.set_xticks([]), ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)
    fig.text(0.02, 0.955, f"합정역 도로망 {definition['version']}", fontsize=24, fontweight="bold")
    fig.text(0.02, 0.928, f"버스 경로 {definition['ego_route']} {definition['route_design'][definition['ego_route']]['length_m']:,.0f}m, "
             f"가로변 정류장 {len(definition['bus_stops'])}곳, 신호 없는 횡단보도 1곳. 주황은 측정으로 고른 지름길, 빨간 X는 자른 점",
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

    merges = ["X_dongmak3", "X_dongmak6"]
    for shortcut in definition["network_source"]["patches"]["shortcuts"]:
        merges += shortcut["merges"]
        for edge_id in shortcut["edges"]:
            ax.plot(*zip(*net.lanes[f"{edge_id}_0"]["points"]), color=SHORTCUT, lw=4.5, solid_capstyle="round", zorder=5)
        points = net.lanes[f"{shortcut['edges'][-1]}_0"]["points"]
        ax.annotate(shortcut["names"][-1], points[len(points) // 2], xytext=(-12, 10), textcoords="offset points", fontsize=13,
                    color=SHORTCUT, fontweight="bold", ha="right", zorder=8,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=SHORTCUT, alpha=0.92))
    for node in merges:
        junction = net.junctions[node]
        ax.plot(junction["x"], junction["y"], "^", ms=12, color="#8e44ad", mec="black", zorder=6)
    for node in definition["network_source"]["patches"]["cuts"]:
        junction = net.junctions[node]
        ax.plot(junction["x"], junction["y"], "X", ms=20, color=CUT, mec="white", mew=1.5, zorder=9)
    for name, node in (("합정역 사거리", "J1"), ("서교동사거리", "J42"), ("홍대입구역사거리", "J96")):
        if node not in net.junctions:
            continue
        junction = net.junctions[node]
        ax.annotate(name, (junction["x"], junction["y"]), xytext=(12, -26), textcoords="offset points", fontsize=13.5,
                    color="#222222", zorder=8, bbox=dict(boxstyle="round,pad=0.2", fc="#fff8e1", ec="#999999", alpha=0.9))

    handles = [plt.Line2D([], [], color=ROUTE, lw=5, label="버스 경로"),
               plt.Line2D([], [], color="#4a4a4a", lw=2, label="차도"),
               plt.Line2D([], [], color="#7aa0d6", lw=2, label="버스 전용 차로"),
               plt.Line2D([], [], marker="s", ls="", ms=11, color=STOP, mec="black", label="가로변 정류장"),
               plt.Line2D([], [], marker="D", ls="", ms=10, color="#2e9d48", mec="black", label="신호 없는 횡단보도"),
               plt.Line2D([], [], marker="^", ls="", ms=11, color="#8e44ad", mec="black", label="골목 합류점(신호 없음)"),
               plt.Line2D([], [], color=SHORTCUT, lw=4.5, label="지름길(측정으로 고름)"),
               plt.Line2D([], [], marker="X", ls="", ms=13, color=CUT, mec="white", label="자른 점(새 경계)")]
    ax.legend(handles=handles, loc="lower left", fontsize=12.5, framealpha=0.95)
    xs = [x for lane in net.lanes.values() if not lane["internal"] for x, _ in lane["points"]]
    ys = [y for lane in net.lanes.values() if not lane["internal"] for _, y in lane["points"]]
    ax.set_xlim(min(xs) - 20, max(xs) + 20)
    ax.set_ylim(min(ys) - 20, max(ys) + 20)
    out = FOLDER / "references" / "hapjeong_map.png"
    fig.savefig(out)
    print(out)


if __name__ == "__main__":
    main()
