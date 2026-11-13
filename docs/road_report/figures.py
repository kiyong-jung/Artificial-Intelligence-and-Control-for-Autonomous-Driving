"""도로 보고서(road_report.tex)에 넣는 그림 두 장을 그린다.

    python3 docs/road_report/figures.py
"""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
BASE = HERE.parents[1]
sys.path.insert(0, str(BASE / "scripts"))
from render_fix_figures import FONT, Net  # noqa: E402

font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
FOLDER = BASE / "maps" / "hapjeong"
ROUTE, ALLEY = "#1565c0", "#d50000"   # 골목은 지도 그림(scripts/render_map.py)과 같은 빨강
COLORS = {"S1": "#f5b400", "S2": "#8e44ad", "S3": "#f5b400", "S4": "#f5b400", "S5": "#e65100", "S6": "#2e9d48"}


def scenario_map():
    net = Net((FOLDER / "source" / "network.net.xml").read_text(encoding="utf-8"))
    definition = json.loads((FOLDER / "map.json").read_text(encoding="utf-8"))
    route = definition["routes"][definition["ego_route"]]
    fig = plt.figure(figsize=(16, 10), dpi=150)
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.98])
    ax.set_facecolor("#eef2ea"), ax.set_aspect("equal"), ax.set_xticks([]), ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)
    for lane_id, lane in net.lanes.items():
        if not lane["internal"] and lane["allow"] != "pedestrian":
            ax.plot(*zip(*lane["points"]), color="#b5b5b5", lw=1.0, zorder=1)
    for edge_id in route:
        lanes = [lane for lane_id, lane in net.lanes.items() if lane_id.rsplit("_", 1)[0] == edge_id and lane["allow"] != "pedestrian"]
        if lanes:
            ax.plot(*zip(*lanes[len(lanes) // 2]["points"]), color=ROUTE, lw=6, alpha=0.55, solid_capstyle="round", zorder=2)
    shortcuts = definition["network_source"]["patches"]["shortcuts"]
    merges = ["X_dongmak3", "X_dongmak6"] + [node for shortcut in shortcuts for node in shortcut["merges"]]
    for shortcut in shortcuts:
        for edge_id in shortcut["edges"]:
            ax.plot(*zip(*net.lanes[f"{edge_id}_0"]["points"]), color=ALLEY, lw=2.2, alpha=0.8, zorder=3)

    def tag(text, xy, offset, color):
        ax.annotate(text, xy, xytext=offset, textcoords="offset points", fontsize=15, fontweight="bold", color="white",
                    ha="center", va="center", zorder=9, bbox=dict(boxstyle="round,pad=0.3", fc=color, ec="white", lw=1.2),
                    arrowprops=dict(arrowstyle="-", color=color, lw=1.6))

    for index, node in enumerate(merges):
        junction = net.junctions[node]
        ax.plot(junction["x"], junction["y"], "^", ms=15, color=COLORS["S5"], mec="black", zorder=6)
    tag("S5 골목 차량 끼어들기\n(세모 8곳)", (net.junctions["X_jandari7"]["x"], net.junctions["X_jandari7"]["y"]), (-150, 30), COLORS["S5"])
    offsets = {"합정역7번출구": (30, -46), "성산중학교": (10, -48), "상수역": (-70, -44), "극동방송": (88, -30), "서교동교회": (96, 34), "합정역": (-70, 40)}
    for stop in definition["bus_stops"]:
        points = net.lanes[stop["lane"]]["points"]
        x, y = points[len(points) // 2]
        ax.plot(x, y, "s", ms=15, color=COLORS["S1"], mec="black", zorder=6)
        tag(f"{stop['name']}\nS1·S3·S4", (x, y), offsets.get(stop["name"], (40, 30)), "#b07d00")
    for lane_id in net.crossings:
        if lane_id.startswith(":S6_"):
            x, y = net.lanes[lane_id]["points"][0]
            ax.plot(x, y, "D", ms=15, color=COLORS["S6"], mec="black", zorder=7)
            tag("S6 신호 없는 횡단보도", (x, y), (120, 36), COLORS["S6"])
    points = net.lanes["R111b_1"]["points"]
    tag("S2 정류장 직후\n좌회전 차로 변경", points[len(points) // 2], (-40, 64), COLORS["S2"])
    start = net.lanes[f"{route[0]}_1"]["points"]
    ax.annotate("출발", start[0], xytext=(-46, 22), textcoords="offset points", fontsize=14, color=ROUTE, fontweight="bold",
                arrowprops=dict(arrowstyle="-|>", color=ROUTE, lw=2))
    for name, node, offset in (("합정역 사거리", "J1", (-70, -34)), ("서교동사거리", "J42", (-70, 30)), ("상수역 사거리", "J82", (70, -30)),
                               ("삼거리포차 앞", "J83", (70, 26))):
        junction = net.junctions[node]
        ax.annotate(name, (junction["x"], junction["y"]), xytext=offset, textcoords="offset points", fontsize=13, color="#333333",
                    ha="center", bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#999999", alpha=0.9), zorder=8)
    xs = [x for edge_id in route for lane_id, lane in net.lanes.items() if lane_id.rsplit("_", 1)[0] == edge_id for x, _ in lane["points"]]
    ys = [y for edge_id in route for lane_id, lane in net.lanes.items() if lane_id.rsplit("_", 1)[0] == edge_id for _, y in lane["points"]]
    ax.set_xlim(min(xs) - 150, max(xs) + 190), ax.set_ylim(min(ys) - 110, max(ys) + 120)
    fig.savefig(HERE / "scenario_map.png")


def progress():
    # (항목, 시작(3.2.0), 최종(3.4.0), 단위). 숫자는 maps/hapjeong/README.md의 측정값이다.
    rows = [("S5 골목에서 버스 방향으로\n합류하는 차", 34, 218, "대/시간"), ("S5 버스 앞에 끼어든 차", 6, 33, "번/80회"),
            ("S1·S3 정류장에서\n옆에 차", 101, 210, "회/약 480회"), ("S6 횡단보도 앞에\n보행자", 45, 72, "회/80회"),
            ("가장 한산한 구간의\n교통량", 76, 135, "대/시간"), ("가장 한산한 구간에서\n버스 옆에 차가 있던 시간", 17, 30, "% (3.3.0 → 3.4.0)")]
    fig, axes = plt.subplots(1, len(rows), figsize=(16, 4.2), dpi=150)
    for ax, (label, before, after, unit) in zip(axes, rows):
        bars = ax.bar(["고치기 전", "고친 뒤"], [before, after], color=["#b9b9b9", "#e65100"], width=0.62)
        for bar, value in zip(bars, (before, after)):
            ax.text(bar.get_x() + bar.get_width() / 2, value, f"{value}", ha="center", va="bottom", fontsize=17, fontweight="bold")
        ax.set_title(label, fontsize=13.5, fontweight="bold", pad=10)
        ax.set_xlabel(unit, fontsize=12, color="#555555")
        ax.set_ylim(0, after * 1.22), ax.set_yticks([]), ax.tick_params(axis="x", labelsize=12, length=0)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)
    fig.tight_layout()
    fig.savefig(HERE / "progress.png")


if __name__ == "__main__":
    scenario_map()
    progress()
    print(HERE)
