"""합정역 실행 도로망 전체를 지도처럼 그린다. 버스 경로, 정류장, 신호 교차로, 신호 없는 횡단보도, 골목(빨강), 골목 합류점, 자른 점을 표시한다.

    python3 scripts/render_map.py
    python3 scripts/render_map.py maps/hapjeong_trial_x "후보 포함, 측정 중"   # 다른 지도 폴더를 그릴 때(제목에 덧붙일 말)
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager, patheffects
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_fix_figures import BASE, FONT, Net

FOLDER = BASE / "maps" / "hapjeong"
PAPER, BUILDING, ROAD, BUS_LANE = "#f6f5f1", "#eae7df", "#c6c3ba", "#a9bfe3"
INK, MUTED = "#222222", "#6b6b6b"
ROUTE, STOP, SIGNAL, ALLEY, MERGE, CROSSING = "#1565c0", "#f2a900", "#ffd400", "#d32f2f", "#7b3fa0", "#2e9d48"
ALLEY_MPS = 30 / 3.6
HALO = [patheffects.withStroke(linewidth=3, foreground="white")]
# 이름표 위치(pt). 정류장은 버스 경로 바깥쪽, 교차로 이름은 교차로 옆 빈 곳에 둔다.
STOP_OFFSETS = {"합정역7번출구": (-6, -16), "성산중학교": (8, -17), "상수역": (-8, -15), "극동방송": (11, 2),
                "서교동교회": (14, -6), "합정역": (-10, 9)}
JUNCTION_LABELS = (("합정역 사거리", "J1", (-22, 14), "right"), ("서교동사거리", "J42", (-12, 10), "right"),
                   ("홍대입구역사거리", "J96", (10, -12), "left"), ("상수역 사거리", "J82", (10, 12), "left"),
                   ("삼거리포차 앞", "J83", (8, 8), "left"))
# 골목 이름표를 길 가운데에서 옮길 거리(m, 길 방향). 이름이 정류장이나 다른 이름과 겹치는 길만 적는다.
ALLEY_SHIFT = {"잔다리로6길": -22}
LABEL_MIN_M = 40   # 이보다 짧은 골목은 이름을 적지 않는다(범례로 충분하다)
# 좁은 곳에 붙어 있는 골목들은 이름을 한 줄로 묶어 지시선 끝(pt)에 적는다.
LABEL_GROUPS = {("독막로4길", "성지1길", "독막로6길"): (-70, -46)}


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


def along(points, fraction):
    """선 위에서 길이 비율 fraction인 점과 그 점의 방향(도)."""
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    target = sum(lengths) * fraction
    for (a, b), length in zip(zip(points, points[1:]), lengths):
        if target <= length or length == lengths[-1]:
            t = target / length if length else 0
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t), math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        target -= length
    return points[-1], 0.0


def main(folder: Path = FOLDER, note: str = ""):
    font_manager.fontManager.addfont(FONT)
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
    net = Net((folder / "source" / "network.net.xml").read_text(encoding="utf-8"))
    definition = json.loads((folder / "map.json").read_text(encoding="utf-8"))
    route = definition["routes"][definition["ego_route"]]
    patches = definition["network_source"]["patches"]

    xs = [x for lane in net.lanes.values() if not lane["internal"] for x, _ in lane["points"]]
    ys = [y for lane in net.lanes.values() if not lane["internal"] for _, y in lane["points"]]
    (x0, x1), (y0, y1) = (min(xs) - 25, max(xs) + 25), (min(ys) - 25, max(ys) + 25)
    fig = plt.figure(figsize=(12, 12 * 0.98 * (y1 - y0) / (x1 - x0) / 0.90), dpi=200, facecolor="white")
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.90])
    ax.set_xlim(x0, x1), ax.set_ylim(y0, y1), ax.set_aspect("equal"), ax.axis("off")
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fc=PAPER, ec="none", zorder=-2))
    fig.canvas.draw()
    meter = ax.get_window_extent().width / (x1 - x0) * 72 / fig.dpi   # 1m가 몇 pt인가

    for poly in ET.parse(folder / "source" / "context.add.xml").getroot().iter("poly"):
        if poly.get("type") in ("building", "amenity", "parking"):
            points = [tuple(map(float, p.split(","))) for p in poly.get("shape").split()]
            if any(x0 < x < x1 and y0 < y < y1 for x, y in points):
                ax.fill(*zip(*points), color=BUILDING, lw=0, zorder=-1)
    named, origins = alleys(folder / "source" / "network.net.xml")
    alley_ids = {edge_id for edge_ids in named.values() for edge_id in edge_ids}
    for junction in net.junctions.values():
        if len(junction["shape"]) > 2:
            ax.fill(*zip(*junction["shape"]), color=ROAD, lw=0, zorder=1)
    # 차로를 실제보다 조금 넓게 칠해 이웃 차로와 겹치게 한다. 딱 맞게 칠하면 차로 사이에 가는 틈이 보인다.
    for lane_id, lane in net.lanes.items():
        if not lane["internal"] and lane["allow"] != "pedestrian" and lane_id.rsplit("_", 1)[0] not in alley_ids:
            ax.plot(*zip(*lane["points"]), color=BUS_LANE if lane["allow"] == "bus" else ROAD,
                    lw=(lane["width"] + 0.6) * meter, solid_capstyle="butt", zorder=1 + (lane["allow"] == "bus"))
    for lane_id, lane in net.lanes.items():
        if lane["crossing"]:
            ax.plot(*zip(*lane["points"]), color="white", lw=lane["width"] * meter, dashes=(0.5, 0.5),
                    solid_capstyle="butt", zorder=2)

    grouped = {name for group in LABEL_GROUPS for name in group}
    for group, offset in LABEL_GROUPS.items():
        points = [point for name in group if name in named for edge_id in named[name]
                  for point in net.lanes[f"{edge_id}_0"]["points"]]
        if points:
            center = (sum(x for x, _ in points) / len(points), sum(y for _, y in points) / len(points))
            ax.annotate("·".join(name for name in group if name in named), center, xytext=offset,
                        textcoords="offset points", ha="center", va="center", fontsize=8, color=ALLEY, fontweight="bold",
                        path_effects=HALO, zorder=9, arrowprops=dict(arrowstyle="-", color=ALLEY, lw=0.8, shrinkB=2))
    for name, edge_ids in named.items():
        lines = [net.lanes[f"{edge_id}_0"]["points"] for edge_id in edge_ids]
        for points in lines:
            ax.plot(*zip(*points), color=ALLEY, lw=max(3.0 * meter, 2.2), solid_capstyle="round", zorder=3)
        if name in grouped:
            continue
        points = max(lines, key=lambda line: sum(math.dist(a, b) for a, b in zip(line, line[1:])))
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        if length < LABEL_MIN_M:
            continue
        (x, y), angle = along(points, min(max(0.5 + ALLEY_SHIFT.get(name, 0) / length, 0.1), 0.9))
        angle = (angle + 90) % 180 - 90
        normal = math.radians(angle + 90)
        ax.annotate(name, (x, y), xytext=(7 * math.cos(normal), 7 * math.sin(normal)), textcoords="offset points",
                    rotation=angle, rotation_mode="anchor", ha="center", va="center", fontsize=8, color=ALLEY,
                    fontweight="bold", path_effects=HALO, zorder=9)

    def route_line(points):
        ax.plot(*zip(*points), color=ROUTE, lw=3.4, alpha=0.85, solid_capstyle="round", solid_joinstyle="round", zorder=4)
    for edge_id in route:
        route_line(net.lanes[f"{edge_id}_1"]["points"])
    for index in range(len(route) - 1):
        points = next((net.path(route[index], str(lane), route[index + 1], str(target))
                       for lane in range(1, 7) for target in range(1, 7)
                       if net.path(route[index], str(lane), route[index + 1], str(target))), None)
        if points:
            route_line(points)
    for junction in net.junctions.values():
        if junction["type"].startswith("traffic_light"):
            ax.plot(junction["x"], junction["y"], "o", ms=5, color=SIGNAL, mec=INK, mew=0.6, zorder=6)
    merges = ["X_dongmak3", "X_dongmak6"] + [node for shortcut in patches["shortcuts"] for node in shortcut["merges"]]
    for node in merges:
        if node in net.junctions:
            ax.plot(net.junctions[node]["x"], net.junctions[node]["y"], "^", ms=7, color=MERGE, mec="white", mew=0.8, zorder=7)
    for node in origins:
        ax.plot(net.junctions[node]["x"], net.junctions[node]["y"], "o", ms=6, color="white", mec=ALLEY, mew=1.8, zorder=7)
    for lane_id in net.crossings:
        if lane_id.startswith(":S6_"):
            x, y = net.lanes[lane_id]["points"][0]
            ax.plot(x, y, "D", ms=7.5, color=CROSSING, mec="white", mew=0.8, zorder=8)
            ax.annotate("신호 없는 횡단보도", (x, y), xytext=(-9, -1), textcoords="offset points", ha="right", va="center",
                        fontsize=8, color=CROSSING, fontweight="bold", path_effects=HALO, zorder=9)
    for stop in definition["bus_stops"]:
        points = net.lanes[stop["lane"]]["points"]
        x, y = points[len(points) // 2]
        ax.plot(x, y, "s", ms=8, color=STOP, mec=INK, mew=0.9, zorder=8)
        dx, dy = STOP_OFFSETS.get(stop["name"], (10, 0))
        ax.annotate(stop["name"], (x, y), xytext=(dx, dy), textcoords="offset points", fontsize=9, fontweight="bold",
                    color=INK, ha="left" if dx >= 0 else "right", va="center", path_effects=HALO, zorder=9)
    for name, node, offset, align in JUNCTION_LABELS:
        junction = net.junctions[node]
        ax.annotate(name, (junction["x"], junction["y"]), xytext=offset, textcoords="offset points", fontsize=8.5,
                    color=MUTED, ha=align, va="center", path_effects=HALO, zorder=9)
    for node in patches["cuts"]:
        junction = net.junctions[node]
        ax.plot(junction["x"], junction["y"], "x", ms=8, color=INK, mew=2, zorder=8)
    start = net.lanes[f"{route[0]}_1"]["points"]
    (x, y), angle = along(start, 0.45)
    ax.annotate("", xy=(x + 18 * math.cos(math.radians(angle)), y + 18 * math.sin(math.radians(angle))), xytext=(x, y),
                arrowprops=dict(arrowstyle="-|>,head_length=0.8,head_width=0.45", color=ROUTE, lw=3.4), zorder=5)
    ax.annotate("출발", (x, y), xytext=(-2, -13), textcoords="offset points", fontsize=8.5, color=ROUTE,
                fontweight="bold", ha="center", path_effects=HALO, zorder=9)

    bar_x, bar_y = x0 + 30, y0 + 30
    ax.plot([bar_x, bar_x + 100], [bar_y, bar_y], color=INK, lw=2, solid_capstyle="butt", zorder=9)
    for tick in (bar_x, bar_x + 100):
        ax.plot([tick, tick], [bar_y - 4, bar_y + 4], color=INK, lw=1.2, zorder=9)
    ax.annotate("100m", (bar_x + 50, bar_y), xytext=(0, 5), textcoords="offset points", ha="center", fontsize=8,
                color=INK, zorder=9)
    ax.annotate("N", (bar_x + 160, bar_y - 10), xytext=(bar_x + 160, bar_y + 30), ha="center", va="bottom", fontsize=10,
                fontweight="bold", color=INK, arrowprops=dict(arrowstyle="<|-", color=INK, lw=1.4), zorder=9)

    fig.text(0.015, 0.965, f"합정역 도로망 {definition['version']}" + (f" ({note})" if note else ""), fontsize=17,
             fontweight="bold", color=INK, va="center")
    fig.text(0.015, 0.932, f"버스 경로 {definition['route_design'][definition['ego_route']]['length_m']:,.0f}m, "
             f"가로변 정류장 {len(definition['bus_stops'])}곳, 신호 교차로 "
             f"{sum(j['type'].startswith('traffic_light') for j in net.junctions.values())}곳, 골목 {len(named)}개",
             fontsize=10, color=MUTED, va="center")
    handles = [plt.Line2D([], [], color=ROUTE, lw=3.4, label="버스 경로(시계 반대 방향 한 바퀴)"),
               plt.Line2D([], [], color=ROAD, lw=8, label="간선 도로"),
               plt.Line2D([], [], color=BUS_LANE, lw=7, label="중앙버스전용차로"),
               plt.Line2D([], [], color=ALLEY, lw=2.4, label="골목(30km/h 이하)"),
               plt.Line2D([], [], marker="s", ls="", ms=8, color=STOP, mec=INK, label="버스 정류장"),
               plt.Line2D([], [], marker="o", ls="", ms=6, color=SIGNAL, mec=INK, label="신호 교차로"),
               plt.Line2D([], [], marker="D", ls="", ms=7, color=CROSSING, mec="white", label="신호 없는 횡단보도"),
               plt.Line2D([], [], marker="^", ls="", ms=8, color=MERGE, mec="white", label="골목 합류점(신호 없음)"),
               plt.Line2D([], [], marker="o", ls="", ms=6, color="white", mec=ALLEY, mew=1.8, label="골목이 시작하는 곳"),
               plt.Line2D([], [], marker="x", ls="", ms=7, color=INK, mew=2, label="도로망 경계(자른 곳)")]
    # 범례는 지도 왼쪽 위의 건물만 있는 곳에 둔다.
    legend = ax.legend(handles=handles, loc="upper left", fontsize=9.5, handlelength=2.2, labelspacing=0.75,
                       borderpad=0.9, framealpha=0.93, edgecolor="#d8d4ca", fancybox=False)
    legend.set_zorder(10)
    ax.text(0.99, 0.01, f"옅은 면은 건물이다. 골목은 모두 일방통행이고, {LABEL_MIN_M}m보다 짧은 골목은 이름을 생략했다.",
            ha="right", va="bottom", fontsize=8, color=MUTED, transform=ax.transAxes, path_effects=HALO, zorder=10)
    out = folder / "references" / "hapjeong_map.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, facecolor="white")
    print(out)


if __name__ == "__main__":
    main(*(Path(sys.argv[1]),) if len(sys.argv) > 1 else (), *sys.argv[2:3])
