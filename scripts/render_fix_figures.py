"""합정역 도로에서 고친 구간마다 기존/수정 후 그림을 만든다. 이유와 근거를 그림 안에 적는다.

    python scripts/render_fix_figures.py

기존 도로망은 git 커밋에서 읽는다(2.0.0은 PR #3의 main, 새 원본을 옮긴 직후는 3a171af).
수정 후는 지금의 maps/hapjeong/source/network.net.xml이다. matplotlib만 쓴다(SUMO 불필요).
"""
from __future__ import annotations

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt

BASE = Path(__file__).resolve().parents[1]
SOURCE = "maps/hapjeong/source"
OUT = BASE / "maps" / "hapjeong" / "references" / "fixes"
V2, PORTED = "cd93867", "3a171af"
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
RED, GREEN, ORANGE = "#e53935", "#2e9d48", "#f59a23"


class Net:
    def __init__(self, text: str):
        root = ET.fromstring(text)
        self.lanes, self.crossings, self.junctions, self.vias, self.states = {}, {}, {}, {}, {}
        for edge in root.iter("edge"):
            for lane in edge.findall("lane"):
                points = [tuple(map(float, p.split(","))) for p in lane.get("shape").split()]
                self.lanes[lane.get("id")] = dict(
                    points=points, width=float(lane.get("width", 3.2)), allow=lane.get("allow", ""),
                    internal=edge.get("id").startswith(":"), crossing=edge.get("function") == "crossing",
                    walk=edge.get("function") == "walkingarea", length=float(lane.get("length")))
            if edge.get("function") == "crossing":
                self.crossings[edge.get("id") + "_0"] = set(e.split(".")[0] for e in edge.get("crossingEdges").split())
        for junction in root.findall("junction"):
            if junction.get("type") != "internal" and junction.get("shape"):
                self.junctions[junction.get("id")] = dict(
                    type=junction.get("type"), x=float(junction.get("x")), y=float(junction.get("y")),
                    shape=[tuple(map(float, p.split(","))) for p in junction.get("shape").split()])
        for c in root.iter("connection"):
            self.vias[(c.get("from"), c.get("fromLane"), c.get("to"), c.get("toLane"))] = c.get("via")
            self.vias[(c.get("from"), c.get("fromLane"))] = c.get("via") or self.vias.get((c.get("from"), c.get("fromLane")))

    def path(self, source, source_lane, target, target_lane):
        """연결이 교차로 안에서 지나는 점들. 연결이 없으면 None이다."""
        via = self.vias.get((source, source_lane, target, target_lane))
        if not via:
            return None
        points = [self.lanes[f"{source}_{source_lane}"]["points"][-1]]
        while via:
            points += self.lanes[via]["points"]
            edge, index = via.rsplit("_", 1)
            via = self.vias.get((edge, index))
        return points + [self.lanes[f"{target}_{target_lane}"]["points"][0]]

    def crossing(self, *edges):
        return next((lane for lane, crossed in self.crossings.items() if crossed == set(edges)), None)


def load(revision: str | None, name: str = "network.net.xml") -> str:
    if revision is None:
        return (BASE / SOURCE / name).read_text(encoding="utf-8")
    return subprocess.check_output(["git", "show", f"{revision}:{SOURCE}/{name}"], cwd=BASE, text=True)


def draw(ax, net: Net, center, half, title, *, lanes=(), paths=(), crossings=(), labels=(), signals=True):
    """도로망의 한 구역을 그린다. lanes/paths/crossings는 (대상, 색) 목록이고 색으로 강조한다."""
    cx, cy = center
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half * 0.72, cy + half * 0.72)
    ax.set_aspect("equal")
    ax.set_facecolor("#dfe8d8")
    ax.set_xticks([]), ax.set_yticks([])
    ax.set_title(title, fontsize=15, fontweight="bold", pad=8)
    scale = ax.get_window_extent().width / (2 * half) * 72 / ax.figure.dpi   # 1m가 몇 pt인가
    for junction in net.junctions.values():
        if len(junction["shape"]) > 2:
            ax.fill(*zip(*junction["shape"]), color="#3a3a3a", zorder=1)
    for lane_id, lane in net.lanes.items():
        if lane["internal"] and not lane["crossing"]:
            continue
        xs, ys = zip(*lane["points"])
        if lane["crossing"]:
            ax.plot(xs, ys, color="white", lw=lane["width"] * scale, dashes=(0.25, 0.25), solid_capstyle="butt", zorder=3)
            continue
        color = "#b4b4b4" if lane["allow"] == "pedestrian" else "#2c5aa0" if lane["allow"] == "bus" else "#3a3a3a"
        ax.plot(xs, ys, color=color, lw=(lane["width"] - 0.3) * scale, solid_capstyle="butt", zorder=2)
    for lane_id, color in lanes:
        if lane_id in net.lanes:
            ax.plot(*zip(*net.lanes[lane_id]["points"]), color=color, lw=(net.lanes[lane_id]["width"] - 0.3) * scale,
                    solid_capstyle="butt", alpha=0.9, zorder=4)
    for lane_id, color in crossings:
        if lane_id in net.lanes:
            ax.plot(*zip(*net.lanes[lane_id]["points"]), color=color, lw=net.lanes[lane_id]["width"] * scale,
                    solid_capstyle="butt", alpha=0.85, zorder=5)
    for connection, color in paths:
        points = net.path(*connection)
        if points:
            ax.plot(*zip(*points), color=color, lw=2.6, zorder=6)
            ax.annotate("", xy=points[-1], xytext=points[-2], zorder=7,
                        arrowprops=dict(arrowstyle="-|>", color=color, lw=2.6, mutation_scale=18))
    if signals:
        for junction_id, junction in net.junctions.items():
            if junction["type"].startswith("traffic_light") and abs(junction["x"] - cx) < half and abs(junction["y"] - cy) < half * 0.72:
                ax.plot(junction["x"], junction["y"], "o", color="#ffd400", ms=9, mec="black", zorder=8)
                ax.annotate(junction_id, (junction["x"], junction["y"]), xytext=(7, 7), textcoords="offset points",
                            fontsize=11, fontweight="bold", color="black", zorder=9,
                            bbox=dict(boxstyle="round,pad=0.15", fc="#ffd400", ec="none"))
    for text, x, y, color in labels:
        ax.annotate(text, (x, y), fontsize=11.5, color="white", fontweight="bold", ha="center", va="center", zorder=10,
                    bbox=dict(boxstyle="round,pad=0.25", fc=color, ec="none"))


def figure(name, heading, before, after, notes):
    """기존/수정 후 두 칸과 그 아래 설명 칸. before와 after는 draw에 넘길 (net, center, half, title, 옵션)이다."""
    fig = plt.figure(figsize=(15, 9.4), dpi=140)
    fig.suptitle(heading, fontsize=20, fontweight="bold", y=0.975)
    axes = [fig.add_axes([0.03, 0.36, 0.455, 0.54]), fig.add_axes([0.515, 0.36, 0.455, 0.54])]
    fig.canvas.draw()
    for ax, (net, center, half, title, options) in zip(axes, (before, after)):
        draw(ax, net, center, half, title, **options)
    box = fig.add_axes([0.03, 0.02, 0.94, 0.31])
    box.axis("off")
    box.add_patch(plt.Rectangle((0, 0), 1, 1, fc="#f6f6f2", ec="#999999", transform=box.transAxes))
    y = 0.90
    for label, text in notes:
        box.text(0.015, y, label, fontsize=13.5, fontweight="bold", color="#b3261e", va="top", transform=box.transAxes)
        box.text(0.105, y, text, fontsize=13, va="top", linespacing=1.45, transform=box.transAxes)
        y -= 0.085 + 0.115 * (text.count("\n") + 1) - 0.03
    box.text(0.985, 0.03, "노란 점은 신호 교차로, 회색은 인도, 파랑은 버스 전용 차로, 흰 줄무늬는 횡단보도",
             fontsize=10, color="#555555", ha="right", transform=box.transAxes)
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / f"{name}.png")
    plt.close(fig)
    print(OUT / f"{name}.png")


def main():
    font_manager.fontManager.addfont(FONT)
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    old, ported, new = Net(load(V2)), Net(load(PORTED)), Net(load(None))

    figure("01_seogyo_junction", "① 서교동사거리(J42): 따로 놀던 신호 교차로 4개를 하나로",
           (old, (403, 372), 62, "기존 (2.0.0)", dict(lanes=[(f"R55_{i}", RED) for i in range(1, 5)],
                                                    labels=[("R55 길이 0.2m", 432, 402, RED)])),
           (new, (403, 372), 62, "수정 후 (3.0.0)", {}),
           [("문제", "양화로 진입로 R55에서 비상 정지 40번(9회 실행). 충돌 3번도 모두 이 사거리의 횡단보도였다."),
            ("원인", "정지선마다 신호 교차로(J32·J44·J45)가 따로 있어 한 사거리에서 신호를 두 번 받았다.\n"
                     "동쪽 진입로 R55는 길이가 0.2m라, 투입된 차가 곧바로 빨간불을 만나 멈출 거리가 없었다."),
            ("수정", "서예인의 새 원본(seogyo-hongdae 7580004)으로 바꿨다. J42 하나로 합치고 양화로를 홍대입구역사거리까지 늘렸다."),
            ("근거", "scripts/audit_road.py 9회 실행에서 R55 비상 정지 40번 → 0번.")])

    figure("02_sangsu_left_turn", "② 상수역 좌회전(J82): 바깥 차로에서만 좌회전하던 곳",
           (old, (808, -172), 46, "기존 (2.0.0)", dict(lanes=[("R111_2", RED), ("R111_3", RED)],
                                                    paths=[(("R111", "1", "R112", "1"), GREEN)],
                                                    labels=[("연결 없는 차로 2개", 790, -150, RED)])),
           (new, (808, -172), 46, "수정 후 (3.0.0)", dict(paths=[(("R111b", "1", "R112", "1"), GREEN),
                                                               (("R111b", "2", "R112", "2"), GREEN)])),
           [("문제", "버스가 좌회전 차로가 아닌 맨 바깥 차로에서 좌회전했다(주현진 제보)."),
            ("원인", "독막로 R111의 차도 3차로 중 1번 차로만 와우산로 R112로 이어졌다. 2·3번 차로(빨강)는 다음 도로로 가는 연결이 없었다."),
            ("수정", "새 원본에서 R111을 상수역 앞에서 2차로(R111b)로 줄이고, 두 차로 모두 2차로가 된 와우산로로 좌회전하게 했다."),
            ("근거", "audit_road.py의 '연결 없는 차로'에서 R111_2, R111_3이 사라졌다. 버스 ego 9회 모두 완주.")])

    walk = [(new.crossing("R53", "R55"), GREEN), (new.crossing("R54a", "R50"), GREEN)]
    turns = [(("R94", "2", "R54a", "1"), ORANGE), (("-R92", "2", "R53", "1"), ORANGE),
             (("R94", "1", "R53", "1"), ORANGE), (("-R92", "1", "R54a", "1"), ORANGE)]
    figure("03_seogyo_pedestrian_phase", "③ 서교동사거리(J42) 신호: 양화로 횡단보도를 보행자 전용 단계로",
           (new, (403, 372), 50, "기존 신호: 잔다리로 차량과 횡단보도가 같은 녹색", dict(crossings=walk, paths=turns)),
           (new, (403, 372), 50, "수정 후: 차량이 모두 멈춘 보행자 전용 단계(34초)", dict(crossings=walk)),
           [("문제", "잔다리로에서 양화로로 좌회전하던 버스가 횡단보도 보행자와 충돌 5번(9회 실행). 버스 연구 경로 위의 좌회전이다."),
            ("원인", "양화로를 건너는 긴 횡단보도 두 곳(36m, 39m, 초록)이 잔다리로 차량 신호와 같이 켜졌다.\n"
                     "좌회전·우회전한 차(주황)가 교차로를 빠져나가는 자리가 바로 이 횡단보도다."),
            ("수정", "두 횡단보도를 잔다리로 단계에서 빼고, 차량이 모두 빨간불인 전용 단계로 옮겼다. 앞뒤로 전적색 5초를 둔다.\n"
                     "앞의 전적색은 교차로 안에서 기다리던 좌회전 차가 빠져나갈 시간이다(없을 때 충돌 1번)."),
            ("근거", "버스 충돌 5번 → 0번. 주기가 121초에서 165초로 늘어 버스 한 바퀴는 약 25초 길어졌다.")])

    figure("04_hapjeong_junction_merge", "④ 합정역 사거리(J1): 교차로 안에서 두 차로가 한 차로로",
           (ported, (-14, -8), 40, "기존 (새 원본 그대로)", dict(paths=[(("R42", "1", "R40", "1"), RED),
                                                                    (("R42", "2", "R40", "1"), ORANGE)])),
           (new, (-14, -8), 40, "수정 후", dict(paths=[(("R42", "2", "R40", "1"), GREEN), (("R42", "3", "R40", "2"), GREEN),
                                                    (("R42", "1", "R59", "1"), GREEN)])),
           [("문제", "교차로 안 측면 충돌 1번, 같은 자리(:J1_18_0)에서 급제동 8번(9회 실행)."),
            ("원인", "양화로 R42의 1번 차로(빨강)와 2번 차로(주황)가 둘 다 R40의 1번 차로로 직진했다.\n"
                     "R40은 맨 왼쪽이 버스 전용(파랑)이 되어 승용차 차로가 둘뿐인데, 직진 차로는 셋이었다."),
            ("수정", "R42 1번 차로의 직진 연결을 지워 우회전 전용으로 했다. 연결 번호가 밀려도 신호가 어긋나지 않게,\n"
                     "빌드가 신호 프로그램을 번호가 아니라 연결 기준으로 옮겨 적는다(양보 녹색도 도로 이름으로 찾는다)."),
            ("근거", "측면 충돌 1번 → 0번, :J1_18_0 급제동 8번 → 0번.")])

    figure("05_entry_stub", "⑤ 경계 진입로 -R97(J64 북쪽): 신호까지 10m뿐이던 진입로",
           (old, (172, 640), 46, "기존: 10m", dict(lanes=[("-R97_1", RED)], labels=[("-R97 10m", 150, 640, RED)])),
           (new, (172, 640), 46, "수정 후: 50m", dict(lanes=[("-R97_1", GREEN)], labels=[("-R97 50m", 140, 650, GREEN)])),
           [("문제", "-R97에서 비상 정지 19번(9회 실행)."),
            ("원인", "배경 차량이 도로망 경계에서 투입되는데, 신호 교차로 J64까지 10m뿐이었다.\n"
                     "투입 속도로는 빨간불 앞에서 멈출 수 없어 9m/s²로 비상 정지했다."),
            ("수정", "경계 노드 J66을 교차로에서 60m 떨어진 곳으로 옮겼다(직선 연장). 진입로가 50m가 됐다."),
            ("근거", "비상 정지 19번 → 0번.")])

    lost = new.crossing("R71", "-R71") or ""
    figure("06_dongmak_crossing", "⑥ 독막로 J23: 도로를 나누면서 사라지던 횡단보도",
           (old, (417, -158), 30, "기존: 서쪽 횡단보도 없음", dict(labels=[("횡단보도 없음", 404, -150, RED)])),
           (new, (417, -158), 30, "수정 후", dict(crossings=[(lane, GREEN) for lane in new.crossings
                                                          if lane.startswith(":J23_") and new.crossings[lane] == {"R71", "-R71"}])),
           [("문제", "J23의 서쪽(독막로) 횡단보도가 실행 도로망에 없었다. 새 원본의 J23 신호 프로그램(20칸)과 연결 수(19개)도 맞지 않았다."),
            ("원인", "골목이 독막로에 합류하는 곳을 만들려고 R71을 나누면 조각의 이름이 R71.256으로 바뀐다.\n"
                     "netconvert는 이름이 없어진 도로를 건너던 횡단보도를 말없이 버린다."),
            ("수정", "원본 횡단보도가 건너던 도로의 조각 중 그 교차로에 닿는 것으로 횡단보도를 다시 놓는다(build_hapjeong.py)."),
            ("근거", "J23 연결 20개가 원본과 같은 순서로 맞는다. 맞지 않으면 빌드가 멈춘다.")])


if __name__ == "__main__":
    main()
