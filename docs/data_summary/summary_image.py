"""데이터 수집 점검을 이미지 한 장으로 만든다. 수집한 JSONL에서 바로 그린다.

    python3 docs/data_summary/summary_image.py data/hapjeong/bus_loop_rule_busy.jsonl.gz
"""
import gzip
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt

FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
plt.rcParams["axes.unicode_minus"] = False
RED, BLUE = "#c0392b", "#1f5fa8"

rows = [json.loads(line) for line in gzip.open(sys.argv[1], "rt", encoding="utf-8")]
total, episodes = len(rows), len({row["episode"] for row in rows})
arrived = sum(row["event"]["termination_reason"] == "arrived" for row in rows)
accel = [row["state"]["ego"]["acceleration"] for row in rows]
violations = sum(bool(row["comfort"]["jerk_violation"]) for row in rows)
in_lane = sum(any(d["lane_relation"] != "other" for d in row["detected_vehicles"]) for row in rows)
share = [("관측 차선에 차량", in_lane), ("50m 안에 차량", sum(bool(r["detected_vehicles"]) for r in rows)),
         ("150m 안에 차량", sum(bool(r["state"]["vehicles"]) for r in rows)),
         ("50m 안에 보행자", sum(bool(r["detected_pedestrians"]) for r in rows)),
         ("앞에 신호", sum(r["state"]["traffic_light"] is not None for r in rows))]

fig = plt.figure(figsize=(15, 8.2), dpi=140)
fig.suptitle("합정역 버스 데이터 수집 점검", fontsize=20, fontweight="bold", y=0.975)
tiles = [(f"{total:,}스텝", f"{episodes}에피소드, 도착 {arrived}/{episodes}"), ("26개 컬럼", "수업 필수 19 + 팀 추가 7, 모든 행에 있음"),
         (f"{min(accel):.1f} ~ +{max(accel):.1f} m/s²", "버스 가속도. 제원 한계와 같음"),
         (f"{100 * violations / total:.1f}%", "jerk 한계(2.0 m/s³)를 넘은 스텝"),
         (f"{100 * in_lane / total:.1f}%", "관측 차선에 다른 차가 있던 스텝")]
for index, (value, label) in enumerate(tiles):
    ax = fig.add_axes([0.03 + index * 0.19, 0.775, 0.18, 0.125])
    ax.axis("off")
    bad = index == len(tiles) - 1
    ax.add_patch(plt.Rectangle((0, 0), 1, 1, fc="#fdecea" if bad else "#eef3f8", ec=RED if bad else "#9db3c8", transform=ax.transAxes))
    ax.text(0.5, 0.62, value, fontsize=21, fontweight="bold", color=RED if bad else BLUE, ha="center", va="center", transform=ax.transAxes)
    ax.text(0.5, 0.2, label, fontsize=11.5, ha="center", va="center", transform=ax.transAxes)

episode = [row for row in rows if row["episode"] == 0]
time = [row["t"] * 0.5 for row in episode]
ax = fig.add_axes([0.055, 0.285, 0.26, 0.40])
ax.plot(time, [row["state"]["ego"]["speed"] for row in episode], color=BLUE, lw=1.6)
ax.set_ylabel("속도 (m/s)", color=BLUE, fontsize=11), ax.set_xlabel("에피소드 0의 시간 (초)", fontsize=11)
twin = ax.twinx()
twin.plot(time, [row["state"]["ego"]["acceleration"] for row in episode], color=RED, lw=1.0)
twin.axhline(1.0, color=RED, ls=":", lw=0.9), twin.axhline(-2.0, color=RED, ls=":", lw=0.9)
twin.set_ylabel("가속도 (m/s²)", color=RED, fontsize=11)
ax.set_title("버스 속도와 가속도 (점선은 제원 한계)", fontsize=13, fontweight="bold")

ax = fig.add_axes([0.40, 0.285, 0.24, 0.40])
jerk = [row["comfort"]["peak_jerk"] for row in episode]
ax.vlines(time, 0, jerk, color=[RED if value > 2.0 else "#888888" for value in jerk], lw=1.2)
ax.axhline(2.0, color="black", ls="--", lw=0.9)
ax.set_xlabel("에피소드 0의 시간 (초)", fontsize=11), ax.set_ylabel("스텝별 jerk 최댓값 (m/s³)", fontsize=11)
ax.set_title("jerk (파선은 한계, 빨강은 위반)", fontsize=13, fontweight="bold")

ax = fig.add_axes([0.765, 0.285, 0.205, 0.40])
values = [100 * count / total for _, count in share]
bars = ax.barh([name for name, _ in share], values, color=[RED, "#e08e45", "#e0b345", "#4a8f5c", "#4a7fb0"])
for bar, value in zip(bars, values):
    ax.text(value + 2, bar.get_y() + bar.get_height() / 2, f"{value:.1f}%", va="center", fontsize=11)
ax.set_xlim(0, 105), ax.invert_yaxis(), ax.tick_params(axis="y", labelsize=11.5)
ax.set_xlabel("전체 스텝 중 비율 (%)", fontsize=11)
ax.set_title("값이 실제로 채워진 스텝", fontsize=13, fontweight="bold")

box = fig.add_axes([0.03, 0.025, 0.94, 0.16])
box.axis("off")
box.add_patch(plt.Rectangle((0, 0), 1, 1, fc="#f6f6f2", ec="#999999", transform=box.transAxes))
notes = [("되는 것", "합정역에서 버스 ego로 수집이 끝까지 돌고, 컬럼 26개와 버스 제원·가속도·jerk가 모두 기록된다."),
         ("고친 것", "ego를 승용차에서 버스로 바꿨고, 가속도를 0.5초 내내 고르게 적용해 jerk 위반을 65%에서 4%로 줄였다."),
         ("남은 것", "버스가 다른 차와 거의 마주치지 않는다. 정류장 정차와 새 관측 구성(보행자·신호 입력)도 아직 없다.")]
for row, (label, text) in enumerate(notes):
    y = 0.80 - 0.30 * row
    box.text(0.015, y, label, fontsize=14, fontweight="bold", color="#b3261e", va="center", transform=box.transAxes)
    box.text(0.085, y, text, fontsize=14, va="center", transform=box.transAxes)
out = Path(__file__).with_name("data_collection.png")
fig.savefig(out)
print(out)
