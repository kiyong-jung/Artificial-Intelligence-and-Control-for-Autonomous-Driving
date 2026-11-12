"""데이터 수집 점검을 슬라이드 한 장으로 만든다. 숫자는 수집한 JSONL에서 센다.

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
INK, GRAY, GREEN, RED = "#1c1c1c", "#6b6b6b", "#2e7d4f", "#c0392b"
# 가속도를 고르게 적용하기 전에 잰 jerk 위반 비율(1,210스텝 중 786스텝). 그 데이터는 남기지 않았다.
JERK_BEFORE = 100 * 786 / 1210

rows = [json.loads(line) for line in gzip.open(sys.argv[1], "rt", encoding="utf-8")]
total, episodes = len(rows), len({row["episode"] for row in rows})
arrived = sum(row["event"]["termination_reason"] == "arrived" for row in rows)
accel = [row["state"]["ego"]["acceleration"] for row in rows]
jerk = 100 * sum(bool(row["comfort"]["jerk_violation"]) for row in rows) / total
met = [("같은 차로·옆 차로에\n차가 있던 시간", sum(any(d["lane_relation"] != "other" for d in r["detected_vehicles"]) for r in rows)),
       ("50m 안에\n차가 있던 시간", sum(bool(r["detected_vehicles"]) for r in rows)),
       ("50m 안에\n보행자가 있던 시간", sum(bool(r["detected_pedestrians"]) for r in rows))]

fig = plt.figure(figsize=(16, 9), dpi=120)
fig.text(0.05, 0.90, "데이터 수집은 된다. 다만 버스가 다른 차를 거의 만나지 않는다.", fontsize=30, fontweight="bold", color=INK)
fig.text(0.05, 0.845, f"합정역 bus_loop, 버스 ego, 교통량 busy, {episodes}에피소드 {total:,}스텝", fontsize=17, color=GRAY)
fig.add_artist(plt.Line2D([0.05, 0.95], [0.815, 0.815], color="#cccccc", lw=1.2))

fig.text(0.05, 0.755, "확인한 것", fontsize=20, fontweight="bold", color=GREEN)
checks = [(f"{arrived}/{episodes}", "에피소드가 도착으로 끝남"),
          ("26개", "컬럼이 모든 행에 있음 (수업 필수 19 + 팀 추가 7)"),
          (f"{min(accel):.1f} ~ +{max(accel):.1f}", "버스 가속도(m/s²)가 제원 한계 안"),
          (f"{JERK_BEFORE:.0f}% → {jerk:.0f}%", "jerk 한계를 넘는 스텝 (환경을 고친 뒤)")]
for index, (value, label) in enumerate(checks):
    y = 0.665 - index * 0.125
    fig.text(0.05, y, value, fontsize=32, fontweight="bold", color=INK, va="center")
    fig.text(0.05, y - 0.047, label, fontsize=16, color=GRAY, va="center")

fig.add_artist(plt.Line2D([0.47, 0.47], [0.20, 0.78], color="#cccccc", lw=1.2))
fig.text(0.51, 0.755, "문제", fontsize=20, fontweight="bold", color=RED)
ax = fig.add_axes([0.685, 0.25, 0.255, 0.46])
values = [100 * count / total for _, count in met]
bars = ax.barh(range(len(met)), values, color=[RED, "#b9b9b9", "#b9b9b9"], height=0.55)
for bar, value, color in zip(bars, values, (RED, INK, INK)):
    ax.text(value + 2.5, bar.get_y() + bar.get_height() / 2, f"{value:.1f}%" if value < 10 else f"{value:.0f}%",
            va="center", fontsize=26, fontweight="bold", color=color)
ax.set_yticks(range(len(met)), [name for name, _ in met], fontsize=17, color=INK)
ax.set_xlim(0, 112), ax.invert_yaxis(), ax.set_xticks([])
ax.tick_params(length=0)
for side in ("top", "right", "bottom", "left"):
    ax.spines[side].set_visible(False)

fig.add_artist(plt.Rectangle((0.05, 0.05), 0.90, 0.105, fc="#f4f4f0", ec="none"))
fig.text(0.07, 0.102, "다음", fontsize=19, fontweight="bold", color=RED, va="center")
fig.text(0.13, 0.102, "버스 앞에 차를 넣는 장치와 학습된 정책이 있어야 상호작용 데이터가 된다. 정류장 정차도 아직 없다.",
         fontsize=19, color=INK, va="center")
out = Path(__file__).with_name("data_collection.png")
fig.savefig(out)
print(out)
