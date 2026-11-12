"""데이터 요약 PDF에 넣을 그림. 수집한 JSONL에서 바로 그린다.

    python docs/data_summary/figure.py data/hapjeong/bus_loop_rule_busy.jsonl.gz
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

rows = [json.loads(line) for line in gzip.open(sys.argv[1], "rt", encoding="utf-8")]
episode = [row for row in rows if row["episode"] == 0]
time = [row["t"] * 0.5 for row in episode]
fig, axes = plt.subplots(1, 3, figsize=(13, 3.3), dpi=200, gridspec_kw=dict(width_ratios=[1.25, 1.25, 1]))

ax = axes[0]
ax.plot(time, [row["state"]["ego"]["speed"] for row in episode], color="#1f5fa8", lw=1.4)
ax.set_ylabel("속도 (m/s)", color="#1f5fa8")
ax.set_xlabel("에피소드 0의 시간 (초)")
twin = ax.twinx()
twin.plot(time, [row["state"]["ego"]["acceleration"] for row in episode], color="#c0392b", lw=0.9)
twin.axhline(1.0, color="#c0392b", ls=":", lw=0.8), twin.axhline(-2.0, color="#c0392b", ls=":", lw=0.8)
twin.set_ylabel("가속도 (m/s²)", color="#c0392b")
ax.set_title("(a) 버스 속도와 가속도. 점선은 제원 한계 +1.0, -2.0", fontsize=10)

ax = axes[1]
jerk = [row["comfort"]["peak_jerk"] for row in episode]
ax.vlines(time, 0, jerk, color=["#c0392b" if value > 2.0 else "#888888" for value in jerk], lw=1.0)
ax.axhline(2.0, color="black", ls="--", lw=0.8)
ax.set_xlabel("에피소드 0의 시간 (초)"), ax.set_ylabel("comfort.peak_jerk (m/s³)")
ax.set_title("(b) 스텝별 jerk 최댓값. 파선은 한계 2.0, 빨강은 위반", fontsize=10)

ax = axes[2]
total = len(rows)
share = [
    ("관측 차선에 차량", sum(any(d["lane_relation"] != "other" for d in row["detected_vehicles"]) for row in rows)),
    ("50m 안에 차량", sum(bool(row["detected_vehicles"]) for row in rows)),
    ("150m 안에 차량", sum(bool(row["state"]["vehicles"]) for row in rows)),
    ("50m 안에 보행자", sum(bool(row["detected_pedestrians"]) for row in rows)),
    ("앞에 신호", sum(row["state"]["traffic_light"] is not None for row in rows)),
]
values = [100 * count / total for _, count in share]
bars = ax.barh([name for name, _ in share], values, color=["#c0392b", "#e08e45", "#e0b345", "#4a8f5c", "#4a7fb0"])
for bar, value in zip(bars, values):
    ax.text(value + 1.5, bar.get_y() + bar.get_height() / 2, f"{value:.1f}%", va="center", fontsize=9)
ax.set_xlim(0, 100), ax.invert_yaxis(), ax.set_xlabel(f"전체 {total:,}스텝 중 비율 (%)")
ax.set_title("(c) 값이 실제로 채워진 스텝", fontsize=10)

fig.tight_layout()
fig.savefig(Path(__file__).with_name("figure.png"))
print({name: count for name, count in share}, total,
      "jerk 위반", sum(bool(row["comfort"]["jerk_violation"]) for row in rows))
