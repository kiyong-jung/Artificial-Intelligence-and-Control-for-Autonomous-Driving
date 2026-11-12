"""지름길을 고른 근거를 슬라이드 한 장으로 그린다. 숫자는 maps/hapjeong/references/shortcuts.json에서 읽는다.

    python3 scripts/render_shortcut_evidence.py
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt

BASE = Path(__file__).resolve().parents[1]
REFERENCES = BASE / "maps" / "hapjeong" / "references"
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
INK, GRAY, GREEN, RED, ORANGE = "#1c1c1c", "#6b6b6b", "#2e7d4f", "#c0392b", "#e65100"
SHORT = {"none": "지름길 없음", "dongmak7": "독막로7길", "jandari3": "잔다리로3길", "eoulmadang": "어울마당로",
         "dongmak7+jandari3": "최종(두 개)", "dongmak7+jandari3+eoulmadang": "셋 모두"}

data = json.loads((REFERENCES / "shortcuts.json").read_text(encoding="utf-8"))
rows = {row["shortcuts"]: row for row in data["rows"]}
base, final = rows["none"], rows["+".join(data["chosen"])]

fig = plt.figure(figsize=(16, 9), dpi=120)
fig.text(0.05, 0.90, f"지름길 두 개를 살렸다. 골목에서 버스 방향으로 합류하는 차가 시간당 "
         f"{base['s5_merges_per_hour']:.0f}대에서 {final['s5_merges_per_hour']:.0f}대로", fontsize=26, fontweight="bold", color=INK)
fig.text(0.05, 0.845, "버스 경로 안쪽의 실제 골목(OSM) 세 개를 하나씩, 그리고 묶어서 측정. 교통량 busy, 배경 교통 1시간 × 3회와 규칙 버스 80회 주행",
         fontsize=16, color=GRAY)
fig.add_artist(plt.Line2D([0.05, 0.95], [0.815, 0.815], color="#cccccc", lw=1.2))

fig.text(0.05, 0.765, "골목에서 버스와 같은 방향으로 합류하는 차 (대/시간)", fontsize=17, fontweight="bold", color=INK)
ax = fig.add_axes([0.16, 0.36, 0.25, 0.37])
keys = list(SHORT)
values = [rows[key]["s5_merges_per_hour"] for key in keys]
colors = ["#b9b9b9" if rows[key]["verdict"] == "기준선" else ORANGE if rows[key]["verdict"] == "채택" else "#d9c3b0" for key in keys]
bars = ax.barh(range(len(keys)), values, color=colors, height=0.62)
for bar, value in zip(bars, values):
    ax.text(value + 1.0, bar.get_y() + bar.get_height() / 2, f"{value:.1f}", va="center", fontsize=20, fontweight="bold", color=INK)
ax.set_yticks(range(len(keys)), [SHORT[key] for key in keys], fontsize=16)
ax.set_xlim(0, 56), ax.invert_yaxis(), ax.set_xticks([]), ax.tick_params(length=0)
for side in ax.spines.values():
    side.set_visible(False)

fig.add_artist(plt.Line2D([0.47, 0.47], [0.33, 0.78], color="#cccccc", lw=1.2))
fig.text(0.49, 0.765, "버스가 실제로 겪은 것 (규칙 버스 80회)과 안전", fontsize=17, fontweight="bold", color=INK)
for text, x in (("끼어든 차(S5)", 0.665), ("정류장에서 옆에 차", 0.775), ("사건 난 실행(300회 점검)", 0.90)):
    fig.text(x, 0.715, text, fontsize=14, color=GRAY, ha="center")
for index, key in enumerate(keys):
    row = rows[key]
    y = 0.66 - index * 0.058
    color = ORANGE if key == final["shortcuts"] else GRAY if row["verdict"] in ("기준선", "보류") else INK
    weight = "bold" if key == final["shortcuts"] else "normal"
    fig.text(0.49, y, SHORT[key], fontsize=16.5, color=color, fontweight=weight, va="center")
    fig.text(0.665, y, f"{row['s5_cut_ins']}번", fontsize=16.5, color=color, fontweight=weight, ha="center", va="center")
    share = 100 * row["stop_passes_with_vehicle_in_lane"] / row["stop_passes"]
    fig.text(0.775, y, f"{share:.0f}%", fontsize=16.5, color=color, fontweight=weight, ha="center", va="center")
    audit = row.get("audit_300_runs") or (data["audit_300_runs_reference"]["자르기만(3.1.0)"] if key == "none" else None)
    fig.text(0.90, y, f"{audit['runs_with_events']} / {audit['runs']}" if audit else "-", fontsize=16.5, color=color,
             fontweight=weight, ha="center", va="center")

fig.add_artist(plt.Rectangle((0.05, 0.05), 0.90, 0.24, fc="#f4f4f0", ec="none"))
gain = lambda key: rows[key]["s5_merges_per_hour"] - base["s5_merges_per_hour"]
notes = [("독막로7길", GREEN, f"합류가 가장 많이 는다(시간당 +{gain('dongmak7'):.1f}대). 골목 차가 이 길로 잔다리로까지 질러가 버스 앞으로 나온다."),
         ("잔다리로3길", GREEN, f"서교동교회 정류장 바로 앞으로 합류한다(시간당 +{gain('jandari3'):.1f}대). 끼어든 차 {rows['jandari3']['s5_cut_ins']}번이 모두 이 합류점이다."),
         ("어울마당로", RED, "끼어들기는 가장 많이 늘지만 보류. 신호 횡단보도 J74의 신호를 없애야 하고, 그 자리에서 골목 차와 보행자의 충돌이 새로 났다."),
         ("읽는 법", GRAY, "끼어든 횟수는 표본이 작다(같은 합류량에서 15번과 25번이 나왔다). 합류하는 차의 수가 더 믿을 만한 숫자다.")]
for index, (label, color, text) in enumerate(notes):
    y = 0.255 - index * 0.055
    fig.text(0.07, y, label, fontsize=16, fontweight="bold", color=color, va="center")
    fig.text(0.165, y, text, fontsize=15.5, color=INK, va="center")
out = REFERENCES / "shortcut_evidence.png"
fig.savefig(out)
print(out)
