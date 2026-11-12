"""연구 시나리오가 수집 데이터에서 일어난 비율을 슬라이드 한 장으로 만든다.

    python3 docs/data_summary/scenario_slide.py data/hapjeong/bus_loop_rule_busy40.jsonl.gz
"""
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from scenario_occurrence import main as count

FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
INK, GRAY, GREEN, RED = "#1c1c1c", "#6b6b6b", "#2e7d4f", "#c0392b"
# 일어나지 않은 이유. S5와 S6의 숫자는 scripts/scenario_supply.py(배경 교통 30분)에서 셌다.
WHY = {
    "S1": "버스가 정류장에 서지 않는다 (정차 미구현)",
    "S2": "차로는 매번 바꾸지만, 옮겨 갈 차로에 차가 있던 적은 0번",
    "S3": "정류장에 서지 않으니 정류장 쪽으로 차로를 바꾸지 않는다",
    "S4": "정차 택시(traffic/)가 수집 환경에 연결되어 있지 않다",
    "S5": "골목 차 30분에 22대, 모두 버스 반대 방향으로 합류",
    "S6": "이 횡단보도를 건넌 보행자가 30분에 8명뿐",
}

result = count(sys.argv[1])
episodes = result["episodes"]
rows = {row["id"]: row for row in result["scenarios"]}
happened = sum(rows[key]["episodes"] > 0 for key in WHY)

fig = plt.figure(figsize=(16, 9), dpi=120)
fig.text(0.05, 0.90, f"계획한 시나리오 6개 중 수집 데이터에서 일어난 것은 {happened}개", fontsize=30, fontweight="bold", color=INK)
fig.text(0.05, 0.845, f"합정역 bus_loop, 규칙으로 달리는 버스, 교통량 busy, {episodes}에피소드 {result['steps']:,}스텝",
         fontsize=17, color=GRAY)
fig.add_artist(plt.Line2D([0.05, 0.95], [0.815, 0.815], color="#cccccc", lw=1.2))
for x, text in ((0.05, "시나리오"), (0.42, f"일어난 에피소드 (전체 {episodes})"), (0.62, "이유")):
    fig.text(x, 0.77, text, fontsize=15, color=GRAY)
for index, key in enumerate(WHY):
    y = 0.695 - index * 0.092
    row = rows[key]
    on = row["episodes"] > 0
    if index % 2 == 0:
        fig.add_artist(plt.Rectangle((0.04, y - 0.045), 0.92, 0.09, fc="#f6f6f3", ec="none"))
    fig.text(0.05, y, key, fontsize=22, fontweight="bold", color=INK, va="center")
    fig.text(0.095, y, row["name"], fontsize=20, color=INK, va="center")
    fig.add_artist(plt.Rectangle((0.42, y - 0.016), 0.10, 0.032, fc="#e3e3e0", ec="none"))
    fig.add_artist(plt.Rectangle((0.42, y - 0.016), 0.10 * row["episodes"] / episodes, 0.032, fc=GREEN if on else RED, ec="none"))
    fig.text(0.535, y, f"{row['episodes']}/{episodes}", fontsize=22, fontweight="bold", color=GREEN if on else RED, va="center")
    fig.text(0.62, y, WHY[key], fontsize=17, color=INK, va="center")
fig.add_artist(plt.Rectangle((0.05, 0.05), 0.90, 0.105, fc="#fdf1ef", ec="none"))
fig.text(0.07, 0.102, "결론", fontsize=19, fontweight="bold", color=RED, va="center")
fig.text(0.13, 0.102, "지금 수집하면 시나리오가 없는 주행 데이터가 된다. 정류장 정차와 시나리오를 넣는 장치가 먼저다.",
         fontsize=19, color=INK, va="center")
out = Path(__file__).with_name("scenario_occurrence.png")
fig.savefig(out)
print(out)
