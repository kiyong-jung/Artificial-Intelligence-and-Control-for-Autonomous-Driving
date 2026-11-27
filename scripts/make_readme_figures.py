"""README와 Hugging Face 카드에 넣는 그림 네 장을 docs/media/figs/에 다시 그린다.

    python scripts/make_readme_figures.py

- pipeline.png: 도로·교통부터 Hugging Face 공개까지의 흐름. 단계 이름과 숫자는 README의 "도로 구현", "AV 학습 설계",
  "DAgger와 커리큘럼", "최종 모델 고르기", "평가 결과", "데이터셋"과 research/av_bc_rl/train_final.sh에서 옮겼다.
- network.png: 신경망 구조. 입력 이름과 가린 15개는 env/data_schema.py(OBSERVATION_FEATURES, BLIND_BUNDLES["compact"])에서,
  층 크기는 models/final/model.pt와 config.json(state_dim 46, hidden 256, depth 2, accel_bins 11)에서 읽는다.
- results.png: 평가 13,000바퀴의 교통별 완주율과 끝난 원인. README "평가 결과" 표의 횟수에서 계산한다.
- dataset.png: 교통별 건수와 걸러 내기 전후. README "데이터셋" 표의 숫자다.
"""

import io
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle
from matplotlib.ticker import FixedLocator, FuncFormatter, NullLocator
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from env.data_schema import BLIND_BUNDLES, OBSERVATION_FEATURES  # noqa: E402

OUT = ROOT / "docs" / "media" / "figs"
STYLE = Path.home() / ".claude/skills/scientific-visualization/assets/nature.mplstyle"
KOREAN_FONT = Path("/System/Library/Fonts/AppleSDGothicNeo.ttc")

ACCENT, DEEP, STOP, INK = "#0a58b5", "#08458f", "#f5b400", "#1a1f2b"
MUTED, GREY, LIGHT, FAINT = "#556577", "#9aa5b1", "#d5dbe2", "#eef1f5"
TRAFFIC_BLUES = ["#c6dbf2", "#8fb9e6", "#4f8fd6", ACCENT, DEEP]

WIDTH_IN, DPI = 8.0, 200

TRAFFIC = ["한산", "보통", "혼잡", "매우 혼잡", "복합"]
LAPS_PER_TRAFFIC = 2600
COMPLETED = [2594, 2544, 2519, 2471, 2297]
OUTCOMES = [("완주", 12425, "ok"), ("멈춰 섬", 357, "stuck"), ("시간 초과", 107, "stuck"),
            ("차와 충돌", 76, "safety"), ("적색 통과", 25, "safety"), ("보행자 충돌", 10, "safety")]
COLLECTED_ROWS = [2328315, 2381523, 2400338, 2447855, 2627985]
KEPT_LAPS = [2540, 2447, 2373, 2308, 2042]
KEPT_ROWS = [2273248, 2232677, 2170971, 2145496, 2042382]
COURSE_ROW_TARGET = 10_000_000


def setup_style():
    if STYLE.exists():
        plt.style.use(str(STYLE))
    family = "sans-serif"
    if KOREAN_FONT.exists():
        font_manager.fontManager.addfont(str(KOREAN_FONT))
        family = font_manager.FontProperties(fname=str(KOREAN_FONT)).get_name()
    plt.rcParams.update({
        "font.family": family, "font.size": 10, "axes.labelsize": 10, "axes.titlesize": 10,
        "xtick.labelsize": 9.5, "ytick.labelsize": 9.5, "legend.fontsize": 9.5,
        "axes.linewidth": 0.8, "xtick.major.width": 0.8, "ytick.major.width": 0.8,
        "xtick.major.size": 3.5, "ytick.major.size": 3.5, "axes.edgecolor": INK,
        "text.color": INK, "axes.labelcolor": INK, "xtick.color": INK, "ytick.color": INK,
        "axes.unicode_minus": False, "figure.facecolor": "white", "savefig.facecolor": "white",
    })


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", dpi=DPI)
    plt.close(fig)
    image = Image.open(buffer).convert("RGB")
    path = OUT / name
    image.quantize(colors=256, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE).save(path, optimize=True)
    print(f"{path.relative_to(ROOT)}  {image.size[0]}x{image.size[1]}px  {path.stat().st_size / 1024:.0f}KB")


def canvas(height_in, xmax=100.0):
    fig = plt.figure(figsize=(WIDTH_IN, height_in))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, xmax)
    ax.set_ylim(0, xmax * height_in / WIDTH_IN)
    ax.set_axis_off()
    return fig, ax


def box(ax, x, y, w, h, face="white", edge=INK, lw=1.0, radius=0.8, **kw):
    patch = FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={radius}",
                           facecolor=face, edgecolor=edge, linewidth=lw, **kw)
    ax.add_patch(patch)
    return patch


def arrow(ax, start, end, color=INK, lw=1.4, style="-|>", ls="-", connection="arc3", **kw):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=11, color=color, linewidth=lw,
                                 linestyle=ls, connectionstyle=connection, shrinkA=0, shrinkB=0, **kw))


def step_box(ax, x, y, w, h, number, title, lines, face="white", edge=INK, title_color=INK):
    box(ax, x, y, w, h, face=face, edge=edge, lw=1.1)
    ax.add_patch(Rectangle((x + 1.2, y + h - 3.0), 2.2, 2.2, facecolor=STOP, edgecolor=INK, linewidth=0.9))
    ax.text(x + 2.3, y + h - 1.9, str(number), ha="center", va="center", fontsize=9, fontweight="bold")
    ax.text(x + 4.3, y + h - 1.9, title, ha="left", va="center", fontsize=10.5, fontweight="bold", color=title_color)
    ax.text(x + 1.4, y + h - 4.6, "\n".join(lines), ha="left", va="top", fontsize=9, color=INK, linespacing=1.45)


def network_layout():
    names = list(OBSERVATION_FEATURES)
    blind = set(BLIND_BUNDLES["compact"])
    lanes = [("left2", "왼쪽 두 칸"), ("left1", "왼쪽 한 칸"), ("current", "내 차로"), ("right1", "오른쪽 한 칸"),
             ("right2", "오른쪽 두 칸")]
    fields = [("leader_gap", "앞차\n간격"), ("leader_relative_speed", "앞차\n속도차"), ("follower_gap", "뒤차\n간격"),
              ("follower_relative_speed", "뒤차\n속도차"), ("density", "밀도"), ("route_connectivity", "노선\n연결")]
    grid = [[f"{lane}_{field}" for field, _ in fields] for lane, _ in lanes]
    rows = [("자차(속도, 가속도)", ["ego_speed", "ego_acceleration"]),
            ("앞차 100m", ["current_leader_gap_far"]),
            ("신호", ["tl_distance", "tl_red", "tl_yellow", "tl_yield", "tl_yellow_elapsed"]),
            ("정류장", ["stop_distance", "stop_distance_near", "stop_dwell_remaining"]),
            ("횡단보도", ["crosswalk_distance", "crosswalk_occupied"]),
            ("제한속도", ["speed_limit", "next_speed_limit", "next_speed_limit_distance"])]
    placed = [n for row in grid for n in row] + [n for _, row in rows for n in row]
    assert sorted(placed) == sorted(names) and len(names) == 46, "관측 이름이 data_schema와 다르다"
    assert blind <= set(names) and len(blind) == 15
    return lanes, fields, grid, rows, blind


def model_sizes():
    config = json.loads((ROOT / "models/final/config.json").read_text())
    sizes = {"state_dim": config["state_dim"], "accel_bins": config["accel_bins"], "hidden": 256, "depth": 2}
    try:
        import torch
        saved = torch.load(ROOT / "models/final/model.pt", map_location="cpu", weights_only=False)
        sizes.update(hidden=saved["hidden"], depth=saved["depth"])
        assert saved["blocks"] == 0 and saved["head"] == "bins"
    except ImportError:
        pass
    assert sizes["depth"] == 2
    return sizes




def draw_pipeline():
    fig, ax = canvas(4.15)
    top, bottom, h, h2, gap = 25.5, 1.5, 21.0, 16.5, 2.6
    widths_top = [20.0, 16.5, 14.5, 39.2]
    widths_bottom = [22.0, 23.6, 21.6, 23.0]
    xs = [1.0 + sum(widths_top[:k]) + k * gap for k in range(4)]
    xb = [1.0 + sum(widths_bottom[:k]) + k * gap for k in range(4)]

    step_box(ax, xs[0], top, widths_top[0], h, 1, "합정역 도로·교통",
             ["OSM 실제 도로를 SUMO로", "노선 1,963m", "정류장 4곳, 좌회전 4곳", "교통 다섯 단계"])
    step_box(ax, xs[1], top, widths_top[1], h, 2, "규칙 운전사",
             ["expert-v7, 교사 역할", "학습 때 정답만 붙임", "주행에는 쓰지 않음"], face=FAINT, edge=MUTED)
    step_box(ax, xs[2], top, widths_top[2], h, 3, "BC",
             ["모방학습 시작", "규칙 운전사의", "300바퀴 기록", "한산·보통·혼잡"])
    dx, dw = xs[3], widths_top[3]
    step_box(ax, dx, top, dw, h, 4, "DAgger 24회", ["신경망이 직접 주행, 회마다 교통별 25바퀴"], edge=ACCENT,
             title_color=DEEP)
    stages = [(12, "1~12회", "한산·보통·혼잡"), (4, "13~16회", "+매우 혼잡"),
              (5, "17~21회", "+택시·골목 차·보행자"), (3, "22~24회", "다섯 교통")]
    shades = ["#b9d2ef", "#7fafe3", "#3f84d1", DEEP]
    strip_x, strip_y, cell_h = dx + 1.4, top + 10.0, 2.2
    cell_w = (dw - 2.8) / 24
    i = 0
    for (count, name, traffic), shade, (col, row) in zip(stages, shades, [(0, 0), (1, 0), (0, 1), (1, 1)]):
        for k in range(count):
            ax.add_patch(Rectangle((strip_x + (i + k) * cell_w + 0.1, strip_y), cell_w - 0.2, cell_h,
                                   facecolor=shade, edgecolor="none"))
        i += count
        lx, ly = strip_x + col * 21.5, strip_y - 2.4 - row * 3.0
        ax.add_patch(Rectangle((lx, ly - 0.8), 1.6, 1.6, facecolor=shade, edgecolor="none"))
        ax.text(lx + 2.2, ly, f"{name} {traffic}", ha="left", va="center", fontsize=8.8)
    for k, label in [(1, "1"), (12, "12"), (16, "16"), (21, "21"), (24, "24")]:
        ax.text(strip_x + (k - 0.5) * cell_w, strip_y + cell_h + 0.4, label, ha="center", va="bottom",
                fontsize=8.6, color=MUTED)
    ax.text(dx + 1.4, top + 1.0, "최종 합본 2,707,818건", fontsize=9, va="bottom", color=MUTED)
    for k in range(3):
        arrow(ax, (xs[k] + widths_top[k] + 0.3, top + h / 2), (xs[k + 1] - 0.3, top + h / 2))
    arc_start, arc_end = (xs[1] + widths_top[1] * 0.55, top + h + 0.3), (dx + 4.0, top + h + 0.3)
    arrow(ax, arc_start, arc_end, color=MUTED, lw=1.0, ls=(0, (3, 2)), connection="arc3,rad=-0.12")
    ax.text((arc_start[0] + arc_end[0]) / 2, top + h + 2.6, "회마다 정답 붙이기", fontsize=8.8, color=MUTED,
            ha="center", va="bottom", bbox=dict(boxstyle="square,pad=0.15", facecolor="white", edgecolor="none"))

    step_box(ax, xb[0], bottom, widths_bottom[0], h2, 5, "최종 모델 고르기",
             ["21~24회 네 모델", "seed 63000, 교통마다 96바퀴", "안전 사건 8·26·13·6번", "가장 적은 24회 선택"])
    step_box(ax, xb[1], bottom, widths_bottom[1], h2, 6, "평가 겸 수집",
             ["학습에 안 쓴 seed", "13,000바퀴(교통마다 2,600)", "완주 95.6%, 충돌 0.66%", "0.5초마다 12,186,016건"])
    step_box(ax, xb[2], bottom, widths_bottom[2], h2, 7, "걸러 내기",
             ["실패·시뮬레이션 이상", "1,290바퀴를 바퀴 단위로 뺌", "11,710바퀴 남김"])
    step_box(ax, xb[3], bottom, widths_bottom[3], h2, 8, "Hugging Face 공개",
             ["10,864,774건", "65묶음, 7.4GB", "JSONL, NPZ, metadata"], face="#e8f0fa", edge=ACCENT,
             title_color=DEEP)
    for k in range(3):
        arrow(ax, (xb[k] + widths_bottom[k] + 0.3, bottom + h2 / 2), (xb[k + 1] - 0.3, bottom + h2 / 2))
    mid_y = (top + bottom + h2) / 2
    path_x, first_x = dx + dw / 2, xb[0] + widths_bottom[0] / 2
    ax.plot([path_x, path_x, first_x], [top - 0.3, mid_y, mid_y], color=INK, lw=1.4, solid_joinstyle="round")
    arrow(ax, (first_x, mid_y), (first_x, bottom + h2 + 0.3))
    save(fig, "pipeline.png")


def draw_network():
    lanes, fields, grid, rows, blind = network_layout()
    sizes = model_sizes()
    n_in, hidden, bins = sizes["state_dim"], sizes["hidden"], sizes["accel_bins"]
    n_out = bins + 3
    layer_weights = [n_in * hidden + hidden, hidden * hidden + hidden, hidden * n_out + n_out]
    trained = sum(layer_weights)
    side2 = sum(name.startswith(("left2_", "right2_")) for name in blind)
    follower = sum(name.startswith("current_follower") for name in blind)
    other = len(blind) - side2 - follower
    assert (side2, follower, other) == (12, 2, 1)

    height = 3.6
    fig, ax = canvas(height)
    H = 100 * height / WIDTH_IN
    gx, cw, ch = 15.0, 4.0, 2.15

    def cell(x, y, name):
        masked = name in blind
        ax.add_patch(Rectangle((x + 0.15, y), cw - 0.3, ch, facecolor=FAINT if masked else ACCENT,
                               edgecolor=GREY if masked else "none", linewidth=0.6,
                               hatch="////" if masked else None))

    ax.text(1.0, H - 2.0, f"관측 {n_in}개", fontsize=11, fontweight="bold", va="center")
    ax.text(10.6, H - 2.0, f"{n_in - len(blind)}개 사용, {len(blind)}개는 0으로 가림", fontsize=9.5, color=MUTED,
            va="center")
    head_y = H - 4.2
    for j, (_, label) in enumerate(fields):
        ax.text(gx + j * cw + cw / 2, head_y, label, ha="center", va="top", fontsize=8.6, linespacing=1.1)
    ax.text(gx - 0.8, head_y - 1.6, "차로", ha="right", va="center", fontsize=8.6, color=MUTED)
    gy0 = head_y - 4.4 - ch
    for i, (_, label) in enumerate(lanes):
        y = gy0 - i * (ch + 0.35)
        ax.text(gx - 0.8, y + ch / 2, label, ha="right", va="center", fontsize=9)
        for j in range(len(fields)):
            cell(gx + j * cw, y, grid[i][j])
    ry0 = gy0 - 4 * (ch + 0.35) - 1.4 - ch
    for i, (label, items) in enumerate(rows):
        y = ry0 - i * (ch + 0.5)
        ax.text(gx - 0.8, y + ch / 2, label, ha="right", va="center", fontsize=9)
        for j, name in enumerate(items):
            cell(gx + j * cw, y, name)
    route_bottom = ry0 - (len(rows) - 1) * (ch + 0.5)

    legend_y = 0.9
    ax.add_patch(Rectangle((1.0, legend_y), 2.2, 1.6, facecolor=ACCENT, edgecolor="none"))
    ax.text(3.7, legend_y + 0.8, "신경망이 보는 입력", fontsize=9, va="center")
    ax.add_patch(Rectangle((17.5, legend_y), 2.2, 1.6, facecolor=FAINT, edgecolor=GREY, hatch="////", lw=0.6))
    ax.text(20.2, legend_y + 0.8, f"0으로 가린 입력: 두 칸 옆 차로 {side2}, 내 차로 뒤차 {follower}, 비보호 신호 {other}",
            fontsize=9, va="center")

    grid_right = gx + len(fields) * cw
    mid = (gy0 + ch + route_bottom) / 2
    layers = [(42.0, 5.5, 11.0, "정규화", [f"{n_in}"], FAINT, MUTED),
              (50.5, 7.0, 24.0, "은닉층 1", [f"{hidden}", "ReLU"], ACCENT, ACCENT),
              (60.5, 7.0, 24.0, "은닉층 2", [f"{hidden}", "ReLU"], ACCENT, ACCENT),
              (70.5, 5.5, 9.0, "출력", [f"{n_out}"], DEEP, DEEP)]
    for idx, (x, w, h, title, body, face, edge) in enumerate(layers):
        y = mid - h / 2
        box(ax, x, y, w, h, face=face, edge=edge, lw=1.0, radius=0.6)
        dark = face != FAINT
        ax.text(x + w / 2, mid + (0.9 if len(body) > 1 else 0), body[0], ha="center", va="center", fontsize=11,
                fontweight="bold", color="white" if dark else INK)
        if len(body) > 1:
            ax.text(x + w / 2, mid - 1.6, body[1], ha="center", va="center", fontsize=9, color="white")
        ax.text(x + w / 2, y + h + 0.9, title, ha="center", va="bottom", fontsize=9.5, fontweight="bold")
        note = "평균·표준편차\n가린 입력은 0" if idx == 0 else f"가중치\n{layer_weights[idx - 1]:,}"
        ax.text(x + w / 2, y - 0.9, note, ha="center", va="top", fontsize=8.6, color=MUTED, linespacing=1.2)
    arrow(ax, (grid_right + 0.4, mid), (layers[0][0] - 0.4, mid))
    for a, b in zip(layers, layers[1:]):
        arrow(ax, (a[0] + a[1] + 0.3, mid), (b[0] - 0.3, mid))
    ax.text(99.0, H - 2.0, f"학습하는 가중치 {trained:,}개 (정규화 값 {2 * n_in}개를 더하면 {trained + 2 * n_in:,}개)",
            fontsize=9, color=MUTED, ha="right", va="center")

    out_x = layers[-1][0] + layers[-1][1]
    hx, bw = 80.0, 1.3
    top_y, low_y = mid + 6.5, mid - 8.0
    arrow(ax, (out_x + 0.3, mid + 1.5), (hx - 0.6, top_y + 1.0), connection="arc3,rad=-0.15")
    arrow(ax, (out_x + 0.3, mid - 1.5), (hx - 0.6, low_y + 1.0), connection="arc3,rad=0.15")
    ax.text(hx, top_y + 2.8, f"가감속 {bins}칸 점수", fontsize=9.5, fontweight="bold", va="bottom")
    for k in range(bins):
        ax.add_patch(Rectangle((hx + k * bw, top_y), bw - 0.2, 2.0, facecolor=DEEP if k == 7 else "#8fb9e6",
                               edgecolor="none"))
    ax.text(hx, top_y - 0.5, "-1", fontsize=8.6, va="top", color=MUTED)
    ax.text(hx + bins * bw - 0.2, top_y - 0.5, "+1", fontsize=8.6, va="top", ha="right", color=MUTED)
    ax.text(hx, top_y - 3.0, "가장 높은 칸과 양옆 칸 평균\n가감속 +1.0 ~ -2.0 m/s²", fontsize=8.8, va="top",
            linespacing=1.35)
    ax.text(hx, low_y + 2.8, "차로 변경 3갈래 점수", fontsize=9.5, fontweight="bold", va="bottom")
    for k, label in enumerate(["오른쪽", "유지", "왼쪽"]):
        ax.add_patch(Rectangle((hx + k * 5.4, low_y), 5.0, 2.0, facecolor=DEEP if k == 1 else "#8fb9e6",
                               edgecolor="none"))
        ax.text(hx + k * 5.4 + 2.5, low_y + 1.0, label, ha="center", va="center", fontsize=8.6,
                color="white" if k == 1 else INK)
    ax.text(hx, low_y - 0.8, "가장 높은 것 하나 {-1, 0, +1}", fontsize=8.8, va="top")
    save(fig, "network.png")


def draw_results():
    fig = plt.figure(figsize=(WIDTH_IN, 3.4))
    ax_a = fig.add_axes([0.075, 0.2, 0.355, 0.68])
    ax_b = fig.add_axes([0.63, 0.2, 0.345, 0.68])

    rates = [100 * c / LAPS_PER_TRAFFIC for c in COMPLETED]
    overall = 100 * sum(COMPLETED) / (LAPS_PER_TRAFFIC * len(COMPLETED))
    assert [round(r, 1) for r in rates] == [99.8, 97.8, 96.9, 95.0, 88.3] and round(overall, 1) == 95.6
    xs = range(len(TRAFFIC))
    ax_a.bar(xs, rates, width=0.62, color=ACCENT, edgecolor="none", zorder=2)
    for x, r in zip(xs, rates):
        ax_a.text(x, r - 2.0, f"{r:.1f}", ha="center", va="top", fontsize=9.5, fontweight="bold", color="white",
                  zorder=3)
    ax_a.axhline(overall, color=INK, lw=1.0, ls=(0, (4, 2)), zorder=1)
    ax_a.text(4.5, overall, f" 전체\n {overall:.1f}%", ha="left", va="center", fontsize=9.5, clip_on=False)
    ax_a.set_xlim(-0.5, 4.5)
    ax_a.set_xticks(list(xs), TRAFFIC)
    ax_a.set_ylim(0, 105)
    ax_a.set_yticks([0, 20, 40, 60, 80, 100])
    ax_a.set_ylabel("완주율 (%)")
    ax_a.set_xlabel(f"교통 (각 {LAPS_PER_TRAFFIC:,}바퀴)")
    ax_a.spines["left"].set_bounds(0, 100)

    labels = [o[0] for o in OUTCOMES][::-1]
    counts = [o[1] for o in OUTCOMES][::-1]
    kinds = [o[2] for o in OUTCOMES][::-1]
    assert sum(counts) == LAPS_PER_TRAFFIC * len(TRAFFIC) and counts[-1] == sum(COMPLETED)
    face = {"ok": ACCENT, "stuck": GREY, "safety": STOP}
    ax_b.barh(range(len(labels)), counts, left=1, height=0.62, color=[face[k] for k in kinds],
              edgecolor=[INK if k == "safety" else "none" for k in kinds], linewidth=0.7, zorder=2)
    ax_b.set_xscale("log")
    ax_b.set_xlim(1, 1.2e5)
    for y, c in enumerate(counts):
        ax_b.text(c * 1.25, y, f"{c:,}", va="center", ha="left", fontsize=9.5, fontweight="bold")
    ax_b.set_yticks(range(len(labels)), labels)
    ax_b.tick_params(axis="y", length=0)
    ax_b.xaxis.set_major_locator(FixedLocator([1, 10, 100, 1000, 10000]))
    ax_b.xaxis.set_minor_locator(NullLocator())
    ax_b.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{int(v):,}"))
    ax_b.set_xlabel("바퀴 수 (로그 눈금)")
    ax_b.spines["bottom"].set_bounds(1, 10000)
    legend = [Rectangle((0, 0), 1, 1, facecolor=face["ok"]), Rectangle((0, 0), 1, 1, facecolor=face["stuck"]),
              Rectangle((0, 0), 1, 1, facecolor=face["safety"], edgecolor=INK, linewidth=0.7)]
    ax_b.legend(legend, ["완주", "진행 못 함", "안전 사건"], loc="lower right", bbox_to_anchor=(1.02, 0.0),
                handlelength=1.0, handleheight=0.9, borderaxespad=0.2, labelspacing=0.3)

    fig.text(0.012, 0.95, "a", fontsize=13, fontweight="bold", va="top")
    fig.text(0.075, 0.95, "교통별 완주율", fontsize=10.5, fontweight="bold", va="top")
    fig.text(0.50, 0.95, "b", fontsize=13, fontweight="bold", va="top")
    fig.text(0.545, 0.95, f"{LAPS_PER_TRAFFIC * len(TRAFFIC):,}바퀴가 끝난 방식", fontsize=10.5, fontweight="bold",
             va="top")
    save(fig, "results.png")


def tint(color, amount):
    r, g, b = (int(color[i:i + 2], 16) for i in (1, 3, 5))
    return "#" + "".join(f"{round(c + (255 - c) * amount):02x}" for c in (r, g, b))


def draw_dataset():
    collected, kept = sum(COLLECTED_ROWS), sum(KEPT_ROWS)
    assert collected == 12_186_016 and kept == 10_864_774 and sum(KEPT_LAPS) == 11_710
    fig = plt.figure(figsize=(WIDTH_IN, 3.0))
    ax = fig.add_axes([0.155, 0.28, 0.80, 0.55])
    bars = [("모은 기록", f"{LAPS_PER_TRAFFIC * len(TRAFFIC):,}바퀴", COLLECTED_ROWS, None),
            ("공개 데이터", f"{sum(KEPT_LAPS):,}바퀴", KEPT_ROWS, KEPT_LAPS)]
    for (label, laps, rows, lap_counts), y in zip(bars, [1.0, 0.0]):
        left = 0
        for k, value in enumerate(rows):
            color = TRAFFIC_BLUES[k] if lap_counts else tint(TRAFFIC_BLUES[k], 0.55)
            ax.barh(y, value, left=left, height=0.62, color=color, edgecolor="white", linewidth=1.2, zorder=2)
            dark = k >= 3 and lap_counts is not None
            text = f"{value:,}" if lap_counts is None else f"{value:,}\n{lap_counts[k]:,}바퀴"
            ax.text(left + value / 2, y, text, ha="center", va="center", fontsize=9, linespacing=1.3,
                    color="white" if dark else INK, zorder=3)
            left += value
        ax.text(left + 1.2e5, y, f"{left:,}건", ha="left", va="center", fontsize=10, fontweight="bold")
        ax.text(-1.5e5, y + 0.12, label, ha="right", va="center", fontsize=10, fontweight="bold")
        ax.text(-1.5e5, y - 0.2, laps, ha="right", va="center", fontsize=9, color=MUTED)
    ax.axvline(COURSE_ROW_TARGET, color=INK, lw=1.1, ls=(0, (4, 2)), zorder=1)
    ax.text(COURSE_ROW_TARGET, 1.45, r"수업 기준 $10^7$건", ha="center", va="bottom", fontsize=9.5,
            bbox=dict(boxstyle="square,pad=0.2", facecolor="white", edgecolor="none"))
    ax.text(COURSE_ROW_TARGET + 2.5e5, 0.5, "실패·시뮬레이션 이상 1,290바퀴 뺌", ha="left", va="center", fontsize=8.8,
            color=MUTED)
    ax.set_xlim(0, 14.5e6)
    ax.set_ylim(-0.45, 1.75)
    ax.set_yticks([])
    ax.spines["left"].set_visible(False)
    ticks = [0, 2e6, 4e6, 6e6, 8e6, 10e6, 12e6]
    ax.set_xticks(ticks, ["0"] + [f"{t / 1e4:,.0f}만" for t in ticks[1:]])
    ax.spines["bottom"].set_bounds(0, 12e6)
    ax.set_xlabel("건수 (0.5초마다 한 건)")
    handles = [Rectangle((0, 0), 1, 1, facecolor=c) for c in TRAFFIC_BLUES]
    fig.legend(handles, TRAFFIC, loc="lower center", ncol=5, bbox_to_anchor=(0.555, 0.0), handlelength=1.2,
               handleheight=0.9, columnspacing=1.6, frameon=False)
    fig.text(0.012, 0.95, "교통별 건수와 걸러 내기 전후", fontsize=10.5, fontweight="bold", va="top")
    save(fig, "dataset.png")


def main():
    setup_style()
    draw_pipeline()
    draw_network()
    draw_results()
    draw_dataset()


if __name__ == "__main__":
    main()
