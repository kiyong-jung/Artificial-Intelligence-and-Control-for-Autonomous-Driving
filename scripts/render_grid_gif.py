"""시나리오 S1~S6을 창 없이 돌려, 2x3 격자 GIF 한 장으로 저장한다.

    python scripts/render_grid_gif.py                         # results/traffic_verification/scenarios.gif
    python scripts/render_grid_gif.py --episodes S1=1,S2=2    # 장면별로 보여 줄 에피소드 지정
    python scripts/render_grid_gif.py --out my.gif --panel 480x360

SUMO는 헤드리스(sumo)로 돌리고, TraCI로 읽은 차량·보행자 위치를 Pillow로 직접 그린다.
카메라는 버스를 따라간다. 장면마다 길이가 다르면 짧은 쪽은 마지막 장면에서 멈춘다.
"""
import argparse
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(__file__))
import view_scenarios as vs  # noqa: E402
from traffic.sumo_conn import traci  # noqa: E402
import sumolib  # noqa: E402

TITLES = {"S1": "S1 정류장 출발 후 본선 재합류", "S2": "S2 정류장 직후 좌회전 연속 차선 변경",
          "S3": "S3 정류장 접근 연속 차선 변경", "S4": "S4 정류장 앞 택시 정차",
          "S5": "S5 합류구간 골목 차량 끼어들기", "S6": "S6 신호 없는 횡단보도"}
HALF = {"S1": 45, "S2": 45, "S3": 55, "S4": 40, "S5": 45, "S6": 28}      # 화면 세로 절반 [m]
DEFAULT_EP = {"S1": 0, "S2": 1, "S3": 2, "S4": 0, "S5": 0, "S6": 0}
FONT = "C:/Windows/Fonts/malgun.ttf"
FONT_B = "C:/Windows/Fonts/malgunbd.ttf"

BG, ROAD, BUSLANE, SIDEWALK, CROSS = (226, 228, 222), (84, 86, 92), (122, 70, 70), (206, 196, 176), (245, 245, 245)


def veh_color(vid):
    """차량 id로 색을 정한다(TraCI가 돌려주는 색은 유형 색을 반영하지 않는다)."""
    if vid == "BUS":
        return (220, 0, 0)
    if vid.startswith("nonyielder"):
        return (255, 140, 0)
    if vid.startswith("yielder"):
        return (0, 170, 0)
    if vid == "CUTIN":
        return (0, 90, 255)
    if vid == "TAXI":
        return (255, 220, 0)
    return (150, 150, 150)


def font(sz, bold=False):
    return ImageFont.truetype(FONT_B if bold else FONT, sz)


def collect(episodes, seed):
    """장면을 헤드리스로 돌려 {장면: (프레임 기록, 자막 줄)}을 돌려준다."""
    args = argparse.Namespace(headless=True, delay=0, trace=False)
    R = vs.Runner(args)
    out = {}
    for n in TITLES:
        R.rec = []
        vs.CAPTION.clear()
        ep = episodes[n]
        vs.SCENES[n](R, ep, seed + ep)
        out[n] = (list(R.rec), list(vs.CAPTION))
        print(f"  {n}: {len(R.rec)}프레임 ({R.rec[-1]['t']:.0f} s)")
    traci.close()
    return out


def bus_track(recs):
    """버스 위치를 따라가는 카메라 중심(지수 평활). 버스가 사라지면 마지막 위치를 유지한다."""
    cx = cy = None
    path = []
    for r in recs:
        b = next((v for v in r["v"] if v[0] == "BUS"), None)
        if b is not None:
            if cx is None:
                cx, cy = b[1], b[2]
            else:
                cx, cy = cx + 0.35 * (b[1] - cx), cy + 0.35 * (b[2] - cy)
        path.append((cx, cy))
    first = next(p for p in path if p[0] is not None)
    return [p if p[0] is not None else first for p in path]


def load_lanes(net_path, boxes):
    """장면별 화면 범위 안에 드는 차로만 골라 둔다."""
    net = sumolib.net.readNet(net_path, withInternal=True)
    lanes = []
    for e in net.getEdges():
        fn = e.getFunction()
        for ln in e.getLanes():
            sh = np.array(ln.getShape(), dtype=float)[:, :2]
            kind = ("cross" if fn == "crossing" else "side" if fn == "walkingarea"
                    else "side" if (ln.allows("pedestrian") and not ln.allows("passenger") and not ln.allows("bus"))
                    else "bus" if (ln.allows("bus") and not ln.allows("passenger")) else "road")
            lanes.append((sh, ln.getWidth(), kind, sh.min(0), sh.max(0)))
    sel = {}
    for n, (x0, y0, x1, y1) in boxes.items():
        sel[n] = [l for l in lanes if not (l[4][0] < x0 or l[3][0] > x1 or l[4][1] < y0 or l[3][1] > y1)]
    return sel


class Panel:
    SS = 2     # 확대해 그린 뒤 줄여서 부드럽게

    def __init__(self, name, recs, caption, lanes, w, h):
        self.name, self.recs, self.caption, self.lanes = name, recs, caption, lanes
        self.w, self.h = w, h
        self.cam = bus_track(recs)
        self.s = h * self.SS / (2 * HALF[name])             # 1 m당 픽셀(확대 좌표계)
        self.f = {k: font(k * self.SS) for k in (11, 12, 14)}
        self.fb = font(15 * self.SS, True)

    def to_px(self, x, y, cx, cy):
        return (self.w * self.SS / 2 + (x - cx) * self.s, self.h * self.SS / 2 - (y - cy) * self.s)

    def frame(self, i):
        i = min(i, len(self.recs) - 1)
        r, (cx, cy) = self.recs[i], self.cam[i]
        W, H, SS = self.w * self.SS, self.h * self.SS, self.SS
        im = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(im)
        hx, hy = W / 2 / self.s, H / 2 / self.s
        for sh, wd, kind, lo, hi in self.lanes:
            if hi[0] < cx - hx or lo[0] > cx + hx or hi[1] < cy - hy or lo[1] > cy + hy:
                continue
            pts = [self.to_px(x, y, cx, cy) for x, y in sh]
            col = {"road": ROAD, "bus": BUSLANE, "side": SIDEWALK, "cross": CROSS}[kind]
            wpx = max(2, int(wd * self.s - (0.0 if kind in ("side", "cross") else 3 * SS)))
            d.line(pts, fill=col, width=wpx, joint="curve")
            if kind == "cross":     # 얼룩말 무늬
                (xa, ya), (xb, yb) = pts[0], pts[-1]
                L = math.hypot(xb - xa, yb - ya)
                for k in range(int(L / (0.9 * self.s))):
                    t0, t1 = k * 0.9 * self.s / L, (k * 0.9 + 0.45) * self.s / L
                    d.line([(xa + (xb - xa) * t0, ya + (yb - ya) * t0), (xa + (xb - xa) * t1, ya + (yb - ya) * t1)],
                           fill=(120, 120, 120), width=wpx)
        blink_on = int(r["t"] * 2) % 2 == 0
        for vid, x, y, ang, L, Wd, col, sig in sorted(r["v"], key=lambda v: v[0] == "BUS"):
            a = math.radians(ang)
            hxv, hyv = math.sin(a), math.cos(a)
            nx, ny = hyv, -hxv
            fx, fy = x, y
            cxv, cyv = fx - hxv * L / 2, fy - hyv * L / 2
            poly = [self.to_px(cxv + hxv * L / 2 * u + nx * Wd / 2 * w, cyv + hyv * L / 2 * u + ny * Wd / 2 * w, cx, cy)
                    for u, w in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
            d.polygon(poly, fill=veh_color(vid), outline=(30, 30, 30))
            wf = [self.to_px(fx - hxv * L * 0.25 + nx * Wd / 2 * w * 0.85, fy - hyv * L * 0.25 + ny * Wd / 2 * w * 0.85, cx, cy)
                  for w in (1, -1)] + [self.to_px(fx + nx * Wd / 2 * w * 0.85, fy + ny * Wd / 2 * w * 0.85, cx, cy)
                                       for w in (-1, 1)]
            d.polygon(wf, fill=(40, 50, 70))                                     # 앞유리
            if sig & 3 and blink_on:
                for bit, side in ((1, 1), (2, -1)):                              # 오른쪽 1, 왼쪽 2
                    if sig & bit:
                        for u in (1, -1):
                            px, py = self.to_px(cxv + hxv * L / 2 * u * 0.95 + nx * Wd / 2 * side,
                                                cyv + hyv * L / 2 * u * 0.95 + ny * Wd / 2 * side, cx, cy)
                            rr = 0.45 * self.s
                            d.ellipse([px - rr, py - rr, px + rr, py + rr], fill=(255, 190, 0), outline=(120, 60, 0))
            px, py = self.to_px(cxv, cyv, cx, cy)
            tx = d.textlength(vid, font=self.f[11])
            d.text((px - tx / 2, py - 24 * SS), vid, font=self.f[11], fill=(20, 20, 20), stroke_width=2 * SS // 2,
                   stroke_fill=(255, 255, 255))
        for pid, x, y, col in r["p"]:
            px, py = self.to_px(x, y, cx, cy)
            rr = 0.8 * self.s
            d.ellipse([px - rr, py - rr, px + rr, py + rr], fill=tuple(col), outline=(0, 0, 0), width=SS)
            d.text((px + rr + 3, py - 14 * SS), pid, font=self.f[11], fill=(20, 20, 20), stroke_width=SS,
                   stroke_fill=(255, 255, 255))
        d.rectangle([0, 0, W, 26 * SS], fill=(35, 40, 52))
        d.text((8 * SS, 4 * SS), TITLES[self.name], font=self.fb, fill=(255, 255, 255))
        tt = f"t={r['t']:.1f}s"
        d.text((W - d.textlength(tt, font=self.f[12]) - 8 * SS, 6 * SS), tt, font=self.f[12], fill=(200, 210, 230))
        lines = self.caption[-4:]
        if lines:
            d.rectangle([0, H - (len(lines) * 16 + 8) * SS, W, H], fill=(255, 255, 255))
            for k, ln in enumerate(lines):
                d.text((6 * SS, H - (len(lines) - k) * 16 * SS - 6 * SS), ln, font=self.f[12], fill=(30, 30, 30))
        d.rectangle([0, 0, W - 1, H - 1], outline=(35, 40, 52), width=2 * SS)
        return im.resize((self.w, self.h), Image.LANCZOS)


LEGEND = [((220, 0, 0), "버스"), ((0, 170, 0), "양보형"), ((255, 140, 0), "비양보형"), ((0, 90, 255), "끼어드는 골목 차량"),
          ((255, 220, 0), "택시"), ((200, 0, 200), "보행자 P1(대기형)"), ((0, 160, 160), "보행자 P2(진입형)")]


def legend_bar(width, h=34):
    im = Image.new("RGB", (width, h), (245, 245, 245))
    d = ImageDraw.Draw(im)
    f = font(14)
    x = 12
    for col, txt in LEGEND:
        d.rectangle([x, 11, x + 18, 24], fill=col, outline=(30, 30, 30))
        d.text((x + 24, 7), txt, font=f, fill=(30, 30, 30))
        x += 24 + int(d.textlength(txt, font=f)) + 22
    d.text((x, 7), "· 노란 점 = 방향지시등 · 도로의 붉은 차로 = 중앙 버스 전용차로", font=f, fill=(90, 90, 90))
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(vs.ROOT, "results", "traffic_verification", "scenarios.gif"))
    ap.add_argument("--episodes", default="", help="예: S1=1,S3=0 (기본 S1=0,S2=1,S3=2,S4=0,S5=0,S6=0)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--panel", default="480x360", help="칸 한 개의 가로x세로 픽셀")
    ap.add_argument("--ms", type=int, default=100, help="프레임당 표시 시간 ms (한 프레임 = 0.4 s 시뮬레이션)")
    a = ap.parse_args()
    ep = dict(DEFAULT_EP)
    for kv in filter(None, a.episodes.split(",")):
        k, v = kv.split("=")
        ep[k.strip().upper()] = int(v)
    pw, ph = (int(x) for x in a.panel.lower().split("x"))

    print("시나리오 실행(창 없음):")
    data = collect(ep, a.seed)
    boxes = {}
    for n, (recs, _) in data.items():
        cam = bus_track(recs)
        xs, ys = [c[0] for c in cam], [c[1] for c in cam]
        m = HALF[n] * 2.2
        boxes[n] = (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)
    print("도로망 읽는 중...")
    lanes = load_lanes(vs.NET, boxes)
    panels = {n: Panel(n, data[n][0], data[n][1], lanes[n], pw, ph) for n in TITLES}
    nf = max(len(p.recs) for p in panels.values())
    leg = legend_bar(pw * 3)
    frames = []
    names = list(TITLES)
    for i in range(nf):
        canvas = Image.new("RGB", (pw * 3, ph * 2 + leg.height), (255, 255, 255))
        for k, n in enumerate(names):
            canvas.paste(panels[n].frame(i), ((k % 3) * pw, (k // 3) * ph))
        canvas.paste(leg, (0, ph * 2))
        frames.append(canvas)
    # 모든 프레임이 같은 256색 팔레트를 쓰게 한다(색 일관성, 용량 감소). 주요 색은 견본으로 반드시 넣는다.
    picks = [frames[min(nf - 1, int(nf * q))] for q in (0.1, 0.3, 0.5, 0.7, 0.9)]
    sw = Image.new("RGB", (pw * 3, 40), (255, 255, 255))
    sd = ImageDraw.Draw(sw)
    keys = [BG, ROAD, BUSLANE, SIDEWALK, CROSS] + [c for c, _ in LEGEND] + [(35, 40, 52), (255, 190, 0), (40, 50, 70),
                                                                          (30, 30, 30), (255, 255, 255)]
    for k, c in enumerate(keys):
        sd.rectangle([k * 40, 0, k * 40 + 39, 39], fill=c)
    sample = Image.new("RGB", (pw * 3, sum(f.height for f in picks) + 40))
    y = 0
    for f in picks + [sw]:
        sample.paste(f, (0, y))
        y += f.height
    pal = sample.quantize(colors=256, method=Image.Quantize.FASTOCTREE, dither=Image.Dither.NONE)
    frames = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    frames[0].save(a.out, save_all=True, append_images=frames[1:], duration=a.ms, loop=0, disposal=1)
    print(f"저장: {a.out}  ({nf}프레임, {os.path.getsize(a.out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
