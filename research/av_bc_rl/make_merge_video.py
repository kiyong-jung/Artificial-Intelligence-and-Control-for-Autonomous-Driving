"""골목 출구(G24 → 와우산로 R112.112)에서 버스가 골목 차 ①은 먼저 보내고(양보) 바로 뒤 골목 차 ②는 기다리게 한
(비양보) 장면을 출구에 고정한 카메라로 그린 영상(MP4)을 만든다. 수집 기록(JSONL)만 쓰고 SUMO를 다시 돌리지 않는다.

장면은 메인 한 바퀴 영상과 같은 바퀴(rich2_s0_011 30번)다. research/av_bc_rl/alley_merge_events.py가 찾은 만남
499번 중 버스가 줄여서 먼저 보낸 유일한 만남과, 그 바로 뒤의 만남이다. 0.5초 기록 사이는 위치를 직선으로 이어
20fps 2배속으로 그린다. 오른쪽 표에는 버스 통과, 골목 차 대기, 최소 간격(두 차 앞범퍼 사이)을 기록에서 재어 적는다.

    python research/av_bc_rl/make_merge_video.py data/final_v2/rich2_s0_011.jsonl.gz \
        --out web_page/assets/videos/alley_merge.mp4
"""
import argparse
import gzip
import json
import math
import os

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw

from make_lap_video import ALLEY, BG, BUS, CAR, CROSS, EGO, LINE, PED, REPO, ROAD, SNAIL, TAXI, WALK, body, font, load_lanes
import sumolib

NODE_Y = -64.942            # 골목 출구 교차로(X_wausan11). 버스는 남쪽에서 올라와 이 선을 넘는다
STOP_LINE = (814.8, -61.8)  # 골목(G24) 끝
CENTER, RADIUS, MAP = (812.0, -78.0), 65.0, 880
EPISODE, STEPS = 30, (303, 345)
CARS = (("alley_G27_to_R115.2", "①", "A  버스 양보"), ("alley_G27_to_R127.1", "②", "B  버스 비양보"))
SUB, FPS = 5, 20            # 기록 한 줄(0.5초)을 나눌 장 수, fps → 2배속
INK, MUTED, PANEL = (236, 236, 240), (170, 170, 178), (24, 24, 28)


def read_steps(path):
    prefix = f'{{"episode":{EPISODE},'
    rows = []
    with gzip.open(path, "rt", encoding="utf-8") as f:
        for line in f:
            if not line.startswith(prefix):
                if rows:
                    break
                continue
            row = json.loads(line)
            if STEPS[0] <= row["t"] <= STEPS[1]:
                rows.append(row)
    return rows


def blend(a, b, w):
    turn = ((b["heading"] - a["heading"] + 180.0) % 360.0) - 180.0
    return {**a, "x": a["x"] + (b["x"] - a["x"]) * w, "y": a["y"] + (b["y"] - a["y"]) * w,
            "heading": a["heading"] + turn * w, "speed": a["speed"] + (b["speed"] - a["speed"]) * w}


def frames(rows):
    """한 줄(0.5초)을 SUB장으로 나눠 (줄 번호, 버스, 차들, 보행자)를 낸다."""
    for i, row in enumerate(rows):
        nxt = rows[min(i + 1, len(rows) - 1)]
        later = {v["id"]: v for v in nxt["state"]["vehicles"]}
        people = {p["id"]: p for p in nxt["state"]["pedestrians"]}
        for k in range(SUB if i + 1 < len(rows) else 1):
            w = k / SUB
            yield (i, blend(row["state"]["ego"], nxt["state"]["ego"], w),
                   [blend(v, later[v["id"]], w) if v["id"] in later else v for v in row["state"]["vehicles"]],
                   [blend(p, people[p["id"]], w) if p["id"] in people else p for p in row["state"]["pedestrians"]])


def waiting(v):
    return v["road_id"] == "G24" and v["speed"] < 0.3 and math.dist((v["x"], v["y"]), STOP_LINE) < 8


def measure(rows, car_id):
    """(버스 통과, 골목 차 대기, 최소 간격) 칸마다 (큰 값, 작은 설명)과, 결과를 보여 줄 줄 번호."""
    bus = [r["state"]["ego"] for r in rows]
    car = [next((v for v in r["state"]["vehicles"] if v["id"] == car_id), None) for r in rows]
    passed = next(i for i, e in enumerate(bus) if e["y"] > NODE_Y)
    alley = next(i for i, v in enumerate(car) if v and v["road_id"] == "G24")
    left = next(i for i, v in enumerate(car) if i > alley and v and v["road_id"] != "G24")
    wait = 0.5 * sum(1 for v in car if v and waiting(v))
    gap = min(math.dist((e["x"], e["y"]), (v["x"], v["y"])) for e, v in zip(bus, car) if v)
    kmh = [e["speed"] * 3.6 for e in bus]
    if left < passed:
        top = max(range(alley, passed + 1), key=kmh.__getitem__)
        crossing = (f"{kmh[top]:.0f} → {min(kmh[top:passed + 1]):.0f} km/h", "줄여서 골목 차를 먼저 보냄")
    else:
        crossing = (f"{kmh[passed]:.0f} km/h", "서지 않고 먼저 지나감")
    waited = (f"{wait:.1f}초", "멈춰 기다림") if wait else ("0초", "서지 않고 들어옴")
    return [crossing, waited, (f"{gap:.1f} m", "두 차 앞범퍼 사이")], max(left, passed)


def car_state(v, waited):
    if v is None:
        return "-"
    if v["road_id"] == "G24":
        return f"멈춰 기다림 {waited:.1f}초" if v["speed"] < 0.3 else "골목에서 나오는 중"
    if v["road_id"].startswith(":"):
        return "끼어드는 중"
    return "들어옴"


def background(lanes):
    scale = MAP / (2 * RADIUS)

    def px(p):
        return (MAP / 2 + (p[0] - CENTER[0]) * scale, MAP / 2 - (p[1] - CENTER[1]) * scale)
    image = Image.new("RGB", (MAP, MAP), BG)
    draw = ImageDraw.Draw(image)
    box = (CENTER[0] - RADIUS - 5, CENTER[1] - RADIUS - 5, CENTER[0] + RADIUS + 5, CENTER[1] + RADIUS + 5)
    for kind, polygon, shape, (x0, y0, x1, y1) in lanes:
        if x1 < box[0] or x0 > box[2] or y1 < box[1] or y0 > box[3]:
            continue
        draw.polygon([px(p) for p in polygon], fill={"road": ROAD, "walk": WALK, "cross": CROSS}[kind])
        if kind == "road":
            draw.line([px(p) for p in shape], fill=LINE, width=1)
    for (x, y), text in (((772.0, -44.0), "골목"), ((846.0, -128.0), "와우산로")):
        lx, ly = px((x, y))
        draw.rounded_rectangle((lx - 52, ly - 18, lx + 52, ly + 18), radius=8, fill=(232, 222, 250) if text == "골목" else (225, 225, 230))
        draw.text((lx, ly), text, font=font(20, True), fill=(40, 30, 60), anchor="mm")
    return image, px


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("jsonl", help="rich2_s0_011.jsonl.gz")
    parser.add_argument("--out", required=True)
    parser.add_argument("--net", default=os.path.join(REPO, "maps", "hapjeong", "source", "network.net.xml"))
    parser.add_argument("--hold", type=float, default=4.0, help="끝 장면을 더 보여 주는 시간(초)")
    args = parser.parse_args()
    base, px = background(load_lanes(sumolib.net.readNet(args.net, withInternal=True)))
    rows = read_steps(args.jsonl)
    results = [measure(rows, car_id) for car_id, _, _ in CARS]
    for (car_id, _, head), (cells, _) in zip(CARS, results):
        print(head, car_id, cells)
    shots = list(frames(rows))
    with imageio.get_writer(args.out, fps=FPS, codec="libx264", quality=8, macro_block_size=4) as writer:
        for n in range(len(shots) + int(FPS * args.hold)):
            i, ego, cars, peds = shots[min(n, len(shots) - 1)]
            image = base.copy()
            d = ImageDraw.Draw(image)
            marks = {car_id: mark for car_id, mark, _ in CARS}
            for v in cars:
                is_bus = "bus" in v["id"].lower()
                fill = BUS if is_bus else SNAIL if v["id"].startswith("snail_") else ALLEY if v["id"].startswith("alley_") \
                    else TAXI if v["id"].startswith("taxi_") else CAR
                focus = v["id"] in marks
                d.polygon([px(p) for p in body(v["x"], v["y"], v["heading"], 12.0 if is_bus else 4.8, 2.5 if is_bus else 1.8)],
                          fill=fill, outline=(255, 255, 255) if focus else None, width=3 if focus else 1)
                if focus:
                    qx, qy = px((v["x"], v["y"]))
                    d.ellipse((qx - 30, qy - 16, qx - 4, qy + 10), fill=(255, 255, 255))
                    d.text((qx - 17, qy - 3), marks[v["id"]], font=font(20, True), fill=(60, 30, 100), anchor="mm")
            for p in peds:
                qx, qy = px((p["x"], p["y"]))
                d.ellipse((qx - 4, qy - 4, qx + 4, qy + 4), fill=PED)
            d.polygon([px(p) for p in body(ego["x"], ego["y"], ego["heading"], 12.0, 2.5)], fill=EGO, outline=(0, 0, 0), width=2)
            d.rounded_rectangle((MAP - 112, MAP - 52, MAP - 16, MAP - 16), radius=8, fill=PANEL)
            d.text((MAP - 64, MAP - 34), "2배속", font=font(20, True), fill=EGO, anchor="mm")

            canvas = Image.new("RGB", (1600, 900), (18, 18, 22))
            canvas.paste(image, (10, 10))
            c = ImageDraw.Draw(canvas)
            x0, right = 930, 1576
            c.text((x0, 34), "골목 합류부 눈치싸움", font=font(38, True), fill=INK)
            c.text((x0, 92), "메인 영상과 같은 바퀴 · 골목 출구에 고정한 화면", font=font(20), fill=MUTED)
            c.rounded_rectangle((x0, 142, right, 330), radius=14, fill=PANEL)
            shown = ego["y"] < CENTER[1] + RADIUS
            c.text((x0 + 28, 182), "버스", font=font(24), fill=MUTED, anchor="lm")
            c.text((right - 28, 182), f"{ego['speed'] * 3.6:.0f} km/h" if shown else "지나감", font=font(34, True), fill=EGO, anchor="rm")
            for k, (car_id, mark, _) in enumerate(CARS):
                v = next((v for v in cars if v["id"] == car_id), None)
                waited = 0.5 * sum(1 for r in rows[:i + 1] for u in r["state"]["vehicles"] if u["id"] == car_id and waiting(u))
                c.text((x0 + 28, 240 + k * 52), f"골목 차 {mark}", font=font(24), fill=MUTED, anchor="lm")
                c.text((right - 28, 240 + k * 52), car_state(v, waited), font=font(26, True), fill=ALLEY, anchor="rm")
            label_w, col = 150, (right - x0 - 150) // 2
            top = 372
            for k, (_, mark, head) in enumerate(CARS):
                cx = x0 + label_w + k * col + col // 2
                c.text((cx, top), head, font=font(24, True), fill=EGO if k == 0 else (150, 190, 235), anchor="mm")
                c.text((cx, top + 34), f"골목 차 {mark}", font=font(18), fill=MUTED, anchor="mm")
            for r, label in enumerate(("버스 통과", "골목 차 대기", "최소 간격")):
                y = top + 70 + r * 140
                c.rounded_rectangle((x0, y, right, y + 124), radius=12, fill=PANEL)
                c.text((x0 + 24, y + 62), label, font=font(22), fill=MUTED, anchor="lm")
                for k, (cells, ready) in enumerate(results):
                    if i < ready:
                        continue
                    big, small = cells[r]
                    cx = x0 + label_w + k * col + col // 2
                    c.text((cx, y + 48), big, font=font(30, True), fill=INK, anchor="mm")
                    c.text((cx, y + 90), small, font=font(18), fill=MUTED, anchor="mm")
            writer.append_data(np.asarray(canvas))
    print(args.out, f"{len(shots) / FPS + args.hold:.1f}초")


if __name__ == "__main__":
    main()
