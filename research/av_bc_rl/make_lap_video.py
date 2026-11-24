"""수집 기록(JSONL)의 한 에피소드를 위에서 본 영상(MP4)으로 만든다. SUMO를 다시 돌리지 않는다.

화면 가운데가 버스다(북쪽이 위). 오른쪽에 속도, 가감속 명령, 차로, 신호, 정류장, 사건을 적는다. 판단 한 번(0.5초)이
한 장이라 fps 15면 7.5배속이다. 차와 보행자는 기록에 남은 것(버스 주변)만 그린다.

    python research/av_bc_rl/make_lap_video.py data/research/eval300_dagger10/busy.jsonl.gz --episode 27 \
        --out reports/videos/worst_red_light.mp4 --title "혼잡 교통량, 적색 통과"
"""
import argparse
import gzip
import json
import math
import os
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(os.environ.get("SUMO_HOME", "/Users/ky/src/sumo-1.27.1-mesa"), "tools"))
import sumolib  # noqa: E402

FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
VIEW, PANEL = 720, 400   # 화면 한 변(px), 정보 칸 폭(px)
ROAD, WALK, CROSS, LINE, BG = (72, 76, 82), (176, 176, 170), (228, 228, 228), (120, 124, 130), (38, 52, 40)
EGO, CAR, BUS, PED, TRAIL = (255, 196, 0), (150, 190, 235), (120, 220, 160), (255, 120, 170), (255, 230, 120)
SIGNAL = {"r": (230, 70, 60), "R": (230, 70, 60), "u": (230, 70, 60), "y": (240, 200, 40), "Y": (240, 200, 40),
          "g": (60, 200, 90), "G": (60, 200, 90), "s": (60, 200, 90)}
TURN = {"left": "좌회전", "right": "우회전", "straight": "직진", None: "-"}
END = {"arrived": "완주", "collision": "차와 충돌", "pedestrian_collision": "보행자 충돌", "red_light": "적색 통과",
       "stuck": "멈춰 섬(200초 동안 5m 미만)", "time_limit": "시간 초과", "teleport": "순간이동", "missing": "사라짐"}


def font(size, bold=False):
    return ImageFont.truetype(FONT, size, index=6 if bold else 0)


def strip(shape, width):
    """중심선 좌표열을 폭이 있는 다각형으로."""
    left, right = [], []
    for i, (x, y) in enumerate(shape):
        ax, ay = shape[max(i - 1, 0)][:2]
        bx, by = shape[min(i + 1, len(shape) - 1)][:2]
        length = math.hypot(bx - ax, by - ay) or 1.0
        nx, ny = -(by - ay) / length * width / 2, (bx - ax) / length * width / 2
        left.append((x + nx, y + ny))
        right.append((x - nx, y - ny))
    return left + right[::-1]


def load_lanes(net_path):
    net = sumolib.net.readNet(net_path, withInternal=True)
    lanes = []
    for edge in net.getEdges(withInternal=True):
        function = edge.getFunction()
        for lane in edge.getLanes():
            walk = not lane.allows("passenger") and not lane.allows("bus")
            kind = "cross" if function == "crossing" else "walk" if walk or function == "walkingarea" else "road"
            shape = [point[:2] for point in lane.getShape()]
            if len(shape) < 2:
                continue
            xs, ys = [p[0] for p in shape], [p[1] for p in shape]
            lanes.append((kind, strip(shape, lane.getWidth()), shape, (min(xs), min(ys), max(xs), max(ys))))
    order = {"walk": 0, "road": 1, "cross": 2}
    return sorted(lanes, key=lambda lane: order[lane[0]])


def body(x, y, heading, length, width):
    """앞범퍼 가운데와 SUMO 방향(북쪽 0도, 시계 방향)으로 차체 네 모서리."""
    a = math.radians(90.0 - heading)
    fx, fy = math.cos(a), math.sin(a)
    lx, ly = -fy * width / 2, fx * width / 2
    return [(x + lx, y + ly), (x - lx, y - ly), (x - fx * length - lx, y - fy * length - ly),
            (x - fx * length + lx, y - fy * length + ly)]


def read_episode(path, episode):
    rows = []
    with gzip.open(path, "rt", encoding="utf-8") as f:
        for line in f:
            row = json.loads(line)
            if row["episode"] == episode:
                rows.append(row)
            elif rows:
                break
    if not rows:
        raise SystemExit(f"{path}에 에피소드 {episode}이 없다")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("jsonl")
    parser.add_argument("--episode", type=int, required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--title", default="")
    parser.add_argument("--net", default=os.path.join(REPO, "maps", "hapjeong", "source", "network.net.xml"))
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--last", type=float, default=None, help="마지막 이만큼(초)만 찍는다. 실패 장면을 느리게 볼 때")
    parser.add_argument("--hold", type=float, default=2.0, help="마지막 장면을 더 보여 주는 시간(초)")
    parser.add_argument("--radius", type=float, default=50.0, help="버스에서 화면 끝까지 거리(m)")
    args = parser.parse_args()
    rows = read_episode(args.jsonl, args.episode)
    half, scale = args.radius, VIEW / (2 * args.radius)
    lanes = load_lanes(args.net)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    trail, changes, banner, banner_left, center = [], 0, "", 0, None
    start = rows[0]["sim_time"]
    first = 0 if args.last is None else max(0, len(rows) - int(args.last / 0.5))
    with imageio.get_writer(args.out, fps=args.fps, codec="libx264", quality=7, macro_block_size=8) as writer:
        for index, row in enumerate(rows):
            state, event, action = row["state"], row["event"], row["action"]
            ego = state["ego"]
            if "x" in ego:
                center = (ego["x"], ego["y"])
                trail.append(center)
            cx, cy = center

            def px(point):
                return (VIEW / 2 + (point[0] - cx) * scale, VIEW / 2 - (point[1] - cy) * scale)
            image = Image.new("RGB", (VIEW + PANEL, VIEW), BG)
            draw = ImageDraw.Draw(image)
            box = (cx - half - 5, cy - half - 5, cx + half + 5, cy + half + 5)
            for kind, polygon, shape, (x0, y0, x1, y1) in lanes:
                if x1 < box[0] or x0 > box[2] or y1 < box[1] or y0 > box[3]:
                    continue
                draw.polygon([px(p) for p in polygon], fill={"road": ROAD, "walk": WALK, "cross": CROSS}[kind])
                if kind == "road":
                    draw.line([px(p) for p in shape], fill=LINE, width=1)
            if len(trail) > 1:
                draw.line([px(p) for p in trail[-120:]], fill=TRAIL, width=2)
            for vehicle in state.get("vehicles", ()):
                is_bus = "bus" in vehicle["id"].lower()
                draw.polygon([px(p) for p in body(vehicle["x"], vehicle["y"], vehicle["heading"], 12.0 if is_bus else 4.8,
                                                   2.5 if is_bus else 1.8)], fill=BUS if is_bus else CAR)
            for person in state.get("pedestrians", ()):
                x, y = px((person["x"], person["y"]))
                draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=PED)
            if "x" in ego:
                draw.polygon([px(p) for p in body(ego["x"], ego["y"], ego["heading"], 12.0, 2.5)], fill=EGO, outline=(0, 0, 0))
            if action.get("lane_change_actual"):
                changes += 1
                banner, banner_left = ("왼쪽으로 차로 변경" if action["lane_change_actual"] > 0 else "오른쪽으로 차로 변경"), 6
            if event.get("service_event"):
                banner, banner_left = ("정류장 정차 완료" if event["service_event"] == "served" else "정류장을 놓침"), 10
            if event.get("red_light_violation"):
                banner, banner_left = "적색 신호를 넘음", 40
            if event.get("collided"):
                banner, banner_left = "충돌", 40
            if event.get("termination_reason"):
                banner, banner_left = "끝: " + END.get(event["termination_reason"], event["termination_reason"]), 40

            draw.rectangle((VIEW, 0, VIEW + PANEL, VIEW), fill=(24, 24, 28))
            x, y = VIEW + 24, 20
            draw.text((x, y), args.title, font=font(24, True), fill="white")
            nav, light = state.get("navigation") or {}, state.get("traffic_light") or {}
            lines = [("시간", f"{row['sim_time'] - start:5.0f} 초"),
                     ("속도", f"{ego.get('speed', 0) * 3.6:5.1f} km/h"),
                     ("가감속 명령", f"{action['accel_raw']:+.2f}"),
                     ("차로(1=가로변)", str(ego.get("lane_index", "-"))),
                     ("차로 변경 누적", f"{changes} 번"),
                     ("다음 회전", f"{TURN.get(nav.get('next_turn'), nav.get('next_turn'))} {nav.get('distance_to_turn') or 0:.0f} m"),
                     ("다음 정류장", f"{nav.get('distance_to_stop'):.0f} m" if nav.get("distance_to_stop") is not None else "-"),
                     ("정차한 정류장", f"{event.get('stops_served', 0)} / 4"),
                     ("놓친 정류장", str(event.get("stops_missed", 0)))]
            y = 70
            for label, value in lines:
                draw.text((x, y), label, font=font(19), fill=(170, 170, 176))
                draw.text((VIEW + PANEL - 24, y), value, font=font(21, True), fill="white", anchor="ra")
                y += 40
            draw.text((x, y + 6), "다음 신호", font=font(19), fill=(170, 170, 176))
            if light.get("state"):
                color = SIGNAL.get(light["state"], (140, 140, 140))
                draw.ellipse((VIEW + PANEL - 160, y + 2, VIEW + PANEL - 130, y + 32), fill=color)
                draw.text((VIEW + PANEL - 24, y + 6), f"{light.get('distance_to_stop_line') or 0:.0f} m", font=font(21, True), fill="white", anchor="ra")
            else:
                draw.text((VIEW + PANEL - 24, y + 6), "-", font=font(21, True), fill="white", anchor="ra")
            y += 56
            bar = VIEW + PANEL - 48 - x   # 가감속 막대: 가운데가 0
            draw.rectangle((x, y, x + bar, y + 16), outline=(90, 90, 96))
            mid = x + bar / 2
            value = max(-1.0, min(1.0, action["accel_raw"]))
            draw.rectangle((min(mid, mid + value * bar / 2), y, max(mid, mid + value * bar / 2), y + 16),
                           fill=(90, 200, 120) if value >= 0 else (230, 110, 90))
            draw.text((x, y + 22), "줄임", font=font(15), fill=(150, 150, 156))
            draw.text((x + bar, y + 22), "밟음", font=font(15), fill=(150, 150, 156), anchor="ra")
            if banner_left > 0:
                draw.rectangle((x - 8, VIEW - 120, VIEW + PANEL - 16, VIEW - 64), fill=(250, 250, 250))
                draw.text((x + 4, VIEW - 92), banner, font=font(24, True), fill=(20, 20, 20), anchor="lm")
                banner_left -= 1
            draw.text((x, VIEW - 40), f"반경 {half:.0f}m  노랑=우리 버스  파랑=차  초록=다른 버스  분홍=보행자", font=font(14), fill=(150, 150, 156))
            if index < first:   # 찍지 않는 앞부분도 궤적과 누적 수는 센다
                continue
            writer.append_data(np.asarray(image))
            if index == len(rows) - 1:   # 마지막 장면을 더 보여 준다
                for _ in range(int(args.fps * args.hold)):
                    writer.append_data(np.asarray(image))
    shown = len(rows) - first
    print(args.out, f"{shown}장, {shown / args.fps + args.hold:.0f}초, 차로 변경 {changes}번")


if __name__ == "__main__":
    main()
