"""버스 경로 bus_loop를 위에서 본 그림으로 그린다(Pillow만 쓴다). 왼쪽은 경로 전체, 오른쪽은 합정역 사거리(J1)에서
R68로 좌회전해 순환이 닫히는 곳의 확대다.

    python scripts/render_bus_route.py [출력 PNG]

기본 출력은 docs/img/bus_loop_route.png다.
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

BASE = Path(__file__).resolve().parents[1]
FOLDER = BASE / "maps" / "hapjeong"
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
PAPER, ROAD, WALK, CROSS, INK, MUTED = "#f6f5f1", "#c9c6bd", "#e4e1d8", "#ffffff", "#222222", "#6b6b6b"
ROUTE, NEW, STOP, START = "#1565c0", "#e53935", "#f2a900", "#2e9d48"
NEW_EDGES = {"R68"}


def _pts(text: str) -> list[tuple[float, float]]:
    return [tuple(map(float, p.split(",")[:2])) for p in text.split()]


def load(net_path: Path):
    lanes, walks, crossings = [], [], []
    shapes = {}
    root = ET.parse(net_path).getroot()
    for edge in root.iter("edge"):
        fn = edge.get("function")
        for lane in edge.iter("lane"):
            shape = _pts(lane.get("shape"))
            width = float(lane.get("width", 3.2))
            shapes[lane.get("id")] = shape
            if fn == "crossing":
                crossings.append((shape, width))
            elif fn == "walkingarea":
                continue
            elif "pedestrian" in (lane.get("allow") or "").split():
                walks.append((shape, width))
            else:
                lanes.append((shape, width))
    links = {}
    for c in root.iter("connection"):
        links.setdefault((c.get("from"), c.get("to")), []).append((int(c.get("fromLane")), c.get("via")))
    return lanes, walks, crossings, shapes, links


class View:
    def __init__(self, box, size, margin=40):
        (x0, y0, x1, y1), (w, h) = box, size
        self.scale = min((w - 2 * margin) / (x1 - x0), (h - 2 * margin) / (y1 - y0))
        self.ox = margin + ((w - 2 * margin) - self.scale * (x1 - x0)) / 2 - self.scale * x0
        self.oy = h - margin - ((h - 2 * margin) - self.scale * (y1 - y0)) / 2 + self.scale * y0

    def __call__(self, p):
        return self.ox + self.scale * p[0], self.oy - self.scale * p[1]


def draw_net(d: ImageDraw.ImageDraw, view: View, lanes, walks, crossings):
    for shape, width in walks:
        d.line([view(p) for p in shape], fill=WALK, width=max(1, round(width * view.scale)))
    for shape, width in lanes:
        d.line([view(p) for p in shape], fill=ROAD, width=max(1, round(width * view.scale)))
    for shape, width in crossings:
        d.line([view(p) for p in shape], fill=CROSS, width=max(1, round(width * view.scale)))


def arrow(d, view, a, b, color, size):
    (ax, ay), (bx, by) = view(a), view(b)
    ang = math.atan2(by - ay, bx - ax)
    tip = (bx, by)
    left = (bx - size * math.cos(ang - 0.45), by - size * math.sin(ang - 0.45))
    right = (bx - size * math.cos(ang + 0.45), by - size * math.sin(ang + 0.45))
    d.polygon([tip, left, right], fill=color)


def route_lanes(shapes, links, route):
    """경로를 따라 (도로, 모양) 목록. 도로마다 다음 도로로 가는 연결 중 가장 오른쪽 차로를 쓰고, 그 연결의 교차로 안
    차로를 사이에 넣는다(J1에서는 좌회전 3번 차로)."""
    out = []
    for i, edge in enumerate(route):
        options = sorted(links.get((edge, route[i + 1]), [])) if i + 1 < len(route) else []
        lane = options[0][0] if options else 1
        out.append((edge, shapes[f"{edge}_{lane}"]))
        via = options[0][1] if options else None
        while via:
            out.append((route[i + 1] if i + 2 == len(route) else edge, shapes[via]))
            nxt = [v for (f, _), vs in links.items() if f == via.rsplit("_", 1)[0] for _, v in vs]
            via = next((v for v in nxt if v), None)
    return out


def render(out: Path) -> None:
    definition = json.loads((FOLDER / "map.json").read_text(encoding="utf-8"))
    route = definition["routes"][definition["ego_route"]]
    lanes, walks, crossings, shapes, links = load(FOLDER / definition["network_source"]["path"])
    path = route_lanes(shapes, links, route)
    xs = [p[0] for _, s in path for p in s]
    ys = [p[1] for _, s in path for p in s]
    pad = 60
    full = (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)
    near = shapes["R42_3"] + shapes["R68_1"] + shapes["R69_2"][:1]
    cx = (min(p[0] for p in near) + max(p[0] for p in near)) / 2
    cy = (min(p[1] for p in near) + max(p[1] for p in near)) / 2
    half = max(max(p[0] for p in near) - min(p[0] for p in near), max(p[1] for p in near) - min(p[1] for p in near)) / 2 + 20
    zoom = (cx - half, cy - half, cx + half, cy + half)
    W, H = 2200, 1150
    img = Image.new("RGB", (W, H), PAPER)
    d = ImageDraw.Draw(img)
    title = ImageFont.truetype(FONT, 30)
    body = ImageFont.truetype(FONT, 20)
    small = ImageFont.truetype(FONT, 17)
    panels = (((0, 70), (1300, 1080), full, "경로 전체"), ((1320, 70), (860, 860), zoom, "합정역 사거리(J1) 확대"))
    for (px, py), size, box, label in panels:
        panel = Image.new("RGB", size, PAPER)
        pd = ImageDraw.Draw(panel)
        view = View(box, size)
        draw_net(pd, view, lanes, walks, crossings)
        width = max(4, round(1.4 * view.scale))
        for edge, shape in path:
            pd.line([view(p) for p in shape], fill=NEW if edge in NEW_EDGES else ROUTE, width=width, joint="curve")
        for edge, shape in path[::3] + [path[-1]]:
            k = len(shape) // 2
            a, b = (shape[k - 1], shape[k]) if len(shape) > 2 else (shape[0], shape[-1])
            arrow(pd, view, a, b, NEW if edge in NEW_EDGES else ROUTE, max(10, width * 3))
        for stop in definition["bus_stops"]:
            shape = shapes[stop["lane"]]
            length = sum(math.dist(a, b) for a, b in zip(shape, shape[1:]))
            for at in (float(stop["startPos"]), float(stop["endPos"])):
                walked, p = 0.0, shape[0]
                for a, b in zip(shape, shape[1:]):
                    seg = math.dist(a, b)
                    if walked + seg >= min(at, length):
                        t = (min(at, length) - walked) / seg
                        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                        break
                    walked += seg
                x, y = view(p)
                r = max(5, width)
                pd.ellipse([x - r, y - r, x + r, y + r], fill=STOP, outline=INK)
            x, y = view(shape[0])
            if box is full:
                pd.text((x + 10, y - 26), stop["name"], font=small, fill=INK)
        sx, sy = view(path[0][1][0])
        pd.ellipse([sx - 10, sy - 10, sx + 10, sy + 10], fill=START, outline=INK)
        if box is zoom:
            pd.text((sx - 40, sy + 14), "출발 R69", font=small, fill=INK)
        ex, ey = view(path[-1][1][-1])
        pd.rectangle([ex - 9, ey - 9, ex + 9, ey + 9], fill=NEW, outline=INK)
        if box is zoom:
            pd.text((ex - 60, ey + 14), "끝(J4 앞)", font=small, fill=INK)
        if box is zoom:
            for lane, name in (("R42_4", "R42 양화로(좌회전 3·4번 차로)"), ("R68_2", "R68 독막로")):
                shape = shapes[lane]
                x, y = view(shape[len(shape) // 2])
                pd.text((x + 16, y - 30), name, font=small, fill=INK)
        img.paste(panel, (px, py))
        d.text((px + 20, py + 10), label, font=body, fill=INK)
        if box is zoom:
            x0, y0 = View(full, panels[0][1])((zoom[0], zoom[3]))
            x1, y1 = View(full, panels[0][1])((zoom[2], zoom[1]))
            d.rectangle([x0, y0 + 70, x1, y1 + 70], outline=NEW, width=3)
    d.text((20, 18), f"합정역 지도 {definition['version']} 버스 경로 bus_loop "
           f"({definition['route_design'][definition['ego_route']]['length_m']:.0f} m)", font=title, fill=INK)
    legend = [(ROUTE, "기존 경로"), (NEW, "3.19.0에 붙인 J1 좌회전 → R68"), (STOP, "정류장 시작·끝"), (START, "출발")]
    x = 1340
    for color, text in legend:
        d.rectangle([x, 960, x + 26, 978], fill=color, outline=INK)
        d.text((x + 34, 956), text, font=small, fill=INK)
        x += 34 + d.textlength(text, font=small) + 30
    d.text((1340, 1000), "회색: 차로, 연한 회색: 인도, 흰색: 횡단보도", font=small, fill=MUTED)
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    print(out)


if __name__ == "__main__":
    render(Path(sys.argv[1]) if len(sys.argv) > 1 else BASE / "docs" / "img" / "bus_loop_route.png")
