"""지름길마다 합정역 지도(scripts/render_map.py와 같은 그림)에서 그 길 둘레를 잘라 shortcut_<이름>.png로 쓴다.

    python3 docs/road_report/shortcut_crops.py
"""
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import render_map  # noqa: E402

# 이름 → 지도에서 함께 보일 도로 id. 합류점이 같이 보이도록 간선 쪽 도로를 조금 넣는다.
GROUPS = {
    "dongmak7": ["G4", "G16"],
    "jandari3": ["G17", "G18"],
    "eoulmadang": ["G31", "G19", "G20", "G21", "G8"],
    "jandari6": ["G22", "G25"],
    "jandari2": ["G23", "G25"],
    "wausan11": ["G24", "G26", "G27"],
    "dongmak6": ["G30", "G11", "G12"],
    "dongmak3": ["G3"],
}
MARGIN_M = 45

boxes = {}
original_savefig = plt.Figure.savefig


def savefig(fig, out, **kwargs):
    original_savefig(fig, out, **kwargs)
    ax = fig.axes[0]
    height = fig.get_size_inches()[1] * fig.dpi
    for name, ids in GROUPS.items():
        points = [p for lane_id, lane in net.lanes.items() if lane_id.rsplit("_", 1)[0] in ids for p in lane["points"]]
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        (left, bottom), (right, top) = ax.transData.transform([(min(xs) - MARGIN_M, min(ys) - MARGIN_M),
                                                               (max(xs) + MARGIN_M, max(ys) + MARGIN_M)])
        boxes[name] = (int(left), int(height - top), int(right), int(height - bottom))


original_net = render_map.Net


def capture(text):
    global net
    net = original_net(text)
    return net


render_map.Net = capture
plt.Figure.savefig = savefig
render_map.main()
image = Image.open(ROOT / "maps" / "hapjeong" / "references" / "hapjeong_map.png")
for name, box in boxes.items():
    image.crop(box).save(Path(__file__).parent / f"shortcut_{name}.png")
    print(name, box)
