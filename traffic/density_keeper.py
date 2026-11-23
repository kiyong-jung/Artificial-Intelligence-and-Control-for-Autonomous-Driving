"""노선 구간 정의 도우미(측정·지도용): 시험용 버스 노선 위의 양화로·독막로·와우산로·잔다리로 도로 조각과 차로 비율.

구간별로 차를 넣고 빼던 `DensityKeeper`는 버스 주변 키퍼(`bus_window_keeper.BusWindowKeeper`)로 합쳐 없앴다. 이 모듈은 `scripts/measure_section_density.py`가
구간별 밀도를 재는 데만 쓴다.
"""
from dataclasses import dataclass

from .signal_areas import is_signalized

SECTION_ROADS = ("양화로", "독막로", "잔다리로", "와우산로")


@dataclass
class Section:
    name: str
    edge: str
    length: float
    signalized_end: bool
    lane_scale: float = 1.0      # 승용차가 쓸 수 있는 차로 수 / 같은 도로 이름 구간 중 가장 많은 수(버스 전용 차로가 있는 구간은 1보다 작다)


def sections_of(net, route_edges, min_len=60.0):
    out = []
    for e in route_edges:
        if not net.hasEdge(e):
            continue
        ed = net.getEdge(e)
        if ed.getName() in SECTION_ROADS and ed.getLength() >= min_len:
            out.append(Section(ed.getName(), e, ed.getLength(), is_signalized(ed.getToNode())))
    cars = {s.edge: sum(1 for ln in net.getEdge(s.edge).getLanes() if ln.allows("passenger")) for s in out}
    widest = {}
    for s in out:
        widest[s.name] = max(widest.get(s.name, 0), cars[s.edge])
    for s in out:
        s.lane_scale = cars[s.edge] / widest[s.name] if widest[s.name] else 1.0
    return out
