"""DensityKeeper: 구간(도로 이름)별 목표 대수를 코드가 직접 유지한다. 시험용 버스 반경 + 여유 안에서는 투입·제거하지 않는다."""
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.density_keeper import DEFAULT_DENSITY_PER_100M, SECTION_ROADS, DensityKeeper, sections_of  # noqa: E402
from traffic.presence import PRESENCE_RADIUS_M, SPAWN_MARGIN_M  # noqa: E402
from sumolib.geomhelper import positionAtShapeOffset  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")
MIN_D = PRESENCE_RADIUS_M + SPAWN_MARGIN_M
_c = {}


def _net():
    if "net" not in _c:
        _c["net"] = read_net(os.path.join(ROOT, "source", "network.net.xml"))
    return _c["net"]


def _map():
    if "m" not in _c:
        _c["m"] = load_map("hapjeong")
    return _c["m"]


def _exits():
    return sorted({f["to"] for f in _map()["flows"]})


class _Edge:
    def __init__(self, tc):
        self.tc = tc

    def getLastStepVehicleNumber(self, e):
        return len(self.tc.on_edge.get(e, []))

    def getLastStepHaltingNumber(self, e):
        return self.tc.halting.get(e, 0)

    def getLastStepVehicleIDs(self, e):
        return tuple(self.tc.on_edge.get(e, []))


class _Veh:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return (["bus0"] if self.tc.bus_xy is not None else []) + list(self.tc.xy)

    def getPosition(self, v):
        return self.tc.bus_xy if v == "bus0" else self.tc.xy[v]

    def getSpeed(self, v):
        return self.tc.speed.get(v, 10.0)

    def add(self, vid, rid, **kw):
        self.tc.added.append((vid, rid, kw))

    def remove(self, vid, *a):
        self.tc.removed.append(vid)


class _Route:
    def __init__(self, tc):
        self.tc = tc

    def add(self, rid, edges):
        self.tc.routes[rid] = list(edges)


class _Tc:
    def __init__(self, bus_xy=None):
        self.bus_xy = bus_xy
        self.on_edge, self.halting, self.xy, self.speed = {}, {}, {}, {}
        self.added, self.removed, self.routes = [], [], {}
        self.edge, self.vehicle, self.route = _Edge(self), _Veh(self), _Route(self)


def _keeper(tc, density=None, **kw):
    net = _net()
    return DensityKeeper(tc, net, "bus0", _map()["routes"]["bus_loop"], density or DEFAULT_DENSITY_PER_100M,
                         rng=random.Random(1), **kw)


def _run(k, seconds):
    for i in range(int(seconds * 10)):
        k.update(i * 0.1)


def _xy(edge, pos):
    return positionAtShapeOffset(_net().getEdge(edge).getShape(), pos)


def test_sections_cover_the_four_named_roads():
    secs = sections_of(_net(), _map()["routes"]["bus_loop"])
    assert {s.name for s in secs} == set(SECTION_ROADS)
    assert all(s.length >= 60.0 for s in secs)


def test_target_rounds_with_minimum_one():
    k = _keeper(_Tc(), density={n: 0.01 for n in SECTION_ROADS})
    assert all(k.target(s) == 1 for s in k.sections)


def test_fills_shortfall_only_where_short():
    tc = _Tc()
    k = _keeper(tc)
    for s in k.sections:
        tc.on_edge[s.edge] = [f"v{i}" for i in range(k.target(s))]
    short = next(s for s in k.sections if s.length >= 90)
    tc.on_edge[short.edge] = []
    _run(k, 3)
    edges = {tc.routes[rid][0] for _, rid, _ in tc.added}
    assert edges == {short.edge}


def test_never_inserts_within_bus_radius_plus_margin():
    secs = sections_of(_net(), _map()["routes"]["bus_loop"])
    tc = _Tc(bus_xy=_xy(secs[0].edge, secs[0].length / 2))
    k = _keeper(tc)
    _run(k, 600)
    assert len(tc.added) > 20 and k.summary()["skipped_near_bus"] > 0
    bx, by = tc.bus_xy
    for _, rid, kw in tc.added:
        x, y = _xy(tc.routes[rid][0], float(kw["departPos"]))
        assert math.hypot(x - bx, y - by) >= MIN_D - 1e-6
        assert kw["typeID"] == "carDist"


def test_fills_when_bus_absent():
    tc = _Tc(bus_xy=None)
    k = _keeper(tc)
    _run(k, 5)
    assert len(tc.added) >= 1


def test_no_insert_near_junction_end_of_any_section():
    tc = _Tc()
    k = _keeper(tc)
    _run(k, 600)
    by_edge = {s.edge: s for s in k.sections}
    seen = 0
    for _, rid, kw in tc.added:
        s = by_edge[tc.routes[rid][0]]
        seen += 1
        assert float(kw["departPos"]) <= s.length - 60.0 + 0.1
    assert seen > 0


def test_no_insert_when_queue_is_long():
    tc = _Tc()
    k = _keeper(tc)
    for s in k.sections:
        tc.halting[s.edge] = 3
    _run(k, 20)
    assert tc.added == [] and k.summary()["skipped_queue"] > 0


def test_removes_surplus_far_from_bus_only():
    secs = sections_of(_net(), _map()["routes"]["bus_loop"])
    s = max(secs, key=lambda x: x.length)
    bus = _xy(s.edge, s.length / 2)
    tc = _Tc(bus_xy=bus)
    k = _keeper(tc, protected=lambda vid: vid == "guarded", surplus_ratio=1.5)
    n = math.ceil(k.target(s) * 1.5) + 8
    ids = [f"c{i}" for i in range(n)] + ["guarded"]
    tc.on_edge[s.edge] = ids
    for i, vid in enumerate(ids):
        tc.xy[vid] = _xy(s.edge, min(s.length - 1.0, 1.0 + i * s.length / (n + 1)))
    _run(k, 30)
    assert tc.removed and "guarded" not in tc.removed
    for vid in tc.removed:
        assert math.hypot(tc.xy[vid][0] - bus[0], tc.xy[vid][1] - bus[1]) >= MIN_D - 1e-6


def test_insert_budget_limits_flow_to_max_per_h():
    tc = _Tc()
    k = _keeper(tc, max_per_h=360.0, burst=5.0)
    _run(k, 1000)                                           # 모든 구간이 계속 비어 있다고 본다
    assert len(tc.added) <= 360.0 / 3600.0 * 1000 + 5 + 1
    assert k.summary()["skipped_budget"] > 0


def test_inserted_cars_follow_the_bus_route_about_ahead_m_and_stop():
    tc = _Tc()
    k = _keeper(tc, ahead_m=300.0)
    _run(k, 300)
    loop = _map()["routes"]["bus_loop"]
    assert tc.routes
    for rid, edges in tc.routes.items():
        i = loop.index(edges[0])
        assert edges == loop[i:i + len(edges)]               # 노선을 그대로 따라간다
        assert sum(_net().getEdge(e).getLength() for e in edges[:-1]) < 300.0   # 마지막 도로 전에 300 m를 넘지 않는다


def test_default_density_is_highest_on_the_wide_road_and_lower_on_the_two_lane_roads():
    d = DEFAULT_DENSITY_PER_100M
    assert d["양화로"] == max(d.values())
    assert d["와우산로"] == 0.0 and d["잔다리로"] == 0.0 and 0 < d["독막로"] < d["양화로"]


def test_budget_goes_first_to_the_section_missing_the_most_cars():
    # 예산이 한 대뿐이면 모든 구간이 비어 있을 때 대수가 가장 많이 모자란(넓고 긴) 구간이 먼저 받는다
    tc = _Tc()
    k = _keeper(tc, max_per_h=0.0, burst=1.0)
    _run(k, 3)
    assert len(tc.added) == 1
    first = tc.routes[tc.added[0][1]][0]
    worst = max(k.sections, key=lambda s: k.target(s) if s.length - 60.0 > 10.0 else 0)
    assert k.target(next(s for s in k.sections if s.edge == first)) == k.target(worst)


def test_bus_lane_section_gets_a_lower_target_by_its_passenger_lane_share():
    # 양화로 R54a는 4차로 중 4번이 버스 전용이라 승용차는 3차로만 쓴다(같은 양화로 R54는 5차로). 목표를 그 비율로 줄인다
    secs = {s.edge: s for s in sections_of(_net(), _map()["routes"]["bus_loop"])}
    assert secs["R54a"].lane_scale == pytest.approx(3 / 5) and secs["R54"].lane_scale == 1.0
    assert all(s.lane_scale == 1.0 for s in secs.values() if s.name != "양화로")
    k = _keeper(_Tc())
    d = DEFAULT_DENSITY_PER_100M["양화로"]
    assert k.target(secs["R54a"]) == max(1, round(d * secs["R54a"].length / 100.0 * 0.6))
    assert k.target(secs["R54a"]) < round(d * secs["R54a"].length / 100.0)


def test_road_with_zero_density_gets_no_target_and_no_insertion():
    # 잔다리로(홍대입구역~서교동사거리)와 와우산로는 밀도 0: 목표도 투입도 없어 예산을 쓰지 않는다
    tc = _Tc()
    k = _keeper(tc)
    assert {s.name for s in k.sections} == {"양화로", "독막로"}
    _run(k, 600)
    assert tc.added
    for _, rid, _ in tc.added:
        assert _net().getEdge(tc.routes[rid][0]).getName() != "잔다리로"
    assert "잔다리로" not in k.summary()["spawned_by_road"] and sum(k.summary()["spawned_by_road"].values()) == len(tc.added)


def test_short_section_without_an_insertion_spot_is_filled_from_an_upstream_road():
    # 독막로 R71.135(64 m)는 끝 60 m 금지 때문에 자기 안에 넣을 자리가 없다. 상류 R71(135 m, 150 m 안)에서 넣어 달려오게 한다
    tc = _Tc()
    k = _keeper(tc)
    short = next(s for s in k.sections if s.edge == "R71.135")
    assert k._anchors["R71.135"] and all(e != "R71.135" for e, _ in k._anchors["R71.135"])
    for s in k.sections:
        tc.on_edge[s.edge] = [f"v{i}" for i in range(k.target(s))]
    tc.on_edge[short.edge] = []
    _run(k, 5)
    assert tc.added
    for _, rid, kw in tc.added:
        edges = tc.routes[rid]
        assert edges[0] == "R71" and "R71.135" in edges                  # 상류에서 넣고 노선을 따라 그 구간으로 들어온다
        assert float(kw["departPos"]) <= _net().getEdge("R71").getLength() - 60.0 + 0.1
    assert k.summary()["spawned_by_road"] == {"독막로": len(tc.added)}


def test_anchors_stay_on_roads_with_density_and_within_upstream_m():
    k0 = _keeper(_Tc(), upstream_m=0.0)
    assert all(len(v) <= 1 for v in k0._anchors.values())                # 거슬러 가지 않으면 구간 자신뿐
    k = _keeper(_Tc())
    for s in k.sections:
        for e, _ in k._anchors[s.edge]:
            assert _net().getEdge(e).getName() in ("양화로", "독막로")


def test_keeper_cars_leave_through_another_road_at_j82_instead_of_entering_wausan_road():
    # 독막로 R111에서 넣은 차의 노선 앞쪽은 R111b → J82 → 와우산로 R112다. 와우산로로 들어가지 않고 J82에서 다른 도로로 빠져나간다
    from traffic.flow_balance import edges_named
    avoid = frozenset(edges_named(_net(), ("와우산로", "잔다리로")))
    k = _keeper(_Tc(), avoid=avoid, ahead_m=300.0)
    route = k._local_route("R111")
    assert route[0] == "R111" and "R111b" in route and not any(e in avoid for e in route)
    assert route[-1] != "R111b" and route.index("R111b") + 1 < len(route)                 # J82 뒤로 다른 도로가 이어진다
    from traffic.presence import ends_at_boundary
    assert ends_at_boundary(_net(), route[-1])
    plain = _keeper(_Tc(), ahead_m=300.0)._local_route("R111")                              # 피할 도로가 없으면 노선을 그대로 따라간다
    assert "R112" in plain


def test_keeper_never_anchors_on_avoid_roads():
    from traffic.flow_balance import edges_named
    avoid = frozenset(edges_named(_net(), ("와우산로", "잔다리로")))
    k = _keeper(_Tc(), density={**DEFAULT_DENSITY_PER_100M, "와우산로": 1.0, "잔다리로": 1.0}, avoid=avoid)
    assert all(e not in avoid for v in k._anchors.values() for e, _ in v)
