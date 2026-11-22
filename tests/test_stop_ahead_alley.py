"""S4: 골목에서 나온 차가 정류장 근처(정류장 10~25 m 앞 또는 끝 5~20 m 뒤)에 서서 시험용 버스를 막는다."""
import json
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import close_loop, find_merge_alleys, read_net  # noqa: E402
from traffic.presence import ends_at_boundary  # noqa: E402
from traffic.signal_areas import forbidden_intervals, unsignalized_alleys  # noqa: E402
from traffic.stop_ahead import StopAheadManager, s4_candidates, stop_position  # noqa: E402
from traffic.test_bus import lap_route  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")
_cache = {}
S2, S4, S5, S6 = "osm_stop_357920024", "osm_stop_5930628870", "osm_stop_3446267493", "osm_stop_3424857165"


def _net():
    if "net" not in _cache:
        _cache["net"] = read_net(os.path.join(ROOT, "source", "network.net.xml"))
    return _cache["net"]


def _map():
    if "map" not in _cache:
        with open(os.path.join(ROOT, "map.json"), encoding="utf-8") as f:
            _cache["map"] = json.load(f)
    return _cache["map"]


def _route():
    return lap_route(_net(), _map()["routes"]["bus_loop"])


def _stops():
    return [{"id": s["id"], "edge": s["lane"].rsplit("_", 1)[0], "lane": int(s["lane"].rsplit("_", 1)[1]),
             "startPos": s["startPos"], "endPos": s["endPos"]} for s in _map()["bus_stops"]]


def _alleys():
    return unsignalized_alleys(find_merge_alleys(_net(), close_loop(_net(), _map()["routes"]["bus_loop"])))


def _dist(route, a, b):
    """노선 위 (도로, 위치) a에서 b까지 앞으로 가는 거리."""
    i, j = route.index(a[0]), route.index(b[0])
    if i == j:
        return b[1] - a[1]
    d = _net().getEdge(route[i]).getLength() - a[1]
    for e in route[i + 1:j]:
        d += _net().getEdge(e).getLength()
    return d + b[1]


def test_s4_candidates_pair_stops_with_unsignalized_upstream_alleys():
    c = s4_candidates(_net(), _route(), _stops(), _alleys())
    stops = {s for s, _ in c}
    assert {S2, S4, S5} <= stops                       # s2 <- G3, s4 <- G24, s5 <- 잔다리로 골목
    assert ("osm_stop_357920065" not in stops) and ("osm_stop_357916587" not in stops)   # s1, s3에는 서지 않는다
    assert all(a.junction not in {"J74", "J48", "cluster_J52_J56_J58"} for _, a in c)
    assert all(a.junction != "X_dongmak6" for s, a in c if s == S2)        # 정류장 뒤에서 합류하는 골목은 아니다


def test_stop_position_keeps_distance_from_stop_and_signal():
    route, bad = _route(), forbidden_intervals(_net(), _route())
    stops = {s["id"]: s for s in _stops()}
    for sid, alley_into in ((S4, "R112.112"), (S5, "-R96.119")):
        s = stops[sid]
        for seed in range(40):
            p = stop_position(_net(), route, s, random.Random(seed), not_before=alley_into)
            assert p is not None
            edge, pos = p
            assert not any(lo <= pos <= hi for lo, hi in bad.get(edge, []))
            assert route.index(edge) >= route.index(alley_into)
            before = _dist(route, (edge, pos), (s["edge"], s["startPos"]))
            after = _dist(route, (s["edge"], s["endPos"]), (edge, pos))
            assert 10.0 - 1e-6 <= before <= 25.0 + 1e-6 or 5.0 - 1e-6 <= after <= 20.0 + 1e-6


class _V:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return ["bus0"] + [v for v in self.tc.cars if v in self.tc.visible]

    def getPosition(self, v):
        return (1e5, 1e5) if v == "bus0" else (0.0, 0.0)

    def getSpeed(self, v):
        return 8.0

    def getRoadID(self, v):
        return self.tc.bus_road

    def getRoute(self, v):
        return _route()

    def getRouteIndex(self, v):
        return _route().index(self.tc.bus_road)

    def getLanePosition(self, v):
        return self.tc.car_pos.get(v, self.tc.bus_pos)

    def add(self, vid, rid, typeID="carDist", **kw):
        self.tc.added.append({"id": vid, "route": self.tc.routes[rid], "type": typeID, **kw})
        self.tc.cars.append(vid)

    def setStop(self, vid, edge, pos=1.0, laneIndex=0, duration=-1, **kw):
        self.tc.stops_set.append((vid, edge, pos, laneIndex, duration))

    def setParameter(self, *a):
        pass

    def isStopped(self, v):
        return False

    def getStopState(self, v):
        return 0

    def getStops(self, v, n=1):
        return []


class _R:
    def __init__(self, tc):
        self.tc = tc

    def add(self, rid, edges):
        self.tc.routes[rid] = list(edges)


class _L:
    def __init__(self, tc=None):
        self.tc = tc

    def getLength(self, lane):
        return _net().getEdge(lane.rsplit("_", 1)[0]).getLength()

    def getLastStepVehicleIDs(self, lane):
        return tuple(self.tc.lane_cars.get(lane, ()))


class _TC:
    def __init__(self, bus_road, bus_pos):
        self.bus_road, self.bus_pos = bus_road, bus_pos
        self.cars, self.visible, self.added, self.routes, self.stops_set = [], set(), [], {}, []
        self.lane_cars, self.car_pos = {}, {}              # 차로 id -> 차 id 목록 / 차 id -> 도로 위 위치
        self.vehicle, self.route, self.lane = _V(self), _R(self), _L(self)


def _mgr(tc, seed=0, lap=lambda: 1, **kw):
    return StopAheadManager(tc, "bus0", _net(), _route(), _stops(), _alleys(), lap_fn=lap, rng=random.Random(seed), **kw)


def test_one_or_two_blocks_planned_per_lap_at_candidate_stops():
    cand = {s for s, _ in s4_candidates(_net(), _route(), _stops(), _alleys())}
    counts, picked = set(), set()
    for seed in range(60):
        m = _mgr(_TC("R68", 1.0), seed)
        m._new_lap(0.0)
        counts.add(len(m.plan))
        picked |= set(m.plan)
    assert counts == {1, 2} and picked <= cand and len(picked) >= 2


def test_spawned_car_comes_from_alley_stops_near_stop_and_route_ends_at_boundary():
    tc = _TC("R112", 10.0)                             # 극동방송(s4) 앞. 골목 G24가 R112.112로 합류한다
    m = _mgr(tc, lead_s=25.0, min_lead_s=3.0)           # 옛 여유(3~25초): 골목 경로 확인
    m._new_lap(0.0)
    m.plan = {S4: m._plan_block(S4)}
    assert m.plan[S4] is not None
    for k in range(200):
        m.update(k * 0.1)
        if tc.added:
            break
    assert tc.added and tc.added[0]["route"][0] == m.plan[S4].alley.feeder
    assert ends_at_boundary(_net(), tc.added[0]["route"][-1])
    vid = tc.added[0]["id"]
    tc.visible.add(vid)                                # 다음 스텝에 SUMO 목록에 나타난다
    m.update(30.0)
    assert tc.stops_set and tc.stops_set[0][0] == vid
    _, edge, pos, lane, dur = tc.stops_set[0]
    assert (edge, round(pos, 3)) == (m.plan[S4].edge, round(m.plan[S4].pos, 3)) and lane == 1 and 15.0 <= dur <= 30.0


def _direct_setup(bus_road="R112", bus_pos=10.0):
    tc = _TC(bus_road, bus_pos)
    m = _mgr(tc)                                       # 기본 여유 15~40초: 이 위치는 골목 차가 15초 먼저 닿지 못한다
    m._new_lap(0.0)
    m.plan = {S4: m._plan_block(S4)}
    m._lap_no = 1                                      # update가 바퀴 계획을 다시 뽑지 않게 한다
    return tc, m, m.plan[S4]


def test_when_alley_car_cannot_make_it_the_car_is_placed_directly_at_the_stop_spot():
    tc, m, b = _direct_setup()
    for k in range(200):
        m.update(k * 0.1)
        if tc.added:
            break
    assert tc.added and b.how == "direct"
    car = tc.added[0]
    assert car["route"][0] == b.edge and car["departSpeed"] == "0" and car["departLane"] == str(b.stop["lane"])
    assert 0.0 <= b.pos - float(car["departPos"]) <= 6.5            # 정차 자리 가까이(6 m 뒤)에서 정지 상태로 세운다
    assert ends_at_boundary(_net(), car["route"][-1])
    tc.visible.add(car["id"])
    m.update(30.0)
    assert tc.stops_set and tc.stops_set[0][1] == b.edge and round(tc.stops_set[0][2], 3) == round(b.pos, 3)
    assert m.summary()["direct"] == 1 and m.summary()["blocks_per_lap"] == [1]


def test_direct_placement_is_skipped_when_the_spot_lane_is_occupied():
    tc, m, b = _direct_setup()
    lane = f"{b.edge}_{b.stop['lane']}"
    tc.lane_cars[lane] = ["x"]
    tc.car_pos["x"] = b.pos - 10.0                      # 정차 자리 10 m 뒤에 차가 달려온다: 부딪힐 수 있다
    for k in range(200):
        m.update(k * 0.1)
    assert tc.added == [] and b.how == "none"


def test_direct_placement_is_skipped_when_the_bus_is_too_close_or_the_spot_is_near_the_bus():
    tc, m, b = _direct_setup(bus_road="R112.112", bus_pos=10.0)
    tc2 = _TC(b.edge, max(0.0, b.pos - 30.0))           # 버스가 정차 자리 30 m 앞(8 m/s로 4초)
    m2 = _mgr(tc2)
    m2._new_lap(0.0)
    m2.plan = {S4: m2._plan_block(S4)}
    m2._lap_no = 1
    for k in range(50):
        m2.update(k * 0.1)
    assert tc2.added == [] and m2.plan[S4].how in ("none", None)
    # 버스 반경(+여유) 안: 위치가 버스에서 55 m 안이면 세우지 않는다
    tc3, m3, b3 = _direct_setup()
    from sumolib.geomhelper import positionAtShapeOffset
    xy = positionAtShapeOffset(_net().getEdge(b3.edge).getShape(), b3.pos)
    tc3.vehicle.getPosition = lambda v: (xy[0] + 30.0, xy[1]) if v == "bus0" else (0.0, 0.0)
    for k in range(200):
        m3.update(k * 0.1)
    assert tc3.added == []


def _s6_mgr(tc, **kw):
    return _mgr(tc, stop_ids=(S2, S6), direct_stops=(S6,), **kw)


def test_s6_is_a_direct_only_candidate_and_stop_ids_drop_the_wausan_and_jandari_stops():
    m = _s6_mgr(_TC("R68", 1.0))
    assert set(m.by_stop) == {S2, S6}                          # s2(골목 있음)와 s6(골목 없음)만. s4 와우산로·s5 잔다리로는 뺐다
    assert m.by_stop[S6] == [] and m.by_stop[S2]
    b = m._plan_block(S6)
    assert b is not None and b.alley is None and b.edge in _route()
    default = _mgr(_TC("R68", 1.0))
    assert S6 not in default.by_stop and len(default.by_stop) >= 3      # 기본 동작은 그대로(골목이 있는 정류장 전부)


def test_s6_car_is_placed_directly_at_the_stop_spot_when_the_bus_is_25_seconds_away():
    from traffic.test_bus import _back
    probe = _s6_mgr(_TC("R68", 1.0))
    b0 = probe._plan_block(S6)
    k = _route().index(b0.edge)
    edge, pos = _back(_net(), _route(), k, b0.pos, 8.0 * 25.0)         # 8 m/s로 25초 앞
    tc = _TC(edge, pos)
    m = _s6_mgr(tc)
    m._new_lap(0.0)
    m.plan = {S6: m._plan_block(S6)}
    m._lap_no = 1
    b = m.plan[S6]
    tc.bus_road, tc.bus_pos = _back(_net(), _route(), _route().index(b.edge), b.pos, 8.0 * 25.0)
    for i in range(30):
        m.update(i * 0.1)
        if tc.added:
            break
    assert tc.added and b.how == "direct"
    car = tc.added[0]
    assert car["route"][0] == b.edge and car["departSpeed"] == "0" and car["departLane"] == str(b.stop["lane"])
    assert ends_at_boundary(_net(), car["route"][-1])
    assert m.summary()["direct"] == 1 and m.summary()["blocks_per_lap"] == [1]


def test_s6_car_waits_until_the_bus_is_within_lead_time_and_is_skipped_if_the_spot_stays_blocked():
    from traffic.test_bus import _back
    probe = _s6_mgr(_TC("R68", 1.0))
    b0 = probe._plan_block(S6)
    k = _route().index(b0.edge)
    far_edge, far_pos = _back(_net(), _route(), k, b0.pos, 8.0 * 60.0)       # 60초 앞: 아직 이르다
    tc = _TC(far_edge, far_pos)
    m = _s6_mgr(tc)
    m._lap_no = 1
    m.plan = {S6: m._plan_block(S6)}
    b = m.plan[S6]
    tc.bus_road, tc.bus_pos = _back(_net(), _route(), _route().index(b.edge), b.pos, 8.0 * 60.0)
    for i in range(30):
        m.update(i * 0.1)
    assert tc.added == [] and not b.done
    tc.bus_road, tc.bus_pos = _back(_net(), _route(), _route().index(b.edge), b.pos, 8.0 * 20.0)      # 20초 앞이지만 자리가 막혔다
    tc.lane_cars[f"{b.edge}_{b.stop['lane']}"] = ["x"]
    tc.car_pos["x"] = b.pos - 10.0
    m.update(5.0)
    assert tc.added == [] and not b.done                                 # 아직 시간이 있으니 다음 스텝에 다시 본다
    tc.bus_road, tc.bus_pos = _back(_net(), _route(), _route().index(b.edge), b.pos, 8.0 * 5.0)       # 5초 앞: 너무 늦었다
    m.update(6.0)
    assert tc.added == [] and b.done and b.how == "none"
