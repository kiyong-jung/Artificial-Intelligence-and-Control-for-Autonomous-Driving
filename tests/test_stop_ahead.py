"""S4: 정류장 앞 정차 자리에 차를 버스가 오기 전에 미리 세워 두고, 버스가 오면 일정 시간 기다렸다가 출발한다(골목에서 나오지 않는다)."""
import json
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from sumolib.geomhelper import positionAtShapeOffset  # noqa: E402

from traffic.alleys import read_net  # noqa: E402
from traffic.presence import ends_at_boundary  # noqa: E402
from traffic.signal_areas import forbidden_intervals  # noqa: E402
from traffic.stop_ahead import StopAheadManager, stop_position  # noqa: E402
from traffic.test_bus import STOP_AT, _back, lap_route  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")
_cache = {}
S2, S4, S5, S6 = STOP_AT


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


class _V:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return ["bus0"] + [v for v in self.tc.cars if v in self.tc.visible]

    def getPosition(self, v):
        return (1e5, 1e5) if v == "bus0" else (0.0, 0.0)

    def getSpeed(self, v):
        return self.tc.bus_speed if v == "bus0" else 8.0

    def getRoadID(self, v):
        return self.tc.car_road.get(v, self.tc.bus_road)

    def getLaneIndex(self, v):
        return self.tc.car_lane.get(v, 1)

    def setRoute(self, v, edges):
        self.tc.routes_set.append((v, list(edges)))

    def changeLane(self, v, lane, dur):
        self.tc.lane_req.append((v, lane, dur))

    def getRoute(self, v):
        return _route()

    def getRouteIndex(self, v):
        return _route().index(self.tc.bus_road.split("#")[0]) if not self.tc.bus_road.startswith(":") else 0

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
        return v in self.tc.stopped

    def getStopState(self, v):
        return 1 if v in self.tc.stopped else 0

    def getStops(self, v, n=1):
        return [1] if v in self.tc.stopped else []

    def resume(self, v):
        self.tc.resumed.append(v)
        self.tc.stopped.discard(v)


class _R:
    def __init__(self, tc):
        self.tc = tc

    def add(self, rid, edges):
        self.tc.routes[rid] = list(edges)


class _L:
    def __init__(self, tc):
        self.tc = tc

    def getLength(self, lane):
        return _net().getEdge(lane.rsplit("_", 1)[0]).getLength()

    def getLastStepVehicleIDs(self, lane):
        return tuple(self.tc.lane_cars.get(lane, ()))


class _TC:
    def __init__(self, bus_road="R68", bus_pos=1.0):
        self.bus_road, self.bus_pos = bus_road, bus_pos
        self.cars, self.visible, self.added, self.routes, self.stops_set = [], set(), [], {}, []
        self.stopped, self.resumed, self.lane_cars, self.car_pos = set(), [], {}, {}
        self.car_road, self.car_lane, self.bus_speed = {}, {}, 8.0
        self.routes_set, self.lane_req = [], []
        self.vehicle, self.route, self.lane = _V(self), _R(self), _L(self)


def _mgr(tc, seed=0, lap=lambda: 1, **kw):
    return StopAheadManager(tc, "bus0", _net(), _route(), _stops(), lap_fn=lap, rng=random.Random(seed), **kw)


def _bus_at(tc, b, back_m):
    tc.bus_road, tc.bus_pos = _back(_net(), _route(), _route().index(b.edge), b.pos, back_m)


def _planned(sid, eta_s=None, **kw):
    """sid 정류장 하나만 계획한 관리자와 그 블록. 버스는 정차 자리 `eta_s`초 앞(8 m/s)에 둔다."""
    tc = _TC()
    m = _mgr(tc, **kw)
    m._lap_no = 1
    m.plan = {sid: m._plan_block(sid)}
    b = m.plan[sid]
    if eta_s is not None:
        _bus_at(tc, b, 8.0 * eta_s)
    return tc, m, b


def test_candidates_follow_stop_ids_and_default_to_all_stopping_stops():
    assert set(_mgr(_TC(), stop_ids=(S2, S6)).candidates) == {S2, S6}
    assert set(_mgr(_TC()).candidates) == set(STOP_AT)


def test_stop_position_keeps_distance_from_stop_and_signal():
    route, bad = _route(), forbidden_intervals(_net(), _route())
    stops = {s["id"]: s for s in _stops()}
    for sid in (S2, S6):
        s = stops[sid]
        for seed in range(40):
            p = stop_position(_net(), route, s, random.Random(seed), not_before=route[0])
            assert p is not None
            edge, pos = p
            assert not any(lo <= pos <= hi for lo, hi in bad.get(edge, []))


def test_car_is_pre_spawned_at_the_stop_spot_not_from_an_alley_when_bus_is_within_lead_time():
    tc, m, b = _planned(S2, eta_s=25.0)
    for i in range(30):
        m.update(i * 0.1)
        if tc.added:
            break
    assert tc.added and b.how == "direct"
    car = tc.added[0]
    assert car["route"][0] == b.edge and not car["route"][0].startswith("G")             # 골목에서 나오지 않는다
    assert car["departSpeed"] == "0" and car["departLane"] == str(b.stop["lane"])
    assert 0.0 <= b.pos - float(car["departPos"]) <= 6.5 and ends_at_boundary(_net(), car["route"][-1])
    assert m.summary()["direct"] == 1 and m.summary()["blocks_per_lap"] == [1]


def test_no_spawn_while_the_bus_is_still_far():
    tc, m, b = _planned(S4, eta_s=60.0)                                                 # 60초 앞: 아직 이르다(s2는 노선 시작에서 가까워 60초 앞을 만들 수 없다)
    for i in range(30):
        m.update(i * 0.1)
    assert tc.added == [] and not b.done


def test_spawn_is_skipped_when_the_spot_is_blocked_and_the_bus_gets_too_close():
    tc, m, b = _planned(S2, eta_s=20.0)
    tc.lane_cars[f"{b.edge}_{b.stop['lane']}"] = ["x"]
    tc.car_pos["x"] = b.pos - 10.0
    m.update(0.0)
    assert tc.added == [] and not b.done                                                # 아직 시간이 있으니 다시 본다
    _bus_at(tc, b, 8.0 * 5.0)                                                           # 5초 앞: 너무 늦었다
    m.update(1.0)
    assert tc.added == [] and b.done and b.how == "none"


def test_spawn_never_happens_inside_the_bus_radius():
    tc, m, b = _planned(S2, eta_s=25.0)
    xy = positionAtShapeOffset(_net().getEdge(b.edge).getShape(), b.pos)
    tc.vehicle.getPosition = lambda v: (xy[0] + 30.0, xy[1]) if v == "bus0" else (0.0, 0.0)
    for i in range(30):
        m.update(i * 0.1)
    assert tc.added == []


def _spawned(sid=S2, **kw):
    tc, m, b = _planned(sid, eta_s=25.0, **kw)
    m.update(0.0)
    vid = tc.added[0]["id"]
    tc.visible.add(vid)
    m.update(0.1)                                                                       # 나타난 다음 스텝에 서 있으라는 명령
    tc.stopped.add(vid)
    m.update(0.2)
    return tc, m, b, vid


def test_car_holds_the_stop_indefinitely_and_does_not_leave_before_the_bus_arrives():
    tc, m, b, vid = _spawned()
    assert tc.stops_set and tc.stops_set[0][0] == vid and tc.stops_set[0][4] >= 1e4     # 대기 시간이 아니라 무기한(출발은 resume)
    assert (tc.stops_set[0][1], round(tc.stops_set[0][2], 3)) == (b.edge, round(b.pos, 3)) and b.confirmed
    for i in range(1, 600):                                                             # 60초가 지나도(버스는 아직 25초 거리) 출발하지 않는다
        m.update(0.2 + i * 0.1)
    assert tc.resumed == [] and b.t_arrive is None


def test_car_leaves_dwell_seconds_after_the_bus_arrives():
    tc, m, b, vid = _spawned(dwell=(20.0, 20.0))
    _bus_at(tc, b, 15.0)                                                                # 버스가 정차 자리 15 m 앞에 왔다
    m.update(10.0)
    assert b.t_arrive == 10.0 and b.release_at == 30.0 and tc.resumed == []
    m.update(29.9)
    assert tc.resumed == []                                                             # 20초 대기가 끝나기 전
    m.update(30.0)
    assert tc.resumed == [vid] and b.released and m.summary()["bus_arrived"] == 1


def test_bus_passing_the_spot_counts_as_arrival():
    tc, m, b, vid = _spawned(dwell=(5.0, 5.0))
    tc.bus_road, tc.bus_pos = _route()[_route().index(b.edge) + 1], 10.0                # 버스가 이미 지나갔다
    m.update(3.0)
    assert b.t_arrive == 3.0 and b.release_at == 8.0


def test_bus_inside_a_junction_does_not_count_as_arrival():
    tc, m, b, vid = _spawned()
    tc.bus_road = ":J23_0"
    m.update(5.0)
    assert b.t_arrive is None


def test_car_is_released_if_the_bus_never_comes_and_at_lap_end():
    tc, m, b, vid = _spawned(max_wait_s=30.0)
    m.update(31.0)
    assert tc.resumed == [vid]
    tc2, m2, b2, vid2 = _spawned()
    m2.lap_fn = lambda: 2                                                               # 바퀴가 끝났다
    m2.update(5.0)
    assert vid2 in tc2.resumed


def test_default_hapjeong_candidates_exclude_s2_and_each_remaining_stop_has_a_spot():
    from traffic.hapjeong import STOP_AHEAD_STOPS
    assert STOP_AHEAD_STOPS == (S4, S5, S6) and S2 not in STOP_AHEAD_STOPS
    m = _mgr(_TC(), stop_ids=STOP_AHEAD_STOPS)
    assert set(m.candidates) == {S4, S5, S6}
    for sid in STOP_AHEAD_STOPS:
        assert m._plan_block(sid) is not None


def test_default_wait_after_the_bus_arrives_is_five_seconds_for_every_candidate_stop():
    from traffic.hapjeong import STOP_AHEAD_STOPS
    for sid in STOP_AHEAD_STOPS:
        tc, m, b = _planned(sid, eta_s=25.0, stop_ids=STOP_AHEAD_STOPS)
        m.update(0.0)
        assert tc.added and b.how == "direct", sid                                      # 세 정류장 모두 정차 자리에 직접 배치
        vid = tc.added[0]["id"]
        tc.visible.add(vid)
        m.update(0.1)
        tc.stopped.add(vid)
        m.update(0.2)
        _bus_at(tc, b, 15.0)                                                            # 버스가 정차 자리 20 m 안으로 왔다
        m.update(10.0)
        assert b.t_arrive == 10.0 and b.release_at == 15.0, sid                         # 5초 뒤 출발
        m.update(14.9)
        assert tc.resumed == []
        m.update(15.0)
        assert tc.resumed == [vid]


def _followers(tc, b, specs):
    """specs: {id: 정차 자리 기준 뒤쪽 거리(m)}. S4 차와 같은 차로에 그 차들이 서 있다."""
    lane_id = f"{b.edge}_{b.stop['lane']}"
    tc.lane_cars[lane_id] = ["bus0"] + list(specs)
    for vid, back in specs.items():
        tc.cars.append(vid)
        tc.visible.add(vid)
        tc.car_road[vid], tc.car_lane[vid], tc.car_pos[vid] = b.edge, b.stop["lane"], b.pos - back


def test_followers_behind_the_waiting_car_are_rerouted_forward_and_sent_to_overtake():
    tc, m, b, vid = _spawned(S2)
    _followers(tc, b, {"f1": 20.0, "f2": 45.0, "far": 75.0})
    m.update(1.0)
    routed = dict(tc.routes_set)
    assert set(routed) == {"f1", "f2"}                                                  # 60 m 밖 차와 시험용 버스는 건드리지 않는다
    for r in routed.values():
        assert r[0] == b.edge and not any(e.startswith("G") for e in r)                 # 골목으로 꺾는 경로를 앞으로 가는 경로로 고친다
        assert ends_at_boundary(_net(), r[-1])
    assert {(v, lane) for v, lane, _ in tc.lane_req} == {("f1", b.stop["lane"] + 1), ("f2", b.stop["lane"] + 1)}   # 왼쪽 차로로 추월
    assert set(m.passing) == {"f1", "f2"} and m.owner.owner("f1") == "S4A"


def test_overtake_waits_for_a_free_adjacent_lane_and_skips_cars_owned_by_others():
    tc, m, b, vid = _spawned(S2)
    _followers(tc, b, {"f1": 20.0, "f2": 30.0})
    left = f"{b.edge}_{b.stop['lane'] + 1}"
    tc.lane_cars[left] = ["x"]
    tc.car_pos["x"] = b.pos - 20.0                                                      # 왼쪽 차로 바로 옆에 차가 있다
    m.owner.claim("f2", "A1")                                                           # f2는 다른 행동(A1)이 쓰고 있다
    m.update(1.0)
    assert dict(tc.routes_set).keys() == {"f1"} and tc.lane_req == []                   # 길은 고치지만 자리가 날 때까지 요청하지 않는다
    tc.lane_cars[left] = []
    m.update(2.5)
    assert tc.lane_req and tc.lane_req[0][:2] == ("f1", b.stop["lane"] + 1)


def test_a_follower_is_released_after_it_has_passed_the_waiting_car():
    tc, m, b, vid = _spawned(S2)
    _followers(tc, b, {"f1": 20.0})
    m.update(1.0)
    assert "f1" in m.passing
    tc.car_pos["f1"] = b.pos + 10.0                                                     # 정차 차를 지나갔다
    m.update(2.0)
    assert "f1" not in m.passing and m.owner.owner("f1") is None


def test_bus_stopped_close_behind_the_queue_counts_as_arrival_after_two_seconds():
    tc, m, b, vid = _spawned(S2, dwell=(5.0, 5.0))
    _bus_at(tc, b, 40.0)                                                                # 줄 선 차들 때문에 정차 자리 40 m 앞에서 못 간다
    tc.bus_speed = 0.0
    m.update(10.0)
    assert b.t_arrive is None
    m.update(12.0)
    assert b.t_arrive == 12.0 and b.release_at == 17.0


def test_bus_still_moving_or_far_away_does_not_count_as_arrival():
    tc, m, b, vid = _spawned(S2)
    _bus_at(tc, b, 40.0)                                                                # 40 m 앞이지만 달리고 있다
    for i in range(50):
        m.update(10.0 + i * 0.1)
    assert b.t_arrive is None
    tc.bus_speed = 0.0
    _bus_at(tc, b, 120.0)                                                               # 120 m 앞에서 서 있다(60 m 밖)
    for i in range(50):
        m.update(20.0 + i * 0.1)
    assert b.t_arrive is None
