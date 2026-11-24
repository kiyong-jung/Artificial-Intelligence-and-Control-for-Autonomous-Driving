"""주입기 바퀴 계획: 매 바퀴 S2·S3·S6, S5 1~2곳, 100% 끼어들기, 버스 반경 밖 생성, 경계로 끝나는 경로."""
import json
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import close_loop, find_merge_alleys, read_net  # noqa: E402
from traffic.presence import PRESENCE_RADIUS_M, ends_at_boundary  # noqa: E402
from traffic.scenario_injector import ScenarioInjector  # noqa: E402
from traffic.signal_areas import unsignalized_alleys  # noqa: E402
from traffic.test_bus import lane_plans, lap_route  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")
_cache = {}


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
    return [{"id": s["id"], "name": s["name"], "edge": s["lane"].rsplit("_", 1)[0], "startPos": s["startPos"],
             "endPos": s["endPos"], "lane_idx": int(s["lane"].rsplit("_", 1)[1])} for s in _map()["bus_stops"]]


def _alleys():
    return unsignalized_alleys(find_merge_alleys(_net(), close_loop(_net(), _map()["routes"]["bus_loop"])))


class _V:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return ["bus0"] + list(self.tc.cars)

    def getPosition(self, v):
        return self.tc.bus_xy if v == "bus0" else (0.0, 0.0)

    def getSpeed(self, v):
        return self.tc.bus_speed if v == "bus0" else 8.0

    def getRoadID(self, v):
        return self.tc.roads.get(v, self.tc.bus_road)

    def getRoute(self, v):
        return self.tc.route

    def getRouteIndex(self, v):
        return self.tc.route.index(self.tc.bus_road)

    def getLanePosition(self, v):
        return self.tc.lane_pos.get(v, self.tc.bus_pos)

    def getLaneIndex(self, v):
        return 2 if v == "bus0" else self.tc.lanes.get(v, 2)

    def getLength(self, v):
        return 12.0

    def getStopState(self, v):
        return self.tc.stop_state

    def getLeftFollowers(self, v):
        return list(self.tc.left_followers)

    def getRightFollowers(self, v):
        return list(self.tc.right_followers)

    def getFollower(self, v, dist):
        return ("", -1.0)

    def add(self, vid, rid, typeID="carDist", **kw):
        self.tc.added.append({"id": vid, "route": self.tc.routes[rid], "type": typeID, **kw})
        self.tc.cars.append(vid)

    def setSpeed(self, v, s):
        self.tc.set_speed[v] = s

    def setParameter(self, v, k, val):
        self.tc.params[(v, k)] = val

    def setLaneChangeMode(self, v, m):
        self.tc.lc_mode[v] = m

    def getLaneChangeMode(self, v):
        return self.tc.lc_mode.get(v, 1621)

    def getLaneID(self, v):
        return "R71_2"

    def getAllowedSpeed(self, v):
        return 13.89

    def setStop(self, vid, edge, pos=1.0, laneIndex=0, duration=-1, **kw):
        self.tc.stops_set.append((vid, edge, round(pos, 3), laneIndex, duration))

    def resume(self, vid):
        if self.tc.resume_fail:
            raise RuntimeError("not stopped")
        self.tc.resumed.append(vid)


class _R:
    def __init__(self, tc):
        self.tc = tc

    def add(self, rid, edges):
        self.tc.routes[rid] = list(edges)


class _L:
    def getLength(self, lane):
        return _net().getEdge(lane.rsplit("_", 1)[0]).getLength()

    def getMaxSpeed(self, lane):
        return 13.89


class _E:
    def __init__(self, tc=None):
        self.tc = tc

    def getLaneNumber(self, e):
        return _net().getEdge(e).getLaneNumber()

    def getLastStepVehicleIDs(self, e):
        return tuple(self.tc.edge_vehicles.get(e, ()))


class _P:
    def getIDList(self):
        return []


class _FakeTC:
    """버스 상태와 기록. route는 버스 경로(도로 id 목록)다."""

    def __init__(self, bus_road="R71", bus_pos=20.0, bus_xy=(1e5, 1e5), bus_speed=8.0):
        self.bus_road, self.bus_pos, self.bus_xy, self.bus_speed = bus_road, bus_pos, bus_xy, bus_speed
        self.route = _route()
        self.cars, self.added, self.routes, self.set_speed, self.lc_mode = [], [], {}, {}, {}
        self.stop_state = 0
        self.stops_set, self.resumed, self.resume_fail = [], [], False
        self.edge_vehicles = {}          # 도로 id -> 차 id 목록(버스 제외)
        self.params = {}                 # (차 id, 키) -> 값
        self.roads, self.lane_pos = {}, {}   # 다른 차 id -> 도로 / 도로 위 위치(없으면 버스와 같다)
        self.left_followers, self.right_followers = [], []     # 버스 옆 차로의 후행차 [(id, TraCI 간격)]
        self.lanes = {}                  # 다른 차 id -> 차로 번호
        self.vehicle = _V(self)
        self.lane, self.edge, self.person = _L(), _E(self), _P()


class _TC2:
    """route 속성 이름이 traci와 같게(tc.route.add) 쓰이도록 감싼다."""

    def __init__(self, **kw):
        self.f = _FakeTC(**kw)
        self.vehicle, self.lane, self.edge, self.person = self.f.vehicle, self.f.lane, self.f.edge, self.f.person
        self.route = _R(self.f)


def _inj(tc, lap=lambda: 1, alleys=None, seed=0, a1_active=None):
    from traffic.test_bus import STOP_AT
    plans = lane_plans(_net(), _route(), _stops())
    return ScenarioInjector(tc, _net(), "bus0", _stops(), _alleys() if alleys is None else alleys, "carDist",
                            lap_fn=lap, plans=plans, rng=random.Random(seed), a1_active=a1_active, stop_ids=STOP_AT)


def test_lap_plan_has_both_s2_one_s3_and_one_or_two_s5():
    for seed in range(10):
        inj = _inj(_TC2(), seed=seed)
        inj.update(0.0)
        p = inj._lap
        assert [x.kind for x in p["S2"]] == ["S2"] * 4 and p["S3"].kind == "S3"
        assert 1 <= len(p["S5"]) <= 2 and len({a.junction for a in p["S5"]}) == len(p["S5"]) and p["S6"] is False


def test_new_lap_follows_pilot_lap_not_bus_presence():
    lap = {"n": 1}
    inj = _inj(_TC2(), lap=lambda: lap["n"])
    inj.update(0.0)
    inj.update(1.0)
    assert len(inj._laps_done) == 0
    lap["n"] = 2
    inj.update(2.0)
    assert len(inj._laps_done) == 1


def test_catch_up_speeds_follower_toward_bus_then_matches():
    tc = _TC2(bus_road="R112.221", bus_pos=10.0, bus_speed=8.0)
    active = set()
    inj = _inj(tc, a1_active=lambda vid: vid in active)
    inj.update(0.0)
    plan = inj._lap["S2"][-1]
    tc.f.cars.append("f")
    inj._track(plan, "f", 0.0)
    gaps = {"g": 30.0}
    inj._follower_gap = lambda vid, p: gaps["g"]
    inj._catch_up(0.1)
    assert tc.f.set_speed["f"] == pytest.approx(16.0)           # 8 + min(8, (30-5.5)/2): 최대한 빨리 다가온다
    gaps["g"], tc.f.bus_speed = 60.0, 15.0
    inj._catch_up(0.15)
    assert tc.f.set_speed["f"] == pytest.approx(13.89 + 8.0)    # 15 + 8 = 23이지만 도로 제한 속도 + 8 m/s(21.9)까지
    tc.f.bus_speed = 8.0
    gaps["g"] = 8.0                                             # 아직 A1 시작 거리 밖: 계속 다가온다(최소 버스 + 2 m/s)
    inj._catch_up(0.18)
    assert tc.f.set_speed["f"] == pytest.approx(10.0)
    gaps["g"] = 4.0                                             # A1 시작 거리 안: 나란히 간다(A1이 곧 켜진다)
    inj._catch_up(0.2)
    assert tc.f.set_speed["f"] == pytest.approx(8.0)
    gaps["g"] = -2.0                                            # 버스 옆이나 앞: 확실히 뒤로 물러난다
    inj._catch_up(0.25)
    assert tc.f.set_speed["f"] == pytest.approx(5.0)
    active.add("f")                                             # A1 상호작용 시작: 손을 뗀다
    inj._catch_up(0.3)
    assert tc.f.set_speed["f"] == -1


def test_nearest_follower_is_the_partner_whoever_it_is():
    # 주입한 차보다 가까운 앞차(자연 후행차)가 있으면 시나리오 상대는 그 차다. 주입 차는 상대가 아니다
    tc = _TC2(bus_road="R112.221", bus_pos=10.0, bus_speed=8.0)
    inj = _inj(tc)
    inj.update(0.0)
    plan = inj._lap["S2"][-1]
    tc.f.cars += ["inj", "nat"]
    inj._track(plan, "inj", 0.0)
    inj._follower_gap = lambda vid, p: 30.0                       # 주입 차는 30 m 뒤
    assert inj._partner(plan, inj._lap["lane"].get(inj._key(plan)))[0] == "inj"
    tc.f.left_followers = [("nat", 2.0)]                          # 자연 후행차가 버스 2 m 뒤(왼쪽 차로)
    assert inj._partner(plan, inj._lap["lane"].get(inj._key(plan)))[0] == "nat"   # 상대가 바뀌었다
    inj._catch_up(0.1)
    assert tc.f.set_speed["inj"] == -1 and inj._lap["lane"][("S2", "R54")]["released"]   # 주입 차는 손을 뗀다


def test_injected_follower_stays_partner_when_it_is_the_nearest():
    tc = _TC2(bus_road="R112.221", bus_pos=10.0, bus_speed=8.0)
    inj = _inj(tc)
    inj.update(0.0)
    plan = inj._lap["S2"][-1]
    tc.f.cars += ["inj", "far"]
    inj._track(plan, "inj", 0.0)
    inj._follower_gap = lambda vid, p: 8.0
    tc.f.left_followers = [("far", 25.0)]                         # 더 먼 자연 차
    inj._catch_up(0.1)
    assert tc.f.set_speed["inj"] == pytest.approx(10.0) and not inj._lap["lane"][("S2", "R54")]["released"]


def test_injector_has_no_gate():
    assert not hasattr(ScenarioInjector, "gate")


def _dongmak3():
    return [a for a in _alleys() if a.junction == "X_dongmak3"]


def _near_merge():
    return _net().getEdge("R71").getLength() - 40.0           # 합류점(R71.135 시작) 40 m 앞






def test_no_spawn_within_radius_of_bus():
    from sumolib.geomhelper import positionAtShapeOffset
    g3 = _net().getEdge("G3")
    xy = positionAtShapeOffset(g3.getShape(), 1.0)
    tc = _TC2(bus_road="R71", bus_pos=_near_merge(), bus_xy=(xy[0] + 0.5 * PRESENCE_RADIUS_M, xy[1]))
    inj = _inj(tc, alleys=_dongmak3())
    inj.update(0.0)
    inj._s5(0.1, inj._state())
    assert not any(a["id"].startswith("inj_s5") for a in tc.f.added)


def _s3_window_start():
    p = next(x for x in lane_plans(_net(), _route(), _stops()) if x.kind == "S3")
    back = p.length - p.end_pos - _net().getEdge("R71.135").getLength()   # R71 위에서 창이 시작하는 곳까지
    return _net().getEdge("R71").getLength() - back


def test_follower_waits_until_bus_has_passed_last_signal_before_window():
    # S3 창 앞에는 신호 교차로 J8(R69a 끝)이 있다. 버스가 그 앞에 있을 때 넣으면 후행차가 적색에 걸려 늦었다(3바퀴 측정)
    tc = _TC2(bus_road="R69", bus_pos=60.0)
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    assert not any(a["id"].startswith("inj_lane") for a in tc.f.added)
    tc.f.bus_road, tc.f.bus_pos = "R71", 70.0                                   # J8을 지났다
    inj._lanes(0.1, inj._state())
    lane = [a for a in tc.f.added if a["id"].startswith("inj_lane")]
    assert lane and ends_at_boundary(_net(), lane[0]["route"][-1]) and lane[0]["route"][0] == "R71"
    assert float(lane[0]["departPos"]) == pytest.approx(70.0 - (PRESENCE_RADIUS_M + 15.0), abs=0.2)
    assert lane[0]["departLane"] == "1" and tc.f.lc_mode[lane[0]["id"]] == 0


def test_platoon_is_queued_behind_while_bus_waits_at_red_before_the_window():
    # S3 창 앞 신호(J8)에서 버스가 적색으로 서 있다. 목표 차로 줄에 후행차를 50 m 이상 뒤부터 여러 대 넣어 녹색에 같이 출발하게 한다
    tc = _TC2(bus_road="R69", bus_pos=_net().getEdge("R69").getLength() - 3.0, bus_speed=0.0)
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    added = [a for a in tc.f.added if a["id"].startswith("inj_lane")]
    assert len(added) >= 2                                                    # 여러 대
    assert all(a["route"][0] == "R69" for a in added)
    assert all(float(a["departPos"]) <= _net().getEdge("R69").getLength() - 3.0 - PRESENCE_RADIUS_M + 0.5 for a in added)
    assert len({a["departPos"] for a in added}) == len(added)                  # 서로 다른 자리
    assert inj._lap["lane"][("S3", "R71.206")]["vid"] == added[0]["id"]       # 버스에 가장 가까운(앞) 차가 상대로 시작한다


def test_no_platoon_while_the_bus_is_still_moving_toward_the_signal():
    tc = _TC2(bus_road="R69", bus_pos=20.0, bus_speed=8.0)
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    assert not any(a["id"].startswith("inj_lane") for a in tc.f.added)


def test_follower_after_signal_starts_10m_into_the_road():
    # 신호 바로 뒤 도로 1 m에 넣으면 교차로를 막 빠져나오던 차와 부딪혔다(baseline 측정, J42). 10 m 안쪽에서 출발시킨다
    tc = _TC2(bus_road="R71", bus_pos=62.0)                                   # 65 m 뒤는 J8 너머: 신호 뒤 도로 시작으로 당긴다
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    lane = [a for a in tc.f.added if a["id"].startswith("inj_lane")]
    assert lane and lane[0]["route"][0] == "R71" and float(lane[0]["departPos"]) == pytest.approx(10.0)


def test_near_bus_is_retried_during_first_half_of_window():
    # J82 창은 신호(cluster) 뒤 R77 20 m에서 시작한다. 버스가 창에 들어선 직후에는 출발 지점이 버스 반경 안이라
    # 예전에는 바로 포기했다(매 바퀴 J82 후행차 없음). 창 앞 절반 동안은 다시 본다
    tc = _TC2(bus_road="R77", bus_pos=25.0)
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    assert ("S2", "R111b") not in inj._lap["lane_done"]
    tc.f.bus_pos = 62.0
    inj._lanes(0.1, inj._state())
    assert any(a["id"].startswith("inj_lane") and a["route"][0] == "R77" for a in tc.f.added)


def test_followers_in_other_lanes_keep_moving_while_bus_dwells():
    # 버스가 정류장에 서 있을 때 버스 차로에 없는 후행차를 버스 뒤에 두려고 세우면 부자연스럽다. 묶지 않고 그대로 달리게 한다
    tc = _TC2(bus_road="R112.221", bus_pos=10.0, bus_speed=0.0)
    inj = _inj(tc)
    inj.update(0.0)
    plan = inj._lap["S2"][-1]
    tc.f.cars.append("f")
    tc.f.lanes["f"] = 1                                          # 버스(차로 2)와 다른 차로
    inj._track(plan, "f", 0.0)
    inj._follower_gap = lambda vid, p: 4.0
    tc.f.stop_state = 1 | 16
    inj.update(0.1)                                              # 정차 시작
    inj._catch_up(0.2)
    assert tc.f.set_speed["f"] == -1                             # 멈추게 하지 않는다(속도 지정을 푼다)


def test_follower_that_passed_the_bus_is_let_go():
    tc = _TC2(bus_road="R112.221", bus_pos=10.0, bus_speed=8.0)
    inj = _inj(tc)
    inj.update(0.0)
    plan = inj._lap["S2"][-1]
    tc.f.cars.append("f")
    inj._track(plan, "f", 0.0)
    inj._follower_gap = lambda vid, p: -15.0                     # 이미 버스를 지나 앞에 있다: 되돌아오게 하지 않는다
    inj._catch_up(0.1)
    assert tc.f.set_speed["f"] == -1 and inj._lap["lane"][("S2", "R54")]["released"]


def test_j1_follower_waits_until_bus_leaves_s6():
    # J1 창은 합정역(s6) 안에서 시작한다. 정차 전에 넣으면 정차하는 동안 옆 차로 후행차가 지나가 버린다
    tc = _TC2(bus_road="R54a", bus_pos=250.0)                                  # s6(245.5~270.5) 안
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    assert not any(a["id"].startswith("inj_lane") for a in tc.f.added)


def test_j1_follower_is_in_the_lane_feeding_the_left_turn_lane():
    # J1 앞은 차로를 여러 칸 옮긴다. 후행차는 마지막 변경(좌회전 차로로 들어가는 변경)의 목표 차로에 있어야 한다
    tc = _TC2(bus_road="R54b", bus_pos=20.0)                                   # s6을 떠났다
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    inj._lanes(0.1, inj._state())
    lane = [a for a in tc.f.added if a["id"].startswith("inj_lane")]
    assert lane and lane[0]["route"][0] == "R54a" and lane[0]["departLane"] == "3"
    # 좌회전 도착 도로(R68)까지 버스와 같은 길로 가야 SUMO가 좌회전 차로(R54b 4 -> R54 5)를 고른다. 예전에는 R42에서
    # 곧장 나가는 경로라 R54b 3 -> R54 4(버스의 중간 목표 차로)로 가 중간 변경에서 A1이 켜지고 마지막 변경 상대가 없었다
    assert lane[0]["route"][:5] == ["R54a", "R54b", "R54", "R42", "R68"]
    p = inj._lap["S2"][-1]
    assert inj._partner(p, inj._lap["lane"].get(inj._key(p))) is not None


def test_released_follower_gets_its_lane_change_mode_back():
    tc = _TC2(bus_road="R71", bus_pos=70.0)
    active = set()
    inj = _inj(tc, alleys=[], a1_active=lambda vid: vid in active)
    inj.update(0.0)
    vid = next(a["id"] for a in tc.f.added if a["id"].startswith("inj_lane"))
    active.add(vid)
    inj._catch_up(0.2)
    assert tc.f.lc_mode[vid] == 1621 and tc.f.set_speed[vid] == -1


def test_no_back_point_is_retried_not_given_up():
    # 첫 바퀴 시작(R68)에서는 노선 뒤가 없다. 포기하지 말고 버스가 나아간 뒤 다시 본다
    tc = _TC2(bus_road="R68", bus_pos=1.0)
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    assert ("S3", "R71.206") not in inj._lap["lane_done"]


def test_near_bus_keeps_being_retried_until_the_window_ends():
    # S2·S3는 항상 일어나야 한다: 창 뒤 절반에서도 키퍼가 못 넣었다고 포기하지 않는다
    tc = _TC2(bus_road="R77", bus_pos=25.0)
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    tc.f.bus_pos = 62.0
    inj._lanes(0.1, inj._state())
    assert ("S2", "R111b") not in inj._lap["lane_done"]


def test_window_that_ended_without_a_partner_is_recorded_as_missed():
    # S2·S3가 안 일어난 창은 조용히 지나가지 않고 'missed'로 남겨 측정에서 보이게 한다
    tc = _TC2(bus_road="R77", bus_pos=25.0)
    inj = _inj(tc, alleys=[])
    inj.update(0.0)
    plan = inj._lap["S2"][-1]
    key = inj._key(plan)
    inj._lap["seen"].add(key)
    orig = inj._dist_to
    inj._dist_to = lambda st, edge, pos: -1.0 if (edge, pos) == (plan.edge, plan.end_pos) else orig(st, edge, pos)   # 이 창은 이미 끝났다
    inj._lanes(5.0, inj._state())
    assert key in inj._lap["lane_done"] and any(r["ok"] == "missed" and r["kind"] == "S2" for r in inj.log)


def test_injector_with_catch_up_off_tracks_the_partner_without_touching_its_speed():
    # catch_up=False: 상대는 고르기만 하고 속도를 건드리지 않는다. 5 m 안 도착을 보장하지 않는다
    tc = _TC2(bus_road="R112.221", bus_pos=10.0, bus_speed=8.0)
    plans = lane_plans(_net(), _route(), _stops())
    from traffic.test_bus import STOP_AT
    inj = ScenarioInjector(tc, _net(), "bus0", _stops(), [], "carDist", lap_fn=lambda: 1, plans=plans,
                           rng=random.Random(0), stop_ids=STOP_AT, catch_up=False)
    inj.update(0.0)
    plan = inj._lap["S2"][-1]
    tc.f.cars.append("f")
    inj._track(plan, "f", 0.0)
    inj._follower_gap = lambda vid, p: 30.0                      # 30 m 뒤: catch_up이면 속도를 높였을 거리
    tc.f.set_speed.pop("f", None)
    inj._catch_up(0.1)
    assert "f" not in tc.f.set_speed or tc.f.set_speed["f"] == -1
