"""시험용 버스 조종(traffic/test_bus.py): R68 출발, 끊김 없는 바퀴, 정해진 차선 변경 창."""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import read_net  # noqa: E402
from traffic.test_bus import BUS, STOP_AT, TestBusPilot, lane_plans, lap_route  # noqa: E402

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


def _loop():
    return _map()["routes"]["bus_loop"]


def _stops():
    return [{"id": s["id"], "name": s["name"], "edge": s["lane"].rsplit("_", 1)[0], "startPos": s["startPos"],
             "endPos": s["endPos"], "lane_idx": int(s["lane"].rsplit("_", 1)[1])} for s in _map()["bus_stops"]]


class _V:
    def __init__(self, tc):
        self.tc = tc

    def add(self, vid, rid, **kw):
        self.tc.added.append((vid, rid, kw))
        self.tc.alive = True
        self.tc.routes[vid] = list(self.tc.route_defs[rid])

    def getIDList(self):
        return ([BUS] if self.tc.alive else []) + list(self.tc.others)

    def getRoadID(self, v):
        return self.tc.others[v]["road"] if v in self.tc.others else self.tc.road

    def getLaneIndex(self, v):
        return self.tc.lane

    def getLaneID(self, v):
        return f"{self.tc.road}_{self.tc.lane}"

    def getLanePosition(self, v):
        return self.tc.others[v]["pos"] if v in self.tc.others else self.tc.pos

    def getLength(self, v):
        return self.tc.others[v]["len"] if v in self.tc.others else 12.0

    def getRoute(self, v):
        return self.tc.routes[v]

    def getRouteIndex(self, v):
        road = self.tc.prev_road if self.tc.road.startswith(":") else self.tc.road
        return self.tc.routes[v].index(road)

    def setRoute(self, v, edges):
        self.tc.routes[v] = list(edges)

    def setBusStop(self, v, sid, duration=0):
        self.tc.bus_stops.append((sid, duration))

    def setLaneChangeMode(self, v, mode):
        self.tc.lc_mode.append(mode)

    def setSignals(self, v, s):
        self.tc.signals = s

    def getSignals(self, v):
        return self.tc.signals

    def changeLane(self, v, lane, dur):
        self.tc.change_req.append(lane)

    def getSpeed(self, v):
        return self.tc.others[v]["speed"] if v in self.tc.others else self.tc.speed

    def getAllowedSpeed(self, v):
        return 13.89

    def setSpeed(self, v, s):
        self.tc.set_speed.append(s)

    def getStopState(self, v):
        return 0

    def getBestLanes(self, v):
        return [(f"{self.tc.road}_{self.tc.lane}", 100.0, 0.0, self.tc.best_offset, self.tc.continues, [])]

    def getLeftFollowers(self, v):
        return self.tc.followers

    def getRightFollowers(self, v):
        return self.tc.followers


class _R:
    def __init__(self, tc):
        self.tc = tc

    def add(self, rid, edges):
        self.tc.route_defs[rid] = list(edges)


class _E:
    def getLaneNumber(self, e):
        return _net().getEdge(e).getLaneNumber()


class _FakeTC:
    def __init__(self, road="R68", lane=1, pos=0.0, speed=8.0):
        self.road, self.lane, self.pos, self.speed = road, lane, pos, speed
        self.prev_road = None            # 교차로 안(내부 도로)이면 방금 지난 경로 도로
        self.alive = False
        self.added, self.bus_stops, self.lc_mode, self.change_req, self.set_speed = [], [], [], [], []
        self.routes, self.route_defs = {}, {}
        self.signals, self.best_offset, self.continues, self.followers = 0, 0, True, []
        self.others = {}                 # 다른 차 id -> {"road", "pos", "len", "speed"}
        self.vehicle, self.route, self.edge = _V(self), _R(self), _E()


def _pilot(tc, **kw):
    return TestBusPilot(tc, _net(), _loop(), _stops(), **kw)


def _run(pilot, tc, t0, t1, dt=0.1):
    t = t0
    while t < t1 - 1e-9:
        pilot.tick(tc, t)
        t = round(t + dt, 6)
    return t


def test_lap_route_starts_at_r68_and_closes_on_itself():
    r = lap_route(_net(), _loop())
    assert r[0] == "R68" and r[1] == "R69" and r[-1] == "R42"


def test_stop_set_is_s2_s4_s5_s6():
    assert STOP_AT == ("osm_stop_357920024", "osm_stop_5930628870", "osm_stop_3446267493", "osm_stop_3424857165")


def test_lane_plans_are_s3_then_two_s2_and_end_before_signals():
    p = lane_plans(_net(), lap_route(_net(), _loop()), _stops())
    assert [x.kind for x in p] == ["S3", "S2", "S2"]
    # 정류장 차로에는 정류장 시작 10 m 전까지 들어가야 한다(성산중학교 13.31 m -> 3.31 m)
    assert (p[0].edge, p[0].target_lane) == ("R71.206", 1) and p[0].end_pos == pytest.approx(3.31, abs=0.01)
    assert (p[1].edge, p[1].target_lane) == ("R111b", 2) and p[1].end_pos == pytest.approx(7.0, abs=0.5)
    # J1은 3.7.0부터 R42 차로 3도 좌회전(R68 차로 1)이다. 버스는 더 가까운 차로 3을 목표로 한다
    assert (p[2].edge, p[2].target_lane) == ("R42", 3) and p[2].end_pos == pytest.approx(6.0, abs=0.5)
    assert all(x.length >= 120.0 for x in p) and p[2].length > p[1].length     # J1 앞은 차로를 더 많이 옮긴다
    # J1 창은 합정역(s6, R54a 245.5~270.5 m) 안에서 시작한다. 감속 판단은 정류장 끝부터 센다
    net = _net()
    eff = (net.getEdge("R54a").getLength() - 270.5 + net.getEdge("R54b").getLength() + net.getEdge("R54").getLength()
           + p[2].end_pos)
    # 3.7.0: 목표 차로가 4 -> 3이라 창이 240 -> 200 m로 줄어 s6 끝(창 끝 211.6 m 전)이 창 밖이다. 그래서 창 전체를 센다
    assert eff > p[2].length and p[2].eff_length == p[2].length
    assert p[0].eff_length == p[0].length and p[1].eff_length == p[1].length


def test_pilot_departs_once_from_r68_and_sets_stops():
    tc = _FakeTC()
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    pilot.tick(tc, 0.1)
    assert len(tc.added) == 1 and tc.added[0][0] == BUS
    assert tc.route_defs[tc.added[0][1]][0] == "R68" and tc.added[0][2]["departLane"] == "1"
    assert [s for s, _ in tc.bus_stops] == list(STOP_AT) and pilot.lap == 1


def test_pilot_extends_route_instead_of_respawning():
    tc = _FakeTC()
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.pos = "R54b", 5.0
    tc.bus_stops.clear()
    pilot.tick(tc, 1.0)
    r = tc.routes[BUS]
    assert len(tc.added) == 1
    i = r.index("R54b")
    assert r[i:i + 3] == ["R54b", "R54", "R42"] and r[i + 3] == "R68"
    assert [s for s, _ in tc.bus_stops] == list(STOP_AT)


def test_lap_counts_when_bus_reenters_r68():
    tc = _FakeTC()
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road = "R42"
    pilot.tick(tc, 500.0)
    tc.road = "R68"
    pilot.tick(tc, 501.0)
    assert pilot.lap == 2 and pilot.lap_times == [pytest.approx(501.0)]


def test_pilot_never_requests_change_inside_signal_cutoff():
    tc = _FakeTC(road="R111b", lane=1, pos=10.0)      # J82 정지선 앞 17 m: 금지 영역
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.pos = "R111b", 10.0
    _run(pilot, tc, 0.1, 6.0)
    assert tc.change_req == [] and tc.signals == 0


def test_pilot_hands_over_to_sumo_when_window_ends_in_wrong_lane():
    tc = _FakeTC()
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R42", 2, 6.5
    pilot.tick(tc, 1.0)
    assert tc.lc_mode[-1] == 1621 and pilot.missed == [(1, "S2", "R42")]
    tc.road, tc.lane, tc.pos = "R68", 2, 1.0
    pilot.tick(tc, 2.0)
    assert tc.lc_mode[-1] == 512


def test_pilot_holds_lane_outside_windows():
    tc = _FakeTC()
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos, tc.best_offset = "R54a", 1, 50.0, 3     # J1까지 차로 3칸이 필요하지만 아직 창 밖
    _run(pilot, tc, 0.1, 6.0)
    assert tc.change_req == []


def test_pilot_changes_in_window_without_follower_after_signal():
    tc = _FakeTC()
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R71.135", 2, 40.0                    # S3 창 안, 목표 차로 1
    _run(pilot, tc, 0.1, 3.5)
    assert tc.change_req and tc.change_req[0] == 1 and tc.set_speed == []


def test_follow_on_change_in_same_window_is_quick():
    # J1 앞은 차로를 3~4칸 옮겨야 한다. 이어지는 변경은 식힘 없이 방향지시등 1초 뒤 바로 요청한다
    tc = _FakeTC()
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos, tc.best_offset = "R54", 2, 10.0, 3
    _run(pilot, tc, 0.1, 3.2)                       # 첫 변경: 방향지시등 3초 뒤 요청
    assert tc.change_req == [3]
    tc.lane, tc.pos, tc.best_offset = 3, 40.0, 2     # 첫 변경이 끝났다
    _run(pilot, tc, 3.2, 3.4)
    assert pilot.phase == "signal" and tc.signals != 0
    _run(pilot, tc, 3.4, 4.5)                       # 1초 안팎 뒤 다음 요청
    assert tc.change_req[-1] == 4


def test_bus_stops_to_guarantee_arrival_in_target_lane_before_window_end():
    # 정류장 차로 10 m 전 도착 보장이 S2·S3 상호작용보다 우선이다. 창 끝이 가까운데 아직 목표 차로가 아니면 버스가 멈춰서
    # 목표 차로의 차를 보내고 빈틈에 차선을 바꾼다(양보형·비양보형과 무관)
    tc = _FakeTC(speed=10.0)
    pilot = _pilot(tc, follower_ready=lambda plan: False)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R71.135", 2, 45.0          # S3 창 끝(R71.206 3.3 m)까지 약 22 m, 목표 차로 1
    tc.followers = [("x", 1.0)]                              # 목표 차로에 차가 바로 옆에 있다(차선 변경 불가)
    _run(pilot, tc, 0.1, 1.0)
    assert tc.set_speed and tc.set_speed[-1] == 0.0          # 멈춘다
    assert tc.change_req and tc.signals != 0                 # 방향지시등을 켜고 차선 변경을 계속 요청한다
    assert pilot.forced == [(1, "S3")]
    tc.lane, tc.followers = 1, []                            # 빈틈이 생겨 목표 차로로 옮겼다
    _run(pilot, tc, 1.0, 1.5)
    assert tc.set_speed[-1] == -1                            # 멈춤을 푼다


def test_forced_stop_compares_lanes_through_the_connections_not_by_number():
    # R54 차로 4는 R42 차로 3(좌회전 차로)으로, 차로 5는 R42 차로 4로 이어진다. 창 끝 도로(R42)의 목표 차로 번호(3)와 지금 차로 번호를
    # 그대로 비교하면, 이미 맞는 차로(4, 5)에 있는 버스를 멈춰 세우고 틀린 차로(3, R42 차로 2로 직진)로 옮기라고 했다
    L = _net().getEdge("R54").getLength()
    for lane in (4, 5):
        tc = _FakeTC(speed=12.0)
        pilot = _pilot(tc, follower_ready=lambda plan: False)
        pilot.tick(tc, 0.0)
        tc.road, tc.lane, tc.pos = "R54", lane, L - 20.0       # 창 끝(R42 6 m)까지 26 m: 제동 거리(32.8 m) 안
        _run(pilot, tc, 0.1, 2.0)
        assert 0.0 not in tc.set_speed and pilot.forced == [], lane


def test_forced_stop_moves_toward_the_lane_that_reaches_the_target():
    # R54 차로 2는 R42 차로 1(직진)로 간다. 멈춰서 옮길 때는 R42 차로 3으로 이어지는 가장 가까운 차로(4)로 간다(위쪽 +1)
    L = _net().getEdge("R54").getLength()
    tc = _FakeTC(speed=12.0)
    pilot = _pilot(tc, follower_ready=lambda plan: False)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R54", 3, L - 20.0              # 차로 3은 R42 차로 2(직진)로 간다: 목표(R42 3)에 못 닿는다
    _run(pilot, tc, 0.1, 1.0)
    assert tc.set_speed[-1] == 0.0 and tc.change_req and tc.change_req[0] == 4


def test_no_forced_stop_when_far_from_window_end_or_already_in_target_lane():
    tc = _FakeTC(speed=10.0)
    pilot = _pilot(tc, follower_ready=lambda plan: False)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R71", 2, 20.0                # 창 끝까지 100 m 넘게 남았다
    _run(pilot, tc, 0.1, 1.0)
    assert 0.0 not in tc.set_speed
    tc.road, tc.lane, tc.pos = "R71.135", 1, 45.0            # 이미 목표 차로
    _run(pilot, tc, 1.0, 2.0)
    assert 0.0 not in tc.set_speed and pilot.forced == []


def test_bus_slows_and_lets_blocker_pass_then_changes():
    tc = _FakeTC()
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R71.135", 2, 20.0                    # S3 창 안, 목표 차로 1
    tc.others["f"] = {"road": "R71.135", "pos": 12.0, "len": 4.5, "speed": 10.0}
    tc.followers = [("f", 3.5)]
    _run(pilot, tc, 0.1, 3.2)                                         # 방향지시등 3초 뒤 첫 요청
    assert pilot.phase == "request" and tc.set_speed == []
    _run(pilot, tc, 3.2, 4.4)                                         # 후행차가 버스보다 2 m/s 빠른 상태가 1초 넘게 이어짐
    assert tc.set_speed and tc.set_speed[-1] == pytest.approx(max(8.0 - 3.0, 2.0))
    n = len(tc.change_req)
    tc.others["f"]["pos"] = tc.pos + 4.5 + 6.0                        # 비양보형이 버스를 지나 앞 6 m
    tc.followers = []
    _run(pilot, tc, 4.4, 4.6)
    assert tc.set_speed[-1] == -1 and len(tc.change_req) > n and tc.change_req[-1] == 1


def test_blocker_on_next_short_road_is_still_tracked():
    # 비양보형이 다음 짧은 도로(R111b 27 m)로 넘어가도 노선 거리로 따라간다. 도로 이름이 다르다고 놓으면 버스가 그 차 앞으로
    # 끼어들 수 있었다(리뷰 지적)
    tc = _FakeTC(speed=2.0)                                            # 느리게 달려 창 끝의 제동 거리 밖(도착 보장 정지와 겹치지 않게)
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    L = _net().getEdge("R111").getLength()
    tc.road, tc.lane, tc.pos, tc.best_offset = "R111", 1, L - 10.0, 1  # J82 창 안, 왼쪽으로 한 칸
    tc.others["f"] = {"road": "R111", "pos": L - 18.0, "len": 4.5, "speed": 10.0}
    tc.followers = [("f", 0.5)]
    _run(pilot, tc, 0.1, 4.4)
    assert tc.set_speed and tc.set_speed[-1] > 0                       # 비양보형으로 보고 속도를 줄였다
    n = len(tc.change_req)
    tc.others["f"].update(road="R111b", pos=2.0)                        # 다음 도로, 앞끝이 버스 앞 5 m(뒤끝은 0.5 m)
    tc.pos, tc.followers = L - 3.0, []
    _run(pilot, tc, 4.4, 5.0)
    assert tc.set_speed[-1] > 0 and len(tc.change_req) == n            # 아직 지나가지 않았다: 계속 기다린다
    tc.others["f"]["pos"] = 20.0                                        # 뒤끝이 버스 앞끝보다 5 m 넘게 앞섰다
    _run(pilot, tc, 5.0, 5.2)
    assert tc.set_speed[-1] == -1 and len(tc.change_req) > n


def test_pilot_slows_but_never_waits_for_follower():
    tc = _FakeTC()
    state = {"ready": False}
    pilot = _pilot(tc, follower_ready=lambda plan: state["ready"])
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R71", 2, _net().getEdge("R71").getLength() - 20.0   # S3 창 앞 절반(창 끝까지 약 98 m)
    _run(pilot, tc, 0.1, 3.5)
    # 감속 속도는 지금 속도가 아니라 도로 제한 속도 기준이다(막 출발한 버스가 3 m/s로 30초 기던 문제)
    # 제한 속도의 45%(6.3 m/s)로 줄여 후행차가 3 m까지 따라붙을 시간을 번다(60%에서는 후행차가 20 m 뒤에서 A1이 못 켜졌다)
    assert tc.change_req == [] and tc.set_speed and tc.set_speed[0] == pytest.approx(max(0.45 * 13.89, 3.0))
    assert tc.signals != 0                                            # 방향지시등은 켠 채로 달린다
    state["ready"] = True
    _run(pilot, tc, 3.5, 4.0)
    assert tc.set_speed[-1] == -1 and tc.change_req and tc.change_req[0] == 1


def test_pilot_keeps_slowing_until_the_last_30_percent_of_the_window():
    # 창 절반이 아니라 마지막 30%까지 속도를 줄여 후행차를 기다린다(원래는 절반에서 그냥 옮겨 후행차가 20 m 뒤였다)
    tc = _FakeTC()
    pilot = _pilot(tc, follower_ready=lambda plan: False)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R71.135", 2, 0.0                      # S3 창 끝까지 약 78 m(창 160 m의 49%)
    _run(pilot, tc, 0.1, 3.5)
    assert tc.change_req == [] and tc.set_speed and tc.set_speed[-1] > 0   # 절반(80 m)은 막 지났지만 아직 기다린다


def test_pilot_gives_up_slowing_after_half_window():
    tc = _FakeTC()
    pilot = _pilot(tc, follower_ready=lambda plan: False)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R71", 2, _net().getEdge("R71").getLength() - 20.0
    _run(pilot, tc, 0.1, 3.5)
    assert tc.change_req == []
    tc.road, tc.pos = "R71.135", 40.0                                 # 창 절반을 지났다(창 끝까지 약 38 m)
    _run(pilot, tc, 3.5, 3.7)
    assert tc.set_speed[-1] == -1 and tc.change_req and pilot.late == [(1, "S3")]


def test_forced_stop_is_released_when_the_bus_enters_a_junction():
    # 강제 정차는 차선을 바꾸려고 서는 것인데 교차로 안에서는 차선을 바꿀 수 없다. 제동 중에 교차로 안으로 밀려 들어간 버스를 세워 두었더니
    # J107 안에서 300초 섰다(quiet 3바퀴째). 교차로 안에서는 정차를 풀어 지나가게 한다
    tc = _FakeTC(speed=6.0)
    pilot = _pilot(tc, follower_ready=lambda plan: False)
    pilot.tick(tc, 0.0)
    L = _net().getEdge("R111").getLength()
    tc.road, tc.lane, tc.pos = "R111", 1, L - 3.0
    _run(pilot, tc, 0.1, 1.0)
    assert pilot.forced == [(1, "S2")] and tc.set_speed[-1] == 0.0       # 강제 정차가 걸렸다
    tc.prev_road, tc.road, tc.lane, tc.pos = "R111", ":J107_2", 1, 1.0
    _run(pilot, tc, 1.0, 2.0)
    assert tc.set_speed[-1] == -1 and pilot._forced_p is None             # 교차로 안: 풀었다
