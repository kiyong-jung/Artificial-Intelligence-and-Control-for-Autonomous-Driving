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
        return self.tc.routes[v].index(self.tc.road)

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
    assert (p[2].edge, p[2].target_lane) == ("R42", 4) and p[2].end_pos == pytest.approx(6.0, abs=0.5)
    assert all(x.length >= 120.0 for x in p) and p[2].length > p[1].length     # J1 앞은 차로를 더 많이 옮긴다
    # J1 창은 합정역(s6, R54a 245.5~270.5 m) 안에서 시작한다. 감속 판단은 정류장 끝부터 센다
    net = _net()
    eff = (net.getEdge("R54a").getLength() - 270.5 + net.getEdge("R54b").getLength() + net.getEdge("R54").getLength()
           + p[2].end_pos)
    assert p[2].eff_length == pytest.approx(eff, abs=0.5) and p[2].eff_length < p[2].length
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
    tc = _FakeTC()
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
    assert tc.change_req == [] and tc.set_speed and tc.set_speed[0] == pytest.approx(max(0.6 * 13.89, 3.0))
    assert tc.signals != 0                                            # 방향지시등은 켠 채로 달린다
    state["ready"] = True
    _run(pilot, tc, 3.5, 4.0)
    assert tc.set_speed[-1] == -1 and tc.change_req and tc.change_req[0] == 1


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
