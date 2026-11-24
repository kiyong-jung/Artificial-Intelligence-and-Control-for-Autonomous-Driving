"""시험용 버스 조종(traffic/test_bus.py): R68 출발, 끊김 없는 바퀴, 정해진 차선 변경 창."""
import json
import os
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import read_net  # noqa: E402
from traffic.scenarios import NET_DIR, _write  # noqa: E402
from traffic.sumo_conn import checkBinary, sumo_cmd, traci  # noqa: E402
from traffic.test_bus import BUS, LC_MANUAL, STOP_AT, TestBusPilot, lane_plans, lap_route  # noqa: E402

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

    def getStops(self, v):
        return [SimpleNamespace(stoppingPlaceID=sid) for sid, _ in self.tc.bus_stops]

    def replaceStop(self, v, i, edge):
        assert edge == ""
        self.tc.skipped.append(self.tc.bus_stops.pop(i)[0])

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
        self.skipped = []                # 건너뛴 정류장
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


def _drive(pilot, tc, edges, t, dt=0.1):
    """버스를 tc.speed로 edges를 따라 끝까지 몬다(차로는 그대로). 끝난 시각."""
    for e in edges:
        tc.road, tc.pos = e, 0.0
        L = _net().getEdge(e).getLength()
        while tc.pos < L:
            pilot.tick(tc, t)
            tc.pos, t = tc.pos + tc.speed * dt, round(t + dt, 6)
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
    # J82는 3.21.0부터 안쪽 R111b 차로 3에서만 좌회전한다(R112 차로 2로)
    assert (p[1].edge, p[1].target_lane) == ("R111b", 3) and p[1].end_pos == pytest.approx(7.0, abs=0.5)
    # J1은 3.7.0부터 R42 차로 3도 좌회전(R68 차로 1)이다. 버스는 더 가까운 차로 3을 목표로 한다
    assert (p[2].edge, p[2].target_lane) == ("R42", 3) and p[2].end_pos == pytest.approx(6.0, abs=0.5)
    assert all(x.length >= 120.0 for x in p) and p[2].length == p[1].length > 120.0     # 두 좌회전 모두 두 칸을 옮긴다


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
    assert tc.lc_mode[-1] == LC_MANUAL


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


def test_bus_hands_over_to_sumo_near_window_end_instead_of_stopping():
    # 창 끝이 제동 거리 안인데 목표 차로에 못 갔으면 달리는 차로에 멈춰 서지 않는다(예전에는 속도 0으로 서서 300초까지 기다렸다).
    # SUMO 기본 차선 변경(1621)에 넘겨 SUMO가 빈틈을 찾고 이웃 차가 협조하게 한다. 창 끝 도로를 지나야 LC_MANUAL(1536)로 돌린다
    tc = _FakeTC(speed=10.0)
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R71.135", 2, 45.0          # S3 창 끝(R71.206 3.3 m)까지 약 22 m, 목표 차로 1
    tc.followers = [("x", 1.0)]                              # 목표 차로에 차가 바로 옆에 있다(차선 변경 불가)
    _run(pilot, tc, 0.1, 1.0)
    assert 0.0 not in tc.set_speed and tc.lc_mode[-1] == 1621 and pilot.handed == [(1, "S3")]
    tc.road, tc.pos = "R71.206", 1.0                         # 창 끝 도로: 아직 SUMO가 몬다
    _run(pilot, tc, 1.0, 1.5)
    assert tc.lc_mode[-1] == 1621 and tc.change_req == []
    tc.lane, tc.road, tc.pos = 1, "R71.256", 1.0             # 창 끝 도로를 지났다
    _run(pilot, tc, 1.5, 2.0)
    assert tc.lc_mode[-1] == LC_MANUAL


@pytest.mark.parametrize("lane, hands_over", [(3, True), (4, False), (5, False)])
def test_handover_compares_lanes_through_the_connections_not_by_number(lane, hands_over):
    # R54 차로 4는 R42 차로 3(좌회전 차로)으로, 차로 5는 R42 차로 4로 이어진다. 차로 3은 R42 차로 2(직진)로 간다. 창 끝 도로(R42)의
    # 목표 차로 번호(3)와 지금 차로 번호를 그대로 비교하면 이미 맞는 차로(4, 5)의 버스를 넘기고 틀린 차로(3)는 그냥 둔다
    L = _net().getEdge("R54").getLength()
    tc = _FakeTC(speed=12.0)
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R54", lane, L - 20.0         # 창 끝(R42 6 m)까지 26 m: 제동 거리(32.8 m) 안
    _run(pilot, tc, 0.1, 2.0)
    assert (pilot.handed == [(1, "S2")]) == hands_over and (1621 in tc.lc_mode) == hands_over
    assert 0.0 not in tc.set_speed


def test_no_handover_when_far_from_window_end_or_already_in_target_lane():
    tc = _FakeTC(speed=10.0)
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.road, tc.lane, tc.pos = "R71", 2, 20.0                # 창 끝까지 100 m 넘게 남았다
    _run(pilot, tc, 0.1, 1.0)
    tc.road, tc.lane, tc.pos = "R71.135", 1, 45.0            # 이미 목표 차로
    _run(pilot, tc, 1.0, 2.0)
    assert pilot.handed == [] and 1621 not in tc.lc_mode


def test_unreachable_curb_stop_is_skipped_instead_of_blocking_the_lane():
    # 창 끝(정류장 10 m 전)까지 정류장 차로에 못 갔으면 그 정류장을 건너뛴다. 달리는 차로에 서서 정류장 차로를 기다리지 않는다
    tc = _FakeTC(speed=10.0)
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.lane, tc.followers = 2, [("x", 1.0)]                  # 목표 차로(1)가 내내 막혀 있다(FakeTC는 차선 변경 요청을 따르지 않는다)
    _drive(pilot, tc, ["R71", "R71.135", "R71.206", "R71.256"], 0.1)
    assert tc.skipped == [STOP_AT[0]] and pilot.missed == [(1, "S3", "R71.206")]
    assert [s for s, _ in tc.bus_stops] == list(STOP_AT[1:])  # 다른 정류장은 그대로
    assert 0.0 not in tc.set_speed and tc.change_req           # 서지 않았고, 그 전에는 차선 변경을 요청했다


def test_reached_curb_stop_is_kept():
    tc = _FakeTC(speed=10.0)
    pilot = _pilot(tc)
    pilot.tick(tc, 0.0)
    tc.lane = 1
    _drive(pilot, tc, ["R71", "R71.135", "R71.206"], 0.1)
    assert tc.skipped == [] and pilot.missed == [] and pilot.handed == []


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


def _sidewalk_left_turn_net():
    """좌회전해 나가는 도로의 가장 오른쪽 차로가 인도인 교차로(합정 R111b -> J82 -> R112와 같은 모양)."""
    d = os.path.join(NET_DIR, "offcenter")
    _write(os.path.join(d, "n.nod.xml"), '<nodes><node id="W" x="-200" y="0"/><node id="C" x="0" y="0" type="priority"/>'
                                         '<node id="N" x="0" y="200"/></nodes>')
    lanes = ('<lane index="0" allow="pedestrian" width="3"/><lane index="1" disallow="pedestrian"/>'
             '<lane index="2" disallow="pedestrian"/>')
    _write(os.path.join(d, "e.edg.xml"), f'<edges><edge id="in" from="W" to="C" numLanes="3" speed="13.9">{lanes}</edge>'
                                         f'<edge id="out" from="C" to="N" numLanes="3" speed="13.9">{lanes}</edge></edges>')
    _write(os.path.join(d, "c.con.xml"), '<connections><connection from="in" to="out" fromLane="2" toLane="1"/></connections>')
    subprocess.run([checkBinary("netconvert"), "-n", os.path.join(d, "n.nod.xml"), "-e", os.path.join(d, "e.edg.xml"),
                    "-x", os.path.join(d, "c.con.xml"), "-o", os.path.join(d, "net.net.xml"), "--no-warnings"],
                   check=True, capture_output=True)
    _write(os.path.join(d, "r.rou.xml"), '<routes><vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" '
                                         'decel="4.0" sigma="0"/><route id="r" edges="in out"/></routes>')
    _write(os.path.join(d, "c.sumocfg"), '<configuration><input><net-file value="net.net.xml"/><route-files value="r.rou.xml"/>'
                                         '</input><processing><lateral-resolution value="0.8"/></processing></configuration>')
    return os.path.join(d, "c.sumocfg")


def test_bus_left_off_center_by_a_lane_change_still_leaves_the_junction():
    # busy seed 3: J82 앞 S2 차선 변경이 도로 경계에서 끊겨 버스가 차로 가운데에서 오른쪽으로 0.97 m 밀린 채 남았다. 나가는 도로
    # R112의 오른쪽 차로는 인도라 SUMO가 교차로 출구에서 버스를 세웠고(MSVehicle::brakeForOverlap) 교차로가 막혔다
    cmd = sumo_cmd(_sidewalk_left_turn_net())
    traci.load(cmd[1:]) if traci.isLoaded() else traci.start(cmd)
    traci.vehicle.add(BUS, "r", typeID="bus_test", departLane="2", departPos="20", departSpeed="10")
    traci.vehicle.setLaneChangeMode(BUS, LC_MANUAL)
    traci.simulationStep()
    traci.vehicle.changeSublane(BUS, -0.97)
    for _ in range(400):
        traci.simulationStep()
        if traci.vehicle.getRoadID(BUS) == "out":
            break
    assert traci.vehicle.getRoadID(BUS) == "out", (traci.vehicle.getLaneID(BUS), traci.vehicle.getLateralLanePosition(BUS))
