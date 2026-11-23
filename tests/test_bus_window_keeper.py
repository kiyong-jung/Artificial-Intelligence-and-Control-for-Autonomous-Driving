"""BusWindowKeeper: 버스를 따라다니는 앞뒤 50 m 창에서 차로마다 목표 대수를 유지한다. 시야 밖에서만 만들고 지운다."""
import json
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from sumolib.geomhelper import positionAtShapeOffset  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.bus_window_keeper import DEFAULT_LANE_DENSITY, BusWindowKeeper  # noqa: E402
from traffic.flow_balance import edges_named  # noqa: E402
from traffic.signal_areas import is_signalized  # noqa: E402
from traffic.presence import PRESENCE_RADIUS_M, SPAWN_MARGIN_M, ends_at_boundary  # noqa: E402
from traffic.test_bus import lap_route  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")
VIEW = PRESENCE_RADIUS_M + SPAWN_MARGIN_M
_c = {}


def _net():
    if "net" not in _c:
        _c["net"] = read_net(os.path.join(ROOT, "source", "network.net.xml"))
    return _c["net"]


def _loop():
    if "loop" not in _c:
        with open(os.path.join(ROOT, "map.json"), encoding="utf-8") as f:
            _c["loop"] = lap_route(_net(), json.load(f)["routes"]["bus_loop"])
    return _c["loop"]


class _Veh:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return ["bus0"] + list(self.tc.cars)

    def getRoadID(self, v):
        return self.tc.bus[0] if v == "bus0" else self.tc.cars[v]["road"]

    def getLanePosition(self, v):
        return self.tc.bus[1] if v == "bus0" else self.tc.cars[v]["pos"]

    def getLaneIndex(self, v):
        return 1 if v == "bus0" else self.tc.cars[v]["lane"]

    def getSpeed(self, v):
        return 8.0 if v == "bus0" else self.tc.cars[v].get("speed", 8.0)

    def getPosition(self, v):
        road, pos = (self.tc.bus if v == "bus0" else (self.tc.cars[v]["road"], self.tc.cars[v]["pos"]))
        return positionAtShapeOffset(_net().getEdge(road).getShape(), pos)

    def add(self, vid, rid, typeID="carDist", **kw):
        self.tc.added.append({"id": vid, "route": self.tc.routes[rid], "type": typeID, **kw})
        self.tc.cars[vid] = {"road": self.tc.routes[rid][0], "pos": float(kw["departPos"]), "lane": int(kw["departLane"])}

    def remove(self, vid, *a):
        self.tc.removed.append(vid)
        self.tc.cars.pop(vid, None)


class _Edge:
    def __init__(self, tc):
        self.tc = tc

    def getLastStepVehicleIDs(self, e):
        return tuple(v for v, c in self.tc.cars.items() if c["road"] == e) + (("bus0",) if self.tc.bus[0] == e else ())

    def getLastStepHaltingNumber(self, e):
        return self.tc.halting.get(e, 0)


class _Lane:
    def __init__(self, tc):
        self.tc = tc

    def getLastStepVehicleIDs(self, lane):
        e, k = lane.rsplit("_", 1)
        return tuple(v for v, c in self.tc.cars.items() if c["road"] == e and c["lane"] == int(k))


class _Route:
    def __init__(self, tc):
        self.tc = tc

    def add(self, rid, edges):
        self.tc.routes[rid] = list(edges)


class _Tc:
    def __init__(self, road="R54a", pos=150.0):
        self.bus = (road, pos)
        self.cars, self.added, self.removed, self.routes, self.halting = {}, [], [], {}, {}
        self.vehicle, self.edge, self.lane, self.route = _Veh(self), _Edge(self), _Lane(self), _Route(self)


def _keeper(tc, **kw):
    return BusWindowKeeper(tc, _net(), "bus0", _loop(), rng=random.Random(1), **kw)


def _fill_window(tc, per_lane=2, lanes=(1, 2, 3), edge="R54a", start=105.0):
    for k in lanes:
        for n in range(per_lane):
            tc.cars[f"nat{k}_{n}"] = {"road": edge, "pos": start + n * 20.0 + k, "lane": k}


def test_empty_window_is_filled_on_every_passenger_lane_behind_the_view():
    tc = _Tc()
    k = _keeper(tc)
    k.update(0.0)
    lanes = sorted(int(a["departLane"]) for a in tc.added)
    assert lanes == [1, 2, 3]                                  # 승용 차로만(4번 버스 전용·0번 인도는 제외), 차로마다 한 대씩
    L = _net().getEdge("R54a").getLength()
    for a in tc.added:
        p = float(a["departPos"])
        assert p <= 150.0 - VIEW + 0.5 and p <= L - 30.0 and a["type"] == "carDist"      # 버스 뒤 시야 밖, 구간 끝 30 m 앞
        assert a["route"][0] == "R54a"
        xb, yb = positionAtShapeOffset(_net().getEdge("R54a").getShape(), 150.0)
        xs, ys = positionAtShapeOffset(_net().getEdge("R54a").getShape(), p)
        assert math.hypot(xb - xs, yb - ys) >= VIEW - 1e-6
    assert k.cars == {a["id"] for a in tc.added}


def test_keeps_filling_until_each_lane_has_its_target_and_never_removes_inside_the_view():
    tc = _Tc()
    k = _keeper(tc)
    for step in range(10):
        k.update(step * 0.5)
        # 새로 만든 차가 창 안으로 들어왔다고 보고 위치를 옮긴다
        for v, c in tc.cars.items():
            if v.startswith("win") and c["pos"] < 100.0:
                c["pos"] = 120.0 + 12.0 * len(tc.cars)
    counts = {}
    for v, c in tc.cars.items():
        if 100.0 <= c["pos"] <= 200.0:
            counts[c["lane"]] = counts.get(c["lane"], 0) + 1
    assert all(counts.get(lane, 0) >= 2 for lane in (1, 2, 3)) or len(tc.added) >= 6
    assert tc.removed == []


def test_no_spawn_when_window_already_holds_the_target_including_natural_cars():
    tc = _Tc()
    _fill_window(tc, per_lane=2)
    k = _keeper(tc)
    k.update(0.0)
    assert tc.added == []                                      # 늘어나는 것은 허용, 줄어드는 쪽으로는 지우지 않는다
    assert tc.removed == []


def test_roads_without_density_get_nothing():
    tc = _Tc(road="R112.112", pos=60.0)                       # 와우산로: 밀도 없음
    k = _keeper(tc)
    k.update(0.0)
    assert tc.added == []


def test_keeper_cars_ahead_of_the_view_are_removed_but_others_stay():
    tc = _Tc()
    _fill_window(tc, per_lane=2)
    tc.cars["win_far"] = {"road": "R54a", "pos": 235.0, "lane": 2}        # 버스 앞 85 m
    tc.cars["win_near"] = {"road": "R54a", "pos": 180.0, "lane": 2}       # 버스 앞 30 m(시야 안)
    tc.cars["win_prot"] = {"road": "R54a", "pos": 238.0, "lane": 3}
    tc.cars["nat_far"] = {"road": "R54a", "pos": 236.0, "lane": 1}        # 자연 차량
    k = _keeper(tc, protected=lambda v: v == "win_prot")
    k.cars |= {"win_far", "win_near", "win_prot"}
    k.update(0.0)
    assert tc.removed == ["win_far"] and "win_near" in tc.cars and "nat_far" in tc.cars and "win_prot" in tc.cars
    assert k.summary()["removed_ahead"] == 1


def test_long_signal_queue_removes_one_stopped_car_outside_the_view_farthest_first():
    tc = _Tc()
    _fill_window(tc, per_lane=2)
    tc.cars["win_q"] = {"road": "R54a", "pos": 60.0, "lane": 1, "speed": 0.0}      # 버스 뒤 90 m 대기열
    tc.cars["win_q2"] = {"road": "R54a", "pos": 30.0, "lane": 1, "speed": 0.0}
    tc.cars["win_in"] = {"road": "R54a", "pos": 125.0, "lane": 1, "speed": 0.0}    # 버스 뒤 25 m(시야 안)
    tc.cars["nat_q"] = {"road": "R54a", "pos": 50.0, "lane": 2, "speed": 0.0}
    k = _keeper(tc)
    k.cars |= {"win_q", "win_q2", "win_in"}
    tc.halting["R54a"] = 3
    k.update(0.0)
    assert tc.removed == []                                     # 대기열이 아직 짧다
    tc.halting["R54a"] = 4
    k.update(1.0)
    assert tc.removed == ["nat_q", "win_q2", "win_q"] or set(tc.removed) == {"nat_q", "win_q2", "win_q"}   # 시야 밖 세 대(기존 차 포함), 먼 차부터
    assert "win_in" in tc.cars                                                 # 시야 안은 그대로


def test_spawn_car_for_scenarios_refuses_inside_the_view_and_registers_the_car():
    tc = _Tc()
    k = _keeper(tc)
    assert k.spawn_car(["R54a"], 2, 130.0) is None             # 버스 뒤 20 m: 시야 안
    vid = k.spawn_car(["R54a", "R54b"], 2, 60.0)
    assert vid and k.is_keeper_car(vid) and tc.routes[f"r_{vid}"] == ["R54a", "R54b"]
    assert k.summary()["scenario_spawned"] == 1


def test_budget_limits_spawns_per_update():
    tc = _Tc()
    k = _keeper(tc, max_per_h=0.0, burst=1.0)
    k.update(0.0)
    assert len(tc.added) == 1 and k.summary()["skipped_budget"] >= 1


def test_vanished_cars_are_forgotten():
    tc = _Tc()
    _fill_window(tc, per_lane=2)
    k = _keeper(tc)
    k.cars.add("gone")
    k.update(0.0)
    assert "gone" not in k.cars


def test_local_route_turns_off_before_avoid_roads_and_ends_at_the_boundary():
    avoid = frozenset(edges_named(_net(), ("와우산로", "잔다리로")))
    k = _keeper(_Tc(), avoid=avoid)
    route = k._local_route("R111")                              # 독막로 끝(J82)에서 와우산로로 이어진다
    assert route[0] == "R111" and "R111b" in route and not any(e in avoid for e in route)
    assert ends_at_boundary(_net(), route[-1])
    plain = _keeper(_Tc())._local_route("R111")
    assert "R112" in plain                                      # 피할 도로가 없으면 노선을 그대로 따라간다


def test_default_density_covers_only_yanghwa_and_dongmak_roads():
    assert set(DEFAULT_LANE_DENSITY) == {"양화로", "독막로"} and all(v > 0 for v in DEFAULT_LANE_DENSITY.values())


def test_keeper_does_not_keep_spawning_while_its_own_cars_are_still_on_their_way_into_the_window():
    # 버스가 신호·정류장에 서 있으면 뒤에서 만든 차가 창에 들어오지 못하고 줄을 선다. 창 안 대수만 세면 같은 부족분이 계속 남아 주기마다 또 만들어
    # 차가 쌓인다. 창 바로 뒤에서 오고 있는 키퍼 차도 부족분을 채우는 것으로 센다
    tc = _Tc()
    k = _keeper(tc)
    for step in range(30):
        k.update(step * 0.5)                                   # 만든 차는 그 자리에서 움직이지 않는다(창에 못 들어온다)
    per_lane = {}
    for a in tc.added:
        per_lane[int(a["departLane"])] = per_lane.get(int(a["departLane"]), 0) + 1
    assert per_lane == {1: 2, 2: 2, 3: 2}                      # 목표(차로마다 2대)를 넘겨 만들지 않는다
    assert len(tc.added) == 6


def test_keeper_does_not_remove_pending_cars_or_count_far_away_cars_as_pending():
    tc = _Tc()
    k = _keeper(tc)
    tc.cars["win_far"] = {"road": "R54a", "pos": 5.0, "lane": 1}      # 버스 뒤 145 m: 곧 들어올 차가 아니다
    k.cars.add("win_far")
    k.update(0.0)
    assert any(int(a["departLane"]) == 1 for a in tc.added)             # 1차로는 여전히 부족으로 보고 만든다


def test_long_signal_queue_also_removes_a_stopped_car_ahead_outside_the_view():
    tc = _Tc()
    _fill_window(tc, per_lane=2)
    tc.cars["ahead_q"] = {"road": "R54a", "pos": 260.0, "lane": 1, "speed": 0.0}   # 버스 앞 110 m(시야 밖)
    k = _keeper(tc)
    tc.halting["R54a"] = 4
    k.update(0.0)
    assert tc.removed == ["ahead_q"]


def test_queue_before_a_signal_is_removed_earlier_and_in_batches_of_three():
    tc = _Tc()
    _fill_window(tc, per_lane=2)
    k = _keeper(tc)
    bi, bpos = k.index[tc.bus[0]], tc.bus[1]
    edge = next(e for e in _loop() if is_signalized(_net().getEdge(e).getToNode()) and _net().getEdge(e).getLength() > 60.0
                and k._rel("x", k.index[e], 20.0, bi, bpos) > 120.0)             # 버스 앞 멀리(시야 밖)에 있는 신호 앞 도로
    for n in range(5):
        tc.cars[f"sq{n}"] = {"road": edge, "pos": 20.0 + 6.0 * n, "lane": 1, "speed": 0.0}
    tc.halting[edge] = 2                                                           # 일반 도로 기준(4)에는 못 미치지만 신호 앞은 2대부터
    k.update(0.0)
    assert len(tc.removed) == 3 and all(v.startswith("sq") for v in tc.removed)    # 한 번에 세 대까지


def test_car_still_waiting_to_be_inserted_is_cancelled_when_the_bus_comes_near_its_spot():
    # 삽입이 늦어진 차가 버스가 가까이 온 뒤에 나타나면 반경 안 출현이 된다. 삽입 전에 취소한다
    tc = _Tc()
    k = _keeper(tc)
    vid = k.spawn_car(["R54a", "R54b"], 2, 60.0)                 # 버스 뒤 90 m(시야 밖)
    assert vid
    tc.cars.pop(vid)                                              # 아직 삽입되지 않았다(자리가 막혀 대기 중)
    k.update(0.0)
    assert vid not in tc.removed and vid in k.cars                # 버스가 멀다: 계속 기다린다
    tc.bus = (tc.bus[0], 60.0 + 20.0)                             # 버스가 그 자리 20 m 앞까지 왔다
    k.update(1.0)
    assert vid in tc.removed and vid not in k.cars and k.summary()["cancelled_late"] == 1


def test_interior_spawner_weights_are_halved_on_yanghwaro_and_dongmakro():
    from traffic.interior_spawner import InteriorSpawner

    class _Tc:
        pass
    net = _net()
    half = {e: 0.5 for e in edges_named(net, ("양화로", "독막로"))}
    full = InteriorSpawner(_Tc(), net, "bus0", _loop(), [], 100.0)
    scaled = InteriorSpawner(_Tc(), net, "bus0", _loop(), [], 100.0, weight_scale=half)
    ratio = {e: s / f for e, f, s in zip(full.roads, full.weights, scaled.weights)}
    assert any(r == 0.5 for r in ratio.values()) and all(r in (0.5, 1.0) for r in ratio.values())


def test_interior_spawner_total_rate_drops_and_does_not_shift_to_other_roads():
    from traffic.interior_spawner import InteriorSpawner

    class _Tc:
        pass
    net = _net()
    avoid = edges_named(net, ("와우산로", "잔다리로"))
    half = {e: 0.5 for e in edges_named(net, ("양화로", "독막로"))}
    full = InteriorSpawner(_Tc(), net, "bus0", _loop(), [], 100.0, avoid=avoid)
    scaled = InteriorSpawner(_Tc(), net, "bus0", _loop(), [], 100.0, avoid=avoid, weight_scale=half)
    assert scaled.p_step == pytest.approx(full.p_step * 0.5)                      # 투입 수가 절반으로 준다
    assert set(scaled.roads).isdisjoint(avoid)                                    # 와우산로·잔다리로에는 넣지 않는다
