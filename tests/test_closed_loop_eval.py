"""Closed-loop 평가: 정책 버스 명령, 판단(관측 v2·직전 행동·끝 판정), 정류장 지표, 요약. SUMO 없이 가짜 TraCI와 고정 행으로 시험한다."""
import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from pdm_fakes import FakeTC, _Simple, hapjeong_ctx  # noqa: E402
from pdm_fixtures import make_row, make_run_dir  # noqa: E402
from pdm_data.closed_loop import PolicyController, analyze_run, stop_metrics  # noqa: E402
from pdm_data.recorder import CollectionDone  # noqa: E402
from pdm_data.schema import BUS  # noqa: E402
from traffic.policy_bus import (LC_MODE_OFF, SPEED_MODE_OFF, PolicyBusPilot, accel_from_normalized,  # noqa: E402
                                blinker_for, lane_target)

CTX = hapjeong_ctx()
START, END, SID, STOP_LANE = CTX.service_stops[0]
STOP_EDGE, STOP_IDX = STOP_LANE.rsplit("_", 1)
STOP_K = CTX.lap_edges.index(STOP_EDGE)


# ---------------------------------------------------------------- 명령 변환
def test_accel_scale_is_inverse_of_label_normalization():
    assert accel_from_normalized(1.0, 1.5, 3.0) == 1.5
    assert accel_from_normalized(-1.0, 1.5, 3.0) == -3.0
    assert accel_from_normalized(-0.5, 1.5, 3.0) == -1.5
    assert accel_from_normalized(2.0, 1.5, 3.0) == 1.5          # 범위 밖은 자른다


def test_lane_target_and_blinker():
    assert lane_target(2, 1, 3, False) == (2, None)
    assert lane_target(-2, 1, 3, False) == (0, None)
    assert lane_target(2, 2, 3, False) == (None, "no_lane")
    assert lane_target(-2, 0, 3, False) == (None, "no_lane")
    assert lane_target(2, 1, 3, True) == (None, "internal")
    assert lane_target(1, 1, 3, False) == (None, None)          # 방향지시등만
    assert (blinker_for(2), blinker_for(1), blinker_for(-1), blinker_for(0)) == (2, 2, 1, -1)


def _pilot(bus=None, **kw):
    from maps import load_map
    from run_hapjeong_scenario import map_stops
    from traffic.alleys import read_net
    m = load_map("hapjeong")
    tc = FakeTC({})
    tc.route = _Simple(tc, "route", {})
    tc.vehicle.add = lambda *a, **k: tc.calls.append(("vehicle.add", a))     # 가짜 도메인은 키워드 인자를 받지 않는다
    p = PolicyBusPilot(tc, read_net(m["net_path"]), m["routes"]["bus_loop"], map_stops(m), **kw)
    p.tick(tc, 0.0)                                              # 첫 tick: 버스를 넣는다
    tc.vehicle.items[BUS] = dict(bus or {"road": "R69", "lane": "R69_1", "lane_index": 1, "speed": 5.0})
    return p, tc


def _calls(tc, name):
    return [args for n, args in tc.calls if n == f"vehicle.{name}"]


def test_policy_bus_turns_off_safety_and_bus_stops():
    p, tc = _pilot()
    assert _calls(tc, "setSpeedMode") == [(BUS, SPEED_MODE_OFF)]
    assert (BUS, LC_MODE_OFF) in _calls(tc, "setLaneChangeMode")
    assert _calls(tc, "setBusStop") == [] and _calls(tc, "setStop") == []
    p.tick(tc, 0.1)                                              # 첫 판단 전: 정지 유지
    assert _calls(tc, "setSpeed")[-1] == (BUS, 0.0)
    p._extend()
    assert _calls(tc, "setBusStop") == []                       # 노선을 이어 붙여도 정차 명령은 없다


def test_policy_bus_applies_accel_and_lane_request_once():
    p, tc = _pilot()
    p.decide(-3.0, 2, t=10.0)
    assert (p.phase, p.target) == ("request", 2)
    tc.calls.clear()
    p.tick(tc, 10.1)
    p.tick(tc, 10.2)
    assert _calls(tc, "setSpeed") == [(BUS, pytest.approx(4.7))] * 2       # 5.0 - 3.0 * 0.1 (가짜 속도는 그대로)
    assert _calls(tc, "changeLane") == [(BUS, 2, 0.5)]                     # 요청은 판단마다 한 번
    assert _calls(tc, "setSignals") == [(BUS, 2)] * 2
    assert p.last_plan["lane_cmd"] == 2 and p.last_plan["policy"] is True


def test_policy_bus_rejects_impossible_lane_change_and_never_goes_backward():
    p, tc = _pilot({"road": "R69", "lane": "R69_2", "lane_index": 2, "speed": 0.1})
    p.decide(-3.0, 2, t=1.0)                                    # 가짜 도로는 3차로: 2번에서 왼쪽은 없다
    tc.calls.clear()
    p.tick(tc, 1.1)
    assert _calls(tc, "changeLane") == []
    assert p.policy_stats["lane_request_rejected"]["no_lane"] == 1
    assert _calls(tc, "setSpeed") == [(BUS, 0.0)]               # 0.1 - 0.3 -> 0에서 멈춘다(후진 없음)


# ---------------------------------------------------------------- 3값 차로 명령(관측 33차원 정책)
def _lc3(bus=None):
    p, tc = _pilot(bus, lane_values=3)
    tc.calls.clear()
    return p, tc


def test_lc3_single_request_and_blinker():
    p, tc = _lc3()
    p.decide(0.0, 1, t=1.0)
    p.tick(tc, 1.1)
    assert _calls(tc, "changeLane") == [(BUS, 2, 0.5)]
    assert _calls(tc, "setSignals")[-1] == (BUS, 2)                        # 왼쪽 방향지시등
    p.decide(0.0, 0, t=1.5)
    p.tick(tc, 1.6)
    assert _calls(tc, "setSignals")[-1] == (BUS, -1) and len(_calls(tc, "changeLane")) == 1


def test_lc3_resend_before_flip():
    p, tc = _lc3()
    tc.vehicle.items[BUS]["lat_speed"] = 0.8
    p.decide(0.0, 1, t=1.0)
    p.tick(tc, 1.1)
    p.decide(0.0, 1, t=1.5)                                                # 아직 차로 1: 같은 목표로 다시
    p.tick(tc, 1.6)
    assert _calls(tc, "changeLane") == [(BUS, 2, 0.5), (BUS, 2, 0.5)]


def test_lc3_guard_after_flip():
    p, tc = _lc3()
    p.decide(0.0, 1, t=1.0)
    p.tick(tc, 1.1)
    tc.vehicle.items[BUS].update(lane="R69_2", lane_index=2, lat_speed=0.5)   # 번호가 바뀌었고 아직 옆으로 움직인다
    p.decide(0.0, 1, t=1.5)
    p.tick(tc, 1.6)
    assert _calls(tc, "changeLane") == [(BUS, 2, 0.5)] and p.policy_stats["lane_guard_ignored"] == 1
    tc.vehicle.items[BUS]["lat_speed"] = 0.0                               # 다 옮겼다: 새 요청
    p.decide(0.0, 1, t=2.0)
    p.tick(tc, 2.1)
    assert _calls(tc, "changeLane")[-1] == (BUS, 3, 0.5) or p.policy_stats["lane_request_rejected"]["no_lane"] == 1


def test_lc3_opposite_releases_latch():
    p, tc = _lc3()
    p.decide(0.0, 1, t=1.0)
    p.tick(tc, 1.1)
    tc.vehicle.items[BUS].update(lane="R69_2", lane_index=2, lat_speed=0.5)
    p.decide(0.0, -1, t=1.5)
    p.tick(tc, 1.6)
    assert _calls(tc, "changeLane")[-1] == (BUS, 1, 0.5)


def test_lc3_latch_rejects_internal_lane():
    p, tc = _lc3()
    tc.vehicle.items[BUS]["lat_speed"] = 0.8
    p.decide(0.0, 1, t=1.0)
    p.tick(tc, 1.1)
    tc.vehicle.items[BUS].update(road=":J107_2", lane=":J107_2_0", lane_index=0)   # 아직 옮기는 중에 교차로 안
    p.decide(0.0, 1, t=1.5)
    p.tick(tc, 1.6)
    assert _calls(tc, "changeLane") == [(BUS, 2, 0.5)]
    assert p.policy_stats["lane_request_rejected"]["internal"] == 1


def test_lc3_latch_road_change_does_not_resend_old_index():
    p, tc = _lc3()
    tc.vehicle.items[BUS]["lat_speed"] = 0.8
    p.decide(0.0, 1, t=1.0)
    p.tick(tc, 1.1)
    tc.vehicle.items[BUS].update(road="R69a", lane="R69a_1", lane_index=1)        # 새 도로: 차로 번호가 옛 도로와 다르다
    p.decide(0.0, 1, t=1.5)
    p.tick(tc, 1.6)
    assert _calls(tc, "changeLane") == [(BUS, 2, 0.5)] and p.policy_stats["lane_guard_ignored"] == 1
    tc.vehicle.items[BUS]["lat_speed"] = 0.0                                       # 다 옮겼다: 새 도로 기준 새 요청
    p.decide(0.0, 1, t=2.0)
    p.tick(tc, 2.1)
    assert _calls(tc, "changeLane")[-1] == (BUS, 2, 0.5) and len(_calls(tc, "changeLane")) == 2


def test_stop_plans_only_drops_s2_s3():
    p, _ = _pilot(stop_plans_only=True)
    assert {q.kind for q in p.plans} == {"STOP"} and {q.kind for q in p.lane_plans_all} > {"STOP"}


def test_controller_obs33_requires_sumo_lc_world():
    with pytest.raises(ValueError):
        PolicyController(lambda o: (0.0, 0), None, {"input": "obs33", "lane_classes": 3})
    with pytest.raises(ValueError):
        PolicyController(lambda o: (0.0, 0), None, {"input": "obs33", "lane_classes": 3}, world="hapjeong")


def test_controller_obs33_path():
    seen = []

    def act(obs):
        seen.append(np.array(obs))
        return (0.0, 1)
    c = PolicyController(act, {"pos": 1.5, "neg": 3.0}, {"input": "obs33", "lane_classes": 3}, world="sumo_lc")
    pilot = StubPilot()
    c.bind(CTX, pilot)
    c.on_decision(make_row(t=0, speed=5.0), None)
    assert seen[0].shape == (33,) and pilot.decisions[0][1] == 1
    assert "signal_state_input" in pilot.decisions[0][3]


# ---------------------------------------------------------------- 판단
class StubPilot:
    plans = []

    def __init__(self):
        self.decisions = []

    def decide(self, a, lane, t, info=None):
        self.decisions.append((a, lane, t, info))


def _controller(action=(0.5, 0)):
    seen = []

    def act(obs):
        seen.append(np.array(obs))
        return action
    c = PolicyController(act, {"pos": 1.5, "neg": 3.0})
    pilot = StubPilot()
    c.bind(CTX, pilot)
    return c, pilot, seen


def _stop_row(t, speed=0.0, **kw):
    pos = START + 10.0
    return make_row(t=t, speed=speed, edge=STOP_EDGE, lane_index=int(STOP_IDX), lap_position=pos,
                    lap_route_index=STOP_K, lane_pos=pos - CTX.route_map.start[STOP_K], window_speeds=[speed] * 5,
                    **kw)


def test_controller_maps_policy_output_to_mps2_and_feeds_realized_prev_action():
    c, pilot, seen = _controller((-0.5, 2))
    r0 = make_row(t=0, speed=5.0, window_speeds=[5.1, 5.2, 5.3, 5.4, 5.5])   # 0.5초에 +0.5 m/s -> +1.0 m/s^2
    c.on_decision(r0, None)
    assert pilot.decisions[0][:2] == (-1.5, 2)
    assert seen[0][2] == 0.0 and seen[0][3] == 0.0                         # 첫 판단: 직전 행동 없음
    c.on_decision(make_row(t=1, speed=5.5), r0)
    assert seen[1][2] == pytest.approx(1.0 / 1.5)                          # 실제로 일어난 가감속(정규화)
    assert seen[1][3] == 0.0                                               # 실제 차로 변화 없음, 방향지시등 없음


def test_dwell_input_counts_down_from_8s_to_0_while_stopped():
    c, pilot, seen = _controller((0.0, 0))
    prev = None
    for t in range(18):
        row = _stop_row(t)
        c.on_decision(row, prev)
        prev = row
    dwell = [d[3]["dwell_remaining_input"] for d in pilot.decisions]
    assert dwell[0] == pytest.approx(8.0)                    # 멈춰 있는 첫 판단: 8초
    assert dwell[1] == pytest.approx(7.5)
    assert dwell[15] == pytest.approx(0.5)
    assert dwell[16] == 0.0 and dwell[17] == 0.0             # 8초를 채우면 0초(다음 정류장 차례)
    assert pilot.decisions[17][3]["tracker_index"] == 1


@pytest.mark.parametrize("kind", ["collision", "red_light", "missed_stop"])
def test_controller_ends_episode_like_episode_end(kind):
    c, _, _ = _controller()
    r0 = make_row(t=0)
    c.on_decision(r0, None)
    if kind == "collision":
        r0["window"]["collisions"].append({"collider": BUS, "victim": "car1", "type": "collision", "ego": True})
    elif kind == "red_light":
        r0["tls"] = [{"id": "J1", "link_index": 0, "distance": 1.0, "state": "r"}]
        for s in r0["window"]["substeps"]:
            s["edge"], s["distance"], s["tls0"] = "R70", 30.0, None
    else:
        for s in r0["window"]["substeps"]:
            s["lap_position"] = END + 5.0
    with pytest.raises(CollectionDone) as e:
        c.on_decision(make_row(t=1), r0)
    assert e.value.reason == kind


# ---------------------------------------------------------------- 정류장 지표
def _approach_rows(stop_rows, depart_after_rows):
    rows = [_stop_row(t) for t in range(stop_rows)]
    for k in range(depart_after_rows):
        rows.append(_stop_row(stop_rows + k))
    rows.append(_stop_row(stop_rows + depart_after_rows, speed=2.0))
    return rows


def test_stop_metrics_service_then_departure_delay():
    rows = _approach_rows(16, 2)                 # 8초 정차, 1초 더 섰다가 출발
    r = stop_metrics(rows, CTX)[0]
    assert r["reached"] and r["stopped_in_area"] and r["service_completed"]
    assert r["interruptions"] == 0
    assert r["departure_delay_s"] == pytest.approx(1.1, abs=0.11)


def test_stop_metrics_counts_moving_during_countdown():
    rows = [_stop_row(t) for t in range(6)] + [_stop_row(6, speed=1.0)] + [_stop_row(7 + t) for t in range(17)]
    r = stop_metrics(rows, CTX)[0]
    assert r["interruptions"] == 1 and r["service_completed"]
    assert r["max_partial_dwell_s"] == pytest.approx(8.0)


def test_stop_metrics_never_departed():
    r = stop_metrics([_stop_row(t) for t in range(20)], CTX)[0]
    assert r["service_completed"] and r["t_depart"] is None and r["departure_delay_s"] is None


# ---------------------------------------------------------------- 지표 전체와 요약
def test_analyze_run_on_fixture(tmp_path):
    m = analyze_run(make_run_dir(tmp_path))
    assert m["end_reason"] == "incomplete_service" and not m["success"]
    assert m["decisions"] == 11 and m["collisions"] == 0
    assert m["mean_speed_mps"] > 0.0 and m["episode_time_s"] == pytest.approx(5.5)
    assert m["stops"] and m["stops"][0]["stop"] == SID


def test_summarize_writes_tables(tmp_path):
    import eval_bc_closed_loop as ev
    base = {"collisions": 0, "progress_ratio": 1.0, "stops_completed": 4, "stops": []}
    for agent, seed, end in [("base/seed1", 1000, "arrived"), ("base/seed1", 1001, "collision"),
                             ("base/seed2", 1000, "arrived"), ("base/seed2", 1001, "arrived"),
                             ("expert", 1000, "arrived")]:
        d = tmp_path / agent / f"seed{seed}"
        d.mkdir(parents=True)
        (d / "result.json").write_text(json.dumps({"agent": agent, "seed": seed, "metrics": {
            **base, "end_reason": end, "success": end == "arrived"}}))
    ev.summarize(str(tmp_path))
    text = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "| 완주율 | 높게 | 1.000 | 0.750 ± 0.354 |" in text
    assert (tmp_path / "episodes.csv").exists() and (tmp_path / "agents.csv").exists()


# ---------------------------------------------------------------- 추가 지표
from pdm_data.closed_loop import ExpertWatch  # noqa: E402
from pdm_data.closed_loop_extra import (StallWatch, _box, background_collisions, boxes_overlap,  # noqa: E402
                                        interaction_counts, longest_stall, outcome, overlap_metrics)


def _subs(points):
    return [{"t": t, "distance": d} for t, d in points]


def test_longest_stall_finds_slow_interval():
    pts = [(0.1 * k, 1.0 * k) for k in range(50)]                       # 10 m/s로 5초
    pts += [(5.0 + 0.1 * k, 49.0 + 0.01 * k) for k in range(1, 300)]   # 30초 동안 3 m
    pts += [(35.0 + 0.1 * k, 52.0 + 1.0 * k) for k in range(1, 20)]
    s = longest_stall(_subs(pts), progress_m=5.0)
    assert s["duration_s"] == pytest.approx(30.0, abs=0.6)
    assert 4.5 <= s["t_start"] <= 5.1


def test_outcome_reclassifies_long_stall_time_limit_only():
    assert outcome("arrived", True, 500.0) == "arrived"
    assert outcome("time_limit", False, 200.0, 180.0) == "stuck"
    assert outcome("time_limit", False, 100.0, 180.0) == "time_limit"
    assert outcome("collision", False, 500.0, 180.0) == "collision"
    assert outcome("time_limit", False, 500.0, 0.0) == "time_limit"            # 0이면 끈다


def _still_row(t, distance=0.0):
    return make_row(t=t, speed=0.0, distance=distance, window_speeds=[0.0] * 5)


def test_stall_watch_fires_after_seconds_without_progress():
    w = StallWatch(10.0, progress_m=5.0)
    fired = [w.feed(_still_row(t)) for t in range(25)]                 # 행마다 0.5초
    assert not any(fired[:19]) and fired[20]
    w2 = StallWatch(0.0)
    assert not any(w2.feed(_still_row(t)) for t in range(100))


def test_policy_and_expert_end_with_stuck():
    c, _, _ = _controller((0.0, 0))
    c.stall = StallWatch(5.0)
    prev = None
    with pytest.raises(CollectionDone) as e:
        for t in range(30):
            row = _still_row(t)
            c.on_decision(row, prev)
            prev = row
    assert e.value.reason == "stuck"
    x = ExpertWatch(5.0)
    prev = None
    with pytest.raises(CollectionDone) as e:
        for t in range(30):
            row = _still_row(t)
            x.on_decision(row, prev)
            prev = row
    assert e.value.reason == "stuck"


def _nb(**slots):
    out = {k: None for k in ("leader", "follower", "left_leader", "right_leader", "left_follower", "right_follower")}
    out.update({k: {"id": f"v_{k}", "gap_raw": g} for k, g in slots.items()})
    return out


def test_interaction_counts_lane_changes_cutin_and_crossing():
    at, cid = CTX.route_map.crossings[0]
    rows = [
        make_row(t=0, lap_position=at - 80, neighbors=_nb(leader=40.0)),
        # 왼쪽 요청 시작(옮길 차로 뒤 20 m에 차) + 앞차 간격 40 -> 25 m
        make_row(t=1, lap_position=at - 70, window_phases=["request"] * 5, window_dirs=[1] * 5,
                 neighbors=_nb(leader=25.0, left_follower=20.0)),
        make_row(t=2, lap_position=at - 60, window_phases=["request"] * 5, window_dirs=[1] * 5),   # 같은 요청 계속
        # 실제로 왼쪽으로 옮김, 옮길 차로 앞 10 m에 차, 신호 앞 30 m
        make_row(t=3, lap_position=at - 40, lane_before=1, lane_after=2, neighbors=_nb(left_leader=10.0),
                 tls=[{"id": "J1", "link_index": 0, "distance": 30.0, "state": "r"}]),
        make_row(t=4, lap_position=at - 10, lane_index=2,
                 pedestrians=[{"id": "p1", "x": 0.0, "y": 0.0, "speed": 1.0, "road_id": cid}]),
        make_row(t=5, lap_position=at + 1, lane_index=2),
    ]
    m = interaction_counts(rows, CTX, [])
    assert (m["ix_lc_req_left"], m["ix_lc_req_left_follower"], m["ix_lc_req_right"]) == (1, 1, 0)
    assert (m["ix_lc_done"], m["ix_lc_done_occupied"], m["ix_lc_done_follower"], m["ix_lc_done_mid"]) == (1, 1, 0, 0)
    assert m["ix_cutin"] == 1
    assert m["ix_crossing_approach"] >= 1 and m["ix_crossing_ped"] == 1


def test_interaction_counts_stop_follower():
    rows = _approach_rows(16, 1)
    rows[3]["neighbors"] = _nb(follower=12.0)
    stops = stop_metrics(rows, CTX)
    m = interaction_counts(rows, CTX, stops)
    assert (m["ix_stop_served"], m["ix_stop_follower"]) == (1, 1)


def test_box_overlap_and_unreported_rows():
    bus = _box(0.0, 0.0, 0.0, 12.0, 2.5)                    # 북쪽을 보는 버스: y -12 ~ 0
    assert boxes_overlap(bus, _box(0.5, -3.0, 0.0, 4.5, 1.8))
    assert not boxes_overlap(bus, _box(4.0, -3.0, 0.0, 4.5, 1.8))
    assert not boxes_overlap(bus, _box(0.0, 6.0, 0.0, 4.5, 1.8))
    car = {"id": "c1", "x": 0.5, "y": -3.0, "heading": 0.0, "length": 4.5, "width": 1.8}
    rows = [make_row(t=0, vehicles=[car]), make_row(t=1, vehicles=[car]), make_row(t=2)]
    rows[1]["window"]["collisions"].append({"collider": BUS, "victim": "c1", "ego": True})
    m = overlap_metrics(rows, {"length": 12.0, "width": 2.5})
    assert (m["bbox_overlap_rows"], m["bbox_overlap_unreported_rows"], m["bbox_overlap_vehicles"]) == (2, 1, 1)


def test_background_collisions_near_bus_only():
    rows = [make_row(t=0, vehicles=[{"id": "a"}]), make_row(t=1, vehicles=[{"id": "a"}])]
    for r in rows:
        r["window"]["collisions"].append({"collider": "a", "victim": "b", "ego": False})
    rows[1]["window"]["collisions"].append({"collider": "far1", "victim": "far2", "ego": False})
    assert background_collisions(rows) == {"bg_collisions_near": 1, "abnormal_traffic": True}
    assert background_collisions([make_row(t=0)])["abnormal_traffic"] is False


def test_analyze_run_has_extra_metrics(tmp_path):
    m = analyze_run(make_run_dir(tmp_path))
    assert m["outcome"] == "incomplete_service" and m["failure_location"]["edge"] == "R69"
    for k in ("longest_stall_s", "ix_lc_done", "ix_cutin", "bbox_overlap_rows", "bg_collisions_near"):
        assert k in m


def test_summary_extra_sections_and_wilson(tmp_path):
    import eval_bc_closed_loop as ev
    lo, hi = ev.wilson(5, 10)
    assert lo == pytest.approx(0.2366, abs=1e-3) and hi == pytest.approx(0.7634, abs=1e-3)
    loc = {"edge": "R81", "next_tls": "J42", "prev_stop": "서교동교회", "t": 900.0}
    cases = [("base/seed1", 1000, "arrived", None), ("base/seed1", 1001, "stuck", loc),
             ("base/seed2", 1000, "time_limit", loc), ("expert", 1000, "arrived", None)]
    for agent, seed, oc, fl in cases:
        d = tmp_path / agent / f"seed{seed}"
        d.mkdir(parents=True)
        m = {"end_reason": "stuck" if oc == "stuck" else oc, "success": oc == "arrived", "stops": [],
             "longest_stall_s": 300.0 if oc != "arrived" else 10.0, "failure_location": fl,
             "ix_lc_done": 8, "ix_lc_done_mid": 6, "abnormal_traffic": False, "bbox_overlap_unreported_rows": 0}
        if oc != "time_limit":
            m["outcome"] = oc                                   # time_limit 쪽은 예전 result.json(outcome 없음) 흉내
        (d / "result.json").write_text(json.dumps({"agent": agent, "seed": seed, "run_dir": f"{agent}/seed{seed}/run",
                                                   "metrics": m}), encoding="utf-8")
    ev.summarize(str(tmp_path))
    text = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "## 결과 분포" in text and "## 실패 위치" in text and "J42 앞 (서교동교회 뒤)" in text
    assert "| 실제 차로 변경 | 8.00 | 8.00 |" in text and "6.00 (75%)" in text
    hl = json.loads((tmp_path / "highlights.json").read_text(encoding="utf-8"))
    assert hl["base"]["worst_stuck"]["clip"] == [870.0, 905.0] and hl["base"]["best"]["seed"] == 1000
    assert (tmp_path / "failures.csv").exists()


def test_seed_parsing_and_agent_names(tmp_path):
    import eval_bc_closed_loop as ev
    assert ev.parse_seeds("1000-1002,1010") == [1000, 1001, 1002, 1010]
    assert ev.agent_name("expert") == "expert"
    assert ev.agent_name(str(tmp_path / "bc" / "base" / "seed1"), str(tmp_path / "bc")) == "base/seed1"


# ---------------------------------------------------------------- SUMO-LC 월드 평가
def test_use_world_sumo_lc_swaps_collect_hooks(monkeypatch):
    import collect_pdm_bus as C
    import collect_pdm_bus_sumo_lc as LC
    import eval_bc_closed_loop as ev
    monkeypatch.setattr(C, "_world", C._world)
    monkeypatch.setattr(C, "build_metadata", C.build_metadata)
    assert ev.use_world("hapjeong") == ("baseline", 120.0) and C._world is not LC._world
    assert ev.use_world("sumo_lc") == ("busy", 0.0)
    assert C._world is LC._world and C.build_metadata is LC._build_metadata


def test_one_cmd_passes_world():
    import argparse
    import eval_bc_closed_loop as ev
    a = argparse.Namespace(out="o", warmup=None, max_seconds=300.0, stall_seconds=180.0, world="sumo_lc")
    cmd = ev.one_cmd(a, "m", "n", 3)
    assert cmd[cmd.index("--world") + 1] == "sumo_lc" and "--warmup" not in cmd
