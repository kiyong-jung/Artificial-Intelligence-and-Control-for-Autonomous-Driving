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


def _pilot(bus=None):
    from maps import load_map
    from run_hapjeong_scenario import map_stops
    from traffic.alleys import read_net
    m = load_map("hapjeong")
    tc = FakeTC({})
    tc.route = _Simple(tc, "route", {})
    tc.vehicle.add = lambda *a, **k: tc.calls.append(("vehicle.add", a))     # 가짜 도메인은 키워드 인자를 받지 않는다
    p = PolicyBusPilot(tc, read_net(m["net_path"]), m["routes"]["bus_loop"], map_stops(m))
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


def test_seed_parsing_and_agent_names(tmp_path):
    import eval_bc_closed_loop as ev
    assert ev.parse_seeds("1000-1002,1010") == [1000, 1001, 1002, 1010]
    assert ev.agent_name("expert") == "expert"
    assert ev.agent_name(str(tmp_path / "bc" / "base" / "seed1"), str(tmp_path / "bc")) == "base/seed1"
