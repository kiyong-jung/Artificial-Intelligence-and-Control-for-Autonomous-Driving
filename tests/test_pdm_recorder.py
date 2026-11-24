"""PDM 버스 데이터: 판단 구간 기록기, 역할·시나리오·사건(pdm_data/recorder.py, scenario_tags.py). 가짜 TraCI로 시험한다."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_fakes import FakePilot, FakePlan, FakeTC, FakeWorld, hapjeong_ctx  # noqa: E402
from pdm_fixtures import make_row  # noqa: E402
from pdm_data.recorder import CallTap, CollectionDone, Recorder  # noqa: E402
from pdm_data.scenario_tags import RoleBook  # noqa: E402

CTX = hapjeong_ctx()


class MemWriter:
    def __init__(self):
        self.steps, self.events = [], []

    def write_step(self, row):
        self.steps.append(row)

    def write_event(self, ev):
        self.events.append(ev)

    def close(self):
        pass


def make_recorder(bus_appears_at_step=0, lap_changes_at_step=None, max_seconds=1e9, phases=None,
                  lane_after_step=None, max_laps=5):
    tc, pilot, world, w = FakeTC({}, now=0.0), FakePilot(), FakeWorld(), MemWriter()
    rec = Recorder(tc, world, pilot, CTX, w, max_laps=max_laps, max_seconds=max_seconds)
    rec._script = dict(appear=bus_appears_at_step, lap=lap_changes_at_step, phases=phases, lane=lane_after_step)
    return rec, w


def run_steps(rec, n):
    tc, pilot, s = rec.tc, rec.pilot, rec._script
    for i in range(1, n + 1):
        if i >= s["appear"] and "bus0" not in tc.vehicle.items:
            tc.vehicle.items["bus0"] = dict(pos=(0.0, 0.0), speed=5.0, road="R69", lane="R69_1", lane_index=1,
                                            lane_pos=10.0)
        if s["lap"] is not None and i == s["lap"]:
            pilot.lap += 1
        if s["phases"] is not None and i - 1 < len(s["phases"]):
            pilot.phase = s["phases"][i - 1]
            if pilot.phase == "signal" and pilot.plan is None:
                pilot.plan, pilot.why, pilot.target = FakePlan("S3", "R69", 80.0, 2), "S3", 2
        if s["lane"] is not None and i == s["lane"]:
            tc.vehicle.items["bus0"].update(lane="R69_2", lane_index=2)
        tc.now = round(i * 0.1, 6)
        rec.on_step(tc.now)


def test_no_rows_until_bus_present():
    rec, w = make_recorder(bus_appears_at_step=7)
    run_steps(rec, 21)
    assert w.steps[0]["meta"]["t"] == 0 and w.steps[0]["meta"]["sim_step"] == 10


def test_rows_every_5_steps_window_has_5_substeps():
    rec, w = make_recorder()
    run_steps(rec, 21)                       # 스냅숏 5·10·15·20 스텝, 행은 구간이 끝난 10·15·20 스텝에 쓴다
    assert [r["meta"]["t"] for r in w.steps] == [0, 1, 2]
    assert all(len(r["window"]["substeps"]) == 5 for r in w.steps)
    assert w.steps[0]["window"]["substeps"][0]["t"] == pytest.approx(0.6)


def test_lap_change_writes_final_then_t0():
    rec, w = make_recorder(lap_changes_at_step=13)
    run_steps(rec, 21)
    eps = [(r["meta"]["episode"], r["meta"]["t"], r["meta"]["final"]) for r in w.steps]
    assert (1, 2, True) in eps and (2, 0, False) in eps
    assert w.steps[[e[:2] for e in eps].index((1, 1))]["window"]["lap_done"] is True


def test_max_laps_stops_after_final():
    rec, w = make_recorder(lap_changes_at_step=13, max_laps=1)
    with pytest.raises(CollectionDone):
        run_steps(rec, 21)
    assert w.steps[-1]["meta"]["final"] and w.steps[-1]["meta"]["end"] == "lap"


def test_time_limit_writes_final_row():
    rec, w = make_recorder(max_seconds=1.0)
    with pytest.raises(CollectionDone):
        run_steps(rec, 30)
    rec.finish("time_limit")
    assert w.steps[-1]["meta"]["final"] and w.steps[-1]["meta"]["end"] == "time_limit"
    assert w.steps[-1]["window"] is None


def test_bus_lost_ends_collection():
    rec, w = make_recorder()
    run_steps(rec, 7)
    del rec.tc.vehicle.items["bus0"]
    with pytest.raises(CollectionDone) as e:
        rec.on_step(0.8)
    rec.finish(e.value.reason)
    assert e.value.reason == "bus_lost" and w.steps[-1]["meta"]["end"] == "bus_lost"


def test_calltap_forwards_and_records():
    tc = FakeTC()
    tap = CallTap(tc, "bus0")
    tap.install()
    tc.vehicle.changeLane("bus0", 2, 5.0)
    tc.vehicle.changeLane("other", 1, 5.0)
    assert [c["args"] for c in tap.drain()] == [("bus0", 2, 5.0)]
    assert ("vehicle.changeLane", ("other", 1, 5.0)) in tc.calls
    tap.uninstall()
    assert tap.drain() == []


def test_lane_window_event_on_signal_request_idle():
    rec, w = make_recorder(phases=["idle"] * 5 + ["signal"] * 5 + ["request"] * 5 + ["idle"] * 5, lane_after_step=14)
    run_steps(rec, 21)
    ev = [e for e in w.events if e["kind"] == "lane_window"]
    assert len(ev) == 1 and ev[0]["detail"]["success"] is True and ev[0]["detail"]["plan_kind"] == "S3"


def test_request_direction_in_substeps():
    rec, w = make_recorder(phases=["idle"] * 5 + ["signal"] * 5 + ["request"] * 10)
    run_steps(rec, 21)
    subs = [s for r in w.steps for s in r["window"]["substeps"]]
    assert any(s["phase"] == "request" and s["request_direction"] == 1 for s in subs)


def test_forced_stop_direction_from_blinker():
    rec, w = make_recorder()
    run_steps(rec, 6)
    rec.pilot._forced_p, rec.pilot.phase = FakePlan("STOP"), "cool"
    rec.tc.vehicle.items["bus0"]["signals"] = 2                   # 왼쪽 방향지시등
    for i in range(7, 12):
        rec.tc.now = round(i * 0.1, 6)
        rec.on_step(rec.tc.now)
    subs = w.steps[-1]["window"]["substeps"]
    assert all(s["forced"] and s["request_direction"] == 1 for s in subs[1:])


def test_ego_teleport_ends_with_teleport():
    rec, w = make_recorder()
    run_steps(rec, 7)
    del rec.tc.vehicle.items["bus0"]
    rec.tc.teleports = ["bus0"]
    with pytest.raises(CollectionDone) as e:
        rec.on_step(0.8)
    rec.finish(e.value.reason)
    assert e.value.reason == "teleport" and w.steps[-1]["meta"]["end"] == "teleport"
    assert rec.summary()["ego_teleports"] == 1 and any(ev["kind"] == "teleport" for ev in w.events)


def test_time_limit_is_per_lap():
    rec, w = make_recorder(lap_changes_at_step=13, max_seconds=1.0)
    run_steps(rec, 21)                         # 바퀴가 1.5 s에 바뀌므로 2.0 s에는 새 바퀴 0.5 s째다(전체 시간 기준이면 1.5 s에 끝난다)
    assert (2, 0, False) in [(r["meta"]["episode"], r["meta"]["t"], r["meta"]["final"]) for r in w.steps]


def test_window_pdm_plans_only_from_window():
    rec, w = make_recorder()
    rec.pilot.last_plan = {"sim_time": 0.3, "fraction": 0.4}
    run_steps(rec, 7)
    rec.pilot.last_plan = {"sim_time": 0.7, "fraction": 0.6}
    run_steps_from(rec, 8, 11)
    assert [p["fraction"] for p in w.steps[0]["window"]["pdm_plans"]] == [0.6]


def test_calltap_restores_class_level_methods():
    class Dom:
        @staticmethod
        def changeLane(*a):
            return "orig"

        setMaxSpeed = setSpeed = changeLane
    tc = FakeTC()
    tc.vehicle = Dom
    tap = CallTap(tc, "bus0")
    tap.install()
    tap.uninstall()
    assert Dom.changeLane("bus0", 1, 1.0) == "orig"


def _bus_before_crossing(back=10.0):
    """S6 횡단보도(신호 없는 곳) back m 앞의 버스 자리: (도로, 차로 위치, 횡단보도 id)."""
    at, cid = CTX.s6_crossings[0]
    pos = at - back
    i = max(k for k, s in enumerate(CTX.route_map.start) if s <= pos)
    return CTX.lap_edges[i], pos - CTX.route_map.start[i], cid


def test_substep_records_crossing_peds():
    rec, w = make_recorder()
    edge, lane_pos, cid = _bus_before_crossing()
    rec.tc.edge.table["getLastStepPersonIDs"] = lambda c: ["p1"] if c == cid else []
    run_steps(rec, 1)
    rec.tc.vehicle.items["bus0"].update(road=edge, lane=f"{edge}_1", lane_pos=lane_pos)
    rec.last_road = edge
    run_steps_from(rec, 2, 11)
    sub = w.steps[0]["window"]["substeps"][-1]
    assert sub["crossings"] == {cid: ["p1"]}


def test_signalized_crossing_not_recorded():
    rec, w = make_recorder()
    at, cid = next((a, c) for a, c in CTX.route_map.crossings if c.startswith(":J82_"))
    i = max(k for k, s in enumerate(CTX.route_map.start) if s <= at - 10.0)
    edge = CTX.lap_edges[i]
    rec.tc.edge.table["getLastStepPersonIDs"] = lambda c: ["p1"]
    run_steps(rec, 1)
    rec.tc.vehicle.items["bus0"].update(road=edge, lane=f"{edge}_1", lane_pos=at - 10.0 - CTX.route_map.start[i])
    rec.last_road = edge
    run_steps_from(rec, 2, 11)
    assert w.steps[0]["window"]["substeps"][-1]["crossings"] == {}


def test_s6_tag_only_for_unsignalized_crossing():
    from pdm_data.scenario_tags import ScenarioTracker
    tr = ScenarioTracker(FakeTC(), FakeWorld(), FakePilot(), CTX)
    j82 = next(a for a, c in CTX.route_map.crossings if c.startswith(":J82_"))
    s6_at, s6_id = CTX.s6_crossings[0]
    assert tr.tags(make_row(lap_position=j82 - 20.0))["s6"] is None
    assert tr.tags(make_row(lap_position=s6_at - 20.0))["s6"]["crossing"] == s6_id


def test_no_per_step_person_scan_near_crossing():
    """S6 판정은 전처리로 옮겼다: 횡단보도 근처에서도 보행자 전체를 0.1 s마다 훑지 않는다(스냅숏 때만 읽는다)."""
    rec, w = make_recorder()
    edge, lane_pos, cid = _bus_before_crossing()
    rec.tc.person.items["p1"] = dict(road=cid)
    run_steps(rec, 1)
    rec.tc.vehicle.items["bus0"].update(road=edge, lane=f"{edge}_1", lane_pos=lane_pos)
    rec.last_road = edge
    run_steps_from(rec, 2, 11)
    n = sum(1 for name, args in rec.tc.calls if name == "person.getRoadID" and args == ("p1",))
    assert n <= 2 and not [e for e in w.events if e["kind"] in ("s5_cutin", "s6_pass")]


def run_steps_from(rec, start, end):
    for i in range(start, end + 1):
        rec.tc.now = round(i * 0.1, 6)
        rec.on_step(rec.tc.now)


def test_rolebook_a1_and_injected():
    rb = RoleBook(FakeWorld(inter={"car7": True}, injector_log=[{"t": 1.0, "lap": 1, "kind": "S5", "id": "inj_S51",
                                                                   "ok": "injected"},
                                                                  {"t": 2.0, "lap": 1, "kind": "S3", "id": "car3",
                                                                   "ok": "natural"}]))
    rb.update()
    r = rb.roles()["vehicles"]
    assert r["car7"]["role"] == "A1_yield" and r["inj_S51"]["injected"] is True and r["inj_S51"]["role"] == "S5"
    assert r["car3"]["injected"] is False and r["car3"]["role"] == "S3"
    assert [e["kind"] for e in rb.new_events()] == ["injection", "injection"] and rb.new_events() == []


def test_rolebook_pedestrian_roles_and_a3():
    rb = RoleBook(FakeWorld(ped_log=[{"t": 3.0, "ped": "s6ped4", "crossing": "c", "ttc": 4.0, "p": 0.3, "cross": True,
                                      "wait": 2.0, "factors": {}, "vehicle": "bus0"}]))
    rb.update()
    p = rb.roles()["pedestrians"]
    assert p["s6ped4"]["role"] == "S6_background" and p["s6ped4"]["a3_decision"] is True
    rb2 = RoleBook(FakeWorld())
    rb2.update()
    assert rb2.role_for_person("inj_s6_2")["role"] == "S6_injected"


def test_fixture_matches_recorder_row_keys():
    rec, w = make_recorder()
    run_steps(rec, 11)
    assert set(w.steps[0]) == set(make_row())
    assert set(w.steps[0]["window"]) == set(make_row()["window"])
    assert set(w.steps[0]["window"]["substeps"][0]) == set(make_row()["window"]["substeps"][0])
    assert set(w.steps[0]["meta"]) == set(make_row()["meta"]) and set(w.steps[0]["expert"]) == set(make_row()["expert"])
