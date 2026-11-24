"""PDM 버스 데이터: TraCI 스냅숏(pdm_data/snapshot.py). 가짜 TraCI로 시험한다."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_fakes import FakeTC, hapjeong_ctx  # noqa: E402
from pdm_data.snapshot import take_snapshot  # noqa: E402

ROLES = {"vehicles": {}, "pedestrians": {}}
EGO = dict(pos=(0.0, 0.0), speed=8.0, road="R69", lane="R69_1", lane_index=1, lane_pos=10.0)


@pytest.fixture(scope="module")
def ctx():
    return hapjeong_ctx()


def test_snapshot_has_all_blocks(ctx):
    snap = take_snapshot(FakeTC({"bus0": EGO}), ctx, "R68", ROLES, 10.0)
    assert set(snap) == {"ego", "route", "tls", "stops", "neighbors", "junction_foes", "vehicles", "pedestrians"}
    assert snap["ego"]["lap_route_index"] == ctx.lap_edges.index("R69")
    assert snap["ego"]["lap_position"] == pytest.approx(ctx.route_map.start[ctx.lap_edges.index("R69")] + 10.0)


def test_vehicles_block_filters_radius_150(ctx):
    tc = FakeTC({"bus0": EGO, "near": dict(pos=(0.0, 100.0)), "far": dict(pos=(0.0, 200.0))})
    v = take_snapshot(tc, ctx, "R68", ROLES, 10.0)["vehicles"]
    assert [x["id"] for x in v] == ["near"]


def test_vehicles_block_skips_vanished(ctx):
    tc = FakeTC({"bus0": EGO, "ok": dict(pos=(0.0, 20.0)), "gone": dict(pos=(0.0, 30.0), gone=True)})
    v = take_snapshot(tc, ctx, "R68", ROLES, 10.0)["vehicles"]
    assert "gone" not in [x["id"] for x in v] and len(v) == 1


def test_vehicle_roles_and_route_distance(ctx):
    tc = FakeTC({"bus0": EGO, "inj_s5_1": dict(pos=(0.0, 20.0), road="R69", lane="R69_2", lane_pos=30.0)})
    roles = {"vehicles": {"inj_s5_1": {"injected": True, "role": "S5"}}, "pedestrians": {}}
    v = take_snapshot(tc, ctx, "R68", roles, 10.0)["vehicles"][0]
    assert v["injected"] is True and v["role"] == "S5" and v["route_distance"] == pytest.approx(20.0)


def test_neighbors_null_when_empty(ctx):
    n = take_snapshot(FakeTC({"bus0": EGO}), ctx, "R68", ROLES, 10.0)["neighbors"]
    assert set(n) == {"leader", "follower", "left_leader", "right_leader", "left_follower", "right_follower"}
    assert all(v is None for v in n.values())


def test_leader_secure_gap_uses_leader_decel(ctx):
    ego = dict(EGO, leader=lambda d: ("lead", 20.0))
    tc = FakeTC({"bus0": ego, "lead": dict(pos=(0.0, 25.0), speed=6.0, decel=4.5)})
    snap = take_snapshot(tc, ctx, "R68", ROLES, 10.0)
    assert ("vehicle.getSecureGap", ("bus0", 8.0, 6.0, 4.5, "lead")) in tc.calls
    assert snap["neighbors"]["leader"]["gap_raw"] == 20.0
    assert snap["neighbors"]["leader"]["follow_speed"] == 9.0


def test_left_follower_picks_nearest_and_uses_follower_frame(ctx):
    ego = dict(EGO, neighbors=lambda mode: (("f1", 12.0), ("f2", 4.0)) if mode == 1 else ())
    tc = FakeTC({"bus0": ego, "f1": dict(pos=(-3.0, -12.0), speed=9.0), "f2": dict(pos=(-3.0, -4.0), speed=10.0)})
    slot = take_snapshot(tc, ctx, "R68", ROLES, 10.0)["neighbors"]["left_follower"]
    assert slot["id"] == "f2" and slot["gap_raw"] == 4.0
    assert ("vehicle.getSecureGap", ("f2", 10.0, 8.0, 4.5, "bus0")) in tc.calls


def test_tls_time_to_switch_relative(ctx):
    ego = dict(EGO, next_tls=(("J82", 3, 40.0, "r"), ("J9", 0, 250.0, "G")))
    t = take_snapshot(FakeTC({"bus0": ego}), ctx, "R68", ROLES, 10.0)["tls"]
    assert len(t) == 1 and t[0]["time_to_switch"] == 27.0 and t[0]["state"] == "r" and t[0]["link_index"] == 3


def test_pedestrian_on_crossing(ctx):
    cid = sorted(ctx.crossing_ids)[0]
    tc = FakeTC({"bus0": EGO}, persons={"p1": dict(pos=(5.0, 10.0), road=cid), "p9": dict(pos=(0.0, 400.0))})
    p = take_snapshot(tc, ctx, "R68", ROLES, 10.0)["pedestrians"]
    assert [x["id"] for x in p] == ["p1"] and p[0]["on_crossing"] is True


def test_no_context_subscription(ctx):
    tc = FakeTC({"bus0": EGO, "near": dict(pos=(0.0, 100.0))})
    take_snapshot(tc, ctx, "R68", ROLES, 10.0)
    assert not any(name.endswith("subscribeContext") for name, _ in tc.calls)
