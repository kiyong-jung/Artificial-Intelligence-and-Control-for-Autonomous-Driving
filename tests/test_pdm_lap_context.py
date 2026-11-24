"""PDM 버스 데이터: 한 바퀴 노선 문맥(pdm_data/lap_context.py). SUMO 없이 도로망 파일만 읽는다."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from maps import load_map  # noqa: E402
from pdm_data.lap_context import build_lap_context, lane_usable, lap_route_index  # noqa: E402
from pdm_data.schema import STOP_IDS  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.test_bus import lap_route  # noqa: E402


@pytest.fixture(scope="module")
def ctx():
    m = load_map("hapjeong")
    net = read_net(m["net_path"])
    edges = lap_route(net, m["routes"]["bus_loop"])
    return build_lap_context(m["net_path"], edges, m["bus_stops"],
                             m["route_design"]["bus_loop"]["max_speed_mps"], {})


def test_lap_edges_unique_and_start_with_R68(ctx):
    assert ctx.lap_edges[0] == "R68" and len(set(ctx.lap_edges)) == len(ctx.lap_edges)


def test_route_map_covers_R68(ctx):
    assert ctx.route_map.edge_ids[0] == "R68" and ctx.route_map.length > 1949.44


def test_service_stops_are_four_required(ctx):
    assert [s[2] for s in ctx.service_stops] == list(STOP_IDS)
    assert all(a[0] < b[0] for a, b in zip(ctx.service_stops, ctx.service_stops[1:]))


def test_lap_route_index_internal_uses_last_road(ctx):
    e = ctx.lap_edges
    assert lap_route_index("R71", "R69", e) == e.index("R71")
    assert lap_route_index(":J1_3_0", "R42", e) == e.index("R42")


def test_crossing_ids_from_net(ctx):
    assert ctx.crossing_ids and all(c.startswith(":") and "_c" in c for c in ctx.crossing_ids)


def test_lane_usable_empty_allowed_means_all(ctx):
    ctx.lane_allowed.update({"X_0": [], "X_1": ["pedestrian"], "X_2": ["bus", "taxi"]})
    assert lane_usable(ctx, "X_0") and not lane_usable(ctx, "X_1") and lane_usable(ctx, "X_2")
    assert not lane_usable(ctx, "NOT_IN_TABLE_0")


def test_vmax_from_map(ctx):
    assert ctx.vmax == pytest.approx(13.89)
