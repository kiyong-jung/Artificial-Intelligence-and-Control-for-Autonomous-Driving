"""노선 구간 정의 도우미(density_keeper.sections_of): 구간별 밀도 측정과 지도에 쓴다."""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.alleys import read_net  # noqa: E402
from traffic.density_keeper import SECTION_ROADS, sections_of  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")


def _net():
    return read_net(os.path.join(ROOT, "source", "network.net.xml"))


def _loop():
    with open(os.path.join(ROOT, "map.json"), encoding="utf-8") as f:
        return json.load(f)["routes"]["bus_loop"]


def test_sections_cover_the_four_named_roads():
    secs = sections_of(_net(), _loop())
    assert {s.name for s in secs} == set(SECTION_ROADS)
    assert all(s.length >= 60.0 for s in secs)


def test_bus_lane_section_has_a_lower_lane_scale_by_its_passenger_lane_share():
    # 양화로 R54a는 4차로 중 4번이 버스 전용이라 승용차는 3차로만 쓴다(같은 양화로 R54는 5차로)
    secs = {s.edge: s for s in sections_of(_net(), _loop())}
    assert secs["R54a"].lane_scale == pytest.approx(3 / 5) and secs["R54"].lane_scale == 1.0
    assert all(s.lane_scale == 1.0 for s in secs.values() if s.name != "양화로")
