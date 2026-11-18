"""우회전 링크가 양보하도록 만들어진 차량 링크(도로망 요청의 response)를 가려, 그런 링크는 직진·좌회전 녹색과 겹쳐도 열 수 있게 한다."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from maps import load_map  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.right_turn import right_turn_links, yields_to  # noqa: E402

_cache = {}


def _net():
    if "net" not in _cache:
        _cache["net"] = read_net(load_map("hapjeong")["net_path"])
    return _cache["net"]


def _tls(tid):
    return next(t for t in _net().getTrafficLights() if t.getID() == tid)


def test_j96_right_turns_that_yield_to_straight_links_are_found():
    tls = _tls("J96")
    links = right_turn_links(tls)
    y = yields_to(tls, links)
    assert {1, 2, 3, 4, 5} <= y[22]                  # -R126 -> R55a 우회전은 직진 링크 1~5에 양보한다
    assert {15, 16, 17, 18, 19, 21} <= y[7]
    assert y[0] == set() and y[14] == set()          # 양보하도록 만들어지지 않은 우회전은 빈 집합이다


def test_j42_right_turns_0_and_11_yield_to_no_vehicle_link_but_18_does():
    tls = _tls("J42")
    y = yields_to(tls, right_turn_links(tls))
    assert y[0] == set() and y[11] == set()
    assert {1, 2, 3, 6, 8} <= y[18]
