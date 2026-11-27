"""순환 노선이 닫혀 있는지, 닫히지 않은 노선의 끝과 처음 사이 도로를 잇는지 확인한다.

3.19.0부터 bus_loop는 J1 좌회전 뒤 R68에서 끝나 처음(R69)과 바로 이어진다."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.alleys import close_loop, read_net  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong")


def _net_and_route():
    net = read_net(os.path.join(ROOT, "source", "network.net.xml"))
    route = json.load(open(os.path.join(ROOT, "map.json"), encoding="utf-8"))["routes"]["bus_loop"]
    return net, route


def test_bus_loop_ends_with_the_j1_left_turn_and_is_closed():
    net, route = _net_and_route()
    assert route[-2:] == ["R42", "R68"]
    assert close_loop(net, route) == route
    # 끝 두 도로 사이 연결이 J1의 좌회전이다
    conns = net.getEdge("R42").getConnections(net.getEdge("R68"))
    assert conns and all(c.getDirection() in ("l", "L") for c in conns)


def test_close_loop_adds_the_edge_between_end_and_start():
    net, route = _net_and_route()
    open_route = route[:-1]
    assert close_loop(net, open_route) == route


def test_close_loop_keeps_route_when_edges_missing():
    net, _ = _net_and_route()
    assert close_loop(net, ["nope", "R69"]) == ["nope", "R69"]
    assert close_loop(net, ["R69"]) == ["R69"]
