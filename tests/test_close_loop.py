"""순환 노선의 끝(R42)과 처음(R69) 사이 도로(R68)를 이어 J1의 좌회전을 S2 구간에 넣는지 확인한다."""
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


def test_close_loop_adds_the_edge_between_end_and_start():
    net, route = _net_and_route()
    closed = close_loop(net, route)
    assert closed[:len(route)] == route
    assert closed[len(route):] == ["R68"]
    # 끝에서 R68로 가는 연결이 J1의 좌회전이다
    conns = net.getEdge("R42").getConnections(net.getEdge("R68"))
    assert conns and all(c.getDirection() in ("l", "L") for c in conns)


def test_close_loop_keeps_route_when_edges_missing():
    net, _ = _net_and_route()
    assert close_loop(net, ["nope", "R69"]) == ["nope", "R69"]
    assert close_loop(net, ["R69"]) == ["R69"]
