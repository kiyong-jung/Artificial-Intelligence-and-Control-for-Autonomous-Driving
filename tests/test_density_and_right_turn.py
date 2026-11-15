"""밀도 분산(InteriorSpawner)과 신호 교차로 우회전(right_turn)."""
import os
import random
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.interior_spawner import InteriorSpawner  # noqa: E402
from traffic.right_turn import right_turn_links  # noqa: E402


@pytest.fixture(scope="module")
def world(tmp_path_factory):
    w = HapjeongTraffic(str(tmp_path_factory.mktemp("dens")), seed=0)
    w.build()
    yield w
    w.close()


def test_entry_flows_are_reduced_by_the_interior_share(world):
    """입구 흐름을 30% 줄이고 그만큼을 노선 위 출발로 옮기므로 전체 대수는 같다."""
    rou = ET.parse(os.path.join(world.out_dir, "routes.rou.xml")).getroot()
    after = sum(float(f.get("period")[4:-1]) * 3600 for f in rou.iter("flow")
                if f.get("type") == "carDist" and (f.get("period") or "").startswith("exp("))
    moved = world.interior_per_h
    assert moved > 0 and world.summary["interior"]["share"] == 0.3
    assert moved / (after + moved) == pytest.approx(0.3, abs=1e-3)


class _FakeTc:
    class vehicle:
        added = []

        @classmethod
        def getIDList(cls): return ["bus0"]

        @classmethod
        def getPosition(cls, v): return (250.0, 150.0)

        @classmethod
        def add(cls, vid, rid, **kw): cls.added.append((vid, rid, kw))

    class route:
        routes = {}

        @classmethod
        def add(cls, rid, edges): cls.routes[rid] = edges


def test_spawner_never_places_cars_near_the_test_bus(world):
    """시험용 버스에서 min_dist 안에는 투입하지 않는다(눈앞에 차가 갑자기 생기지 않는다)."""
    m = load_map("hapjeong")
    net = read_net(os.path.join(world.out_dir, "network.net.xml"))
    tc = _FakeTc()
    tc.vehicle.added.clear()
    sp = InteriorSpawner(tc, net, "bus0", m["routes"]["bus_loop"], sorted({f["to"] for f in m["flows"]}),
                         per_h=3600.0 * 10, rng=random.Random(1), min_dist=200.0, dt=0.1)
    for k in range(3000):
        sp.update(k * 0.1)
    assert sp.n_spawned > 100 and sp.n_skipped_near > 0
    for vid, rid, kw in tc.vehicle.added:
        edge = tc.route.routes[rid][0]
        x, y = sp._xy(edge, float(kw["departPos"]))
        assert ((x - 250.0) ** 2 + (y - 150.0) ** 2) ** 0.5 >= 200.0 - 1e-6
        assert kw["typeID"] == "carDist"


def test_right_on_red_changes_every_signal_but_the_main_crossing(world):
    tc = world.start(warmup=5)
    ror = world.summary["right_on_red"]
    assert "J1" not in ror                                           # 합정역 사거리는 제외
    assert set(ror) == {"J42", "J74", "J82", "J83", "J96", "cluster_J52_J56_J58"}
    assert all(v["links"] > 0 and v["opened_share"] > 0 for v in ror.values())
    # 횡단보도 보행자 신호가 켜져 있는(건너는 사람이 있을 수 있는) 위상에서는 열지 않는다
    assert ror["J83"]["blocked_by_ped"] > 0.2 and ror["J42"]["blocked_by_ped"] > 0.1
    assert all(v["opened_share"] + v["blocked_by_ped"] <= 1.0 for v in ror.values())
    # 바꾼 뒤에도 우회전 링크는 모든 위상에서 적색이 아닌 시간이 늘었다
    import sumolib
    net = sumolib.net.readNet(os.path.join(world.out_dir, "network.net.xml"), withInternal=False, withPrograms=True)
    tls = next(t for t in net.getTrafficLights() if t.getID() == "J42")
    links = right_turn_links(tls)
    greens = sum(ph.duration for lg in tc.trafficlight.getAllProgramLogics("J42") for ph in lg.phases
                 for i in links if ph.state[i] in "Gg")
    assert greens > 0
