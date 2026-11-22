"""신호 교차로 우회전(right_turn)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.right_turn import right_turn_links  # noqa: E402


@pytest.fixture(scope="module")
def world(tmp_path_factory):
    w = HapjeongTraffic(str(tmp_path_factory.mktemp("dens")), seed=0)
    w.build()
    yield w
    w.close()


def test_right_on_red_changes_every_signal_but_the_main_crossing(world):
    tc = world.start(warmup=5)
    ror = world.summary["right_on_red"]
    assert "J1" not in ror and "J82" not in ror and "J83" not in ror and "J42" not in ror   # 합정역·상수역·삼거리포차 앞·서교동사거리는 제외
    assert set(ror) == {"J74", "J96", "cluster_J52_J56_J58"}
    assert all(v["links"] > 0 and v["opened_share"] > 0 for v in ror.values())
    # 횡단보도 보행자 신호가 켜져 있는(건너는 사람이 있을 수 있는) 위상에서는 열지 않는다
    assert ror["J74"]["blocked_by_ped"] > 0.1
    assert all(v["opened_share"] + v["blocked_by_ped"] <= 1.0 for v in ror.values())
    # 바꾼 뒤에도 우회전 링크는 모든 위상에서 적색이 아닌 시간이 늘었다
    import sumolib
    net = sumolib.net.readNet(os.path.join(world.out_dir, "network.net.xml"), withInternal=False, withPrograms=True)
    tls = next(t for t in net.getTrafficLights() if t.getID() == "J74")
    links = right_turn_links(tls)
    greens = sum(ph.duration for lg in tc.trafficlight.getAllProgramLogics("J74") for ph in lg.phases
                 for i in links if ph.state[i] in "Gg")
    assert greens > 0
