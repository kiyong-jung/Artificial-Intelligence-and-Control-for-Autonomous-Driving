"""끼어들기 차는 시험용 버스만 무시하고 배경 차와는 상호작용(앞차 무시)하지 않는다."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import test_injector_plan as T  # noqa: E402


def test_cutter_ignores_only_the_test_bus():
    tc = T._TC2(bus_road="R71", bus_pos=T._near_merge())
    inj = T._inj(tc, alleys=[a for a in T._alleys() if a.junction == "X_dongmak3"])
    inj.update(0.0)
    inj._s5(0.1, inj._state())
    cut = [a for a in tc.f.added if a["id"].startswith("inj_s5")]
    assert cut
    assert cut[0]["type"] == "carDist"                             # 앞차를 모두 무시하는 vType(cutin_*)이 아니다
    inj.update(0.2)                                                # 나타난 다음 스텝에 무시 대상을 지정한다
    assert tc.f.params[(cut[0]["id"], "junctionModel.ignoreIDs")] == "bus0"


def test_no_cutter_when_bus_is_far_from_the_alley():
    tc = T._TC2(bus_road="R69", bus_pos=10.0)                      # 합류점에서 멀다
    inj = T._inj(tc, alleys=[a for a in T._alleys() if a.junction == "X_dongmak3"])
    inj.update(0.0)
    inj._s5(0.1, inj._state())
    assert not any(a["id"].startswith("inj_s5") for a in tc.f.added)
