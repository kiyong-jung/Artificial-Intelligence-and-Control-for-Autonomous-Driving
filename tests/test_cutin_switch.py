"""끼어들기 차는 합류 골목 안에서만 끼어들기 vType이고, 그 밖(신호 횡단보도 포함)에서는 보통 차 vType이다."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.cutin import CutinTypeSwitch  # noqa: E402


class _V:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return list(self.tc.cars)

    def getTypeID(self, v):
        return self.tc.cars[v]["type"]

    def getRoadID(self, v):
        return self.tc.cars[v]["road"]

    def setType(self, v, t):
        self.tc.cars[v]["type"] = t
        self.tc.calls.append((v, t))


class _S:
    def __init__(self, tc):
        self.tc = tc

    def getDepartedIDList(self):
        return list(self.tc.departed)


class _TC:
    def __init__(self):
        self.cars, self.departed, self.calls = {}, [], []
        self.vehicle, self.simulation = _V(self), _S(self)


def test_cutin_type_only_inside_the_merge_alley():
    tc = _TC()
    sw = CutinTypeSwitch(tc, feeders={"G12"}, normal_type="car_krauss")
    tc.cars["a"] = {"type": "cutin_3", "road": "R71"}          # 큰길에서 출발(골목 가는 길)
    tc.departed = ["a"]
    sw.update(0.0)
    assert tc.cars["a"]["type"] == "car_krauss"                 # 골목 밖: 보행자를 무시하지 않는 보통 차
    tc.departed = []
    tc.cars["a"]["road"] = "G12"
    sw.update(1.0)
    assert tc.cars["a"]["type"] == "cutin_3"                    # 골목 안: 버스 앞으로 끼어든다
    tc.cars["a"]["road"] = ":X_dongmak6_3"
    sw.update(2.0)
    assert tc.cars["a"]["type"] == "cutin_3"                    # 합류 중(교차로 안)
    tc.cars["a"]["road"] = "R71.256"
    sw.update(3.0)
    assert tc.cars["a"]["type"] == "car_krauss"                 # 합류를 마쳤다: 다시 보통 차


def test_vehicle_departing_inside_alley_keeps_cutin_type_until_merged():
    tc = _TC()
    sw = CutinTypeSwitch(tc, feeders={"G24"}, normal_type="car_krauss")
    tc.cars["inj_s51"] = {"type": "cutin_0", "road": "G24"}     # 주입기가 골목 안에 넣은 끼어들기 차
    tc.departed = ["inj_s51"]
    sw.update(0.0)
    assert tc.cars["inj_s51"]["type"] == "cutin_0" and tc.calls == []
    tc.departed, tc.cars["inj_s51"]["road"] = [], "R112.112"
    sw.update(1.0)
    assert tc.cars["inj_s51"]["type"] == "car_krauss"


def test_existing_cutin_vehicles_are_picked_up_on_first_update():
    tc = _TC()
    tc.cars["w"] = {"type": "cutin_8", "road": "R73"}           # 예열 중에 나온 차
    CutinTypeSwitch(tc, feeders={"G12"}, normal_type="car_krauss").update(0.0)
    assert tc.cars["w"]["type"] == "car_krauss"
