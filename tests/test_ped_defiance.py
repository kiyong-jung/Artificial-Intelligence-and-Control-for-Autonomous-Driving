"""A3 보강(PedDefianceManager): 계수 함수와, 합성 횡단보도에서 통과 시간이 길수록 더 건너는지."""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.ped_defiance import (DefianceCfg, age_factor, distract_chance, group_factor,  # noqa: E402
                                  size_factor, ttc_factor, wait_factor)
from traffic.pedestrian import BASE, CROSSING, build_a3_net, ped_vtypes_xml, traci  # noqa: E402
from traffic.ped_defiance import PedDefianceManager  # noqa: E402
from traffic.scenarios import NET_DIR, _write  # noqa: E402
from traffic.sumo_conn import sumo_cmd  # noqa: E402

C = DefianceCfg()


def test_factor_functions():
    assert ttc_factor(0.5, C) == C.f_extreme and ttc_factor(2.0, C) == C.f_under_lower
    assert ttc_factor(3.0, C) < ttc_factor(4.5, C) < ttc_factor(6.0, C) <= C.f_over_upper
    assert wait_factor(10, C) == 1.0 and wait_factor(48, C) > 1.0
    assert group_factor(1, C) == 1.0 < group_factor(2, C) < group_factor(5, C)
    assert size_factor(2.0 * 2.5, C) == C.f_large                       # 버스는 큰 차량이라 건너기 어렵다
    assert age_factor(6) < age_factor(25) and age_factor(80) < age_factor(25)
    assert distract_chance(16) > distract_chance(40) > 0


def test_vehicle_wait_factor_lets_long_waiting_cars_go():
    from traffic.ped_defiance import veh_wait_factor
    assert veh_wait_factor(0, C) == veh_wait_factor(C.veh_wait_free, C) == 1.0
    assert veh_wait_factor(6, C) < 1.0
    assert veh_wait_factor(C.veh_wait_stop, C) == veh_wait_factor(60, C) == C.f_veh_wait_min
    assert veh_wait_factor(4, C) > veh_wait_factor(8, C) > veh_wait_factor(11, C)


def test_standing_car_is_not_treated_as_far_away():
    """서 있는 차는 곧 출발할 차(통과 시간 2~3초대)로 본다. 예전처럼 10초로 보면 건너기 계수가 최대(×3)가 된다."""
    class _V:
        def getLanePosition(self, v): return 95.0
        def getSpeed(self, v): return 0.0
    class _L:
        def getLastStepVehicleIDs(self, lane): return ["v"]
        def getLength(self, lane): return 100.0
    class _Tc:
        vehicle, lane = _V(), _L()
    m = object.__new__(PedDefianceManager)
    m.traci, m.cfg = _Tc(), C
    veh, ttc = m._closest("lane")
    assert veh == "v" and ttc == C.standing_ttc                       # 5 m 앞에 선 차: 하한 2초
    assert ttc_factor(ttc, C) == C.f_under_lower                      # 건너기 계수 ×0.1
    assert ttc_factor(10.0, C) == C.f_over_upper                      # 예전 값은 ×3이었다


def _episode(v, T, seed, on):
    d = os.path.join(NET_DIR, "a3")
    _write(os.path.join(d, "r.rou.xml"), f"""<routes>
<vType id="bus_test" vClass="bus" length="12" width="2.5" height="3.2" accel="1.2" decel="4.0" tau="1.0" sigma="0"/>
<vType id="ped_base" {BASE}/>{ped_vtypes_xml(1.5, 0.75)}
<route id="r_main" edges="main_in main_out"/>
<person id="p0" type="ped_base" depart="0" departPos="289"><walk from="main_in" to="rev_out" arrivalPos="20"/></person>
</routes>""")
    cmd = sumo_cmd(build_a3_net(), seed=seed)
    traci.load(cmd[1:]) if traci.isLoaded() else traci.start(cmd)
    traci.vehicle.add("bus0", "r_main", typeID="bus_test", departPos=str(292.8 - v * T), departLane="1", departSpeed=str(v))
    mgr = PedDefianceManager(traci, "bus0", rng=random.Random(seed)) if on else None
    enter = bus = None
    t, k = 0.0, 0
    while t < 50:
        if "bus0" in traci.vehicle.getIDList():
            traci.vehicle.setSpeed("bus0", v)
        if mgr:
            mgr.update(t)
        traci.simulationStep()
        k += 1
        t = round(k * 0.1, 6)
        if bus is None and "bus0" in traci.vehicle.getIDList() and traci.vehicle.getRoadID("bus0") != "main_in":
            bus = t
        if enter is None and "p0" in traci.person.getIDList() and traci.person.getRoadID("p0") == CROSSING:
            enter = t
        if bus is not None and (enter is not None or t > bus + 20):
            break
    return enter is not None and (bus is None or enter < bus)


def test_defiance_raises_crossing_with_longer_ttc():
    n = 6
    short = sum(_episode(6.0, 3, s, True) for s in range(n))
    long_ = sum(_episode(6.0, 12, s, True) for s in range(n))
    off = sum(_episode(6.0, 12, s, False) for s in range(n))
    assert short == 0 and long_ >= 4 and long_ > off
