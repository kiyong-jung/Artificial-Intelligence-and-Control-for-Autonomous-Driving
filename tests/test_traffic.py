"""주변 교통 알고리즘(A1~A3)이 시뮬레이션에서 의도대로 실현되는지 확인하는 빠른 검증."""
import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.scenarios import (build_a1_net, run_a1_episode, traci, _write,  # noqa: E402
                               NET_DIR)
from traffic.scenarios_a2 import (build_a2_net, run_cutin_episode, build_s4_net,  # noqa: E402
                                  run_s4)
from traffic.pedestrian import build_a3_net, run_crossing_episode  # noqa: E402
from traffic.sumo_conn import sumo_cmd, checkBinary  # noqa: E402


@pytest.fixture(scope="module")
def a1():
    return build_a1_net()


def test_a1_outcome_follows_decision(a1):
    rows = [run_a1_episode(a1, t, s) for t in ("yielder", "nonyielder") for s in range(15)]
    assert all(bool(r["decision"]) == r["changed"] for r in rows)
    assert any(r["decision"] for r in rows) and any(not r["decision"] for r in rows)


def test_a1_no_leak_when_p_zero(a1):
    for t in ("yielder", "nonyielder"):
        rows = [run_a1_episode(a1, t, 500 + s, p_override=0.0) for s in range(10)]
        assert not any(r["changed"] for r in rows)


def test_a1_same_seed_same_result(a1):
    a = run_a1_episode(a1, "yielder", 3, record=True)
    b = run_a1_episode(a1, "yielder", 3, record=True)
    assert a == b


def test_a1_opengap_reaches_target_within_duration(a1):
    r = run_a1_episode(a1, "yielder", 1, record=True, bus_kwargs={"t_request": 1e9})
    target = r["new_tau"] * 9.0
    reach = next(s["t"] for s in r["series"] if s["t"] >= r["t_open"] and s["gap"] >= target - 0.5)
    assert reach - r["t_open"] <= 12.0


def test_lane_collision_is_recorded():
    d = os.path.join(NET_DIR, "coll")
    _write(os.path.join(d, "n.nod.xml"),
           '<nodes><node id="A" x="0" y="0"/><node id="B" x="500" y="0"/></nodes>')
    _write(os.path.join(d, "e.edg.xml"),
           '<edges><edge id="E0" from="A" to="B" numLanes="1" speed="13.9"/></edges>')
    subprocess.run([checkBinary("netconvert"), "-n", os.path.join(d, "n.nod.xml"), "-e",
                    os.path.join(d, "e.edg.xml"), "-o", os.path.join(d, "net.net.xml"),
                    "--no-warnings"], check=True, capture_output=True)
    _write(os.path.join(d, "r.rou.xml"),
           '<routes><vType id="c" sigma="0"/><route id="r" edges="E0"/></routes>')
    _write(os.path.join(d, "c.sumocfg"),
           '<configuration><input><net-file value="net.net.xml"/>'
           '<route-files value="r.rou.xml"/></input></configuration>')
    cmd = sumo_cmd(os.path.join(d, "c.sumocfg"))
    if traci.isLoaded():
        traci.load(cmd[1:])
    else:
        traci.start(cmd)
    traci.vehicle.add("lead", "r", typeID="c", departPos="100", departSpeed="0")
    traci.vehicle.add("fol", "r", typeID="c", departPos="50", departSpeed="12")
    traci.vehicle.setSpeedMode("fol", 0)
    hit = False
    for _ in range(100):
        traci.vehicle.setSpeed("lead", 0)
        if "fol" in traci.vehicle.getIDList():
            traci.vehicle.setSpeed("fol", 12)
        traci.simulationStep()
        hit = hit or bool(traci.simulation.getCollisions())
    assert hit


def test_a2_cutin_ignores_priority():
    cfg = build_a2_net(single_bin=(1.0, 0.5, 0.5))
    rows = [run_cutin_episode(cfg, "cutin_0", s, threshold_m=28.0) for s in range(12)]
    assert sum(r["entered"] for r in rows) >= 3


def test_a2_default_vign_does_not_ignore_moving_bus():
    cfg = build_a2_net(single_bin=(1.0, 0.5, 0.5), v_ign=0.0)
    rows = [run_cutin_episode(cfg, "cutin_0", s, threshold_m=28.0) for s in range(12)]
    assert sum(r["entered"] for r in rows) == 0


def test_a2_junction_collision_recorded():
    cfg = build_a2_net(single_bin=(1.0, 0.1, 1.0))
    rows = [run_cutin_episode(cfg, "cutin_0", s, bus_speedmode=0, threshold_m=28.0)
            for s in range(30)]
    assert any(r["collision"] for r in rows)


def test_s4_parked_taxi_blocks_bus():
    rows = run_s4(build_s4_net(stop_duration=40))
    parked = [r for r in rows if r[5]]
    assert not any(r[3] > r[1] for r in parked)                # 버스가 추월하지 못함
    assert any(r[4] < 0.1 for r in parked)                     # 정차 중 버스도 섬
    assert rows[-1][4] > 5.0                                   # 택시가 떠난 뒤 통과


def test_a3_p1_waits_for_fast_bus_p2_accepts_gap():
    cfg = build_a3_net()
    v, T = 2.0, 10                       # v_low(1.5)보다 빠른 버스, 시간 간격 10 s
    p1 = run_crossing_episode(cfg, "ped_p1", v, bus_dist=v * T)
    p2 = run_crossing_episode(cfg, "ped_p2", v, bus_dist=v * T)
    assert not p1["crossed_before_bus"] and p2["crossed_before_bus"]


def test_a3_vehicle_pedestrian_collision_recorded():
    cfg = build_a3_net()
    rows = [run_crossing_episode(cfg, "ped_force", 10.0, bus_dist=10.0 * T, bus_speedmode=0)
            for T in (5, 6, 7)]
    assert any(r["collision"] for r in rows)


def test_pet_function():
    from traffic.metrics import pet, passenger_weighted_delay
    assert pet((0, 5), (7, 9)) == 2          # 보행자가 먼저 나가고 2 s 뒤 버스 진입
    assert pet((6, 10), (1, 4)) == 2         # 버스가 먼저 지나간 뒤 2 s 후 보행자 진입
    assert pet((3, 8), (6, 9)) == 0          # 겹침
    assert pet(None, (1, 2)) is None
    assert passenger_weighted_delay([2.0, 1.0], [10, 30]) == 50.0


def test_metrics_in_crossing_episodes():
    from traffic.metrics import MetricsRecorder
    cfg = build_a3_net()
    ok = run_crossing_episode(cfg, "ped_p2", 5.0, bus_dist=5.0 * 10, recorder_cls=MetricsRecorder)
    assert ok["metrics"]["collisions"] == 0
    assert ok["metrics"]["pet"].get("p0") is not None and ok["metrics"]["pet"]["p0"] >= 0.0
    bad = run_crossing_episode(cfg, "ped_force", 10.0, bus_dist=60.0, bus_speedmode=0,
                               recorder_cls=MetricsRecorder)
    assert bad["metrics"]["collisions"] >= 1


def test_runner_realizes_decision_and_calls_hook_before_update(a1):
    """공통 루프에 버스 훅(시험용 버스)을 끼워도 A1 결과가 같고, 훅이 갱신보다 먼저 불린다."""
    import random
    from traffic.runner import run
    from traffic.scenarios import ScriptedBus
    from traffic.yield_logic import YieldManager, DEFAULT_TYPES
    from traffic.metrics import MetricsRecorder
    from traffic.sumo_conn import sumo_cmd as _cmd
    cmd = _cmd(a1, seed=4)
    traci.load(cmd[1:]) if traci.isLoaded() else traci.start(cmd)
    bus = ScriptedBus("bus0")
    bus.add(traci)
    traci.vehicle.add("f0", "r0", typeID="yielder", departPos="21.5", departLane="1", departSpeed="9")
    mgr = YieldManager(traci, "bus0", rng=random.Random(4), types=DEFAULT_TYPES,
                       stop_exit_pos=("E0", 100.0), exit_max_dist=60.0)
    seen, lanes = [], set()
    mgr_update = mgr.update
    mgr.update = lambda t: (seen.append(("update", t)), mgr_update(t))[1]
    rec = MetricsRecorder(traci, "bus0")
    run(traci, 30.0, "bus0", bus_hook=lambda tc, t: (seen.append(("hook", t)), bus.act(tc, t))[1],
        yield_mgr=mgr, metrics=rec,
        on_step=lambda t: lanes.add(traci.vehicle.getLaneIndex("bus0")))
    assert seen[0][0] == "hook" and seen[1][0] == "update"
    inter = list(mgr.inter.values()) + [type("L", (), {"yields": x["yield"]}) for x in mgr.log]
    assert len(inter) == 1
    assert (1 in lanes) == bool(inter[0].yields)   # 결정대로 실현
    s = rec.summary()
    assert s["weighted_delay_person_s"] > 0 and s["collisions"] == 0


@pytest.fixture(scope="module")
def a1_mixed_models(a1):
    """배경 차량처럼 이름이 car_로 시작하고 추종 모델이 제각각인 후행차를 추가한 A1 도로."""
    import shutil
    src = os.path.dirname(a1)
    dst = os.path.join(NET_DIR, "a1_mixed")
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(src, dst)
    rou = os.path.join(dst, "r.rou.xml")
    text = open(rou, encoding="utf-8").read()
    base = ('length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" '
            'speedFactor="1" speedDev="0" actionStepLength="0.5"')
    types = "".join(f'<vType id="car_{k}" {base} carFollowModel="{m}"/>\n'
                    for k, m in (("krauss", "Krauss"), ("idm", "IDM"), ("eidm", "EIDM"), ("acc", "ACC")))
    _write(rou, text.replace("<route id", types + "<route id", 1))
    return os.path.join(dst, "a1.sumocfg")


MIX = dict(type_mix={"yielder": 0.5, "nonyielder": 0.5}, mix_vtype_prefixes=("car_",))


@pytest.mark.parametrize("model", ["krauss", "idm", "eidm", "acc"])
def test_a1_personality_attaches_to_any_driving_model(a1_mixed_models, model):
    rows = [run_a1_episode(a1_mixed_models, f"car_{model}", s, mgr_kwargs=MIX) for s in range(12)]
    assert {r["assigned_type"] for r in rows} == {"yielder", "nonyielder"}
    assert all(bool(r["decision"]) == r["changed"] for r in rows)


def test_a1_mix_off_leaves_unnamed_vehicles_alone(a1_mixed_models):
    r = run_a1_episode(a1_mixed_models, "car_idm", 0)
    assert r["assigned_type"] == "" and r["decision"] is None and r["p_yield"] is None


def test_a1_mix_rejects_bad_config():
    from traffic.yield_logic import YieldManager, DEFAULT_TYPES
    with pytest.raises(ValueError):
        YieldManager(None, "bus0", types=DEFAULT_TYPES, type_mix={"nobody": 1.0}, mix_vtype_prefixes=("car_",))
    with pytest.raises(ValueError):
        YieldManager(None, "bus0", types=DEFAULT_TYPES, type_mix={"yielder": 1.0})
