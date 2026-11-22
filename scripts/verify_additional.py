"""추가 행동 검증(S4 정차 차량, S5 골목 합류, A3 보행자, 결정성, 합정역 전체 실행).

    python scripts/verify_additional.py s4 s5 ped det hap        # 골라서 실행(인자 없으면 전부)

A1·S1은 배경 차량을 조종하지 않고 관찰만 하므로 예전 r1·s1·scope(양보율, 차로 고정, 소유권) 검증은 없다.

결과는 results/traffic_verification/additional.json 에 합쳐 쓰고, 항목마다 요약을 출력한다. 표본 수는 환경변수 ADD_N으로 줄이거나 늘린다.
"""
import collections
import json
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
from traffic.scenarios import traci, NET_DIR  # noqa: E402
from traffic.scenarios_a2 import (build_a2_net, build_s4_kind_net, run_cutin_episode, run_s4)  # noqa: E402
from traffic.sumo_conn import sumo_cmd  # noqa: E402

N = int(os.environ.get("ADD_N", 60))
OUT = os.path.join(os.path.dirname(__file__), "..", "results", "traffic_verification", "additional.json")
RESULT = {}


# ---------------------------------------------------------------------------------------------------------
def s4_synthetic(kind):
    cfg = build_s4_kind_net(kind, stop_duration=40)
    rows = run_s4(cfg, "stopped0")
    parked = [r for r in rows if r[5]]
    return {"kind": kind, "parked_s": round(len(parked) * 0.1, 1),
            "bus_passed_while_parked": any(r[3] > r[1] for r in parked),
            "bus_stopped_behind_s": round(sum(1 for r in parked if r[4] < 0.1) * 0.1, 1),
            "bus_resumed_after": rows[-1][4] > 5.0,
            "min_gap_to_stopped_m": round(min(r[1] - r[3] for r in parked), 2) if parked else None}


def check_s4():
    out = {"synthetic": [s4_synthetic(k) for k in ("taxi", "bus", "car")]}
    RESULT["s4"] = out
    return out


# ---------------------------------------------------------------------------------------------------------
def check_s5():
    from traffic.alleys import find_merge_alleys, flow_merges, read_net
    from traffic.hapjeong import HapjeongTraffic
    out = {}
    # (a) 합정역: 합류 골목 8곳, 흐름 수, 교차로 유형
    w = HapjeongTraffic(os.path.join(tempfile.gettempdir(), "hap_s5"), scenario="busy", seed=0)
    w.build()
    out["hapjeong_alleys"] = w.summary["cutin_alleys"]
    out["hapjeong_cutin_flows"] = w.summary["cutin_flows"]
    # (b) 합성 도로: 골목 위치(본선 위 거리)와 방향을 바꿔도 찾고, 끼어들기 vType이 듣는가
    syn = []
    for x in (150.0, 300.0, 600.0):
        for side in (-1, 1):
            cfg = build_a2_net(single_bin=(1.0, 0.5, 0.5), alley_x=x, alley_side=side, name=f"a2_{int(x)}_{side}")
            net = read_net(os.path.join(os.path.dirname(cfg), "net.net.xml"))
            al = find_merge_alleys(net, ["main_in", "main_out"])
            flow_ok = flow_merges(net, {"from": "alley", "to": "main_out"}, al)
            rows = [run_cutin_episode(cfg, "cutin_0", s, threshold_m=28.0, route_len=x) for s in range(16)]
            syn.append({"alley_x": x, "side": side, "found": [(a.junction, a.feeder, a.junction_type) for a in al],
                        "flow_detected": flow_ok, "entered": sum(r["entered"] for r in rows), "n": len(rows),
                        "collisions": sum(r["collision"] for r in rows)})
    out["synthetic"] = syn
    # (c) 진입 판정이 상호작용당 한 번인가: SUMO 기본(무시 확률 p) vs 차량당 한 번(0/1 vType + 비율)
    cap_cfg = build_a2_net(single_bin=(1.0, 0.5, 0.5), name="a2_cap")
    cap = sum(run_cutin_episode(cap_cfg, "cutin_0", s, threshold_m=28.0)["entered"] for s in range(60)) / 60
    native = {}
    for p in (0.1, 0.3, 0.5):
        cfg = build_a2_net(single_bin=(p, 0.5, 0.5), name="a2_n")
        native[p] = sum(run_cutin_episode(cfg, "cutin_0", s, threshold_m=28.0)["entered"] for s in range(60)) / 60
    once = {}
    for w_ in (0.1, 0.3, 0.5):
        cfg = build_a2_net(bins=[(1.0, 0.5, 0.5, w_), (0.0, 0.5, 0.5, 1 - w_)], name="a2_o")
        rng = random.Random(7)
        ent = 0
        for s in range(60):
            typ = "cutin_0" if rng.random() < w_ else "cutin_1"
            ent += run_cutin_episode(cfg, typ, s, threshold_m=28.0)["entered"]
        once[w_] = ent / 60
    out["decision_once"] = {"entered_when_p=1": cap, "native_by_p": native, "once_by_weight": once,
                            "expected_native_if_once": {p: round(p * cap, 3) for p in native},
                            "expected_once": {w_: round(w_ * cap, 3) for w_ in once}}
    RESULT["s5"] = out
    return out


# ---------------------------------------------------------------------------------------------------------
def hap_run(seed, seconds, stopped_args=(), triggers=None, warmup=120.0, scenario="busy"):
    import run_hapjeong_scenario as RS
    from maps import load_map
    from traffic.alleys import read_net
    from traffic.hapjeong import HapjeongTraffic, stopped_from_arg
    from traffic.metrics import MetricsRecorder
    from traffic.test_bus import TestBusPilot
    stopped = [stopped_from_arg(a, warmup, i) for i, a in enumerate(stopped_args)]
    world = HapjeongTraffic(os.path.join(tempfile.gettempdir(), f"hap_ver_{seed}"), scenario=scenario, seed=seed,
                            stopped=stopped)
    world.build()
    tc = world.start(warmup=warmup)
    m = load_map("hapjeong")
    bus = TestBusPilot(tc, read_net(os.path.join(world.out_dir, "network.net.xml")), m["routes"]["bus_loop"],
                       RS.map_stops(m))
    rec = MetricsRecorder(tc, "bus0")
    stats = {"teleports": 0, "stopped_parked_s": collections.Counter()}

    def on_step(t):
        stats["teleports"] += len(tc.simulation.getStartingTeleportIDList())
        for item in world.summary["stopped"]:
            if item["id"] in tc.vehicle.getIDList() and tc.vehicle.isStoppedParking(item["id"]):
                stats["stopped_parked_s"][item["id"]] += 0.1

    mgr, t_end = world.run(tc.simulation.getTime() + seconds, bus_id="bus0", bus_hook=bus.tick, metrics=rec,
                           on_step=on_step, triggers=triggers, ped_defiance=True)
    out = {"seed": seed, "a1": dict(collections.Counter(r["outcome"] for r in mgr.log)), "a1_n": len(mgr.log),
           "s1": world.stop.summary(), "ped": world.ped.summary(), "collisions": rec.summary()["collisions"],
           "teleports": stats["teleports"],
           "laps": len(bus.lap_times), "stopped": dict(stats["stopped_parked_s"]),
           "ped_crossings_found": len(world.ped.incoming), "fingerprint": [
               (r["follower"], r["outcome"], round(r["t_start"], 1)) for r in mgr.log][:30]
           + [(r["follower"], r["outcome"], round(r["t_decide"], 1)) for r in world.stop.log][:10]
           + [(r["ped"], r["cross"], round(r["t"], 1)) for r in world.ped.log][:20]}
    world.close()
    return out


def check_hap():
    stopped = ("kind=taxi,stop=합정역7번출구,depart=20,duration=40", "kind=car,stop=성산중학교,depart=60,duration=40",
               "kind=bus,edge=R112,lane=1,pos=60,depart=100,duration=40")
    runs = [hap_run(s, 600, stopped) for s in range(int(os.environ.get("HAP_SEEDS", 3)))]
    RESULT["hapjeong"] = runs
    return runs


def check_det():
    a = hap_run(0, 240)
    b = hap_run(0, 240)
    out = {"identical": a == b, "a1_n": a["a1_n"], "fingerprint_len": len(a["fingerprint"])}
    RESULT["determinism"] = out
    return out


def ped_episode(v, T, seed, vtype="bus_test", crossing_model=None):
    """합성 신호 없는 횡단보도: 차량이 v m/s로 T초 뒤 도착하는 시점에 보행자가 건너기를 판정한다."""
    from traffic.metrics import MetricsRecorder
    from traffic.ped_defiance import PedDefianceManager
    from traffic.pedestrian import BASE, CROSSING, build_a3_net, ped_vtypes_xml
    from traffic.scenarios import _write
    d = os.path.join(NET_DIR, "a3")
    cm = f' carFollowModel="{crossing_model}"' if crossing_model else ""
    _write(os.path.join(d, "r.rou.xml"), f"""<routes>
<vType id="bus_test" vClass="bus" length="12" width="2.5" height="3.2" accel="1.2" decel="4.0" tau="1.0" sigma="0"/>
<vType id="car_t" length="4.5" width="1.8" height="1.5" accel="2.6" decel="4.5" tau="1.0" sigma="0"{cm}/>
<vType id="ped_base" {BASE}/>{ped_vtypes_xml(1.5, 0.75)}
<route id="r_main" edges="main_in main_out"/>
<person id="p0" type="ped_base" depart="0" departPos="289"><walk from="main_in" to="rev_out" arrivalPos="20"/></person>
</routes>""")
    cmd = sumo_cmd(build_a3_net(), seed=seed)
    traci.load(cmd[1:]) if traci.isLoaded() else traci.start(cmd)
    traci.vehicle.add("v0", "r_main", typeID=vtype, departPos=str(292.8 - v * T), departLane="1", departSpeed=str(v))
    mgr = PedDefianceManager(traci, "v0", rng=random.Random(seed))
    rec = MetricsRecorder(traci, "v0", crossing_edge=CROSSING, ped_mgr=mgr)
    enter = veh_out = None
    t, k = 0.0, 0
    while t < 50:
        if "v0" in traci.vehicle.getIDList():
            traci.vehicle.setSpeed("v0", v)          # 일정 속도로 접근(안전 규칙이 막을 때만 줄인다)
        mgr.update(t)
        traci.simulationStep()
        k += 1
        t = round(k * 0.1, 6)
        rec.step(t)
        if veh_out is None and "v0" in traci.vehicle.getIDList() and traci.vehicle.getRoadID("v0") != "main_in":
            veh_out = t
        if enter is None and "p0" in traci.person.getIDList() and traci.person.getRoadID("p0") == CROSSING:
            enter = t
        if veh_out is not None and (enter is not None or t > veh_out + 20):
            break
    s = rec.summary()
    return {"crossed_first": enter is not None and (veh_out is None or enter < veh_out), "collisions": s["collisions"],
            "harsh": s["ped_defiance_harsh_decel_events"], "emergency": s["ped_defiance_emergency_decel_events"],
            "max_decel": s["max_induced_decel"]}


def check_ped():
    out = {}
    for label, vt, model in (("bus", "bus_test", None), ("car_krauss", "car_t", "Krauss"), ("car_idm", "car_t", "IDM"),
                             ("car_eidm", "car_t", "EIDM"), ("car_acc", "car_t", "ACC")):
        rows = {}
        for T in (3, 6, 12):
            r = [ped_episode(6.0, T, s, vt, model) for s in range(max(8, N // 4))]
            rows[f"T={T}s"] = {"n": len(r), "crossed_first": sum(x["crossed_first"] for x in r),
                               "collisions": sum(x["collisions"] for x in r),
                               "harsh_decel_events": sum(x["harsh"] for x in r),
                               "emergency_decel_events": sum(x["emergency"] for x in r)}
        out[label] = rows
    RESULT["ped_defiance"] = out
    return out


CHECKS = {"ped": check_ped, "s4": check_s4, "s5": check_s5, "det": check_det, "hap": check_hap}


def main():
    names = sys.argv[1:] or list(CHECKS)
    if os.path.exists(OUT):
        RESULT.update(json.load(open(OUT, encoding="utf-8")))
    for n in names:
        print(f"== {n} ==", flush=True)
        r = CHECKS[n]()
        print(json.dumps(r, ensure_ascii=False, indent=1, default=str), flush=True)
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        json.dump(RESULT, open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=str)
    try:
        traci.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
