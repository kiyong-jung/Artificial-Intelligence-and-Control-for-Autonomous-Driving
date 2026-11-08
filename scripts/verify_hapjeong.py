"""합정역 실제 도로망에서 A2(S5, S4)와 A3(S6)가 합성 도로와 같은 방향으로 실현되는지 확인한다.

배경 교통 없이 버스 역할의 시험 차량 한 대만 둔다. 결과는 results/traffic_verification/hapjeong.json.
"""
import json
import os
import random
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT)
from traffic.cutin import cutin_vtype_xml, parking_area_xml  # noqa: E402
from traffic.pedestrian import ped_vtypes_xml  # noqa: E402
from traffic.scenarios import NET_DIR, _write  # noqa: E402
from traffic.sumo_conn import traci, sumo_cmd  # noqa: E402

NET = os.path.abspath(os.path.join(ROOT, "maps", "hapjeong", "source", "network.net.xml"))
D = os.path.join(NET_DIR, "hapjeong")
BUS = '<vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" tau="1.0" sigma="0"/>'


def nonpriority_s6_net():
    """실제 도로망의 S6 횡단보도는 priority=true라 P1/P2가 의미 없다. 계획대로 비우선으로 다시 만든 복사본."""
    import subprocess
    from traffic.sumo_conn import checkBinary
    os.makedirs(D, exist_ok=True)
    _write(os.path.join(D, "c.con.xml"), '<connections><crossing node="S6_wausanro" '
           'edges="-R112.88 R112" priority="false"/></connections>')
    out = os.path.join(D, "np.net.xml")
    subprocess.run([checkBinary("netconvert"), "-s", NET, "-x", os.path.join(D, "c.con.xml"),
                    "-o", out, "--no-warnings"], check=True, capture_output=True)
    return out


BG = False            # True면 baseline 배경 교통 속에서 실행(WARM초 예열 뒤 시나리오 시작)
WARM = 300.0


def cfg_with(routes_xml, add_xml=None, net=None):
    if BG:
        return bg_cfg(add_xml or "", BUS + routes_xml)
    _write(os.path.join(D, "r.rou.xml"), f"<routes>{BUS}{routes_xml}</routes>")
    add = ""
    if add_xml:
        _write(os.path.join(D, "a.add.xml"), f"<additional>{add_xml}</additional>")
        add = '<additional-files value="a.add.xml"/>'
    _write(os.path.join(D, "h.sumocfg"),
           f'<configuration><input><net-file value="{net or NET}"/><route-files value="r.rou.xml"/>{add}'
           '</input><time><begin value="0"/><end value="200"/></time></configuration>')
    return os.path.join(D, "h.sumocfg")


def start(cfg, seed=0):
    cmd = sumo_cmd(cfg, seed=seed)
    traci.load(cmd[1:]) if traci.isLoaded() else traci.start(cmd)
    if BG:
        for _ in range(int(WARM / 0.1)):
            traci.simulationStep()


def s5(p, n=30):
    """독막로(R71)를 달리는 버스 앞에 독막로3길(G3)이 신호 없이 합류하는 X_dongmak3."""
    vt = cutin_vtype_xml("cutin", bins=[(p, 0.5, 0.5, 1.0)], v_ign=15.0)
    cfg = cfg_with(vt + '<route id="rm" edges="R71 R71.135"/><route id="ra" edges="G3 R71.135"/>')
    ent, coll, dists = 0, 0, []
    for s in range(n):
        start(cfg, s)
        rng = random.Random(s)
        t_dep = rng.uniform(2.0, 4.5)
        traci.vehicle.add("bus0", "rm", typeID="bus_test", departPos="0", departLane="1", departSpeed="11")
        added, entered, c = False, False, False
        for k in range(1, 450):
            t = k * 0.1
            if not added and t >= t_dep:
                traci.vehicle.add("c0", "ra", typeID="cutin_0", departPos="0", departSpeed="0")
                added = True
            if "bus0" in traci.vehicle.getIDList():
                traci.vehicle.setSpeed("bus0", 11.0)
            traci.simulationStep()
            c = c or bool(traci.simulation.getCollisions())
            if added and "c0" in traci.vehicle.getIDList() and not entered:
                r = traci.vehicle.getRoadID("c0")
                if r.startswith(":X_dongmak3") or r == "R71.135":
                    if "bus0" in traci.vehicle.getIDList() and traci.vehicle.getRoadID("bus0") == "R71":
                        d = 130.459 - traci.vehicle.getLanePosition("bus0")
                        dists.append(d)
                        entered = 0 <= d <= 28.0
                    break
        ent += entered
        coll += c
    return {"p": p, "entry_rate": ent / n, "collision_rate": coll / n,
            "median_bus_dist_at_entry": float(np.median(dists)) if dists else None}


def s4():
    """R69 안쪽 차로에 onRoad 주차 구역을 두고, 차로 변경을 끈 버스가 택시 뒤에서 서는지 본다."""
    cfg = cfg_with('<vType id="taxi" vClass="taxi" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0"/>'
                   '<route id="r" edges="R69"/>'
                   f'<vehicle id="taxi0" type="taxi" route="r" depart="{WARM if BG else 0}" departLane="1" departSpeed="max">'
                   '<stop parkingArea="pa0" duration="30"/></vehicle>'
                   f'<vehicle id="bus0" type="bus_test" route="r" depart="{(WARM if BG else 0) + 3}" departLane="1" departSpeed="max"/>',
                   parking_area_xml("pa0", "R69_1", 30, 40, 1))
    start(cfg)
    stopped_behind, passed, lcset = 0, False, False
    for k in range(1, 700):
        traci.simulationStep()
        ids = traci.vehicle.getIDList()
        if "bus0" in ids and not lcset:
            traci.vehicle.setLaneChangeMode("bus0", 0)     # 차로 변경으로 피하지 않게 한다
            lcset = True
        if "bus0" in ids and "taxi0" in ids and traci.vehicle.isStoppedParking("taxi0"):
            stopped_behind += traci.vehicle.getSpeed("bus0") < 0.1
            passed = passed or traci.vehicle.getLanePosition("bus0") > traci.vehicle.getLanePosition("taxi0")
        if "bus0" not in ids and k > 50:
            break
    return {"bus_stopped_behind_taxi_s": stopped_behind * 0.1, "bus_passed_parked_taxi": passed}


def s6(ptype, v, T, net=None, ):
    pv = ped_vtypes_xml(1.5, 0.75)
    cfg = cfg_with(pv + '<route id="r" edges="R112 R112.221"/>'
                   f'<person id="p0" type="{ptype}" depart="{WARM if BG else 0}" departPos="210">'
                   '<walk from="R112" to="-R112" arrivalPos="40"/></person>', net=net)
    start(cfg)
    L = 216.725
    traci.vehicle.add("bus0", "r", typeID="bus_test", departPos=str(max(0.0, L - v * T - 0.5)),
                      departLane="1", departSpeed=str(v))
    enter, passed_t = None, None
    for k in range(1, 900):
        if "bus0" in traci.vehicle.getIDList():
            traci.vehicle.setSpeed("bus0", v)
        traci.simulationStep()
        t = k * 0.1
        if passed_t is None and "bus0" in traci.vehicle.getIDList() and traci.vehicle.getRoadID("bus0") != "R112":
            passed_t = t
        if enter is None and "p0" in traci.person.getIDList() and traci.person.getRoadID("p0") == ":S6_wausanro_c0":
            enter = t
        if enter and passed_t and t > max(enter, passed_t) + 3:
            break
    return enter is not None and (passed_t is None or enter < passed_t)


def a1(n=100):
    """독막로 이후 4차로 R43(382 m, 보행자 차로 0)에서 A1 양보율·누출. 버스 차로 1 -> 목표 차로 2."""
    from traffic.scenarios import run_a1_episode
    types = """<vType id="yielder" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" speedFactor="1" speedDev="0" actionStepLength="0.5"/>
<vType id="nonyielder" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" speedFactor="1" speedDev="0" actionStepLength="0.5"/>
<route id="r0" edges="R43"/>"""
    cfg = cfg_with(types)
    out = {}
    for t in ("yielder", "nonyielder"):
        rows = [run_a1_episode(cfg, t, s, lanes=(1, 2), edge="R43") for s in range(n)]
        leak = [run_a1_episode(cfg, t, 9000 + s, lanes=(1, 2), edge="R43", p_override=0.0) for s in range(n // 2)]
        out[t] = {"n": n, "mean_set_p": float(np.mean([r["p_yield"] for r in rows])),
                  "realized": float(np.mean([r["changed"] for r in rows])),
                  "agree": float(np.mean([bool(r["decision"]) == r["changed"] for r in rows])),
                  "leak_p0": float(np.mean([r["changed"] for r in leak]))}
    return out


def bg_cfg(extra_add, extra_routes):
    """합정역 baseline 배경 교통(900대/시간 + 보행자)이 있는 설정에 시험용 차종과 경로를 더한다."""
    sys.path.insert(0, ROOT)
    from env import road_builder
    from env.road_config import ROAD, EGO, TRAFFIC, configure_map
    configure_map("baseline", 0, None, "hapjeong")
    base = os.path.abspath(os.path.join(NET_DIR, "hap_bg"))
    os.makedirs(base, exist_ok=True)
    road_builder.build(ROAD, EGO, TRAFFIC, base)
    _write(os.path.join(base, "extra.add.xml"), f"<additional>{extra_add}</additional>")
    _write(os.path.join(base, "extra.rou.xml"), f"<routes>{extra_routes}</routes>")
    txt = open(os.path.join(base, "scenario.sumocfg"), encoding="utf-8").read()
    txt = txt.replace("pedestrians.rou.xml\"", "pedestrians.rou.xml,extra.rou.xml\"").replace(
        "stops.add.xml\"", "stops.add.xml,extra.add.xml\"")
    _write(os.path.join(base, "bg.sumocfg"), txt)
    return os.path.join(base, "bg.sumocfg")


def a1_bg(n=40):
    from traffic.scenarios import run_a1_episode
    add = BUS + """<vType id="yielder" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" speedFactor="1" speedDev="0" actionStepLength="0.5"/>
<vType id="nonyielder" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" speedFactor="1" speedDev="0" actionStepLength="0.5"/>"""
    cfg = bg_cfg(add, '<route id="r0" edges="R43"/>')
    out = {}
    for t in ("yielder", "nonyielder"):
        rows = [run_a1_episode(cfg, t, s, lanes=(1, 2), edge="R43", warmup=300.0) for s in range(n)]
        out[t] = {"n": n, "mean_set_p": float(np.mean([r["p_yield"] for r in rows])),
                  "realized": float(np.mean([r["changed"] for r in rows])),
                  "agree": float(np.mean([bool(r["decision"]) == r["changed"] for r in rows]))}
    return out


def main():
    res = {"A1_R43": a1(), "A1_R43_with_background": a1_bg(int(os.environ.get("A1_BG_N", 200))), "S5": [s5(p) for p in (0.0, 0.3, 1.0)], "S4": s4(),
           "S6": {pt: {f"v{v}_T{T}": s6(pt, v, T) for v, T in ((1.0, 8), (2.0, 10), (5.0, 10), (5.0, 3))}
                                      for pt in ("ped_p1", "ped_p2")}}
    global BG
    BG = True
    res["with_background"] = {
        "S5": [s5(p) for p in (0.0, 0.3, 1.0)], "S4": s4(),
        "S6": {pt: {f"v{v}_T{T}": s6(pt, v, T) for v, T in ((1.0, 8), (2.0, 10), (5.0, 10), (5.0, 3))}
               for pt in ("ped_p1", "ped_p2")}}
    BG = False
    out = os.path.join(ROOT, "results", "traffic_verification", "hapjeong.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(res, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(json.dumps(res, ensure_ascii=False, indent=1))
    traci.close()


if __name__ == "__main__":
    main()
