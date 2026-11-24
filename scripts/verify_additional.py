"""추가 행동 알고리즘 검증(R1 속도 조절, R2/S1 정차 버스 뒤 차량, 소유권·복구, S4 정차 차량, S5 골목 합류, 결정성, 합정역 전체 실행).

    python scripts/verify_additional.py r1 s1 scope s4 s5 det hap        # 골라서 실행(인자 없으면 전부)

결과는 results/traffic_verification/additional.json 에 합쳐 쓰고, 항목마다 요약을 출력한다. 표본 수는 환경변수 ADD_N으로 줄이거나 늘린다.
"""
import collections
import json
import math
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
from traffic.common import Ownership  # noqa: E402
from traffic.scenarios import build_a1_mixed_net, run_a1_episode, ScriptedBus, traci, NET_DIR  # noqa: E402
from traffic.scenarios_a2 import (build_a2_net, build_s4_kind_net, run_cutin_episode, run_s4)  # noqa: E402
from traffic.scenarios_s1 import MODELS, build_s1_net, run_s1_episode  # noqa: E402
from traffic.sumo_conn import sumo_cmd  # noqa: E402
from traffic.yield_logic import DEFAULT_TYPES, Trigger, YieldManager, Zone  # noqa: E402

N = int(os.environ.get("ADD_N", 60))
OUT = os.path.join(os.path.dirname(__file__), "..", "results", "traffic_verification", "additional.json")
MIX = dict(type_mix={"yielder": 0.5, "nonyielder": 0.5}, mix_vtype_prefixes=("car_",))
RESULT = {}


def z_score(k, expected, var):
    return (k - expected) / math.sqrt(var) if var > 0 else 0.0


# ---------------------------------------------------------------------------------------------------------
def check_r1():
    """4개 추종 모델 각각에서 R1 판정 비율(감속/유지/가속)이 설정과 일치하고, 판정이 실제 행동과 100% 일치하는가."""
    cfg = build_a1_mixed_net()
    out = {}
    for model in ("krauss", "idm", "eidm", "acc"):
        rows = [run_a1_episode(cfg, f"car_{model}", s, mgr_kwargs=MIX) for s in range(N)]
        rows = [r for r in rows if r["action"] is not None]
        cnt = collections.Counter(r["action"] for r in rows)
        # 행동은 투입 때 확정된다: 양보형 = 감속, 비양보형 = 유지 또는 가속(가속 확률 p_close). 그 집단 구성에서 기대값을 낸다
        e_y = sum(r["assigned_type"] == "yielder" for r in rows)
        e_c = sum((r["p_close"] or 0.0) for r in rows if r["assigned_type"] == "nonyielder")
        v_y = 1.0
        # 행동 일치: 감속(양보)이면 버스가 차선변경에 성공하고, 유지·가속이면 못 한다
        agree = sum((r["action"] == "yield") == r["changed"] for r in rows)
        out[model] = {"n": len(rows), "yield": cnt["yield"], "keep": cnt["keep"], "close": cnt["close"],
                      "expected_yield": round(e_y, 1), "expected_close": round(e_c, 1),
                      "expected_keep": round(len(rows) - e_y - e_c, 1),
                      "yield_z": round(z_score(cnt["yield"], e_y, v_y), 2),
                      "decision_matches_outcome": f"{agree}/{len(rows)}"}
    RESULT["r1"] = out
    return out


def check_s1():
    """R2: 기다림 판정 차량이 실제로 차선을 안 바꾸는가(누출), 목표 추월률이 실제로 나오는가."""
    cfg = build_s1_net()
    out = {"leak": {}, "rate": {}}
    for model, _ in MODELS:
        for lock in (True, False):
            rows = [run_s1_episode(cfg, model, s, p_overtake=0.0, lock_wait=lock) for s in range(max(10, N // 3))]
            ev = [r["event"] for r in rows if r["event"]]
            out["leak"][f"{model}/{'locked' if lock else 'unlocked'}"] = {
                "events": len(ev), "leaked": sum(e["outcome"] == "leaked" for e in ev),
                "waited": sum(e["outcome"] == "waited" for e in ev)}
        for p in (0.3, 0.7):
            rows = [run_s1_episode(cfg, model, 100 + s, p_overtake=p) for s in range(N)]
            ev = [r["event"] for r in rows if r["event"]]
            n_ov = sum(e["action"] == "overtake" for e in ev)
            out["rate"][f"{model}/p={p}"] = {
                "events": len(ev), "overtake_decisions": n_ov,
                "overtook": sum(e["outcome"] == "overtook" for e in ev),
                "blocked": sum(e["outcome"] == "blocked" for e in ev),
                "waited": sum(e["outcome"] == "waited" for e in ev),
                "leaked": sum(e["outcome"] == "leaked" for e in ev),
                "decision_rate": round(n_ov / max(1, len(ev)), 3),
                "decision_matches_outcome": sum((e["action"] == "overtake") == (e["outcome"] == "overtook")
                                                 for e in ev)}
    # 공격성 값에서 나온 확률로 뽑을 때(자연 상태): 평균 설정 확률과 실제 추월률
    nat = []
    for model, _ in MODELS:
        nat += [run_s1_episode(cfg, model, 200 + s) for s in range(N)]
    ev = [r["event"] for r in nat if r["event"]]
    out["natural"] = {"events": len(ev), "mean_p_overtake": round(sum(e["p"] for e in ev) / max(1, len(ev)), 3),
                      "overtake_decisions": sum(e["action"] == "overtake" for e in ev),
                      "overtook": sum(e["outcome"] == "overtook" for e in ev)}
    RESULT["s1"] = out
    return out


# ---------------------------------------------------------------------------------------------------------
class CallRecorder:
    """traci.vehicle의 제약 명령(changeLane, openGap, setAcceleration, setLaneChangeMode, setSpeedMode)이 누구에게 나갔는지 기록한다."""
    NAMES = ("changeLane", "openGap", "setAcceleration", "setLaneChangeMode", "setSpeedMode")

    def __init__(self, tc):
        self.tc, self.calls, self.on = tc, [], False
        self._orig = {}
        for n in self.NAMES:
            self._orig[n] = getattr(tc.vehicle, n)
            setattr(tc.vehicle, n, self._wrap(n))

    def _wrap(self, name):
        orig = self._orig[name]

        def f(vid, *a, **k):
            if self.on:
                self.calls.append((name, vid, a))
            return orig(vid, *a, **k)
        return f

    def restore(self):
        for n, o in self._orig.items():
            setattr(self.tc.vehicle, n, o)


def scope_episode(cfg, seed):
    """버스(차로 0)가 왼쪽 차로 1로 옮길 때: 대상(차로 1에서 가장 가까운 후행차 f0) 한 대만 제약을 받는가, 끝나면 복구되는가."""
    tc = traci
    cmd = sumo_cmd(cfg, seed=seed)
    tc.load(cmd[1:]) if tc.isLoaded() else tc.start(cmd)
    mgr = YieldManager(tc, "bus0", rng=random.Random(seed), types=DEFAULT_TYPES, stop_exit_pos=("E0", 100.0),
                       exit_max_dist=60.0, **MIX)
    bus = ScriptedBus("bus0")
    bus.add(tc)
    spec = [("f0", 1, 21.5), ("f1", 1, 5.0), ("g0", 0, 60.0), ("h0", 1, 150.0)]   # id, 차로, 위치
    for vid, lane, x in spec:
        tc.vehicle.add(vid, "r0", typeID="car_krauss", departPos=str(x), departLane=str(lane), departSpeed="9")
        tc.vehicle.setMaxSpeed(vid, 9.0)
    mgr.on_departed([v for v, _, _ in spec])
    tc.simulationStep()
    modes0 = {v: (tc.vehicle.getLaneChangeMode(v), tc.vehicle.getSpeedMode(v)) for v, _, _ in spec}
    rec = CallRecorder(tc)
    t, step, locked_ever, owned_ever = 0.0, 0, set(), {}
    while t < 30.0:
        mgr.on_departed(tc.simulation.getDepartedIDList())
        rec.on = True
        mgr.update(t)
        rec.on = False
        locked_ever |= set(mgr.lock._last)
        for v in mgr.inter:
            owned_ever[v] = mgr.owner.owner(v)
        bus.act(tc, t)
        tc.simulationStep()
        step += 1
        t = round(step * 0.1, 6)
    rec.restore()
    ids = set(tc.vehicle.getIDList())
    modes1 = {v: (tc.vehicle.getLaneChangeMode(v), tc.vehicle.getSpeedMode(v)) for v, _, _ in spec if v in ids}
    by_vid = collections.defaultdict(list)
    for name, vid, a in rec.calls:
        by_vid[vid].append((name, a))
    last_gap = [a for n, a in by_vid.get("f0", []) if n == "openGap"]
    last_cl = [a for n, a in by_vid.get("f0", []) if n == "changeLane"]
    return {"interacted": sorted({r["follower"] for r in mgr.log}), "constrained": sorted(by_vid),
            "lock_held_for": sorted(locked_ever), "owned": owned_ever,
            "modes_unchanged": all(modes0[v] == modes1[v] for v in modes1),
            "owner_after": {v: mgr.owner.owner(v) for v, _, _ in spec},
            "lock_after": sorted(mgr.lock._last), "inter_after": sorted(mgr.inter),
            "opengap_released": (not last_gap) or last_gap[-1][:4] == (-1, -1, -1, -1) or tuple(last_gap[-1][:1]) == (-1,),
            "lane_request_released": bool(last_cl) and last_cl[-1][1] == 0.0,
            "action": mgr.log[0]["action"] if mgr.log else None}


def check_scope():
    cfg = build_a1_mixed_net()
    rows = [scope_episode(cfg, s) for s in range(max(8, N // 4))]
    out = {"episodes": len(rows),
           "only_nearest_follower_interacts": all(r["interacted"] == ["f0"] for r in rows),
           "constraints_only_on_target": all(set(r["constrained"]) <= {"f0"} for r in rows),
           "lock_only_on_target": all(set(r["lock_held_for"]) <= {"f0"} for r in rows),
           "laneChangeMode_speedMode_unchanged": all(r["modes_unchanged"] for r in rows),
           "ownership_released": all(all(o is None for o in r["owner_after"].values()) for r in rows),
           "no_lock_left": all(not r["lock_after"] and not r["inter_after"] for r in rows),
           "opengap_released": all(r["opengap_released"] for r in rows if r["action"] == "yield"),
           "lane_request_released": all(r["lane_request_released"] for r in rows),
           "actions": dict(collections.Counter(r["action"] for r in rows))}
    RESULT["scope_recovery"] = out
    return out


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


def s4_to_r1(seed=0):
    """정차 차량이 버스 차로를 막으면 버스가 차선변경을 하고, 그때 목표 차로 후행차가 R1(속도 조절)로 처리되는가."""
    tc = traci
    car = ('<vType id="car_krauss" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" speedFactor="1" '
           'speedDev="0" actionStepLength="0.5" lcKeepRight="0"/>')
    cfg = build_s4_kind_net("car", stop_duration=120, lanes=2, stop_pos=200.0, name="s4r1", bus_depart=None,
                            extra_vtypes=car)
    tc.load(sumo_cmd(cfg, seed=seed)[1:]) if tc.isLoaded() else tc.start(sumo_cmd(cfg, seed=seed))
    own = Ownership()
    zone = Zone("E0", 100.0, 200.0)
    mgr = YieldManager(tc, "bus0", rng=random.Random(seed), types=DEFAULT_TYPES, owner=own,
                       triggers=[Trigger(+1, [zone]), Trigger(-1, [zone])], **MIX)
    # 대상 후행차는 car_krauss. 통과 시간 문제를 피하려고 처음부터 상류에 둔다
    tc.vehicle.add("bus0", "r0", typeID="bus_test", departLane="0", departPos="60", departSpeed="8")
    tc.vehicle.setMaxSpeed("bus0", 8.0)
    tc.vehicle.add("f0", "r0", typeID="car_krauss", departLane="1", departPos="40", departSpeed="8")
    tc.vehicle.setMaxSpeed("f0", 8.0)
    t, step, lane_req, signalled, t_sig = 0.0, 0, False, False, 0.0
    while t < 40.0:
        mgr.on_departed(tc.simulation.getDepartedIDList())
        if "bus0" in tc.vehicle.getIDList():
            pos = tc.vehicle.getLanePosition("bus0")
            if pos > 120 and not signalled:
                tc.vehicle.setSignals("bus0", 2)       # 정차 차량(300 m)을 피하려 왼쪽 방향지시등
                signalled, t_sig = True, t
            if signalled and t - t_sig > 3.0 and not lane_req:
                tc.vehicle.changeLane("bus0", 1, 10.0)
                lane_req = True
        mgr.update(t)
        tc.simulationStep()
        step += 1
        t = round(step * 0.1, 6)
    return {"signalled": signalled, "bus_lane_after": tc.vehicle.getLaneIndex("bus0"),
            "interactions": [(r["follower"], r["action"]) for r in mgr.log]}


def check_s4():
    out = {"synthetic": [s4_synthetic(k) for k in ("taxi", "bus", "car")], "r1_link": s4_to_r1()}
    RESULT["s4"] = out
    return out


# ---------------------------------------------------------------------------------------------------------
def check_s5():
    from traffic.alleys import find_merge_alleys, flow_merges, read_net
    from traffic.cutin import cutin_bins_once, default_bins, cutin_vtype_xml
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
    from traffic.hapjeong import HapjeongTraffic, stopped_from_arg
    from traffic.metrics import MetricsRecorder
    stopped = [stopped_from_arg(a, warmup, i) for i, a in enumerate(stopped_args)]
    world = HapjeongTraffic(os.path.join(tempfile.gettempdir(), f"hap_ver_{seed}"), scenario=scenario, seed=seed,
                            stopped=stopped)
    world.build()
    tc = world.start(warmup=warmup)
    m = load_map("hapjeong")
    loop = m["routes"]["bus_loop"]
    stops = [{"id": s["id"], "name": s["name"], "edge": s["lane"].rsplit("_", 1)[0], "startPos": s["startPos"],
              "lane_idx": int(s["lane"].rsplit("_", 1)[1])} for s in m["bus_stops"] if s["lane"].rsplit("_", 1)[0] in loop]
    bus = RS.LoopBus(tc, loop, stops)
    rec = MetricsRecorder(tc, "bus0")
    stats = {"teleports": 0, "owner_conflicts": 0, "stopped_parked_s": collections.Counter()}

    def on_step(t):
        stats["teleports"] += len(tc.simulation.getStartingTeleportIDList())
        both = set(world.mgr.inter) & ({world.stop.event.follower} if world.stop and world.stop.event else set())
        stats["owner_conflicts"] += len(both)
        for item in world.summary["stopped"]:
            if item["id"] in tc.vehicle.getIDList() and tc.vehicle.isStoppedParking(item["id"]):
                stats["stopped_parked_s"][item["id"]] += 0.1

    mgr, t_end = world.run(tc.simulation.getTime() + seconds, bus_id="bus0", bus_hook=bus.tick, metrics=rec,
                           on_step=on_step, triggers=triggers)
    out = {"seed": seed, "a1": dict(collections.Counter(r["action"] for r in mgr.log)), "a1_n": len(mgr.log),
           "s1": world.stop.summary(), "ped": world.ped.summary(), "collisions": rec.summary()["collisions"],
           "teleports": stats["teleports"], "owner_conflicts": stats["owner_conflicts"],
           "laps": bus.laps, "stopped": dict(stats["stopped_parked_s"]),
           "ped_crossings_found": len(world.ped.incoming), "fingerprint": [
               (r["follower"], r["action"], round(r["t_start"], 1)) for r in mgr.log][:30]
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


CHECKS = {"ped": check_ped, "r1": check_r1, "s1": check_s1, "scope": check_scope, "s4": check_s4, "s5": check_s5,
          "det": check_det, "hap": check_hap}


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
