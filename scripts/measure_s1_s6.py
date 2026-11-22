"""시험용 버스가 노선을 3바퀴 도는 동안 시나리오 S1~S6이 몇 번 일어나는지 센다(사건 단위, 바퀴별).

    python scripts/measure_s1_s6.py baseline 0 route   # 인자: 수요(quiet|baseline|busy) 시드(기본 0) 꼬리표

시험용 버스는 traffic/test_bus.py(R68 출발, 끊김 없는 바퀴, 정해진 차선 변경 창)로 달린다. 배경 교통은 수요와 SUMO 운전자
모델만으로 움직인다(버스 위치에 맞춰 넣거나 조종하는 차·사람이 없다). 결과는
results/traffic_verification/s1_s6_<수요>_<시드>_<꼬리표>.json 에 쓴다.

사건 정의
  S1  시험용 버스가 정류장에 서서 출발한 횟수(stops) / 그때 정차 버스 바로 뒤 후행차에 대한 S1 판정이 일어난 횟수(rear_events)
  S2  J82·J1 앞 창에서 왼쪽 차선변경을 시도한 횟수(attempts) / 그중 목표 차로 후행차 A1 상호작용이 있었던 횟수(a1)
  S3  s2 앞 창에서 오른쪽 차선변경을 시도한 횟수 / 같은 A1 상호작용 횟수
  S4  시험용 버스가 서는 정류장 150 m 앞까지 왔을 때, 그 정류장 차로의 정류장 30 m 앞~끝 20 m 뒤에 정차 중(stop)인 차가 있던
      정류장 수(바퀴·정류장마다 한 번)
  S5  골목에서 나온 차가 15초 안에 버스와 같은 도로의 앞 50 m에 들어온 횟수
  S6  신호 없는 횡단보도 50 m 앞에서 보행자가 건너고 있던 통과 횟수(버스 한 번 지날 때 한 번)
"""
import collections
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
import run_hapjeong_scenario as RS  # noqa: E402
from maps import load_map  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.presence import PresenceMonitor  # noqa: E402
from traffic.signal_areas import forbidden_intervals, is_signalized  # noqa: E402
from traffic.test_bus import STOP_AT, TestBusPilot  # noqa: E402

SCEN = sys.argv[1] if len(sys.argv) > 1 else "baseline"
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 0
TAG = sys.argv[3] if len(sys.argv) > 3 else ""          # 결과 파일 이름 꼬리표
OLD_PED = "old" in TAG                                  # 꼬리표에 old가 있으면 보행자 수정 전 설정으로 잰다
PED_X = 4 if "hiped" in TAG else 1                      # 꼬리표에 hiped가 있으면 횡단보도 보행자를 4배로 늘린다
PED_DEFIANCE = "pedsumo" in TAG or OLD_PED              # 꼬리표에 pedsumo가 있으면 PedSUMO 방식 건너기 판정을 켠다
LAPS, T_MAX = int(os.environ.get("MEASURE_LAPS", "3")), float(os.environ.get("MEASURE_TMAX", "2600"))
CUT_WINDOW, NEAR = 15.0, 50.0
S6_CROSS, S6_EDGE = ":S6_wausanro_c", "R112.112"      # 횡단보도는 R112.112의 끝(116 m)에 있다. 예전 값 R112는 횡단보도 116 m 전이라 건너는 사람을 놓쳤다
S6_APPROACH = ["-R112_0", "-R112_1", "-R112_2", "R112.112_0", "R112.112_1", "R112.112_2"]   # 횡단보도로 들어오는 차로
S6_QUEUE_M = 40.0                                       # 횡단보도 앞 이 거리 안에 서 있는 차를 대기 차로 센다
S4_APPROACH_M, S4_BEFORE_M, S4_AFTER_M = 150.0, 30.0, 20.0


class Done(Exception):
    pass


def rewrite_persons(path, k, seed):
    """보행자 경로 파일을 고친다. k=0이면 보행자를 모두 빼고, k>1이면 신호 없는 횡단보도를 건너는 보행자(s6ped*)를 k배로
    늘린다(사본은 출발 시각을 0~40초 흩뜨린다). 출발 시각 순으로 다시 쓴다."""
    import copy
    import random
    import xml.etree.ElementTree as ET
    rng = random.Random(seed + 77)
    tree = ET.parse(path)
    root = tree.getroot()
    persons = list(root.iter("person"))
    for p in persons:
        root.remove(p)
    if k == 0:
        persons = []
    for p in [p for p in persons if p.get("id").startswith("s6ped")]:
        for j in range(1, k):
            c = copy.deepcopy(p)
            c.set("id", f"{p.get('id')}x{j}")
            c.set("depart", f"{float(p.get('depart')) + rng.uniform(0, 40):.2f}")
            persons.append(c)
    root.extend(sorted(persons, key=lambda p: float(p.get("depart"))))
    tree.write(path, encoding="utf-8", xml_declaration=True)


def main():
    out_dir = os.path.join(tempfile.gettempdir(), f"occ_{SCEN}_{SEED}{'_' + TAG if TAG else ''}")
    world = HapjeongTraffic(out_dir, scenario=SCEN, seed=SEED, bus_traffic=True,
                            right_on_red="noror" not in TAG, interior_share=0.0 if "nointer" in TAG else 0.3)
    world.build()
    if PED_X > 1 or "noped" in TAG:                     # 꼬리표에 noped가 있으면 배경 보행자를 모두 뺀다(J82 정지 원인 가리기)
        rewrite_persons(os.path.join(out_dir, "pedestrians.rou.xml"), 0 if "noped" in TAG else PED_X, SEED)
    tc = world.start(warmup=120)
    m = load_map("hapjeong")
    loop = m["routes"]["bus_loop"]
    stops = RS.map_stops(m)
    bus = TestBusPilot(tc, read_net(os.path.join(out_dir, "network.net.xml")), loop, stops)
    feeders = {a.feeder for a in world.alleys}
    ev = {k: collections.Counter() for k in ("S1_stops", "S2_attempts", "S2_done", "S2_a1", "S3_attempts", "S3_done",
                                             "S3_a1", "S4", "S5", "S6", "other_attempts", "S2_J82_attempts", "S2_J82_a1",
                                             "S2_J1_attempts", "S2_J1_a1")}
    served = [s for s in stops if s["id"] in STOP_AT]
    s4 = {"seen": set(), "events": []}
    net = read_net(os.path.join(out_dir, "network.net.xml"))
    presence = PresenceMonitor(tc, net, "bus0")
    forbidden = forbidden_intervals(net, bus.lap_edges)
    sig = {"seen": set(), "in_area": [], "stops": collections.defaultdict(list)}
    WIN_JUNCTION = {"R111b": "J82", "R42": "J1"}         # S2 창 끝 도로 -> 좌회전 교차로
    st = {"prev_phase": "idle", "attempt": None, "n_a1": 0, "stopped": False, "alley_seen": {}, "cut": set(),
          "s6_pass": set(), "s6_now": False, "stop_ev_n": 0, "t0": None}

    def lap():
        return bus.lap

    stop_edges = {s["edge"] for s in stops}
    timing = {}                                        # 바퀴 -> 시간 구성
    bw = {"cur": 0.0, "long": [], "s3_trace": []}      # 시험용 버스가 정류장 밖에서 15초 넘게 선 곳, S3 시도 중 궤적

    def on_step(t):
        ids = tc.vehicle.getIDList()
        if "bus0" in ids:
            st["t0"] = st["t0"] if st["t0"] is not None else t
            road, idx = tc.vehicle.getRoadID("bus0"), tc.vehicle.getLaneIndex("bus0")
            pos = tc.vehicle.getLanePosition("bus0")
            # 정류장 밖에서 오래 서 있는 곳(한 바퀴가 길어지는 원인 찾기)
            stopstate = tc.vehicle.getStopState("bus0")
            # 바퀴 시간을 나눠 센다(0.1초 단위): 정류장 정차 / 적색 대기 / 그 밖의 정지 / 주행. 긴 바퀴가 신호 주기 때문인지 본다
            tm = timing.setdefault(lap(), {"stop": 0.0, "red": 0.0, "other": 0.0, "moving": 0.0})
            if stopstate & 1 and stopstate & 16:
                tm["stop"] += 0.1
            elif tc.vehicle.getSpeed("bus0") < 0.1:
                tls = tc.vehicle.getNextTLS("bus0")
                tm["red" if (tls and tls[0][3] in "rRyY" and tls[0][2] < 25.0) else "other"] += 0.1
            else:
                tm["moving"] += 0.1
            if tc.vehicle.getSpeed("bus0") < 0.1 and not (stopstate & 1 and stopstate & 16):
                bw["cur"] += 0.1
                if bw["cur"] > 15.0 and not bw.get("noted") and len(bw["long"]) < 12:
                    ld = tc.vehicle.getLeader("bus0", 60.0)
                    tls = tc.vehicle.getNextTLS("bus0")
                    bw["long"].append({"t": round(t), "lap": lap(), "road": road, "lane": idx, "pos": round(pos),
                                       "leader": [ld[0], tc.vehicle.getTypeID(ld[0]).split("@")[0], round(ld[1], 1)] if ld else None,
                                       "tls": [tls[0][0], tls[0][3], round(tls[0][2])] if tls else None,
                                       "phase": bus.phase, "why": bus.why})
                    bw["noted"] = True
            else:
                bw["cur"], bw["noted"] = 0.0, False
            if st["attempt"] and st["attempt"]["kind"] == "S3" and int(round(t * 10)) % 10 == 0 and len(bw["s3_trace"]) < 80:
                bw["s3_trace"].append((round(t), road, idx, bus.target, bus.phase, round(tc.vehicle.getSpeed("bus0"), 1)))
            # S1: 정류장에 서서 출발
            state = tc.vehicle.getStopState("bus0")
            at_stop = bool(state & 1 and state & 16)
            if st["stopped"] and not at_stop:
                ev["S1_stops"][lap()] += 1
            if at_stop and not st["stopped"]:
                sig["stops"][lap()].append(road)            # 어느 정류장에 섰나(s2·s4·s5·s6만이어야 한다)
            st["stopped"] = at_stop
            # A1이 신호 교차로 영역(정지선 앞 금지 거리, 교차로 안)에서 시작했나
            for vid in list(world.mgr.inter):
                if vid in sig["seen"]:
                    continue
                sig["seen"].add(vid)
                node = road[1:].rsplit("_", 1)[0] if road.startswith(":") else None
                inside = node is not None and net.hasNode(node) and is_signalized(net.getNode(node))
                if inside or any(lo <= pos <= hi for lo, hi in forbidden.get(road, [])):
                    sig["in_area"].append((round(t, 1), lap(), vid, road, round(pos, 1)))
            # S2/S3: 차선변경 시도(시험용 버스 규칙의 방향지시등 시작)
            if bus.phase == "signal" and st["prev_phase"] != "signal":
                # S2 = 경로상 왼쪽 변경. S3 = 정류장이 있는 도로에서 오른쪽으로 옮기는 변경(SUMO가 정류장 위치를 알고 '경로'로
                # 요청하기도 하고 시험용 버스 규칙이 '정류장'으로 요청하기도 한다)
                kind = bus.why if bus.why in ("S2", "S3") else "other"     # 정해진 창(S3 s2 앞, S2 J82·J1 앞)의 변경
                win = WIN_JUNCTION.get(bus.plan.edge) if (kind == "S2" and bus.plan is not None) else None
                st["attempt"] = {"kind": kind, "target": bus.target, "lap": lap(), "win": win,
                                 "a1_before": len(world.mgr.log) + len(world.mgr.inter)}
                ev[f"{kind}_attempts" if kind != "other" else "other_attempts"][lap()] += 1
                if win:
                    ev[f"S2_{win}_attempts"][lap()] += 1
            if st["attempt"] and idx == st["attempt"]["target"] and st["attempt"]["kind"] != "other" and not st["attempt"].get("done"):
                st["attempt"]["done"] = True
                ev[f"{st['attempt']['kind']}_done"][st["attempt"]["lap"]] += 1
            if st["prev_phase"] != "idle" and bus.phase == "idle" and st["attempt"]:
                a = st["attempt"]
                if len(world.mgr.log) + len(world.mgr.inter) > a["a1_before"] and a["kind"] != "other":
                    ev[f"{a['kind']}_a1"][a["lap"]] += 1
                    if a.get("win"):
                        ev[f"S2_{a['win']}_a1"][a["lap"]] += 1
                st["attempt"] = None
            st["prev_phase"] = bus.phase
            # S4: 서는 정류장에 다가갈 때 정류장 차로의 정류장 근처에 정차 중인 차가 있다(누가 세웠는지와 무관)
            for s in (served if int(round(t * 10)) % 5 == 0 and not at_stop else ()):
                if (lap(), s["id"]) in s4["seen"]:
                    continue
                dd = tc.vehicle.getDrivingDistance("bus0", s["edge"], s["startPos"])
                if not 0 < dd <= S4_APPROACH_M:
                    continue
                lane = f"{s['edge']}_{s['lane_idx']}"
                lo, hi = s["startPos"] - S4_BEFORE_M, s["endPos"] + S4_AFTER_M
                for v in tc.lane.getLastStepVehicleIDs(lane):
                    if v != "bus0" and tc.vehicle.isStopped(v) and lo <= tc.vehicle.getLanePosition(v) <= hi:
                        s4["seen"].add((lap(), s["id"]))
                        ev["S4"][lap()] += 1
                        s4["events"].append((round(t), lap(), s["id"], v, tc.vehicle.getTypeID(v).split("@")[0],
                                             round(tc.vehicle.getLanePosition(v), 1), round(dd)))
                        break
            # S5: 골목에서 나온 차가 버스 앞 50 m에 들어옴
            for v in (ids if int(round(t * 10)) % 5 == 0 else ()):          # 0.5초마다만 훑는다(속도)
                if v == "bus0":
                    continue
                r = tc.vehicle.getRoadID(v)
                if r in feeders:
                    st["alley_seen"][v] = t
                elif (v in st["alley_seen"] and v not in st["cut"] and t - st["alley_seen"][v] <= CUT_WINDOW
                      and r == road and 0 < tc.vehicle.getLanePosition(v) - pos <= NEAR):
                    st["cut"].add(v)
                    ev["S5"][lap()] += 1
            # S6: 신호 없는 횡단보도 50 m 앞에서 보행자가 건너고 있다
            near_cross = (road == S6_EDGE and tc.lane.getLength(f"{S6_EDGE}_0") - pos <= NEAR) or road.startswith(":S6_wausanro")
            if near_cross:
                if any(tc.person.getRoadID(p).startswith(S6_CROSS) for p in tc.person.getIDList()):
                    if (lap(), "x") not in st["s6_pass"] and not st["s6_now"]:
                        ev["S6"][lap()] += 1
                        st["s6_pass"].add((lap(), "x"))
                    st["s6_now"] = True
            else:
                st["s6_now"] = False
                st["s6_pass"].discard((lap(), "x"))
        if "bus0" not in ids and st["attempt"] and st["attempt"]["kind"] in ("S2", "S3") and not st["attempt"].get("done"):
            a = st["attempt"]                          # 한 바퀴를 마치며 버스가 사라졌다: 진행 중이던 시도는 거짓 시도다(노선 끝 R69)
            ev[f"{a['kind']}_attempts"][a["lap"]] -= 1
            st["attempt"] = None
        if len(bus.lap_times) >= LAPS or t > T_MAX:
            raise Done()

    wait = {"max_wait": 0.0, "over20": set(), "over30": set(), "max_queue": 0, "occ_steps": 0, "steps": 0,
            "streak": 0.0, "max_streak": 0.0, "gap": 0.0, "wait_by_veh": {}}

    safety = {"collisions": 0, "teleports": 0, "popin_100m": [], "popin_200m": 0, "departed_near": [],
              "by_type": {}, "pairs": set(), "coll_examples": []}

    def sample_safety(t):
        """충돌·순간이동과, 시험용 버스 가까이에서 새 차가 생기는지(갑작스러운 출현)를 센다."""
        for c in tc.simulation.getCollisions():
            safety["collisions"] += 1
            key = (c.type, str(c.colliderType).split("@")[0], str(c.victimType).split("@")[0])
            safety["by_type"][key] = safety["by_type"].get(key, 0) + 1
            new_pair = (c.collider, c.victim) not in safety["pairs"]
            safety["pairs"].add((c.collider, c.victim))
            if new_pair and len(safety["coll_examples"]) < 30:      # 같은 쌍은 한 번만 적는다(한 사건이 스텝마다 반복돼 예시를 채웠다)
                bus_pos = (tc.vehicle.getRoadID("bus0"), round(tc.vehicle.getLanePosition("bus0"), 1)) \
                    if "bus0" in tc.vehicle.getIDList() else None
                coll_extra = (c.colliderSpeed, c.victimSpeed, bus_pos)
                tls_info = None
                for tid, ror in (world.summary.get("right_on_red") or {}).items():
                    if c.lane.startswith(f":{tid}_"):          # 우회전을 바꾼 신호 교차로 안: 그때 위상과 원래 프로그램의 상태
                        ph = tc.trafficlight.getPhase(tid)
                        tls_info = (tid, ph, tc.trafficlight.getRedYellowGreenState(tid), ror["orig"][ph])
                safety["coll_examples"].append((round(t), c.type, c.collider, c.victim, c.lane, tls_info, coll_extra))
        safety["teleports"] += len(tc.simulation.getStartingTeleportIDList())
        ids = tc.vehicle.getIDList()
        if "bus0" in ids:
            bx, by = tc.vehicle.getPosition("bus0")
            for v in tc.simulation.getDepartedIDList():
                if v == "bus0" or v not in ids or v.startswith("bg") and False:
                    continue
                x, y = tc.vehicle.getPosition(v)
                d = ((x - bx) ** 2 + (y - by) ** 2) ** 0.5
                if d < 200:
                    safety["popin_200m"] += 1
                if d < 100:
                    safety["popin_100m"].append((round(t), v, round(d)))

    trace = {"a1": []}                                           # 방향지시등이 켜진 동안의 목표 차로 후행차(원인 확인용)

    def sample_trace(t):
        ids = tc.vehicle.getIDList()
        busr = ((tc.vehicle.getRoadID("bus0"), round(tc.vehicle.getLanePosition("bus0")), tc.vehicle.getLaneIndex("bus0"))
                if "bus0" in ids else None)
        if busr and tc.vehicle.getSignals("bus0") & 3 and len(trace["a1"]) < 400:   # 방향지시등이 켜진 동안 A1이 왜 안 켜지는지
            side = 1 if tc.vehicle.getSignals("bus0") & 2 else -1
            mgr = world.mgr
            f = tc.vehicle.getLeftFollowers("bus0") if side > 0 else tc.vehicle.getRightFollowers("bus0")
            tf = mgr._target_follower(side)
            trace["a1"].append((round(t, 1), lap(), busr, side, bus.phase,
                                [(i, round(d)) for i, d in f if i][:3], tf,
                                tc.vehicle.getParameter(tf, "driver.aggressiveness") if tf else None,
                                any(tr.matches(side, busr[0], busr[1]) for tr in (mgr.triggers or [])),
                                len(mgr.inter)))

    def sample_s6(t):
        """신호 없는 횡단보도 앞 대기: 가장 오래 기다린 차, 20·30초 넘게 기다린 차 수, 대기 줄 길이, 보행자 점유."""
        w = wait
        w["steps"] += 1
        queue = 0
        for lane in S6_APPROACH:
            length = tc.lane.getLength(lane)
            for v in tc.lane.getLastStepVehicleIDs(lane):
                if tc.vehicle.getSpeed(v) < 0.1 and length - tc.vehicle.getLanePosition(v) <= S6_QUEUE_M:
                    queue += 1
                    wt = tc.vehicle.getWaitingTime(v)
                    w["wait_by_veh"][v] = max(w["wait_by_veh"].get(v, 0.0), wt)
                    w["max_wait"] = max(w["max_wait"], wt)
                    if wt > 20:
                        w["over20"].add(v)
                    if wt > 30:
                        w["over30"].add(v)
        w["max_queue"] = max(w["max_queue"], queue)
        occupied = any(tc.person.getRoadID(p).startswith(S6_CROSS) for p in tc.person.getIDList())
        w["occ_steps"] += occupied
        if occupied:                                   # 1초 안의 틈은 같은 연속 점유로 본다(0.5초 간격 표본)
            w["streak"] += 0.5 + w["gap"]
            w["gap"] = 0.0
            w["max_streak"] = max(w["max_streak"], w["streak"])
        else:
            w["gap"] += 0.5
            if w["gap"] > 1.0:
                w["streak"], w["gap"] = 0.0, 0.0

    _on_step = on_step

    probe = {"still": 0.0, "log": []}

    def stuck_probe(t):
        """버스가 교차로 안에서 5초 넘게 서 있으면 10초마다 무엇을 기다리는지 적는다(J82 정지 원인 찾기)."""
        if "bus0" not in tc.vehicle.getIDList() or not tc.vehicle.getRoadID("bus0").startswith(":") \
                or tc.vehicle.getSpeed("bus0") > 0.1:
            probe["still"] = 0.0
            return
        probe["still"] += 0.1
        if probe["still"] < 5.0 or int(round(probe["still"] * 10)) % 100 != 0 or len(probe["log"]) >= 40:
            return
        road = tc.vehicle.getRoadID("bus0")
        node = road[1:].rsplit("_", 1)[0]
        links = []
        try:
            links = [(l[0], l[1], l[2], l[3], l[4], l[5]) for l in tc.vehicle.getNextLinks("bus0")[:2]]
        except Exception:
            pass
        around = []
        for lane in tc.lane.getIDList():
            if lane.startswith(f":{node}") and "_c" not in lane and "_w" not in lane:
                around += [(v, lane, round(tc.vehicle.getSpeed(v), 1)) for v in tc.lane.getLastStepVehicleIDs(lane) if v != "bus0"]
        peds = [(p, tc.person.getRoadID(p), round(tc.person.getSpeed(p), 2)) for p in tc.person.getIDList()
                if tc.person.getRoadID(p).startswith(f":{node}")]
        nxt = []
        for lk in links[:1]:
            nxt = [(v, round(tc.vehicle.getSpeed(v), 1)) for v in tc.lane.getLastStepVehicleIDs(lk[0])] if lk[0] else []
        probe["log"].append({"t": round(t, 1), "lap": getattr(bus, "lap", None), "road": road,
                             "pos": round(tc.vehicle.getLanePosition("bus0"), 1), "still": round(probe["still"], 1),
                             "leader": tc.vehicle.getLeader("bus0"), "next_links": links,
                             "speed_without_traci": round(tc.vehicle.getSpeedWithoutTraCI("bus0"), 2),
                             "stop_state": tc.vehicle.getStopState("bus0"), "speed_mode": tc.vehicle.getSpeedMode("bus0"),
                             "route_idx": tc.vehicle.getRouteIndex("bus0"), "route_tail": tc.vehicle.getRoute("bus0")[max(0, tc.vehicle.getRouteIndex("bus0") - 1):][:4],
                             "next_stops": [(getattr(s, "stoppingPlaceID", ""), round(float(getattr(s, "endPos", 0.0)), 1))
                                            for s in tc.vehicle.getStops("bus0")[:2]] if hasattr(tc.vehicle, "getStops") else None,
                             "pilot_phase": bus.phase, "pilot_forced": bus._forced_p is not None,
                             "tls": {node: [tc.trafficlight.getRedYellowGreenState(node), tc.trafficlight.getPhase(node),
                                            round(tc.trafficlight.getNextSwitch(node) - t, 1)]}
                             if node in tc.trafficlight.getIDList() else None,
                             "in_junction": around[:12], "persons_in_junction": peds[:12], "on_exit_lane": nxt[:12]})

    def on_step(t):                                   # noqa: F811
        stuck_probe(t)
        if int(round(t * 10)) % 5 == 0:                # 횡단보도 계측은 0.5초마다(속도)
            sample_s6(t)
            sample_trace(t)
        sample_safety(t)
        presence.update(t)                             # 시험용 버스 반경 안 출현·소멸·순간이동
        _on_step(t)

    ped_cfg = None
    if OLD_PED:                                       # 보행자 수정 전 설정: 서 있는 차 = 10초, 차량 대기 계수 없음
        from traffic.ped_defiance import DefianceCfg
        ped_cfg = DefianceCfg(standing_speed=1e9, standing_ttc=10.0,
                              use=("ttc", "wait", "group", "width", "size", "momentum", "age", "distract"))
    try:
        world.run(tc.simulation.getTime() + T_MAX + 10, bus_id="bus0", bus_hook=bus.tick, on_step=on_step,
                  ped_cfg=ped_cfg, ped_defiance=PED_DEFIANCE, pilot=bus, triggers="scenario")
    except Done:
        pass
    # S1 후행차 판정은 관찰 기록에서 센다(바퀴는 시험용 버스 한 바퀴)
    laps_done = len(bus.lap_times)
    per_lap = lambda c: [c.get(i, 0) for i in range(1, LAPS + 1)]
    bounds, acc = [], st["t0"] or 0.0
    for d in bus.lap_times:                      # 바퀴 끝 시각(루프 시각)
        acc += d
        bounds.append(acc)
    s1_rear = collections.Counter()
    for r in world.stop.log:                     # S1 판정이 일어난 시각 -> 바퀴
        s1_rear[sum(1 for b in bounds if b < r["t_decide"]) + 1] += 1
    res = {"scenario": SCEN, "seed": SEED, "laps_done": laps_done, "lap_seconds": [round(x) for x in bus.lap_times],
           "S1": {"bus_stops_then_departs": per_lap(ev["S1_stops"]), "rear_vehicle_events": per_lap(s1_rear),
                  "rear_outcomes": world.stop.summary()},
           "S2": {"attempts": per_lap(ev["S2_attempts"]), "done": per_lap(ev["S2_done"]), "with_a1_follower": per_lap(ev["S2_a1"])},
           "S3": {"attempts": per_lap(ev["S3_attempts"]), "done": per_lap(ev["S3_done"]), "with_a1_follower": per_lap(ev["S3_a1"])},
           "S2_by_junction": {j: {"attempts": per_lap(ev[f"S2_{j}_attempts"]), "with_a1": per_lap(ev[f"S2_{j}_a1"])}
                              for j in ("J82", "J1")},
           "S4": {"blocks_per_lap": per_lap(ev["S4"]), "total": sum(per_lap(ev["S4"])), "events": s4["events"]},
           "presence": presence.summary(),
           "a1_in_signal_area": len(sig["in_area"]), "a1_in_signal_area_examples": sig["in_area"][:8],
           "stops_by_lap": {k: v for k, v in sig["stops"].items()},
           "bus": {"missed": bus.missed, "forced": bus.forced, "slowed": bus.slowed, "changes": bus.changes},
           "S5": {"per_lap": per_lap(ev["S5"])},
           "S6": {"per_lap": per_lap(ev["S6"]), "ped_decisions": world.ped.summary() if world.ped else None},
           "S6_vehicle_waiting": {"max_wait_s": round(wait["max_wait"], 1), "vehicles_over_20s": len(wait["over20"]),
                                  "vehicles_over_30s": len(wait["over30"]), "max_queue": wait["max_queue"],
                                  "crossing_occupied_share": round(wait["occ_steps"] / max(1, wait["steps"]), 3),
                                  "longest_continuous_occupied_s": round(wait["max_streak"], 1),
                                  "sim_seconds": round(wait["steps"] * 0.5)},
           "safety": {"collisions": safety["collisions"], "collision_events": len(safety["pairs"]),
                      "collisions_by_type": {"|".join(k): v for k, v in safety["by_type"].items()},
                      "collision_examples": safety["coll_examples"], "teleports": safety["teleports"],
                      "new_vehicles_within_200m_of_bus": safety["popin_200m"],
                      "new_vehicles_within_100m_of_bus": len(safety["popin_100m"]),
                      "within_100m_examples": safety["popin_100m"][:8]},
           "ped_x": PED_X, "ped_cfg": "old" if OLD_PED else "new", "variant": TAG, "trace": trace,
           "lap_time_breakdown": {k: {kk: round(v, 1) for kk, v in d.items()} for k, d in timing.items()},
           "bus_long_waits_outside_stops": bw["long"], "bus_stuck_probe": probe["log"], "s3_trace": bw["s3_trace"],
           "interior": world.interior.summary() if world.interior else None,
           "right_on_red": world.summary.get("right_on_red"),
           "other_lane_change_attempts": per_lap(ev["other_attempts"]),
           "a1_interactions_total": len(world.mgr.log)}
    path = os.path.join(os.path.dirname(__file__), "..", "results", "traffic_verification", f"s1_s6_{SCEN}_{SEED}{'_' + TAG if TAG else ''}.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    json.dump(res, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(json.dumps(res, ensure_ascii=False))
    world.close()


if __name__ == "__main__":
    main()
