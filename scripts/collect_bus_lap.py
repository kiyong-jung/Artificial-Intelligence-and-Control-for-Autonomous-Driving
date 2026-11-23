"""합정역 시나리오(scripts/run_hapjeong_scenario.py와 같은 도로·교통·시험용 버스 규칙)에서 버스가 한 바퀴 도는 동안의 데이터를 저장한다.

    python scripts/collect_bus_lap.py                       # baseline, seed 0, 1바퀴
    python scripts/collect_bus_lap.py --scenario busy --seed 3 --name busy_s3

시험용 버스(traffic/test_bus.py)가 R68에서 출발해 노선을 돌고 다시 R68에 들어서면(= 1바퀴) 멈춘다.
판단 간격(0.5초 = SUMO 5스텝)마다 한 행을 기록한다. 컬럼 설명은 같이 저장되는 .metadata.json에 있다.

출력 (data/bus_lap/ 아래):
    <name>.jsonl.gz       한 줄이 한 행(중첩 dict). 주변 차량·보행자 목록까지 전부 담는다
    <name>.csv            같은 행의 평평한 요약(스칼라만). 엑셀·pandas로 바로 열 수 있다
    <name>.metadata.json  실행 설정, 바퀴 시간, 컬럼 설명
"""
import argparse
import csv
import gzip
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.bus_window_keeper import DEFAULT_LANE_DENSITY  # noqa: E402
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.sumo_conn import traci  # noqa: E402
from traffic.test_bus import BUS, TestBusPilot  # noqa: E402
from run_hapjeong_scenario import map_stops, MODEL_NAME  # noqa: E402

DECISION_STEPS = 5          # 0.5초 = 0.1초 스텝 5개
VISIBILITY_M = 50.0         # 주변 감지 반경(env/mdp_config.py의 visibility와 같다)
DT = 0.1
SIGNAL_BITS = {"right": 1, "left": 2, "brake": 8}
C = traci.constants


class LapDone(Exception):
    """버스가 한 바퀴를 마쳤다."""


def rel(dx, dy, heading_deg):
    """도로망 좌표 차이를 버스 차체 좌표로. 종방향 +앞, 횡방향 +왼쪽."""
    h = math.radians(heading_deg)
    return dx * math.sin(h) + dy * math.cos(h), -dx * math.cos(h) + dy * math.sin(h)


def rel_heading(other_deg, ego_deg):
    """상대가 버스보다 왼쪽(반시계)으로 돌아 있으면 +, -180~180도."""
    return (ego_deg - other_deg + 180.0) % 360.0 - 180.0


class Recorder:
    def __init__(self, tc, world, bus, jf, csv_writer, nd):
        self.tc, self.world, self.bus, self.jf, self.cw, self.nd = tc, world, bus, jf, csv_writer, nd
        self.step, self.rows = 0, 0
        self.subscribed = False
        self.t0 = None
        self.prev = None                 # 직전 판단 시점의 (도로, 차로 번호)
        self.acc_peak = self.jerk_peak = 0.0
        self.last_acc = None
        self.collisions = self.teleports = 0
        self.flat_header = None

    # ---- 매 0.1초 스텝: 승차감(가속도·jerk 최댓값)과 사건을 누적 ----
    def on_step(self, t):
        tc = self.tc
        self.collisions += len(tc.simulation.getCollisions())
        self.teleports += len(tc.simulation.getStartingTeleportIDList())
        if BUS not in tc.vehicle.getIDList():
            return
        if not self.subscribed:                          # 버스가 생긴 뒤 한 번: 반경 안 차량·보행자를 구독
            tc.vehicle.subscribeContext(BUS, C.CMD_GET_VEHICLE_VARIABLE, VISIBILITY_M,
                                    [C.VAR_POSITION, C.VAR_SPEED, C.VAR_ANGLE, C.VAR_SIGNALS, C.VAR_TYPE,
                                     C.VAR_ROAD_ID, C.VAR_LANE_INDEX])
            self.subscribed, self.t0 = True, t
        acc = tc.vehicle.getAcceleration(BUS)
        self.acc_peak = max(self.acc_peak, abs(acc))
        if self.last_acc is not None:
            self.jerk_peak = max(self.jerk_peak, abs(acc - self.last_acc) / DT)
        self.last_acc = acc
        self.step += 1
        if self.step % DECISION_STEPS == 0:
            self.record(t)
            self.acc_peak = self.jerk_peak = 0.0
        if len(self.bus.lap_times) >= 1:
            raise LapDone

    # ---- 판단 한 번의 행 ----
    def record(self, t):
        tc, bus = self.tc, self.bus
        v = tc.vehicle
        x, y = v.getPosition(BUS)
        heading = v.getAngle(BUS)
        road, lane_idx = v.getRoadID(BUS), v.getLaneIndex(BUS)
        signals = v.getSignals(BUS)
        before = self.prev
        self.prev = (road, lane_idx)
        actual = 0
        if before and before[0] == road and before[1] != lane_idx:
            actual = 1 if lane_idx > before[1] else -1             # 같은 도로에서 번호가 오르면 왼쪽
        phase_dir = 0
        if bus.phase in ("signal", "request") and bus.target is not None:
            phase_dir = 1 if bus.target > lane_idx else -1 if bus.target < lane_idx else 0

        stop_info = None
        try:
            ns = v.getNextStops(BUS)
            if ns:
                lane, end_pos, sid = ns[0][0], ns[0][1], ns[0][2]      # (차로, 끝 위치, 정류장 id, 플래그, ...)
                dist = None
                try:
                    dist = round(v.getDrivingDistance(BUS, lane.rsplit("_", 1)[0], end_pos), 1)
                except Exception:
                    pass
                stop_info = {"id": sid, "distance": dist}
        except Exception:
            pass
        tls = None
        try:
            nt = v.getNextTLS(BUS)
            if nt:
                tls = {"id": nt[0][0], "link_index": nt[0][1], "distance_to_stop_line": round(nt[0][2], 1),
                       "state": nt[0][3]}
        except Exception:
            pass
        try:
            dest = round(v.getDrivingDistance(BUS, v.getRoute(BUS)[-1], 0.0), 1)
        except Exception:
            dest = None

        state = {"x": x, "y": y, "speed": v.getSpeed(BUS), "acceleration": v.getAcceleration(BUS),
                 "heading": heading, "road_id": road, "lane_id": v.getLaneID(BUS), "lane_index": lane_idx,
                 "lane_position": v.getLanePosition(BUS), "lateral_position": v.getLateralLanePosition(BUS),
                 "signals": signals, "route_index": v.getRouteIndex(BUS), "distance_driven": v.getDistance(BUS),
                 "stopped": bool(v.getStopState(BUS) & 1), "waiting_time": v.getWaitingTime(BUS)}
        ctl = {"phase": bus.phase, "reason": bus.why if bus.phase not in ("idle",) else "",
               "target_lane": bus.target if bus.phase in ("signal", "request") else None,
               "request_direction": phase_dir,
               "blinker": "left" if signals & SIGNAL_BITS["left"] else "right" if signals & SIGNAL_BITS["right"] else "none",
               "lane_change_actual": actual,
               "forced_stop": bus._forced_p is not None,
               "speed_limited": bool(bus._limited),
               "blocker_active": bool(bus.blocker_active()) if bus.blocker_active else False}
        inter = self.world.mgr.inter

        vehicles, persons = [], []
        for vid, d in tc.vehicle.getContextSubscriptionResults(BUS).items():
            if vid == BUS:
                continue
            ox, oy = d[C.VAR_POSITION]
            lon, lat = rel(ox - x, oy - y, heading)
            sig = d[C.VAR_SIGNALS]
            vtype = d[C.VAR_TYPE].split("@")[0]
            item = {"id": vid, "type": MODEL_NAME.get(vtype, vtype), "x": ox, "y": oy,
                    "rel_longitudinal": lon, "rel_lateral": lat, "speed": d[C.VAR_SPEED],
                    "rel_heading": rel_heading(d[C.VAR_ANGLE], heading), "road_id": d[C.VAR_ROAD_ID],
                    "lane_index": d[C.VAR_LANE_INDEX], "left_signal": bool(sig & 2), "right_signal": bool(sig & 1),
                    "braking": bool(sig & 8)}
            if vid in inter:                                 # 버스를 마주한 차량의 A1 판정
                item["a1"] = "yield" if inter[vid].yields else "block"
            vehicles.append(item)
        vehicles.sort(key=lambda r: r["rel_longitudinal"] ** 2 + r["rel_lateral"] ** 2)
        for pid in tc.person.getIDList():
            ox, oy = tc.person.getPosition(pid)
            if math.hypot(ox - x, oy - y) > VISIBILITY_M:
                continue
            lon, lat = rel(ox - x, oy - y, heading)
            persons.append({"id": pid, "x": ox, "y": oy, "rel_longitudinal": lon, "rel_lateral": lat,
                            "speed": tc.person.getSpeed(pid), "road_id": tc.person.getRoadID(pid)})
        persons.sort(key=lambda r: r["rel_longitudinal"] ** 2 + r["rel_lateral"] ** 2)

        row = {"step": self.rows, "sim_time": round(tc.simulation.getTime(), 2), "lap": bus.lap, "bus": state,
               "control": ctl,
               "navigation": {"next_stop": stop_info, "traffic_light": tls, "distance_to_destination": dest},
               "comfort": {"peak_acceleration": self.acc_peak, "peak_jerk": self.jerk_peak},
               "detected_vehicles": vehicles, "detected_pedestrians": persons,
               "event": {"collisions_total": self.collisions, "teleports_total": self.teleports,
                         "a1_interacting": len(inter)}}
        self.jf.write(json.dumps(round_floats(row, self.nd), ensure_ascii=False, separators=(",", ":")) + "\n")
        self.write_flat(row)
        self.rows += 1

    def write_flat(self, row):
        flat = {"step": row["step"], "sim_time": row["sim_time"], "lap": row["lap"]}
        for k, v in row["bus"].items():
            flat[f"bus.{k}"] = v
        for k, v in row["control"].items():
            flat[f"control.{k}"] = v
        ns, tls = row["navigation"]["next_stop"], row["navigation"]["traffic_light"]
        flat.update({"nav.next_stop_id": ns["id"] if ns else "", "nav.next_stop_distance": ns["distance"] if ns else "",
                     "nav.tls_state": tls["state"] if tls else "",
                     "nav.tls_distance": tls["distance_to_stop_line"] if tls else "",
                     "nav.distance_to_destination": row["navigation"]["distance_to_destination"]})
        flat["comfort.peak_acceleration"] = row["comfort"]["peak_acceleration"]
        flat["comfort.peak_jerk"] = row["comfort"]["peak_jerk"]
        veh = row["detected_vehicles"]
        flat["n_vehicles"], flat["n_pedestrians"] = len(veh), len(row["detected_pedestrians"])
        lead = [d for d in veh if abs(d["rel_lateral"]) < 1.8 and d["rel_longitudinal"] > 0]
        flat["lead_gap"] = min((d["rel_longitudinal"] for d in lead), default="")
        flat["a1_interacting"] = row["event"]["a1_interacting"]
        flat["collisions_total"], flat["teleports_total"] = row["event"]["collisions_total"], row["event"]["teleports_total"]
        flat = round_floats(flat, self.nd)
        if self.cw[0] is None:
            self.cw[0] = csv.DictWriter(self.cw[1], fieldnames=list(flat))
            self.cw[0].writeheader()
        self.cw[0].writerow(flat)


def round_floats(o, nd):
    if isinstance(o, float):
        return round(o, nd)
    if isinstance(o, dict):
        return {k: round_floats(v, nd) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [round_floats(v, nd) for v in o]
    return o


COLUMN_DOC = {
    "step / sim_time / lap": "행 번호(0.5초마다 1), SUMO 시각(초), 버스가 도는 바퀴 번호",
    "bus.*": "버스(ego)의 실제 상태: 위치 x,y, 속도, 가속도, 진행 방향, 도로·차로, 차로 안 위치, 횡방향 위치, 방향지시등 비트, "
             "경로 인덱스, 누적 주행거리, 정차 여부, 대기 시간",
    "control.*": "시험용 버스 규칙이 이번 판단에서 낸 명령: 차선 변경 단계(phase), 이유(S2 좌회전 창, S3 정류장 접근 등), 목표 차로, "
                 "요청 방향(+1 왼쪽, -1 오른쪽), 방향지시등, 실제 차로 이동(lane_change_actual), 도착 보장 강제 정차, 속도 제한(골목 차·"
                 "막는 차), 막는 차 활성",
    "navigation": "다음 정류장 id·남은 거리, 다음 신호등 상태·정지선까지 거리, 노선 끝까지 남은 주행 거리",
    "comfort": "이번 0.5초 동안 가속도·jerk 크기의 최댓값(승차감)",
    "detected_vehicles": "버스 반경 50 m 안 차량(가까운 순): 종류(Krauss·IDM·EIDM·ACC·버스·택시 등), 버스 기준 상대 위치·속도·방향, 도로·차로, "
                         "방향지시등·제동등, 버스를 마주했다면 A1 판정(양보/비양보)",
    "detected_pedestrians": "버스 반경 50 m 안 보행자: 상대 위치, 속도, 도로",
    "event": "누적 충돌 수, 순간이동 수, 지금 A1 상호작용 중인 차량 수",
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=("quiet", "baseline", "busy"), default="baseline")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--warmup", type=float, default=120.0)
    ap.add_argument("--max-seconds", type=float, default=1800.0, help="한 바퀴가 이 시간 안에 끝나지 않으면 멈춘다")
    ap.add_argument("--lane-buses", type=float, default=30.0)
    ap.add_argument("--taxis", type=float, default=40.0)
    ap.add_argument("--route-boost", type=float, default=1.3)
    ap.add_argument("--ndigits", type=int, default=3)
    ap.add_argument("--name", default=None)
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "..", "data", "bus_lap"))
    ap.add_argument("--work-dir", default=os.path.join(tempfile.gettempdir(), "hapjeong_collect"))
    a = ap.parse_args()

    name = a.name or f"hapjeong_{a.scenario}_s{a.seed}"
    os.makedirs(a.out_dir, exist_ok=True)
    from traffic.bus_traffic import BusTrafficCfg
    bt = BusTrafficCfg(lane_buses_per_h=a.lane_buses, taxis_per_h=a.taxis) if a.lane_buses > 0 else None
    world = HapjeongTraffic(a.work_dir, scenario=a.scenario, seed=a.seed, bus_traffic=bt,
                            lane_density=DEFAULT_LANE_DENSITY, route_flow_boost=a.route_boost)
    world.build()
    tc = world.start(warmup=a.warmup, gui=False)
    m = load_map("hapjeong")
    net = read_net(os.path.join(a.work_dir, "network.net.xml"))
    bus = TestBusPilot(tc, net, m["routes"]["bus_loop"], map_stops(m))

    jsonl, csv_path, meta_path = (os.path.join(a.out_dir, name + ext) for ext in (".jsonl.gz", ".csv", ".metadata.json"))
    rec, done = None, False
    with gzip.open(jsonl, "wt", encoding="utf-8", compresslevel=6) as jf, \
            open(csv_path, "w", newline="", encoding="utf-8-sig") as cf:
        rec = Recorder(tc, world, bus, jf, [None, cf], a.ndigits)
        try:
            world.run(tc.simulation.getTime() + a.max_seconds, bus_id=BUS, bus_hook=bus.tick, on_step=rec.on_step,
                      stop_bypass=False, ped_defiance=True, stop_ahead=True, scenario_inject=True, pilot=bus,
                      triggers="scenario")
        except LapDone:
            done = True
    meta = {"script": "scripts/collect_bus_lap.py", "scenario": a.scenario, "seed": a.seed, "warmup_s": a.warmup,
            "decision_interval_s": DECISION_STEPS * DT, "visibility_m": VISIBILITY_M,
            "lap_completed": done, "lap_times_s": bus.lap_times, "rows": rec.rows,
            "bus_lane_change_requests": bus.changes, "missed_windows": [list(x) for x in bus.missed],
            "forced_stops": [list(x) for x in bus.forced], "collisions": rec.collisions, "teleports": rec.teleports,
            "lane_buses_per_h": a.lane_buses, "taxis_per_h": a.taxis, "route_boost": a.route_boost,
            "columns": COLUMN_DOC}
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    world.close()
    print(f"한 바퀴 완주: {done} | 행 {rec.rows} | 바퀴 시간 {bus.lap_times} | 충돌 {rec.collisions} | 순간이동 {rec.teleports}")
    print(jsonl, csv_path, meta_path, sep="\n")


if __name__ == "__main__":
    main()
