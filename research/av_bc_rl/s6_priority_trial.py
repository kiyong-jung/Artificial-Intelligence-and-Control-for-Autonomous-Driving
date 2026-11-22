"""S6 횡단보도를 보행자 우선으로 바꾼 임시 지도를 만들고, 커밋된 지도와 같은 seed로 견준다.

    python research/av_bc_rl/s6_priority_trial.py make
    python research/av_bc_rl/s6_priority_trial.py collect --episodes 20 --workers 4 --seed 900
    python research/av_bc_rl/s6_priority_trial.py collect --lookahead --episodes 20 --workers 4 --seed 900
    python research/av_bc_rl/s6_priority_trial.py background --seeds 0 1 2 3 4 --seconds 3600 --warmup 300
    python research/av_bc_rl/s6_priority_trial.py report
    python research/av_bc_rl/s6_priority_trial.py clean

make는 커밋된 maps/hapjeong을 maps/hapjeong_trial_s6prio(gitignore)로 복사하고, netconvert의
`<crossing node="S6_wausanro" ... priority="true"/>`가 바꾸는 도로망 줄만 옮긴다(make의 설명). 커밋된 도로망은
build_hapjeong.py가 모르는 패치가 얹혀 있어 다시 빌드하지 않는다(루트 README "알려진 한계").
collect는 두 지도에서 전문가 버스(collect_pomdp_data.py --policy expert, busy)를 같은 에피소드 seed로 달린다.
--lookahead는 우선 지도에서만, 보도 위에서 횡단보도로 걸어 들어가는 사람도 crosswalk_occupied로 보게 env를 이 실행
안에서만 바꿔 달린다(env/ 파일은 그대로다). 우선 횡단보도의 SUMO 보행자는 서지 않고 바로 들어서서, 기존 규칙(멈춰 선
사람만 셈)으로는 버스가 늦게 알아챈다.
background는 버스 없이 배경 교통만 돌려 S6의 보행자 대기와 차량 정지를 TraCI로 센다.
report는 모은 결과를 나란히 찍고 results/s6_priority/summary.json에 쓴다.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import gzip
import json
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parents[2]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

import sumolib  # noqa: E402

from env.data_schema import OBSERVATION_FEATURES  # noqa: E402
from scripts.add_j1_left_lane import tls_links  # noqa: E402
from scripts.patch_network import digest  # noqa: E402
from scripts.prune_network import OPTIONS  # noqa: E402

COMMITTED = "hapjeong"
TRIAL = "hapjeong_trial_s6prio"
MAPS = (COMMITTED, TRIAL)
LOOKAHEAD = "lookahead"   # 결과 이름: 우선 지도 + 걸어 들어오는 사람도 보는 전문가
RUNS = {COMMITTED: COMMITTED, TRIAL: TRIAL, LOOKAHEAD: TRIAL}   # 결과 이름 → 지도
NODE = "S6_wausanro"
CROSSING = f":{NODE}_c0"
WALKINGAREAS = (f":{NODE}_w0", f":{NODE}_w1")
APPROACHES = ("-R112", "R112.112")   # S6로 들어오는 차도. R112.112가 버스 방향이고 극동방송 정류장이 있다
OUT = BASE / "results" / "s6_priority"

NEAR_M = 50.0         # 버스 앞범퍼에서 횡단보도까지 이 거리 안이면 S6에 다가가는 중이다(scenario_occurrence와 같다)
WAIT_NEAR_M = 15.0    # 이 안에서 멈추면 횡단보도 앞 정지다. 극동방송 정류장 정차 위치는 약 20m 앞이라 빠진다
STOPPED_MPS = 0.1
CREEP_MPS = 1.0       # 보행자 뒤를 따라 기어가는 것도 양보로 센다
PED_WAITING_MPS = 0.5  # env/sumo_env.py _crossing_occupied와 같은 기준
DWELL = OBSERVATION_FEATURES.index("stop_dwell_remaining")
OCCUPIED = OBSERVATION_FEATURES.index("crosswalk_occupied")


def roundtrip(net: Path, crossing: dict | None, out: Path) -> list[str]:
    """net을 netconvert로 한 번 더 변환한다. crossing을 주면 그 횡단보도 정의를 함께 넘긴다."""
    con = out.with_suffix(".con.xml")
    root = ET.Element("connections")
    if crossing:
        ET.SubElement(root, "crossing", crossing)
    ET.ElementTree(root).write(con, encoding="utf-8")
    done = subprocess.run([sumolib.checkBinary("netconvert"), "--sumo-net-file", str(net), "--connection-files", str(con),
                           *OPTIONS, "--output-file", str(out)], capture_output=True, text=True, timeout=300)
    if done.returncode:
        raise RuntimeError("netconvert 실패: " + done.stderr[-3000:])
    ET.ElementTree(ET.parse(out).getroot()).write(out, encoding="utf-8", xml_declaration=True)
    return out.read_text(encoding="utf-8").splitlines()


def make() -> Path:
    """커밋된 도로망에서 S6 우선 여부가 바꾸는 줄만 바꾼다.

    커밋된 도로망을 netconvert로 그냥 다시 변환해도 S6 밖이 바뀐다(3.12.0이 좌표를 직접 고친 J1 c2가 되돌아가고, 횡단보도
    다섯 곳의 outlineShape와 origBoundary 자릿수가 달라진다). 그래서 아무것도 안 바꾼 변환(noop)과 우선으로 바꾼 변환(prio)을
    둘 다 만들고, 둘이 다른 줄만 커밋된 파일에 옮긴다. 그 줄들은 커밋된 파일과 noop에서 같아야 한다.
    """
    folder = BASE / "maps" / TRIAL
    shutil.rmtree(folder, ignore_errors=True)
    shutil.copytree(BASE / "maps" / COMMITTED, folder)
    net = folder / "source" / "network.net.xml"
    edges = next(e for e in ET.parse(net).getroot().iter("edge") if e.get("id") == CROSSING).get("crossingEdges")
    raw = net.read_bytes()
    newline = b"\r\n" if b"\r\n" in raw else b"\n"   # 커밋된 도로망은 CRLF이고 끝 줄바꿈이 없다. 바이트를 그대로 지킨다
    committed = raw.decode("utf-8").split(newline.decode())
    with tempfile.TemporaryDirectory() as name:
        noop = roundtrip(net, None, Path(name) / "noop.net.xml")
        prio = roundtrip(net, {"node": NODE, "edges": edges, "priority": "true"}, Path(name) / "prio.net.xml")
    if not len(committed) == len(noop) == len(prio):
        raise ValueError("줄 수가 달라 줄 단위로 옮길 수 없다")
    changed = [i for i, (a, b) in enumerate(zip(noop, prio)) if a != b]
    if any(committed[i] != noop[i] for i in changed):
        raise ValueError("바뀌는 줄이 커밋된 파일과 noop 변환에서 다르다")
    trial = list(committed)
    for i in changed:
        trial[i] = prio[i]
    net.write_bytes(newline.decode().join(trial).encode("utf-8"))
    owner, head = {}, ""   # 줄 번호 → 그 줄을 품은 최상위 요소의 첫 줄
    for i, line in enumerate(committed):
        if line.startswith("    <"):
            head = line
        owner[i] = head
    strays = [i for i in changed if NODE not in owner[i]]
    if strays:
        raise ValueError("S6 밖의 줄이 바뀌었다: " + ", ".join(committed[i].strip() for i in strays))
    if tls_links(BASE / "maps" / COMMITTED / "source" / "network.net.xml") != tls_links(net):
        raise ValueError("신호 교차로의 연결 번호가 바뀌었다")
    state = next(c.get("state") for c in ET.parse(net).getroot().iter("connection")
                 if c.get("from") == WALKINGAREAS[0] and c.get("to") == CROSSING)
    if state != "M":
        raise ValueError(f"보도→횡단보도 연결이 우선(M)이 아니다: {state}")
    definition = json.loads((folder / "map.json").read_text(encoding="utf-8"))
    definition["version"] += "+s6prio"   # id는 그대로 둔다. 정차 정류장(road_config._SERVICE)이 id로 정해진다
    definition["network_source"]["sha256"] = digest(net)
    definition["network_source"]["patches"]["s6_priority_crossing"] = {
        "crossing": f"{NODE} {edges}", "priority": True, "script": "research/av_bc_rl/s6_priority_trial.py"}
    (folder / "map.json").write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{folder.relative_to(BASE)}: 바뀐 줄 {len(changed)}개(모두 {NODE}), 보도→횡단보도 {state}")
    for i in changed:
        print(f"{i + 1}: {committed[i].strip()}  ->  {trial[i].strip()}")
    return folder


def patch_lookahead() -> None:
    """우선 횡단보도 양끝 보도에서 다음 도로가 그 횡단보도인 사람도 '사람 있음'으로 본다."""
    from env import sumo_env
    original = sumo_env.SumoHighwayEnv._crossing_occupied

    def occupied(self, crossing_edge):
        if original(self, crossing_edge):
            return True
        if crossing_edge not in self.route_map.pedestrian_priority:
            return False
        areas = f"{crossing_edge.rsplit('_c', 1)[0]}_w"
        return any(sumo_env.traci.person.getRoadID(pid).startswith(areas)
                   and sumo_env.traci.person.getNextEdge(pid) == crossing_edge
                   for pid in sumo_env.traci.person.getIDList())

    sumo_env.SumoHighwayEnv._crossing_occupied = occupied


if os.environ.get("S6_LOOKAHEAD") == "1":   # spawn으로 뜬 수집 작업자도 이 모듈을 다시 읽으므로 맨 위에서 건다
    patch_lookahead()


def collect(episodes: int, workers: int, seed: int, max_steps: int, lookahead: bool) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ((LOOKAHEAD,) if lookahead else MAPS):
        args = ["--map", RUNS[name], "--policy", "expert", "--scenario", "busy", "--episodes", str(episodes),
                "--workers", str(workers), "--seed", str(seed), "--max-steps", str(max_steps),
                "--out-dir", str(OUT), "--name", f"expert_{name}"]
        if name == LOOKAHEAD:
            subprocess.run([sys.executable, __file__, "_collect", *args], cwd=BASE, check=True,
                           env={**os.environ, "S6_LOOKAHEAD": "1"})
        else:
            subprocess.run([sys.executable, "collect_pomdp_data.py", *args], cwd=BASE, check=True)


def episode_rows(path: Path):
    rows = []
    with gzip.open(path, "rt", encoding="utf-8") as lines:
        for line in lines:
            row = json.loads(line)
            if rows and row["episode"] != rows[-1]["episode"]:
                yield rows
                rows = []
            rows.append(row)
    if rows:
        yield rows


def runs(flags: list[bool], step_s: float) -> list[float]:
    """참이 이어진 구간마다 그 길이(초)."""
    out, length = [], 0
    for flag in [*flags, False]:
        if flag:
            length += 1
        elif length:
            out.append(length * step_s)
            length = 0
    return out


def episode_summary(rows: list[dict]) -> dict:
    step_s = 0.5
    last = rows[-1]
    near, creep, approach = [], [], []
    dwell_end = leave = None   # 극동방송 정차를 마친 시각, 그 뒤 횡단보도를 지나기까지 걸린 시간
    comfort = Counter()        # 정차 중이 아닌 S6 접근 동안의 승차감 위반과 SUMO 개입(요청 속도와 실제 속도가 다름)
    passes, crossing_seen, waiting_seen, flagged = 0, 0, 0, 0
    current = None   # 지금 다가가는 S6 통과 하나: [건너는 사람 봤나, 기다리는 사람 봤나, 관측 플래그]
    for row in rows:
        nav, ego = row["state"].get("navigation") or {}, row["state"]["ego"]
        at_s6 = str(nav.get("crossing_id")) == CROSSING and nav.get("distance_to_crossing") is not None
        approaching = at_s6 and nav["distance_to_crossing"] <= NEAR_M
        if approaching:
            if current is None:
                current, passes = [False, False, False], passes + 1
            peds = row["state"]["pedestrians"]
            current[0] |= any(p["road_id"] == CROSSING for p in peds)
            current[1] |= any(p["road_id"] in WALKINGAREAS and p["speed"] < PED_WAITING_MPS for p in peds)
            current[2] |= row["observation"][OCCUPIED] > 0.5
            if row["observation"][DWELL] > 0.0:
                dwell_end = row["sim_time"] + step_s
            else:
                comfort["accel_violation"] += row["comfort"]["acceleration_violation"]
                comfort["jerk_violation"] += row["comfort"]["jerk_violation"]
                comfort["sumo_intervened"] += row["action"]["speed_intervened"]
        elif current is not None:
            crossing_seen, waiting_seen, flagged = crossing_seen + current[0], waiting_seen + current[1], flagged + current[2]
            current = None
            if dwell_end is not None and leave is None:
                leave = row["sim_time"] - dwell_end
        approach.append(approaching)
        close = at_s6 and nav["distance_to_crossing"] <= WAIT_NEAR_M and row["observation"][DWELL] <= 0.0
        near.append(close and ego.get("speed", 1.0) < STOPPED_MPS)
        creep.append(close and ego.get("speed", 1.0) < CREEP_MPS)
    if current is not None:
        crossing_seen, waiting_seen, flagged = crossing_seen + current[0], waiting_seen + current[1], flagged + current[2]
    waits = runs(near, step_s)
    return {
        "episode": rows[0]["episode"], "reason": last["event"]["termination_reason"] or ("time_limit" if last["truncated"] else None),
        "duration_s": len(rows) * step_s, "stops_served": last["event"]["stops_served"], "stops_missed": last["event"]["stops_missed"],
        "ego_collision": any(r["event"]["collided"] for r in rows),
        "sim_collision_steps": sum(r["event"]["simulation_collided"] for r in rows),
        "s6_passes": passes, "s6_with_crossing_ped": crossing_seen, "s6_with_waiting_ped": waiting_seen,
        "s6_flagged": flagged, "s6_waits_s": waits, "s6_creep_s": sum(creep) * step_s, "s6_leave_s": leave,
        "s6_approach_s": sum(approach) * step_s, **{f"s6_{key}_steps": comfort[key] for key in
                                                     ("accel_violation", "jerk_violation", "sumo_intervened")},
    }


def collected(name: str) -> list[dict]:
    return [episode_summary(rows) for rows in episode_rows(OUT / f"expert_{name}.jsonl.gz")]


def background_run(map_id: str, seed: int, seconds: float, warmup: float) -> dict:
    import traci
    from env import road_builder
    from env.road_config import EGO, ROAD, TRAFFIC, configure_map
    configure_map("busy", seed, None, map_id)
    with tempfile.TemporaryDirectory() as temporary:
        cfg = road_builder.build(ROAD, EGO, TRAFFIC, temporary)
        label = f"bg_{map_id}_{seed}"
        traci.start([sumolib.checkBinary("sumo"), "-c", cfg, "--no-step-log", "true", "--no-warnings", "true",
                     "--duration-log.disable", "true"], label=label)
        sim = traci.getConnection(label)
        try:
            dt = sim.simulation.getDeltaT()
            wait, crossed = Counter(), set()
            halting = {edge: 0.0 for edge in APPROACHES}
            peak = dict.fromkeys(APPROACHES, 0)
            passed = {edge: set() for edge in APPROACHES}
            longest_vehicle_wait = 0.0
            collisions = Counter()
            step = departed = arrived = 0
            while sim.simulation.getTime() < warmup + seconds:
                sim.simulationStep()
                step += 1
                for collision in sim.simulation.getCollisions():
                    collisions["s6" if NODE in collision.lane else "other"] += 1
                if sim.simulation.getTime() < warmup:
                    continue
                departed += sim.simulation.getDepartedNumber()
                arrived += sim.simulation.getArrivedNumber()
                for area in WALKINGAREAS:
                    for person in sim.edge.getLastStepPersonIDs(area):
                        if sim.person.getSpeed(person) < STOPPED_MPS:
                            wait[person] += dt
                crossed.update(sim.edge.getLastStepPersonIDs(CROSSING))
                for edge in APPROACHES:
                    stopped = sim.edge.getLastStepHaltingNumber(edge)
                    halting[edge] += stopped * dt
                    peak[edge] = max(peak[edge], stopped)
                    passed[edge].update(sim.edge.getLastStepVehicleIDs(edge))
                    if step % 10 == 0 and stopped:
                        longest_vehicle_wait = max(longest_vehicle_wait, *(sim.vehicle.getWaitingTime(v)
                                                                           for v in sim.edge.getLastStepVehicleIDs(edge)))
            pending = len(sim.simulation.getPendingVehicles())
        finally:
            sim.close()
    waits = [wait[person] for person in crossed]
    return {"map": map_id, "seed": seed, "seconds": seconds, "crossers": len(crossed),
            "ped_wait_mean_s": round(statistics.fmean(waits), 2) if waits else 0.0,
            "ped_wait_p90_s": round(sorted(waits)[int(0.9 * (len(waits) - 1))], 2) if waits else 0.0,
            "ped_wait_max_s": round(max(waits), 2) if waits else 0.0,
            "ped_waited_share": round(sum(w > 0 for w in waits) / len(waits), 3) if waits else 0.0,
            "vehicles": {edge: len(passed[edge]) for edge in APPROACHES},
            "halting_s_per_vehicle": {edge: round(halting[edge] / max(len(passed[edge]), 1), 2) for edge in APPROACHES},
            "peak_halting": peak, "longest_vehicle_wait_s": round(longest_vehicle_wait, 1),
            "collisions": dict(collisions), "departed": departed, "arrived": arrived, "pending_at_end": pending}


def background(seeds: list[int], seconds: float, warmup: float, workers: int) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = [(map_id, seed) for seed in seeds for map_id in MAPS]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(background_run, *zip(*jobs), [seconds] * len(jobs), [warmup] * len(jobs)))
    (OUT / "background.json").write_text(json.dumps(results, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    for result in results:
        print(json.dumps(result, ensure_ascii=False))


def mean(values: list[float]) -> float | None:
    return round(statistics.fmean(values), 2) if values else None


def report() -> None:
    summary = {}
    names = [name for name in RUNS if (OUT / f"expert_{name}.jsonl.gz").exists()]
    for map_id in names:
        episodes = collected(map_id)
        waits = [w for e in episodes for w in e["s6_waits_s"]]
        arrived = [e["duration_s"] for e in episodes if e["reason"] == "arrived"]
        passes = sum(e["s6_passes"] for e in episodes)
        summary[map_id] = {
            "episodes": len(episodes), "termination": dict(Counter(e["reason"] for e in episodes)),
            "lap_s_mean": mean(arrived), "lap_s_max": max(arrived, default=None),
            "ego_collisions": sum(e["ego_collision"] for e in episodes),
            "sim_collision_steps": sum(e["sim_collision_steps"] for e in episodes),
            "stops_missed": sum(e["stops_missed"] for e in episodes),
            "s6_passes": passes,
            "s6_with_crossing_ped": sum(e["s6_with_crossing_ped"] for e in episodes),
            "s6_with_waiting_ped": sum(e["s6_with_waiting_ped"] for e in episodes),
            "s6_flagged_by_obs": sum(e["s6_flagged"] for e in episodes),
            "s6_wait_count": len(waits), "s6_wait_episodes": sum(bool(e["s6_waits_s"]) for e in episodes),
            "s6_wait_mean_s": mean(waits), "s6_wait_max_s": max(waits, default=None), "s6_wait_total_s": round(sum(waits), 1),
            "s6_creep_s_mean": mean([e["s6_creep_s"] for e in episodes]),
            "s6_creep_s_max": max(e["s6_creep_s"] for e in episodes),
            "s6_leave_s_mean": mean([e["s6_leave_s"] for e in episodes if e["s6_leave_s"] is not None]),
            "s6_leave_s_max": max((e["s6_leave_s"] for e in episodes if e["s6_leave_s"] is not None), default=None),
            "s6_approach_s_mean": mean([e["s6_approach_s"] for e in episodes]),
            **{key: sum(e[key] for e in episodes) for key in
               ("s6_accel_violation_steps", "s6_jerk_violation_steps", "s6_sumo_intervened_steps")},
            "per_episode": episodes,
        }
    path = OUT / "background.json"
    if path.exists():
        rows = json.loads(path.read_text(encoding="utf-8"))
        for map_id in MAPS:
            mine = [row for row in rows if row["map"] == map_id]
            summary[map_id]["background"] = {
                "runs": len(mine), "seeds": [row["seed"] for row in mine],
                "crossers_per_run": mean([row["crossers"] for row in mine]),
                "ped_wait_mean_s": mean([row["ped_wait_mean_s"] for row in mine]),
                "ped_wait_p90_s": mean([row["ped_wait_p90_s"] for row in mine]),
                "ped_wait_max_s": max(row["ped_wait_max_s"] for row in mine),
                "ped_waited_share": mean([row["ped_waited_share"] for row in mine]),
                **{f"vehicles {edge}": mean([row["vehicles"][edge] for row in mine]) for edge in APPROACHES},
                "departed": mean([row["departed"] for row in mine]), "arrived": mean([row["arrived"] for row in mine]),
                "pending_at_end": mean([row["pending_at_end"] for row in mine]),
                **{f"halting_s_per_vehicle {edge}": mean([row["halting_s_per_vehicle"][edge] for row in mine]) for edge in APPROACHES},
                **{f"peak_halting {edge}": max(row["peak_halting"][edge] for row in mine) for edge in APPROACHES},
                "longest_vehicle_wait_s": max(row["longest_vehicle_wait_s"] for row in mine),
                "collisions_s6": sum(row["collisions"].get("s6", 0) for row in mine),
                "collisions_other": sum(row["collisions"].get("other", 0) for row in mine),
            }
    base = {e["episode"]: e for e in summary[COMMITTED]["per_episode"]}
    for name in names:
        deltas = [e["duration_s"] - base[e["episode"]]["duration_s"] for e in summary[name]["per_episode"]
                  if e["reason"] == base[e["episode"]]["reason"] == "arrived"]
        summary[name]["lap_delta_s_mean"], summary[name]["lap_delta_s_max"] = mean(deltas), max(deltas, default=None)
    (OUT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print(f"{'':32s}" + "".join(f"{name:>28s}" for name in names))
    for key in summary[COMMITTED]:
        if key not in ("per_episode", "background"):
            print(f"{key:32s}" + "".join(f"{str(summary[name][key]):>28s}" for name in names))
    for key in summary[COMMITTED].get("background", {}):
        print(f"bg {key:29s}" + "".join(f"{str(summary[name]['background'][key]):>28s}" for name in MAPS))


def main() -> int:
    if sys.argv[1:2] == ["_collect"]:   # collect --lookahead가 부르는 수집기. 위에서 건 env 변경이 이 프로세스에 들어 있다
        sys.argv = ["collect_pomdp_data.py", *sys.argv[2:]]
        import collect_pomdp_data
        return collect_pomdp_data.main() or 0
    parser = argparse.ArgumentParser(description="S6 보행자 우선 임시 지도 실험")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("make")
    c = sub.add_parser("collect")
    c.add_argument("--episodes", type=int, default=20)
    c.add_argument("--workers", type=int, default=4)
    c.add_argument("--seed", type=int, default=900)
    c.add_argument("--max-steps", type=int, default=1200)
    c.add_argument("--lookahead", action="store_true", help="우선 지도에서 걸어 들어오는 사람도 보는 전문가만 달린다")
    b = sub.add_parser("background")
    b.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    b.add_argument("--seconds", type=float, default=3600.0)
    b.add_argument("--warmup", type=float, default=300.0)
    b.add_argument("--workers", type=int, default=4)
    sub.add_parser("report")
    sub.add_parser("clean")
    args = parser.parse_args()
    if args.command == "make":
        make()
    elif args.command == "collect":
        collect(args.episodes, args.workers, args.seed, args.max_steps, args.lookahead)
    elif args.command == "background":
        background(args.seeds, args.seconds, args.warmup, args.workers)
    elif args.command == "report":
        report()
    else:
        shutil.rmtree(BASE / "maps" / TRIAL, ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
