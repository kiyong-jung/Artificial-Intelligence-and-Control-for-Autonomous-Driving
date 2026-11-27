"""버스 없이 배경 교통만 돌려 교통량별로 시뮬레이션이 건강한지 잰다. 버스 위치와 무관한 도로 전체 측정이다.

    python research/av_bc_rl/sim_health.py --scenarios heavy x2 x3 --seeds 3 --seed 60000 --out results/sim_health.json

한 번은 warmup(최대 360초)과 한 바퀴 시간 제한(700초)을 덮는 seconds 동안 돈다. 10초마다 센다.
  inserted_share   요청한 차 중 실제로 도로에 들어간 비율(나머지는 진입로 대기열 backlog)
  backlog_end      끝날 때 아직 못 들어간 차 수
  running          도로 위 차 수(마지막 300초 평균)
  standing_120s    120초 넘게 한 번도 0.1m/s를 넘지 못한 차 수(마지막 300초 평균). 커지기만 하면 gridlock이다
  collisions       배경 차끼리 충돌 보고 수. collision_pairs는 서로 다른 (충돌한 차, 맞은 차) 쌍의 수
  teleports        SUMO 순간이동 수(time-to-teleport -1이라 0이어야 한다)
  standing_roads   120초 넘게 선 차가 있던 도로와 그 차 수(마지막 300초 평균). 어느 교차로가 막히는지 본다
  pending_alley    골목 출발 도로별 투입 대기 차 수(마지막 300초 평균)
  taxi_stops       정류장 앞 택시(taxi_)가 정차를 시작한 횟수(시간당). person_collision_pairs는 보행자가 낀 충돌 쌍
  arrived_per_h    도착한 차(시간당). 수요를 키워도 늘지 않으면 도로가 포화다
  route_near/mid   버스 경로 위 차 밀도(대/차로-km). near는 다음 신호 정지선 60m 안, mid는 그 밖(블록 중간)
"""
import argparse
import collections
import json
import multiprocessing
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
NEAR = 60.0


def route_zones(net, edges):
    """버스 경로의 각 도로에서 도로 끝 너머 다음 신호 정지선까지 남은 거리와, near·mid 차로 길이(m)."""
    beyond, following = {}, 1e9   # 순환 노선이라 두 바퀴 돌려 처음 도로의 값도 채운다
    for edge_id in list(reversed(edges)) * 2:
        edge = net.getEdge(edge_id)
        beyond[edge_id] = 0.0 if edge.getToNode().getType().startswith("traffic_light") else following
        following = beyond[edge_id] + edge.getLength()
    near_m = mid_m = 0.0
    for edge_id in edges:
        edge = net.getEdge(edge_id)
        length = edge.getLength()
        near_len = float(np.clip(NEAR - beyond[edge_id], 0.0, length))
        near_m += near_len * len(edge.getLanes())
        mid_m += (length - near_len) * len(edge.getLanes())
    return beyond, near_m, mid_m


def probe(job):
    scenario, seed, seconds, label = job
    import traci
    from sumolib import checkBinary
    from sumolib.net import readNet
    from env.road_builder import build
    from env.road_config import EGO, ROAD, TRAFFIC, configure_map

    configure_map(scenario, seed=seed, map_id="hapjeong")
    out = tempfile.mkdtemp(prefix=f"health_{scenario}_{seed}_")
    cfg = build(ROAD, EGO, TRAFFIC, out)
    net = readNet(os.path.join(out, "network.net.xml"))
    edges = [e for e in ROAD["network"]["routes"][ROAD["ego_route"]] if not e.startswith(":")]
    beyond, near_m, mid_m = route_zones(net, edges)
    edge_set = set(edges)
    traci.start([checkBinary("sumo"), "-c", cfg, "--no-step-log", "--no-warnings", "--seed", str(seed)], label=label)
    conn = traci.getConnection(label)
    loaded = departed = arrived = collisions = teleports = taxi_stops = 0
    pairs = set()   # 같은 충돌이 여러 스텝 보고될 수 있어 (충돌한 차, 맞은 차) 쌍도 센다
    samples = []
    steps_per_sample = int(round(10.0 / float(conn.simulation.getDeltaT())))
    for step in range(int(seconds / conn.simulation.getDeltaT())):
        conn.simulationStep()
        loaded += conn.simulation.getLoadedNumber()
        departed += conn.simulation.getDepartedNumber()
        arrived += conn.simulation.getArrivedNumber()
        teleports += conn.simulation.getStartingTeleportNumber()
        taxi_stops += sum(v.startswith("taxi_") for v in conn.simulation.getStopStartingVehiclesIDList())
        for collision in conn.simulation.getCollisions():
            collisions += 1
            pairs.add((collision.collider, collision.victim))
        if (step + 1) % steps_per_sample:
            continue
        ids = conn.vehicle.getIDList()
        standing_ids = [v for v in ids if conn.vehicle.getWaitingTime(v) >= 120.0]
        standing = len(standing_ids)
        standing_roads = collections.Counter(conn.vehicle.getRoadID(v) for v in standing_ids)
        pending_ids = conn.simulation.getPendingVehicles()
        pending_alley = collections.Counter(v.split("_to_")[0][len("alley_"):] for v in pending_ids if v.startswith("alley_"))
        near = mid = 0
        for v in ids:
            road = conn.vehicle.getRoadID(v)
            if road in edge_set:
                left = net.getEdge(road).getLength() - conn.vehicle.getLanePosition(v) + beyond[road]
                near, mid = (near + 1, mid) if left < NEAR else (near, mid + 1)
        samples.append(dict(t=conn.simulation.getTime(), running=len(ids), pending=len(pending_ids),
                            standing=standing, standing_roads=standing_roads, pending_alley=pending_alley, near=near, mid=mid, loaded=loaded, departed=departed, arrived=arrived))
    conn.close()
    tail = [s for s in samples if s["t"] > seconds - 300]
    def tail_mean(key):
        return {k: round(v / len(tail), 2) for k, v in sum((s[key] for s in tail), collections.Counter()).items()}

    people = ("ped", "s6ped")
    return dict(scenario=scenario, seed=seed, collisions=collisions, collision_pairs=len(pairs), pairs=sorted(pairs),
                person_collision_pairs=sum(a.startswith(people) or b.startswith(people) for a, b in pairs),
                taxi_stops_per_h=taxi_stops * 3600.0 / seconds,
                teleports=teleports, standing_roads=tail_mean("standing_roads"), pending_alley=tail_mean("pending_alley"),
                loaded=loaded, departed=departed,
                inserted_share=departed / max(loaded, 1), backlog_end=samples[-1]["pending"],
                running=float(np.mean([s["running"] for s in tail])),
                standing_120s=float(np.mean([s["standing"] for s in tail])),
                standing_120s_end=samples[-1]["standing"],
                arrived_per_h=arrived * 3600.0 / seconds,
                route_near=float(np.mean([s["near"] for s in tail])) / near_m * 1000.0,
                route_mid=float(np.mean([s["mid"] for s in tail])) / mid_m * 1000.0,
                route_near_lane_km=near_m / 1000.0, route_mid_lane_km=mid_m / 1000.0,
                series=[{k: s[k] for k in ("t", "running", "pending", "standing")} for s in samples[::6]])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenarios", nargs="+", required=True)
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--seed", type=int, default=60000)
    parser.add_argument("--seconds", type=float, default=1100.0)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    jobs = [(s, args.seed + k, args.seconds, f"h{i}_{k}") for i, s in enumerate(args.scenarios) for k in range(args.seeds)]
    with multiprocessing.Pool(min(args.workers, len(jobs))) as pool:
        runs = pool.map(probe, jobs)
    table = {}
    for scenario in args.scenarios:
        mine = [r for r in runs if r["scenario"] == scenario]
        table[scenario] = {key: round(float(np.mean([r[key] for r in mine])), 3) for key in (
            "inserted_share", "backlog_end", "running", "standing_120s", "standing_120s_end", "collisions", "collision_pairs",
            "arrived_per_h", "route_near", "route_mid", "teleports", "person_collision_pairs", "taxi_stops_per_h")}
        table[scenario]["runs"] = len(mine)
        print(scenario, table[scenario], flush=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "table": table, "runs": runs}, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
