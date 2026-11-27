"""배경 차가 버스를 보고 만들어지는지 본다. 같은 seed로 버스 없이 한 번, 버스(SUMO 기본 운전)를 넣어 한 번 돌려
차마다 출발 시각과 경로를 비교한다. 버스 때문에 생기거나 경로가 바뀌는 차가 있으면 여기서 드러난다.

    PYTHONPATH=. python research/snail/bus_independence.py rich2 63000
"""
import os
import sys
import tempfile
from multiprocessing import Pool

SECONDS, ENTER = 960.0, 360.0


def run(job):
    scenario, seed, with_bus = job
    import traci
    from sumolib import checkBinary
    from env.road_builder import build
    from env.road_config import EGO, ROAD, TRAFFIC, configure_map

    configure_map(scenario, seed=seed, map_id="hapjeong")
    cfg = build(ROAD, EGO, TRAFFIC, tempfile.mkdtemp(prefix=f"indep_{scenario}_{seed}_{with_bus}_"))
    label = f"indep{with_bus}"
    traci.start([checkBinary("sumo"), "-c", cfg, "--no-step-log", "--no-warnings", "--seed", str(seed)], label=label)
    conn = traci.getConnection(label)
    departed = {}
    for step in range(int(SECONDS / conn.simulation.getDeltaT())):
        if with_bus and abs(conn.simulation.getTime() - ENTER) < 1e-6:
            conn.vehicle.add("ego", ROAD["ego_route"], typeID="ego_type", departPos="0")
        conn.simulationStep()
        for vid in conn.simulation.getDepartedIDList():
            if vid != "ego":
                departed[vid] = (conn.simulation.getTime(), tuple(conn.vehicle.getRoute(vid)), conn.vehicle.getTypeID(vid))
    conn.close()
    return departed


def main():
    scenario, seed = sys.argv[1], int(sys.argv[2])
    with Pool(2) as pool:
        without, with_ = pool.map(run, [(scenario, seed, False), (scenario, seed, True)])
    both = set(without) & set(with_)
    print(f"{scenario} seed {seed}: 버스 없이 출발한 차 {len(without)}, 버스 넣고 출발한 차 {len(with_)}, 둘 다 {len(both)}")
    print("버스 넣었을 때만 있는 차", sorted(set(with_) - set(without))[:20])
    print("버스 없을 때만 있는 차", sorted(set(without) - set(with_))[:20])
    print("경로나 차종이 다른 차", sum(without[v][1:] != with_[v][1:] for v in both))
    diffs = [with_[v][0] - without[v][0] for v in both]
    print("출발 시각이 같은 차", sum(abs(d) < 1e-6 for d in diffs), "/", len(diffs))
    late = sorted(((with_[v][0] - without[v][0], v, without[v][0], without[v][1][0]) for v in both if abs(with_[v][0] - without[v][0]) > 1e-6), key=lambda p: -abs(p[0]))
    for d, v, t, edge in late[:15]:
        print(f"  {d:+6.1f}초  {v}  (버스 없이 {t:.1f}초, 출발 도로 {edge})")


if __name__ == "__main__":
    main()
