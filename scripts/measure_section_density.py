"""구간별 밀도·신호 앞 대기·버스 주변 순간이동을 잰다(DensityKeeper 전후 비교용).

    python scripts/measure_section_density.py --scenario baseline --seed 0 --laps 3 [--no-density] [--out 결과.json]

시험용 버스(traffic/test_bus.py)가 노선을 --laps 바퀴 도는 동안 1초마다
  - 양화로·독막로·잔다리로·와우산로 구간의 100 m당 차량 수(per_road),
  - 신호 교차로로 끝나는 구간의 멈춘 차 수(queue),
를 모으고, 버스 반경 안 출현·소멸·정체 순간이동(presence), 충돌, 버스 바퀴 시간을 함께 적는다.
--no-density는 버스 주변 키퍼(BusWindowKeeper)를 끈 기준선이다. 구간별 밀도(per_road)는 구간 전체 차량 수를 100 m당으로 환산한 값이다(차로 합).
"""
import argparse
import collections
import json
import os
import statistics
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
import run_hapjeong_scenario as RS  # noqa: E402
from maps import load_map  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.bus_window_keeper import DEFAULT_LANE_DENSITY  # noqa: E402
from traffic.density_keeper import sections_of  # noqa: E402
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.presence import PresenceMonitor  # noqa: E402
from traffic.test_bus import TestBusPilot  # noqa: E402


T0 = time.time()


class Done(Exception):
    pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=("quiet", "baseline", "busy"), default="baseline")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--tmax", type=float, default=2600.0, help="버스가 이 시간(초) 안에 못 끝내면 멈춘다")
    ap.add_argument("--warmup", type=float, default=120.0)
    ap.add_argument("--no-density", action="store_true", help="버스 주변 키퍼(BusWindowKeeper)를 끈다(기준선)")
    ap.add_argument("--scale", type=float, default=1.0, help="목표 밀도(대/100 m/차로)에 곱하는 배율")
    ap.add_argument("--route-boost", type=float, default=1.3, help="노선 구간을 지나는 배경 흐름의 배율(지나지 않는 흐름을 줄여 전체 대수 유지). 1.0이면 끈다")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    tag = ("off" if a.no_density else ("on" + (f"_x{a.scale:g}" if a.scale != 1.0 else ""))) + (f"_rb{a.route_boost:g}" if a.route_boost != 1.3 else "")
    out = a.out or os.path.join(os.path.dirname(__file__), "..", "results", f"density_{a.scenario}_{a.seed}_{tag}.json")
    out_dir = os.path.join(tempfile.gettempdir(), f"dens_{a.scenario}_{a.seed}_{tag}")

    dens = {k: v * a.scale for k, v in DEFAULT_LANE_DENSITY.items()}
    world = HapjeongTraffic(out_dir, scenario=a.scenario, seed=a.seed, bus_traffic=True,
                            lane_density=None if a.no_density else dens, route_flow_boost=a.route_boost)
    world.build()
    tc = world.start(warmup=a.warmup)
    m = load_map("hapjeong")
    loop = m["routes"]["bus_loop"]
    net = read_net(os.path.join(out_dir, "network.net.xml"))
    bus = TestBusPilot(tc, net, loop, RS.map_stops(m))
    presence = PresenceMonitor(tc, net, "bus0")
    probe = world.keeper                                 # 창 안 밀도를 재는 키퍼. 키퍼를 끈 실행에서는 갱신하지 않는 측정용 사본을 쓴다
    if probe is None:
        world.lane_density = dens
        probe = world._make_keeper("bus0")
        world.lane_density = None
    win = collections.defaultdict(list)                  # 창 안 차로 번호 -> (대수, 목표) 표본
    sections = sections_of(net, bus.lap_edges)
    per_road = collections.defaultdict(list)          # 도로 이름 -> 구간 밀도 표본(대/100 m)
    queue = collections.defaultdict(list)             # 신호 끝 구간 -> 멈춘 차 수 표본
    safety = {"collisions": 0, "teleports": 0, "examples": []}

    def on_step(t):
        presence.update(t)
        for c in tc.simulation.getCollisions():
            safety["collisions"] += 1
            if len(safety["examples"]) < 10:
                safety["examples"].append((round(t), c.type, c.collider, c.victim, c.lane))
        tel = tc.simulation.getStartingTeleportIDList()
        safety["teleports"] += len(tel)
        if tel and len(safety["examples"]) < 20:
            safety["examples"].append((round(t), "teleport", list(tel)[:3], [tc.vehicle.getRoadID(v) for v in list(tel)[:3]]))
        if int(round(t * 10)) % 500 == 0:
            ids = tc.vehicle.getIDList()
            kinds = collections.Counter(("keep" if v.startswith("keep") else "interior" if v.startswith("interior") else "inj" if v.startswith("inj") else "other") for v in ids)
            halted = collections.Counter(tc.vehicle.getRoadID(v) for v in ids if tc.vehicle.getSpeed(v) < 0.1)
            print(f"t={t:.0f} 차량 {len(ids)} {dict(kinds)} 멈춤 {sum(halted.values())} 상위 {halted.most_common(5)} 바퀴 {len(bus.lap_times)} 경과 {time.time() - T0:.0f}s", flush=True)
        if int(round(t * 10)) % 10 == 0:
            try:
                road = tc.vehicle.getRoadID("bus0")
                if road in probe.index:
                    for k, (n, tgt) in probe.window_density(probe.index[road], tc.vehicle.getLanePosition("bus0")).items():
                        win[k].append((n, tgt))
            except Exception:
                pass
            for s in sections:
                per_road[s.name].append(tc.edge.getLastStepVehicleNumber(s.edge) / s.length * 100.0)
                if s.signalized_end:
                    queue[s.edge].append(tc.edge.getLastStepHaltingNumber(s.edge))
        if len(bus.lap_times) >= a.laps or t > a.tmax:
            raise Done()

    try:
        world.run(tc.simulation.getTime() + a.tmax + 10, bus_id="bus0", bus_hook=bus.tick, on_step=on_step,
                  pilot=bus, triggers="scenario", scenario_inject=True)
    except Done:
        pass
    res = {
        "scenario": a.scenario, "seed": a.seed, "density": not a.no_density,
        "per_road": {n: {"mean_per_100m": round(statistics.fmean(v), 2),
                         "p10_per_100m": round(sorted(v)[len(v) // 10], 2), "samples": len(v),
                         "target_per_lane_per_100m": None if a.no_density else dens.get(n)}
                     for n, v in per_road.items() if v},
        "window": {str(k): {"mean_count": round(statistics.fmean(n for n, _ in v), 2), "mean_target": round(statistics.fmean(g for _, g in v), 2),
                             "share_at_or_above_target": round(sum(1 for n, g in v if n >= g) / len(v), 3), "samples": len(v)}
                   for k, v in sorted(win.items()) if v},
        "queue": {e: {"max_halting": max(v), "mean_halting": round(statistics.fmean(v), 2)} for e, v in queue.items() if v},
        "presence": presence.summary(),
        "safety": safety,
        "bus": {"laps_done": len(bus.lap_times), "lap_seconds": [round(x) for x in bus.lap_times],
                "forced": bus.forced, "missed": bus.missed, "changes": bus.changes},
        "density_summary": {"keeper": world.keeper.summary() if world.keeper else None},
    }
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: res[k] for k in ("per_road", "window", "presence", "safety", "bus", "density_summary")},
                     ensure_ascii=False, indent=1)[:3000])
    world.close()


if __name__ == "__main__":
    main()
