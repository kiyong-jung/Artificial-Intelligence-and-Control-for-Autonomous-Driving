"""배경 차의 옆 이동 설정안을 같은 교통(같은 seed)에서 비교한다. 학습한 모델 없이 도는 빠른 시험이다.

버스는 SUMO 기본 운전으로 노선을 돌고, 정류장에서는 학습한 버스처럼 속도 명령 0으로 8초 선다(SUMO의 정차 기능을 쓰지
않으므로 뒤차는 버스가 정류장에 선 줄 모른다). 모든 배경 차에서 두 가지를 센다.

  걸침  거의 선 채(0.5m/s 미만) 차로 가운데에서 0.75m 넘게 벗어나 2초 넘게 있던 일
  돌아감  거의 선 채 차로 방향에서 15도 넘게 돌아가 2초 넘게 있던 일

    PYTHONPATH=. python research/traffic_checks/lateral_screen.py rich2 63000 63001 --variants now stand03 early
"""
import argparse
import math
import tempfile
from multiprocessing import Pool

SECONDS, ENTER, DWELL = 1320.0, 360.0, 8.0
# 설정안: lateral은 lateral_model에, lane_change는 배경 차 vType 속성에 덮어쓴다.
VARIANTS = {
    "now": {},
    "stand03": {"lateral": {"standing_speed": 0.3}},
    "stand10": {"lateral": {"standing_speed": 1.0}},
    "sumo_default": {"lateral": {"standing_speed": 1.0, "speed_factor": 1.0}},
    "factor05": {"lateral": {"speed_factor": 0.5}},
    "early": {"lane_change": {"lcSpeedGainLookahead": 10, "lcSpeedGain": 3}},
    "no_gain": {"lane_change": {"lcSpeedGain": 0}},
    "gap02": {"lane_change": {"minGapLat": 0.2}},
    "early_stand03": {"lateral": {"standing_speed": 0.3}, "lane_change": {"lcSpeedGainLookahead": 10, "lcSpeedGain": 3}},
    "factor05_stand03": {"lateral": {"standing_speed": 0.3, "speed_factor": 0.5}},
    "factor05_acc": {"lateral": {"speed_factor": 0.5, "accel": 2.0}},
    "factor10_acc": {"lateral": {"speed_factor": 1.0, "accel": 2.0}},
    "factor05_acc_early": {"lateral": {"speed_factor": 0.5, "accel": 2.0},
                           "lane_change": {"lcSpeedGainLookahead": 10, "lcSpeedGain": 3}},
    "nogain_stand03": {"lateral": {"standing_speed": 0.3}, "lane_change": {"lcSpeedGain": 0}},
    "nogain_factor05_acc": {"lateral": {"speed_factor": 0.5, "accel": 2.0}, "lane_change": {"lcSpeedGain": 0}},
    "gain03": {"lane_change": {"lcSpeedGain": 0.3}},
    "gain03_factor05_acc": {"lateral": {"speed_factor": 0.5, "accel": 2.0}, "lane_change": {"lcSpeedGain": 0.3}},
    "sublane0": {"lane_change": {"lcSublane": 0}},
    "lookahead0": {"lane_change": {"lcSpeedGainLookahead": 0}},
    "factor05_acc_stand01": {"lateral": {"speed_factor": 0.5, "accel": 2.0, "standing_speed": 0.1}},
    "factor05_acc_gap02": {"lateral": {"speed_factor": 0.5, "accel": 2.0}, "lane_change": {"minGapLat": 0.2}},
}


def streaks(flags, step, least=2.0):
    """차마다 이어진 True 구간 가운데 least초 넘는 것의 길이(초) 목록."""
    out = []
    for series in flags.values():
        run = 0
        for on in series + [False]:
            if on:
                run += 1
            else:
                if run * step >= least:
                    out.append(run * step)
                run = 0
    return out


def run(job):
    scenario, seed, variant = job
    import traci
    from sumolib import checkBinary
    from env.road_builder import build
    from env.road_config import EGO, ROAD, TRAFFIC, configure_map

    configure_map(scenario, seed=seed, map_id="hapjeong")
    change = VARIANTS[variant]
    TRAFFIC["lateral_model"] = {**TRAFFIC["lateral_model"], **change.get("lateral", {})}
    TRAFFIC["background_lane_change"] = {**TRAFFIC["background_lane_change"], **change.get("lane_change", {})}
    cfg = build(ROAD, EGO, TRAFFIC, tempfile.mkdtemp(prefix=f"lat_{variant}_{seed}_"))
    label = f"lat_{variant}_{seed}"
    traci.start([checkBinary("sumo"), "-c", cfg, "--no-step-log", "--no-warnings", "--seed", str(seed)], label=label)
    conn = traci.getConnection(label)
    step = conn.simulation.getDeltaT()
    stops = {s["lane"]: (s["startPos"], s["endPos"]) for s in ROAD["network"]["bus_stops"] if s["id"] in EGO["service"]["stops"]}
    lane_shape = {}
    hold_until, served = None, set()
    straddle, rotated, behind = {}, {}, {}
    arrived = collisions = teleports = 0
    speed_sum = speed_n = 0
    every = max(1, round(0.5 / step))
    stops_made = 0
    for tick in range(int(SECONDS / step)):
        now = conn.simulation.getTime()
        if abs(now - ENTER) < 1e-6:
            conn.vehicle.add("ego", ROAD["ego_route"], typeID="ego_type", departPos="0")
        conn.simulationStep()
        arrived += conn.simulation.getArrivedNumber()
        collisions += conn.simulation.getCollidingVehiclesNumber()
        teleports += conn.simulation.getStartingTeleportNumber()
        ids = conn.vehicle.getIDList()
        bus = None
        if "ego" in ids:
            lane, pos = conn.vehicle.getLaneID("ego"), conn.vehicle.getLanePosition("ego")
            bx, by = conn.vehicle.getPosition("ego")
            bh = math.radians(conn.vehicle.getAngle("ego"))
            bus = (bx, by, bh)
            if hold_until is not None and now >= hold_until:
                conn.vehicle.setSpeed("ego", -1)
                hold_until = None
            elif hold_until is None and lane in stops and lane not in served and stops[lane][0] + 12 <= pos <= stops[lane][1]:
                conn.vehicle.setSpeed("ego", 0)
                served.add(lane)
                stops_made += 1
                hold_until = now + DWELL + 4
            if lane not in stops and len(served) == len(stops):
                served = set()
        if tick % every:
            continue
        for vid in ids:
            if vid == "ego":
                continue
            v = conn.vehicle.getSpeed(vid)
            speed_sum += v
            speed_n += 1
            slow = v < 0.5
            lane = conn.vehicle.getLaneID(vid)
            off = slow and abs(conn.vehicle.getLateralLanePosition(vid)) > 0.75
            turn = False
            if slow and not lane.startswith(":"):
                if lane not in lane_shape:
                    lane_shape[lane] = conn.lane.getShape(lane)
                x, y = conn.vehicle.getPosition(vid)
                shape = lane_shape[lane]
                (x0, y0), (x1, y1) = min(zip(shape, shape[1:]), key=lambda s: _seg_dist(x, y, s))
                lane_angle = math.degrees(math.atan2(x1 - x0, y1 - y0))
                turn = abs((conn.vehicle.getAngle(vid) - lane_angle + 180) % 360 - 180) > 15
            straddle.setdefault(vid, []).append(slow and off and not lane.startswith(":"))
            rotated.setdefault(vid, []).append(turn)
            near = False
            if bus and slow and off:
                x, y = conn.vehicle.getPosition(vid)
                dx, dy = x - bus[0], y - bus[1]
                lon = dx * math.sin(bus[2]) + dy * math.cos(bus[2])
                lat = dx * math.cos(bus[2]) - dy * math.sin(bus[2])
                near = -30 < lon < -12 and abs(lat) < 3
            behind.setdefault(vid, []).append(near)
    conn.close()
    step *= every
    s, r, b = streaks(straddle, step), streaks(rotated, step), streaks(behind, step)
    return dict(variant=variant, seed=seed, straddle=len(s), straddle_s=sum(s), straddle_max=max(s, default=0),
                rotated=len(r), rotated_s=sum(r), behind_bus=len(b), arrived=arrived, collisions=collisions,
                teleports=teleports, stops_made=stops_made, mean_speed=speed_sum / max(1, speed_n))


def _seg_dist(x, y, seg):
    (x0, y0), (x1, y1) = seg
    dx, dy = x1 - x0, y1 - y0
    t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / max(1e-9, dx * dx + dy * dy)))
    return math.hypot(x - x0 - t * dx, y - y0 - t * dy)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("seeds", nargs="+", type=int)
    parser.add_argument("--variants", nargs="+", default=list(VARIANTS))
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    jobs = [(args.scenario, seed, variant) for variant in args.variants for seed in args.seeds]
    with Pool(args.workers, maxtasksperchild=1) as pool:
        rows = pool.map(run, jobs, chunksize=1)
    print("설정안\tseed수\t걸침\t걸침초\t최장\t돌아감\t돌아감초\t버스뒤걸침\t버스정차\t도착\t충돌\t텔레포트\t평균속도")
    for variant in args.variants:
        mine = [r for r in rows if r["variant"] == variant]
        total = lambda key: sum(r[key] for r in mine)
        print(f"{variant}\t{len(mine)}\t{total('straddle')}\t{total('straddle_s'):.0f}\t{max(r['straddle_max'] for r in mine):.0f}\t"
              f"{total('rotated')}\t{total('rotated_s'):.0f}\t{total('behind_bus')}\t{total('stops_made')}\t{total('arrived')}\t{total('collisions')}\t"
              f"{total('teleports')}\t{total('mean_speed') / len(mine):.2f}")


if __name__ == "__main__":
    main()
