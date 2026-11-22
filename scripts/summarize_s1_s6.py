"""measure_s1_s6.py 결과 여러 시드를 시나리오별 바퀴당 비율로 모은다(평균, 표준편차, 최소~최대).

    python scripts/summarize_s1_s6.py busy             # results/traffic_verification/s1_s6_busy_<시드>.json 전부
    python scripts/summarize_s1_s6.py busy hiped       # 꼬리표가 붙은 결과

자연 발생률은 보장이 아니라 측정값이다. 시드마다 다르므로 범위와 함께 읽는다. 충돌·순간이동·반경 안 출현·소멸은 실행당 횟수다.
"""
import glob
import json
import math
import os
import statistics
import sys

DIR = os.path.join(os.path.dirname(__file__), "..", "results", "traffic_verification")
MIDBLOCK_S = 3.0                # 길 가운데 이만큼 이상 서 있으면 한 번으로 센다


def per_lap(d, xs):
    laps = max(1, d["laps_done"])
    return sum(xs[:laps]) / laps


def row(d):
    s1 = d["S1"]
    laps = max(1, d["laps_done"])
    return {
        "laps_done": d["laps_done"],
        "lap_s": statistics.mean(d["lap_seconds"]) if d["lap_seconds"] else float("nan"),
        "S1_stops/lap": per_lap(d, s1["bus_stops_then_departs"]),
        "S1_rear/lap": per_lap(d, s1["rear_vehicle_events"]),
        "S1_overtook/lap": s1["rear_outcomes"].get("overtook", 0) / laps,
        "S2_attempts/lap": per_lap(d, d["S2"]["attempts"]),
        "S2_partner/lap": per_lap(d, d["S2"]["with_a1_follower"]),
        "S2_J82_partner/lap": per_lap(d, d["S2_by_junction"]["J82"]["with_a1"]),
        "S2_J1_partner/lap": per_lap(d, d["S2_by_junction"]["J1"]["with_a1"]),
        "S3_attempts/lap": per_lap(d, d["S3"]["attempts"]),
        "S3_partner/lap": per_lap(d, d["S3"]["with_a1_follower"]),
        "S4/lap": per_lap(d, d["S4"]["blocks_per_lap"]),
        "S5/lap": per_lap(d, d["S5"]["per_lap"]),
        "S6/lap": per_lap(d, d["S6"]["per_lap"]),
        "collision_events": d["safety"]["collision_events"],
        "teleports": d["safety"]["teleports"],
        "presence_appear": d["presence"]["appear"],
        "presence_vanish": d["presence"]["vanish"],
        "presence_teleport": d["presence"]["teleport"],
        "S6_max_wait_s": d["S6_vehicle_waiting"]["max_wait_s"],
        "S6_over20s": d["S6_vehicle_waiting"]["vehicles_over_20s"],
        "bus_forced_stops": len(d["bus"]["forced"]),
        "bus_missed": len(d["bus"]["missed"]),
        **still_rows(d.get("bus_still")),
    }


def still_rows(still):
    """버스가 정류장 밖에서 앞차도 적색도 없이 선 구간(measure_s1_s6.py의 bus_still). 그 기록이 없는 예전 결과는 nan이다."""
    if still is None:
        return dict.fromkeys(("bus_midblock_n", "bus_midblock_s", "bus_midblock_max_s", "bus_stopline_s"), float("nan"))
    mid = [e["s"] for e in still["midblock"]]
    return {"bus_midblock_n": sum(x >= MIDBLOCK_S for x in mid), "bus_midblock_s": sum(mid),
            "bus_midblock_max_s": max(mid, default=0.0), "bus_stopline_s": sum(e["s"] for e in still["stopline"])}


def main():
    scen = sys.argv[1] if len(sys.argv) > 1 else "busy"
    tag = sys.argv[2] if len(sys.argv) > 2 else ""
    pattern = os.path.join(DIR, f"s1_s6_{scen}_*{'_' + tag if tag else ''}.json")
    rows = {}
    for f in sorted(glob.glob(pattern)):
        seed = os.path.basename(f)[len(f"s1_s6_{scen}_"):-len(".json")]
        if (seed.split("_", 1) + [""])[1] != tag:
            continue
        rows[int(seed.split("_", 1)[0])] = row(json.load(open(f, encoding="utf-8")))
    if not rows:
        sys.exit(f"결과가 없다: {pattern}")
    seeds = sorted(rows)
    print("metric".ljust(20) + "".join(f"s{s}".rjust(8) for s in seeds) + "     mean      sd    min~max")
    for k in rows[seeds[0]]:
        xs = [rows[s][k] for s in seeds]
        if any(math.isnan(x) for x in xs):             # 끝난 바퀴가 없거나 그 기록이 없는 예전 결과
            print(k.ljust(20) + "".join(f"{x:8.2f}" for x in xs))
            continue
        sd = statistics.stdev(xs) if len(xs) > 1 else 0.0
        print(k.ljust(20) + "".join(f"{x:8.2f}" for x in xs)
              + f"  {statistics.mean(xs):7.2f} {sd:7.2f}   {min(xs):.2f}~{max(xs):.2f}")


if __name__ == "__main__":
    main()
