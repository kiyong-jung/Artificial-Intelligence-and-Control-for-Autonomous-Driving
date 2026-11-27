"""sim_health.py와 traffic_around_bus.py 결과를 교통량 한 줄씩의 표(Markdown)로 묶는다.

    python research/av_bc_rl/sweep_table.py --health research/av_bc_rl/metrics/sim_health_sweep.json \
        --around research/av_bc_rl/metrics/around_bus_sweep.json --policies v7 it16
"""
import argparse
import json


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--health", nargs="+", required=True)
    parser.add_argument("--around", required=True, help="그룹 이름이 <정책>:<교통량>인 traffic_around_bus.py 결과")
    parser.add_argument("--policies", nargs="+", default=["v7", "it16"])
    args = parser.parse_args()
    health = {}
    for path in args.health:
        health.update(json.load(open(path, encoding="utf-8"))["table"])
    around = json.load(open(args.around, encoding="utf-8"))["table"]
    scenarios = list(dict.fromkeys(name.split(":", 1)[1] for name in around))
    print("| 교통량 | 넣은 비율 | backlog | 도로 위 차 | 120초+ 정지 차 | 배경 충돌/1100초 | 경로 밀도 near/mid (대/차로-km) |")
    print("|---|---|---|---|---|---|---|")
    for scenario in scenarios:
        h = health.get(scenario)
        if h:
            print(f"| {scenario} | {h['inserted_share']:.0%} | {h['backlog_end']:.0f} | {h['running']:.0f} | "
                  f"{h['standing_120s']:.1f} | {h['collisions']:.1f} | {h['route_near']:.1f} / {h['route_mid']:.1f} |")
    print()
    print("| 정책 | 교통량 | 바퀴 | 완주 | 끝난 이유 | 멈춘 곳 | 바퀴 시간(s) | 50m 안 차 near/mid | 앞 30m 차 near/mid "
          "| 정차(뒤차) | 차로 변경 | 옮길 차로 ±30m 차 | 뒤 30m 차 | 끼어들기 | 블록 중간 느린 앞차(s) | 느린 차 만남/앞지름 | 앞지른 차 |")
    print("|" + "---|" * 17)
    for policy in args.policies:
        for scenario in scenarios:
            s = around.get(f"{policy}:{scenario}")
            if not s:
                continue
            reasons = {k: v for k, v in s["reasons"].items() if k != "arrived"}
            print(f"| {policy} | {scenario} | {s['laps']} | {s['reasons'].get('arrived', 0)} | {reasons or '-'} | "
                  f"{s['stuck_roads'] or '-'} | {s['lap_s'] or 0:.0f} | {s['near_n50']:.1f} / {s['mid_n50']:.1f} | "
                  f"{s['near_leader30']:.0%} / {s['mid_leader30']:.0%} | {s['stop']:.2f} ({s['stop_follower50']:.2f}) | {s['lc']:.2f} | {s['lc_target30']:.2f} | "
                  f"{s['lc_follower30']:.2f} | {s['cut_in']:.2f} | {s['slow_mid_s']:.1f} | "
                  f"{s['snail_met']:.2f} / {s['snail_passed']:.2f} | {s['passed']:.2f} |")


if __name__ == "__main__":
    main()
