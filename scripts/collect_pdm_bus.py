"""합정역 시나리오(run_hapjeong_pdm_bus.py와 같은 도로·교통·시나리오)에서 PDM-Closed 버스의 원시 데이터를 모은다.

    python scripts/collect_pdm_bus.py                                   # baseline, seed 0, 1바퀴
    python scripts/collect_pdm_bus.py --scenario busy --seed 3 --laps 2
    python scripts/collect_pdm_bus.py --warmup 30 --max-seconds 120 --check-obs 200 --name smoke   # 짧은 확인

에피소드는 한 바퀴(R68 -> R68). 0.5초마다 한 행(스냅숏 + 그 구간의 0.1초 기록)을 쓴다. 관측·행동 라벨·보상은 쓰지 않는다:
scripts/preprocess_pdm_bus.py가 만든다. 설계: docs/superpowers/specs/2026-11-24-pdm-bus-data-collection-design.md

출력 data/pdm_bus/<name>/ (수집 중에는 .<name>.partial/에 쓰고 끝나면 옮긴다. metadata.json이 있으면 끝까지 된 것이다)
    steps.jsonl.gz  events.jsonl  ssm.xml  metadata.json  assets/
"""
import argparse
import datetime
import glob
import os
import shutil
import sys
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import traffic.hapjeong as hapjeong  # noqa: E402
from maps import load_map  # noqa: E402
from pdm_data.lap_context import build_lap_context  # noqa: E402
from pdm_data.metadata import build_metadata  # noqa: E402
from pdm_data.obs_check import OnlineObsCheck  # noqa: E402
from pdm_data.recorder import CollectionDone, Recorder, RunWriter  # noqa: E402
from pdm_data.schema import BUS, DECISION_STEPS  # noqa: E402
from pdm_data.ssm import ssm_vtype  # noqa: E402
from run_hapjeong_scenario import map_stops  # noqa: E402
from traffic.alleys import read_net  # noqa: E402
from traffic.bus_window_keeper import DEFAULT_LANE_DENSITY  # noqa: E402
from traffic.pdm_bus import BUS_PDM_VTYPE, PDMClosedBusPilot  # noqa: E402


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=("quiet", "baseline", "busy"), default="baseline")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--warmup", type=float, default=120.0)
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--max-seconds", type=float, default=1800.0, help="기록 시작 뒤 이 시간이 지나면 멈춘다(바퀴 중간이면 time_limit)")
    ap.add_argument("--lane-buses", type=float, default=30.0)
    ap.add_argument("--taxis", type=float, default=40.0)
    ap.add_argument("--route-boost", type=float, default=1.3)
    ap.add_argument("--stopped", action="append", default=[], metavar="SPEC", help="run_hapjeong_scenario.py와 같다")
    ap.add_argument("--no-inject", action="store_true")
    ap.add_argument("--no-stop-ahead", action="store_true")
    ap.add_argument("--no-ped-defiance", action="store_true")
    ap.add_argument("--no-density", action="store_true")
    ap.add_argument("--check-obs", type=int, default=0, help="처음 N행에 BusTask.observation 값을 함께 남긴다(오프라인 계산 확인용)")
    ap.add_argument("--no-record", action="store_true", help="기록·SSM 없이 돌려 버스 궤적만 낸다(동작 불변 비교용)")
    ap.add_argument("--name", default=None)
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "data", "pdm_bus"))
    ap.add_argument("--work-dir", default=None)
    a = ap.parse_args(argv)
    a.name = a.name or f"pdm_{a.scenario}_s{a.seed}"
    a.work_dir = a.work_dir or os.path.join(tempfile.gettempdir(), f"pdm_collect_{a.name}")
    return a


def _world(a):
    from traffic.bus_traffic import BusTrafficCfg
    stopped = [hapjeong.stopped_from_arg(x, a.warmup, i) for i, x in enumerate(a.stopped)]
    bt = BusTrafficCfg(lane_buses_per_h=a.lane_buses, taxis_per_h=a.taxis) if a.lane_buses > 0 else None
    return hapjeong.HapjeongTraffic(a.work_dir, scenario=a.scenario, seed=a.seed, stopped=stopped, bus_traffic=bt,
                                    lane_density=None if a.no_density else DEFAULT_LANE_DENSITY,
                                    route_flow_boost=a.route_boost)


def _copy_assets(work_dir, assets):
    os.makedirs(assets, exist_ok=True)
    for path in [os.path.join(work_dir, "network.net.xml"), *glob.glob(os.path.join(work_dir, "*.add.xml")),
                 *glob.glob(os.path.join(work_dir, "*.sumocfg"))]:
        if os.path.exists(path):
            shutil.copy2(path, assets)


def collect(a) -> dict:
    final_dir = os.path.join(a.out_dir, a.name)
    part = os.path.join(a.out_dir, f".{a.name}.partial")
    if not a.no_record:
        if os.path.exists(final_dir):
            raise SystemExit(f"이미 있다: {final_dir} (다른 --name을 쓰거나 지운 뒤 다시)")
        shutil.rmtree(part, ignore_errors=True)
        os.makedirs(part)
    vtype = BUS_PDM_VTYPE if a.no_record else ssm_vtype(BUS_PDM_VTYPE, os.path.abspath(os.path.join(part, "ssm.xml")))
    hapjeong.BUS_VTYPE = vtype                     # 월드가 버스 유형을 만들 때 읽는다
    world = _world(a)
    world.build()
    started_at = datetime.datetime.now().isoformat(timespec="seconds")
    tc = world.start(warmup=a.warmup, gui=False)
    from traffic.sumo_conn import traci
    m = load_map("hapjeong")
    net_path = os.path.join(a.work_dir, "network.net.xml")
    pilot = PDMClosedBusPilot(tc, read_net(net_path), m["routes"]["bus_loop"], map_stops(m))
    trace, state = [], {"step": 0}

    def trace_step():
        state["step"] += 1
        if state["step"] % DECISION_STEPS == 0 and BUS in tc.vehicle.getIDList():
            x, y = tc.vehicle.getPosition(BUS)
            trace.append((tc.simulation.getTime(), x, y, tc.vehicle.getSpeed(BUS)))

    rec, writer, ctx = None, None, None
    if not a.no_record:
        lane_allowed = {f"{e}_{i}": list(tc.lane.getAllowed(f"{e}_{i}"))
                        for e in pilot.lap_edges for i in range(tc.edge.getLaneNumber(e))}
        ctx = build_lap_context(net_path, pilot.lap_edges, m["bus_stops"], m["route_design"]["bus_loop"]["max_speed_mps"],
                                lane_allowed)
        _copy_assets(a.work_dir, os.path.join(part, "assets"))
        writer = RunWriter(part)
        check = OnlineObsCheck(ctx, tc, a.check_obs) if a.check_obs > 0 else None
        rec = Recorder(tc, world, pilot, ctx, writer, max_laps=a.laps, max_seconds=a.max_seconds, check=check,
                       run_id=a.name)

    def on_step(t):
        trace_step()
        if rec is not None:
            rec.on_step(t)

    end = "time_limit"
    try:
        world.run(tc.simulation.getTime() + a.max_seconds + 1.0, bus_id=BUS, bus_hook=pilot.tick, on_step=on_step,
                  stop_bypass=False, ped_defiance=not a.no_ped_defiance, stop_ahead=not a.no_stop_ahead,
                  scenario_inject=not a.no_inject, pilot=pilot, triggers="scenario")
    except CollectionDone as e:
        end = e.reason
    except (traci.exceptions.FatalTraCIError, ConnectionError):
        end = "sumo_closed"
    meta = None
    if rec is not None:
        rec.finish(end)
        writer.close()
        meta = build_metadata(a, world, pilot, ctx, rec, tc, vtype=vtype,
                              net_copy=os.path.join(part, "assets", "network.net.xml"), bus_stops=m["bus_stops"],
                              started_at=started_at)
    world.close()
    out = {"run_dir": None, "rows": rec.rows if rec else 0, "laps_completed": len(pilot.lap_times), "end": end,
           "bus_trace": trace}
    if rec is not None:
        import json
        with open(os.path.join(part, "metadata.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=1, default=list)
        os.replace(part, final_dir)
        out["run_dir"] = final_dir
    return out


def main():
    a = parse_args()
    r = collect(a)
    print(f"끝: {r['end']} | 행 {r['rows']} | 완주 바퀴 {r['laps_completed']} | {r['run_dir'] or '(기록 없음)'}")


if __name__ == "__main__":
    main()
