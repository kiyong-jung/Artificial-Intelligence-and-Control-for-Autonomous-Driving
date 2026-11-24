"""실행 메타데이터(설계 10절). 수집이 끝난 뒤, world.close() 전에 만든다(신호 프로그램을 TraCI로 읽는다)."""
from __future__ import annotations

import dataclasses
import hashlib
import os
import platform
import subprocess
import sys

from .schema import (COLUMN_DOC, DECISION_STEPS, DETECT_RADIUS_M, DT, FOES_LOOKAHEAD_M, FOLLOWER_LOOKBACK_M,
                     LEADER_LOOKAHEAD_M, RAW_SCHEMA_VERSION, SERVICE_SECONDS, STATE_RADIUS_M, STOP_IDS, STOPS_AHEAD,
                     TLS_LOOKAHEAD_M)

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
EGO_DIMS = {"length": 12.0, "width": 2.5, "min_gap": 1.0}     # BUS_PDM_VTYPE


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _git(*args):
    try:
        return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return None


def tls_programs(tc, ids) -> dict:
    out = {}
    for tid in sorted(ids):
        try:
            logics = tc.trafficlight.getAllProgramLogics(tid)
        except Exception:
            continue
        out[tid] = [{"program": lg.programID, "type": lg.type, "current_phase": lg.currentPhaseIndex,
                     "phases": [{"duration": ph.duration, "state": ph.state, "min_dur": ph.minDur, "max_dur": ph.maxDur}
                                for ph in lg.phases]} for lg in logics]
    return out


def build_metadata(args, world, pilot, ctx, recorder, tc, *, vtype: str, net_copy: str, bus_stops: list,
                   started_at: str) -> dict:
    status = _git("status", "--porcelain")
    plans = [{**{k: (sorted(v) if isinstance(v, frozenset) else v) for k, v in dataclasses.asdict(p).items()}}
             for p in pilot.plans]
    return {
        "schema_version": RAW_SCHEMA_VERSION,
        "reproduce": {"script": "scripts/collect_pdm_bus.py", "argv": sys.argv, "args": vars(args),
                      "git_commit": _git("rev-parse", "HEAD"), "git_dirty": bool(status), "sumo_version": tc.getVersion(),
                      "python": sys.version.split()[0], "platform": platform.platform(), "started_at": started_at},
        "world": {"scenario": args.scenario, "seed": args.seed, "warmup_s": args.warmup, "lane_buses_per_h": args.lane_buses,
                  "taxis_per_h": args.taxis, "route_boost": args.route_boost, "density": not args.no_density,
                  "stopped": list(args.stopped), "triggers": "scenario", "inject": not args.no_inject,
                  "ped_defiance": not args.no_ped_defiance, "stop_ahead": not args.no_stop_ahead, "stop_bypass": False,
                  "summary": getattr(world, "summary", None)},
        "expert": {"pdm_cfg": dataclasses.asdict(pilot.cfg), "vtype": vtype, "ego_dims": EGO_DIMS,
                   "service_seconds": SERVICE_SECONDS,
                   "pilot_constants": {k: getattr(pilot, k) for k in ("SIGNAL_S", "REQUEST_DURATION", "TIMEOUT_S",
                                                                       "COOL_S", "NEXT_SIGNAL_S")}},
        "time": {"step_length": DT, "decision_interval": DT * DECISION_STEPS, "time_base": "SUMO simulation time [s]",
                 "runner_t_offset": recorder.time_offset},
        "ranges": {"state_radius_m": STATE_RADIUS_M, "detect_radius_m": DETECT_RADIUS_M,
                   "leader_lookahead_m": LEADER_LOOKAHEAD_M, "follower_lookback_m": FOLLOWER_LOOKBACK_M,
                   "tls_lookahead_m": TLS_LOOKAHEAD_M, "foes_lookahead_m": FOES_LOOKAHEAD_M, "stops_ahead": STOPS_AHEAD},
        "route": {"lap_edges": ctx.lap_edges, "lap_length": ctx.route_map.length, "vmax": ctx.vmax,
                  "bus_stops": bus_stops, "service_stop_ids": list(STOP_IDS), "service_stops": ctx.service_stops,
                  "lane_plans": plans, "forbidden_intervals": {k: list(v) for k, v in pilot.forbidden.items()},
                  "lane_allowed": ctx.lane_allowed, "crossings": ctx.route_map.crossings,
                  "s5_feeders": list(recorder.feeders)},
        "tls_programs": tls_programs(tc, recorder.tls_seen),
        "net_sha256": sha256(net_copy),
        "result": {"recorder": recorder.summary(), "pdm": pilot.summary(), "lap_times_s": list(pilot.lap_times),
                   "lane_change_requests": dict(pilot.changes), "missed_windows": [list(x) for x in pilot.missed],
                   "forced_stops": [list(x) for x in pilot.forced]},
        "conventions": {"gap_raw": "앞차와는 ego minGap을, 뒤차와는 뒤차 minGap을 뺀 간격. 현재 차로는 TraCI getLeader/getFollower. "
                                   "옆 차로는 getNeighbors가 SUMO 1.27.1을 죽여서 같은 도로의 차량 목록으로 계산한다(다른 도로의 옆 차로 차는 빠진다)",
                        "signals": "1 오른쪽, 2 왼쪽, 8 제동", "lane_index": "오른쪽 0, 합정역 0번은 인도",
                        "coordinate": "SUMO 도로망 좌표 m, heading 북쪽 0도 시계 방향", "missing": "null"},
        "column_doc": COLUMN_DOC,
    }
