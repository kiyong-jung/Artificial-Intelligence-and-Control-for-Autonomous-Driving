"""판단 구간 기록기(설계 7절). runner의 on_step(t)로 0.1 s마다 불린다. TraCI는 조회만 하고, set 호출은 CallTap이 그대로 넘긴다.

행 t = 구간 [t, t+0.5)의 시작에서 찍은 스냅숏 + 구간이 끝날 때 채운 window. 바퀴가 바뀌면 새 스냅숏을 이전 에피소드의
final 행(window 없음)으로 한 번, 새 에피소드의 t=0 행으로 한 번 쓴다.
"""
from __future__ import annotations

import gzip
import json
import os

from .lap_context import LapContext, lap_route_index
from .scenario_tags import RoleBook, ScenarioTracker
from .schema import BUS, DECISION_STEPS, DETECT_RADIUS_M, DT
from .snapshot import take_snapshot


class CollectionDone(Exception):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class CallTap:
    """tc.vehicle의 changeLane/setMaxSpeed/setSpeed를 감싸 버스 호출만 기록하고 원래 함수를 그대로 부른다."""
    NAMES = ("changeLane", "setMaxSpeed", "setSpeed")

    def __init__(self, tc, bus_id):
        self.tc, self.bus = tc, bus_id
        self._buf, self._installed = [], False

    def install(self):
        dom = self.tc.vehicle
        for name in self.NAMES:
            orig = getattr(dom, name)

            def wrap(*args, _name=name, _orig=orig, **kw):
                if args and args[0] == self.bus:
                    self._buf.append({"t": self.tc.simulation.getTime(), "call": _name, "args": tuple(args)})
                return _orig(*args, **kw)
            setattr(dom, name, wrap)
        self._installed = True

    def uninstall(self):
        if self._installed:
            for name in self.NAMES:
                try:
                    delattr(self.tc.vehicle, name)
                except AttributeError:
                    pass
        self._installed = False
        self._buf = []

    def drain(self) -> list:
        out, self._buf = self._buf, []
        return out


def _round(o, nd=6):
    if isinstance(o, float):
        return round(o, nd)
    if isinstance(o, dict):
        return {k: _round(v, nd) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_round(v, nd) for v in o]
    return o


class RunWriter:
    def __init__(self, run_dir: str):
        os.makedirs(run_dir, exist_ok=True)
        self._steps = gzip.open(os.path.join(run_dir, "steps.jsonl.gz"), "wt", encoding="utf-8", compresslevel=6)
        self._events = open(os.path.join(run_dir, "events.jsonl"), "w", encoding="utf-8")

    def write_step(self, row: dict):
        self._steps.write(json.dumps(_round(row), ensure_ascii=False, separators=(",", ":")) + "\n")

    def write_event(self, ev: dict):
        self._events.write(json.dumps(_round(ev), ensure_ascii=False, separators=(",", ":")) + "\n")

    def close(self):
        self._steps.close()
        self._events.close()


def read_steps(run_dir: str):
    with gzip.open(os.path.join(run_dir, "steps.jsonl.gz"), "rt", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def read_events(run_dir: str) -> list:
    with open(os.path.join(run_dir, "events.jsonl"), encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


class Recorder:
    def __init__(self, tc, world, pilot, ctx: LapContext, writer, max_laps: int, max_seconds: float, check=None,
                 run_id: str = "run"):
        self.tc, self.world, self.pilot, self.ctx, self.writer = tc, world, pilot, ctx, writer
        self.max_laps, self.max_seconds, self.check, self.run_id = max_laps, max_seconds, check, run_id
        self.tap = CallTap(tc, BUS)
        self.tap.install()
        self.roles = RoleBook(world, lap_fn=lambda: pilot.lap)
        self.scen = ScenarioTracker(tc, world, pilot, ctx, {a.feeder for a in getattr(world, "alleys", [])})
        self.sim_step, self.started, self.pending, self._win = 0, False, None, None
        self.episode, self.t_ep, self.t_start = None, 0, None
        self.last_road, self.stopped_since = None, None
        self.tls_seen = set()
        self.rows, self.laps_completed = 0, 0
        self.collisions_total, self.ego_collisions, self.ego_teleports = 0, 0, 0
        self.end, self._finished = None, False
        self.event_counts = {}
        self.time_offset = None

    # ---- 매 0.1 s ----
    def on_step(self, runner_t):
        """runner_t는 runner의 시계(실행 시작 0). 기록은 모두 SUMO 시각(getTime)으로 한다. 관리자 로그 시각은 offset만큼 옮긴다."""
        tc = self.tc
        self.sim_step += 1
        sim = tc.simulation
        t = sim.getTime()
        if self.time_offset is None:
            self.time_offset = t - runner_t
            self.roles.offset = self.time_offset
        coll = list(sim.getCollisions())
        tele = list(sim.getStartingTeleportIDList())
        emerg = list(sim.getEmergencyStoppingVehiclesIDList())
        self.collisions_total += len(coll)
        if BUS not in tc.vehicle.getIDList():
            if self.started:
                raise CollectionDone("bus_lost")
            return
        road = tc.vehicle.getRoadID(BUS)
        if not road.startswith(":"):
            self.last_road = road
        ss = tc.vehicle.getStopState(BUS)
        self.stopped_since = (self.stopped_since if self.stopped_since is not None else t) if ss & 1 else None
        self.roles.update()
        self.scen.step(t, coll, tele, emerg)
        self._flush_events()
        if self.check is not None and self.started:
            self.check.tick()
        if self.pending is not None:
            self._accumulate(t, coll, tele, emerg)
        if self.sim_step % DECISION_STEPS:
            return
        if not self.started:
            self.started, self.t_start, self.episode, self.t_ep = True, t, self.pilot.lap, 0
            self.tap.drain()
            self.pending = self._snapshot(t, self.episode, 0)
            return
        lap_now = self.pilot.lap
        self.pending["window"] = self._close_window(lap_now != self.episode)
        self._write(self.pending)
        if lap_now != self.episode:
            self.laps_completed += 1
            self._write(self._snapshot(t, self.episode, self.t_ep + 1, final=True, end="lap"))
            if self.laps_completed >= self.max_laps:
                self.pending, self.end = None, "laps"
                raise CollectionDone("laps")
            self.episode, self.t_ep = lap_now, 0
            if self.check is not None:
                self.check.reset()
        else:
            self.t_ep += 1
        self.pending = self._snapshot(t, self.episode, self.t_ep)
        if t - self.t_start >= self.max_seconds - 1e-9:
            raise CollectionDone("time_limit")

    def finish(self, reason: str):
        """수집을 끝낸다. 남은 스냅숏을 final 행(window 없음)으로 쓴다. 여러 번 불러도 한 번만 쓴다."""
        if self._finished:
            return
        self._finished = True
        self.end = self.end or reason
        if self.pending is not None:
            row = self.pending
            row["meta"].update(final=True, end=reason)
            row["window"] = None
            self._write(row)
            self.pending = None
        self._flush_events()
        self.tap.uninstall()

    def summary(self) -> dict:
        return {"rows": self.rows, "laps_completed": self.laps_completed, "end": self.end,
                "collisions_total": self.collisions_total, "ego_collisions": self.ego_collisions,
                "ego_teleports": self.ego_teleports, "events": dict(self.event_counts)}

    # ---- 내부 ----
    def _flush_events(self):
        for ev in self.roles.new_events() + self.scen.new_events():
            self.event_counts[ev["kind"]] = self.event_counts.get(ev["kind"], 0) + 1
            self.writer.write_event(ev)

    def _write(self, row):
        self.writer.write_step(row)
        self.rows += 1

    def _direction(self, lane_idx):
        p = self.pilot
        if p.phase in ("signal", "request") and p.target is not None:
            return 1 if p.target > lane_idx else -1 if p.target < lane_idx else 0
        return 0

    def _expert(self, lane_idx):
        p = self.pilot
        ba = p.blocker_active
        return {"phase": p.phase, "why": p.why, "plan_kind": p.plan.kind if p.plan is not None else None,
                "target_lane": p.target if p.phase in ("signal", "request") else None,
                "request_direction": self._direction(lane_idx), "forced_stop": p._forced_p is not None,
                "speed_limited": bool(p._limited), "blocker_active": bool(ba()) if callable(ba) else False,
                "pdm": getattr(p, "last_plan", None)}

    def _snapshot(self, t, episode, t_ep, final=False, end=None):
        snap = take_snapshot(self.tc, self.ctx, self.last_road, self.roles.roles(), t)
        self.tls_seen.update(x["id"] for x in snap["tls"])
        self.scen.set_nearby(snap)
        row = {"meta": {"run_id": self.run_id, "episode": episode, "t": t_ep, "sim_time": t, "sim_step": self.sim_step,
                        "final": final, "end": end},
               "expert": self._expert(snap["ego"]["lane_index"]), "scenario": self.scen.tags(snap),
               "stopped_since": self.stopped_since, "check": {}, **snap, "window": None}
        if self.check is not None and not final:
            obs = self.check.observe()
            if obs is not None:
                row["check"]["obs93_online"] = obs
        if not final:
            ego = snap["ego"]
            self._win = {"substeps": [], "peak_acceleration": 0.0, "peak_jerk": 0.0, "collisions": [],
                         "teleports": [], "emergency_stops": [], "pdm_plans": [], "lane_before": ego["lane_index"],
                         "edge_before": ego["road_id"], "last_acc": ego["acceleration"]}
        return row

    def _accumulate(self, t, coll, tele, emerg):
        v, w, p = self.tc.vehicle, self._win, self.pilot
        road, lane, lane_pos = v.getRoadID(BUS), v.getLaneID(BUS), v.getLanePosition(BUS)
        lane_idx, acc = v.getLaneIndex(BUS), v.getAcceleration(BUS)
        try:
            idx = lap_route_index(road, self.last_road, self.ctx.lap_edges)
            lap_pos = self.ctx.route_map.position(idx, lane, lane_pos)
        except ValueError:
            idx, lap_pos = None, None
        tls = v.getNextTLS(BUS)
        w["substeps"].append({"t": t, "edge": road, "lane_id": lane, "lane_index": lane_idx, "lane_position": lane_pos,
                              "lap_index": idx, "lap_position": lap_pos, "distance": v.getDistance(BUS),
                              "speed": v.getSpeed(BUS), "acceleration": acc, "tls0": list(tls[0]) if tls else None,
                              "phase": p.phase, "request_direction": self._direction(lane_idx)})
        w["peak_acceleration"] = max(w["peak_acceleration"], abs(acc))
        w["peak_jerk"] = max(w["peak_jerk"], abs(acc - w["last_acc"]) / DT)
        w["last_acc"] = acc
        for c in coll:
            ids = (getattr(c, "collider", None), getattr(c, "victim", None))
            ego = BUS in ids
            self.ego_collisions += int(ego)
            w["collisions"].append({"collider": ids[0], "victim": ids[1], "type": getattr(c, "type", None), "ego": ego})
        for vid in tele:
            self.ego_teleports += int(vid == BUS)
            if vid == BUS:
                w["teleports"].append({"id": vid, "ego": True})
        bx, by = v.getPosition(BUS)
        for vid in emerg:
            near = vid == BUS
            if not near:
                try:
                    x, y = v.getPosition(vid)
                    near = ((x - bx) ** 2 + (y - by) ** 2) ** 0.5 <= DETECT_RADIUS_M
                except Exception:
                    near = False
            if near:
                w["emergency_stops"].append({"id": vid, "ego": vid == BUS})
        lp = getattr(p, "last_plan", None)
        if lp is not None and (not w["pdm_plans"] or w["pdm_plans"][-1].get("sim_time") != lp.get("sim_time")):
            w["pdm_plans"].append(lp)

    def _close_window(self, lap_done):
        v, w = self.tc.vehicle, self._win
        return {"substeps": w["substeps"], "peak_acceleration": w["peak_acceleration"], "peak_jerk": w["peak_jerk"],
                "calls": self.tap.drain(), "lane_before": w["lane_before"], "lane_after": v.getLaneIndex(BUS),
                "edge_before": w["edge_before"], "edge_after": v.getRoadID(BUS), "collisions": w["collisions"],
                "teleports": w["teleports"], "emergency_stops": w["emergency_stops"], "lap_done": bool(lap_done),
                "pdm_plans": w["pdm_plans"]}
