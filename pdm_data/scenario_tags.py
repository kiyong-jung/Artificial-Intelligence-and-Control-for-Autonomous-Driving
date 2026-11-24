"""역할(주입·시나리오·A1/A3)과 시나리오 사건(설계 7.11, 8절). 판정 규칙은 scripts/measure_s1_s6.py와 같다."""
from __future__ import annotations

import math

from traffic.metrics import pet
from traffic.sumo_conn import traci

from .lap_context import LapContext, lap_route_index
from .schema import BUS, DETECT_RADIUS_M, DT, INDUCED_BRAKE_MPS2, NEAR_M, S5_WINDOW_S, STATE_RADIUS_M

TraCIException = traci.exceptions.TraCIException
VEH_PREFIX = (("inj_s5", "S5"), ("inj_S5", "S5"), ("stopahead", "S4"), ("stopped", "S4"), ("taxi0", "S4"))
PED_PREFIX = (("inj_s6_", "S6_injected"), ("s6ped", "S6_background"))
BUS_LENGTH_M, CROSSING_WIDTH_M = 12.0, 4.0


def _event(kind, lap, t_start, t_end=None, ids=(), injected=False, detail=None):
    return {"kind": kind, "lap": lap, "t_start": t_start, "t_end": t_end, "ids": list(ids), "injected": injected,
            "detail": detail or {}}


def _jsonable(d):
    return {k: v for k, v in (d or {}).items() if isinstance(v, (str, int, float, bool, type(None)))}


class _RoleMap(dict):
    """명시 역할 + id 접두어 역할. get(id)이 없는 id에도 접두어 역할을 돌려준다."""

    def __init__(self, explicit, prefix_fn):
        super().__init__(explicit)
        self._prefix_fn = prefix_fn

    def get(self, key, default=None):
        base = self._prefix_fn(key)
        base.update(dict.get(self, key, {}))
        return base if base else default

    def __getitem__(self, key):
        got = self.get(key)
        if got is None:
            raise KeyError(key)
        return got


class RoleBook:
    def __init__(self, world, lap_fn=lambda: 0):
        self.world, self.lap_fn = world, lap_fn
        self._veh, self._ped, self._events = {}, {}, []
        self._seen = {"injector": 0, "mgr": 0, "ped": 0, "ahead": 0}
        self.offset = 0.0              # 관리자 로그 시각(runner 시계) + offset = SUMO 시각

    def _ts(self, x):
        return x + self.offset if isinstance(x, (int, float)) and not isinstance(x, bool) else x

    @staticmethod
    def _prefix(oid, table):
        for p, role in table:
            if oid.startswith(p):
                return {"role": role, "injected": p.startswith(("inj_", "stopahead"))}
        return {}

    def role_for_vehicle(self, vid):
        return self.roles()["vehicles"].get(vid, {})

    def role_for_person(self, pid):
        return self.roles()["pedestrians"].get(pid, {})

    def _new(self, name, log):
        start = self._seen[name]
        self._seen[name] = len(log)
        return log[start:]

    def update(self):
        w = self.world
        inj = getattr(w, "injector", None)
        for r in self._new("injector", getattr(inj, "log", []) if inj is not None else []):
            injected = r.get("ok") == "injected"
            if r.get("id"):
                cur = self._veh.setdefault(r["id"], {})
                cur["role"] = r["kind"]
                cur["injected"] = cur.get("injected", False) or injected
            self._events.append(_event("injection", r.get("lap"), self._ts(r.get("t")), ids=[r["id"]] if r.get("id") else [],
                                       injected=injected, detail=_jsonable(r)))
        mgr = getattr(w, "mgr", None)
        for r in self._new("mgr", getattr(mgr, "log", []) if mgr is not None else []):
            self._events.append(_event("a1", self.lap_fn(), self._ts(r.get("t_start")), self._ts(r.get("t_end")), ids=[r.get("follower")],
                                       injected=self._veh.get(r.get("follower"), {}).get("injected", False),
                                       detail=_jsonable(r)))
        ped = getattr(w, "ped", None)
        for r in self._new("ped", getattr(ped, "log", []) if ped is not None else []):
            self._ped.setdefault(r["ped"], {})["a3_decision"] = bool(r.get("cross"))
            self._events.append(_event("a3", self.lap_fn(), self._ts(r.get("t")), ids=[r["ped"]], detail=_jsonable(r)))
        ahead = getattr(w, "ahead", None)
        if ahead is not None:
            for r in self._new("ahead", getattr(ahead, "log", [])):
                ids = [b.get("vid") for b in r.get("blocks", []) if b.get("vid")]
                self._events.append(_event("s4_block", self.lap_fn(), self._ts(r.get("t0")), self._ts(r.get("t1")), ids=ids, injected=True,
                                           detail={"planned": r.get("planned"), "blocks": r.get("blocks")}))
            for b in (getattr(ahead, "plan", {}) or {}).values():
                if getattr(b, "vid", None):
                    self._veh.setdefault(b.vid, {}).update(role="S4", injected=True)

    def roles(self) -> dict:
        veh = {k: dict(v) for k, v in self._veh.items()}
        mgr = getattr(self.world, "mgr", None)
        for vid, it in (getattr(mgr, "inter", {}) or {}).items():
            veh.setdefault(vid, {})["role"] = "A1_yield" if it.yields else "A1_block"
        return {"vehicles": _RoleMap(veh, lambda k: self._prefix(k, VEH_PREFIX)),
                "pedestrians": _RoleMap({k: dict(v) for k, v in self._ped.items()}, lambda k: self._prefix(k, PED_PREFIX))}

    def new_events(self) -> list:
        out, self._events = self._events, []
        return out


class ScenarioTracker:
    """0.1 s마다 step()을 부른다. 사건은 new_events()로 꺼낸다."""

    def __init__(self, tc, world, pilot, ctx: LapContext, feeders):
        self.tc, self.world, self.pilot, self.ctx, self.feeders = tc, world, pilot, ctx, set(feeders)
        self._events = []
        self._prev_phase, self._attempt = "idle", None
        self._stop = None
        self._alley_seen, self._cut = {}, set()
        self._s6 = {}
        self._nearby, self._prev_speed, self._braking = (), {}, {}
        self._follower = None
        self._last_road = None
        self._n = 0

    def _lap(self):
        return self.pilot.lap

    def _a1_count(self):
        mgr = getattr(self.world, "mgr", None)
        return len(getattr(mgr, "log", [])) + len(getattr(mgr, "inter", {})) if mgr is not None else 0

    def set_nearby(self, snap):
        """스냅숏마다: 유발 급제동을 볼 반경 50 m 차량과 지금 후행차."""
        ex, ey = snap["ego"]["x"], snap["ego"]["y"]
        self._nearby = tuple(v["id"] for v in snap["vehicles"] if math.hypot(v["x"] - ex, v["y"] - ey) <= NEAR_M)
        f = snap["neighbors"].get("follower")
        self._follower = f["id"] if f else None

    # ---- 매 0.1 s ----
    def step(self, t, collisions=(), teleports=(), emergency=()):
        tc, v = self.tc, self.tc.vehicle
        self._n += 1
        bx, by = v.getPosition(BUS)
        road, lane, lane_pos, lane_idx = v.getRoadID(BUS), v.getLaneID(BUS), v.getLanePosition(BUS), v.getLaneIndex(BUS)
        if not road.startswith(":"):
            self._last_road = road
        try:
            pos = self.ctx.route_map.position(lap_route_index(road, self._last_road, self.ctx.lap_edges), lane, lane_pos)
        except ValueError:
            pos = None
        self._sim_events(t, bx, by, collisions, teleports, emergency)
        self._lane_window(t, lane_idx)
        self._s1(t, road, lane_pos)
        self._s5(t, road, lane_pos)
        if pos is not None:
            self._s6_step(t, pos, v.getSpeed(BUS))
        self._induced(t)

    def _near(self, oid, bx, by, radius):
        try:
            x, y = self.tc.vehicle.getPosition(oid)
        except TraCIException:
            return None
        return math.hypot(x - bx, y - by) <= radius

    def _sim_events(self, t, bx, by, collisions, teleports, emergency):
        for c in collisions:
            ids = [getattr(c, "collider", None), getattr(c, "victim", None)]
            self._events.append(_event("collision", self._lap(), t, t, ids=ids,
                                       detail={"type": getattr(c, "type", None), "ego": BUS in ids,
                                               "lane": getattr(c, "lane", None), "pos": getattr(c, "pos", None)}))
        for vid in teleports:
            if vid == BUS or self._near(vid, bx, by, STATE_RADIUS_M):
                self._events.append(_event("teleport", self._lap(), t, t, ids=[vid], detail={"ego": vid == BUS}))
        for vid in emergency:
            if vid == BUS or self._near(vid, bx, by, DETECT_RADIUS_M):
                self._events.append(_event("emergency_stop", self._lap(), t, t, ids=[vid], detail={"ego": vid == BUS}))

    def _lane_window(self, t, lane_idx):
        p = self.pilot
        if p.phase == "signal" and self._prev_phase != "signal" and self._attempt is None:
            plan = p.plan
            self._attempt = {"t": t, "lap": self._lap(), "plan_kind": plan.kind if plan else None,
                             "why": p.why, "edge": plan.edge if plan else None, "target": p.target,
                             "a1_before": self._a1_count(), "missed_before": len(p.missed),
                             "forced_before": len(p.forced), "t_done": None}
        a = self._attempt
        if a and a["t_done"] is None and a["target"] is not None and lane_idx == a["target"]:
            a["t_done"] = t
        if a and self._prev_phase != "idle" and p.phase == "idle":
            self._events.append(_event("lane_window", a["lap"], a["t"], t, detail={
                "plan_kind": a["plan_kind"], "why": a["why"], "edge": a["edge"], "target": a["target"],
                "success": a["t_done"] is not None, "t_done": a["t_done"],
                "a1": self._a1_count() > a["a1_before"], "missed": len(p.missed) > a["missed_before"],
                "forced_stop": len(p.forced) > a["forced_before"]}))
            self._attempt = None
        self._prev_phase = p.phase

    def _s1(self, t, road, lane_pos):
        ss = self.tc.vehicle.getStopState(BUS)
        at_stop = bool(ss & 1 and ss & 16)
        if at_stop and self._stop is None:
            fo = self.tc.vehicle.getFollower(BUS, 80.0)
            self._stop = {"t": t, "edge": road, "lane_position": lane_pos, "follower": fo[0] if fo and fo[0] else None}
        elif not at_stop and self._stop is not None:
            fo = self.tc.vehicle.getFollower(BUS, 80.0)
            end = fo[0] if fo and fo[0] else None
            s = self._stop
            self._events.append(_event("s1_stop", self._lap(), s["t"], t, ids=[s["follower"]] if s["follower"] else [],
                                       detail={"edge": s["edge"], "lane_position": s["lane_position"],
                                               "follower_start": s["follower"], "follower_end": end,
                                               "follower_overtook": s["follower"] is not None and end != s["follower"]}))
            self._stop = None

    def _s5(self, t, road, lane_pos):
        if not self.feeders or self._n % 5:
            return
        v = self.tc.vehicle
        for vid in v.getIDList():
            if vid == BUS:
                continue
            try:
                r = v.getRoadID(vid)
                if r in self.feeders:
                    self._alley_seen[vid] = (t, r)
                elif vid in self._alley_seen and vid not in self._cut and t - self._alley_seen[vid][0] <= S5_WINDOW_S \
                        and r == road and 0 < v.getLanePosition(vid) - lane_pos <= NEAR_M:
                    self._cut.add(vid)
                    self._events.append(_event("s5_cutin", self._lap(), self._alley_seen[vid][0], t, ids=[vid],
                                               injected=vid.lower().startswith("inj_s5"),
                                               detail={"alley": self._alley_seen[vid][1],
                                                       "gap": v.getLanePosition(vid) - lane_pos,
                                                       "bus_speed": v.getSpeed(BUS)}))
            except TraCIException:
                continue

    def _s6_step(self, t, pos, speed):
        for at, cid in self.ctx.route_map.crossings:
            st = self._s6.get(cid)
            zone = -NEAR_M <= pos - at <= BUS_LENGTH_M + CROSSING_WIDTH_M
            if st is None and zone and pos <= at:
                st = self._s6[cid] = {"t": t, "at": at, "bus_in": None, "bus_out": None, "peds": {}, "min_speed": speed}
            if st is None:
                continue
            st["min_speed"] = min(st["min_speed"], speed)
            if st["bus_in"] is None and pos >= at:
                st["bus_in"] = t
            for pid in self.tc.person.getIDList():
                try:
                    on = self.tc.person.getRoadID(pid) == cid
                except TraCIException:
                    continue
                iv = st["peds"].get(pid)
                if on and iv is None:
                    st["peds"][pid] = [t, None]
                elif not on and iv is not None and iv[1] is None:
                    iv[1] = t
            if pos - BUS_LENGTH_M > at + CROSSING_WIDTH_M or not (-NEAR_M - 1.0 <= pos - at <= 100.0):
                st["bus_out"] = t
                self._close_s6(cid, t)

    def _close_s6(self, cid, t):
        st = self._s6.pop(cid)
        if not st["peds"]:
            return
        bus_iv = (st["bus_in"], st["bus_out"]) if st["bus_in"] is not None else None
        pets = {pid: pet((iv[0], iv[1] if iv[1] is not None else t), bus_iv) for pid, iv in st["peds"].items()}
        vals = [p for p in pets.values() if p is not None]
        self._events.append(_event("s6_pass", self._lap(), st["t"], t, ids=list(st["peds"]),
                                   injected=any(p.startswith("inj_s6_") for p in st["peds"]),
                                   detail={"crossing": cid, "bus_interval": bus_iv, "ped_intervals": st["peds"],
                                           "pet": pets, "min_pet": min(vals) if vals else None,
                                           "bus_min_speed": st["min_speed"]}))

    def _induced(self, t):
        v = self.tc.vehicle
        for vid in self._nearby:
            try:
                sp = v.getSpeed(vid)
            except TraCIException:
                self._prev_speed.pop(vid, None)
                continue
            prev = self._prev_speed.get(vid)
            self._prev_speed[vid] = sp
            if prev is None:
                continue
            dec = (prev - sp) / DT
            cur = self._braking.get(vid)
            if dec >= INDUCED_BRAKE_MPS2:
                if cur is None:
                    self._braking[vid] = {"t": t, "max": dec, "follower": vid == self._follower}
                else:
                    cur["max"] = max(cur["max"], dec)
            elif cur is not None:
                self._events.append(_event("induced_brake", self._lap(), cur["t"], t, ids=[vid],
                                           detail={"max_decel": cur["max"], "bus_follower": cur["follower"]}))
                del self._braking[vid]

    # ---- 스냅숏 표시 ----
    def tags(self, snap) -> dict:
        p, ego, ctx = self.pilot, snap["ego"], self.ctx
        pos = ego["lap_position"]
        ss = ego["stop_state"]
        out = {"s1": None, "s2": None, "s3": None, "s4": [], "s5": [], "s6": None, "a1": [], "injector_plan": None}
        if ss & 1 and ss & 16:
            f = snap["neighbors"].get("follower")
            out["s1"] = {"follower": f["id"] if f else None}
        plan = p.plan
        if plan is not None and p.phase in ("signal", "request") and plan.kind in ("S2", "S3"):
            dist = None
            if pos is not None and plan.edge in ctx.lap_edges:
                dist = ctx.route_map.start[ctx.lap_edges.index(plan.edge)] + plan.end_pos - pos
            out[plan.kind.lower()] = {"edge": plan.edge, "target": p.target, "dist_to_end": dist,
                                      "a1": list(getattr(getattr(self.world, "mgr", None), "inter", {}) or {})}
        ex, ey = ego["x"], ego["y"]
        for veh in snap["vehicles"]:
            rd = veh.get("route_distance")
            if veh.get("role") == "S4" and rd is not None and 0 < rd <= NEAR_M:
                out["s4"].append(veh["id"])
            if veh.get("role") == "S5" and math.hypot(veh["x"] - ex, veh["y"] - ey) <= NEAR_M:
                out["s5"].append(veh["id"])
        if pos is not None:
            ahead = next(((at, cid) for at, cid in ctx.route_map.crossings if 0 <= at - pos <= NEAR_M), None)
            if ahead is not None:
                peds = [q["id"] for q in snap["pedestrians"] if q["road_id"] == ahead[1] or q["next_edge"] == ahead[1]]
                out["s6"] = {"crossing": ahead[1], "distance": ahead[0] - pos, "peds": peds}
        mgr = getattr(self.world, "mgr", None)
        out["a1"] = [{"id": vid, "action": it.action} for vid, it in (getattr(mgr, "inter", {}) or {}).items()]
        inj = getattr(self.world, "injector", None)
        out["injector_plan"] = _jsonable(getattr(inj, "_lap", None)) if inj is not None else None
        return out

    def new_events(self) -> list:
        out, self._events = self._events, []
        return out
