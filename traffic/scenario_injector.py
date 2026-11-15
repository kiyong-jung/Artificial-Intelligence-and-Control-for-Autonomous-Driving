"""방법 2: 시나리오 주입기. 시험용 버스가 지나는 순간에 상호작용 상대가 없으면 상대를 미리 투입한다.

밀도 분산(방법 1)만으로는 시험용 버스 곁에 차가 있다는 보장이 없어(S1 뒤차 0인 바퀴, S2 후행차 상호작용 0~3건, S5 0~6건),
버스가 시나리오 지점에 가까워질 때 **자연 발생이 없을 때만** 상대를 넣는다. 눈앞에 갑자기 생기지 않도록 투입 위치는 버스에서 멀다.

| 시나리오 | 언제 | 무엇을 | 어디에 |
|---|---|---|---|
| S1 뒤차 | 한 바퀴에 한 번, 무작위 정류장에서 버스가 서고 뒤 200 m 안에 차가 없을 때 | 승용차 한 대 | 버스 뒤 `back_s1`(220 m), 버스와 같은 차로 |
| S2·S3 후행차 | 버스가 방향지시등을 켜고 옮겨 갈 차로 뒤 80 m 안에 차가 없을 때(바퀴당 최대 `max_lane`번) | 승용차 한 대 | 버스 뒤 `back_lane`(110 m), 옮겨 갈 차로 |
| S5 끼어들기 | 한 바퀴에 한 번, 앞에 있는 합류 골목 하나를 무작위로 골라 버스가 곧 합류점에 닿을 때 | 끼어들기(무시 확률 1) 차 한 대 | 그 골목의 시작점에서 출발 |
| S6 보행자 | 한 바퀴에 한 번, 버스가 신호 없는 횡단보도 100 m 앞에 왔고 횡단보도에 사람이 없을 때 | 보행자 한 명 | 횡단보도 바로 옆 인도에서 출발(걸어서 건넌다) |

투입한 것은 `log`에 따로 기록해 자연 발생과 구분한다. 투입된 차는 `carDist`라 주행 모델 4종과 양보 성격이 그대로 붙는다.
"""
import random

BLINKER_RIGHT, BLINKER_LEFT = 1, 2


class ScenarioInjector:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(runner의 extras)."""

    def __init__(self, traci, net, bus_id, stops, alleys, cutin_type, rng=None, back_s1=220.0, back_lane=110.0,
                 max_lane=2, s6_edges=("-R112", "R112.112"), s6_edge_before="R112", trigger_s6=100.0,
                 follower_check=80.0, exits=(), ready_gap=40.0, s5_lead=60.0):
        self.s5_lead = s5_lead                           # S5 끼어들기 차를 합류점 이만큼 앞에서 출발시킨다
        self._escort = {}                                # 주입한 후행차 -> 시작 시각. 버스를 앞지르지 않게 속도를 묶는다
        self.ready_gap = ready_gap                       # 목표 차로 후행차가 이만큼 가까이 오면 버스가 차선변경을 요청한다
        self._pending = None
        self.why_fn = None                               # 시험용 버스의 차선변경 이유를 돌려주는 함수(경로·정류장만 주입 대상)
        self.tc, self.net, self.bus = traci, net, bus_id
        self.stops = {s["id"]: s for s in stops}
        self.alleys = list(alleys)
        self.cutin_type = cutin_type
        self.rng = rng or random.Random(0)
        self.back_s1, self.back_lane, self.max_lane = back_s1, back_lane, max_lane
        self.s6_walk, self.s6_before, self.trigger_s6 = list(s6_edges), s6_edge_before, trigger_s6
        self.follower_check = follower_check
        self.exits = [e for e in exits if net.hasEdge(e)]
        self.log = []
        self._lens = {}
        self._lap = None
        self._seq = 0
        self._prev_sig = 0
        self._was_stopped = False

    # ---- 바퀴 ---------------------------------------------------------------------------------
    def _new_lap(self, t):
        stop_ids = sorted(self.stops)
        self._lap = {"t0": t, "s1_stop": self.rng.choice(stop_ids) if stop_ids else None, "s1": False, "lane": 0,
                     "s5": False, "s6": False,
                     "s5_alley": self.rng.choice(self.alleys) if self.alleys else None}

    # ---- 경로 위 거리(모두 시험용 버스의 경로 기준) --------------------------------------------
    def _len(self, edge):
        n = self._lens.get(edge)
        if n is None:                                  # 도로 길이는 변하지 않으니 한 번만 묻는다(속도)
            n = self._lens[edge] = self.tc.lane.getLength(f"{edge}_0")
        return n

    def _state(self):
        tc = self.tc
        road = tc.vehicle.getRoadID(self.bus)
        i = tc.vehicle.getRouteIndex(self.bus)
        if road.startswith(":") or i < 0:
            return None
        return list(tc.vehicle.getRoute(self.bus)), i, tc.vehicle.getLanePosition(self.bus), tc.vehicle.getLaneIndex(self.bus)

    def _back_point(self, route, i, pos, back):
        """경로를 거슬러 back m 뒤 지점 (도로 index, 위치). 경로 시작보다 더 뒤면 None."""
        if pos >= back:
            return i, pos - back
        left, k = back - pos, i - 1
        while k >= 0:
            if self.net.getEdge(route[k]).getToNode().getType() == "traffic_light":
                # 신호 교차로 뒤에서 출발시키면 후행차가 적색에 걸려 버스에 닿지 못한다(S2 후행차가 목표 차로에 없던 원인).
                # 신호 교차로를 이미 지난 도로(k+1)의 시작에서 출발시킨다.
                return k + 1, 1.0
            L = self._len(route[k])
            if left <= L:
                return k, L - left
            left -= L
            k -= 1
        return None

    def _gap(self, route, i, pos, j, pos2):
        if j == i:
            return pos2 - pos
        d = self._len(route[i]) - pos
        for e in route[i + 1:j]:
            d += self._len(e)
        return d + pos2

    def _add_car(self, kind, route_edges, lane, pos, vtype="carDist", speed="max"):
        self._seq += 1
        vid, rid = f"inj_{kind}{self._seq}", f"r_inj_{kind}{self._seq}"
        try:
            self.tc.route.add(rid, route_edges)
            self.tc.vehicle.add(vid, rid, typeID=vtype, departLane=lane, departPos=f"{max(1.0, pos):.1f}",
                                departSpeed=speed)
        except Exception:
            return None
        return vid

    def _record(self, kind, t, ok, vid=None):
        self.log.append({"t": round(t, 1), "lap": len(self._laps_done) + 1, "kind": kind, "ok": ok, "id": vid})

    _laps_done = ()

    # ---- 매 스텝 ------------------------------------------------------------------------------
    def update(self, t):
        tc = self.tc
        if self.bus not in tc.vehicle.getIDList():
            if self._lap is not None:
                self._laps_done = self._laps_done + (self._lap,)
                self._lap = None
            self._prev_sig, self._was_stopped, self._pending = 0, False, None
            return
        if self._lap is None:
            self._new_lap(t)
        self._escort_step(t)
        st = self._state()
        if st is None:
            return
        self._s1(t, st)
        self._lane(t, st)
        self._s5(t, st)
        self._s6(t, st)

    def _s1(self, t, st):
        tc, lap = self.tc, self._lap
        state = tc.vehicle.getStopState(self.bus)
        stopped = bool(state & 1 and state & 16)
        if stopped and not self._was_stopped and not lap["s1"]:
            nxt = tc.vehicle.getStops(self.bus, 1)
            here = tc.vehicle.getRoadID(self.bus)
            at = next((sid for sid, s in self.stops.items() if s["edge"] == here), None)
            if at == lap["s1_stop"]:
                lap["s1"] = True
                f = tc.vehicle.getFollower(self.bus, 200.0)
                if f and f[0] and f[1] <= 200.0:
                    self._record("S1", t, "natural")
                else:
                    route, i, pos, idx = st
                    bp = self._back_point(route, i, pos, self.back_s1)
                    if bp is None:
                        self._record("S1", t, False)
                    else:
                        k, p = bp
                        vid = self._add_car("s1", route[k:i + 2], str(idx), p)
                        self._record("S1", t, "injected" if vid else False, vid)
        self._was_stopped = stopped

    def _lane(self, t, st):
        tc, lap = self.tc, self._lap
        sig = tc.vehicle.getSignals(self.bus) & (BLINKER_LEFT | BLINKER_RIGHT)
        rising = sig and not (self._prev_sig & sig)
        self._prev_sig = sig
        if not rising:
            return
        if self.why_fn is not None and self.why_fn() not in ("경로", "정류장"):
            return                                       # 순항·추월 같은 선택적 차선변경에는 넣지 않는다(S2·S3는 필수 변경이다)
        left = bool(sig & BLINKER_LEFT)
        key = "lane_l" if left else "lane_r"
        if lap.get(key, 0) >= self.max_lane:
            return
        f = tc.vehicle.getLeftFollowers(self.bus) if left else tc.vehicle.getRightFollowers(self.bus)
        near = [(d, v) for v, d in f if v and 0 <= d <= self.follower_check]
        if near:
            self._record("S2" if left else "S3", t, "natural")
            lap[key] = lap.get(key, 0) + 1
            self._pending = {"vid": min(near)[1], "left": left, "t": t}        # 자연 후행차도 가까이 올 때까지 기다린다
            return
        route, i, pos, idx = st
        bp = self._back_point(route, i, pos, self.back_lane)
        if bp is None:
            return
        k, p = bp
        target = idx + (1 if left else -1)
        lane = str(target) if 0 <= target < tc.edge.getLaneNumber(route[k]) and target >= 1 else "best"
        vid = self._add_car("lane", route[k:i + 8], lane, p)       # 경로를 넉넉히 줘서 중간에 사라지지 않게 한다
        self._record("S2" if left else "S3", t, "injected" if vid else False, vid)
        if vid:
            self._pending = {"vid": vid, "left": left, "t": t}
            self._escort[vid] = t
        lap[key] = lap.get(key, 0) + 1

    def _escort_step(self, t):
        """주입한 후행차가 시험용 버스를 앞지르지 않게 최고 속도를 버스 속도 +2 m/s로 묶는다(최대 30초).
        그래야 버스가 방향지시등을 켜고 차선변경을 요청할 때 후행차가 가까운 뒤에 있어 A1이 켜진다(앞지르면 상호작용이 사라진다)."""
        if not self._escort:
            return
        tc = self.tc
        ids = set(tc.vehicle.getIDList())
        v_bus = tc.vehicle.getSpeed(self.bus) if self.bus in ids else 0.0
        for vid, t0 in list(self._escort.items()):
            if vid not in ids or t - t0 > 30.0 or self.bus not in ids:
                if vid in ids:
                    tc.vehicle.setMaxSpeed(vid, 13.89)      # 묶음을 푼다
                del self._escort[vid]
                continue
            tc.vehicle.setMaxSpeed(vid, max(4.0, v_bus + 2.0))

    def gate(self, t):
        """시험용 버스가 차선변경을 요청해도 되는가. 목표 차로 후행차가 `ready_gap` 안으로 오기 전에는 False다.
        그래서 A1이 켜지는 때(방향지시등 + 후행차 접근)가 두 차가 가까워 버스가 바로 옮길 수 없는 때가 되고, 버스는 후행차의
        행동(양보하면 옮기고, 아니면 기다리거나 다시 시도)에 따라 차선변경 시점을 정한다. 최대 대기는 버스 쪽(SIGNAL_MAX)이 정한다."""
        p = self._pending
        if p is None:
            return True
        tc = self.tc
        if p["vid"] not in tc.vehicle.getIDList() or t - p["t"] > 30.0:
            self._pending = None
            return True
        f = tc.vehicle.getLeftFollowers(self.bus) if p["left"] else tc.vehicle.getRightFollowers(self.bus)
        if any(v == p["vid"] and 0 <= d <= self.ready_gap for v, d in f):
            self._pending = None
            return True
        return False

    def _s5(self, t, st):
        tc, lap = self.tc, self._lap
        a = lap["s5_alley"]
        if lap["s5"] or a is None:
            return
        route, i, pos, idx = st

        def dist(x):                                          # 버스에서 합류점(들어가는 도로 시작)까지 경로상 거리. 지났으면 None
            if x.into not in route[i:]:
                return None
            d0 = self._gap(route, i, pos, route.index(x.into, i), 0.0)
            return d0 if d0 >= 0 else None

        d = dist(a)
        if d is None or d < 60.0:                             # 이미 지났거나 너무 가까우면 앞에 있는 다른 골목으로 바꾼다
            ahead = [x for x in self.alleys if (dist(x) or 0) >= 150.0]
            if not ahead:
                return
            a = lap["s5_alley"] = self.rng.choice(ahead)
            d = dist(a)
        feeder_len = self.net.getEdge(a.feeder).getLength()
        # 골목이 길면(예: G16 190 m) 시작점에서 출발해 합류점에 닿기까지 20초 넘게 걸려 그 사이 버스가 정류장·신호로 늦어지면
        # 골목 차가 이미 멀리 가 버린다. 합류점 `lead` m 앞에서 출발시킨다(골목 안은 버스가 볼 수 없어 갑자기 생겨도 자연스럽다).
        spawn_pos = max(1.0, feeder_len - self.s5_lead)
        t_cut = (feeder_len - spawn_pos) / 8.0 + 0.5          # 골목 차가 합류점에 닿는 데 걸리는 시간
        if d / max(tc.vehicle.getSpeed(self.bus), 5.0) > t_cut + 3.0:
            return                                            # 아직 이르다: 버스가 합류점에 닿기 3초 전에 골목 차가 먼저 닿도록 맞춘다(2초일 때 버스 뒤로 처진 경우가 있었다)
        lap["s5"] = True
        out = self.rng.choice(self.exits) if self.exits else None
        path = None
        if out is not None:
            seg, _ = self.net.getOptimalPath(self.net.getEdge(a.into), self.net.getEdge(out), vClass="passenger")
            path = [a.feeder] + [e.getID() for e in seg] if seg else None
        path = path or [a.feeder, a.into]
        vid = self._add_car("s5", path, "0", spawn_pos, vtype=self.cutin_type)
        self._record("S5", t, "injected" if vid else False, vid)

    def _s6(self, t, st):
        tc, lap = self.tc, self._lap
        if lap["s6"]:
            return
        route, i, pos, idx = st
        if route[i] != self.s6_before:
            return                                       # 횡단보도 바로 앞 도로에 들어온 뒤에만 넣는다(그 전에 신호 대기가 끼면 보행자가 먼저 건너 버린다)
        d = self._len(self.s6_before) - pos              # 횡단보도(그 도로의 끝)까지 남은 거리
        # 보행자는 7초 걸어 횡단보도에 닿고 약 13초 건넌다. 보행자가 닿을 때 버스가 60 m 앞에 오도록 맞춘다.
        if d < 0 or d > max(60.0, 7.0 * max(tc.vehicle.getSpeed(self.bus), 5.0) + 60.0):
            return
        lap["s6"] = True
        # 이미 횡단보도 위에 사람이 있어도 그 사람이 버스가 50 m 안에 올 때까지 건너고 있을지 알 수 없으므로, 바퀴마다 한 명은
        # 항상 넣는다(자연 발생이 있었는지는 따로 적는다).
        natural = any(tc.person.getRoadID(p).startswith(":S6_wausanro_c") for p in tc.person.getIDList())
        pid = f"inj_s6_{self._seq + 1}"
        self._seq += 1
        try:
            first = self.s6_walk[0]
            tc.person.add(pid, first, max(0.0, tc.lane.getLength(f"{first}_0") - 8.0))
            tc.person.appendWalkingStage(pid, self.s6_walk, 20.0)
            # 버스가 와도 건너는 사람이다(PedSUMO의 '무시하고 건너기'와 같다). 이 사람이 있어야 버스가 서는 상호작용이 보장된다.
            tc.person.setParameter(pid, "junctionModel.ignoreTypes", " ".join(tc.vehicletype.getIDList()))
        except Exception:
            self._record("S6", t, False)
            return
        self._record("S6", t, "injected", pid)
        self.log[-1]["natural_present"] = natural

    def summary(self):
        laps = list(self._laps_done) + ([self._lap] if self._lap else [])
        by = {}
        for r in self.log:
            by.setdefault(r["kind"], {"natural": 0, "injected": 0, "failed": 0})
            k = r["ok"] if r["ok"] in ("natural", "injected") else "failed"
            by[r["kind"]][k] += 1
        return {"laps": len(laps), "by_kind": by}
