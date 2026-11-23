"""방법 2: 시나리오 주입기. 시험용 버스의 바퀴마다 상호작용 상대를 계획해 넣는다.

| 시나리오 | 바퀴마다 | 무엇을 | 어디에 |
|---|---|---|---|
| S1 뒤차 | **기본은 끔**(`inject_s1=False`). 켜면 무작위 정류장 하나에서 버스가 서고 뒤 200 m 안에 차가 없을 때 | 승용차 한 대 | 버스 뒤 `back_s1`(220 m), 같은 차로 |
| S2·S3 후행차 | 차선 변경 창(test_bus.lane_plans: S3 s2 앞, S2 J82·J1 앞) 시작 `lane_lead`(60 m) 앞에서, **키퍼 차든 자연 후행차든** 목표 차로 뒤 가장 가까운 차를 상대로 고른다(없으면 키퍼가 한 대 만든다. 주입기는 직접 넣지 않는다) | 키퍼 차 한 대 | 버스 뒤 `back_lane`(버스 반경 + 15 m), 목표 차로. 창이 끝날 때까지 차로를 붙잡는다 |
| S5 끼어들기 | 신호 없는 합류 골목 1~2곳(무작위). 버스가 합류점 90~300 m 앞일 때 차를 골목 끝에 세워 둔다. 버스가 45 m 안이면 시험용 버스가 그 차를 본 것으로 보고 줄인다. 버스가 합류점에 2.5~5초 안으로 닿고 버스와 합류점 사이·들어갈 차로가 비었을 때 풀어 끼어들게 한다(버스를 무시). 버스가 더 빠르면 지나간 뒤에 나온다 | 끼어들기 차 | 골목 끝 정지선(앞부분이 합류점 1.5 m 앞) |
| S6 보행자 | 신호 없는 횡단보도 앞 | 보행자 한 명 | 횡단보도 바로 옆 인도 |

- 바퀴는 시험용 버스 조종의 바퀴 번호(`lap_fn`)로 나눈다. 버스는 바퀴 사이에 사라지지 않는다.
- S2·S3 상대를 고른 뒤 기본(`catch_up=True`)은 상대를 버스 뒤 `follower_gap` 안(5 m + 최소 간격)으로 속도를 높여 붙인다. 버스가 방향지시등을
  켠 순간 목표 차로 후행차가 5 m 안이면 A1이 시작된다. `catch_up=False`면 후행차의 속도를 건드리지 않아 5 m 안 도착을 보장하지 않는다.
- 모든 생성은 시험용 버스 반경(`presence.PRESENCE_RADIUS_M`) 밖에서만 하고, 모든 주입 차량의 경로는 지도 경계에서 끝난다.
- 투입한 것은 `log`에 따로 기록해 자연 발생과 구분한다. 투입된 차는 `carDist`라 주행 모델 4종과 양보 성격이 그대로 붙는다.
"""
import random

from sumolib.geomhelper import positionAtShapeOffset

from .presence import PERSON_MARGIN_M, PRESENCE_RADIUS_M, SPAWN_MARGIN_M, far_from_bus, nearest_boundary_exit, sidewalk_xy
from .signal_areas import is_signalized

from .yield_logic import A1_TRIGGER_GAP_M

CATCH_UP_DV, CATCH_UP_OVER_LIMIT, DROP_BACK_DV = 8.0, 8.0, 3.0   # 후행차 따라잡기: 버스 + 최대 8 m/s, 제한 속도 + 8 m/s까지
# A1은 TraCI 후행차 간격(최소 간격을 뺀 값)이 5 m 안일 때 시작한다. 노선 거리(앞끝·뒤끝 사이)로는 최소 간격 2.5 m를 더한다
FOLLOWER_MIN_GAP_M = 2.5
# S5: 골목 끝에 끼어들기 차를 세워 두고, 버스가 합류점에 곧 닿을 때 풀어 준다(버스는 기다리지 않는다)
S5_PLACE_MIN_M, S5_PLACE_MAX_M = 90.0, 300.0   # 버스가 합류점에서 이 거리 안일 때 골목 끝에 차를 세워 둔다
S5_HOLD_BACK_M, S5_STOP_BEFORE_END_M = 10.0, 1.5   # 정지선 10 m 뒤에서 출발해 앞부분이 합류점 1.5 m 앞에 선다
S5_GO_ETA_MIN_S, S5_GO_ETA_MAX_S = 2.5, 5.0    # 버스가 합류점에 닿기까지 이 시간 안일 때만 풀어 끼어들게 한다(더 빠르면 대기)
S5_CLEAR_AHEAD_M = 30.0                         # 합류 도로에서 이 거리 안의 들어갈 차로에 다른 차가 없어야 한다
S5_SEE_M, S5_SEE_FRAC, S5_SEE_MIN_SPEED = 45.0, 0.5, 3.0   # 버스가 골목 차를 볼 수 있는 거리, 그때 줄이는 속도 비율(제한 속도 기준)
LET_GO_AHEAD_M = 10.0      # 후행차가 버스 뒤끝보다 이만큼 앞서 있으면(이미 지나갔으면) 놓는다

BLINKER_RIGHT, BLINKER_LEFT = 1, 2


class ScenarioInjector:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(runner의 extras)."""

    def __init__(self, traci, net, bus_id, stops, alleys, cutin_type, lap_fn, plans, rng=None, a1_active=None,
                 back_s1=220.0, back_lane=PRESENCE_RADIUS_M + 15.0, s5_per_lap=(1, 2),
                 follower_gap=(0.0, A1_TRIGGER_GAP_M + FOLLOWER_MIN_GAP_M),
                 lane_lead=300.0,
                 s6_edges=("-R112", "R112.112"), s6_edge_before="R112", s5_lead=60.0, follower_check=80.0,
                 dwell_s=8.0, stop_ids=None, inject_s1=False, keeper=None, catch_up=True):
        # dwell_s: 시험용 버스의 정류장 정차 시간. stop_ids: 버스가 실제로 서는 정류장(없으면 stops 전부).
        # 끼어들기 출발 시점에 버스의 정차 시간을 넣는다(X_dongmak6은 s2 끝 바로 뒤에서 합류해 정차 중에 골목 차가 먼저 지나갔다)
        # inject_s1: False(기본)면 S1(정차 버스 뒤 차 한 대) 주입을 끈다. 정차 시각 기록은 S5 출발 시점에 쓰므로 그대로 한다.
        self.inject_s1 = inject_s1
        # keeper: 버스 주변 차량 밀도 유지기(BusWindowKeeper). 있으면 S2·S3는 차를 주입하지 않고 키퍼 차든 자연 후행차든 버스 목표 차로 뒤 가장 가까운 차를
        # 상대로 고른다(없으면 키퍼에게 한 대를 만들게 한다). 5 m 안으로 붙이는 것(_catch_up)과 A1은 그대로다. 없으면(None) 예전처럼 직접 투입한다.
        self.keeper = keeper
        # catch_up: True(기본)면 상대 후행차를 버스 5 m 안으로 속도를 높여 붙인다. False면 상대는 고르기만 하고 속도를 건드리지 않는다
        # (버스와 가까운 후행차가 자연스럽게 가까워질 때만 A1이 시작한다. 5 m 안 도착을 보장하지 않는다)
        self.catch_up = catch_up
        self.platoon_n, self.platoon_gap = 3, 14.0       # 적색 대기 중 넣는 후행차 수와 간격(m)
        self.dwell_s = dwell_s
        self.stop_ids = set(stop_ids) if stop_ids is not None else {s["id"] for s in stops}
        self._stop_t0, self._left_stop = None, None
        self._t_now, self._cur_alley = 0.0, None
        self.tc, self.net, self.bus = traci, net, bus_id
        self.stops = {s["id"]: s for s in stops}
        self.alleys = list(alleys)
        self.cutin_type = cutin_type
        self.lap_fn, self.plans = lap_fn, list(plans)
        self.a1_active = a1_active
        self.rng = rng or random.Random(0)
        self.back_s1, self.back_lane = back_s1, back_lane
        self.s5_per_lap, self.follower_gap, self.lane_lead = s5_per_lap, follower_gap, lane_lead
        self.s6_walk, self.s6_before = list(s6_edges), s6_edge_before
        self.s5_lead, self.follower_check = s5_lead, follower_check
        self.log = []
        self._lens = {}
        self._lap, self._lap_no = None, None
        self._laps_done = ()
        self._seq = 0
        self._was_stopped = False

    # ---- 바퀴 ---------------------------------------------------------------------------------
    def _new_lap(self, t):
        stop_ids = sorted(i for i in self.stops if i in self.stop_ids)      # S1은 버스가 실제로 서는 정류장에서만
        k = min(len(self.alleys), self.rng.randint(*self.s5_per_lap)) if self.alleys else 0
        self._lap = {"t0": t, "s1_stop": self.rng.choice(stop_ids) if stop_ids else None, "s1": False,
                     "S3": next((p for p in self.plans if p.kind == "S3"), None),
                     "S2": [p for p in self.plans if p.kind == "S2"],
                     "lane": {}, "lane_done": set(), "seen": set(),
                     "S5": self.rng.sample(self.alleys, k) if k else [], "s5_done": set(), "s5": {},
                     "S6": False}

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
            if is_signalized(self.net.getEdge(route[k]).getToNode()):
                # 신호 교차로 뒤에서 출발시키면 후행차가 적색에 걸려 버스에 닿지 못한다. 신호를 지난 도로(k+1) 시작에서 출발시킨다.
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

    def _ahead_index(self, route, i, edge, span=None):
        """버스 위치(i)부터 앞으로 edge가 처음 나오는 index. 없으면 None."""
        hi = len(route) if span is None else min(len(route), i + span)
        for j in range(i, hi):
            if route[j] == edge:
                return j
        return None

    def _dist_to(self, st, edge, pos2):
        route, i, pos, _ = st
        j = self._ahead_index(route, i, edge, span=40)
        return None if j is None else self._gap(route, i, pos, j, pos2)

    # ---- 생성 ---------------------------------------------------------------------------------
    def _xy(self, edge, pos):
        return positionAtShapeOffset(self.net.getEdge(edge).getShape(), pos)

    def _exit_path(self, edges):
        """edges 끝 도로에서 지도 경계까지 이어 붙인 경로(중간에서 사라지지 않게)."""
        tail = nearest_boundary_exit(self.net, edges[-1])
        return list(edges[:-1]) + tail if tail else list(edges)

    def _add_car(self, kind, route_edges, lane, pos, vtype="carDist", speed="max"):
        """(차 id, 이유). 버스 반경 안이면 ("", "near_bus"), 넣지 못하면 ("", "failed")."""
        pos = max(1.0, pos)
        if not far_from_bus(self.tc, self.bus, self._xy(route_edges[0], pos), PRESENCE_RADIUS_M + SPAWN_MARGIN_M):
            return "", "near_bus"       # 한 스텝에 버스와 차가 가까워지는 여유(SPAWN_MARGIN_M)를 더한다
        self._seq += 1
        vid, rid = f"inj_{kind}{self._seq}", f"r_inj_{kind}{self._seq}"
        try:
            self.tc.route.add(rid, self._exit_path(route_edges))
            self.tc.vehicle.add(vid, rid, typeID=vtype, departLane=lane, departPos=f"{pos:.1f}", departSpeed=speed)
        except Exception:
            return "", "failed"
        return vid, "injected"

    def _record(self, kind, t, ok, vid=None):
        self.log.append({"t": round(t, 1), "lap": len(self._laps_done) + 1, "kind": kind, "ok": ok, "id": vid})

    # ---- 매 스텝 ------------------------------------------------------------------------------
    def update(self, t):
        lap = self.lap_fn()
        if lap != self._lap_no:
            if self._lap is not None:
                self._laps_done = self._laps_done + (self._lap,)
            self._lap_no = lap
            self._new_lap(t)
        tc = self.tc
        self._t_now = t
        if self.bus not in tc.vehicle.getIDList():
            return
        self._catch_up(t)
        st = self._state()
        if st is None:
            return
        self._s1(t, st)
        self._lanes(t, st)
        self._s5(t, st)
        self._s6(t, st)

    def _s1(self, t, st):
        tc, lap = self.tc, self._lap
        state = tc.vehicle.getStopState(self.bus)
        stopped = bool(state & 1 and state & 16)
        if stopped and not self._was_stopped:
            self._stop_t0 = t                            # 정차를 시작한 시각(끼어들기 출발 시점 계산에 쓴다)
        elif not stopped:
            if self._was_stopped:
                self._left_stop = tc.vehicle.getRoadID(self.bus)     # 방금 떠난 정류장 도로(다시 '앞의 정류장'으로 보지 않는다)
            self._stop_t0 = None
        if stopped and not self._was_stopped and not lap["s1"] and self.inject_s1:
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
                        vid, why = self._add_car("s1", route[k:i + 2], str(idx), p)
                        self._record("S1", t, why if vid else why, vid or None)
        self._was_stopped = stopped

    # ---- S2·S3 후행차 ------------------------------------------------------------------------
    @staticmethod
    def _key(plan):
        return (plan.kind, plan.edge)

    def _lane_plans(self):
        lap = self._lap
        return ([lap["S3"]] if lap["S3"] is not None else []) + list(lap["S2"])

    def _track(self, plan, vid, t):
        """이번 창의 상대로 삼는다. 창이 끝나거나 A1이 시작될 때까지 차로를 붙잡는다(SUMO가 버스 뒤 같은 차로로 옮기면
        목표 차로 후행차가 아니게 된다 — 짧은 실행에서 확인)."""
        tc = self.tc
        try:
            lc0 = tc.vehicle.getLaneChangeMode(vid)
            tc.vehicle.setLaneChangeMode(vid, 0)
        except Exception:
            lc0 = None
        self._lap["lane"][self._key(plan)] = {"vid": vid, "t": t, "released": False, "plan": plan, "lc0": lc0}

    def _release(self, rec):
        tc = self.tc
        tc.vehicle.setSpeed(rec["vid"], -1)
        if rec.get("lc0") is not None:
            tc.vehicle.setLaneChangeMode(rec["vid"], rec["lc0"])
        rec["released"] = True

    def _in_served_stop(self, st):
        """버스가 서는 정류장의 구간(시작~끝) 안에 있는가."""
        route, i, pos, _ = st
        for s in self.stops.values():
            if s["id"] in self.stop_ids and s["edge"] == route[i] and "endPos" in s \
                    and s.get("startPos", 0.0) - 1.0 <= pos <= s["endPos"] + 1.0:
                return True
        return False

    def _signal_ahead(self, route, i, j_end):
        """버스 도로(i)부터 창 끝 도로 앞(j_end - 1)까지 신호 교차로로 끝나는 도로가 있는가."""
        return any(is_signalized(self.net.getEdge(route[k]).getToNode()) for k in range(i, j_end))

    def _lane_back(self, route, k, j_end, target):
        """창 끝 도로(route[j_end])의 target 차로로 곧장 이어지는 route[k]의 차로. 못 찾으면 None.
        J1 앞처럼 차로를 여러 칸 옮기는 창에서는 후행차가 마지막 변경의 목표 차로(좌회전 차로)에 있게 된다."""
        lane = target
        for m in range(j_end, k, -1):
            a, b = self.net.getEdge(route[m - 1]), self.net.getEdge(route[m])
            froms = [c.getFromLane().getIndex() for c in a.getConnections(b) if c.getToLane().getIndex() == lane]
            if not froms:
                return None
            lane = max(froms)
        return lane

    def tracked_ids(self):
        """이번 바퀴에서 S2·S3 상대로 추적 중인(아직 놓지 않은) 차 id. 키퍼가 이 차들은 지우지 않는다."""
        lap = self._lap
        return {rec["vid"] for rec in lap["lane"].values() if not rec["released"]} if lap else set()

    def _is_keeper_car(self, vid):
        return self.keeper is not None and self.keeper.is_keeper_car(vid)

    def _plan_seen(self, key):
        """이번 바퀴에서 이 창에 한 번이라도 들어섰는가(창이 아직 오지 않은 것과 이미 지난 것을 가른다)."""
        return key in self._lap["seen"]

    def _add_lane_car(self, path, lane, p):
        """S2·S3 상대 한 대를 만든다. 키퍼가 있으면 키퍼가 만든다(키퍼 차로 관리됨). (차 id, 이유)."""
        if self.keeper is not None:
            vid = self.keeper.spawn_car(path, int(lane), p)
            return (vid, "injected") if vid else ("", "near_bus")
        return self._add_car("lane", path, lane, p)

    def _lanes(self, t, st):
        tc, lap = self.tc, self._lap
        route, i, pos, idx = st
        for plan in self._lane_plans():
            key = self._key(plan)
            if key in lap["lane"] or key in lap["lane_done"]:
                continue
            d_end = self._dist_to(st, plan.edge, plan.end_pos)
            if d_end is not None and d_end <= 0 and self._plan_seen(key):
                lap["lane_done"].add(key)               # 창이 끝났는데 상대를 못 정했다: 놓친 것으로 기록한다(측정에서 보이게)
                self._record(plan.kind, t, "missed")
                continue
            if d_end is None or d_end <= 0:
                continue
            lap["seen"].add(key)
            d_start = d_end - plan.length
            if d_start > self.lane_lead:
                continue
            if plan.eff_length < plan.length - 1e-6 and d_end > plan.eff_length:
                continue                                 # 창 안 정류장(J1 앞 s6)을 아직 떠나지 않았다: 떠난 뒤 넣는다
            if self._in_served_stop(st):
                continue                                 # 정류장 구간 안(J1 창이 길어져 s6이 창 밖이어도): 떠난 뒤 넣는다
            j_end = self._ahead_index(route, i, plan.edge, span=40)
            if self._signal_ahead(route, i, j_end):
                # 버스와 창 사이에 신호가 있다. 지나는 중에 넣으면 후행차가 적색에 걸린다. 버스가 그 신호에서 적색으로 서 있으면
                # 목표 차로 줄에 여러 대를 50 m 이상 뒤부터 넣어 녹색에 같이 출발하게 한다. 아니면 지난 뒤 넣는다
                if self.keeper is None and tc.vehicle.getSpeed(self.bus) < 0.5 and self._queue_platoon(t, st, plan, j_end):
                    continue
                continue
            left = plan.kind == "S2"
            f = tc.vehicle.getLeftFollowers(self.bus) if left else tc.vehicle.getRightFollowers(self.bus)
            near = [(d, v) for v, d in f if v and 0 <= d <= self.follower_check]
            if near:
                self._track(plan, min(near)[1], t)
                self._record(plan.kind, t, "natural", min(near)[1])
                continue
            bp = self._back_point(route, i, pos, self.back_lane)
            if bp is None:
                if d_start > 0:
                    continue                             # 노선 뒤가 아직 없다(첫 바퀴 시작). 버스가 나아간 뒤 다시 본다
                lap["lane_done"].add(key)
                self._record(plan.kind, t, False)
                continue
            k, p = bp
            if k > 0 and p <= 1.0 and is_signalized(self.net.getEdge(route[k - 1]).getToNode()):
                p = 10.0                                 # 신호 바로 뒤 도로 시작은 교차로를 빠져나오는 차와 부딪힌다(J42에서 확인)
            lanes = tc.edge.getLaneNumber(route[k])
            target = self._lane_back(route, k, j_end, plan.target_lane)
            if target is None:
                target = plan.target_lane if plan.kind == "S3" else idx + 1
            lane = str(min(max(1, target), lanes - 1))
            vid, why = self._add_lane_car(self._window_path(route, k, j_end), lane, p)
            if vid:
                self._track(plan, vid, t)
                self._record(plan.kind, t, "injected", vid)
            elif why in ("near_bus", "failed"):
                continue                                 # 버스가 출발 지점에서 멀어질 때까지 창이 끝나기 전까지 계속 다시 본다(S2·S3는 항상 일어나야 한다)
            else:
                lap["lane_done"].add(key)
                self._record(plan.kind, t, why)

    def _window_path(self, route, k, j_end):
        """route[k]에서 창 끝 도로(j_end)까지, 그 다음 도로(좌회전 도착 도로)까지 버스와 같은 길. 그래야 SUMO가 그 회전 차로로
        이어지는 차로를 고른다(J1: 곧장 나가는 경로였을 때 R54b 3 -> R54 4로 가 버스의 중간 목표 차로에 있었다)."""
        path = list(route[k:j_end + 1])
        m = j_end
        for _ in range(3):                                                     # 창 끝 도로가 신호 앞이 아니면(J1: R54) 신호를 지나 회전 도착 도로까지 잇는다
            nxt = route[(m + 1) % len(route)]                                  # 노선 끝이면 다음 바퀴 처음(R42 -> R68)
            if not self.net.getEdge(route[m]).getConnections(self.net.getEdge(nxt)):
                break
            path.append(nxt)
            if is_signalized(self.net.getEdge(route[m]).getToNode()):
                break
            m = (m + 1) % len(route)
        return path

    def _queue_platoon(self, t, st, plan, j_end):
        """버스가 신호에서 적색으로 서 있을 때, 목표 차로 줄에 후행차 `platoon_n`대를 버스 반경 + 10 m 이상 뒤부터 간격을 두고
        넣는다. 가장 앞(버스에 가까운) 차를 이번 창의 상대로 삼는다. 넣었으면 True."""
        tc, lap = self.tc, self._lap
        route, i, pos, idx = st
        left = plan.kind == "S2"
        f = tc.vehicle.getLeftFollowers(self.bus) if left else tc.vehicle.getRightFollowers(self.bus)
        if any(v and 0 <= d <= self.follower_check for v, d in f):
            return False                                 # 이미 가까이 있는 차가 있다: 그 차가 상대다
        bp = self._back_point(route, i, pos, PRESENCE_RADIUS_M + 10.0)
        if bp is None:
            return False
        k, p = bp
        target = self._lane_back(route, k, j_end, plan.target_lane)
        lanes = tc.edge.getLaneNumber(route[k])
        if target is None:
            return False
        lane = str(min(max(1, target), lanes - 1))
        path = self._window_path(route, k, j_end)
        made = []
        for n in range(self.platoon_n):
            at = p - n * self.platoon_gap
            if at < 1.0:
                break
            vid, why = self._add_car("lane", path, lane, at)
            if not vid:
                break
            made.append(vid)
        if not made:
            return False
        self._track(plan, made[0], t)
        for vid in made:
            self._record(plan.kind, t, "injected", vid)
        return True

    def _follower_gap(self, vid, plan):
        """버스 뒤끝에서 후행차 앞끝까지 노선 거리(양수 = 버스 뒤). 같은 노선 위에 없으면 None."""
        tc = self.tc
        st = self._state()
        if st is None:
            return None
        route, i, pos, _ = st
        road = tc.vehicle.getRoadID(vid)
        if road.startswith(":"):
            return None
        j = next((k for k in range(i, max(-1, i - 12), -1) if route[k] == road), None)
        if j is None:
            return None
        fpos = tc.vehicle.getLanePosition(vid)
        ahead = self._gap(route, j, fpos, i, pos) if j < i else pos - fpos
        return ahead - tc.vehicle.getLength(self.bus)

    def _partner(self, plan, rec):
        """이번 창에서 버스에 가장 가까운 목표 차로 후행차 (id, 노선 간격). 주입한 차든 자연 차든 가장 가까운 차가 상대다.
        TraCI 후행차 간격에는 최소 간격을 더해 주입 차의 노선 간격과 같은 기준으로 견준다. 없으면 None."""
        tc = self.tc
        best = None
        if rec is not None and rec["vid"] in tc.vehicle.getIDList():
            g = self._follower_gap(rec["vid"], plan)
            if g is not None:
                best = (rec["vid"], g)
        try:
            f = tc.vehicle.getLeftFollowers(self.bus) if plan.kind == "S2" else tc.vehicle.getRightFollowers(self.bus)
        except Exception:
            f = []
        for vid, d in f:
            if vid and 0 <= d and (best is None or d + FOLLOWER_MIN_GAP_M < best[1]):
                best = (vid, d + FOLLOWER_MIN_GAP_M)
        return best

    def _catch_up(self, t):
        tc = self.tc
        if self._lap is None or not self._lap["lane"]:
            return
        ids = set(tc.vehicle.getIDList())
        v_bus = tc.vehicle.getSpeed(self.bus)
        st = self._state()
        lo, hi = self.follower_gap
        for rec in self._lap["lane"].values():
            if rec["released"]:
                continue
            vid, plan = rec["vid"], rec["plan"]
            if vid not in ids:
                rec["released"] = True
                continue
            passed = st is not None and (self._dist_to(st, plan.edge, plan.end_pos) or 0.0) <= 0
            if (self.a1_active is not None and self.a1_active(vid)) or passed:
                self._release(rec)                       # A1 상호작용이 시작되었거나 창이 끝났다: 손을 뗀다
                continue
            partner = self._partner(plan, rec)
            if partner is not None and partner[0] != vid:
                self._release(rec)                       # 더 가까운 차가 있다: 그 차가 상대다. 주입한 차는 시나리오에서 빠진다
                continue
            if not self.catch_up:
                continue                                 # 속도를 건드리지 않는다: 자연 주행하는 후행차를 상대로 추적만 한다
            if self._stop_t0 is not None and tc.vehicle.getLaneIndex(vid) != tc.vehicle.getLaneIndex(self.bus):
                tc.vehicle.setSpeed(vid, -1)             # 버스가 정류장에 서 있다: 다른 차로의 차를 버스 뒤에 두려고 세우지 않는다
                continue
            g = self._follower_gap(vid, plan)
            if g is None:
                continue
            if g < -LET_GO_AHEAD_M:
                self._release(rec)                       # 이미 버스를 지나 앞에 있다: 되돌아오게 하지 않는다
                continue
            if g > hi:
                try:
                    cap = tc.lane.getMaxSpeed(tc.vehicle.getLaneID(vid)) + CATCH_UP_OVER_LIMIT
                except Exception:
                    cap = 22.0
                target = min(max(v_bus + min(CATCH_UP_DV, (g - hi) / 2.0), v_bus + 2.0), cap)   # 최대한 빨리 5 m 안으로
            elif g < lo:
                target = max(0.0, v_bus - DROP_BACK_DV)       # 버스 옆이나 앞에 있다: 확실히 뒤로 물러난다
            else:
                target = v_bus
            tc.vehicle.setSpeed(vid, target)

    # ---- S5 ---------------------------------------------------------------------------------
    def _stop_pending(self, st, edge):
        """버스가 edge(합류 도로) 시작에 닿기 전에 설 정류장이 남아 있는가. 합류 도로 위 정류장은 합류점(도로 시작) 뒤에
        있으므로 넣지 않는다(X_jandari3: s5가 합류 도로 -R96.201 위라 정차 시간을 더하면 출발 조건을 끝내 못 맞췄다)."""
        route, i, pos, _ = st
        j = self._ahead_index(route, i, edge, span=40)
        if j is None:
            return False
        for s in self.stops.values():
            if s["id"] not in self.stop_ids or "endPos" not in s:
                continue
            k = self._ahead_index(route, i, s["edge"], span=40)
            if k is None or k >= j:
                continue
            if k > i or (s["endPos"] >= pos - 1.0 and route[i] != self._left_stop):
                return True
        return False

    def _bus_eta(self, t, st, d, edge):
        """시험용 버스가 d m 앞(edge 시작)에 닿기까지 걸리는 시간. 정류장 정차 시간을 넣는다."""
        v = max(self.tc.vehicle.getSpeed(self.bus), 5.0)
        if self._stop_t0 is not None:                    # 지금 정류장에 서 있다: 남은 정차 + 출발해 닿는 시간
            return max(0.0, self.dwell_s - (t - self._stop_t0)) + d / 5.0 + 2.0
        if self._stop_pending(st, edge):                 # 합류점 앞에 설 정류장이 남았다: 정차와 감·가속 시간을 더한다
            return d / v + self.dwell_s + 5.0
        return d / v

    def _s5(self, t, st):
        """S5: 골목 끝(정지선)에 끼어들기 차를 세워 두었다가, 버스가 합류점에 곧 닿을 때 풀어 끼어들게 한다. 버스는 기다리지 않는다."""
        lap = self._lap
        for a in lap["S5"]:
            if a.junction in lap["s5_done"]:
                continue
            d = self._dist_to(st, a.into, 0.0) if st is not None else None
            rec = lap["s5"].get(a.junction)
            if rec is None:
                self._s5_place(t, st, a, d)
            else:
                self._s5_hold(t, st, a, d, rec)

    def _s5_place(self, t, st, a, d):
        lap = self._lap
        if d is None or d < 0:
            return                                        # 이번 바퀴에 아직 앞에 없다(또는 지났다)
        j_into = self._ahead_index(st[0], st[1], a.into, span=40)
        if j_into is not None and self._signal_ahead(st[0], st[1], j_into):
            return                                        # 버스와 합류점 사이에 신호가 있다: 지난 뒤에 세운다
        if d < S5_PLACE_MIN_M and self._stop_t0 is None and not self._stop_pending(st, a.into):
            lap["s5_done"].add(a.junction)
            self._record("S5", t, "late")                 # 버스가 너무 가까워 골목 끝에 미리 세워 둘 수 없다
            return
        if d > S5_PLACE_MAX_M:
            return
        feeder_len = self.net.getEdge(a.feeder).getLength()
        self._cur_alley = a
        vid, why = self._add_car("s5", [a.feeder, a.into], "0", feeder_len - S5_HOLD_BACK_M, vtype=self.cutin_type,
                                 speed="0")
        if vid:
            lap["s5"][a.junction] = {"vid": vid, "t0": t, "seen": False, "cmd": False, "released": False}
            self._record("S5", t, "injected", vid)

    def _s5_hold(self, t, st, a, d, rec):
        tc, lap = self.tc, self._lap
        vid = rec["vid"]
        if vid not in tc.vehicle.getIDList():
            if rec["seen"] or t - rec["t0"] > 10.0:
                lap["s5_done"].add(a.junction)            # 사라졌거나 끝내 나타나지 않았다
            return
        rec["seen"] = True
        feeder_len = self.net.getEdge(a.feeder).getLength()
        if not rec["cmd"]:                                # 정지선 앞에서 서 있게 한다(앞부분이 합류점 가까이, 뒷부분은 골목)
            try:
                tc.vehicle.setStop(vid, a.feeder, pos=feeder_len - S5_STOP_BEFORE_END_M, laneIndex=0, duration=1e5)
            except Exception:
                lap["s5_done"].add(a.junction)
                return
            rec["cmd"] = True
        if tc.vehicle.getRoadID(vid) != a.feeder:
            lap["s5_done"].add(a.junction)                # 골목을 벗어났다(합류 중이거나 지났다)
            return
        if rec["released"] or st is None:
            return
        passed = d is None or d < 0
        if passed:                                        # 버스가 먼저 지나갔다: 보통 차처럼 나온다(길을 터 준 셈이다)
            self._s5_release(vid, ignore=False)
            rec["released"] = True
            return
        eta = self._bus_eta(t, st, d, a.into)
        if S5_GO_ETA_MIN_S <= eta <= S5_GO_ETA_MAX_S and self._merge_clear(st, a, vid):
            if self._s5_release(vid, ignore=True):
                rec["released"] = True

    def _s5_release(self, vid, ignore):
        tc = self.tc
        try:
            tc.vehicle.resume(vid)                        # 아직 정지선에 서지 않았으면 실패한다: 다음 스텝에 다시 한다
        except Exception:
            return False
        if ignore:
            try:
                tc.vehicle.setParameter(vid, "junctionModel.ignoreIDs", self.bus)
            except Exception:
                pass
        return True

    def _merge_clear(self, st, a, own_vid):
        """끼어들기 차와 버스 사이, 그리고 차가 들어갈 차로에 다른 차가 없는가.

        버스에서 합류점까지 사이 도로(어느 차로든)와 합류 교차로 안, 합류 도로 앞 `S5_CLEAR_AHEAD_M` m 안의 들어갈 차로를 본다."""
        tc = self.tc
        route, i, pos, _ = st
        j = self._ahead_index(route, i, a.into, span=40)
        if j is None:
            return False
        lanes = {c.getToLane().getIndex() for c in self.net.getEdge(a.feeder).getConnections(self.net.getEdge(a.into))}
        skip = {self.bus, own_vid}
        for k in range(i, j):                             # 버스 앞 ~ 합류 도로 직전
            for v in tc.edge.getLastStepVehicleIDs(route[k]):
                if v in skip:
                    continue
                if k == i and tc.vehicle.getLanePosition(v) <= pos:
                    continue                              # 버스 뒤
                return False
        for v in tc.edge.getLastStepVehicleIDs(a.into):
            if v in skip:
                continue
            if tc.vehicle.getLanePosition(v) <= S5_CLEAR_AHEAD_M and tc.vehicle.getLaneIndex(v) in lanes:
                return False
        for v in tc.vehicle.getIDList():                  # 합류 교차로 안
            if v not in skip and tc.vehicle.getRoadID(v).startswith(f":{a.junction}"):
                return False
        return True

    def bus_speed_limit(self):
        """시험용 버스의 속도 제한(m/s) 또는 None. 골목 끝에 끼어들기 차가 대기 중이고 버스가 합류점 `S5_SEE_M` m 안이면
        버스가 그 차를 본 것으로 보고 제한 속도의 `S5_SEE_FRAC` 배로 줄인다(의미 있는 상호작용). 차가 골목을 벗어나면 푼다."""
        lap = self._lap
        if lap is None or not lap["s5"]:
            return None
        st = self._state()
        if st is None:
            return None
        for a in lap["S5"]:
            rec = lap["s5"].get(a.junction)
            if not rec or not rec["seen"] or a.junction in lap["s5_done"]:
                continue
            d = self._dist_to(st, a.into, 0.0)
            if d is not None and 0.0 <= d <= S5_SEE_M:
                return max(S5_SEE_MIN_SPEED, S5_SEE_FRAC * self.tc.vehicle.getAllowedSpeed(self.bus))
        return None

    # ---- S6 ---------------------------------------------------------------------------------
    def _s6(self, t, st):
        tc, lap = self.tc, self._lap
        if lap["S6"]:
            return
        route, i, pos, idx = st
        if route[i] != self.s6_before:
            return                                       # 횡단보도 바로 앞 도로에 들어온 뒤에만 넣는다
        d = self._len(self.s6_before) - pos
        # 보행자는 7초 걸어 횡단보도에 닿고 약 13초 건넌다. 보행자가 닿을 때 버스가 60 m 앞에 오도록 맞춘다.
        if d < 0 or d > max(60.0, 7.0 * max(tc.vehicle.getSpeed(self.bus), 5.0) + 60.0):
            return
        first = self.s6_walk[0]
        start = max(0.0, tc.lane.getLength(f"{first}_0") - 8.0)
        if not far_from_bus(tc, self.bus, sidewalk_xy(self.net, first, start), PRESENCE_RADIUS_M + PERSON_MARGIN_M):
            return                                       # 버스 반경 안: 버스가 지나가기 전 다음 스텝에 다시 본다
        lap["S6"] = True
        natural = any(tc.person.getRoadID(p).startswith(":S6_wausanro_c") for p in tc.person.getIDList())
        self._seq += 1
        pid = f"inj_s6_{self._seq}"
        try:
            tc.person.add(pid, first, start)
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
        return {"laps": len(laps), "by_kind": by,
                "s5_alleys_per_lap": [[a.junction for a in lap["S5"]] for lap in laps]}
