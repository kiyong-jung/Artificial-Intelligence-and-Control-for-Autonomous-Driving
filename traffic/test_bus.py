"""시험용 버스 조종(규칙). 주행 정책이 아니라 시나리오를 재현하려는 규칙이다.

노선(사양서 docs/superpowers/specs/2026-11-16-test-bus-route-and-scenarios.md):
  R68 출발 → 성산중학교(s2) 정차, 그 앞에서 오른쪽 차선 변경(S3) → J82 앞 좌회전 차로로 변경(S2) → J82 좌회전 →
  극동방송(s4) → 서교동교회(s5) → J42 좌회전 → 합정역(s6) → J1 앞 좌회전 차로로 변경(S2) → J1 좌회전 → R68(한 바퀴).
  합정역 7번 출구(s1)와 상수역(s3)에는 서지 않는다.

- 버스는 한 번만 넣는다. 노선 끝이 가까워지면 다음 바퀴 노선을 이어 붙인다(다시 넣지 않으므로 바퀴 사이 순간이동이 없다).
- SUMO 자율 차선 변경은 끄고(512) 차선 변경은 **정해진 창(`LanePlan`) 안에서만** 요청한다. 창 밖에서는 차로를 지킨다
  (다음 도로로 이어지지 않는 차로에서 도로 끝이 가까울 때만 예외).
- 신호 교차로 정지선 앞(signal_areas)에서는 차선 변경을 요청하지 않는다. 창 끝까지 목표 차로에 못 가면 SUMO 기본
  차선 변경(1621)으로 넘기고 `missed`에 남긴다. 다음 도로에서 다시 512로 돌린다.
- 후행차를 기다리려고 속도를 줄이지는 않는다. 다만 **비양보형(막는 차)이 A1로 막고 있는 동안에는 버스가 `BLOCKER_PASS_SPEED`로 속도를
  줄여** 그 차를 먼저 보낸다(버스는 차선 변경을 해야 하므로). 막는 차가 지나가거나 풀리면 다시 속도를 푼다. 차선 변경이 늦어도 도착 보장
  (`_force_arrival`)이 정류장 차로·좌회전 차로 앞에서만 세운다.
"""
from dataclasses import dataclass

from .alleys import close_loop
from .signal_areas import forbidden_intervals, is_signalized, no_change_m

BUS = "bus0"
START_EDGE = "R68"
# 서는 정류장: s2 성산중학교, s4 극동방송, s5 서교동교회, s6 합정역
STOP_AT = ("osm_stop_357920024", "osm_stop_5930628870", "osm_stop_3446267493", "osm_stop_3424857165")
CHANGE_WINDOW_M = 120.0
STOP_LANE_LEAD_M = 10.0         # 정류장 차로에는 정류장 시작 이만큼 전까지 들어가 있어야 한다
EXTRA_PER_LANE_M = 40.0         # 차로를 한 칸 더 옮겨야 하면 창을 이만큼 늘린다
BLINKER_RIGHT, BLINKER_LEFT = 1, 2
LC_MANUAL, LC_SUMO = 512, 1621
# 교차로별로 버스가 쓸 좌회전 차로를 정한다(사용자 결정). J82: 독막로 오른쪽에서 4번째 차로(R111b 차로 3) -> 와우산로 오른쪽에서 3번째 차로(R112 차로 2)
LEFT_TURN_LANE = {"J82": 3}
STOP_ARRIVE_LEAD_M = 20.0      # s4·s5·s6: 정류장 시작 이만큼 전까지 정류장 차로에 완전히 들어가 있어야 한다(도착 보장). s2는 위 STOP_LANE_LEAD_M
BLOCKER_PASS_SPEED = 1.5      # 비양보형이 막는 동안 버스 속도 [m/s]. 막는 차는 최소 4 m/s로 달려 이 버스를 앞지른다
FORCE_DECEL, FORCE_MARGIN_M = 2.5, 4.0   # 도착 보장: 이 감속도로 멈출 거리 + 여유 안에 창 끝이 들어오면 멈춰서 기다린다


@dataclass
class LanePlan:
    kind: str              # "S3" 정류장 접근(오른쪽) | "S2" 좌회전 직전(왼쪽)
    edge: str              # 창 끝 도로
    end_pos: float         # 창 끝 위치: 이 지점까지 목표 차로에 있어야 한다
    target_lane: int       # 창 끝 도로에서의 목표 차로
    length: float          # 창 길이(창 끝에서 거슬러)
    eff_length: float = None   # 감속 판단에 쓰는 길이: 창 안에 정류장이 있으면 정류장 끝부터 창 끝까지(J1 창은 s6 안에서 시작)
    ok_lanes: frozenset = None  # S2: 창 끝 도로(edge)의 차로 중 좌회전 차로로 이어지는 차로 번호들

    def __post_init__(self):
        if self.eff_length is None:
            self.eff_length = self.length


def lap_route(net, loop):
    """한 바퀴 노선: R68 다음 bus_loop 전체. 끝(R42)에서 R68로 이어져야 한다."""
    closed = close_loop(net, loop)
    if closed[len(loop):] != [START_EDGE]:
        raise ValueError(f"노선 끝 {loop[-1]}에서 처음 {loop[0]} 사이가 {START_EDGE} 하나가 아니다: {closed[len(loop):]}")
    return [START_EDGE, *loop]


def _back(net, route, k, pos, back):
    """route[k]의 pos에서 back m 거슬러 간 (도로, 위치). 노선 앞을 넘으면 맨 앞."""
    while back > pos and k > 0:
        back -= pos
        k -= 1
        pos = net.getEdge(route[k]).getLength()
    return route[k], max(0.0, pos - back)


def lane_plans(net, route, stops):
    """한 바퀴의 차선 변경 창: S3(s2 정류장 앞) 하나와, 가로변 차로로는 돌 수 없는 신호 교차로 좌회전 앞 S2들(J82, J1)."""
    plans = []
    s2 = next(s for s in stops if s["id"] == STOP_AT[0])
    top = net.getEdge(s2["edge"]).getLaneNumber() - 1
    # 정류장 차로에는 정류장 시작 STOP_LANE_LEAD_M m 전까지 들어가야 한다
    edge, end = s2["edge"], s2["startPos"] - STOP_LANE_LEAD_M
    if end < 0:
        edge, end = _back(net, route, route.index(s2["edge"]), s2["startPos"], STOP_LANE_LEAD_M)
    plans.append(LanePlan("S3", edge, end, s2["lane_idx"],
                          CHANGE_WINDOW_M + EXTRA_PER_LANE_M * max(0, top - s2["lane_idx"] - 1)))
    for sid in STOP_AT[1:]:                            # s4·s5·s6도 정류장 차로 도착을 보장한다(상대 차 시나리오는 없다: kind "STOP")
        s = next((x for x in stops if x["id"] == sid and x["edge"] in route), None)
        if s is None:
            continue
        top = net.getEdge(s["edge"]).getLaneNumber() - 1
        edge, end = s["edge"], s["startPos"] - STOP_ARRIVE_LEAD_M
        ok, target = frozenset({s["lane_idx"]}), s["lane_idx"]
        if end < 0:
            edge, end = _back(net, route, route.index(s["edge"]), s["startPos"], STOP_ARRIVE_LEAD_M)
            ok = _lanes_back(net, route, route.index(s["edge"]), route.index(edge), ok)
            target = min(ok) if ok else s["lane_idx"]
        plans.append(LanePlan("STOP", edge, end, target, CHANGE_WINDOW_M + EXTRA_PER_LANE_M * max(0, top - s["lane_idx"] - 1), ok_lanes=ok))
    for k, (a, b) in enumerate(zip(route, route[1:] + route[:1])):
        ea, eb = net.getEdge(a), net.getEdge(b)
        conns = ea.getConnections(eb)
        node = ea.getToNode()
        if not conns or not all(c.getDirection() in ("l", "L") for c in conns) or not is_signalized(node):
            continue
        if any(c.getFromLane().getIndex() == 1 for c in conns):
            continue                                   # 가로변 차로(1)에서 바로 돌 수 있으면 차선 변경이 없다
        want = LEFT_TURN_LANE.get(node.getID())
        if want is not None and any(c.getFromLane().getIndex() == want for c in conns):
            conns = [c for c in conns if c.getFromLane().getIndex() == want]       # 정한 좌회전 차로 하나만 쓴다
        target = min(c.getFromLane().getIndex() for c in conns)
        end = ea.getLength() - no_change_m(node.getID())
        edge, ok, length = a, frozenset(c.getFromLane().getIndex() for c in conns), CHANGE_WINDOW_M + EXTRA_PER_LANE_M * max(0, target - 1)
        if end < 0:                                    # 창 끝이 앞 도로로 넘어가면 그 도로 기준: 연결을 거슬러 그 도로에서 좌회전 차로로 이어지는 차로를 구한다
            edge, end = _back(net, route, k, ea.getLength(), no_change_m(node.getID()))
            ok = _lanes_back(net, route, k, route.index(edge), ok)
            target = min(ok) if ok else target
        plans.append(LanePlan("S2", edge, end, target, length, ok_lanes=ok))
    plans = [p for p in plans if p.kind != "STOP"] + [p for p in plans if p.kind == "STOP"]   # S3·S2 순서(색인)를 그대로 두고 정류장 도착 창은 뒤에 둔다
    for p in plans:                                    # 창 안에 서는 정류장이 있으면 감속 판단은 정류장 끝부터 센다
        for s in stops:
            if s["id"] in STOP_AT and s["edge"] in route and "endPos" in s:
                d = _route_dist(net, route, s["edge"], s["endPos"], p.edge, p.end_pos)
                if d is not None and 0 < d < p.length:
                    p.eff_length = min(p.eff_length, d)
    return plans


def _lanes_back(net, route, k, j, lanes):
    """route[k]의 차로 집합 lanes로 이어지는 route[j](j <= k)의 차로 번호 집합. 연결을 한 도로씩 거슬러 간다."""
    for m in range(k - 1, j - 1, -1):
        a, b = net.getEdge(route[m]), net.getEdge(route[m + 1])
        lanes = frozenset(c.getFromLane().getIndex() for c in a.getConnections(b) if c.getToLane().getIndex() in lanes)
    return lanes


def _route_dist(net, route, e1, p1, e2, p2):
    """한 바퀴 노선 위 (e1, p1)에서 앞으로 (e2, p2)까지의 거리. e2가 e1 앞에 없으면 None."""
    i, j = route.index(e1), route.index(e2)
    if j < i:
        return None
    if i == j:
        return p2 - p1
    d = net.getEdge(e1).getLength() - p1
    for e in route[i + 1:j]:
        d += net.getEdge(e).getLength()
    return d + p2


class TestBusPilot:
    """tick(tc, t)를 매 스텝 부른다(runner의 bus_hook)."""
    __test__ = False                                   # pytest가 테스트 클래스로 모으지 않게

    SIGNAL_S, REQUEST_DURATION, TIMEOUT_S, COOL_S = 3.0, 5.0, 14.0, 1.0
    # 같은 창에서 이어지는 변경(J1 앞 3~4칸)은 식힘 없이 방향지시등을 켠 채 이 시간 뒤 요청한다.
    # 한 칸에 6초 남짓 걸리면 14 m/s에서 4칸에 350 m가 필요해 창(240 m)을 넘었다(짧은 실행에서 확인).
    NEXT_SIGNAL_S = 1.0

    def __init__(self, tc, net, loop, stops, stop_s=8.0):
        self.tc, self.net = tc, net
        self.lap_edges = lap_route(net, loop)
        self.stops = {s["id"]: s for s in stops}
        self.stop_s = stop_s
        self.plans = lane_plans(net, self.lap_edges, stops)
        self.forbidden = forbidden_intervals(net, self.lap_edges)
        self._len = {}
        self.lap, self.lap_times, self.lap_t0 = 0, [], None
        self.missed = []
        self.changes = {}
        self.why = ""
        self.phase, self.t_phase, self.plan, self.target = "idle", 0.0, None, None
        self._phase_road = None
        self._done = set()
        self._added = False
        self._last_road = None
        self._handover = False
        self._chain = None              # 방금 변경을 마친 창(같은 창의 다음 변경은 빠르게)
        self._signal_s = self.SIGNAL_S
        self.forced = []                    # 도착 보장으로 멈춘 창 (바퀴, 종류)
        self.blocker_active = None          # callable() -> 비양보형이 지금 버스를 막고 있는가(A1). 있으면 버스가 속도를 줄여 먼저 보낸다
        self.speed_limit = None             # callable() -> 속도 제한(m/s) 또는 None. 골목 끝에서 기다리는 차를 본 버스의 감속(S5)
        self._limited = False
        self._forced_p, self._forced_t0 = None, 0.0

    # ---- 노선 ----------------------------------------------------------------------------------
    def _L(self, edge):
        if edge not in self._len:
            self._len[edge] = self.net.getEdge(edge).getLength()
        return self._len[edge]

    def _set_stops(self):
        for sid in STOP_AT:
            if sid in self.stops:
                try:
                    self.tc.vehicle.setBusStop(BUS, sid, duration=self.stop_s)
                except Exception:
                    pass

    def _spawn(self, t):
        tc = self.tc
        tc.route.add("bus_lap", self.lap_edges)
        # R68 차로 1은 R69 차로 2로 이어진다. 가로변 차로(1)가 아니므로 s2 앞에서 오른쪽 차선 변경(S3)이 생긴다
        tc.vehicle.add(BUS, "bus_lap", typeID="bus_test", departLane="1", departPos="0", departSpeed="0")
        try:
            tc.vehicle.setColor(BUS, (255, 0, 0, 255))
        except Exception:
            pass
        tc.vehicle.setLaneChangeMode(BUS, LC_MANUAL)
        self._set_stops()
        self._added, self.lap, self.lap_t0, self._last_road = True, 1, t, START_EDGE

    def _extend(self):
        tc = self.tc
        route, i = list(tc.vehicle.getRoute(BUS)), tc.vehicle.getRouteIndex(BUS)
        if len(route) - i - 1 > 3:
            return
        tc.vehicle.setRoute(BUS, route[i:] + self.lap_edges)
        self._set_stops()

    def _new_lap(self, t):
        self.lap_times.append(t - self.lap_t0)
        self.lap, self.lap_t0 = self.lap + 1, t
        self._done = set()

    # ---- 매 스텝 -------------------------------------------------------------------------------
    def tick(self, tc, t):
        if not self._added:
            self._spawn(t)
            return
        if BUS not in tc.vehicle.getIDList():
            return                                   # 출발 자리가 막혀 아직 들어가지 못했다
        road = tc.vehicle.getRoadID(BUS)
        if not road.startswith(":") and road != self._last_road:
            if road == START_EDGE:
                self._new_lap(t)
            if self._handover:
                tc.vehicle.setLaneChangeMode(BUS, LC_MANUAL)
                self._handover = False
            self._last_road = road
        if road.startswith(":"):
            # 교차로 안에서는 차선을 바꿀 수 없다. 제동 중에 교차로 안으로 밀려 들어간 버스에 강제 정차(속도 0)가 걸려 있으면 풀지 않는 한
            # 그 자리에 선다(J107 안에서 300초). 지나가게 푼다
            if self._forced_p is not None:
                self._end_forced(tc, t)
            return
        self._extend()
        self._pilot(tc, t, road)
        self._apply_speed_limit(tc)

    def _apply_speed_limit(self, tc):
        """골목 차를 본 감속(`speed_limit`)을 건다. 정류장 정차 중이거나 도착 보장 정지 중이면 건드리지 않는다."""
        lim = self.speed_limit() if self.speed_limit is not None else None
        if self.blocker_active is not None and self.blocker_active():
            lim = BLOCKER_PASS_SPEED if lim is None else min(lim, BLOCKER_PASS_SPEED)
        busy = self._forced_p is not None or bool(tc.vehicle.getStopState(BUS) & 1)
        if lim is not None and not busy:
            tc.vehicle.setSpeed(BUS, lim)
            self._limited = True
        elif self._limited and not busy:
            tc.vehicle.setSpeed(BUS, -1)
            self._limited = False

    def _dist_to(self, route, i, pos, plan):
        """버스에서 창 끝까지 노선 거리(지났으면 음수). 이번 바퀴 안에서 찾는다. 없으면 None."""
        if route[i] == plan.edge:
            return plan.end_pos - pos
        d = self._L(route[i]) - pos
        for j in range(i + 1, min(len(route), i + len(self.lap_edges))):
            if route[j] == plan.edge:
                return d + plan.end_pos
            d += self._L(route[j])
        return None

    def _forbidden(self, road, pos):
        return any(lo <= pos <= hi for lo, hi in self.forbidden.get(road, []))

    def _active(self, tc, road, idx, pos):
        route, i = tc.vehicle.getRoute(BUS), tc.vehicle.getRouteIndex(BUS)
        for k, p in enumerate(self.plans):
            if k in self._done:
                continue
            d = self._dist_to(route, i, pos, p)
            if d is None:
                continue
            if d <= 0:                               # 창 끝에 닿았다
                self._done.add(k)
                if road == p.edge and idx != p.target_lane:
                    self.missed.append((self.lap, p.kind, p.edge))
                    tc.vehicle.setLaneChangeMode(BUS, LC_SUMO)
                    self._handover = True
                    self._reset(tc, 0.0)
                continue
            if d <= p.length:
                return k, p, d
        return None

    def _offset(self, tc, road, idx, p):
        if p.kind == "S2":                        # 좌회전 차로 창: 왼쪽으로만 옮긴다(SUMO 최적 차로 오프셋이 오락가락해 오른쪽으로도 갔다)
            good = self._lanes_reaching_target(tc, p)
            if not good or idx in good:
                return 0
            return max(0, min(good, key=lambda k: (abs(k - idx), -k)) - idx)
        if p.kind == "STOP":                      # 지금 도로에서 정류장 차로로 이어지는 차로 쪽으로(도로마다 차로 번호가 달라도 연결을 따른다)
            good = self._lanes_reaching_target(tc, p)
            return 0 if (not good or idx in good) else min(good, key=lambda k: (abs(k - idx), -k)) - idx
        if p.kind == "S3" or road == p.edge:
            return p.target_lane - idx
        lane = tc.vehicle.getLaneID(BUS)
        for entry in tc.vehicle.getBestLanes(BUS):
            if entry[0] == lane:
                return int(entry[3])
        return 0

    def _remap_target(self, tc, t, road):
        """차선 변경 요청 중 도로가 바뀌었다(R54b -> R54: 같은 차로가 번호는 다르다). 연결을 따라 목표 차로를 새 도로 번호로 바꾼다.
        옛 번호 그대로 요청하면 오른쪽으로 되돌아가 처음부터 다시 시작했다. 이어지는 차로가 없으면 요청을 접는다."""
        old, new = self._phase_road, road
        mapped = None
        if self.net.hasEdge(old) and self.net.hasEdge(new):
            outs = [c.getToLane().getIndex() for c in self.net.getEdge(old).getConnections(self.net.getEdge(new))
                    if c.getFromLane().getIndex() == self.target]
            if outs:                                       # 한 차로가 여러 차로로 갈라지면 계획의 목표 차로로 이어지는 쪽을 고른다
                good = self.plan.ok_lanes if (self.plan is not None and self.plan.edge == new and self.plan.ok_lanes) else ()
                mapped = next((k for k in outs if k in good), outs[0])
        if mapped is None:
            self._reset(tc, t)
            return
        self.target, self._phase_road = mapped, new
        if self.phase == "request":
            tc.vehicle.changeLane(BUS, self.target, self.REQUEST_DURATION)

    LATERAL_OK_M = 0.3         # 차선을 바꿨다고 보려면 차로 중심에서 이 안으로 완전히 들어와야 한다

    def _centered(self, tc):
        """버스가 지금 차로 중심 가까이에 있는가(차선 변경 중 차로 번호만 바뀌고 몸이 반쯤 걸친 상태를 제외한다)."""
        try:
            return abs(tc.vehicle.getLateralLanePosition(BUS)) <= self.LATERAL_OK_M
        except Exception:
            return True

    def _pilot(self, tc, t, road):
        idx, pos = tc.vehicle.getLaneIndex(BUS), tc.vehicle.getLanePosition(BUS)
        act = self._active(tc, road, idx, pos)
        forbidden = self._forbidden(road, pos) or self._handover
        if self.phase in ("signal", "request") and self._phase_road not in (None, road):
            self._remap_target(tc, t, road)                # 변경 도중 도로가 바뀌면 목표 차로 번호를 새 도로 번호로 옮긴다
            if self.phase not in ("signal", "request"):
                return
        if self._force_arrival(tc, t, act, idx):
            return                                       # 도착 보장 중(멈춰서 빈틈을 기다린다): 시나리오 단계는 건너뛴다
        if act is not None:
            k, p, d = act
            off = self._offset(tc, road, idx, p)
            why = p.kind
        else:
            k, p, d, off, why = None, None, None, self._emergency(tc, road, pos), "경로"
        if self.phase == "idle":
            if off == 0 or forbidden or tc.vehicle.getStopState(BUS) & 1:
                if self._chain is not None:
                    self._chain = None
                    tc.vehicle.setSignals(BUS, 0)
                return
            n = tc.edge.getLaneNumber(road)
            target = idx + (1 if off > 0 else -1)
            if not 0 <= target < n:
                return
            self._signal_s = self.NEXT_SIGNAL_S if (self._chain is not None and self._chain is p) else self.SIGNAL_S
            self._chain = None
            self.plan, self.target, self.why = p, target, why
            self.changes[why] = self.changes.get(why, 0) + 1
            tc.vehicle.setSignals(BUS, BLINKER_LEFT if off > 0 else BLINKER_RIGHT)
            self.phase, self.t_phase, self._phase_road = "signal", t, road
        elif self.phase == "signal":
            tc.vehicle.setSignals(BUS, BLINKER_LEFT if self.target > idx else BLINKER_RIGHT)
            if off == 0 or forbidden or (self.plan is not None and p is not self.plan):
                self._reset(tc, t)
                return
            if t - self.t_phase < self._signal_s:
                return
            tc.vehicle.changeLane(BUS, self.target, self.REQUEST_DURATION)
            self.phase, self.t_phase = "request", t
        elif self.phase == "request":
            tc.vehicle.setSignals(BUS, BLINKER_LEFT if self.target > idx else BLINKER_RIGHT)
            arrived = idx == self.target and self._centered(tc)
            if arrived and self.plan is not None and not forbidden:
                # 한 칸 옮겼다. 같은 창에서 더 옮겨야 하면 방향지시등을 켠 채 바로 다음 변경으로 간다
                self._chain, self.phase, self.t_phase = self.plan, "idle", t
                self.plan = None
            elif arrived or t - self.t_phase > self.TIMEOUT_S or forbidden \
                    or self.target >= tc.edge.getLaneNumber(road):
                self._reset(tc, t)
            elif int(round((t - self.t_phase) * 10)) % 20 == 0:
                tc.vehicle.changeLane(BUS, self.target, self.REQUEST_DURATION)
        elif self.phase == "cool" and t - self.t_phase >= self.COOL_S:
            self.phase = "idle"

    def _force_arrival(self, tc, t, act, idx):
        """창 끝(정류장 차로 정류장 10 m 전, 좌회전 차로 신호 정지선 앞)까지 목표 차로에 도착하는 것을 보장한다.

        S2·S3 상호작용(후행차 기다리기, 비양보형 보내기)으로 차선 변경이 늦어져 창 끝을 놓치면 버스가 정류장·좌회전 차로 앞에서
        이상한 자세로 서 있으면서 주변 차를 막았다(3바퀴 측정에서 700초 넘는 바퀴). 도착 보장이 시나리오보다 우선이다:
        창 끝이 제동 거리 안인데 아직 목표 차로가 아니면 버스가 멈춰서 목표 차로의 차를 먼저 보내고, 빈틈에 차선을 바꾼다.
        목표 차로에 닿으면 멈춤을 푼다. 하고 있으면 True.
        """
        if act is None or tc.vehicle.getStopState(BUS) & 1:
            return self._end_forced(tc, t) if self._forced_p is not None else False
        _, p, d = act
        # 차로 번호를 창 끝 도로의 목표 차로와 그대로 견주면 안 된다. 도로마다 차로 번호가 달라서(R54 차로 4 -> R42 차로 3) 이미 맞는
        # 차로의 버스를 멈춰 세우고 틀린 차로로 옮기라고 했다. 연결을 따라 창 끝에서 목표 차로에 닿는 차로인지 본다
        good = self._lanes_reaching_target(tc, p)
        if idx in good or not good:
            return self._end_forced(tc, t) if self._forced_p is not None else False
        if p.kind == "S2" and min(good, key=lambda k: (abs(k - idx), -k)) < idx:
            return self._end_forced(tc, t) if self._forced_p is not None else False   # 좌회전 창에서는 오른쪽으로 옮기려고 세우지 않는다
        v = tc.vehicle.getSpeed(BUS)
        if self._forced_p is None:
            if d > v * v / (2.0 * FORCE_DECEL) + FORCE_MARGIN_M:
                return False
            self._forced_p, self._forced_t0 = p, t
            self.forced.append((self.lap, p.kind))
        tc.vehicle.setSpeed(BUS, 0.0)
        step = 1 if min(good, key=lambda k: (abs(k - idx), -k)) > idx else -1       # 목표에 닿는 가장 가까운 차로 쪽으로
        tc.vehicle.setSignals(BUS, BLINKER_LEFT if step > 0 else BLINKER_RIGHT)
        if int(round((t - self._forced_t0) * 10)) % 10 == 0:          # 1초마다 다시 요청한다(빈틈이 생기면 SUMO가 옮긴다)
            tc.vehicle.changeLane(BUS, idx + step, self.REQUEST_DURATION)
        self.phase, self.t_phase = "cool", t
        return True

    def _lanes_reaching_target(self, tc, p):
        """지금 도로의 차로 중, 노선의 연결을 따라가 창 끝 도로(p.edge)의 목표 차로(p.target_lane)에 닿는 차로 번호 집합.
        노선에서 창 끝 도로를 못 찾으면 빈 집합."""
        route, i = tc.vehicle.getRoute(BUS), tc.vehicle.getRouteIndex(BUS)
        j = next((m for m in range(i, len(route)) if route[m] == p.edge), None)
        if j is None or not self.net.hasEdge(route[i]):
            return set()
        n = self.net.getEdge(route[i]).getLaneNumber()
        good = set()
        for k in range(n):
            lanes = {k}
            for m in range(i, j):
                a, b = self.net.getEdge(route[m]), self.net.getEdge(route[m + 1])
                lanes = {c.getToLane().getIndex() for c in a.getConnections(b) if c.getFromLane().getIndex() in lanes}
                if not lanes:
                    break
            if p.ok_lanes and p.kind in ("S2", "STOP"):
                ok = bool(lanes & p.ok_lanes)             # 창 끝 도로에서 좌회전 차로로 이어지는 차로에 닿으면 맞다(J1은 좌회전 차로가 2개)
            elif p.kind == "S2" and j + 1 < len(route) and self.net.hasEdge(route[j + 1]):
                # 좌회전 직전 창: 목표 차로 하나만이 아니라 다음 도로로 이어지는 차로면 맞다(J1은 좌회전 차로가 2개)
                nxt = self.net.getEdge(route[j + 1])
                ok = any(c.getFromLane().getIndex() in lanes for c in self.net.getEdge(p.edge).getConnections(nxt))
            else:
                ok = p.target_lane in lanes
            if ok:
                good.add(k)
        return good

    def _end_forced(self, tc, t):
        tc.vehicle.setSpeed(BUS, -1)
        tc.vehicle.setSignals(BUS, 0)
        self._forced_p, self.phase, self.t_phase, self.plan, self._chain = None, "cool", t, None, None
        return False

    def _emergency(self, tc, road, pos):
        """창 밖 예외: 지금 차로가 다음 도로로 이어지지 않는데 도로 끝이 30 m 안이면 옮긴다."""
        lane = tc.vehicle.getLaneID(BUS)
        for entry in tc.vehicle.getBestLanes(BUS):
            if entry[0] == lane:
                if not entry[4] and self._L(road) - pos < 30.0:
                    return int(entry[3])
                return 0
        return 0

    def _reset(self, tc, t):
        tc.vehicle.setSignals(BUS, 0)
        self.phase, self.t_phase, self.plan, self._chain = "cool", t, None, None
