"""시험용 버스 조종(규칙). 주행 정책이 아니라 시나리오를 재현하려는 규칙이다.

노선(사양서 docs/superpowers/specs/2026-11-16-test-bus-route-and-scenarios.md):
  R68 출발 → 성산중학교(s2) 정차, 그 앞에서 오른쪽 차선 변경(S3) → J82 앞 좌회전 차로로 변경(S2) → J82 좌회전 →
  극동방송(s4) → 서교동교회(s5) → J42 좌회전 → 합정역(s6) → J1 앞 좌회전 차로로 변경(S2) → J1 좌회전 → R68(한 바퀴).
  합정역 7번 출구(s1)와 상수역(s3)에는 서지 않는다.

- 버스는 한 번만 넣는다. 노선 끝이 가까워지면 다음 바퀴 노선을 이어 붙인다(다시 넣지 않으므로 바퀴 사이 순간이동이 없다).
- SUMO 자율 차선 변경은 끄고(1536) 차선 변경은 **정해진 창(`LanePlan`) 안에서만** 요청한다. 창 밖에서는 차로를 지킨다
  (다음 도로로 이어지지 않는 차로에서 도로 끝이 가까울 때만 예외).
- 신호 교차로 정지선 앞(signal_areas)에서는 차선 변경을 요청하지 않는다.
- 창 끝이 제동 거리 안인데 지금 차로가 목표 차로에 닿지 못하면 SUMO 기본 차선 변경(1621)에 넘긴다(`handed`). 달리는 차로에
  멈춰 서지 않고, SUMO의 전략 차선 변경과 이웃 차의 협조로 빈틈을 찾는다. 창 끝까지 못 가면 `missed`에 남긴다. 정류장 창(S3)이면
  그 정류장을 건너뛰고, 좌회전 창(S2)이면 SUMO에 맡긴 채 정지선에서 기다릴 수 있다. 창 끝 도로를 지나면 다시 1536으로 돌린다.
- 상대를 기다리지 않는다. 목표 차로에 차가 있든 없든 방향지시등을 켜고 정해진 시간 뒤 요청한다.
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
# 1536 = 512(자율 변경 끔, TraCI 요청의 안전 간격은 지킴) + bit10(TraCI 요청과 겹치지 않는 차로 안 횡이동, 곧 가운데 정렬).
# 512만 쓰면 차선 변경이 차로 가운데에서 끝나지 않을 때 다시 정렬하지 않는다. busy seed 3에서 R77->R111 경계에서 끊긴 S2 변경이
# 버스를 오른쪽으로 0.97 m 밀어 둔 채 J82를 좌회전했고, 나가는 도로 R112의 오른쪽 차로가 인도라 SUMO가 교차로 출구에서 버스를
# 세웠다(MSVehicle::brakeForOverlap). env의 ego도 같은 이유로 1536이다(env/road_config.py)
LC_MANUAL, LC_SUMO = 1536, 1621
HANDOVER_DECEL, HANDOVER_MARGIN_M = 2.5, 4.0   # 이 감속도로 설 거리 + 여유 안에 창 끝이 들어오면 SUMO에 넘긴다


@dataclass
class LanePlan:
    kind: str              # "S3" 정류장 접근(오른쪽) | "S2" 좌회전 직전(왼쪽)
    edge: str              # 창 끝 도로
    end_pos: float         # 창 끝 위치: 이 지점까지 목표 차로에 있어야 한다
    target_lane: int       # 창 끝 도로에서의 목표 차로
    length: float          # 창 길이(창 끝에서 거슬러)
    stop: str = ""         # S3: 창 끝까지 정류장 차로에 못 가면 건너뛸 정류장


def lap_route(net, loop):
    """한 바퀴 노선: R68에서 출발해 R42까지. bus_loop는 3.19.0부터 R68로 끝난다(그 전에는 R42로 끝나 R68을 잇는다)."""
    closed = close_loop(net, loop)
    if closed[-1] != START_EDGE or START_EDGE in closed[:-1]:
        raise ValueError(f"노선 끝 {loop[-1]}에서 처음 {loop[0]} 사이가 {START_EDGE} 하나가 아니다: {closed[len(loop):]}")
    return [START_EDGE, *closed[:-1]]


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
                          CHANGE_WINDOW_M + EXTRA_PER_LANE_M * max(0, top - s2["lane_idx"] - 1), s2["id"]))
    for k, (a, b) in enumerate(zip(route, route[1:] + route[:1])):
        ea, eb = net.getEdge(a), net.getEdge(b)
        conns = ea.getConnections(eb)
        node = ea.getToNode()
        if not conns or not all(c.getDirection() in ("l", "L") for c in conns) or not is_signalized(node):
            continue
        if any(c.getFromLane().getIndex() == 1 for c in conns):
            continue                                   # 가로변 차로(1)에서 바로 돌 수 있으면 차선 변경이 없다
        target = min(c.getFromLane().getIndex() for c in conns)
        end = ea.getLength() - no_change_m(node.getID())
        edge = a
        if end < 0:                                    # 창 끝이 앞 도로로 넘어가면 그 도로 기준(같은 차로 번호로 본다)
            edge, end = _back(net, route, k, ea.getLength(), no_change_m(node.getID()))
        plans.append(LanePlan("S2", edge, end, target, CHANGE_WINDOW_M + EXTRA_PER_LANE_M * max(0, target - 1)))
    return plans


class TestBusPilot:
    """tick(tc, t)를 매 스텝 부른다(runner의 bus_hook)."""
    __test__ = False                                   # pytest가 테스트 클래스로 모으지 않게

    SIGNAL_S, REQUEST_DURATION, TIMEOUT_S, COOL_S = 3.0, 5.0, 14.0, 1.0
    # 같은 창에서 이어지는 변경(J1 앞 3~4칸)은 식힘 없이 방향지시등을 켠 채 이 시간 뒤 요청한다.
    # 한 칸에 6초 남짓 걸리면 14 m/s에서 4칸에 350 m가 필요해 창(240 m)을 넘었다(짧은 실행에서 확인).
    NEXT_SIGNAL_S = 1.0
    # 비양보형 대응: 목표 차로 후행차가 버스보다 0.5 m/s 넘게 빠른 상태가 1초 이어지면 속도를 줄여 먼저 보낸다
    BLOCKER_DV, BLOCKER_S, PASS_MARGIN, BLOCKER_WAIT_S = 0.5, 1.0, 5.0, 25.0

    def __init__(self, tc, net, loop, stops, stop_s=8.0):
        self.tc, self.net = tc, net
        self.lap_edges = lap_route(net, loop)
        self.stops = {s["id"]: s for s in stops}
        self.stop_s = stop_s
        self.plans = lane_plans(net, self.lap_edges, stops)
        self.forbidden = forbidden_intervals(net, self.lap_edges)
        self._len = {}
        self.lap, self.lap_times, self.lap_t0 = 0, [], None
        self.missed, self.slowed, self.handed = [], [], []
        self.changes = {}
        self.why = ""
        self.phase, self.t_phase, self.plan, self.target = "idle", 0.0, None, None
        self._done = set()
        self._added = False
        self._last_road = None
        self._handover = None           # SUMO 차선 변경에 넘긴 창. 창 끝 도로를 지나면 1536으로 돌린다
        self._slow_t0 = None
        self._chain = None              # 방금 변경을 마친 창(같은 창의 다음 변경은 빠르게)
        self._signal_s = self.SIGNAL_S
        self._blocker, self._blocker_t0, self._fast_t0, self._blocker_seen = None, 0.0, None, 0.0

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
            if self._handover is not None and self._last_road == self._handover.edge:
                tc.vehicle.setLaneChangeMode(BUS, LC_MANUAL)
                self._handover = None
            self._last_road = road
        if road.startswith(":"):
            return
        self._extend()
        self._pilot(tc, t, road)

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
                if road == p.edge and idx not in self._lanes_reaching_target(tc, p):
                    self.missed.append((self.lap, p.kind, p.edge))
                    if p.stop:
                        self._skip_stop(tc, p.stop)
                        self._reset(tc, 0.0)
                    elif self._handover is None:
                        self._hand_over(tc, p)
                continue
            if d <= p.length:
                return k, p, d
        return None

    def _offset(self, tc, road, idx, p):
        if p.kind == "S3" or road == p.edge:
            return p.target_lane - idx
        lane = tc.vehicle.getLaneID(BUS)
        for entry in tc.vehicle.getBestLanes(BUS):
            if entry[0] == lane:
                return int(entry[3])
        return 0

    def _pilot(self, tc, t, road):
        idx, pos = tc.vehicle.getLaneIndex(BUS), tc.vehicle.getLanePosition(BUS)
        act = self._active(tc, road, idx, pos)
        if act is not None and self._handover is None and self._cannot_make_it(tc, act, idx):
            self.handed.append((self.lap, act[1].kind))
            self._hand_over(tc, act[1])
        forbidden = self._forbidden(road, pos) or self._handover is not None
        if act is not None:
            p = act[1]
            off = self._offset(tc, road, idx, p)
            why = p.kind
        else:
            p, off, why = None, self._emergency(tc, road, pos), "경로"
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
            self.phase, self.t_phase = "signal", t
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
            if idx != self.target and not forbidden and self._let_blocker_pass(tc, t, road, pos):
                return                                   # 비양보형을 먼저 보내는 중
            if idx == self.target and self.plan is not None and not forbidden:
                # 한 칸 옮겼다. 같은 창에서 더 옮겨야 하면 방향지시등을 켠 채 바로 다음 변경으로 간다
                self._release_speed(tc, t)
                self._chain, self.phase, self.t_phase = self.plan, "idle", t
                self.plan = None
            elif idx == self.target or t - self.t_phase > self.TIMEOUT_S or forbidden \
                    or self.target >= tc.edge.getLaneNumber(road):
                self._reset(tc, t)
            elif int(round((t - self.t_phase) * 10)) % 20 == 0:
                tc.vehicle.changeLane(BUS, self.target, self.REQUEST_DURATION)
        elif self.phase == "cool" and t - self.t_phase >= self.COOL_S:
            self.phase = "idle"

    def _cannot_make_it(self, tc, act, idx):
        """창 끝이 제동 거리 안인데 지금 차로가 창 끝에서 목표 차로에 닿지 못한다. 정류장에 서 있는 동안은 아니다."""
        _, p, d = act
        if tc.vehicle.getStopState(BUS) & 1:
            return False
        good = self._lanes_reaching_target(tc, p)
        v = tc.vehicle.getSpeed(BUS)
        return bool(good) and idx not in good and d <= v * v / (2.0 * HANDOVER_DECEL) + HANDOVER_MARGIN_M

    def _hand_over(self, tc, p):
        """남은 변경을 SUMO 기본 차선 변경(전략·협조, 1621)에 맡긴다. SUMO는 차로 끝까지 빈틈을 찾고, 이웃 차는 제 협조 성향만큼
        속도를 줄여 자리를 내준다. 버스 규칙은 창 끝 도로를 지날 때까지 차선 변경을 요청하지 않는다."""
        tc.vehicle.setLaneChangeMode(BUS, LC_SUMO)
        self._handover = p
        self._reset(tc, 0.0)

    def _skip_stop(self, tc, sid):
        """정류장 차로에 못 들어갔다. 달리는 차로에 서서 막지 않고 이번 바퀴의 그 정류장을 건너뛴다."""
        for i, s in enumerate(tc.vehicle.getStops(BUS)):
            if s.stoppingPlaceID == sid:
                tc.vehicle.replaceStop(BUS, i, "")
                return

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
            if p.kind == "S2" and j + 1 < len(route) and self.net.hasEdge(route[j + 1]):
                # 좌회전 직전 창: 목표 차로 하나만이 아니라 다음 도로로 이어지는 차로면 맞다(J1은 좌회전 차로가 2개)
                nxt = self.net.getEdge(route[j + 1])
                ok = any(c.getFromLane().getIndex() in lanes for c in self.net.getEdge(p.edge).getConnections(nxt))
            else:
                ok = p.target_lane in lanes
            if ok:
                good.add(k)
        return good

    def _let_blocker_pass(self, tc, t, road, pos):
        """목표 차로 후행차가 버스보다 `BLOCKER_DV` 넘게 빠른 상태가 `BLOCKER_S` 이어지면 비양보형으로 보고, 속도를 줄여
        먼저 보낸다. 그 차 뒤끝이 버스 앞끝보다 `PASS_MARGIN` 앞서면 속도를 풀고 다시 요청한다. 진행 중이면 True.
        버스는 상대의 운전자 성향을 읽지 않고 속도만 본다."""
        v = tc.vehicle.getSpeed(BUS)
        if self._blocker is not None:
            b = self._blocker
            gone = b not in tc.vehicle.getIDList()
            ahead = None if gone else self._route_ahead(tc, b, pos)
            if ahead is not None:
                self._blocker_seen = t
            passed = ahead is not None and ahead - tc.vehicle.getLength(b) > self.PASS_MARGIN
            lost = ahead is None and t - self._blocker_seen > 3.0      # 노선 밖(다른 길로 갔다)
            if gone or passed or lost or t - self._blocker_t0 > self.BLOCKER_WAIT_S:
                self._blocker = None
                self._release_speed(tc, t)
                tc.vehicle.changeLane(BUS, self.target, self.REQUEST_DURATION)
                self.t_phase = t                          # 다시 요청했으니 시간 초과를 새로 센다
                return False
            return True
        f = tc.vehicle.getLeftFollowers(BUS) if self.target > tc.vehicle.getLaneIndex(BUS) \
            else tc.vehicle.getRightFollowers(BUS)
        near = [(d, vid) for vid, d in f if vid and 0 <= d <= 50.0]
        if not near:
            self._fast_t0 = None
            return False
        vid = min(near)[1]
        if tc.vehicle.getSpeed(vid) > v + self.BLOCKER_DV:
            self._fast_t0 = t if self._fast_t0 is None else self._fast_t0
            if t - self._fast_t0 >= self.BLOCKER_S:
                self._blocker, self._blocker_t0, self._fast_t0, self._blocker_seen = vid, t, None, t
                if self._slow_t0 is None:
                    tc.vehicle.setSpeed(BUS, max(v - 3.0, 2.0))
                    self._slow_t0 = t
                return True
        else:
            self._fast_t0 = None
        return False

    def _route_ahead(self, tc, vid, pos):
        """버스 앞끝에서 vid 앞끝까지 버스 노선 거리(앞이면 양수). 노선 위에서 못 찾거나 교차로 안이면 None.
        도로 이름으로만 비교하면 짧은 다음 도로(R111b 27 m, R54b 38 m)로 넘어간 비양보형을 놓쳤다."""
        road = tc.vehicle.getRoadID(vid)
        if road.startswith(":"):
            return None
        route, i = tc.vehicle.getRoute(BUS), tc.vehicle.getRouteIndex(BUS)
        fpos = tc.vehicle.getLanePosition(vid)
        for j in range(max(0, i - 3), min(len(route), i + 4)):
            if route[j] != road:
                continue
            if j == i:
                return fpos - pos
            if j > i:
                return self._L(route[i]) - pos + sum(self._L(e) for e in route[i + 1:j]) + fpos
            return -(pos + sum(self._L(e) for e in route[j + 1:i]) + self._L(route[j]) - fpos)
        return None

    def _emergency(self, tc, road, pos):
        """창 밖 예외: 지금 차로가 다음 도로로 이어지지 않는데 도로 끝이 30 m 안이면 옮긴다."""
        lane = tc.vehicle.getLaneID(BUS)
        for entry in tc.vehicle.getBestLanes(BUS):
            if entry[0] == lane:
                if not entry[4] and self._L(road) - pos < 30.0:
                    return int(entry[3])
                return 0
        return 0

    def _release_speed(self, tc, t):
        if self._slow_t0 is not None:
            tc.vehicle.setSpeed(BUS, -1)
            self.slowed.append((self.lap, self.why, round(t - self._slow_t0, 1)))
            self._slow_t0 = None

    def _reset(self, tc, t):
        self._release_speed(tc, t)
        tc.vehicle.setSignals(BUS, 0)
        self.phase, self.t_phase, self.plan, self._chain = "cool", t, None, None
        self._blocker, self._fast_t0 = None, None
