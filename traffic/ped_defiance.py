"""A3 보강. 신호 없는 횡단보도에서 보행자가 버스의 우선권을 무시하고 건너는 행동(PedSUMO 방식).

기본 보행자(SUMO 기본 모델, 또는 vType P1/P2) 위에 얹는 추가 행동이다.
참고: M-Colley/pedsumo (HRI '24). 횡단보도 앞 보행자가 '건널까'를 확률로 정하고, 건너기로 하면
`junctionModel.ignoreTypes`를 사람 단위로 설정해 해당 차량을 무시하게 한다.

PedSUMO와 다르게 한 점
- 무시 대상과 개입 조건은 버스로 한정하지 않는다. 가로막는 차량(통과 시간 < 건너는 시간)이 버스든 배경 차량이든
  같은 확률로 판정하고, 건너기로 하면 모든 차량 유형을 무시한다. 무시해도 차량은 SUMO 기본 안전 규칙으로 보행자에게
  반응한다.
- 합정역 실행(`HapjeongTraffic.run`)에서는 기본으로 끈다. 보행자의 간격 수용은 보행자 vType 분포(pedestrian.ped_dist_xml)가
  맡는다. 이 모듈은 PedSUMO 방식을 따로 시험할 때 켠다.
- 판정 주기: PedSUMO는 1초 스텝마다 굴린다. 여기는 스텝이 0.1초라 `period`(기본 1 s)마다 굴려
  같은 빈도를 유지한다.
- 빠진 요소: eHMI, 성별·시력 계수. 근거 없이 독일 값을 옮기지 않으려고 뺐다.
- 값의 출처(중요): 기본 건너기 확률 0.4, 통과 시간·대기 시간·그룹·폭·차량 크기·걷는 중 계수 곡선, 나이 계수, 스마트폰
  주의 분산 확률은 모두 PedSUMO(독일 자료 기반)의 값을 옮긴 **초깃값**이다. 한국 현장 자료(합정역 횡단보도 관측)로
  바꿔야 하며, 바꾸기 전의 결과는 정성적 경향(통과 시간이 길수록 더 건넌다)만 믿는다. 값은 `DefianceCfg`에 모여 있다.
무단횡단(횡단보도 없음)은 PedSUMO에도 없다. 이 모듈은 횡단보도가 있는 곳 전부에 적용된다.
"""
import random
import re
from dataclasses import dataclass, field

CROSSING_LANE = re.compile(r"^:.+_c\d+_\d+$")


@dataclass
class DefianceCfg:
    base: float = 0.4               # 기본 건너기 확률(1 판정당). 아래 계수를 곱한다
    period: float = 1.0             # 판정 주기 [s]
    walk_speed: float = 1.0         # 건너는 시간 추정용 보행 속도 [m/s]
    # 통과 시간(TTC) 계수. 3 s 미만에서는 거의 안 건너고, 6 s 이상에서는 잘 건넌다
    ttc_extreme: float = 1.0
    ttc_lower: float = 3.0
    ttc_upper: float = 6.0
    f_extreme: float = 0.01
    f_under_lower: float = 0.1
    f_over_upper: float = 3.0
    f_at_lower: float = 0.2
    f_at_upper: float = 2.0
    # 서 있는 차량의 통과 시간 = 횡단보도까지 거리 / standing_speed, 단 standing_ttc 이상(곧 출발할 차로 본다).
    # 예전에는 서 있는 차에 무조건 10초를 줬다. 그러면 계수가 최대(×3)가 되어 차가 서는 순간 보행자가 가장 잘 건너고,
    # 다음 보행자도 같은 판정을 받아 차가 영원히 서 있게 된다(검증으로 확인).
    standing_speed: float = 3.0     # 정지한 차가 출발해 달릴 때의 대표 속도 [m/s]
    standing_ttc: float = 2.0       # 서 있는 차의 통과 시간 하한 [s]. 2초면 건너기 계수 ×0.1 구간이다
    # 차량 대기 계수: 횡단보도 앞 첫 차가 오래 기다렸으면 보행자가 먼저 보내 준다(보행자 대기 계수의 짝)
    veh_wait_free: float = 3.0      # 이 시간까지 기다린 차에는 영향이 없다 [s]
    veh_wait_stop: float = 12.0     # 이 시간 이상 기다린 차에는 계수가 최소가 된다 [s]
    f_veh_wait_min: float = 0.05
    # 대기 상한: 횡단보도 앞 첫 차가 이 시간 이상 기다렸으면 아직 횡단보도에 들어서지 않은 보행자는 무시하고 건너지 않는다
    # (이미 정한 사람의 결정도 취소). 이미 건너는 사람이 다 건너는 데 걸리는 시간(약 13초)을 더해도 차의 대기가 20초 미만이 되게 5초로 둔다.
    veh_wait_hard: float = 5.0
    wait_accepted: float = 28.0     # 이 시간 넘게 기다리면 점점 건너려 한다 [s]
    wait_slope: float = 0.0494
    group_2_3: float = 1.2
    group_4_up: float = 1.4
    neutral_width: float = 7.0      # 이 길이보다 좁으면 건너기 쉽다
    walking_momentum: float = 1.2   # 이미 걷고 있으면 건너려는 경향이 커진다
    small_area: float = 1.3 * 1.35
    neutral_area: float = 1.4 * 1.8
    large_area: float = 2.0 * 2.0
    f_small: float = 1.3
    f_neutral: float = 1.0
    f_large: float = 0.7
    distract_dfv: float = 1.5
    use: tuple = ("ttc", "wait", "vwait", "group", "width", "size", "momentum", "age", "distract")
    watch_s: float = 15.0           # 건너기로 정한 뒤 가장 가까운 차량의 급감속을 지표로 재는 시간 [s]


def age_factor(age):
    """PedSUMO의 나이 계수: 6세 0.6 -> 18~30세 1.2 -> 50세 1.0 -> 90세 이상 0.6."""
    pts = [(6, 0.6), (18, 1.2), (30, 1.2), (50, 1.0), (90, 0.6)]
    if age <= pts[0][0]:
        return pts[0][1]
    for (a0, v0), (a1, v1) in zip(pts, pts[1:]):
        if age <= a1:
            return v0 + (age - a0) * (v1 - v0) / (a1 - a0)
    return pts[-1][1]


def distract_chance(age):
    """스마트폰에 주의가 분산될 확률(PedSUMO): 8세 0.02 -> 16세 0.10 -> 50세 0.01."""
    if 8 <= age <= 16:
        return 0.02 + (age - 8) * (0.10 - 0.02) / 8
    if 16 < age <= 50:
        return 0.10 - (age - 16) * (0.10 - 0.01) / 34
    return 0.01


def ttc_factor(ttc, c):
    if ttc <= c.ttc_extreme:
        return c.f_extreme
    if ttc <= c.ttc_lower:
        return c.f_under_lower
    if ttc >= c.ttc_upper:
        return c.f_over_upper
    return c.f_at_lower + (ttc - c.ttc_lower) * (c.f_at_upper - c.f_at_lower) / (c.ttc_upper - c.ttc_lower)


def wait_factor(wait, c):
    return 1.0 if wait <= c.wait_accepted else 1.0 + (wait - c.wait_accepted) * c.wait_slope


def veh_wait_factor(wait, c):
    """횡단보도 앞 첫 차의 대기 시간 계수: veh_wait_free까지 1, veh_wait_stop 이상 f_veh_wait_min, 그 사이는 직선으로 줄인다."""
    if wait <= c.veh_wait_free:
        return 1.0
    if wait >= c.veh_wait_stop:
        return c.f_veh_wait_min
    return 1.0 + (wait - c.veh_wait_free) * (c.f_veh_wait_min - 1.0) / (c.veh_wait_stop - c.veh_wait_free)


def group_factor(n, c):
    return 1.0 if n <= 1 else (c.group_2_3 if n <= 3 else c.group_4_up)


def size_factor(area, c):
    if area <= c.small_area:
        return c.f_small
    if area >= c.large_area:
        return c.f_large
    if area < c.neutral_area:
        return c.f_neutral + (c.neutral_area - area) * (c.f_small - c.f_neutral) / (c.neutral_area - c.small_area)
    return c.f_neutral - (area - c.neutral_area) * (c.f_neutral - c.f_large) / (c.large_area - c.neutral_area)


@dataclass
class _Ped:
    age: int
    distracted: bool
    t_wait0: float
    crossing: str
    decided: bool = False


class PedDefianceManager:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(A1의 YieldManager.update와 같은 자리)."""

    def __init__(self, traci, bus_id, rng=None, cfg=None, crossings=None, age_range=(10, 80)):
        self.traci, self.bus = traci, bus_id
        self.rng = rng or random.Random(0)
        self.cfg = cfg or DefianceCfg()
        self.age_range = age_range
        self.incoming = self._build_incoming(crossings)   # 횡단보도 차로 -> 그 횡단보도로 오는 차로 집합
        self.peds = {}                          # 보행자 id -> _Ped (횡단보도 앞에 있는 동안)
        self.log = []                           # 판정마다 한 줄
        self.watch = {}                         # 보행자가 무시하고 건너기로 한 때의 선두 차량 id -> 관찰 끝 시각
        self.revoked = 0                        # 선두 차가 오래 기다려 취소한 건너기 결정 수
        self._last = -1e9

    # ---- 초기화: 신호 없는 횡단보도와 거기로 들어오는 차로 -------------------------------------
    def _build_incoming(self, crossings):
        tc = self.traci
        foes = {}
        for lane in tc.lane.getIDList():
            if CROSSING_LANE.match(lane) and (crossings is None or lane in crossings):
                foes[lane] = set(tc.lane.getInternalFoes(lane))
        inc = {}
        for lane in tc.lane.getIDList():
            for link in tc.lane.getLinks(lane):
                if link[5] == "M" and link[4]:        # 'M' = 우선권이 차량에 있는 교차로 연결
                    for cl, fs in foes.items():
                        if link[4] in fs:
                            inc.setdefault(cl, set()).add(lane)
        return inc

    # ---- 매 스텝 ---------------------------------------------------------------------------------
    def update(self, t):
        if t - self._last < self.cfg.period - 1e-9:
            return
        self._last = t
        tc = self.traci
        now = set(tc.person.getIDList())
        for pid in sorted(now):
            nxt = tc.person.getNextEdge(pid)
            lane = nxt + "_0" if nxt else ""
            if lane in self.incoming:
                st = self.peds.get(pid)
                if st is None:
                    st = self.peds[pid] = self._new_ped(t, lane)
                if st.decided:
                    self._maybe_revoke(pid, st)
                if not st.decided:
                    self._decide(pid, st, t)
            elif pid in self.peds:
                self._release(pid)
        for pid in set(self.peds) - now:
            del self.peds[pid]

    def _new_ped(self, t, lane):
        age = self.rng.randint(*self.age_range)
        return _Ped(age=age, distracted=self.rng.random() < distract_chance(age), t_wait0=t, crossing=lane)

    def _release(self, pid):
        st = self.peds.pop(pid)
        if st.decided:
            try:
                self.traci.person.setParameter(pid, "junctionModel.ignoreTypes", "")
            except Exception:
                pass

    # ---- 판정 ------------------------------------------------------------------------------------
    def _closest(self, lane):
        """차로에서 횡단보도에 가장 가까운 차량과 그 통과 시간."""
        tc = self.traci
        best, pos = None, -1.0
        for v in tc.lane.getLastStepVehicleIDs(lane):
            p = tc.vehicle.getLanePosition(v)
            if p > pos:
                best, pos = v, p
        if best is None:
            return None, None
        dist = tc.lane.getLength(lane) - pos
        sp = tc.vehicle.getSpeed(best)
        c = self.cfg
        return best, (dist / sp if sp > 0.1 else max(dist / c.standing_speed, c.standing_ttc))

    def _lead(self, crossing):
        """횡단보도로 들어오는 차로들 중 통과 시간이 가장 짧은 선두 차량과 그 통과 시간."""
        lowest, lead = 100.0, None
        for lane in sorted(self.incoming[crossing]):
            veh, ttc = self._closest(lane)
            if veh is not None and ttc < lowest:
                lowest, lead = ttc, veh
        return lead, lowest

    def _maybe_revoke(self, pid, st):
        """무시하고 건너기로 했지만 아직 횡단보도에 들어서지 않은 사람의 결정을, 선두 차가 오래 기다렸으면 취소한다."""
        tc = self.traci
        road = tc.person.getRoadID(pid)
        if road.startswith(":") and "_c" in road:
            return                       # 이미 건너는 중이다: 끝까지 건넌다
        lead, _ = self._lead(st.crossing)
        if lead is not None and tc.vehicle.getWaitingTime(lead) >= self.cfg.veh_wait_hard:
            try:
                tc.person.setParameter(pid, "junctionModel.ignoreTypes", "")
            except Exception:
                return
            st.decided = False
            self.revoked += 1

    def _decide(self, pid, st, t):
        c, tc = self.cfg, self.traci
        t_cross = tc.lane.getLength(st.crossing) / c.walk_speed
        lead, lowest = self._lead(st.crossing)
        if lead is None or lowest >= t_cross:
            return                       # 막는 차량이 없다: 기본 모델대로 건넌다
        if tc.vehicle.getWaitingTime(lead) >= c.veh_wait_hard:
            self.log.append({"t": t, "ped": pid, "crossing": st.crossing, "ttc": lowest, "p": 0.0, "cross": False,
                             "wait": t - st.t_wait0, "factors": {"hard_cap": 1}, "vehicle": lead})
            return                       # 차가 오래 기다렸다: 이번에는 차를 먼저 보낸다
        grp = sum(1 for q in self.peds.values() if q.crossing == st.crossing and not q.decided)
        f = {"ttc": ttc_factor(lowest, c),
             "wait": wait_factor(t - st.t_wait0, c),
             "vwait": veh_wait_factor(tc.vehicle.getWaitingTime(lead), c),
             "group": group_factor(grp, c),
             "width": c.neutral_width / max(tc.lane.getLength(st.crossing), 0.1),
             "size": size_factor(tc.vehicle.getWidth(lead) * tc.vehicle.getHeight(lead), c),
             "momentum": c.walking_momentum if tc.person.getSpeed(pid) > 0.6 else 1.0,
             "age": age_factor(st.age),
             "distract": c.distract_dfv if st.distracted else 1.0}
        p = c.base
        for k in c.use:
            p *= f[k]
        p = min(p, 1.0)
        cross = self.rng.random() <= p
        self.log.append({"t": t, "ped": pid, "crossing": st.crossing, "ttc": lowest, "p": p, "cross": cross,
                         "wait": t - st.t_wait0, "factors": f, "vehicle": lead})
        if cross:
            st.decided = True
            self.watch[lead] = t + c.watch_s
            tc.person.setParameter(pid, "junctionModel.ignoreTypes", " ".join(tc.vehicletype.getIDList()))

    def watched(self, t):
        """지금 급감속을 재는 차량 id 집합(보행자가 무시하고 건너기로 한 때의 선두 차량)."""
        for vid in [v for v, end in self.watch.items() if end < t]:
            del self.watch[vid]
        return set(self.watch)

    # ---- 요약 ------------------------------------------------------------------------------------
    def summary(self):
        n = len(self.log)
        crossed = sum(1 for r in self.log if r["cross"])
        return {"decisions": n, "cross_decisions": crossed, "revoked": self.revoked,
                "mean_ttc_at_cross": (sum(r["ttc"] for r in self.log if r["cross"]) / crossed) if crossed else None}
