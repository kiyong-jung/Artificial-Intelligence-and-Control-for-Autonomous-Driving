"""A1 (R1). 운전자 성향 분포와, 버스가 차로를 바꾸려 할 때 목표 차로 후행차가 한 일의 관찰.

배경 차량은 투입될 때 운전자 성향 하나(`a`, 0~1)를 받고, 그 값과 차량마다의 흔들림으로 SUMO 차선 변경 파라미터
(`lcCooperative`, `lcCooperativeSpeed`, `lcSpeedGain`, `lcAssertive`, `lcImpatience`)와 `speedFactor`를 연속 분포에서
뽑는다(`draw_driver`). 양보할지는 이 값들을 받은 SUMO 차선 변경 모델(SL2015)이 스스로 정한다.

`YieldManager`는 아무것도 조종하지 않는다. 버스가 방향지시등을 켜고 진입 의사(`Trigger` 구간 + 직전 2초 전진)를 보이면
들어가려는 차로에서 버스에 가장 가까운 후행차 한 대를 골라, 누가 먼저 그 차로의 앞자리를 차지했는지 기록한다.
- `bus_first`: 버스가 후행차 앞으로 들어갔다(후행차의 선행차가 버스가 됐다).
- `follower_first`: 후행차가 버스 옆을 지나 앞서 갔다.
- `ended`: 진입 의사가 끝났거나(창을 벗어남, 방향지시등 끔) `MAX_OBSERVE_S`가 지났다 / `gone`: 후행차가 사라졌다.
"""
import random
from dataclasses import dataclass, field

BLINKER_RIGHT = 1    # SUMO signal 비트
BLINKER_LEFT = 2

# 목표 차로 후행차로 보는 거리[m]. 도시 속도(14 m/s)에서 2초 간격이 28 m라 이보다 먼 차는 버스의 차선 변경과 상관이 없다.
FOLLOWER_RANGE_M = 30.0
MAX_OBSERVE_S = 30.0
LC_ATTRS = ("lcCooperative", "lcCooperativeSpeed", "lcSpeedGain", "lcAssertive", "lcImpatience")


@dataclass(frozen=True)
class DriverDist:
    """운전자 성향 분포. 값은 자리표시다(현장 자료로 바꿔야 한다).

    a ~ Beta(alpha, beta). 파라미터마다 a에 N(0, trait_sd) 흔들림을 더해 [0, 1]로 자른 값을 선형으로 옮긴다.
    공격적일수록(a가 클수록) 덜 협조하고, 속도 이득 차선 변경을 더 하고, 좁은 간격도 받아들이고, 조금 빨리 달린다.
    """
    alpha: float = 2.0
    beta: float = 2.0
    trait_sd: float = 0.15
    cooperative: tuple = (1.0, 0.2)     # a=0일 때, a=1일 때 (SUMO 범위 0~1, 기본 1)
    speed_gain: tuple = (0.5, 2.0)      # (0~, 기본 1)
    assertive: tuple = (0.8, 1.6)       # (양수, 기본 1. 필요 간격을 이 값으로 나눈다)
    impatience: tuple = (-0.5, 0.5)     # (-1~1, 기본 0. SL2015만)
    speed_factor: tuple = (0.92, 1.08)  # 원하는 속도 = 제한 속도 x speedFactor
    speed_factor_sd: float = 0.05
    speed_factor_clip: tuple = (0.8, 1.2)


def draw_driver(rng, dist=DriverDist()):
    """운전자 한 명의 파라미터 dict. 'aggressiveness'는 기록용이고 나머지는 SUMO에 그대로 넣는다."""
    a = rng.betavariate(dist.alpha, dist.beta)

    def trait(lo_hi):
        x = min(1.0, max(0.0, a + rng.gauss(0.0, dist.trait_sd)))
        return lo_hi[0] + (lo_hi[1] - lo_hi[0]) * x

    coop = trait(dist.cooperative)
    sf = trait(dist.speed_factor) + rng.gauss(0.0, dist.speed_factor_sd)
    return {"aggressiveness": a, "lcCooperative": coop, "lcCooperativeSpeed": coop,
            "lcSpeedGain": trait(dist.speed_gain), "lcAssertive": trait(dist.assertive),
            "lcImpatience": trait(dist.impatience),
            "speedFactor": min(dist.speed_factor_clip[1], max(dist.speed_factor_clip[0], sf))}


@dataclass
class Zone:
    """도로 위 구간. edge의 [start, end] m."""
    edge: str
    start: float
    end: float

    def contains(self, edge, pos):
        return edge == self.edge and self.start <= pos <= self.end


@dataclass
class Trigger:
    """진입 의사를 읽는 조건: 방향(+1 좌 = 왼쪽 방향지시등, -1 우) + 구간 목록.

    zones가 비어 있으면 구간 조건 없이 방향만 본다.
    S2: Trigger(+1, [좌회전 직전 구간]), S3: Trigger(-1, [정류장 접근 구간]), S1 출구: Trigger(+1, [Zone(출구 ± d)]).
    """
    side: int
    zones: list = field(default_factory=list)

    def matches(self, side, edge, pos):
        if side != self.side:
            return False
        return not self.zones or any(z.contains(edge, pos) for z in self.zones)


@dataclass
class _Observation:
    follower: str
    side: int
    t_start: float
    aggressiveness: float = None     # 후행차의 운전자 성향(배경 차량이 아니면 None)
    max_decel: float = 0.0           # 관찰 중 후행차의 최대 감속 [m/s²] (양수)
    prev_speed: float = None


class YieldManager:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다. on_departed(ids)는 투입된 차량마다 부른다."""

    def __init__(self, traci, bus_id, rng=None, triggers=None, stop_exit_pos=None, exit_max_dist=30.0,
                 advance_window=2.0, advance_min=0.5, follower_range=FOLLOWER_RANGE_M, dt=0.1,
                 driver_prefixes=("car_",), driver_dist=DriverDist()):
        self.traci = traci
        self.bus = bus_id
        self.rng = rng or random.Random(0)
        # triggers: [Trigger]. 주지 않으면 stop_exit_pos(정류장 출구, 좌측) 하나로 만든다.
        # 둘 다 없으면 방향지시등만 본다(어느 쪽이든, 구간 조건 없음).
        if triggers is None and stop_exit_pos is not None:
            e, ep = stop_exit_pos
            triggers = [Trigger(+1, [Zone(e, ep - exit_max_dist, ep + exit_max_dist)])]
        self.triggers = triggers
        self.advance_window = advance_window
        self.advance_min = advance_min
        self.follower_range = follower_range
        self.dt = dt
        self.driver_prefixes = tuple(driver_prefixes)    # 이 이름으로 시작하는 vType의 차량에 운전자 성향을 준다
        self.driver_dist = driver_dist
        self._lc2013_types = set()   # lcImpatience를 받지 않는 vType(SL2015 전용 값이다)
        self.inter = {}          # 후행차 id -> _Observation (관찰 중)
        self.pos_hist = []       # (t, lanePos, edge)
        self.log = []            # 끝난 관찰 기록

    # ---- 투입 시점 운전자 성향 ------------------------------------------------------------------
    def on_departed(self, ids):
        tc = self.traci
        for vid in ids:
            vtype = tc.vehicle.getTypeID(vid)
            if vid == self.bus or not vtype.startswith(self.driver_prefixes):
                continue
            d = draw_driver(self.rng, self.driver_dist)
            tc.vehicle.setParameter(vid, "driver.aggressiveness", f"{d['aggressiveness']:.4f}")
            for attr in LC_ATTRS:
                if attr == "lcImpatience" and vtype in self._lc2013_types:
                    continue
                try:
                    tc.vehicle.setParameter(vid, f"laneChangeModel.{attr}", f"{d[attr]:.4f}")
                except Exception:
                    if attr != "lcImpatience":
                        raise
                    self._lc2013_types.add(vtype)          # 합성 도로의 LC2013 차량
            tc.vehicle.setSpeedFactor(vid, d["speedFactor"])

    # ---- 진입 의사 (방향지시등 + 구간 + 직전 2 s 전진) -----------------------------
    def _intent(self, t):
        tc = self.traci
        sig = tc.vehicle.getSignals(self.bus)
        side = 1 if sig & BLINKER_LEFT else (-1 if sig & BLINKER_RIGHT else 0)
        pos = tc.vehicle.getLanePosition(self.bus)
        edge = tc.vehicle.getRoadID(self.bus)
        self.pos_hist.append((t, pos, edge))
        self.pos_hist = [h for h in self.pos_hist if t - h[0] <= self.advance_window + 1e-6]
        if side == 0:
            return 0
        if self.triggers is not None and not any(tr.matches(side, edge, pos) for tr in self.triggers):
            return 0
        old = self.pos_hist[0]
        # 직전 2 s 사이 도로가 바뀌었으면 앞으로 간 것이다(짧은 도로 R111b 27 m는 진입 2 s 안에 항상 이전 도로가 이력에 남는다).
        # 같은 도로에서만 전진 거리를 본다.
        if old[2] == edge and pos - old[1] < self.advance_min and not self._waiting_at_line(edge, pos):
            return 0
        return side

    def _waiting_at_line(self, edge, pos, within=5.0):
        """정지선(도로 끝) 앞에서 신호를 기다리며 방향지시등을 켠 버스인가. 이때는 서 있어도 진입 의사로 본다."""
        try:
            lane = self.traci.vehicle.getLaneID(self.bus)
            return self.traci.lane.getLength(lane) - pos <= within
        except Exception:
            return False

    def _target_follower(self, side):
        """들어가려는 차로 하나에서 버스에 가장 가까운 후행차 한 대. 없으면 None."""
        tc = self.traci
        f = (tc.vehicle.getLeftFollowers(self.bus) if side > 0
             else tc.vehicle.getRightFollowers(self.bus))
        cand = [(d, i) for i, d in f if i and 0 <= d <= self.follower_range]
        return min(cand)[1] if cand else None

    def _aggressiveness(self, vid):
        a = self.traci.vehicle.getParameter(vid, "driver.aggressiveness")
        return float(a) if a else None

    # ---- 매 스텝 ------------------------------------------------------------
    def update(self, t):
        tc = self.traci
        ids = set(tc.vehicle.getIDList())
        if self.bus not in ids:        # 버스가 아직 없거나 나갔다: 진행 중 관찰을 끝내고 쉰다
            for vid in list(self.inter):
                self._finish(vid, t, "ended")
            return
        side = self._intent(t)
        if side:
            vid = self._target_follower(side)
            if vid is not None and vid in ids and vid not in self.inter and not self.inter:
                self.inter[vid] = _Observation(vid, side, t, self._aggressiveness(vid))
        for vid in list(self.inter):
            ob = self.inter[vid]
            if vid not in ids:
                self._finish(vid, t, "gone")
                continue
            sp = tc.vehicle.getSpeed(vid)
            if ob.prev_speed is not None:
                ob.max_decel = max(ob.max_decel, (ob.prev_speed - sp) / self.dt)
            ob.prev_speed = sp
            outcome = self._outcome(ob)
            if outcome is None and (side != ob.side or t - ob.t_start > MAX_OBSERVE_S):
                outcome = "ended"
            if outcome is not None:
                self._finish(vid, t, outcome)

    def _outcome(self, ob):
        """후행차와 버스 중 누가 그 차로의 앞자리를 차지했는가. 아직이면 None."""
        tc = self.traci
        leader = tc.vehicle.getLeader(ob.follower, 2.0 * self.follower_range)
        if leader and leader[0] == self.bus:
            return "bus_first"
        leaders = tc.vehicle.getLeftLeaders(self.bus) if ob.side > 0 else tc.vehicle.getRightLeaders(self.bus)
        if any(v == ob.follower for v, _ in leaders):
            return "follower_first"
        return None

    def _finish(self, vid, t, outcome):
        ob = self.inter.pop(vid)
        self.log.append({"follower": vid, "outcome": outcome, "side": ob.side, "t_start": ob.t_start, "t_end": t,
                         "aggressiveness": ob.aggressiveness, "max_decel": ob.max_decel})
