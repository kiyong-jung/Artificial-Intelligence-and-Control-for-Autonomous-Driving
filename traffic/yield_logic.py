"""A1 (R1). 목표 차로 후행차의 속도 조절 로직.

버스가 지금 들어가려는 차로 하나에서 버스에 가장 가까운 후행차 한 대만 대상이다. 그 한 대에 대해 차량 투입 때 정해진
행동(`driver.action`)을 한다.
- 양보형(`yield`): 반응 지연 뒤 버스보다 `YIELD_DV` 느리게 달려 길을 터 준다(`setSpeed`).
- 비양보형(`block`): 반응 지연 뒤 버스보다 `BLOCK_DV` 빠르게(최저 `BLOCK_MIN_SPEED`) 달려 버스가 못 들어오게 막는다. 그 차의
  뒤끝이 버스 앞끝보다 `PASS_MARGIN` 앞서거나 `BLOCK_MAX_S`가 지나면 놓는다. 버스는 속도를 줄여 이 차를 먼저 보낸 뒤 옮긴다
  (traffic/test_bus.py).
상호작용 동안 그 차는 changeLane으로 현재 차로에 고정한다(기본 모델의 속도 이득 차선변경이 뒤차를 옆 차로로 보내는 것을 막는다).
연속 차선변경은 변경마다 따로 켜졌다 꺼진다. 상호작용이 끝나면 속도 지정(`setSpeed(-1)`)과 차로 고정을 해제한다.
진입 의사는 `Trigger`(구간 목록 + 방향)로 정한다. S2(좌회전 직전 구간, 좌측), S3(정류장 접근 구간, 우측) 모두 받는다.
"""
import random
from dataclasses import dataclass, field

from .common import AGGRESSION, LaneLock, Ownership

BLINKER_RIGHT = 1    # SUMO signal 비트
BLINKER_LEFT = 2

YIELD_DV = 2.0          # 양보형: 버스보다 이만큼 느리게 [m/s]
BLOCK_DV = 2.0          # 비양보형: 버스보다 이만큼 빠르게 [m/s]
BLOCK_MIN_SPEED = 4.0   # 비양보형 목표 속도의 바닥(버스가 서 있어도 지나가게) [m/s]
BLOCK_MAX_S = 20.0      # 비양보형이 막는 최대 시간 [s]
PASS_MARGIN = 5.0       # 비양보형 뒤끝이 버스 앞끝보다 이만큼 앞서면 지나간 것으로 본다 [m]
# A1은 목표 차로 후행차가 버스에 이만큼 다가왔을 때 시작한다(TraCI getLeft/RightFollowers 간격) [m].
# 50 m 안이면 바로 켜던 때는 비양보형이 먼 뒤에서 억지로 따라와 버스를 추월해 부자연스러웠다. 후행차는 50 m 밖에서 들어와
# 버스보다 빠르게 다가온다(scenario_injector).
A1_TRIGGER_GAP_M = 3.0


@dataclass
class DriverTypeCfg:
    p_yield: tuple          # 양보 성향 U(lo, hi) — 투입 시 1회 샘플. 공격성 값(1-p)의 원천이고 행동은 yields가 정한다
    delay: tuple            # 반응 지연 U(lo, hi) [s]
    new_tau: tuple          # openGap newTau U(lo, hi) [s]
    yields: bool = False    # True면 투입 때 '양보(yield)', False면 '막기(block)'로 확정된다
    lc_params: dict = field(default_factory=dict)   # laneChangeModel.<속성>


DEFAULT_TYPES = {
    # 양보형 차선변경 파라미터를 SUMO 기본값으로 두면(lcCooperative=1) SUMO가
    # 양보 결정과 무관하게 간격을 내줘 실측 양보율이 100%가 된다(검증으로 확인).
    # 양보 여부를 A1만 정하도록 양보형도 내장 협조를 끈다.
    "yielder": DriverTypeCfg(p_yield=(0.70, 0.95), yields=True, delay=(0.5, 1.5),
                             new_tau=(2.0, 3.0),
                             lc_params={"lcCooperative": -1, "lcCooperativeSpeed": 0}),
    "nonyielder": DriverTypeCfg(
        p_yield=(0.05, 0.30), delay=(0.5, 1.5), new_tau=(2.0, 3.0),
        # lcCooperativeHelpTime은 SUMO 1.27.1 LC2013에서 지원되지 않아 뺐다.
        lc_params={"lcCooperative": -1, "lcCooperativeSpeed": 0}),
}


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
class _Interaction:
    follower: str
    action: str               # "yield" 버스보다 느리게 | "block" 버스보다 빠르게
    delay: float
    t_start: float
    t_open: float = None      # 속도 지정을 처음 한 시각
    v0: float = 0.0           # 그때 후행차 속도(막는 차는 이보다 느려지지 않는다)
    acted: bool = False       # 행동을 마쳤는가(비양보형을 놓은 뒤 True)
    max_decel: float = 0.0    # 관측된 최대 감속 [m/s²] (양수)
    prev_speed: float = None
    lane0: int = None         # 상호작용 시작 때 차로
    side: int = 0

    @property
    def yields(self):
        return self.action == "yield"


class YieldManager:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다."""

    OWNER = "A1"

    def __init__(self, traci, bus_id, rng=None, types=None, stop_exit_pos=None,
                 exit_max_dist=30.0, advance_window=2.0, advance_min=0.5,
                 follower_range=A1_TRIGGER_GAP_M, dt=0.1, type_mix=None, mix_vtype_prefixes=None,
                 triggers=None, owner=None, lock_lane=True, aggression=None, vtype_aggr=None):
        self.traci = traci
        self.bus = bus_id
        self.rng = rng or random.Random(0)
        self.types = types or DEFAULT_TYPES
        # triggers: [Trigger]. 주지 않으면 stop_exit_pos(정류장 출구, 좌측) 하나로 만든다(예전 방식 그대로).
        # 둘 다 없으면 방향지시등만 본다(어느 쪽이든, 구간 조건 없음).
        if triggers is None and stop_exit_pos is not None:
            e, ep = stop_exit_pos
            triggers = [Trigger(+1, [Zone(e, ep - exit_max_dist, ep + exit_max_dist)])]
        self.triggers = triggers
        self.advance_window = advance_window
        self.advance_min = advance_min
        self.follower_range = follower_range
        self.dt = dt
        self.owner = owner or Ownership()
        self.lock = LaneLock(traci) if lock_lane else None
        self.aggression = aggression or AGGRESSION
        # vType 이름 -> 공격성 값. 양보 성격이 없는 차량(예: S5 끼어들기 vType)에도 공격성 값을 기록해 둔다.
        self.vtype_aggr = dict(vtype_aggr or {})
        # 주행 모델과 양보 성격을 독립으로 붙인다. type_mix = {유형: 가중치}를 주면,
        # vType 이름이 유형 이름으로 시작하지 않는 차량 중 mix_vtype_prefixes로 시작하는
        # 차량(예: 배경 차량 "car_")에 투입 시 유형을 뽑아 준다. 주행 모델(vType)은 그대로다.
        # 주지 않으면 기존처럼 이름이 유형으로 시작하는 차량만 대상이다.
        self.type_mix = dict(type_mix) if type_mix else None
        self.mix_vtype_prefixes = tuple(mix_vtype_prefixes or ())
        if self.type_mix:
            unknown = set(self.type_mix) - set(self.types)
            if unknown or not self.mix_vtype_prefixes:
                raise ValueError("type_mix의 유형은 types에 있어야 하고 mix_vtype_prefixes가 필요하다")
        self.inter = {}          # follower id -> _Interaction
        self.pos_hist = []       # (t, lanePos, edge)
        self.log = []            # 종료된 상호작용 기록

    # ---- 투입 시점 유형 확정 -------------------------------------------------
    def on_departed(self, ids):
        tc = self.traci
        for vid in ids:
            vt = tc.vehicle.getTypeID(vid)
            base = vt.split("@")[0]
            if base in self.vtype_aggr:
                tc.vehicle.setParameter(vid, "driver.aggressiveness", f"{self.vtype_aggr[base]:.4f}")
                continue
            key = next((k for k in self.types if vt.startswith(k)), None)
            if key is None and self.type_mix and vid != self.bus and vt.startswith(self.mix_vtype_prefixes):
                key = self.rng.choices(list(self.type_mix), weights=list(self.type_mix.values()))[0]
            if key is None:
                continue
            cfg = self.types[key]
            p = self.rng.uniform(*cfg.p_yield)
            d = self.rng.uniform(*cfg.delay)
            tau = self.rng.uniform(*cfg.new_tau)
            tc.vehicle.setParameter(vid, "driver.type", key)
            tc.vehicle.setParameter(vid, "driver.pYield", f"{p:.4f}")
            # 행동은 투입 때 확정한다: 양보형은 버스보다 느리게, 비양보형은 버스보다 빠르게(막기)
            action = "yield" if cfg.yields else "block"
            tc.vehicle.setParameter(vid, "driver.action", action)
            tc.vehicle.setParameter(vid, "driver.aggressiveness", f"{1.0 - p:.4f}")   # 시나리오 확률은 이 값에서 나온다
            tc.vehicle.setParameter(vid, "driver.delay", f"{d:.4f}")
            tc.vehicle.setParameter(vid, "driver.newTau", f"{tau:.4f}")
            for attr, val in cfg.lc_params.items():
                tc.vehicle.setParameter(vid, f"laneChangeModel.{attr}", str(val))

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
        # 직전 2 s 사이 도로가 바뀌었으면 앞으로 간 것이다(짧은 도로 R111b 27 m는 진입 2 s 안에 항상 이전 도로가 이력에 남아
        # 예전에는 여기서 거부되어 S2 A1이 켜지지 않았다). 같은 도로에서만 전진 거리를 본다.
        if old[2] == edge and pos - old[1] < self.advance_min and not self._waiting_at_line(edge, pos):
            return 0
        return side

    def _waiting_at_line(self, edge, pos, within=5.0):
        """정지선(도로 끝) 앞에서 신호를 기다리며 방향지시등을 켠 버스인가. 이때는 서 있어도 진입 의사로 본다
        (신호 대기 중 뒤차가 다가오는 S2 상황이 A1에서 빠지던 원인)."""
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

    def _pick_action(self, vid):
        """투입 때 확정된 행동(driver.action)을 읽는다. 상호작용마다 새로 뽑지 않는다."""
        return self.traci.vehicle.getParameter(vid, "driver.action")

    # ---- 매 스텝 ------------------------------------------------------------
    def update(self, t):
        tc = self.traci
        ids = set(tc.vehicle.getIDList())
        if self.bus not in ids:        # 버스가 아직 없거나 나갔다: 진행 중 상호작용을 끝내고 쉰다
            for vid in list(self.inter):
                self._finish(vid, t, vid in ids)
            return
        side = self._intent(t)
        if side:
            vid = self._target_follower(side)
            if vid is not None and vid in ids and vid not in self.inter:
                key = tc.vehicle.getParameter(vid, "driver.type")
                if key in self.types and self.owner.claim(vid, self.OWNER):
                    delay = float(tc.vehicle.getParameter(vid, "driver.delay"))
                    self.inter[vid] = _Interaction(vid, self._pick_action(vid), delay, t,
                                                   lane0=tc.vehicle.getLaneIndex(vid), side=side)
        for vid in list(self.inter):
            it = self.inter[vid]
            alive = vid in ids
            passed = False
            if alive:
                sp = tc.vehicle.getSpeed(vid)
                if it.prev_speed is not None:
                    it.max_decel = max(it.max_decel, (it.prev_speed - sp) / self.dt)
                it.prev_speed = sp
                passed = self._passed_by(vid, PASS_MARGIN) if it.action == "block" else self._passed(vid)
            if (not alive) or passed or side == 0:
                self._finish(vid, t, alive)
                continue
            if self.lock is not None:
                self.lock.hold(vid, t)          # 자발적 차선변경 방지(상호작용 내내)
            if t - it.t_start < it.delay or it.acted:
                continue
            v_bus = tc.vehicle.getSpeed(self.bus)
            if it.t_open is None:
                it.t_open, it.v0 = t, tc.vehicle.getSpeed(vid)
            if it.action == "yield":
                tc.vehicle.setSpeed(vid, max(0.0, v_bus - YIELD_DV))
            elif t - it.t_open > BLOCK_MAX_S:
                tc.vehicle.setSpeed(vid, -1)      # 오래 막았다: 놓는다
                it.acted = True
            else:
                # 막는 차는 감속하지 않는다: 버스가 막 출발해 느릴 때 '버스 + 2'로 맞추면 오던 속도보다 느려져 간격이 열렸다
                tc.vehicle.setSpeed(vid, max(v_bus + BLOCK_DV, BLOCK_MIN_SPEED, it.v0))

    def _passed(self, vid):
        tc = self.traci
        same = tc.vehicle.getRoadID(vid) == tc.vehicle.getRoadID(self.bus)
        return same and (tc.vehicle.getLanePosition(vid)
                         > tc.vehicle.getLanePosition(self.bus) + 1.0)

    def _passed_by(self, vid, margin):
        """후행차 뒤끝이 버스 앞끝보다 margin 넘게 앞섰는가(같은 도로)."""
        tc = self.traci
        if tc.vehicle.getRoadID(vid) != tc.vehicle.getRoadID(self.bus):
            return False
        rear = tc.vehicle.getLanePosition(vid) - tc.vehicle.getLength(vid)
        return rear > tc.vehicle.getLanePosition(self.bus) + margin

    def _finish(self, vid, t, alive):
        it = self.inter.pop(vid)
        if alive and it.t_open is not None:
            self.traci.vehicle.setSpeed(vid, -1)              # 속도 지정 해제
        if alive and self.lock is not None:
            self.lock.release(vid)
        self.owner.release(vid, self.OWNER)
        self.log.append({"follower": vid, "yield": it.yields, "action": it.action, "t_start": it.t_start,
                         "delay": it.delay, "t_open": it.t_open, "t_end": t,
                         "max_decel": it.max_decel, "side": it.side})
