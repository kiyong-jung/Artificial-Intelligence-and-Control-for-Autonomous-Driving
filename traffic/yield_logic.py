"""A1. 배경 운전자 양보 로직 (양보형·비양보형).

양보 여부는 상호작용 1회마다 한 번만 정하고 끝날 때까지 바꾸지 않는다.
양보 동작은 TraCI openGap(기준 차량 = 버스)으로 구현한다.
"""
import random
from dataclasses import dataclass, field

BLINKER_RIGHT = 1    # SUMO signal 비트
BLINKER_LEFT = 2


@dataclass
class DriverTypeCfg:
    p_yield: tuple          # 양보 확률 U(lo, hi) — 차량마다 투입 시 1회 샘플
    delay: tuple            # 반응 지연 U(lo, hi) [s]
    new_tau: tuple          # openGap newTau U(lo, hi) [s]
    max_decel: float = 2.0
    duration: float = 12.0   # openGap 지속 시간 [s]
    change_rate: float = 1.0
    new_space: float = 0.0
    p_close_gap: float = 0.0     # 비양보형이 짧게 가속해 간격을 좁힐 확률
    lc_params: dict = field(default_factory=dict)   # laneChangeModel.<속성>


DEFAULT_TYPES = {
    # 계획은 양보형 차선변경 파라미터를 기본값으로 뒀지만, 기본 lcCooperative=1이면 SUMO가
    # 양보 결정과 무관하게 간격을 내줘 실측 양보율이 100%가 된다(검증으로 확인).
    # 양보 여부를 A1만 정하도록 양보형도 내장 협조를 끈다.
    "yielder": DriverTypeCfg(p_yield=(0.70, 0.95), delay=(0.5, 1.5),
                             new_tau=(2.0, 3.0),
                             lc_params={"lcCooperative": -1, "lcCooperativeSpeed": 0}),
    "nonyielder": DriverTypeCfg(
        p_yield=(0.05, 0.30), delay=(0.5, 1.5), new_tau=(2.0, 3.0),
        # lcCooperativeHelpTime은 SUMO 1.27.1 LC2013에서 지원되지 않아 뺐다(계획 보정).
        lc_params={"lcCooperative": -1, "lcCooperativeSpeed": 0}),
}


@dataclass
class _Interaction:
    follower: str
    yields: bool
    delay: float
    t_start: float
    t_open: float = None      # openGap 실제 발행 시각
    closed_gap: bool = False
    max_decel: float = 0.0    # 관측된 최대 감속 [m/s²] (양수)
    prev_speed: float = None


class YieldManager:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다."""

    def __init__(self, traci, bus_id, rng=None, types=None, stop_exit_pos=None,
                 exit_max_dist=30.0, advance_window=2.0, advance_min=0.5,
                 follower_range=50.0, dt=0.1):
        self.traci = traci
        self.bus = bus_id
        self.rng = rng or random.Random(0)
        self.types = types or DEFAULT_TYPES
        self.stop_exit_pos = stop_exit_pos       # (edge, pos) 또는 None = 거리 조건 생략
        self.exit_max_dist = exit_max_dist
        self.advance_window = advance_window
        self.advance_min = advance_min
        self.follower_range = follower_range
        self.dt = dt
        self.inter = {}          # follower id -> _Interaction
        self.pos_hist = []       # (t, lanePos, edge)
        self.log = []            # 종료된 상호작용 기록

    # ---- 투입 시점 유형 확정 -------------------------------------------------
    def on_departed(self, ids):
        tc = self.traci
        for vid in ids:
            vt = tc.vehicle.getTypeID(vid)
            key = next((k for k in self.types if vt.startswith(k)), None)
            if key is None:
                continue
            cfg = self.types[key]
            p = self.rng.uniform(*cfg.p_yield)
            d = self.rng.uniform(*cfg.delay)
            tau = self.rng.uniform(*cfg.new_tau)
            tc.vehicle.setParameter(vid, "driver.type", key)
            tc.vehicle.setParameter(vid, "driver.pYield", f"{p:.4f}")
            tc.vehicle.setParameter(vid, "driver.delay", f"{d:.4f}")
            tc.vehicle.setParameter(vid, "driver.newTau", f"{tau:.4f}")
            for attr, val in cfg.lc_params.items():
                tc.vehicle.setParameter(vid, f"laneChangeModel.{attr}", str(val))

    # ---- 진입 의사 (방향지시등 + 출구 거리 + 직전 2 s 전진) ---------------------
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
        if self.stop_exit_pos is not None:
            e, ep = self.stop_exit_pos
            if edge != e or abs(pos - ep) > self.exit_max_dist:
                return 0
        old = self.pos_hist[0]
        if old[2] != edge or pos - old[1] < self.advance_min:
            return 0
        return side

    def _followers(self, side):
        tc = self.traci
        f = (tc.vehicle.getLeftFollowers(self.bus) if side > 0
             else tc.vehicle.getRightFollowers(self.bus))
        return [(i, d) for i, d in f if i and 0 <= d <= self.follower_range]

    # ---- 매 스텝 ------------------------------------------------------------
    def update(self, t):
        tc = self.traci
        side = self._intent(t)
        ids = set(tc.vehicle.getIDList())
        if side:
            for vid, _ in self._followers(side):
                if vid in self.inter or vid not in ids:
                    continue
                key = tc.vehicle.getParameter(vid, "driver.type")
                if key not in self.types:
                    continue
                p = float(tc.vehicle.getParameter(vid, "driver.pYield"))
                delay = float(tc.vehicle.getParameter(vid, "driver.delay"))
                self.inter[vid] = _Interaction(vid, self.rng.random() < p, delay, t)
        for vid in list(self.inter):
            it = self.inter[vid]
            alive = vid in ids
            passed = False
            if alive:
                sp = tc.vehicle.getSpeed(vid)
                if it.prev_speed is not None:
                    it.max_decel = max(it.max_decel, (it.prev_speed - sp) / self.dt)
                it.prev_speed = sp
                passed = self._passed(vid)
            if (not alive) or passed or side == 0:
                self._finish(vid, t, alive)
                continue
            if t - it.t_start < it.delay or it.t_open is not None or it.closed_gap:
                continue
            cfg = self.types[tc.vehicle.getParameter(vid, "driver.type")]
            if it.yields:
                tau = float(tc.vehicle.getParameter(vid, "driver.newTau"))
                tc.vehicle.openGap(vid, tau, cfg.new_space, cfg.duration,
                                   cfg.change_rate, cfg.max_decel, self.bus)
                it.t_open = t
            elif cfg.p_close_gap and self.rng.random() < cfg.p_close_gap:
                tc.vehicle.setAcceleration(vid, 1.5, 1.0)
                it.closed_gap = True
            else:
                it.closed_gap = True      # 비양보: 명령 없음, 결정은 유지

    def _passed(self, vid):
        tc = self.traci
        same = tc.vehicle.getRoadID(vid) == tc.vehicle.getRoadID(self.bus)
        return same and (tc.vehicle.getLanePosition(vid)
                         > tc.vehicle.getLanePosition(self.bus) + 1.0)

    def _finish(self, vid, t, alive):
        it = self.inter.pop(vid)
        if alive and it.t_open is not None:
            self.traci.vehicle.openGap(vid, -1, -1, -1, -1)   # 해제
        self.log.append({"follower": vid, "yield": it.yields, "t_start": it.t_start,
                         "delay": it.delay, "t_open": it.t_open, "t_end": t,
                         "max_decel": it.max_decel})
