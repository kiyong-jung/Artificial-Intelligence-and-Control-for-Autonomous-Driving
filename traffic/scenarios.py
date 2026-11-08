"""합성 검증 시나리오와 판단 없는 시험용 버스(고정 시나리오 차량).

고정 시나리오 차량: 정차 -> 방향지시등 -> 일정 가속 전진 -> 정해진 시각에 차선변경 요청.
이 차량은 주변 교통 검증 도구이며 버스 기준선이 아니다.
"""
import os
import subprocess

from .sumo_conn import traci, sumo_cmd, checkBinary
from .yield_logic import YieldManager, DEFAULT_TYPES, BLINKER_LEFT

NET_DIR = os.path.join(os.path.dirname(__file__), "_build")
SUMO_BIN_NETCONVERT = checkBinary("netconvert")


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


def build_a1_net():
    """2차로 직선 2 km. 속도 제한 13.9 m/s."""
    d = os.path.join(NET_DIR, "a1")
    _write(os.path.join(d, "n.nod.xml"),
           '<nodes><node id="A" x="0" y="0"/><node id="B" x="2000" y="0"/></nodes>')
    _write(os.path.join(d, "e.edg.xml"),
           '<edges><edge id="E0" from="A" to="B" numLanes="2" speed="13.9"/></edges>')
    subprocess.run([SUMO_BIN_NETCONVERT, "-n", os.path.join(d, "n.nod.xml"),
                    "-e", os.path.join(d, "e.edg.xml"), "-o", os.path.join(d, "net.net.xml"),
                    "--no-warnings"], check=True)
    _write(os.path.join(d, "r.rou.xml"), """<routes>
<vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" tau="1.0" sigma="0"/>
<vType id="yielder" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" speedFactor="1" speedDev="0" actionStepLength="0.5" lcCooperative="1"/>
<vType id="nonyielder" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" speedFactor="1" speedDev="0" actionStepLength="0.5"/>
<route id="r0" edges="E0"/>
</routes>""")
    _write(os.path.join(d, "a1.sumocfg"), """<configuration>
<input><net-file value="net.net.xml"/><route-files value="r.rou.xml"/></input>
<time><begin value="0"/><end value="60"/></time>
</configuration>""")
    return os.path.join(d, "a1.sumocfg")


class ScriptedBus:
    """판단 없는 재생용 버스. 단계별 시각은 모두 고정값이다."""

    def __init__(self, vid, x0=100.0, t_signal=3.0, t_go=3.5, accel=1.5, v_max=9.0,
                 t_request=9.5, req_duration=15.0, target_lane=1):
        self.vid, self.x0 = vid, x0
        self.t_signal, self.t_go, self.accel, self.v_max = t_signal, t_go, accel, v_max
        self.t_request, self.req_duration, self.target_lane = t_request, req_duration, target_lane
        self.requested = False

    def add(self, tc):
        tc.vehicle.add(self.vid, "r0", typeID="bus_test", departPos=str(self.x0),
                       departLane="0", departSpeed="0")
        tc.vehicle.setSpeedMode(self.vid, 31)
        tc.vehicle.setSpeed(self.vid, 0.0)

    def act(self, tc, t):
        if t >= self.t_signal:
            tc.vehicle.setSignals(self.vid, BLINKER_LEFT)
        if t < self.t_go:
            tc.vehicle.setSpeed(self.vid, 0.0)
        else:
            tc.vehicle.setSpeed(self.vid, min(self.v_max, self.accel * (t - self.t_go)))
        if t >= self.t_request and not self.requested:
            tc.vehicle.changeLane(self.vid, self.target_lane, self.req_duration)
            self.requested = True


def run_a1_episode(cfg, driver_type, seed, bus_lc_mode=None, t_end=30.0,
                   follower_x0=21.5, p_override=None, record=False, bus_kwargs=None):
    """한 번의 상호작용을 돌려 결과 dict를 돌려준다.

    p_override: 양보 확률을 고정(예: 0)해 누출 검사에 쓴다.
    """
    import random
    tc = traci
    cmd = sumo_cmd(cfg, seed=seed)
    if tc.isLoaded():
        tc.load(cmd[1:])
    else:
        tc.start(cmd)
    rng = random.Random(seed)
    bus = ScriptedBus("bus0", **(bus_kwargs or {}))
    bus.add(tc)
    types = DEFAULT_TYPES
    if p_override is not None:
        import copy
        types = copy.deepcopy(DEFAULT_TYPES)
        types[driver_type].p_yield = (p_override, p_override)
    mgr = YieldManager(tc, "bus0", rng=rng, types=types, stop_exit_pos=("E0", 100.0),
                       exit_max_dist=60.0)
    tc.vehicle.add("f0", "r0", typeID=driver_type, departPos=str(follower_x0),
                   departLane="1", departSpeed="9")
    if bus_lc_mode is not None:
        tc.vehicle.setLaneChangeMode("bus0", bus_lc_mode)
    series = []
    changed_at = None
    t = 0.0
    step = 0
    while t < t_end:
        mgr.on_departed(tc.simulation.getDepartedIDList())
        mgr.update(t)
        bus.act(tc, t)
        tc.simulationStep()
        step += 1
        t = round(step * 0.1, 6)
        if "f0" not in tc.vehicle.getIDList():
            break
        if changed_at is None and tc.vehicle.getLaneIndex("bus0") == 1:
            changed_at = t
        if record:
            series.append({"t": t, "f_speed": tc.vehicle.getSpeed("f0"),
                           "bus_pos": tc.vehicle.getLanePosition("bus0"),
                           "f_pos": tc.vehicle.getLanePosition("f0"),
                           "gap": tc.vehicle.getLanePosition("bus0") - 12.0
                                  - tc.vehicle.getLanePosition("f0")})
    inter = mgr.log[0] if mgr.log else None
    if inter is None and mgr.inter:
        inter = {"yield": next(iter(mgr.inter.values())).yields,
                 "max_decel": next(iter(mgr.inter.values())).max_decel,
                 "t_open": next(iter(mgr.inter.values())).t_open}
    out = {"type": driver_type, "seed": seed,
           "p_yield": float(tc.vehicle.getParameter("f0", "driver.pYield")) if "f0" in tc.vehicle.getIDList() else None,
           "new_tau": float(tc.vehicle.getParameter("f0", "driver.newTau")) if "f0" in tc.vehicle.getIDList() else None,
           "decision": None if inter is None else inter["yield"],
           "t_open": None if inter is None else inter["t_open"],
           "max_decel": None if inter is None else inter["max_decel"],
           "changed": changed_at is not None, "changed_at": changed_at}
    if record:
        out["series"] = series
    return out
