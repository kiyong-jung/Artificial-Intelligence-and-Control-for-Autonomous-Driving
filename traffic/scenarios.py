"""합성 검증 시나리오와 판단 없는 시험용 버스(고정 시나리오 차량).

고정 시나리오 차량: 정차 -> 방향지시등 -> 일정 가속 전진 -> 정해진 시각에 차선변경 요청.
이 차량은 주변 교통 검증 도구이며 버스 기준선이 아니다.
"""
import os
import subprocess

from .sumo_conn import traci, sumo_cmd, checkBinary
from .yield_logic import YieldManager, BLINKER_LEFT

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
                 t_request=9.5, req_duration=15.0, target_lane=1, bus_lane=0, route="r0"):
        self.vid, self.x0 = vid, x0
        self.t_signal, self.t_go, self.accel, self.v_max = t_signal, t_go, accel, v_max
        self.t_request, self.req_duration, self.target_lane = t_request, req_duration, target_lane
        self.bus_lane = bus_lane
        self.route = route
        self.requested = False

    def add(self, tc):
        tc.vehicle.add(self.vid, self.route, typeID="bus_test", departPos=str(self.x0),
                       departLane=str(self.bus_lane), departSpeed="0")
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


def _param_float(tc, vid, key):
    """차량이 없거나 파라미터가 비어 있으면(운전자 성향을 못 받은 차량) None."""
    if vid not in tc.vehicle.getIDList():
        return None
    v = tc.vehicle.getParameter(vid, key)
    return float(v) if v != "" else None


def run_a1_episode(cfg, vtype, seed, bus_lc_mode=None, t_end=30.0,
                   follower_x0=21.5, record=False, bus_kwargs=None, lanes=(0, 1), edge="E0", warmup=0.0, mgr_kwargs=None):
    """버스가 옆 차로로 옮기려 할 때 그 차로 후행차(vType vtype, 운전자 성향은 YieldManager가 뽑는다)가 한 일을 본다.

    아무것도 조종하지 않는다. mgr_kwargs: YieldManager에 더 넘길 인자(예: follower_range).
    """
    import random
    tc = traci
    cmd = sumo_cmd(cfg, seed=seed)
    if tc.isLoaded():
        tc.load(cmd[1:])
    else:
        tc.start(cmd)
    for _ in range(int(round(warmup / 0.1))):      # 배경 교통 예열
        tc.simulationStep()
    rng = random.Random(seed)
    bus = ScriptedBus("bus0", **{"bus_lane": lanes[0], "target_lane": lanes[1], **(bus_kwargs or {})})
    bus.add(tc)
    mgr = YieldManager(tc, "bus0", rng=rng, stop_exit_pos=(edge, 100.0), exit_max_dist=60.0,
                       driver_prefixes=(vtype,), **{"follower_range": 50.0, **(mgr_kwargs or {})})
    tc.vehicle.add("f0", "r0", typeID=vtype, departPos=str(follower_x0),
                   departLane=str(lanes[1]), departSpeed="9")
    if bus_lc_mode is not None:
        tc.vehicle.setLaneChangeMode("bus0", bus_lc_mode)
    series = []
    changed_at = None
    f_ahead_at_change = None       # 버스가 옮긴 순간 후행차가 버스 앞에 있었나(비양보형이 먼저 지나간 경우)
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
        if changed_at is None and tc.vehicle.getLaneIndex("bus0") == lanes[1]:
            changed_at = t
            f_ahead_at_change = tc.vehicle.getLanePosition("f0") > tc.vehicle.getLanePosition("bus0")
        if record:
            series.append({"t": t, "f_speed": tc.vehicle.getSpeed("f0"),
                           "bus_pos": tc.vehicle.getLanePosition("bus0"),
                           "f_pos": tc.vehicle.getLanePosition("f0"),
                           "gap": tc.vehicle.getLanePosition("bus0") - 12.0
                                  - tc.vehicle.getLanePosition("f0")})
    ob = mgr.log[0] if mgr.log else None
    out = {"type": vtype, "seed": seed,
           "outcome": None if ob is None else ob["outcome"],
           "observing": bool(mgr.inter),
           "n_observations": len(mgr.log) + len(mgr.inter),
           "aggressiveness": _param_float(tc, "f0", "driver.aggressiveness"),
           "lcCooperative": _param_float(tc, "f0", "laneChangeModel.lcCooperative"),
           "max_decel": None if ob is None else ob["max_decel"],
           "changed": changed_at is not None, "changed_at": changed_at, "f_ahead_at_change": f_ahead_at_change}
    if record:
        out["series"] = series
    return out


def build_a1_mixed_net():
    """배경 차량처럼 이름이 car_로 시작하고 추종 모델이 제각각(Krauss/IDM/EIDM/ACC)인 후행차를 둔 A1 도로."""
    import shutil
    src = os.path.dirname(build_a1_net())
    dst = os.path.join(NET_DIR, "a1_mixed")
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(src, dst)
    rou = os.path.join(dst, "r.rou.xml")
    text = open(rou, encoding="utf-8").read()
    base = ('length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" '
            'speedFactor="1" speedDev="0" actionStepLength="0.5"')
    types = "".join(f'<vType id="car_{k}" {base} carFollowModel="{m}"/>\n'
                    for k, m in (("krauss", "Krauss"), ("idm", "IDM"), ("eidm", "EIDM"), ("acc", "ACC")))
    _write(rou, text.replace("<route id", types + "<route id", 1))
    return os.path.join(dst, "a1.sumocfg")
