"""S1 검증용 합성 도로: 2차로 직선, 1차로(0번)의 차로 정차형 정류장에 버스가 선다.

버스가 서 있는 동안 바로 뒤에 후행차 한 대가 접근한다. 후행차는 car_* 배경 차량이고(주행 모델 4종),
운전자 성향은 YieldManager.on_departed가 붙인다. 기다릴지 추월할지는 SUMO 차선 변경 모델이 정한다.
"""
import os
import random
import subprocess

from .scenarios import NET_DIR, _write
from .sumo_conn import traci, sumo_cmd, checkBinary
from .stop_bypass import StopBypassManager
from .yield_logic import YieldManager

STOP_POS = (300.0, 320.0)
MODELS = (("krauss", "Krauss"), ("idm", "IDM"), ("eidm", "EIDM"), ("acc", "ACC"))


def build_s1_net(stop_s=40, n_lanes=2):
    d = os.path.join(NET_DIR, "s1")
    _write(os.path.join(d, "n.nod.xml"),
           '<nodes><node id="A" x="0" y="0"/><node id="B" x="900" y="0"/></nodes>')
    _write(os.path.join(d, "e.edg.xml"),
           f'<edges><edge id="E0" from="A" to="B" numLanes="{n_lanes}" speed="13.9"/></edges>')
    subprocess.run([checkBinary("netconvert"), "-n", os.path.join(d, "n.nod.xml"),
                    "-e", os.path.join(d, "e.edg.xml"), "-o", os.path.join(d, "net.net.xml"),
                    "--no-warnings"], check=True, capture_output=True)
    _write(os.path.join(d, "stop.add.xml"),
           f'<additional><busStop id="bs0" lane="E0_0" startPos="{STOP_POS[0]}" endPos="{STOP_POS[1]}"/></additional>')
    base = ('length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" speedFactor="1" speedDev="0" '
            'actionStepLength="0.5"')
    types = "".join(f'<vType id="car_{k}" {base} carFollowModel="{m}"/>\n' for k, m in MODELS)
    _write(os.path.join(d, "r.rou.xml"), f"""<routes>
<vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" tau="1.0" sigma="0"/>
{types}<route id="r0" edges="E0"/>
</routes>""")
    _write(os.path.join(d, "s1.sumocfg"), """<configuration>
<input><net-file value="net.net.xml"/><route-files value="r.rou.xml"/><additional-files value="stop.add.xml"/></input>
<time><begin value="0"/><end value="200"/></time>
</configuration>""")
    return os.path.join(d, "s1.sumocfg")


def run_s1_episode(cfg, model, seed, *, stop_s=40.0, t_end=90.0, mgr_kwargs=None):
    """버스가 정류장에 서고(stop_s초), 바로 뒤에 후행차가 접근한다. 반환: 결과 dict."""
    tc = traci
    cmd = sumo_cmd(cfg, seed=seed)
    tc.load(cmd[1:]) if tc.isLoaded() else tc.start(cmd)
    ymgr = YieldManager(tc, "bus0", rng=random.Random(seed + 1), **(mgr_kwargs or {}))
    smgr = StopBypassManager(tc, "bus0")
    tc.vehicle.add("bus0", "r0", typeID="bus_test", departLane="0", departPos="230", departSpeed="8")
    tc.vehicle.setBusStop("bus0", "bs0", duration=stop_s)
    added = False
    lanes_seen, t, step, passed_stop = set(), 0.0, 0, None
    while t < t_end:
        ymgr.on_departed(tc.simulation.getDepartedIDList())
        if not added:
            # 후행차는 처음부터 상류에서 달려온다(투입 순간에 이미 차선변경을 끝내 버리는 일을 피한다)
            tc.vehicle.add("f0", "r0", typeID=f"car_{model}", departLane="0", departPos="0", departSpeed="9")
            tc.vehicle.setMaxSpeed("f0", 9.0)
            ymgr.on_departed(["f0"])
            added = True
        ymgr.update(t)
        smgr.update(t)
        tc.simulationStep()
        step += 1
        t = round(step * 0.1, 6)
        ids = tc.vehicle.getIDList()
        if "f0" in ids:
            lanes_seen.add(tc.vehicle.getLaneIndex("f0"))
        if "bus0" not in ids and "f0" not in ids:
            break
    ev = smgr.log[0] if smgr.log else None
    return {"model": model, "seed": seed, "event": ev, "changed_lane": 1 in lanes_seen,
            "a": None if ev is None else ev["aggressiveness"], "n_events": len(smgr.log)}
