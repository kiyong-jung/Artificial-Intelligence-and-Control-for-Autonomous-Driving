"""A2 검증용 합성 도로: 우선 교차로 합류(S5)와 정차 구역(S4)."""
import os
import random
import subprocess

from .cutin import cutin_vtype_xml, parking_area_xml
from .scenarios import NET_DIR, _write
from .sumo_conn import traci, sumo_cmd, checkBinary

ROUTE_LEN = 300.0     # main_in 길이 = 교차로까지 거리


def build_a2_net(v_ign=15.0, bins=None, single_bin=None):
    """main_in -> J(priority) -> main_out, 비우선 접근로 alley(J로 합류).

    single_bin=(p, timegap, impatience): vType 하나만 쓰는 보정용.
    """
    d = os.path.join(NET_DIR, "a2")
    _write(os.path.join(d, "n.nod.xml"),
           '<nodes><node id="W" x="-300" y="0"/><node id="J" x="0" y="0" type="priority"/>'
           '<node id="E" x="300" y="0"/><node id="S" x="0" y="-60"/></nodes>')
    _write(os.path.join(d, "e.edg.xml"),
           '<edges><edge id="main_in" from="W" to="J" numLanes="1" speed="13.9"/>'
           '<edge id="main_out" from="J" to="E" numLanes="1" speed="13.9"/>'
           '<edge id="alley" from="S" to="J" numLanes="1" speed="8.3"/></edges>')
    subprocess.run([checkBinary("netconvert"), "-n", os.path.join(d, "n.nod.xml"),
                    "-e", os.path.join(d, "e.edg.xml"), "-o", os.path.join(d, "net.net.xml"),
                    "--no-warnings"], check=True, capture_output=True)
    if single_bin is not None:
        p, tg, imp = single_bin
        vt = cutin_vtype_xml("cutin", bins=[(p, tg, imp, 1.0)], v_ign=v_ign)
    else:
        vt = cutin_vtype_xml("cutin", bins=bins, v_ign=v_ign)
    _write(os.path.join(d, "r.rou.xml"), f"""<routes>
<vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" tau="1.0" sigma="0"/>
<vType id="plain" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0"/>
{vt}
<route id="r_main" edges="main_in main_out"/>
<route id="r_alley" edges="alley main_out"/>
</routes>""")
    _write(os.path.join(d, "a2.sumocfg"), """<configuration>
<input><net-file value="net.net.xml"/><route-files value="r.rou.xml"/></input>
<time><begin value="0"/><end value="60"/></time>
</configuration>""")
    return os.path.join(d, "a2.sumocfg")


def run_cutin_episode(cfg, cutter_type, seed, bus_speedmode=31, bus_speed=11.0,
                      threshold_m=40.0, cutter_pos=0.0):
    """버스가 11 m/s로 접근할 때 골목 차량이 우선권을 어기고 들어오는지 본다.

    entered: 버스가 교차로까지 threshold_m 이내일 때 골목 차량이 교차로에 진입했는가.
    """
    tc = traci
    cmd = sumo_cmd(cfg, seed=seed)
    if tc.isLoaded():
        tc.load(cmd[1:])
    else:
        tc.start(cmd)
    rng = random.Random(seed)
    t_dep = rng.uniform(15.2, 17.5)          # 정지선 도착 시 버스가 15~40 m 앞
    tc.vehicle.add("bus0", "r_main", typeID="bus_test", departPos="0", departSpeed=str(bus_speed))
    tc.vehicle.setSpeedMode("bus0", bus_speedmode)
    entered, bus_dist_at_entry, collided, added = False, None, False, False
    t = 0.0
    step = 0
    while t < 45.0:
        if not added and t >= t_dep:
            tc.vehicle.add("c0", "r_alley", typeID=cutter_type, departPos=str(cutter_pos),
                           departSpeed="0")
            added = True
        if "bus0" in tc.vehicle.getIDList():
            tc.vehicle.setSpeed("bus0", bus_speed)
        tc.simulationStep()
        step += 1
        t = round(step * 0.1, 6)
        if tc.simulation.getCollisions():
            collided = True
        if added and not entered and "c0" in tc.vehicle.getIDList():
            road = tc.vehicle.getRoadID("c0")
            if road.startswith(":J") or road == "main_out":
                if "bus0" in tc.vehicle.getIDList():
                    br = tc.vehicle.getRoadID("bus0")
                    dist = (ROUTE_LEN - tc.vehicle.getLanePosition("bus0")) if br == "main_in" else 0.0
                    bus_dist_at_entry = dist
                    entered = 0.0 <= dist <= threshold_m and br == "main_in"
                else:
                    break
                if not entered:
                    # 이미 버스가 지나갔거나 먼 경우: 한 번만 판정하고 끝낸다
                    break
        if entered and t > 0 and "c0" not in tc.vehicle.getIDList():
            break
    return {"type": cutter_type, "seed": seed, "entered": entered,
            "bus_dist_at_entry": bus_dist_at_entry, "collision": collided}


# ---------------- S4: 정류장 앞 정차 차량이 버스를 막는가 ----------------

def build_s4_net(stop_duration=40):
    d = os.path.join(NET_DIR, "s4")
    _write(os.path.join(d, "n.nod.xml"),
           '<nodes><node id="A" x="0" y="0"/><node id="B" x="600" y="0"/></nodes>')
    _write(os.path.join(d, "e.edg.xml"),
           '<edges><edge id="E0" from="A" to="B" numLanes="1" speed="13.9"/></edges>')
    subprocess.run([checkBinary("netconvert"), "-n", os.path.join(d, "n.nod.xml"),
                    "-e", os.path.join(d, "e.edg.xml"), "-o", os.path.join(d, "net.net.xml"),
                    "--no-warnings"], check=True, capture_output=True)
    _write(os.path.join(d, "pa.add.xml"),
           "<additional>" + parking_area_xml("pa0", "E0_0", 300, 310, 1) + "</additional>")
    _write(os.path.join(d, "r.rou.xml"), f"""<routes>
<vType id="bus_test" vClass="bus" length="12" accel="1.2" decel="4.0" tau="1.0" sigma="0"/>
<vType id="taxi" vClass="taxi" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0"/>
<route id="r0" edges="E0"/>
<vehicle id="taxi0" type="taxi" route="r0" depart="0" departSpeed="max"><stop parkingArea="pa0" duration="{stop_duration}"/></vehicle>
<vehicle id="bus0" type="bus_test" route="r0" depart="8" departSpeed="max"/>
</routes>""")
    _write(os.path.join(d, "s4.sumocfg"), """<configuration>
<input><net-file value="net.net.xml"/><route-files value="r.rou.xml"/><additional-files value="pa.add.xml"/></input>
<time><begin value="0"/><end value="120"/></time>
</configuration>""")
    return os.path.join(d, "s4.sumocfg")


def run_s4(cfg):
    """택시가 정차하는 동안 버스가 뒤에서 멈추는지, 택시가 떠난 뒤 통과하는지 시계열로 기록."""
    tc = traci
    cmd = sumo_cmd(cfg, seed=0)
    if tc.isLoaded():
        tc.load(cmd[1:])
    else:
        tc.start(cmd)
    rows = []
    t = 0.0
    for step in range(1, 1201):
        tc.simulationStep()
        t = round(step * 0.1, 6)
        ids = tc.vehicle.getIDList()
        if "bus0" in ids and "taxi0" in ids:
            rows.append((t, tc.vehicle.getLanePosition("taxi0"), tc.vehicle.getSpeed("taxi0"),
                         tc.vehicle.getLanePosition("bus0"), tc.vehicle.getSpeed("bus0"),
                         tc.vehicle.isStoppedParking("taxi0")))
        elif "bus0" in ids:
            rows.append((t, None, None, tc.vehicle.getLanePosition("bus0"),
                         tc.vehicle.getSpeed("bus0"), False))
        if "bus0" not in ids and t > 10:
            break
    return rows
