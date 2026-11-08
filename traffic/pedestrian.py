"""A3. 보행자 P1(대기형)·P2(진입형): vType 파라미터로 구현하고 합성 횡단보도에서 검증한다.

P1: jmIgnoreFoeProb 1 + jmIgnoreFoeSpeed = v_low, impatience 0 -> 버스가 v_low 이하로 느려야 건넌다.
P2: 기본 간격 수용 + impatience(0.5~1) -> 버스가 충분히 멀면 건넌다.
"""
import os
import subprocess

from .scenarios import NET_DIR, _write
from .sumo_conn import traci, sumo_cmd, checkBinary

CROSSING = ":C_c0"         # 비우선(priority=false) 횡단보도
BASE = 'vClass="pedestrian" width="0.5" length="0.3" minGap="0.2" maxSpeed="1.4" speedFactor="1" speedDev="0"'


def ped_vtypes_xml(v_low=1.5, p2_impatience=0.75):
    return (f'<vType id="ped_p1" {BASE} jmIgnoreFoeProb="1" jmIgnoreFoeSpeed="{v_low}" impatience="0"/>\n'
            f'<vType id="ped_p2" {BASE} impatience="{p2_impatience}"/>\n'
            f'<vType id="ped_force" {BASE} jmIgnoreFoeProb="1" jmIgnoreFoeSpeed="100" impatience="1"/>')


def build_a3_net():
    d = os.path.join(NET_DIR, "a3")
    _write(os.path.join(d, "n.nod.xml"),
           '<nodes><node id="W" x="-300" y="0"/><node id="C" x="0" y="0" type="priority"/>'
           '<node id="E" x="300" y="0"/><node id="N" x="0" y="100"/></nodes>')
    sw = 'sidewalkWidth="2"'
    _write(os.path.join(d, "e.edg.xml"),
           f'<edges><edge id="main_in" from="W" to="C" numLanes="1" speed="13.9" {sw}/>'
           f'<edge id="main_out" from="C" to="E" numLanes="1" speed="13.9" {sw}/>'
           f'<edge id="rev_in" from="E" to="C" numLanes="1" speed="13.9" {sw}/>'
           f'<edge id="rev_out" from="C" to="W" numLanes="1" speed="13.9" {sw}/>'
           f'<edge id="side_in" from="N" to="C" numLanes="1" speed="8.3" {sw}/>'
           f'<edge id="side_out" from="C" to="N" numLanes="1" speed="8.3" {sw}/></edges>')
    _write(os.path.join(d, "c.con.xml"),
           '<connections><crossing node="C" edges="rev_out main_in" priority="false"/></connections>')
    subprocess.run([checkBinary("netconvert"), "-n", os.path.join(d, "n.nod.xml"),
                    "-e", os.path.join(d, "e.edg.xml"), "-x", os.path.join(d, "c.con.xml"),
                    "-o", os.path.join(d, "net.net.xml"), "--no-warnings"],
                   check=True, capture_output=True)
    _write(os.path.join(d, "a3.sumocfg"), """<configuration>
<input><net-file value="net.net.xml"/><route-files value="r.rou.xml"/></input>
<time><begin value="0"/><end value="120"/></time>
</configuration>""")
    return os.path.join(d, "a3.sumocfg")


def _write_routes(ptype, v_low, impatience, ped_depart, ped_pos=289):
    d = os.path.join(NET_DIR, "a3")
    _write(os.path.join(d, "r.rou.xml"), f"""<routes>
<vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" tau="1.0" sigma="0"/>
{ped_vtypes_xml(v_low, impatience)}
<route id="r_main" edges="main_in main_out"/>
<person id="p0" type="{ptype}" depart="{ped_depart}" departPos="{ped_pos}">
<walk from="main_in" to="rev_out" arrivalPos="20"/></person>
</routes>""")


def run_crossing_episode(cfg, ptype, bus_speed, bus_dist=30.0, v_low=1.5, impatience=0.75,
                         bus_speedmode=31, ped_depart=0.0, max_t=90.0, seed=0, ped_pos=289):
    """버스가 횡단보도 bus_dist m 앞에서 bus_speed로 직진할 때, 보행자가 버스 통과 전에 건너기 시작하는가."""
    _write_routes(ptype, v_low, impatience, ped_depart, ped_pos)
    tc = traci
    cmd = sumo_cmd(cfg, seed=seed, extra=["--collision-output",
                                          os.path.join(NET_DIR, "a3", "coll.xml")])
    if tc.isLoaded():
        tc.load(cmd[1:])
    else:
        tc.start(cmd)
    x_stop = 292.8                   # main_in 차로 끝 = 횡단보도 앞
    tc.vehicle.add("bus0", "r_main", typeID="bus_test", departPos=str(x_stop - bus_dist),
                   departLane="1", departSpeed=str(bus_speed))
    tc.vehicle.setSpeedMode("bus0", bus_speedmode)
    enter_t, bus_pos_at_enter, bus_speed_at_enter, collided = None, None, None, False
    bus_pass_t = None
    t, step = 0.0, 0
    while t < max_t:
        if "bus0" in tc.vehicle.getIDList():
            tc.vehicle.setSpeed("bus0", bus_speed)
        tc.simulationStep()
        step += 1
        t = round(step * 0.1, 6)
        if tc.simulation.getCollisions():
            collided = True
        if bus_pass_t is None and "bus0" in tc.vehicle.getIDList() and \
                tc.vehicle.getRoadID("bus0") != "main_in":
            bus_pass_t = t
        if enter_t is None and "p0" in tc.person.getIDList() and tc.person.getRoadID("p0") == CROSSING:
            enter_t = t
            if "bus0" in tc.vehicle.getIDList():
                bus_pos_at_enter = tc.vehicle.getLanePosition("bus0") \
                    if tc.vehicle.getRoadID("bus0") == "main_in" else x_stop
                bus_speed_at_enter = tc.vehicle.getSpeed("bus0")
        if enter_t is not None and bus_pass_t is not None and "p0" not in tc.person.getIDList():
            break
        if bus_pass_t is not None and enter_t is not None and t > enter_t + 12:
            break
        if "bus0" not in tc.vehicle.getIDList() and "p0" not in tc.person.getIDList():
            break
    bus_passed_first = enter_t is None or (bus_pass_t is not None and enter_t >= bus_pass_t)
    return {"ptype": ptype, "bus_speed": bus_speed, "bus_dist": bus_dist, "enter_t": enter_t,
            "crossed_before_bus": enter_t is not None and not bus_passed_first,
            "bus_speed_at_enter": bus_speed_at_enter, "collision": collided,
            "bus_pass_t": bus_pass_t}
