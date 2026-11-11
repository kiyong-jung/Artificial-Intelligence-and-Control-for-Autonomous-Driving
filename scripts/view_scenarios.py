"""합정역 지도 위에서 상호작용 시나리오 S1~S6을 sumo-gui로 눈으로 확인한다.

    python scripts/view_scenarios.py S1            # GUI 창이 열리고 자동 재생
    python scripts/view_scenarios.py S5 --episodes 4 --delay 80
    python scripts/view_scenarios.py all --headless   # 창 없이 로그만(점검용)

색: 빨강=버스(시험용, 판단 없음), 초록=양보형, 주황=비양보형, 파랑=끼어드는 골목 차량, 노랑=택시.
버스 정책은 범위 밖이므로 버스는 정해진 시각·위치에서 차선 변경을 요청하는 시험용이다.
"""
import argparse
import os
import random
import sys

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT)
from traffic.cutin import cutin_vtype_xml, parking_area_xml  # noqa: E402
from traffic.pedestrian import ped_vtypes_xml  # noqa: E402
from traffic.scenarios import NET_DIR, _write  # noqa: E402
from traffic.sumo_conn import traci, sumo_cmd  # noqa: E402
from traffic.yield_logic import (YieldManager, DEFAULT_TYPES, BLINKER_LEFT,  # noqa: E402
                                 BLINKER_RIGHT)

NET = os.path.abspath(os.path.join(ROOT, "maps", "hapjeong", "source", "network.net.xml"))
STOPS = os.path.abspath(os.path.join(ROOT, "maps", "hapjeong", "source", "stops.add.xml"))
OUT = os.path.join(NET_DIR, "view")
VIEW = "View #0"

TYPES = (
    '<vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" tau="1.0" sigma="0" color="220,0,0"/>'
    '<vType id="yielder" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" speedFactor="1" speedDev="0" actionStepLength="0.5" color="0,170,0"/>'
    '<vType id="nonyielder" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" maxSpeed="9" speedFactor="1" speedDev="0" actionStepLength="0.5" color="255,140,0"/>'
    '<vType id="taxi" vClass="taxi" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" color="255,220,0"/>')

GUI_SETTINGS = """<viewsettings>
<scheme name="real world">
<vehicles vehicleQuality="2" showBlinker="1" vehicleName_show="1" vehicleName_size="45" vehicleName_color="0,0,0"/>
<persons personQuality="2" personName_show="1" personName_size="45" personName_color="0,0,0"/>
</scheme>
</viewsettings>"""


def make_cfg(name, routes, add="", end=120, stops=False):
    d = os.path.join(OUT, name)
    _write(os.path.join(d, "r.rou.xml"), f"<routes>{TYPES}{routes}</routes>")
    names = []
    if add:
        _write(os.path.join(d, "a.add.xml"), f"<additional>{add}</additional>")
        names.append("a.add.xml")
    if stops:
        names.append(STOPS)
    files = f'<additional-files value="{",".join(names)}"/>' if names else ""
    _write(os.path.join(d, "v.sumocfg"),
           f'<configuration><input><net-file value="{NET}"/><route-files value="r.rou.xml"/>{files}'
           f'</input><time><begin value="0"/><end value="{end}"/></time></configuration>')
    return os.path.join(d, "v.sumocfg")


CAPTION = []          # 장면이 끝나며 남기는 결과 문장(GIF 자막용)


def say(text):
    print(text)
    CAPTION.append(text)


class Runner:
    def __init__(self, args):
        self.a = args
        self.started = False
        self.step = 0
        self.rec = None          # 켜 두면 4스텝(0.4 s)마다 차량·보행자 상태를 기록한다(GIF 그리기용)
        self.rec_every = 4
        _write(os.path.join(OUT, "gui.xml"), GUI_SETTINGS)

    def start(self, cfg, seed):
        extra = []
        binary = "sumo" if self.a.headless else "sumo-gui"
        if not self.a.headless:
            extra += ["--start", "--delay", str(self.a.delay), "--gui-settings-file",
                      os.path.join(OUT, "gui.xml"), "--window-size", "1400,900"]
        cmd = sumo_cmd(cfg, seed=seed, extra=extra, binary=binary)
        if traci.isLoaded():
            if self.a.headless:
                traci.load(cmd[1:])
            else:                       # GUI는 load로 다시 읽으면 멈추므로 창을 닫고 새로 연다
                traci.close()
                traci.start(cmd)
        else:
            traci.start(cmd)
        self.cam_set = False
        self.step = 0
        if self.rec is not None:
            self.rec = []
            self.snapshot()

    def camera(self, vid, zoom):
        if self.a.headless or self.cam_set or vid not in traci.vehicle.getIDList():
            return
        try:
            traci.gui.setSchema(VIEW, "real world")
            traci.gui.trackVehicle(VIEW, vid)
            traci.gui.setZoom(VIEW, zoom)
            self.cam_set = True
        except Exception:
            pass

    def snapshot(self):
        tc = traci
        vs = [(v, *tc.vehicle.getPosition(v), tc.vehicle.getAngle(v), tc.vehicle.getLength(v),
               tc.vehicle.getWidth(v), tuple(tc.vehicle.getColor(v))[:3], tc.vehicle.getSignals(v))
              for v in tc.vehicle.getIDList()]
        ps = [(p, *tc.person.getPosition(p), tuple(tc.person.getColor(p))[:3])
              for p in tc.person.getIDList()]
        self.rec.append({"t": tc.simulation.getTime(), "v": vs, "p": ps})

    def tick(self, name):
        traci.simulationStep()
        self.step += 1
        if self.rec is not None and self.step % self.rec_every == 0:
            self.snapshot()


# ---------------------------------------------------------------- S1~S3 (A1)
class Bus:
    """정해진 위치에서 차선 변경을 요청하는 시험용 버스. 판단은 없다."""

    def __init__(self, vid, changes, v_max=9.0, accel=1.5, t_go=3.0, v0=0.0, release_after_last=False):
        self.vid, self.changes, self.v_max, self.accel = vid, list(changes), v_max, accel
        self.t_go, self.v0, self.release_after_last = t_go, v0, release_after_last
        self.requested, self.done_at, self.released = False, [], False

    def act(self, tc, t):
        v = self.vid
        if self.released:
            return
        if t < self.t_go:
            tc.vehicle.setSpeed(v, self.v0)
        else:
            tc.vehicle.setSpeed(v, min(self.v_max, self.v0 + self.accel * (t - self.t_go)))
        if not self.changes:
            tc.vehicle.setSignals(v, 0)
            if self.release_after_last:
                tc.vehicle.setSpeed(v, -1)
                self.released = True
            return
        c = self.changes[0]
        if tc.vehicle.getRoadID(v) != c["edge"]:
            return
        pos, lane = tc.vehicle.getLanePosition(v), tc.vehicle.getLaneIndex(v)
        if pos >= c["pos"] - 20:
            tc.vehicle.setSignals(v, BLINKER_LEFT if c["lane"] > lane else BLINKER_RIGHT)
        if pos >= c["pos"] and not self.requested:
            tc.vehicle.changeLane(v, c["lane"], 15.0)
            self.requested = True
        if lane == c["lane"]:
            self.done_at.append((round(t, 1), c["edge"], c["lane"]))
            self.changes.pop(0)
            self.requested = False
            if not self.changes or self.changes[0]["edge"] != c["edge"]:
                tc.vehicle.setSignals(v, 0)


def a1_scene(R, name, route_bus, route_f, bus, bus_start, followers, ftypes, seed, t_end, zoom=350,
             stop=None):
    """followers: [{id, route, lane, pos, when:(edge,lanePos)}]  ftypes: 같은 순서의 유형(양보/비양보)."""
    cfg = make_cfg(name, f'<route id="rb" edges="{route_bus}"/>' + "".join(
        f'<route id="{k}" edges="{e}"/>' for k, e in route_f.items()), stops=bool(stop))
    R.start(cfg, seed)
    tc = traci
    rng = random.Random(seed)
    edge, pos, lane = bus_start
    tc.vehicle.add("BUS", "rb", typeID="bus_test", departPos=str(pos), departLane=str(lane),
                   departSpeed=str(bus.v0))
    tc.vehicle.setLaneChangeMode("BUS", 512)   # 모델이 임의로 바꾸지 않고, 요청한 변경만 주변을 보고 실행
    tc.vehicle.setSpeedMode("BUS", 15)         # 적색 신호는 무시, 안전거리는 유지
    tc.vehicle.setSpeed("BUS", bus.v0)
    if stop:
        tc.vehicle.setBusStop("BUS", stop, duration=10)
    mgr = YieldManager(tc, "BUS", rng=rng, types=DEFAULT_TYPES, stop_exit_pos=None, follower_range=60.0)
    pending = [dict(f, type=t) for f, t in zip(followers, ftypes)]
    t = 0.0
    while t < t_end:
        R.camera("BUS", zoom)
        for f in list(pending):
            e, p = f["when"]
            if "BUS" in tc.vehicle.getIDList() and tc.vehicle.getRoadID("BUS") == e and \
                    tc.vehicle.getLanePosition("BUS") >= p:
                tc.vehicle.add(f["id"], f["route"], typeID=f["type"], departPos=str(f["pos"]),
                               departLane=str(f["lane"]), departSpeed="9")
                tc.vehicle.setLaneChangeMode(f["id"], 0)     # 후행차는 제 차로를 유지한다
                pending.remove(f)
        mgr.on_departed(tc.simulation.getDepartedIDList())
        if "BUS" in tc.vehicle.getIDList():
            bus.act(tc, t)
        mgr.update(t)
        R.tick(name)
        if R.a.trace and R.step % 10 == 0 and "BUS" in tc.vehicle.getIDList():
            row = [f"t={t:5.1f}"]
            for v in tc.vehicle.getIDList():
                row.append(f"{v}:{tc.vehicle.getRoadID(v)}/{tc.vehicle.getLaneIndex(v)}"
                           f"@{tc.vehicle.getLanePosition(v):.0f} v{tc.vehicle.getSpeed(v):.1f}")
            print("  ".join(row))
        t = round(t + 0.1, 6)
        if "BUS" not in tc.vehicle.getIDList():
            break
    rows = list(mgr.log) + [{"follower": k, "yield": v.yields, "t_open": v.t_open, "max_decel": v.max_decel}
                            for k, v in mgr.inter.items()]
    return rows, bus.done_at


def report_a1(name, seed, types, rows, done):
    say(f"[{name}] seed={seed} 후행차 유형={types}")
    for r in rows:
        say(f"   {r['follower']}: 양보 결정={'양보' if r['yield'] else '비양보'}, "
              f"openGap 시각={r['t_open']}, 최대 감속={r['max_decel']:.2f} m/s²")
    say(f"   버스 차로 변경 완료: {done if done else '못 함(비양보 후행차에 막힘)'}")


def run_s1(R, ep, seed):
    """S1 정류장 출발 후 본선 재합류: 중앙 정류장(B1)을 떠나 R43 3차로에서 2차로로 우측 합류."""
    types = ["yielder", "nonyielder"][ep % 2]
    bus = Bus("BUS", [dict(edge="R43", pos=42, lane=2)], v_max=6.0, t_go=1.0)
    rows, done = a1_scene(R, "S1", "B1 R41 R43", {"rf": "R41 R43"}, bus, ("B1", 160, 0),
                          [dict(id=f"{types}-1", route="rf", lane=1, pos=0, when=("R43", 1))],
                          [types], seed, 40)
    report_a1("S1", seed, [types], rows, done)


def run_s2(R, ep, seed):
    """S2 정류장 직후 좌회전 연속 차선 변경: R43에서 4차로로, 이어 R50에서 5차로로(좌측 이동)."""
    ts = [["yielder", "yielder"], ["nonyielder", "yielder"], ["yielder", "nonyielder"],
          ["nonyielder", "nonyielder"]][ep % 4]
    bus = Bus("BUS", [dict(edge="R43", pos=300, lane=4), dict(edge="R50", pos=55, lane=5)], v_max=6.0)
    rows, done = a1_scene(R, "S2", "B1 R41 R43 R50", {"rf": "R43 R50", "rg": "R50"}, bus,
                          ("B1", 160, 0),
                          [dict(id=f"{ts[0]}-A", route="rf", lane=4, pos=0, when=("R43", 115)),
                           dict(id=f"{ts[1]}-B", route="rg", lane=5, pos=0, when=("R50", 34))],
                          ts, seed, 100, zoom=350)
    report_a1("S2", seed, ts, rows, done)


def run_s3(R, ep, seed):
    """S3 정류장 접근 연속 차선 변경: R51 1차로에서 2, 3차로로 좌측 이동해 중앙 버스 차로(B1)로 진입."""
    ts = [["yielder", "yielder"], ["nonyielder", "yielder"], ["yielder", "nonyielder"],
          ["nonyielder", "nonyielder"]][ep % 4]
    bus = Bus("BUS", [dict(edge="R51", pos=110, lane=2), dict(edge="R51", pos=160, lane=3)],
              v_max=6.0, accel=3.0, t_go=0.0, v0=6.0, release_after_last=True)
    rows, done = a1_scene(R, "S3", "R51 R49a B1", {"rf": "R51 R49a R49"}, bus, ("R51", 60, 1),
                          [dict(id=f"{ts[0]}-A", route="rf", lane=2, pos=5, when=("R51", 61)),
                           dict(id=f"{ts[1]}-B", route="rf", lane=3, pos=61, when=("R51", 110))],
                          ts, seed, 70, zoom=350, stop="stop_0")
    report_a1("S3", seed, ts, rows, done)


# ---------------------------------------------------------------- S4 (택시 정차)
def run_s4(R, ep, seed):
    """S4 정류장 앞 택시 정차: R69 안쪽 차로에 택시가 30초 정차, 차선 변경을 끈 버스가 뒤에서 막힌다."""
    cfg = make_cfg("S4", '<route id="r" edges="R69"/>'
                   '<vehicle id="TAXI" type="taxi" route="r" depart="0" departLane="1" departSpeed="max">'
                   '<stop parkingArea="pa0" duration="30"/></vehicle>'
                   '<vehicle id="BUS" type="bus_test" route="r" depart="3" departLane="1" departSpeed="max"/>',
                   parking_area_xml("pa0", "R69_1", 30, 40, 1))
    R.start(cfg, seed)
    lcset, blocked, t = False, 0.0, 0.0
    for _ in range(900):
        ids = traci.vehicle.getIDList()
        if "BUS" in ids and not lcset:
            traci.vehicle.setLaneChangeMode("BUS", 0)
            lcset = True
        R.camera("BUS", 300)
        R.tick("S4")
        t += 0.1
        ids = traci.vehicle.getIDList()
        if "BUS" in ids and "TAXI" in ids and traci.vehicle.isStoppedParking("TAXI") \
                and traci.vehicle.getSpeed("BUS") < 0.1:
            blocked += 0.1
        if "BUS" not in ids and t > 5:
            break
    say(f"[S4] 택시 정차 중 버스가 뒤에서 멈춘 시간 {blocked:.1f} s")


# ---------------------------------------------------------------- S5 (끼어들기)
def run_s5(R, ep, seed):
    """S5 합류부 골목 차량 끼어들기: 독막로 버스 앞으로 독막로3길 차량이 신호 없이 진입한다."""
    vt = cutin_vtype_xml("cutin", bins=[(1.0, 0.5, 0.5, 1.0)], v_ign=15.0)
    cfg = make_cfg("S5", vt + '<route id="rm" edges="R71 R71.135"/><route id="ra" edges="G3 R71.135"/>')
    R.start(cfg, seed)
    rng = random.Random(seed)
    t_dep = rng.uniform(2.0, 4.5)
    traci.vehicle.add("BUS", "rm", typeID="bus_test", departPos="0", departLane="1", departSpeed="11")
    added, entered, coll = False, None, False
    for k in range(1, 500):
        t = k * 0.1
        R.camera("BUS", 400)
        if not added and t >= t_dep:
            traci.vehicle.add("CUTIN", "ra", typeID="cutin_0", departPos="0", departSpeed="0")
            traci.vehicle.setColor("CUTIN", (0, 90, 255, 255))
            added = True
        if "BUS" in traci.vehicle.getIDList():
            traci.vehicle.setSpeed("BUS", 11.0)
        R.tick("S5")
        coll = coll or bool(traci.simulation.getCollisions())
        if added and "CUTIN" in traci.vehicle.getIDList() and entered is None:
            r = traci.vehicle.getRoadID("CUTIN")
            if (r.startswith(":X_dongmak3") or r == "R71.135") and traci.vehicle.getRoadID("BUS") == "R71":
                entered = 130.459 - traci.vehicle.getLanePosition("BUS")
        if "BUS" not in traci.vehicle.getIDList() and k > 50:
            break
    msg = f"버스 앞 {entered:.1f} m에서 신호 없이 진입" if entered is not None else "버스를 보내고 난 뒤 진입"
    say(f"[S5] seed={seed}: {msg}, 충돌={'있음' if coll else '없음'}")


# ---------------------------------------------------------------- S6 (횡단보도)
def run_s6(R, ep, seed):
    """S6 신호 없는 횡단보도: 보행자 한 명이 접근한다. P1(대기형)은 버스가 느릴 때만, P2(진입형)는 간격이 충분하면 건넌다."""
    ptype, v, T = [("ped_p1", 2.0, 10), ("ped_p2", 2.0, 10), ("ped_p2", 5.0, 3), ("ped_p1", 1.0, 8)][ep % 4]
    L = 216.725
    cfg = make_cfg("S6", ped_vtypes_xml(1.5, 0.75) + '<route id="r" edges="R112 R112.221"/>'
                   f'<person id="{ptype}" type="{ptype}" depart="0" departPos="210">'
                   '<walk from="R112" to="-R112" arrivalPos="40"/></person>')
    R.start(cfg, seed)
    traci.person.setColor(ptype, (200, 0, 200, 255) if ptype == "ped_p1" else (0, 160, 160, 255))
    traci.vehicle.add("BUS", "r", typeID="bus_test", departPos=str(max(0.0, L - v * T - 0.5)),
                      departLane="1", departSpeed=str(v))
    enter, passed = None, None
    for k in range(1, 900):
        t = k * 0.1
        R.camera("BUS", 700)
        if "BUS" in traci.vehicle.getIDList():
            traci.vehicle.setSpeed("BUS", v)
        R.tick("S6")
        if passed is None and "BUS" in traci.vehicle.getIDList() and traci.vehicle.getRoadID("BUS") != "R112":
            passed = t
        if enter is None and ptype in traci.person.getIDList() and                 traci.person.getRoadID(ptype) == ":S6_wausanro_c0":
            enter = t
        if enter and passed and t > max(enter, passed) + 3:
            break
        if "BUS" not in traci.vehicle.getIDList() and t > (passed or 0) + 8:
            break
    before = enter is not None and (passed is None or enter < passed)
    say(f"[S6] {'P1(대기형)' if ptype == 'ped_p1' else 'P2(진입형)'} | 버스 {v} m/s, 횡단보도까지 {T} s 거리 -> "
          f"{'버스보다 먼저 건넘' if before else '버스를 보내고 건넘'}")


SCENES = {"S1": run_s1, "S2": run_s2, "S3": run_s3, "S4": run_s4, "S5": run_s5, "S6": run_s6}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", help="S1~S6 또는 all")
    ap.add_argument("--episodes", type=int, default=2, help="같은 장면을 시드/유형을 바꿔 몇 번 보여 줄지")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--delay", type=int, default=60, help="스텝마다 지연 ms (클수록 느리게)")
    ap.add_argument("--headless", action="store_true", help="GUI 없이 실행(점검용)")
    ap.add_argument("--trace", action="store_true", help="1초마다 차량 위치 출력(S1~S3)")
    a = ap.parse_args()
    R = Runner(a)
    names = list(SCENES) if a.scene.lower() == "all" else [a.scene.upper()]
    for n in names:
        for ep in range(1 if n == "S4" else a.episodes):
            SCENES[n](R, ep, a.seed + ep)
    traci.close()


if __name__ == "__main__":
    main()
