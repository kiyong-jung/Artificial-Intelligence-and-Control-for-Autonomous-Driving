"""합정역 버스 순환 노선(bus_loop) 시나리오: 직접 눈으로 확인하는 시뮬레이션.

    python scripts/run_hapjeong_scenario.py                         # sumo-gui가 열리고 버스를 따라간다
    python scripts/run_hapjeong_scenario.py --nogui --seconds 900   # 화면 없이 요약만

화면에서 보이는 것
  빨간 버스      : 시험용 버스. 차로 변경이 필요하면 방향지시등을 켜고 요청한다(주행 정책이 아니라 규칙).
  주변 차량 색   : 주행 모델. 노랑 Krauss, 청록 IDM, 주황 EIDM, 자홍 ACC, 회색 끼어들기(골목 합류) 차량.
  버스를 마주한 차: 버스가 차로 변경 의사를 보이면 뒤차 주위에 고리가 생긴다.
                    초록 고리 = 이번 상호작용에서 양보로 판정(길을 터 줌), 빨간 고리 = 비양보로 판정.
                    차량은 양보형(양보 확률 높음)/비양보형(낮음) 성격을 갖고, 판정은 그 확률로 상호작용마다 한 번 뽑는다.
                    고리는 상호작용이 끝난 뒤 4초 더 남는다. 같은 내용을 터미널에도 한 줄씩 적는다.
  차량을 더블클릭하면 파라미터에서 driver.type, driver.pYield(설정 양보 확률)를 볼 수 있다.
"""
import argparse
import collections
import math
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maps import load_map  # noqa: E402
from traffic.hapjeong import HapjeongTraffic  # noqa: E402
from traffic.sumo_conn import traci  # noqa: E402
from traffic.yield_logic import BLINKER_LEFT, BLINKER_RIGHT  # noqa: E402

BUS = "bus0"
MODEL_NAME = {"car_krauss": "Krauss", "car_idm": "IDM", "car_eidm": "EIDM", "car_acc": "ACC"}
YIELD_RGB, NOYIELD_RGB = (0, 190, 0, 255), (255, 0, 0, 255)


class LoopBus:
    """bus_loop를 계속 도는 시험용 버스. 차로 변경만 이 클래스가 하고 나머지는 SUMO 기본 주행에 맡긴다.

    SUMO 자율 차선 변경은 끄고(laneChangeMode 512) 이 클래스가 요청한다. 경로상 가야 할 차로
    (getBestLanes)와 지금 차로가 다르면 방향지시등을 SIGNAL_S초 켠 뒤 changeLane을 요청한다.
    한 바퀴를 마치면 같은 자리에서 다시 출발한다.
    """
    SIGNAL_S, REQUEST_DURATION, TIMEOUT_S = 3.0, 5.0, 14.0

    def __init__(self, tc, loop_edges, stops, with_stops=True, stop_s=8.0, discretionary=True, probe_s=10.0, companions=True):
        self.tc, self.stops, self.with_stops, self.stop_s = tc, stops, with_stops, stop_s
        closure = tc.simulation.findRoute(loop_edges[-1], loop_edges[0], vType="bus_test").edges
        self.route = list(loop_edges) + list(closure[1:])      # 끝에서 시작점으로 이어 붙여 이음새 없이 돈다
        tc.route.add("bus_loop_closed", self.route)
        self.laps, self.lap_t0, self.lap_times = 0, None, []
        self.phase, self.t_phase, self.target = "idle", 0.0, None
        self.alive = False
        self._pending = False
        self.changes = collections.Counter()
        self.probe_s, self.t_probe, self.probe_n = probe_s, -99.0, 0
        self.companions, self.n_comp = companions, 0
        self.discretionary, self.t_disc, self.why, self._left = discretionary, -99.0, "", True

    def _spawn(self):
        tc = self.tc
        tc.vehicle.add(BUS, "bus_loop_closed", typeID="bus_test", departLane="best", departPos="0",
                       departSpeed="0", arrivalPos="5")
        tc.vehicle.setColor(BUS, (255, 0, 0, 255))
        tc.vehicle.setLaneChangeMode(BUS, 512)
        if self.with_stops:
            ordered = sorted(self.stops, key=lambda s: (self.route.index(s["edge"]), s["startPos"]))
            for s in ordered:
                try:
                    tc.vehicle.setBusStop(BUS, s["id"], duration=self.stop_s)
                except traci.exceptions.TraCIException as exc:
                    print(f"  (정류장 {s['name']} 건너뜀: {exc})")
        self.phase, self.target = "idle", None

    def tick(self, tc, t):
        if BUS not in tc.vehicle.getIDList():
            if self.alive:                                    # 바퀴 완주
                self.alive = False
                self.laps += 1
                self.lap_times.append(t - self.lap_t0)
                print(f"[{t:7.1f}s] 버스 {self.laps}바퀴 완주 ({self.lap_times[-1]:.0f}초)")
            if not self._pending:                             # 출발 자리가 막혀 있으면 SUMO가 빌 때까지 기다린다
                self._spawn()
                self._pending = True
            return
        self._pending = False
        if not self.alive:
            self.alive, self.lap_t0 = True, t
            print(f"[{t:7.1f}s] 버스 출발 ({self.laps + 1}바퀴째)")
        self._pilot(tc, t)

    def _drivable(self, tc, road, k):
        """차로 k가 버스가 갈 수 있는 차로인가(인도 차로 제외)."""
        try:
            allowed = tc.lane.getAllowed(f"{road}_{k}")
        except traci.exceptions.TraCIException:
            return False
        return not allowed or "bus" in allowed or "passenger" in allowed

    def _spawn_companion(self, tc, road, lane, pos):
        """옮겨 갈 차로 뒤 30m에 배경 차량 한 대를 넣는다(교통이 한산해 상호작용이 드물기 때문).
        주행 모델은 배경 차량 분포(carDist)에서 뽑히고, 양보 성격은 투입 시 YieldManager가 붙인다."""
        route = tc.vehicle.getRoute(BUS)
        i = tc.vehicle.getRouteIndex(BUS)
        self.n_comp += 1
        rid = f"comp_route{self.n_comp}"
        tc.route.add(rid, list(route[i:i + 3]))
        try:
            tc.vehicle.add(f"comp{self.n_comp}", rid, typeID="carDist", departLane=str(lane),
                           departPos=f"{pos - 30.0:.1f}", departSpeed=f"{tc.vehicle.getSpeed(BUS):.1f}")
        except traci.exceptions.TraCIException:
            return False
        return True

    def _want_offset(self, tc, t, road, idx, pos):
        """바꿔야 할 차로 수(양수=왼쪽). 필수(경로·정류장) 우선, 없으면 느린 앞차를 피하는 선택적 변경."""
        for entry in tc.vehicle.getBestLanes(BUS):
            if entry[0] == tc.vehicle.getLaneID(BUS) and entry[3] != 0:
                return entry[3], "경로"
        if self.with_stops:
            for s in self.stops:
                if s["edge"] == road and 0 <= s["startPos"] - pos < 150.0 and idx != s["lane_idx"]:
                    return s["lane_idx"] - idx, "정류장"
        if self.discretionary and t - self.t_disc >= 25.0 and tc.vehicle.getStopState(BUS) == 0:
            lead = tc.vehicle.getLeader(BUS, 40.0)
            if lead and lead[1] < 30.0 and tc.vehicle.getSpeed(lead[0]) < tc.vehicle.getSpeed(BUS) - 1.0                     and tc.lane.getLength(tc.vehicle.getLaneID(BUS)) - pos > 80.0:
                for k in (idx + 1, idx - 1):
                    if k >= 0 and self._drivable(tc, road, k):
                        return k - idx, "추월"
        if self.probe_s and t - self.t_probe >= self.probe_s and tc.vehicle.getStopState(BUS) == 0                 and tc.lane.getLength(tc.vehicle.getLaneID(BUS)) - pos > 100.0                 and not any(st["edge"] == road and st["startPos"] > pos - 20 for st in self.stops):
            # 옮겨 갈 쪽 차로 뒤 40m 안에 차가 있을 때만 시험한다(상호작용이 생기는 순간을 만든다)
            for k in (idx + 1, idx - 1):
                if k < 0 or not self._drivable(tc, road, k):
                    continue
                near = tc.vehicle.getLeftFollowers(BUS) if k > idx else tc.vehicle.getRightFollowers(BUS)
                if any(v and 0 <= d <= 40.0 for v, d in near):
                    return k - idx, "시험"
                if self.companions and pos >= 40.0 and self._spawn_companion(tc, road, k, pos):
                    return k - idx, "시험(동행차)"
        return 0, ""

    def _pilot(self, tc, t):
        road, idx, pos = tc.vehicle.getRoadID(BUS), tc.vehicle.getLaneIndex(BUS), tc.vehicle.getLanePosition(BUS)
        off, why = (0, "") if road.startswith(":") else self._want_offset(tc, t, road, idx, pos)
        if self.phase == "idle":
            if off != 0:
                self.target, self.why, self._left = idx + (1 if off > 0 else -1), why, off > 0
                self.t_disc = t if why == "추월" else self.t_disc
                if why.startswith("시험"):
                    self.t_probe, self.probe_n = t, self.probe_n + 1
                tc.vehicle.setSignals(BUS, BLINKER_LEFT if off > 0 else BLINKER_RIGHT)
                self.phase, self.t_phase = "signal", t
                self.changes[why] += 1
                print(f"[{t:7.1f}s] 버스 방향지시등 {'왼쪽' if off > 0 else '오른쪽'} ({why}) "
                      f"{road} 차로 {idx}→{self.target}")
        elif self.phase == "signal":
            if off == 0 and self.why not in ("추월", "시험", "시험(동행차)"):
                self._reset(tc, t)
            elif t - self.t_phase >= self.SIGNAL_S:
                tc.vehicle.changeLane(BUS, self.target, self.REQUEST_DURATION)
                self.phase, self.t_phase = "request", t
        elif self.phase == "request":
            tc.vehicle.setSignals(BUS, BLINKER_LEFT if self._left else BLINKER_RIGHT)   # 변경이 끝날 때까지 유지
            if idx == self.target or t - self.t_phase > self.TIMEOUT_S:
                self._reset(tc, t)
            elif int(round((t - self.t_phase) * 10)) % 20 == 0:
                tc.vehicle.changeLane(BUS, self.target, self.REQUEST_DURATION)
        elif self.phase == "cool" and t - self.t_phase > 2.0:
            self.phase = "idle"

    def _reset(self, tc, t):
        tc.vehicle.setSignals(BUS, 0)
        self.phase, self.t_phase = "cool", t


class Rings:
    """버스를 마주해 양보/비양보가 정해진 차량 주위에 고리를 그리고 터미널에 기록한다."""
    HOLD_S, RADIUS = 4.0, 4.0

    def __init__(self, tc, mgr):
        self.tc, self.mgr = tc, mgr
        self.rings = {}                    # 차량 id -> [고리 id, 소멸 시각]
        self.counts = collections.Counter()
        self.seq = 0

    def _shape(self, vid):
        x, y = self.tc.vehicle.getPosition(vid)
        return [(x + self.RADIUS * math.cos(a * math.pi / 8), y + self.RADIUS * math.sin(a * math.pi / 8))
                for a in range(17)]

    def step(self, t):
        tc = self.tc
        ids = set(tc.vehicle.getIDList())
        for vid, it in self.mgr.inter.items():
            if vid in self.rings or vid not in ids:
                continue
            model = MODEL_NAME.get(tc.vehicle.getTypeID(vid).split("@")[0], tc.vehicle.getTypeID(vid))
            self.seq += 1
            pid = f"ring{self.seq}"
            tc.polygon.add(pid, self._shape(vid), YIELD_RGB if it.yields else NOYIELD_RGB,
                           fill=False, layer=200, lineWidth=0.7)
            self.rings[vid] = [pid, None]
            self.counts[(model, it.yields)] += 1
            p = float(tc.vehicle.getParameter(vid, "driver.pYield"))
            kind = "양보형" if tc.vehicle.getParameter(vid, "driver.type") == "yielder" else "비양보형"
            print(f"[{t:7.1f}s] {vid:<14} {model:<6} {kind}(설정 양보 확률 {p:.2f}) → 이번 판정: "
                  f"{'양보' if it.yields else '비양보'}")
        for vid in list(self.rings):
            pid, expire = self.rings[vid]
            if expire is None and vid not in self.mgr.inter:
                expire = self.rings[vid][1] = t + self.HOLD_S
            if vid not in ids or (expire is not None and t >= expire):
                tc.polygon.remove(pid)
                del self.rings[vid]
            else:
                tc.polygon.setShape(pid, self._shape(vid))

    def summary(self):
        rows = []
        for model in ("Krauss", "IDM", "EIDM", "ACC"):
            y, n = self.counts[(model, True)], self.counts[(model, False)]
            if y + n:
                rows.append(f"  {model:<6} 양보 {y:3d}  비양보 {n:3d}")
        return "\n".join(rows) or "  (상호작용 없음)"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nogui", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--scenario", choices=("quiet", "baseline", "busy"), default="baseline")
    ap.add_argument("--warmup", type=float, default=120.0, help="버스 출발 전 배경 교통을 흘리는 시간(초)")
    ap.add_argument("--seconds", type=float, default=1800.0, help="warmup 뒤 시뮬레이션 시간(초)")
    ap.add_argument("--delay", type=int, default=50, help="GUI에서 0.1초 스텝마다 쉬는 시간(ms). 클수록 느리다")
    ap.add_argument("--zoom", type=float, default=900.0, help="버스를 따라갈 때의 확대율")
    ap.add_argument("--no-track", action="store_true", help="버스를 자동으로 따라가지 않는다")
    ap.add_argument("--no-overtake", action="store_true", help="느린 앞차를 피하는 선택적 차로 변경을 끈다")
    ap.add_argument("--probe", type=float, default=10.0,
                    help="옮겨 갈 차로 뒤에 차가 있으면, 이 간격(초)마다 일부러 그쪽으로 차로를 바꿔 상호작용을 만든다. 0이면 끈다")
    ap.add_argument("--no-companion", action="store_true", help="시험 차로 변경 때 뒤에 차를 따로 넣지 않는다(자연 교통만)")
    ap.add_argument("--no-stops", action="store_true", help="정류장에 서지 않는다")
    ap.add_argument("--out-dir", default=os.path.join(tempfile.gettempdir(), "hapjeong_scenario"))
    a = ap.parse_args()

    world = HapjeongTraffic(a.out_dir, scenario=a.scenario, seed=a.seed)
    world.build()
    print(f"합정역 {a.scenario} seed {a.seed}: 끼어들기 흐름 {world.summary['cutin_flows']}개, "
          f"S6 보행자 {world.summary['s6_persons']}명. 실행 폴더 {a.out_dir}")
    print("범례: 버스=빨강 | 노랑 Krauss, 청록 IDM, 주황 EIDM, 자홍 ACC, 회색 끼어들기 | "
          "고리 초록=양보 빨강=비양보")
    tc = world.start(warmup=a.warmup, gui=not a.nogui)

    m = load_map("hapjeong")
    loop = m["routes"]["bus_loop"]
    stops = [{"id": s["id"], "name": s["name"], "edge": s["lane"].rsplit("_", 1)[0], "startPos": s["startPos"],
              "lane_idx": int(s["lane"].rsplit("_", 1)[1])}
             for s in m["bus_stops"] if s["lane"].rsplit("_", 1)[0] in loop]
    bus = LoopBus(tc, loop, stops, with_stops=not a.no_stops, discretionary=not a.no_overtake, probe_s=a.probe, companions=not a.no_companion)
    state = {"rings": None, "tracked": False}

    def on_step(t):
        if state["rings"] is None:
            state["rings"] = Rings(tc, world.mgr)
        state["rings"].step(t)
        if not a.nogui:
            if not a.no_track and not state["tracked"] and BUS in tc.vehicle.getIDList():
                view = tc.gui.getIDList()[0]
                tc.gui.trackVehicle(view, BUS)
                tc.gui.setZoom(view, a.zoom)
                state["tracked"] = True
            time.sleep(a.delay / 1000.0)

    try:
        world.run(tc.simulation.getTime() + a.seconds, bus_id=BUS, bus_hook=bus.tick, on_step=on_step)
    except (traci.exceptions.FatalTraCIError, ConnectionError):
        print("창이 닫혀 시뮬레이션을 마칩니다.")
    finally:
        print("\n== 요약 ==")
        print(f"버스 완주 {bus.laps}바퀴" + (f", 바퀴당 평균 {sum(bus.lap_times) / len(bus.lap_times):.0f}초 "
              f"(경로 {m['route_design']['bus_loop']['length_m']:.0f}m)" if bus.lap_times else ""))
        print("버스 차로 변경 요청: " + (", ".join(f"{k} {v}회" for k, v in bus.changes.items()) or "없음"))
        if state["rings"]:
            print("버스를 마주친 차량의 판정(주행 모델별):\n" + state["rings"].summary())
        try:
            world.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
