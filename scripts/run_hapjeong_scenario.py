"""합정역 버스 순환 노선 시나리오: 직접 눈으로 확인하는 시뮬레이션.

    python scripts/run_hapjeong_scenario.py                         # sumo-gui가 열리고 버스를 따라간다
    python scripts/run_hapjeong_scenario.py --nogui --seconds 900   # 화면 없이 요약만

화면에서 보이는 것
  빨간 버스      : 시험용 버스(traffic/test_bus.py). R68에서 한 번 출발해 끊김 없이 노선을 돈다. 정해진 곳(s2 앞 S3, J82·J1 앞 S2)
                   에서만 방향지시등을 켜고 차선 변경을 요청한다(주행 정책이 아니라 규칙).
  주변 차량 색   : 주행 모델. 노랑 Krauss, 청록 IDM, 주황 EIDM, 자홍 ACC, 회색 끼어들기(골목 합류) 차량.
  버스를 마주한 차: 버스가 차로 변경 의사를 보이면 뒤차 주위에 고리가 생긴다.
                    초록 고리 = 양보형(버스보다 느려져 길을 터 줌), 빨간 고리 = 비양보형(버스보다 빨라져 막음).
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
from traffic.alleys import read_net  # noqa: E402
from traffic.hapjeong import HapjeongTraffic, stopped_from_arg  # noqa: E402
from traffic.sumo_conn import traci  # noqa: E402
from traffic.test_bus import BUS, TestBusPilot  # noqa: E402

MODEL_NAME = {"car_krauss": "Krauss", "car_idm": "IDM", "car_eidm": "EIDM", "car_acc": "ACC"}
YIELD_RGB, NOYIELD_RGB = (0, 190, 0, 255), (255, 0, 0, 255)


def map_stops(m):
    """지도의 정류장 정의를 시험용 버스 조종이 쓰는 모양으로."""
    return [{"id": s["id"], "name": s["name"], "edge": s["lane"].rsplit("_", 1)[0], "startPos": s["startPos"],
             "lane_idx": int(s["lane"].rsplit("_", 1)[1])} for s in m["bus_stops"]]


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
    ap.add_argument("--inject", action="store_true",
                    help="시나리오 주입기: 매 바퀴 S2·S3·S6과 S5 1~2곳의 상대를 넣는다")
    ap.add_argument("--stopped", action="append", default=[], metavar="SPEC",
                    help="S4 정차 차량(여러 번 줄 수 있다). 쉼표로 이은 key=value: kind=taxi|bus|car, "
                         "edge=R69,lane=1,pos=30 또는 stop=<정류장 id·이름>[,gap=14], depart=60(warmup 뒤 초), "
                         "duration=30(정차 시간 초), prob=1(발생 확률). 예) --stopped kind=taxi,stop=합정역7번출구,depart=60")
    ap.add_argument("--triggers", choices=("any", "scenario"), default="any",
                    help="A1이 버스의 차로 변경 의사를 읽는 조건. any=방향지시등만, scenario=S2(좌회전 직전)·S3(정류장 접근)·S4(정차 차량 앞) 구간만")
    ap.add_argument("--lane-buses", type=float, default=30.0,
                    help="배경 버스(전용 차로 노선, 정류장에 서고 출발) 시간당 대수. 0이면 배경 버스·택시를 넣지 않는다")
    ap.add_argument("--taxis", type=float, default=40.0, help="택시 시간당 대수")
    ap.add_argument("--no-stop-ahead", action="store_true",
                    help="시험용 버스가 정류장에 들어갈 때 앞의 택시·버스가 정류장 앞에 서는 동작을 끈다")
    ap.add_argument("--no-stop-bypass", action="store_true", help="S1(정차 버스 뒤 차량의 기다림·추월)을 끈다")
    ap.add_argument("--no-ped-defiance", action="store_true", help="A3 보강(신호 없는 횡단보도 보행자의 건너기 판정)을 끈다")
    ap.add_argument("--out-dir", default=os.path.join(tempfile.gettempdir(), "hapjeong_scenario"))
    a = ap.parse_args()

    stopped = [stopped_from_arg(x, a.warmup, i) for i, x in enumerate(a.stopped)]
    from traffic.bus_traffic import BusTrafficCfg
    bt = (BusTrafficCfg(lane_buses_per_h=a.lane_buses, taxis_per_h=a.taxis)
          if a.lane_buses > 0 else None)
    world = HapjeongTraffic(a.out_dir, scenario=a.scenario, seed=a.seed, stopped=stopped, bus_traffic=bt)
    world.build()
    print(f"합정역 {a.scenario} seed {a.seed}: 끼어들기 흐름 {world.summary['cutin_flows']}개"
          f"(합류 골목 {sum(1 for x in world.summary['cutin_alleys'] if x['flows'])}곳), "
          f"S6 보행자 {world.summary['s6_persons']}명, 정차 차량 {len(world.summary['stopped'])}대. 실행 폴더 {a.out_dir}")
    print("범례: 버스=빨강 | 노랑 Krauss, 청록 IDM, 주황 EIDM, 자홍 ACC, 회색 끼어들기 | "
          "고리 초록=양보 빨강=비양보")
    tc = world.start(warmup=a.warmup, gui=not a.nogui)

    m = load_map("hapjeong")
    net = read_net(os.path.join(a.out_dir, "network.net.xml"))
    bus = TestBusPilot(tc, net, m["routes"]["bus_loop"], map_stops(m))
    state = {"rings": None, "tracked": False, "laps": 0}

    def on_step(t):
        state["collisions"] = state.get("collisions", 0) + len(tc.simulation.getCollisions())
        state["teleports"] = state.get("teleports", 0) + len(tc.simulation.getStartingTeleportIDList())
        if state["rings"] is None:
            state["rings"] = Rings(tc, world.mgr)
        state["rings"].step(t)
        if len(bus.lap_times) > state["laps"]:
            state["laps"] = len(bus.lap_times)
            print(f"[{t:7.1f}s] 버스 {state['laps']}바퀴 완주 ({bus.lap_times[-1]:.0f}초)")
        if not a.nogui:
            if not a.no_track and not state["tracked"] and BUS in tc.vehicle.getIDList():
                view = tc.gui.getIDList()[0]
                tc.gui.trackVehicle(view, BUS)
                tc.gui.setZoom(view, a.zoom)
                state["tracked"] = True
            time.sleep(a.delay / 1000.0)

    try:
        world.run(tc.simulation.getTime() + a.seconds, bus_id=BUS, bus_hook=bus.tick, on_step=on_step,
                  stop_bypass=not a.no_stop_bypass, ped_defiance=not a.no_ped_defiance,
                  stop_ahead=not a.no_stop_ahead, scenario_inject=a.inject, pilot=bus,
                  triggers="scenario" if a.triggers == "scenario" else None)
    except (traci.exceptions.FatalTraCIError, ConnectionError):
        print("창이 닫혀 시뮬레이션을 마칩니다.")
    finally:
        print("\n== 요약 ==")
        laps = bus.lap_times
        print(f"버스 완주 {len(laps)}바퀴" + (f", 바퀴당 평균 {sum(laps) / len(laps):.0f}초" if laps else ""))
        print("버스 차로 변경 요청: " + (", ".join(f"{k} {v}회" for k, v in bus.changes.items()) or "없음"))
        print(f"창 끝까지 못 옮김(missed): {bus.missed}, 늦은 상대(late): {bus.late}, 속도 줄임: {bus.slowed}")
        if state["rings"]:
            print("버스를 마주친 차량의 판정(주행 모델별):\n" + state["rings"].summary())
        if getattr(world, "stop", None) is not None:
            print("S1 정차 버스 뒤 차량:", world.stop.summary())
        if getattr(world, "ped", None) is not None:
            print("A3 보행자 건너기 판정:", world.ped.summary())
        print("정차 차량(S4):", world.summary["stopped"])
        if getattr(world, "ahead", None) is not None:
            print("정류장 앞 정차(시험용 버스 진입 시):", world.ahead.summary())
        print(f"충돌 {state.get('collisions', 0)}건, 순간이동 {state.get('teleports', 0)}건")
        try:
            world.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
