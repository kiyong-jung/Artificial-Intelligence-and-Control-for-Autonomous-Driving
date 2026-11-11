"""합정역 도로에 주변 교통(A1~A3, S4)을 얹는 연결 모듈.

도로는 공식 경로로만 만든다: maps/hapjeong(scripts/build_hapjeong.py가 만든 map.json과 도로망)을
env.road_config.configure_map이 읽고, env.road_builder.build가 실행 폴더(배경 교통 포함)를 만든다.
이 모듈은 그 실행 폴더에 다음을 더한다. 도로망과 maps/는 건드리지 않는다.

  A1  배경 차량(vType `car_*`, 추종 모델 4종)에 양보 성격을 투입 시 독립으로 붙인다(YieldManager type_mix).
  S5  독막로에 신호 없이 합류하는 골목(G3, G12)에서 나오는 흐름을 끼어들기형 vType으로 바꾼다.
  S6  와우산로 횡단보도를 건너는 보행자를 P1(대기형)/P2(진입형)로 나눈다.
  S4  (선택) 가로변 정류장 앞에 서는 택시.

버스는 이 모듈이 만들지 않는다. 버스 제어 쪽이 `bus_hook(traci, t)`로 버스를 넣고 몰면 된다.
"""
import os
import random
import xml.etree.ElementTree as ET

from env import road_builder
from env.road_config import ROAD, EGO, TRAFFIC, configure_map

from .cutin import cutin_vtype_xml, default_bins, parking_area_xml
from .pedestrian import ped_vtypes_xml
from .runner import run as run_loop
from .sumo_conn import traci, sumo_cmd
from .yield_logic import YieldManager, DEFAULT_TYPES

MAP_ID = "hapjeong"
MERGE_ALLEYS = ("G3", "G12")           # 독막로에 신호 없이 합류하는 골목 진입 도로(X_dongmak3, X_dongmak6)
S6_EDGES = ("R112", "-R112")           # 와우산로. 이 둘을 모두 걷는 보행자가 횡단보도를 건넌다
BUS_VTYPE = ('<vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" '
             'tau="1.0" sigma="0" guiShape="bus" color="255,0,0"/>')   # 버스는 빨간색


def _fragment(xml_text):
    return list(ET.fromstring(f"<x>{xml_text}</x>"))


def _krauss_attrs():
    """끼어들기 차량이 배경 차량과 같은 물리 특성을 갖게 env의 Krauss 설정을 쓴다."""
    c = TRAFFIC["controllers"]["krauss"]
    return (f'carFollowModel="Krauss" accel="{c["accel"]}" decel="{c["decel"]}" tau="{c["tau"]}" '
            f'minGap="{c["min_gap"]}" sigma="{c["sigma"]}" speedDev="{c["speed_dev"]}" '
            f'maxSpeed="{TRAFFIC["max_speed"]}" vClass="passenger" length="4.5" color="0.55,0.55,0.55"')   # 끼어들기 차량은 회색


class HapjeongTraffic:
    """합정역 실행 폴더를 만들고, 주변 교통 규칙을 붙여 시뮬레이션을 돌린다.

    yield_mix: A1 양보 성격 가중치. None이면 A1을 쓰지 않는다.
    cutin: True면 S5 끼어들기. cutin_bins는 cutin.default_bins() 형식(자리표시 분포)이다.
    ped_mix: S6 보행자 P1 비율(0~1). None이면 보행자를 그대로 둔다.
    taxi_depart: 숫자면 그 시각(초)에 R69 안쪽 차로 정류장 앞에 택시 한 대가 선다(S4).
    """

    def __init__(self, out_dir, scenario="baseline", seed=0, *, yield_mix=None, cutin=True,
                 cutin_bins=None, ped_mix=0.5, ped_v_low=1.5, taxi_depart=None):
        self.out_dir = os.path.abspath(out_dir)
        self.scenario, self.seed = scenario, seed
        self.yield_mix = yield_mix if yield_mix is not None else {"yielder": 0.5, "nonyielder": 0.5}
        self.cutin, self.cutin_bins = cutin, cutin_bins
        self.ped_mix, self.ped_v_low, self.taxi_depart = ped_mix, ped_v_low, taxi_depart
        self.cfg = None
        self.summary = {}

    # ---- 실행 폴더 만들기 -------------------------------------------------
    def build(self):
        configure_map(self.scenario, self.seed, None, MAP_ID)
        cfg = road_builder.build(ROAD, EGO, TRAFFIC, self.out_dir)
        rng = random.Random(self.seed)
        self.summary = {"cutin_flows": 0, "s6_persons": 0, "taxi": self.taxi_depart is not None}

        overlay_routes = [BUS_VTYPE]              # 다른 경로 파일보다 먼저 읽혀야 한다
        overlay_add = []
        if self.cutin:
            bins = self.cutin_bins or default_bins()
            overlay_routes.append(cutin_vtype_xml("cutin", bins=bins, v_ign=15.0, base_attrs=_krauss_attrs()))
            self.summary["cutin_flows"] = self._retype_alley_flows()
        if self.ped_mix is not None:
            overlay_routes.append(ped_vtypes_xml(self.ped_v_low, 0.75))
            self.summary["s6_persons"] = self._retype_s6_persons(rng)
        if self.taxi_depart is not None:
            overlay_routes.append('<vType id="taxi" vClass="taxi" length="4.5" accel="2.6" decel="4.5" '
                                  'tau="1.0" sigma="0"/><route id="taxi_r" edges="R69"/>'
                                  f'<vehicle id="taxi0" type="taxi" route="taxi_r" depart="{self.taxi_depart}" '
                                  'departLane="1" departSpeed="max"><stop parkingArea="pa_taxi" duration="30"/></vehicle>')
            overlay_add.append(parking_area_xml("pa_taxi", "R69_1", 30, 40, 1))

        self._write("traffic.rou.xml", "routes", overlay_routes)
        self._write("traffic.add.xml", "additional", overlay_add)
        self._register(cfg)
        self.cfg = cfg
        return cfg

    def _write(self, name, root, parts):
        r = ET.Element(root)
        for part in parts:
            r.extend(_fragment(part))
        ET.ElementTree(r).write(os.path.join(self.out_dir, name), encoding="utf-8", xml_declaration=True)

    def _register(self, cfg):
        tree = ET.parse(cfg)
        inputs = tree.getroot().find("input")
        routes = inputs.find("route-files")
        routes.set("value", ",".join(["traffic.rou.xml", *routes.get("value").split(",")]))
        add = inputs.find("additional-files")
        add.set("value", ",".join([*add.get("value").split(","), "traffic.add.xml"]))
        tree.write(cfg, encoding="utf-8", xml_declaration=True)

    def _retype_alley_flows(self):
        path = os.path.join(self.out_dir, "routes.rou.xml")
        tree = ET.parse(path)
        n = 0
        for flow in tree.getroot().iter("flow"):
            if flow.get("from") in MERGE_ALLEYS:
                flow.set("type", "cutin")
                n += 1
        tree.write(path, encoding="utf-8", xml_declaration=True)
        return n

    def _retype_s6_persons(self, rng):
        path = os.path.join(self.out_dir, "pedestrians.rou.xml")
        tree = ET.parse(path)
        n = 0
        for person in tree.getroot().iter("person"):
            edges = {e for walk in person.iter("walk") for e in (walk.get("edges") or "").split()}
            if all(e in edges for e in S6_EDGES):
                person.set("type", "ped_p1" if rng.random() < self.ped_mix else "ped_p2")
                n += 1
        tree.write(path, encoding="utf-8", xml_declaration=True)
        return n

    # ---- 실행 --------------------------------------------------------------
    def start(self, warmup=180.0, gui=False):
        """SUMO를 띄우고 배경 교통을 warmup초 흘린다(env와 같은 180초가 기본).

        gui=True면 sumo-gui로 띄운다(재생 속도는 호출 쪽이 on_step에서 조절한다).
        """
        if self.cfg is None:
            self.build()
        extra = ["--start", "--delay", "0"] if gui else None
        cmd = sumo_cmd(self.cfg, seed=self.seed, extra=extra, binary="sumo-gui" if gui else "sumo")
        traci.load(cmd[1:]) if traci.isLoaded() else traci.start(cmd)
        for _ in range(int(round(warmup / 0.1))):
            traci.simulationStep()
        return traci

    def run(self, t_end, bus_id="bus0", bus_hook=None, metrics=None, yield_types=None, on_step=None,
            assign_existing=True, **mgr_kwargs):
        """warmup 뒤 t_end초 동안 공통 실행 루프(runner.run)로 돌린다. 반환: (YieldManager, 마지막 시각)."""
        mgr = YieldManager(traci, bus_id, rng=random.Random(self.seed), types=yield_types or DEFAULT_TYPES,
                           type_mix=self.yield_mix, mix_vtype_prefixes=("car_",), **{"follower_range": 50.0, **mgr_kwargs})
        self.mgr = mgr
        if assign_existing:        # warmup 중에 투입된 차량에도 성격을 준다
            mgr.on_departed(traci.vehicle.getIDList())
        t = run_loop(traci, t_end, bus_id, bus_hook=bus_hook, yield_mgr=mgr, metrics=metrics,
                     on_step=on_step)
        return mgr, t

    def close(self):
        if traci.isLoaded():
            traci.close()
