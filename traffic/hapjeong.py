"""합정역 도로에 주변 교통(A1~A3, S4)을 얹는 연결 모듈.

도로는 공식 경로로만 만든다: maps/hapjeong(scripts/build_hapjeong.py가 만든 map.json과 도로망)을
env.road_config.configure_map이 읽고, env.road_builder.build가 실행 폴더(배경 교통 포함)를 만든다.
이 모듈은 그 실행 폴더에 다음을 더한다. 도로망과 maps/는 건드리지 않는다.

  A1  배경 차량(vType `car_*`, 추종 모델 4종)에 양보 성격을 투입 시 독립으로 붙인다(YieldManager type_mix).
  S1  차로 정차형 정류장에 선 버스 바로 뒤 차량이 기다리거나 추월한다(StopBypassManager).
  S5  버스 경로에 합류하는 골목을 지도에서 자동으로 찾아(traffic.alleys), 그 골목으로 나오는 흐름을 끼어들기형 vType으로
      바꾼다. 골목에서 출발하는 흐름과, 고리 골목을 거쳐(via) 나오는 흐름이 둘 다 대상이다.
  S6  와우산로 횡단보도를 건너는 보행자를 P1(대기형)/P2(진입형)로 나눈다. 신호 없는 횡단보도 앞 보행자의 건너기 판정은
      PedDefianceManager가 한다.
  S4  (선택) 정차 차량. 택시·버스·일반 차량, 위치·시각·정차 시간·확률을 StoppedSpec으로 받는다.

버스는 이 모듈이 만들지 않는다. 버스 제어 쪽이 `bus_hook(traci, t)`로 버스를 넣고 몰면 된다.
"""
import json
import os
import random
import xml.etree.ElementTree as ET

from env import road_builder
from maps import load_map
from env.road_config import ROAD, EGO, TRAFFIC, configure_map

from .alleys import close_loop, find_merge_alleys, flow_merges, read_net
from .common import AGGRESSION, Ownership
from .cutin import STOPPED_KINDS, StoppedSpec, cutin_bins_once, cutin_vtype_xml, stopped_vehicle_xml
from .ped_defiance import PedDefianceManager
from .pedestrian import ped_vtypes_xml
from .runner import run as run_loop
from .sumo_conn import traci, sumo_cmd
from .bus_traffic import BusTrafficCfg, build_bus_traffic
from .interior_spawner import InteriorSpawner
from .person_spawner import PersonSpawner, extract_persons
from .presence import VehicleArrivalGuard
from .scenario_injector import ScenarioInjector
from .right_turn import apply_right_on_red
from .signal_areas import clip_zones, unsignalized_alleys, unsignalized_crossings
from .test_bus import STOP_AT
from .stop_ahead import StopAheadManager
from .stop_bypass import StopBypassManager
from .yield_logic import YieldManager, DEFAULT_TYPES, Trigger, Zone

MAP_ID = "hapjeong"
S6_EDGES = ("R112", "-R112")           # 와우산로. 이 둘을 모두 걷는 보행자가 횡단보도를 건넌다
BUS_VTYPE = ('<vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" '
             'tau="1.0" sigma="0" guiShape="bus" color="255,0,0"/>')   # 버스는 빨간색


class _TimeAdapter:
    """실행 루프의 t는 0부터 세지만 보행자 출발 시각은 SUMO 시각이다. SUMO 시각으로 바꿔 넘긴다."""

    def __init__(self, inner):
        self.inner = inner

    def update(self, t):
        self.inner.update(traci.simulation.getTime())


def _fragment(xml_text):
    return list(ET.fromstring(f"<x>{xml_text}</x>"))


def _krauss_attrs():
    """끼어들기 차량이 배경 차량과 같은 물리 특성을 갖게 env의 Krauss 설정을 쓴다."""
    c = TRAFFIC["controllers"]["krauss"]
    return (f'carFollowModel="Krauss" accel="{c["accel"]}" decel="{c["decel"]}" tau="{c["tau"]}" '
            f'minGap="{c["min_gap"]}" sigma="{c["sigma"]}" speedDev="{c["speed_dev"]}" '
            f'maxSpeed="{TRAFFIC["max_speed"]}" vClass="passenger" length="4.5" color="0.55,0.55,0.55"')   # 끼어들기 차량은 회색


def stopped_from_arg(arg, warmup=0.0, index=0):
    """명령행 문자열 -> StoppedSpec. 쉼표로 이은 key=value 목록이다.

      kind=taxi|bus|car  edge=R69 lane=1 pos=30   정차 위치를 직접 준다
      stop=<정류장 id 또는 이름>  gap=14            그 정류장 바로 앞(gap m 앞, 같은 차로)에 선다
      depart=60 duration=30 prob=1                  투입 시각(warmup 끝 기준 초), 정차 시간, 발생 확률
    """
    kv = dict(item.split("=", 1) for item in arg.split(","))
    spec = StoppedSpec(kv.get("kind", "taxi"), kv.get("edge", ""), int(kv.get("lane", 0)), float(kv.get("pos", 0)),
                       warmup + float(kv.get("depart", 0)), float(kv.get("duration", 30)), float(kv.get("prob", 1)),
                       kv.get("id") or f"stopped{index}")
    if "stop" in kv:
        stop = next((s for s in load_map(MAP_ID)["bus_stops"] if kv["stop"] in (s["id"], s["name"])), None)
        if stop is None:
            raise ValueError(f"정류장을 찾지 못했다: {kv['stop']}")
        spec.edge, _, lane = stop["lane"].rpartition("_")
        spec.lane = int(lane)
        area = max(10.0, STOPPED_KINDS[spec.kind][2] + 4.0)             # stopped_vehicle_xml이 만드는 정차 구역 길이
        spec.pos = max(0.0, stop["startPos"] - float(kv.get("gap", 14)) - area)   # 구역이 정류장 gap m 앞에서 끝나게
    if not spec.edge:
        raise ValueError("edge=와 pos=를 주거나 stop=을 줘야 한다")
    return spec


class HapjeongTraffic:
    """합정역 실행 폴더를 만들고, 주변 교통 규칙을 붙여 시뮬레이션을 돌린다.

    yield_mix: A1 양보 성격 가중치. None이면 A1을 쓰지 않는다.
    cutin: True면 S5 끼어들기. cutin_bins를 안 주면 cutin.cutin_bins_once()(진입 판정이 차량당 한 번)를 쓴다.
           cutin.default_bins()를 주면 SUMO 기본 방식(접근 중 여러 번 뽑음)이다.
    ped_mix: S6 보행자 P1 비율(0~1). None이면 보행자를 그대로 둔다.
    stopped: [StoppedSpec] S4 정차 차량 목록(택시·버스·일반 차량). None이면 없다.
    taxi_depart: 예전 방식. 숫자면 그 시각(초)에 R69 안쪽 차로 정류장 앞에 택시 한 대가 선다(StoppedSpec 하나와 같다).
    merge_route: S5에서 '합류'의 기준이 되는 버스 경로(도로 id 목록). None이면 지도의 bus_loop.
    """

    def __init__(self, out_dir, scenario="baseline", seed=0, *, yield_mix=None, cutin=True,
                 cutin_bins=None, ped_mix=0.5, ped_v_low=1.5, taxi_depart=None, stopped=None, merge_route=None,
                 bus_traffic=None, right_on_red=True, right_on_red_exclude=("J1",),
                 interior_share=0.3, interior_min_dist=200.0):
        # interior_share: 입구 배경 차량 흐름 중 이 비율을 노선 위 임의의 지점에서 출발시킨다(0이면 끈다). 시험용 버스에서
        # interior_min_dist m 안에서는 투입하지 않아 눈앞에 갑자기 생기지 않는다.
        self.interior_share, self.interior_min_dist = interior_share, interior_min_dist
        # right_on_red: True면 신호 교차로의 우회전이 신호와 무관하게 간다. 단 우회전 차가 지나는 횡단보도의 보행자 신호가
        # 적색이고 겹치는 직진·좌회전이 녹색이 아닌 위상에서만 열린다(신호 프로그램만 바꾼다). exclude는 합정역 사거리.
        self.right_on_red, self.right_on_red_exclude = right_on_red, tuple(right_on_red_exclude)
        self.interior, self.interior_per_h = None, 0.0
        # bus_traffic: BusTrafficCfg 또는 True(기본 설정). 배경 버스(전용 차로 노선·통과)와 택시를 투입한다. None이면 없다.
        self.bus_traffic = BusTrafficCfg() if bus_traffic is True else bus_traffic
        self.out_dir = os.path.abspath(out_dir)
        self.scenario, self.seed = scenario, seed
        self.yield_mix = yield_mix if yield_mix is not None else {"yielder": 0.5, "nonyielder": 0.5}
        self.cutin, self.cutin_bins = cutin, cutin_bins
        self.ped_mix, self.ped_v_low, self.taxi_depart = ped_mix, ped_v_low, taxi_depart
        self.stopped = list(stopped or [])
        if taxi_depart is not None:
            self.stopped.append(StoppedSpec("taxi", "R69", 1, 30.0, float(taxi_depart), 30.0, 1.0, "taxi0"))
        self.merge_route = merge_route
        self.cfg = None
        self.summary = {}

    # ---- 실행 폴더 만들기 -------------------------------------------------
    def build(self):
        configure_map(self.scenario, self.seed, None, MAP_ID)
        cfg = road_builder.build(ROAD, EGO, TRAFFIC, self.out_dir)
        rng = random.Random(self.seed)
        self.summary = {"cutin_flows": 0, "cutin_alleys": [], "s6_persons": 0, "stopped": []}

        overlay_routes = [BUS_VTYPE]              # 다른 경로 파일보다 먼저 읽혀야 한다
        overlay_add = []
        if self.cutin:
            bins = self.cutin_bins or cutin_bins_once()
            overlay_routes.append(cutin_vtype_xml("cutin", bins=bins, v_ign=15.0, base_attrs=_krauss_attrs()))
            self.summary["cutin_flows"] = self._retype_alley_flows()
            self.cutin_aggr = self._cutin_aggressiveness(bins)
            self.cutin_full_ignore = next((f"cutin_{i}" for i, b in enumerate(bins) if b[0] >= 1.0), "cutin_0")
        if self.ped_mix is not None:
            overlay_routes.append(ped_vtypes_xml(self.ped_v_low, 0.75))
            self.summary["s6_persons"] = self._retype_s6_persons(rng)
        # 보행자는 경로 파일에서 떼어 PersonSpawner가 시험용 버스 반경 밖에서만 넣는다(버스 옆에서 갑자기 생기지 않게)
        persons = extract_persons(os.path.join(self.out_dir, "pedestrians.rou.xml"))
        with open(os.path.join(self.out_dir, "persons.json"), "w", encoding="utf-8") as f:
            json.dump(persons, f, ensure_ascii=False)
        self.summary["persons"] = len(persons)
        self.interior_per_h = 0.0
        if self.interior_share > 0:
            self.interior_per_h = self._shift_entry_flows(self.interior_share)
            self.summary["interior"] = {"share": self.interior_share, "per_h": round(self.interior_per_h, 1)}
        vtypes_seen = set()
        for i, spec in enumerate(self.stopped):
            if rng.random() >= spec.prob:             # 이번 실행에서는 이 정차가 일어나지 않는다
                continue
            vts, vehs, pa = stopped_vehicle_xml(spec, i, car_attrs=_krauss_attrs().rsplit(' color=', 1)[0])
            for vt in vts:
                if vt not in vtypes_seen:
                    vtypes_seen.add(vt)
                    overlay_routes.append(vt)
            overlay_routes.extend(vehs)
            overlay_add.append(pa)
            self.summary["stopped"].append({"id": spec.sid or f"stopped{i}", "kind": spec.kind, "edge": spec.edge,
                                            "lane": spec.lane, "pos": spec.pos, "depart": spec.depart,
                                            "duration": spec.duration})

        if self.bus_traffic is not None:
            m = load_map(MAP_ID)
            net = read_net(os.path.join(self.out_dir, "network.net.xml"))
            parts, bsum = build_bus_traffic(net, m["flows"], m["routes"]["bus_loop"],
                                            [s["id"] for s in m["bus_stops"]], self.bus_traffic,
                                            random.Random(self.seed + 5))
            overlay_routes.extend(parts)
            self.summary["bus_traffic"] = bsum

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
        """버스 경로에 합류하는 골목(지도에서 자동 탐지)으로 나오는 흐름을 모두 끼어들기 vType으로 바꾼다."""
        net = read_net(os.path.join(self.out_dir, "network.net.xml"))
        route = close_loop(net, self.merge_route or load_map(MAP_ID)["routes"]["bus_loop"])
        # 신호 교차로로 합류하는 골목(cluster_J52_J56_J58, J74, J48)은 신호가 순서를 정해 끼어들기가 없으므로 뺀다
        alleys = unsignalized_alleys(find_merge_alleys(net, route))
        path = os.path.join(self.out_dir, "routes.rou.xml")
        tree = ET.parse(path)
        per_alley = {a.junction: 0 for a in alleys}
        n = 0
        for flow in tree.getroot().iter("flow"):
            hit = [a for a in alleys if flow_merges(net, flow.attrib, [a])]
            if hit:
                flow.set("type", "cutin")
                n += 1
                for a in hit:
                    per_alley[a.junction] += 1
        tree.write(path, encoding="utf-8", xml_declaration=True)
        self.alleys = alleys
        self.summary["cutin_alleys"] = [
            {"junction": a.junction, "feeder": a.feeder, "into": a.into, "junction_type": a.junction_type,
             "minor": a.minor, "regulated": a.regulated, "flows": per_alley[a.junction]} for a in alleys]
        return n

    def _cutin_aggressiveness(self, bins):
        """끼어들기 vType(구간 i)의 공격성 값. 구간의 무시 확률 p_ignore에서 거꾸로 구한다."""
        return {f"cutin_{i}": (b[4] if len(b) > 4 else min(1.0, max(0.0, AGGRESSION.aggressiveness_from_ignore(b[0]))))
                for i, b in enumerate(bins)}

    def _shift_entry_flows(self, share):
        """입구에서 들어오는 배경 차량(carDist) 흐름을 share 비율만큼 줄이고, 그 대수(대/시간)를 돌려준다.
        줄인 만큼은 InteriorSpawner가 노선 위 임의의 지점에서 넣어 전체 대수를 지킨다."""
        path = os.path.join(self.out_dir, "routes.rou.xml")
        tree = ET.parse(path)
        moved = 0.0
        for flow in tree.getroot().iter("flow"):
            if flow.get("type") != "carDist":
                continue
            period = flow.get("period") or ""
            if period.startswith("exp("):
                rate = float(period[4:-1])                       # 대/초
                flow.set("period", f"exp({rate * (1 - share):.6f})")
                moved += rate * share * 3600.0
            elif period:
                p = float(period)
                flow.set("period", f"{p / (1 - share):.6f}")
                moved += 3600.0 / p * share
        tree.write(path, encoding="utf-8", xml_declaration=True)
        return moved

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
        if self.right_on_red:      # 신호와 무관한 우회전(횡단보도에 사람이 없을 때). 합정역 사거리(J1)는 제외
            net = read_net(os.path.join(self.out_dir, "network.net.xml"))
            self.summary["right_on_red"] = apply_right_on_red(traci, net, exclude=self.right_on_red_exclude)
        self.interior = self._make_interior("bus0")
        self.persons = self._make_person_spawner("bus0")
        for k in range(int(round(warmup / 0.1))):
            if self.persons is not None:
                self.persons.update(traci.simulation.getTime())
            if self.interior is not None:
                self.interior.update(k * 0.1)      # 예열 중에도 노선 위에서 출발시켜 시작 때부터 밀도를 맞춘다
            traci.simulationStep()
        return traci

    def _make_person_spawner(self, bus_id):
        path = os.path.join(self.out_dir, "persons.json")
        if not os.path.exists(path):
            return None
        with open(path, encoding="utf-8") as f:
            persons = json.load(f)
        net = read_net(os.path.join(self.out_dir, "network.net.xml"))
        return PersonSpawner(traci, net, bus_id, persons)

    def _make_interior(self, bus_id):
        if self.interior_share <= 0 or self.interior_per_h <= 0:
            return None
        m = load_map(MAP_ID)
        net = read_net(os.path.join(self.out_dir, "network.net.xml"))
        return InteriorSpawner(traci, net, bus_id, m["routes"]["bus_loop"], sorted({f["to"] for f in m["flows"]}),
                               self.interior_per_h, rng=random.Random(self.seed + 6), min_dist=self.interior_min_dist)

    def scenario_triggers(self, approach=120.0, loop=None, plans=None):
        """A1 진입 의사 구간: S2(좌회전 직전, 좌측), S3(정류장 접근, 우측), S4(정차 차량 앞, 양쪽).

        plans(test_bus.lane_plans)를 주면 S2·S3 구간은 시험용 버스의 차선 변경 창과 같다(J1 앞은 차로를 3~4칸 옮기므로
        창이 240 m로 길어 120 m 구간으로는 첫 변경이 빠졌다). 주지 않으면 지도의 버스 경로(loop)에서 읽는다: 다음 도로로 가는
        연결이 좌회전이면 앞 도로의 끝 approach m가 S2 구간, 정류장 앞 approach m가 S3 구간이다.
        """
        m = load_map(MAP_ID)
        route = loop or m["routes"]["bus_loop"]
        net = read_net(os.path.join(self.out_dir, "network.net.xml"))
        out = []
        route = close_loop(net, route)      # 노선 끝과 처음 사이에 시뮬레이션이 잇는 도로(R42 -> R68 -> R69)를 포함한다

        def zones_before(k, pos, length=approach):
            """경로의 k번째 도로 pos 앞 length m를 도로 경계를 넘어 거슬러 올라가며 구간으로 만든다.
            짧은 도로(R111b 27 m)가 구간이면 버스가 방향지시등을 켜는 앞 도로(R111)가 빠져 A1이 켜지지 않았다."""
            zs, left_m = [], length
            while left_m > 0 and k >= 0 and net.hasEdge(route[k]):
                lo = max(0.0, pos - left_m)
                zs.append(Zone(route[k], lo, pos))
                left_m -= pos - lo
                k -= 1
                if k >= 0 and net.hasEdge(route[k]):
                    pos = net.getEdge(route[k]).getLength()
            return zs

        left, right = [], []
        if plans is not None:
            for p in plans:
                if p.edge in route:
                    zs = zones_before(route.index(p.edge), p.end_pos, p.length)
                    (left if p.kind == "S2" else right).extend(zs)
        else:
            for k, (a, b) in enumerate(zip(route, route[1:])):
                if not (net.hasEdge(a) and net.hasEdge(b)):
                    continue
                conns = net.getEdge(a).getConnections(net.getEdge(b))
                if conns and all(c.getDirection() in ("l", "L") for c in conns):
                    left += zones_before(k, net.getEdge(a).getLength())
            for s in m["bus_stops"]:
                edge = s["lane"].rsplit("_", 1)[0]
                if edge in route:
                    right += zones_before(route.index(edge), s["startPos"])
        left = clip_zones(net, route, left)           # 신호 교차로 정지선 앞(기본 20 m, J1 10 m)에서는 A1을 켜지 않는다
        if left:
            out.append(Trigger(+1, left))
        right = clip_zones(net, route, right)
        if right:
            out.append(Trigger(-1, right))
        for item in self.summary.get("stopped", []):
            zs = clip_zones(net, route, [Zone(item["edge"], max(0.0, item["pos"] - approach), item["pos"])])
            if zs:
                out += [Trigger(+1, zs), Trigger(-1, zs)]
        return out

    def run(self, t_end, bus_id="bus0", bus_hook=None, metrics=None, yield_types=None, on_step=None,
            assign_existing=True, ped_defiance=True, stop_bypass=True, triggers=None, stop_ahead=True,
            ped_cfg=None, scenario_inject=False, pilot=None, **mgr_kwargs):
        """warmup 뒤 t_end초 동안 공통 실행 루프(runner.run)로 돌린다. 반환: (YieldManager, 마지막 시각).

        triggers: A1 진입 의사 구간. None이면 방향지시등만 본다. "scenario"면 scenario_triggers()를 쓴다.
        stop_bypass: S1(정차 버스 뒤 차량)을 켠다. 한 차량은 한 순간에 하나의 추가 행동에만 속한다(공유 소유권).
        """
        self.owner = Ownership()
        if triggers == "scenario":
            triggers = self.scenario_triggers(plans=pilot.plans if pilot is not None else None)
        mgr = YieldManager(traci, bus_id, rng=random.Random(self.seed), types=yield_types or DEFAULT_TYPES,
                           type_mix=self.yield_mix, mix_vtype_prefixes=("car_",), owner=self.owner, triggers=triggers,
                           vtype_aggr=getattr(self, "cutin_aggr", None),
                           **{"follower_range": 50.0, **mgr_kwargs})
        self.mgr = mgr
        if assign_existing:        # warmup 중에 투입된 차량에도 성격을 준다
            mgr.on_departed(traci.vehicle.getIDList())
        # A3는 신호 없는 횡단보도에서만 판정한다(신호 횡단보도는 보행자 신호가 순서를 정한다)
        crossings = (unsignalized_crossings(read_net(os.path.join(self.out_dir, "network.net.xml")),
                                            traci.lane.getIDList()) if ped_defiance else None)
        self.ped = (PedDefianceManager(traci, bus_id, rng=random.Random(self.seed + 2), cfg=ped_cfg,
                                       crossings=crossings)
                    if ped_defiance else None)
        if metrics is not None and self.ped is not None and getattr(metrics, "ped_mgr", None) is None:
            metrics.ped_mgr = self.ped        # 보행자가 무시하고 건널 때의 급감속·비상 감속을 센다
        self.stop = (StopBypassManager(traci, bus_id, rng=random.Random(self.seed + 3), owner=self.owner)
                     if stop_bypass else None)
        self.ahead = None
        if stop_ahead and pilot is not None:      # S4: 골목에서 나온 차가 정류장 근처에 선다(바퀴마다 1~2곳)
            m = load_map(MAP_ID)
            stops = [{"id": s["id"], "edge": s["lane"].rsplit("_", 1)[0], "lane": int(s["lane"].rsplit("_", 1)[1]),
                      "startPos": s["startPos"], "endPos": s["endPos"]}
                     for s in m["bus_stops"] if s["lane"].rsplit("_", 1)[0] in m["routes"]["bus_loop"]]
            self.ahead = StopAheadManager(traci, bus_id, read_net(os.path.join(self.out_dir, "network.net.xml")),
                                          pilot.lap_edges, stops, self.alleys, lap_fn=lambda: pilot.lap,
                                          rng=random.Random(self.seed + 4), owner=self.owner)
        extras = [self.ahead] if self.ahead else []
        self.injector = None
        if scenario_inject and pilot is not None:   # 방법 2: 시나리오 주입기(바퀴마다 S2·S3·S5·S6 상대를 계획해 넣는다)
            m = load_map(MAP_ID)
            net = read_net(os.path.join(self.out_dir, "network.net.xml"))
            self.injector = ScenarioInjector(
                traci, net, bus_id,
                [{"id": s["id"], "edge": s["lane"].rsplit("_", 1)[0], "startPos": s["startPos"], "endPos": s["endPos"]}
                 for s in m["bus_stops"] if s["lane"].rsplit("_", 1)[0] in m["routes"]["bus_loop"]],
                self.alleys, getattr(self, "cutin_full_ignore", "cutin_0"),
                lap_fn=lambda: pilot.lap, plans=pilot.plans, rng=random.Random(self.seed + 8),
                a1_active=lambda vid: vid in mgr.inter, dwell_s=pilot.stop_s, stop_ids=STOP_AT)
            pilot.follower_ready = self.injector.follower_ready     # 상대가 늦으면 시험용 버스가 속도를 줄인다
            extras.append(self.injector)
        if getattr(self, "persons", None) is not None:
            self.persons.bus = bus_id
            extras.insert(0, _TimeAdapter(self.persons))
        # 지도 안쪽에서 끝나는 경로의 차가 시험용 버스 가까이에서 사라지지 않게 목적지를 지도 경계로 늘린다
        self.arrival_guard = VehicleArrivalGuard(traci, read_net(os.path.join(self.out_dir, "network.net.xml")), bus_id)
        extras.append(self.arrival_guard)
        if self.interior is None:
            self.interior = self._make_interior(bus_id)
        if self.interior is not None:
            self.interior.bus = bus_id
            extras.append(self.interior)
        t = run_loop(traci, t_end, bus_id, bus_hook=bus_hook, yield_mgr=mgr, metrics=metrics,
                     on_step=on_step, ped_mgr=self.ped, stop_mgr=self.stop, extras=extras)
        return mgr, t

    def close(self):
        if traci.isLoaded():
            traci.close()
