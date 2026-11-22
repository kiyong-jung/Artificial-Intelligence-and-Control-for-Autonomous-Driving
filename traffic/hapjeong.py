"""합정역 도로에 주변 교통(A1~A3, S4)을 얹는 연결 모듈.

도로는 공식 경로로만 만든다: maps/hapjeong(scripts/build_hapjeong.py가 만든 map.json과 도로망)을
env.road_config.configure_map이 읽고, env.road_builder.build가 실행 폴더(배경 교통 포함)를 만든다.
이 모듈은 그 실행 폴더에 다음을 더한다. 도로망과 maps/는 건드리지 않는다.

  A1  배경 차량(vType `car_*`, 추종 모델 4종)이 투입될 때 운전자 성향(차선 변경 파라미터, speedFactor)을 연속 분포에서
      뽑아 준다. 버스가 차로를 바꾸려 할 때 목표 차로 후행차가 한 일을 기록한다(YieldManager, 조종하지 않는다).
  S1  차로 정차형 정류장에 선 버스 바로 뒤 차량이 기다렸는지 추월했는지 기록한다(StopBypassManager, 조종하지 않는다).
  S5  도로망의 신호 없는 골목 출구를 도로 종류로 모두 찾아(traffic.alleys), 그 출구로 나오는 흐름을 골목 운전자
      분포(cutin, impatience·jmTimegapMinor 구간, 앞차 무시 없음)로 바꾼다. 버스 경로는 보지 않는다. 골목에서 출발하는
      흐름과, 고리 골목을 거쳐(via) 나오는 흐름이 둘 다 대상이다.
  S6  모든 보행자에게 보행자 분포(pedDist: P1·P2를 impatience 구간으로 넓힌 것)를 준다. 보행자는 경로 파일대로 SUMO가
      넣고 지운다.
  S4  bus_traffic을 주면 도로망의 가로변 정류장 근처에 서는 포아송 택시(bus_traffic.curb_taxi_stops)가 들어간다. 그 밖에
      (선택) 위치·시각·정차 시간·확률을 정한 정차 차량(택시·버스·일반 차량)을 StoppedSpec으로 받는다.

버스는 이 모듈이 만들지 않는다. 버스 제어 쪽이 `bus_hook(traci, t)`로 버스를 넣고 몰면 된다.
"""
import os
import random
import xml.etree.ElementTree as ET

from env import road_builder
from maps import load_map
from env.road_config import ROAD, EGO, TRAFFIC, configure_map

from .alleys import close_loop, find_alley_exits, find_merge_alleys, flow_merges, read_net
from .cutin import STOPPED_KINDS, StoppedSpec, alley_driver_bins, cutin_vtype_xml, stopped_vehicle_xml
from .pedestrian import PED_DIST, ped_dist_xml
from .runner import run as run_loop
from .sumo_conn import traci, sumo_cmd
from .bus_traffic import BusTrafficCfg, build_bus_traffic, curb_taxi_stops
from .lane_route import LaneRouteAdapter
from .right_turn import apply_right_on_red
from .signal_areas import clip_zones, unsignalized_alleys
from .stop_bypass import StopBypassManager
from .yield_logic import YieldManager, Trigger, Zone

MAP_ID = "hapjeong"
BUS_VTYPE = ('<vType id="bus_test" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" '
             'tau="1.0" sigma="0" guiShape="bus" color="255,0,0"/>')   # 버스는 빨간색


def _fragment(xml_text):
    return list(ET.fromstring(f"<x>{xml_text}</x>"))


def _krauss_attrs():
    """끼어들기 차량이 배경 차량과 같은 물리 특성을 갖게 env의 Krauss 설정과 횡방향(SL2015) 설정을 쓴다."""
    c, lat = TRAFFIC["controllers"]["krauss"], TRAFFIC["lateral_model"]
    lateral = (f'laneChangeModel="SL2015" lcAccelLat="{lat["accel"]}" maxSpeedLat="{lat["max_speed"]}" '
               f'lcMaxSpeedLatStanding="{lat["standing_speed"]}" lcMaxSpeedLatFactor="{lat["speed_factor"]}" '
               f'lcCooperativeRoundabout="{lat["roundabout_cooperation"]}" '
               if float(TRAFFIC["lateral_resolution"]) > 0 else "")
    return (f'carFollowModel="Krauss" accel="{c["accel"]}" decel="{c["decel"]}" tau="{c["tau"]}" '
            f'minGap="{c["min_gap"]}" sigma="{c["sigma"]}" speedDev="{c["speed_dev"]}" {lateral}'
            f'maxSpeed="{TRAFFIC["max_speed"]}" vClass="passenger" length="4.5" color="0.55,0.55,0.55"')   # 골목 차량은 회색


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

    cutin: True면 신호 없는 골목 출구로 나오는 흐름을 골목 운전자 분포(cutin)로 바꾼다. cutin_bins를 안 주면
           cutin.alley_driver_bins()(앞차 무시 없음, impatience·jmTimegapMinor 구간)를 쓴다.
    ped_types: True면 모든 보행자가 보행자 분포(pedDist)를 쓴다. ped_v_low는 P1 구간이 무시하는 차의 최고 속도[m/s].
    stopped: [StoppedSpec] S4 정차 차량 목록(택시·버스·일반 차량). None이면 없다.
    taxi_depart: 예전 방식. 숫자면 그 시각(초)에 R69 안쪽 차로 정류장 앞에 택시 한 대가 선다(StoppedSpec 하나와 같다).
    merge_route: S5 측정에서 '버스 경로 합류'의 기준이 되는 경로(도로 id 목록). None이면 지도의 bus_loop.
    """

    def __init__(self, out_dir, scenario="baseline", seed=0, *, cutin=True,
                 cutin_bins=None, ped_types=True, ped_v_low=1.5, taxi_depart=None, stopped=None, merge_route=None,
                 bus_traffic=None, right_on_red=True, right_on_red_exclude=("J1", "J82", "J83", "J42")):
        # right_on_red: True면 신호 교차로의 우회전이 신호와 무관하게 간다. 단 우회전 차가 지나는 횡단보도의 보행자 신호가
        # 적색이고 겹치는 직진·좌회전이 녹색이 아닌 위상에서만 열린다(신호 프로그램만 바꾼다). exclude는 합정역 사거리(J1)와
        # 상수역 사거리(J82). J82는 우회전을 연 뒤 교차로가 막히는 문제가 생겨 원래 신호 프로그램으로 되돌렸다(사용자 결정).
        self.right_on_red, self.right_on_red_exclude = right_on_red, tuple(right_on_red_exclude)
        # bus_traffic: BusTrafficCfg 또는 True(기본 설정). 배경 버스(전용 차로 노선·통과)와 택시를 투입한다. None이면 없다.
        self.bus_traffic = BusTrafficCfg() if bus_traffic is True else bus_traffic
        self.out_dir = os.path.abspath(out_dir)
        self.scenario, self.seed = scenario, seed
        self.cutin, self.cutin_bins = cutin, cutin_bins
        self.ped_types, self.ped_v_low, self.taxi_depart = ped_types, ped_v_low, taxi_depart
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
        self.summary = {"cutin_flows": 0, "cutin_alleys": [], "typed_persons": 0, "stopped": []}

        overlay_routes = [BUS_VTYPE]              # 다른 경로 파일보다 먼저 읽혀야 한다
        overlay_add = []
        self.alleys = self._find_alleys()
        if self.cutin:
            bins = self.cutin_bins or alley_driver_bins()
            overlay_routes.append(cutin_vtype_xml("cutin", bins=bins, v_ign=15.0, base_attrs=_krauss_attrs()))
            self.summary["cutin_flows"] = self._retype_alley_flows()
        if self.ped_types:
            overlay_routes.append(ped_dist_xml(self.ped_v_low))
            self.summary["typed_persons"] = self._type_persons()
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
            parts, bsum = build_bus_traffic(net, m["flows"], self.bus_traffic, random.Random(self.seed + 5))
            overlay_routes.extend(parts)
            taxi_parts, taxi_add, self.summary["curb_taxis"] = curb_taxi_stops(
                net, m["flows"], self._bus_stops(cfg), self.bus_traffic, random.Random(self.seed + 7))
            overlay_routes.extend(taxi_parts)
            overlay_add.extend(taxi_add)
            self.summary["bus_traffic"] = bsum

        self._write("traffic.rou.xml", "routes", overlay_routes)
        self._write("traffic.add.xml", "additional", overlay_add)
        self._register(cfg)
        self.cfg = cfg
        return cfg

    def _bus_stops(self, cfg):
        """실행 폴더의 추가 파일에 있는 모든 정류장(busStop): [{id, lane, startPos, endPos}]."""
        names = ET.parse(cfg).getroot().find("input").find("additional-files").get("value").split(",")
        return [{"id": b.get("id"), "lane": b.get("lane"), "startPos": float(b.get("startPos")),
                 "endPos": float(b.get("endPos"))}
                for name in names for b in ET.parse(os.path.join(self.out_dir, name)).getroot().iter("busStop")]

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

    def _find_alleys(self):
        """버스 경로에 합류하는 신호 없는 골목 전부(지도에서 자동 탐지). 신호 교차로로 합류하는 골목(cluster_J52_J56_J58,
        J74, J48)은 신호가 순서를 정해 끼어들기가 없으므로 뺀다. S5 측정(골목에서 나온 차가 버스 앞에 들어왔나)만 쓴다."""
        net = read_net(os.path.join(self.out_dir, "network.net.xml"))
        route = close_loop(net, self.merge_route or load_map(MAP_ID)["routes"]["bus_loop"])
        return unsignalized_alleys(find_merge_alleys(net, route))

    def _retype_alley_flows(self):
        """도로망의 신호 없는 골목 출구로 나오는 배경 흐름(carDist)을 모두 골목 운전자 분포(cutin)로 바꾸고 그 수를 돌려준다.
        골목은 도로 종류로 정하고(alleys.is_alley) 버스 경로는 보지 않는다.

        골목 차는 큰길 차를 무시하지 않는다. 얼마나 좁은 간격에 합류하는지만 운전자마다 다르다(impatience, jmTimegapMinor).
        끼어들기는 시험용 버스와 배경 차 모두에게 같은 규칙으로 일어난다.
        """
        net = read_net(os.path.join(self.out_dir, "network.net.xml"))
        path = os.path.join(self.out_dir, "routes.rou.xml")
        tree = ET.parse(path)
        exits = unsignalized_alleys(find_alley_exits(net))
        per_exit = {(a.feeder, a.into): 0 for a in exits}
        n = 0
        for flow in tree.getroot().iter("flow"):
            merged = [a for a in exits if flow_merges(net, flow.attrib, [a])]
            for a in merged:
                per_exit[(a.feeder, a.into)] += 1
            if merged and flow.get("type") == "carDist":
                flow.set("type", "cutin")
                n += 1
        tree.write(path, encoding="utf-8", xml_declaration=True)
        self.summary["cutin_alleys"] = [
            {"junction": a.junction, "feeder": a.feeder, "into": a.into, "junction_type": a.junction_type,
             "minor": a.minor, "regulated": a.regulated, "flows": per_exit[(a.feeder, a.into)]} for a in exits]
        return n

    def _type_persons(self):
        """보행자 경로 파일의 모든 보행자에게 보행자 분포를 준다. 반환: 그 수."""
        path = os.path.join(self.out_dir, "pedestrians.rou.xml")
        tree = ET.parse(path)
        n = 0
        for person in tree.getroot().iter("person"):
            person.set("type", PED_DIST)
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
        for _ in range(int(round(warmup / 0.1))):
            traci.simulationStep()
        return traci

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

    def run(self, t_end, bus_id="bus0", bus_hook=None, metrics=None, on_step=None,
            assign_existing=True, stop_bypass=True, triggers=None, pilot=None, lane_route=True, **mgr_kwargs):
        """warmup 뒤 t_end초 동안 공통 실행 루프(runner.run)로 돌린다. 반환: (YieldManager, 마지막 시각).

        triggers: A1 진입 의사 구간. None이면 방향지시등만 본다. "scenario"면 scenario_triggers()를 쓴다.
        stop_bypass: S1(정차 버스 뒤 차량) 관찰을 켠다.
        lane_route: 정지선에서 옆 차로로 끝내 못 옮긴 배경 차가 지금 차로가 가는 쪽으로 가게 한다(lane_route.py).
        """
        if triggers == "scenario":
            triggers = self.scenario_triggers(plans=pilot.plans if pilot is not None else None)
        mgr = YieldManager(traci, bus_id, rng=random.Random(self.seed), triggers=triggers,
                           driver_prefixes=("car_", "cutin_"), **mgr_kwargs)
        self.mgr = mgr
        if assign_existing:        # warmup 중에 투입된 차량에도 운전자 성향을 준다
            mgr.on_departed(traci.vehicle.getIDList())
        self.stop = StopBypassManager(traci, bus_id) if stop_bypass else None
        self.lane_route = (LaneRouteAdapter(traci, read_net(os.path.join(self.out_dir, "network.net.xml")), bus_id)
                           if lane_route else None)
        extras = [self.lane_route] if self.lane_route is not None else []
        t = run_loop(traci, t_end, bus_id, bus_hook=bus_hook, yield_mgr=mgr, metrics=metrics,
                     on_step=on_step, stop_mgr=self.stop, extras=extras)
        return mgr, t

    def close(self):
        if traci.isLoaded():
            traci.close()
