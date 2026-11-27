"""팔달문 도로를 SUMO가 읽을 실행 파일로 만든다.

지도 원본(maps/paldalmun/)은 수정하지 않고 out_dir 안에만 파일을 만든다.
학습·평가·수집·검증이 같은 빌더를 쓴다.

    network.net.xml   검수한 native 도로망의 바이트 단위 사본
    routes.rou.xml    차량 타입, 경로, 유입 수요
    map.add.xml       배경 폴리곤과 정류장 시설(busStop). 버스·승객은 없다
    view.gui.xml      sumo-gui 초기 시점
    scenario.sumocfg  위 파일을 묶는 실행 진입점
    manifest.json     지도 버전·해시와 이번 실행의 설정
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from sumolib import checkBinary
from sumolib.geomhelper import distancePointToPolygon, positionAtShapeOffset
from sumolib.net import readNet

from env.mdp_config import SIMULATION
from env.road_config import vtype_attributes
from maps import MAPS_DIR


def _write_xml(path: Path, root: ET.Element) -> None:
    ET.indent(root, space="    ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def _attrs(values: dict) -> dict:
    return {str(key): str(value).lower() if isinstance(value, bool) else str(value)
            for key, value in values.items() if value is not None}


def _lateral_attributes(traffic: dict, background: bool = False) -> dict:
    """옆 이동 속성. 배경 차는 background_lateral이 있으면 그것을 쓴다(버스는 늘 lateral_model)."""
    model = traffic.get("background_lateral", traffic["lateral_model"]) if background else traffic["lateral_model"]
    return {attribute: model[key] for key, attribute in (
        ("accel", "lcAccelLat"), ("max_speed", "maxSpeedLat"),
        ("standing_speed", "lcMaxSpeedLatStanding"), ("speed_factor", "lcMaxSpeedLatFactor"),
        ("roundabout_cooperation", "lcCooperativeRoundabout"))}


def _write_vehicle_types(routes: ET.Element, ego: dict, traffic: dict) -> None:
    """ego는 지도가 고른 제원 한 종류, 배경차는 추종모델 네 종류의 확률 분포다."""
    lateral = _lateral_attributes(traffic)
    if float(traffic["lateral_resolution"]) > 0:
        lateral["laneChangeModel"] = "SL2015"
    ET.SubElement(routes, "vType", _attrs(dict(
        id="ego_type", speedFactor=ego["speed_factor"], speedDev=ego["speed_dev"], color="1,0,0",
        **vtype_attributes(ego), **lateral)))
    lateral = _lateral_attributes(traffic, background=True)
    if float(traffic["lateral_resolution"]) > 0:
        lateral["laneChangeModel"] = "SL2015"
    controllers = traffic["controllers"]
    total = sum(float(c["probability"]) for c in controllers.values())
    distribution = ET.SubElement(routes, "vTypeDistribution", id="carDist")
    for name, c in controllers.items():
        # 도시에서는 회전 경로를 따라갈 차선변경이 필요하다. SUMO의 기본
        # 전략/협조 차선변경을 유지한다.
        ET.SubElement(distribution, "vType", _attrs(dict(
            id=f"car_{name}", probability=float(c["probability"]) / total,
            carFollowModel=c["carFollowModel"], accel=c["accel"], decel=c["decel"],
            tau=c["tau"], minGap=c["min_gap"], maxSpeed=traffic["max_speed"],
            speedDev=c["speed_dev"], color=c["color"], sigma=c.get("sigma"),
            collisionAvoidanceOverride=c.get("collision_avoidance_override"),
            vClass="passenger", guiShape="passenger/sedan", **lateral, **traffic["background_lane_change"])))
    if "slow" in traffic:
        # 느린 차는 Krauss 차의 최고속도만 낮춘 것이다. 배달 트럭 모양이라 영상에서 구별된다.
        c = controllers["krauss"]
        ET.SubElement(routes, "vType", _attrs(dict(
            id="snail", carFollowModel="Krauss", accel=c["accel"], decel=c["decel"], tau=c["tau"],
            minGap=c["min_gap"], sigma=c["sigma"], maxSpeed=traffic["slow"]["max_speed"], speedDev=c["speed_dev"],
            color=".55,.3,.1", vClass="passenger", guiShape="delivery",
            **lateral, **traffic["background_lane_change"])))


def _write_krauss_type(routes: ET.Element, traffic: dict, vtype: str, color: str, shape: str, extra: dict) -> None:
    """Krauss 배경 차와 같은 제원의 따로 이름 붙인 vType(택시, 골목 차). extra는 더하는 속성이다."""
    c = traffic["controllers"]["krauss"]
    ET.SubElement(routes, "vType", _attrs(dict(
        id=vtype, carFollowModel="Krauss", accel=c["accel"], decel=c["decel"], tau=c["tau"], minGap=c["min_gap"],
        sigma=c["sigma"], maxSpeed=traffic["max_speed"], speedDev=c["speed_dev"], color=color, vClass="passenger",
        guiShape=shape, **_lateral_attributes(traffic, background=True), **traffic["background_lane_change"], **extra)))


def _appear_distance(net, edge: str, route_shapes: list) -> float:
    """차가 처음 나타나는 자리(edge 0번 차로의 처음 10m)와 버스 노선 차로 사이의 가장 짧은 거리(m)."""
    lane = net.getEdge(edge).getLanes()[0]
    return min(distancePointToPolygon(positionAtShapeOffset(lane.getShape(), offset), shape)
               for offset in range(0, 11) for shape in route_shapes)


def _taxis_per_stop(taxi: dict) -> float:
    """정류장 하나에 시간당 오는 택시 수. 점유율 = 도착률 x 평균 정차 시간."""
    dwells = [float(d) for d in taxi["dwells_s"]]
    return float(taxi["occupancy"]) * 3600.0 / (sum(dwells) / len(dwells))


def _write_taxi_flows(routes: ET.Element, net, network: dict, ego_route: str, taxi: dict, traffic: dict) -> None:
    """정류장 앞 택시(taxi*). 정류장마다 시간당 occupancy*3600/평균 정차 시간 대의 택시가 원래 흐름 중 가장 빠른 경로가 그
    정류장 도로를 지나는 (출발, 도착) 쌍으로 달리며, 정류장 차로 안 두 자리(head, tail)에 dwells_s 중 하나만큼 선다(SUMO
    차로 정차 stop). 출발·도착은 지도의 경계 도로라 버스 가까이에서 생기거나 사라지지 않는다. 정류장 차로에서 경로의 다음
    도로로 바로 이어지는 쌍만 쓴다(정류장 뒤 차로를 여러 칸 옮겨야 하면 서서 막는다). clearance가 있으면 출발 도로의 처음
    10m가 버스 노선 차로에서 그보다 가까운 쌍은 빼서, 지도 원래 흐름의 출발 골목(G31 등)에서도 택시가 버스 눈앞에 생기지 않게
    한다(taxi25·taxi50·rich·rich_lo는 clearance 없이 만들었다)."""
    _write_krauss_type(routes, traffic, "taxi", "1,.75,0", "passenger/hatchback", {})
    paths = {(f["from"], f["to"]): [e.getID() for e in net.getFastestPath(
        net.getEdge(f["from"]), net.getEdge(f["to"]), vClass="passenger")[0] or ()] for f in network["flows"]}
    dwells = [float(d) for d in taxi["dwells_s"]]
    per_stop = _taxis_per_stop(taxi)
    for stop in taxi["stops"]:
        lane = net.getLane(stop["lane"])
        edge = lane.getEdge().getID()
        onward = {c.getTo().getID() for c in lane.getOutgoing()}
        pairs = sorted(od for od, p in paths.items() if edge in p[:-1] and p[p.index(edge) + 1] in onward)
        if taxi.get("clearance"):
            route_shapes = [lane.getShape() for e in network["routes"][ego_route] for lane in net.getEdge(e).getLanes()]
            pairs = [(o, d) for o, d in pairs if _appear_distance(net, o, route_shapes) >= taxi["clearance"]]
        if not pairs:
            raise ValueError(f"정류장 {stop['id']} 차로에서 이어 갈 원래 흐름이 없다.")
        spots = {"head": (float(stop["endPos"]) - 6.0, float(stop["endPos"])),
                 "tail": (float(stop["startPos"]), float(stop["startPos"]) + 6.0)}
        for spot, (start, end) in spots.items():
            for dwell in dwells:
                for o, d in pairs:
                    flow = ET.SubElement(routes, "flow", _attrs({
                        "id": f"taxi_{stop['id']}_{spot}_{dwell:.0f}_{o}_to_{d}", "type": "taxi", "from": o, "to": d,
                        "begin": "0", "end": 100000,
                        "period": f"exp({per_stop / len(spots) / len(dwells) / len(pairs) / 3600.0:.6f})",
                        "departLane": "best", "departSpeed": traffic["depart_speed"]}))
                    ET.SubElement(flow, "stop", _attrs({"lane": stop["lane"], "startPos": f"{start:.2f}",
                                                        "endPos": f"{end:.2f}", "duration": f"{dwell:.0f}"}))


def _scaled_people(output: Path, source: str, scale: float) -> str:
    """보행자 경로 파일의 personFlow 도착률(exp(rate))을 scale배로 바꿔 실행 폴더에 쓴다. 원본은 그대로 둔다."""
    tree = ET.parse(source)
    for flow in tree.getroot().iter("personFlow"):
        period = flow.get("period", "")
        if not period.startswith("exp("):
            raise ValueError(f"보행자 흐름 {flow.get('id')}의 period가 포아송(exp)이 아니다: {period}")
        flow.set("period", f"exp({float(period[4:-1]) * scale:.8f})")
    name = Path(source).name
    tree.write(output / name, encoding="utf-8", xml_declaration=True)
    return name


def _write_side_flows(routes: ET.Element, net, network: dict, side: dict, total: float, traffic: dict) -> None:
    """골목·주차장 출입 수요. 블록 중간 도로 M마다 길이 비례 몫을 받아, M에서 출발해 지도의 도착 도로로 가는 흐름(id mid_in_)과
    지도의 출발 도로에서 M으로 와서 그 위 무작위 위치에서 사라지는 흐름(id mid_out_)을 만든다. 상대 도로는 원래 흐름 중 가장
    빠른 경로가 M을 지나는 것만 고른다. 원래 여행을 M에서 시작하거나 끝낸 것이라 원래 수요가 쓰지 않던 회전이 생기지 않는다
    (아무 도로나 고르면 R73으로 가려고 동막 교차로에서 비보호 좌회전해 G30을 돌아오다 교차로 안에서 부딪혔다). 출발 위치는
    random_free(빈 자리를 열 번 찾고 없으면 free)이고, SUMO가 앞뒤 차 간격을 확인한 뒤에만 넣는다. 도착 속도는 SUMO
    기본값(current)이라 서지 않고 그 자리에서 사라진다. 모두 고정 포아송 수요이고 버스 위치와 무관하다."""
    if network.get("flow_arrivals") != "poisson":
        raise ValueError("골목 출입 수요는 포아송 도착 지도에서만 만든다(고정 간격 흐름은 시작 시각 순서가 필요하다).")
    lengths = {m: net.getEdge(m).getLength() for m in side["edges"]}
    paths = {(f["from"], f["to"]): {e.getID() for e in net.getFastestPath(
        net.getEdge(f["from"]), net.getEdge(f["to"]), vClass="passenger")[0] or ()} for f in network["flows"]}

    for m, length in lengths.items():
        mine = total * length / sum(lengths.values())
        for prefix, share, pairs, position in (
                ("mid_in", side["share_in"], sorted({(m, d) for (o, d), p in paths.items() if m in p}), {"departPos": "random_free"}),
                ("mid_out", side["share_out"], sorted({(o, m) for (o, d), p in paths.items() if m in p}), {"arrivalPos": "random"})):
            for a, b in pairs if share > 0 else ():
                ET.SubElement(routes, "flow", _attrs({
                    "id": f"{prefix}_{a}_to_{b}", "type": "carDist", "from": a, "to": b, "begin": "0", "end": 100000,
                    "period": f"exp({mine * share / len(pairs) / 3600.0:.6f})", "departLane": "best",
                    "departSpeed": traffic["depart_speed"], **position}))


def _write_alley_flows(routes: ET.Element, net, network: dict, ego_route: str, alley: dict, total: float,
                       traffic: dict) -> None:
    """골목 합류 수요. 골목 (출발 도로 O, 합류 도로 M)마다 total*share를 똑같이 나눠, O에서 출발해 지도의 도착 도로로 가는 흐름
    (id alley_)을 만든다. 차는 O의 시작에서 보통 투입(departPos base)되고, 골목 끝 신호 없는 교차로에서 양보한 뒤 M으로 합류한다.
    도착 도로는 원래 흐름 중 가장 빠른 경로가 M을 지나는 것의 도착 도로이고, O에서 그 도로까지의 가장 빠른 경로가 M을 지나며
    M 뒤의 회전이 모두 원래 흐름이 쓰는 회전인 것만 고른다(원래 수요가 쓰지 않던 회전은 교차로 안에서 멈춰 섰다, mid* 기록).
    도착은 지도의 경계 도로뿐이라 버스 노선 위에서 사라지는 차가 없다. 차가 처음 나타나는 자리(O의 처음 10m)가 버스 노선
    차로에서 clearance(버스 감지 범위)보다 가까우면 만들지 않는다."""
    route_shapes = [lane.getShape() for edge in network["routes"][ego_route] for lane in net.getEdge(edge).getLanes()]
    vtype = "carDist"
    if alley.get("driver"):
        # 끼어드는 골목 차(cut*): Krauss 배경 차에 교차로 성향(road_config.CUT_DRIVERS)만 더한 것이다.
        vtype = "alley_driver"
        _write_krauss_type(routes, traffic, vtype, ".6,.6,.6", "passenger/sedan", alley["driver"])
    paths = {}
    for f in network["flows"]:
        path = net.getFastestPath(net.getEdge(f["from"]), net.getEdge(f["to"]), vClass="passenger")[0] or ()
        paths[f["to"]] = paths.get(f["to"], []) + [[e.getID() for e in path]]
    turns = {pair for options in paths.values() for p in options for pair in zip(p, p[1:])}
    for origin, merge in alley["edges"]:
        appear = _appear_distance(net, origin, route_shapes)
        if appear < alley["clearance"]:
            raise ValueError(f"골목 출발 도로 {origin}은 버스 노선에서 {appear:.0f}m라 차가 버스 감지 범위 안에서 나타난다.")
        partners = []
        for destination in sorted(d for d, options in paths.items() if any(merge in p for p in options)):
            path = [e.getID() for e in net.getFastestPath(net.getEdge(origin), net.getEdge(destination), vClass="passenger")[0] or ()]
            if merge in path and all(pair in turns for pair in zip(path[path.index(merge):], path[path.index(merge) + 1:])):
                partners.append(destination)
        if not partners:
            raise ValueError(f"골목 {origin}에서 {merge}로 합류해 갈 도착 도로가 없다.")
        for destination in partners:
            ET.SubElement(routes, "flow", _attrs({
                "id": f"alley_{origin}_to_{destination}", "type": vtype, "from": origin, "to": destination,
                "begin": "0", "end": 100000,
                "period": f"exp({total * alley['share'] / len(alley['edges']) / len(partners) / 3600.0:.6f})",
                "departLane": "best", "departSpeed": traffic["depart_speed"]}))


def _copy_into(output: Path, source: str) -> str:
    name = Path(source).name
    (output / name).write_bytes(Path(source).read_bytes())
    return name


def build(road: dict, ego: dict, traffic: dict, out_dir: str) -> str:
    """실행 파일을 out_dir에 만들고 scenario.sumocfg 경로를 반환한다."""
    network = road["network"]
    output = Path(out_dir).expanduser().resolve()
    if output == MAPS_DIR or MAPS_DIR in output.parents:
        raise ValueError("out_dir는 maps/ 밖의 실행 폴더로 지정하세요. 지도 원본은 덮어쓰지 않습니다.")
    output.mkdir(parents=True, exist_ok=True)

    # 1) 검수한 native 도로망을 바이트 그대로 복사한다. 실행 때 netconvert를
    #    다시 돌리면 교차로 곡선이 SUMO 버전에 따라 달라질 수 있다. 경로와 정류장이
    #    이 도로망에 맞는지는 scripts/build_road.py가 게시 전에 검사했다.
    content = Path(network["net_path"]).read_bytes()
    network_sha256 = hashlib.sha256(content).hexdigest()
    if network_sha256 != network["network_source"]["sha256"]:
        raise ValueError("native 도로망의 해시가 map.json과 다릅니다.")
    (output / "network.net.xml").write_bytes(content)
    compiled = readNet(str(output / "network.net.xml"))

    # 2) 차량 타입, 경로, 유입 수요
    routes = ET.Element("routes")
    _write_vehicle_types(routes, ego, traffic)
    for route_id, edge_ids in network["routes"].items():
        ET.SubElement(routes, "route", id=route_id, edges=" ".join(edge_ids))
    scale = float(traffic["demand_scale"])
    # 같은 진입로의 흐름들이 같은 시각에 차를 넣으면 겹쳐 투입되어 충돌한다. 진입로 전체의 투입 간격으로
    # 시작 시각을 엇갈려, 진입로마다 차가 고른 간격으로 들어오게 한다.
    entry_total, entry_seen = {}, {}
    for flow in network["flows"]:
        entry_total[flow["from"]] = entry_total.get(flow["from"], 0.0) + float(flow["vehs_per_hour"]) * scale
    staggered = []
    for flow in network["flows"]:
        volume = float(flow["vehs_per_hour"]) * scale
        if volume > 0:
            staggered.append((entry_seen.get(flow["from"], 0) * 3600.0 / entry_total[flow["from"]], volume, flow))
            entry_seen[flow["from"]] = entry_seen.get(flow["from"], 0) + 1
    # SUMO는 시작 시각 순서가 아닌 흐름을 무시한다.
    # 지도가 flow_arrivals를 "poisson"으로 두면 차가 무작위 간격으로 들어온다(평균은 같다). 고정 간격이면 seed가 달라도
    # 같은 시각에 같은 차가 나와, 버스가 겪는 일이 에피소드마다 되풀이된다.
    poisson = network.get("flow_arrivals") == "poisson"
    # 느린 차는 흐름마다 share만큼을 떼어 같은 출발·도착의 별도 흐름으로 넣는다. 포아송 도착을 나눈 것이라 합은 원래 흐름과
    # 같은 분포다. 차 id가 snail_로 시작해 기록에서 느린 차를 가려낼 수 있다.
    share = float(traffic["slow"]["share"]) if "slow" in traffic else 0.0
    side = traffic.get("side")
    keep = 1.0 - side["share_in"] - side["share_out"] if side else 1.0
    alley = traffic.get("alley")
    keep -= alley["share"] if alley else 0.0
    taxi = traffic.get("taxi")
    if taxi:
        keep -= _taxis_per_stop(taxi) * len(taxi["stops"]) / sum(volume for _begin, volume, _flow in staggered)
    for begin, volume, flow in sorted(staggered, key=lambda item: item[0]):
        for flow_id, vtype, part in ((flow["id"], "carDist", volume * keep * (1 - share)),
                                     (f"snail_{flow['id']}", "snail", volume * keep * share)):
            if part <= 0:
                continue
            # 배경 차량은 출발·도착 도로만 정하고 경로는 SUMO가 고른다. 지도가 via를 적은 흐름은 그 도로들을 거쳐 간다.
            ET.SubElement(routes, "flow", _attrs({
                "id": flow_id, "type": vtype, "from": flow["from"], "to": flow["to"], "via": flow.get("via"),
                "begin": "0" if poisson else f"{begin:.3f}", "end": 100000,
                "period": f"exp({part / 3600.0:.6f})" if poisson else f"{3600.0 / part:.6f}", "departLane": "best",
                "departSpeed": traffic["depart_speed"]}))
    if side:
        _write_side_flows(routes, compiled, network, side, sum(volume for _begin, volume, _flow in staggered), traffic)
    if alley:
        _write_alley_flows(routes, compiled, network, road["ego_route"], alley,
                           sum(volume for _begin, volume, _flow in staggered), traffic)
    if taxi:
        _write_taxi_flows(routes, compiled, network, road["ego_route"], taxi, traffic)
    _write_xml(output / "routes.rou.xml", routes)

    # 지도에 딸린 SUMO 파일(신호 프로그램, 교통섬, 보행자 등)은 그대로 복사해 함께 읽는다. 보행자 배율(ped*)이 있으면
    # 보행자 흐름의 도착률만 바꾼 사본을 쓴다.
    extra_additional = [_copy_into(output, path) for path in network.get("extra_additional_paths", [])]
    people = traffic.get("pedestrian_scale")
    extra_routes = [_scaled_people(output, path, float(people)) if people else _copy_into(output, path)
                    for path in network.get("extra_route_paths", [])]

    # 3) 정류장은 도로 시설로만 표시한다. 버스와 승객은 넣지 않는다.
    additional = ET.parse(network["context_path"]).getroot()
    for stop in network.get("bus_stops", []):
        ET.SubElement(additional, "busStop", _attrs(dict(
            id=stop["id"], name=stop["name"], lane=stop["lane"],
            startPos=stop["startPos"], endPos=stop["endPos"])))
    _write_xml(output / "map.add.xml", additional)

    # 4) GUI 초기 시점: 관심 영역(focus_bbox)이 화면에 맞도록 확대한다.
    left, bottom, right, top = compiled.getBoundary()
    west, south, east, north = network["focus_bbox"]
    x0, y0 = compiled.convertLonLat2XY(west, south)
    x1, y1 = compiled.convertLonLat2XY(east, north)
    zoom = 100.0 * max(right - left, top - bottom) / max(abs(x1 - x0), abs(y1 - y0))
    view = ET.Element("viewsettings")
    ET.SubElement(view, "scheme", name="real world")
    ET.SubElement(view, "viewport", _attrs(dict(
        zoom=zoom, x=network["gui_center"]["x"], y=network["gui_center"]["y"], angle=0)))
    ET.SubElement(view, "delay", value=str(SIMULATION["gui_delay"]))
    _write_xml(output / "view.gui.xml", view)

    # 5) 교차로 내부의 충돌도 집계한다. 평가 때도 같은 판정 기준을 쓴다.
    configuration = ET.Element("configuration")
    inputs = ET.SubElement(configuration, "input")
    for name, value in (("net-file", "network.net.xml"),
                        ("route-files", ",".join(["routes.rou.xml", *extra_routes])),
                        ("additional-files", ",".join(["map.add.xml", *extra_additional]))):
        ET.SubElement(inputs, name, value=value)
    processing = ET.SubElement(configuration, "processing")
    for name, value in (("lateral-resolution", traffic["lateral_resolution"]),
                        ("collision.check-junctions", True),
                        # 부딪힌 두 차는 collision_stop_s 동안 그 자리에 섰다가 치워진다(견인). 바로 지우면 사고가 '사라짐'으로
                        # 보이고, warn으로 다시 달리게 하면 겹친 두 차가 교차로에서 또 부딪혀 245초 넘게 막았다(검사 2회차 R2-1).
                        # 충돌은 info의 background_collisions와 경고 기록에 남는다. ego 충돌은 getCollidingVehiclesIDList로 읽어
                        # 에피소드를 끝낸다(치워지기 전이다).
                        ("collision.action", "remove"), ("collision.stoptime", traffic["collision_stop_s"]),
                        # 순간이동을 끈다. ego가 막히면 환경이 stuck으로 끝낸다(SIMULATION["stuck_timeout"]).
                        ("time-to-teleport", -1),
                        ("collision.mingap-factor", traffic["collision_min_gap_factor"])):
        ET.SubElement(processing, name, _attrs(dict(value=value)))
    ET.SubElement(ET.SubElement(configuration, "time"), "step-length",
                  value=str(SIMULATION["step_length"]))
    ET.SubElement(ET.SubElement(configuration, "random_number"), "seed", value=str(road["seed"]))
    ET.SubElement(ET.SubElement(configuration, "gui_only"), "gui-settings-file", value="view.gui.xml")
    cfg_path = output / "scenario.sumocfg"
    _write_xml(cfg_path, configuration)

    # 6) 지도 버전만으로는 내용이 같은지 알 수 없다. 원본과 생성 파일의 해시를 함께 남긴다.
    generated = ["network.net.xml", "routes.rou.xml", "map.add.xml", "view.gui.xml", "scenario.sumocfg",
                 *extra_routes, *extra_additional]
    sumo_version = subprocess.run([checkBinary("sumo"), "--version"], capture_output=True,
                                  text=True, check=True).stdout.splitlines()[0]
    manifest = dict(
        map_id=network["id"], map_version=network["version"], network_sha256=network_sha256,
        scenario=road["scenario"], seed=road["seed"], demand_scale=scale, ego=ego, traffic=traffic,
        generated_sha256={name: hashlib.sha256((output / name).read_bytes()).hexdigest()
                          for name in generated},
        sumo_version=sumo_version, step_length=SIMULATION["step_length"])
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                                          encoding="utf-8")
    return str(cfg_path)
