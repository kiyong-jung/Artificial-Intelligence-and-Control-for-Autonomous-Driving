"""판단 시점의 TraCI 스냅숏(설계 7.2, 7.4~7.10). 조회만 한다: set 호출과 context 구독을 하지 않는다."""
from __future__ import annotations

import math

from traffic.sumo_conn import traci

from .lap_context import LapContext, lap_route_index
from .schema import (BUS, FOES_LOOKAHEAD_M, FOLLOWER_LOOKBACK_M, LEADER_LOOKAHEAD_M, STATE_RADIUS_M, STOPS_AHEAD,
                     TLS_LOOKAHEAD_M)

TraCIException = traci.exceptions.TraCIException
MODEL_NAME = {"car_krauss": "Krauss", "car_idm": "IDM", "car_eidm": "EIDM", "car_acc": "ACC"}
# 옆 차로 슬롯 이름(값은 getNeighbors 모드 비트였다. getNeighbors는 쓰지 않는다: _side_slots 참고)
NEIGHBOR_MODES ={"left_leader": 3, "right_leader": 2, "left_follower": 1, "right_follower": 0}
STOP_FIELDS = ("lane", "startPos", "endPos", "stoppingPlaceID", "stopFlags", "duration", "until", "intendedArrival",
               "arrival", "depart")
FOE_FIELDS = ("foe_id", "ego_dist", "foe_dist", "ego_exit_dist", "foe_exit_dist", "ego_lane", "foe_lane",
              "ego_response", "foe_response")
INVALID = -1e6               # TraCI가 닿을 수 없는 거리에 주는 큰 음수


def _lap_position(ctx, road, last_road, lane, lane_pos):
    try:
        idx = lap_route_index(road, last_road, ctx.lap_edges)
    except ValueError:
        return None, None
    return idx, ctx.route_map.position(idx, lane, lane_pos)


def ego_block(tc, ctx: LapContext, last_road) -> dict:
    v = tc.vehicle
    x, y = v.getPosition(BUS)
    road, lane, lane_pos = v.getRoadID(BUS), v.getLaneID(BUS), v.getLanePosition(BUS)
    idx, pos = _lap_position(ctx, road, last_road, lane, lane_pos)
    return {"x": x, "y": y, "heading": v.getAngle(BUS), "speed": v.getSpeed(BUS), "acceleration": v.getAcceleration(BUS),
            "lateral_speed": v.getLateralSpeed(BUS), "speed_without_traci": v.getSpeedWithoutTraCI(BUS),
            "allowed_speed": v.getAllowedSpeed(BUS), "lane_max_speed": tc.lane.getMaxSpeed(lane),
            "max_speed": v.getMaxSpeed(BUS), "road_id": road, "lane_id": lane, "lane_index": v.getLaneIndex(BUS),
            "lane_count": tc.edge.getLaneNumber(road), "lane_position": lane_pos,
            "lateral_position": v.getLateralLanePosition(BUS), "route_index": v.getRouteIndex(BUS),
            "lap_route_index": idx, "lap_position": pos, "distance": v.getDistance(BUS), "signals": v.getSignals(BUS),
            "stop_state": v.getStopState(BUS), "waiting_time": v.getWaitingTime(BUS),
            "accumulated_waiting": v.getAccumulatedWaitingTime(BUS),
            "lc_state_left": list(v.getLaneChangeState(BUS, 1)), "lc_state_right": list(v.getLaneChangeState(BUS, -1)),
            "could_change_left": bool(v.couldChangeLane(BUS, 1)), "could_change_right": bool(v.couldChangeLane(BUS, -1))}


def route_block(tc, ctx: LapContext, ego: dict) -> dict:
    best = [{"lane_id": b[0], "length": b[1], "occupation": b[2], "offset": b[3], "allows_continuation": bool(b[4]),
             "next_lanes": list(b[5])} for b in tc.vehicle.getBestLanes(BUS)]
    links = [{"lane": k[0], "priority": bool(k[1]), "opened": bool(k[2]), "foe": bool(k[3]), "via": k[4],
              "state": k[5], "direction": k[6], "length": k[7]} for k in tc.vehicle.getNextLinks(BUS)]
    pos = ego["lap_position"]
    return {"best_lanes": best, "next_links": links,
            "lap_progress": {"position": pos, "length": ctx.route_map.length},
            "next_turn": ctx.route_map.ahead(pos) if pos is not None else None}


def tls_block(tc, now: float) -> list:
    out = []
    for tid, idx, dist, state in tc.vehicle.getNextTLS(BUS):
        if dist > TLS_LOOKAHEAD_M:
            continue
        tl = tc.trafficlight
        out.append({"id": tid, "link_index": idx, "distance": dist, "state": state,
                    "time_to_switch": tl.getNextSwitch(tid) - now, "spent": tl.getSpentDuration(tid),
                    "phase": tl.getPhase(tid), "program": tl.getProgram(tid)})
    return out


def stops_block(tc) -> dict:
    stops, dists = [], []
    for s in tc.vehicle.getStops(BUS, STOPS_AHEAD):
        d = {k: getattr(s, k, None) for k in STOP_FIELDS}
        stops.append(d)
        try:
            dd = tc.vehicle.getDrivingDistance(BUS, d["lane"].rsplit("_", 1)[0], d["startPos"])
            dists.append(dd if dd > INVALID else None)
        except (TraCIException, AttributeError, TypeError):
            dists.append(None)
    return {"stop_state": tc.vehicle.getStopState(BUS), "stops": stops, "distances": dists}


def _leader_slot(tc, lid, gap, v_ego):
    v = tc.vehicle
    v_lead, decel = v.getSpeed(lid), v.getDecel(lid)
    return {"id": lid, "gap_raw": gap, "secure_gap": v.getSecureGap(BUS, v_ego, v_lead, decel, lid),
            "follow_speed": v.getFollowSpeed(BUS, v_ego, gap, v_lead, decel, lid),
            "stop_speed": v.getStopSpeed(BUS, v_ego, gap)}


def _follower_slot(tc, fid, gap, v_ego):
    v = tc.vehicle
    v_f = v.getSpeed(fid)
    return {"id": fid, "gap_raw": gap, "secure_gap": v.getSecureGap(fid, v_f, v_ego, v.getDecel(BUS), BUS)}


def _side_slots(tc, ego, vehicles) -> dict:
    """옆 차로(같은 도로, 차로 번호 ±1)의 가장 가까운 선행·후행. getNeighbors는 SUMO 1.27.1에서 시뮬레이션을 죽여서
    (합정역 한 바퀴 수집에서 재현, simulationStep 중 연결 끊김) 쓰지 않는다. 간격 정의는 TraCI와 같다:
    선행 = 앞차 뒤끝 - ego 앞끝 - ego minGap, 후행 = ego 뒤끝 - 뒤차 앞끝 - 뒤차 minGap. 다른 도로에 있는 옆 차로 차는 못 잡는다."""
    if ego["road_id"].startswith(":"):
        return {name: None for name in NEIGHBOR_MODES}
    ego_len, ego_gap = tc.vehicle.getLength(BUS), tc.vehicle.getMinGap(BUS)
    front, back = ego["lane_position"], ego["lane_position"] - ego_len
    best = {name: None for name in NEIGHBOR_MODES}
    for o in vehicles:
        if o["road_id"] != ego["road_id"]:
            continue
        side = {ego["lane_index"] + 1: "left", ego["lane_index"] - 1: "right"}.get(o["lane_index"])
        if side is None:
            continue
        if o["lane_position"] > front:
            name, gap, limit = f"{side}_leader", o["lane_position"] - o["length"] - front - ego_gap, LEADER_LOOKAHEAD_M
        else:
            name, gap, limit = f"{side}_follower", back - o["lane_position"] - o["min_gap"], FOLLOWER_LOOKBACK_M
        if gap <= limit and (best[name] is None or gap < best[name][1]):
            best[name] = (o["id"], gap)
    return best


def neighbors_block(tc, ego=None, vehicles=()) -> dict:
    v = tc.vehicle
    v_ego = v.getSpeed(BUS)
    found = {}
    ld = v.getLeader(BUS, LEADER_LOOKAHEAD_M)
    found["leader"] = (ld[0], ld[1]) if ld and ld[0] else None
    fo = v.getFollower(BUS, FOLLOWER_LOOKBACK_M)
    found["follower"] = (fo[0], fo[1]) if fo and fo[0] else None
    found.update(_side_slots(tc, ego, vehicles) if ego is not None else {name: None for name in NEIGHBOR_MODES})
    out = {}
    for name in ("leader", "follower", *NEIGHBOR_MODES):
        hit = found[name]
        try:
            out[name] = None if hit is None else (
                _leader_slot(tc, hit[0], hit[1], v_ego) if name.endswith("leader") else
                _follower_slot(tc, hit[0], hit[1], v_ego))
        except TraCIException:
            out[name] = None
    return out


def junction_foes_block(tc) -> list:
    try:
        foes = tc.vehicle.getJunctionFoes(BUS, FOES_LOOKAHEAD_M)
    except TraCIException:
        return []
    return [dict(zip(FOE_FIELDS, f)) for f in foes]


def _param(tc, vid, key):
    p = tc.vehicle.getParameter(vid, key)
    return p if p != "" else None


def vehicles_block(tc, ctx: LapContext, ego_xy, ego_pos, roles: dict) -> list:
    v, out = tc.vehicle, []
    for vid in v.getIDList():
        if vid == BUS:
            continue
        try:
            x, y = v.getPosition(vid)
            if math.hypot(x - ego_xy[0], y - ego_xy[1]) > STATE_RADIUS_M:
                continue
            road, lane, lane_pos = v.getRoadID(vid), v.getLaneID(vid), v.getLanePosition(vid)
            vtype = v.getTypeID(vid).split("@")[0]
            p_yield = _param(tc, vid, "driver.pYield")
            route_distance = None
            if ego_pos is not None and road in ctx.lap_edges:
                route_distance = ctx.route_map.position(ctx.lap_edges.index(road), lane, lane_pos) - ego_pos
            role = roles.get(vid, {})
            out.append({"id": vid, "type": vtype, "vclass": v.getVehicleClass(vid), "model": MODEL_NAME.get(vtype, vtype),
                        "x": x, "y": y, "heading": v.getAngle(vid), "speed": v.getSpeed(vid),
                        "acceleration": v.getAcceleration(vid), "lateral_speed": v.getLateralSpeed(vid),
                        "road_id": road, "lane_id": lane, "lane_index": v.getLaneIndex(vid), "lane_position": lane_pos,
                        "lateral_position": v.getLateralLanePosition(vid), "signals": v.getSignals(vid),
                        "length": v.getLength(vid), "width": v.getWidth(vid), "min_gap": v.getMinGap(vid),
                        "waiting_time": v.getWaitingTime(vid), "driver_type": _param(tc, vid, "driver.type"),
                        "p_yield": float(p_yield) if p_yield is not None else None,
                        "injected": bool(role.get("injected", False)), "role": role.get("role"),
                        "route_distance": route_distance})
        except TraCIException:
            continue
    return out


def pedestrians_block(tc, ctx: LapContext, ego_xy, roles: dict) -> list:
    p, out = tc.person, []
    for pid in p.getIDList():
        try:
            x, y = p.getPosition(pid)
            if math.hypot(x - ego_xy[0], y - ego_xy[1]) > STATE_RADIUS_M:
                continue
            road = p.getRoadID(pid)
            try:
                stage = p.getStage(pid).type
            except (TraCIException, AttributeError):
                stage = None
            role = roles.get(pid, {})
            out.append({"id": pid, "x": x, "y": y, "heading": p.getAngle(pid), "speed": p.getSpeed(pid),
                        "road_id": road, "lane_position": p.getLanePosition(pid), "next_edge": p.getNextEdge(pid),
                        "waiting_time": p.getWaitingTime(pid), "stage": stage, "on_crossing": road in ctx.crossing_ids,
                        "injected": bool(role.get("injected", False)), "role": role.get("role"),
                        "a3_decision": role.get("a3_decision")})
        except TraCIException:
            continue
    return out


def take_snapshot(tc, ctx: LapContext, last_road, roles: dict, now: float) -> dict:
    ego = ego_block(tc, ctx, last_road)
    xy = (ego["x"], ego["y"])
    vehicles = vehicles_block(tc, ctx, xy, ego["lap_position"], roles.get("vehicles", {}))
    return {"ego": ego, "route": route_block(tc, ctx, ego), "tls": tls_block(tc, now), "stops": stops_block(tc),
            "neighbors": neighbors_block(tc, ego, vehicles), "junction_foes": junction_foes_block(tc),
            "vehicles": vehicles, "pedestrians": pedestrians_block(tc, ctx, xy, roles.get("pedestrians", {}))}
