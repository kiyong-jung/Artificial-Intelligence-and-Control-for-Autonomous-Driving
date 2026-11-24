"""PDM 원시 행 고정 데이터. 기록기(pdm_data/recorder.py)가 쓰는 행과 같은 키를 빠짐없이 채운 최소 행을 만든다."""


def make_run_dir(tmp_path, n=11):
    """원시 수집 폴더 모양의 작은 고정 데이터: 에피소드 1의 행 n개(마지막 행에서 바퀴 완료) + final(end='lap') 행."""
    import gzip
    import json
    import os
    import shutil
    from pdm_fakes import _ctx_args
    net_path, edges, stops, vmax = _ctx_args()
    d = os.path.join(str(tmp_path), "run")
    os.makedirs(os.path.join(d, "assets"))
    shutil.copy(net_path, os.path.join(d, "assets", "network.net.xml"))
    meta = {"schema_version": "pdm_bus_raw_v1", "net_sha256": "x",
            "route": {"lap_edges": list(edges), "bus_stops": stops, "vmax": vmax,
                      "lane_allowed": {"R69_1": [], "R69_2": [], "R69_0": ["pedestrian"]}},
            "expert": {"ego_dims": {"length": 12.0, "width": 2.5, "min_gap": 1.0}}, "tls_programs": {}}
    with open(os.path.join(d, "metadata.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f)
    rows = [make_row(t=i, speed=5.0 + 0.2 * i, distance=10.0 * i, lap_position=100.0 + 2.5 * i,
                     window_speeds=[5.0 + 0.2 * i + 0.04 * (k + 1) for k in range(5)]) for i in range(n)]
    rows[-1]["window"]["lap_done"] = True
    rows.append(make_row(t=n, final=True, end="lap", lap_position=100.0 + 2.5 * n))
    with gzip.open(os.path.join(d, "steps.jsonl.gz"), "wt", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
    open(os.path.join(d, "events.jsonl"), "w").close()
    return d


def substep(t, edge, lane_index, lane_pos, speed, phase="idle", direction=0, lap_position=100.0, distance=0.0,
            lane_id=None, acceleration=0.0, tls0=None, lap_index=1):
    return {"t": t, "edge": edge, "lane_id": lane_id or f"{edge}_{lane_index}", "lane_index": lane_index,
            "lane_position": lane_pos, "lap_index": lap_index, "lap_position": lap_position, "distance": distance,
            "speed": speed, "acceleration": acceleration, "tls0": tls0, "phase": phase, "request_direction": direction,
            "forced": False, "crossings": {}}


def make_row(episode=1, t=0, speed=5.0, edge="R69", lane_index=1, x=0.0, y=0.0, phase="idle", heading=0.0,
             lane_id=None, lane_pos=10.0, lap_position=100.0, lap_route_index=1, distance=0.0, vehicles=(),
             pedestrians=(), tls=(), window_speeds=None, window_phases=None, window_dirs=None, lane_before=None,
             lane_after=None, edge_before=None, edge_after=None, stop_state=0, next_stop_distance=None, final=False,
             end=None, neighbors=None, request_direction=0, sim_time=None, **over):
    lane_id = lane_id or f"{edge}_{lane_index}"
    sim_time = sim_time if sim_time is not None else 100.0 + 0.5 * t
    ego = {"x": x, "y": y, "heading": heading, "speed": speed, "acceleration": 0.0, "lateral_speed": 0.0,
           "speed_without_traci": speed, "allowed_speed": 13.89, "lane_max_speed": 13.89, "max_speed": 13.89,
           "road_id": edge, "lane_id": lane_id, "lane_index": lane_index, "lane_count": 3, "lane_position": lane_pos,
           "lateral_position": 0.0, "route_index": lap_route_index, "lap_route_index": lap_route_index,
           "lap_position": lap_position, "distance": distance, "signals": 0, "stop_state": stop_state,
           "waiting_time": 0.0, "accumulated_waiting": 0.0, "lc_state_left": [0, 0], "lc_state_right": [0, 0],
           "could_change_left": True, "could_change_right": True}
    row = {
        "meta": {"run_id": "test", "episode": episode, "t": t, "sim_time": sim_time, "sim_step": int(round(sim_time * 10)),
                 "final": final, "end": end},
        "expert": {"phase": phase, "why": "", "plan_kind": None, "target_lane": None,
                   "request_direction": request_direction, "forced_stop": False, "speed_limited": False,
                   "blocker_active": False, "pdm": None},
        "scenario": {"s1": None, "s2": None, "s3": None, "s4": [], "s5": [], "s6": None, "a1": [], "injector_plan": None},
        "stopped_since": None,
        "check": {},
        "ego": ego,
        "route": {"best_lanes": [], "next_links": [], "lap_progress": {"position": lap_position, "length": 2000.0},
                  "next_turn": None},
        "tls": list(tls),
        "stops": {"stop_state": stop_state, "stops": [], "distances": [next_stop_distance] if next_stop_distance is not None else []},
        "neighbors": neighbors or {k: None for k in ("leader", "follower", "left_leader", "right_leader",
                                                     "left_follower", "right_follower")},
        "junction_foes": [],
        "vehicles": list(vehicles),
        "pedestrians": list(pedestrians),
        "window": None,
    }
    if not final:
        speeds = window_speeds or [speed] * 5
        phases = window_phases or [phase] * 5
        dirs = window_dirs or [request_direction] * 5
        lb = lane_index if lane_before is None else lane_before
        la = lb if lane_after is None else lane_after
        eb, ea = edge_before or edge, edge_after or edge
        subs, d = [], distance
        for k in range(5):
            d += speeds[k] * 0.1
            subs.append(substep(round(sim_time + 0.1 * (k + 1), 6), ea, la, lane_pos + (d - distance), speeds[k],
                                phases[k], dirs[k], lap_position + (d - distance), d, lap_index=lap_route_index))
        row["window"] = {"substeps": subs, "peak_acceleration": 0.0, "peak_jerk": 0.0, "calls": [],
                         "lane_before": lb, "lane_after": la, "edge_before": eb, "edge_after": ea,
                         "collisions": [], "teleports": [], "emergency_stops": [], "lap_done": False, "pdm_plans": []}
    for k, v in over.items():
        row[k] = v
    return row
