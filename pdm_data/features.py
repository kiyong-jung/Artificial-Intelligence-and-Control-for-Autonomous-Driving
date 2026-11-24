"""입력 설계 특징(SUMO_AV_input_설계.md, 설계 11.2). 원시 행에서 130차원 [-1, 1] 벡터를 만든다.

묶음 순서: ego(7) · 차선 변경 가능(2) · 경로(9) · 정류장(7) · 신호(10) · lane-slot(6x6) · 안전 여유(5) · 골목·교차 차량(3x7)
· 보행자(3x11). 정규화: 전방 160 m, 후방 80 m, 속도 vmax, 가속 3 m/s², 정차 60 s, 신호 120 s. 빈 슬롯은 전방 +1·후방 -1,
속도 0, presence 0이고, 교차 차량 빈 칸은 phantom(전방 끝, 제한속도)이다. 신호 정보가 없으면 ttg·ttr은 -1이다.
"""
from __future__ import annotations

import math

import numpy as np

from .interaction import SLOTS, ego_frame, pair_metrics, slot_pair
from .schema import DETECT_RADIUS_M, SERVICE_SECONDS

FWD, REAR, ACC, DWELL_CAP, SIGNAL_CAP, FOE_RANGE, BRAKE = 160.0, 80.0, 3.0, 60.0, 120.0, 100.0, 3.0
N_FOES, N_PEDS = 3, 3
SLOT_F = ("present", "ds", "dv", "da", "vy", "inv_ttc")
FOE_F = ("present", "phantom", "d_other", "v_other", "d_ego", "dt_arrival", "has_priority")
PED_F = ("present", "d_crossing", "lateral", "speed", "cos", "sin", "on_crossing", "intent", "waiting", "t_ped", "t_ego")
STAGES = ("cruise", "approach", "dwell", "depart")

FEATURE_NAMES = (
    ["ego_speed", "ego_accel", "ego_prev_accel", "ego_prev_lane_change", "ego_lc_in_progress", "ego_lane_index",
     "ego_lane_count"]
    + ["lc_could_left", "lc_could_right"]
    + ["route_best_offset", "route_cont_current", "route_cont_left", "route_cont_right", "route_dir_left",
       "route_dir_straight", "route_dir_right", "route_dist_turn", "route_progress"]
    + ["stop_distance", "stop_lanes_to"] + [f"stop_stage_{s}" for s in STAGES] + ["stop_remaining_dwell"]
    + ["tls_present", "tls_distance", "tls_r", "tls_y", "tls_G", "tls_g", "tls_ttg", "tls_ttr", "tls_dilemma_time",
       "tls_dilemma_dist"]
    + [f"slot_{s}_{f}" for s in SLOTS for f in SLOT_F]
    + ["safe_thw", "safe_margin", "safe_rear_inv_ttc", "safe_rear_req_decel", "safe_queue"]
    + [f"foe{k}_{f}" for k in range(N_FOES) for f in FOE_F]
    + [f"ped{k}_{f}" for k in range(N_PEDS) for f in PED_F])


def _c(x):
    return float(np.clip(x, -1.0, 1.0))


def time_to_green_red(phases: list, phase: int, time_to_switch: float, link_index: int):
    """현재 phase부터 순환하며 이 link가 녹색(G/g)·적색(r/R)이 되기까지의 시간. 지금 그 색이면 0, 없으면 inf."""
    n = len(phases)

    def ch(i):
        s = phases[i % n]["state"]
        return s[link_index] if link_index < len(s) else "r"
    cur = ch(phase)
    ttg = 0.0 if cur in "Gg" else None
    ttr = 0.0 if cur in "rR" else None
    acc = float(time_to_switch)
    for k in range(1, n + 1):
        c = ch(phase + k)
        if ttg is None and c in "Gg":
            ttg = acc
        if ttr is None and c in "rR":
            ttr = acc
        acc += phases[(phase + k) % n]["duration"]
    return (ttg if ttg is not None else math.inf, ttr if ttr is not None else math.inf)


def _next_stop_distance(row):
    d = row["stops"]["distances"]
    return d[0] if d else None


def stage(row, prev_stage, since_dep_s: float) -> str:
    ss = row["ego"]["stop_state"]
    if ss & 1 and ss & 16:
        return "dwell"
    if prev_stage in ("dwell", "depart") and since_dep_s <= 10.0:
        return "depart"
    d = _next_stop_distance(row)
    if d is not None and d <= 100.0:
        return "approach"
    return "cruise"


def _ego(row, prev_row, ctx, prev_label):
    e = row["ego"]
    moved = bool(prev_row and prev_row.get("window") and prev_row["window"]["lane_after"] != prev_row["window"]["lane_before"])
    pa, pl = prev_label if prev_label is not None else (0.0, 0)
    return [_c(e["speed"] / ctx.vmax), _c(e["acceleration"] / ACC), _c(pa), _c(pl),
            float(abs(e["lateral_speed"]) > 0.05 or moved), _c(e["lane_index"] / max(e["lane_count"] - 1, 1)),
            _c(e["lane_count"] / 5.0)]


def _route(row, ctx):
    e, r = row["ego"], row["route"]
    by_lane = {b["lane_id"]: b for b in r["best_lanes"]}
    cur = by_lane.get(e["lane_id"])
    edge, idx = e["road_id"], e["lane_index"]
    cont = [float(bool(by_lane.get(f"{edge}_{i}", {}).get("allows_continuation"))) for i in (idx, idx + 1, idx - 1)]
    d = r["next_links"][0]["direction"] if r["next_links"] else ""
    nt = r["next_turn"] or {}
    dist = nt.get("distance_to_turn")
    pos = e["lap_position"]
    return [_c(cur["offset"] / 3.0) if cur else 0.0, *cont, float(d in "lL" and d != ""), float(d == "s"),
            float(d in "rR" and d != ""), _c(dist / FWD) if dist is not None else 1.0,
            _c(pos / ctx.route_map.length) if pos is not None else 0.0]


def _stop(row, stage_state):
    e = row["ego"]
    now, ss = row["meta"]["sim_time"], e["stop_state"]
    prev = stage_state.get("stage")
    if prev == "dwell" and not (ss & 1 and ss & 16):
        stage_state["dep_t"], stage_state["dep_dist"] = now, e["distance"]
    since = math.inf
    if stage_state.get("dep_t") is not None and e["distance"] - stage_state["dep_dist"] <= 30.0:
        since = now - stage_state["dep_t"]
    s = stage(row, prev, since)
    stage_state["stage"] = s
    d = _next_stop_distance(row)
    stops = row["stops"]["stops"]
    lanes_to = 0.0
    if stops and stops[0].get("lane"):
        lanes_to = _c((int(stops[0]["lane"].rsplit("_", 1)[1]) - e["lane_index"]) / 3.0)
    remaining = 0.0
    if s == "dwell" and row.get("stopped_since") is not None:
        remaining = _c(max(0.0, SERVICE_SECONDS - (now - row["stopped_since"])) / DWELL_CAP)
    return [_c(d / FWD) if d is not None else 1.0, lanes_to, *[float(s == k) for k in STAGES], remaining]


def _tls(row, programs):
    if not row["tls"]:
        return [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0]
    t = row["tls"][0]
    st, d, v = t["state"], t["distance"], row["ego"]["speed"]
    ttg = ttr = None
    logics = programs.get(t["id"]) or []
    logic = next((lg for lg in logics if lg.get("program") == t.get("program")), logics[0] if logics else None)
    if logic and logic.get("phases") and t.get("phase") is not None and t.get("time_to_switch") is not None:
        ttg, ttr = time_to_green_red(logic["phases"], t["phase"], t["time_to_switch"], t["link_index"])
    norm = (lambda x: -1.0 if x is None else (1.0 if math.isinf(x) else _c(x / SIGNAL_CAP)))
    dil_t = 0.0 if ttr is None or math.isinf(ttr) else _c((d / max(v, 0.1) - ttr) / SIGNAL_CAP)
    return [1.0, _c(d / FWD), float(st in "rRu"), float(st in "yY"), float(st == "G"), float(st == "g"),
            norm(ttg), norm(ttr), dil_t, _c((v * v / (2 * BRAKE) - d) / FWD)]


def _slots(row, ctx, ego_dims):
    e, out = row["ego"], []
    for name in SLOTS:
        p = slot_pair(row, name, ego_dims)
        lead = name.endswith("leader")
        if p is None:
            out += [0.0, 1.0 if lead else -1.0, 0.0, 0.0, 0.0, 0.0]
            continue
        gap, v_back, v_front, other, _ = p
        o = other or {}
        out += [1.0, _c(gap / FWD) if lead else _c(-gap / REAR),
                _c((o.get("speed", e["speed"]) - e["speed"]) / ctx.vmax),
                _c((o.get("acceleration", e["acceleration"]) - e["acceleration"]) / ACC),
                _c(o.get("lateral_speed", 0.0) / 2.0), _c(pair_metrics(gap, v_back, v_front)["inv_ttc"])]
    return out


def _safety(row, ctx, ego_dims):
    e = row["ego"]
    thw, margin, rear_inv, rear_dec = 1.0, 1.0, 0.0, 0.0
    p = slot_pair(row, "leader", ego_dims)
    if p is not None:
        m = pair_metrics(p[0], p[1], p[2])
        thw = _c(m["thw"] / 4.0) if m["thw"] is not None else 1.0
        if p[4].get("secure_gap") is not None:
            margin = _c((p[0] - p[4]["secure_gap"]) / FWD)
    f = slot_pair(row, "follower", ego_dims)
    if f is not None:
        m = pair_metrics(f[0], f[1], f[2])
        rear_inv, rear_dec = _c(m["inv_ttc"]), _c(m["drac"] / BRAKE)
    queue = sum(1 for v in row["vehicles"] if v.get("route_distance") is not None and 0 < v["route_distance"] <= FWD
                and v.get("lane_index") == e["lane_index"])
    return [thw, margin, rear_inv, rear_dec, _c(queue / 10.0)]


def _foes(row, ctx):
    e = row["ego"]
    speed = {v["id"]: v["speed"] for v in row["vehicles"]}
    foes = sorted(row["junction_foes"], key=lambda f: -(1 - math.hypot(f["foe_dist"] / FOE_RANGE,
                                                                        f["ego_dist"] / FOE_RANGE) / math.sqrt(2)))
    out = []
    for k in range(N_FOES):
        if k >= len(foes):
            out += [0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0]
            continue
        f = foes[k]
        vo = speed.get(f["foe_id"], 0.0)
        dt = f["foe_dist"] / max(vo, 0.1) - f["ego_dist"] / max(e["speed"], 0.1)
        out += [1.0, 0.0, _c(f["foe_dist"] / FOE_RANGE), _c(vo / ctx.vmax), _c(f["ego_dist"] / FOE_RANGE), _c(dt / 10.0),
                0.0 if f.get("ego_response") else 1.0]
    return out


def _peds(row, ctx):
    e = row["ego"]
    pos = e["lap_position"]
    dc = ctx.route_map.ahead(pos)["distance_to_crossing"] if pos is not None else None
    near = []
    for q in row["pedestrians"]:
        lon, lat = ego_frame(row, q["x"], q["y"])
        dist = math.hypot(lon, lat)
        if dist <= DETECT_RADIUS_M:
            near.append((dist, q["id"], q, lon, lat))
    near.sort(key=lambda x: (x[0], x[1]))
    out = []
    for k in range(N_PEDS):
        if k >= len(near):
            out += [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0]
            continue
        _, _, q, lon, lat = near[k]
        rel = math.radians(q["heading"] - e["heading"])
        t_ego = lon / max(e["speed"], 0.1) if lon > 0 else None
        t_ped = abs(lat) / max(q["speed"], 0.1)
        out += [1.0, _c(dc / DETECT_RADIUS_M) if dc is not None else 1.0, _c(lat / 10.0), _c(q["speed"] / 2.0),
                math.cos(rel), math.sin(rel), float(bool(q.get("on_crossing"))),
                float(q.get("next_edge") in ctx.crossing_ids), _c(q.get("waiting_time", 0.0) / 30.0), _c(t_ped / 10.0),
                _c(t_ego / 10.0) if t_ego is not None else 1.0]
    return out


def input_design_features(row, prev_row, ctx, programs, ego_dims, prev_label, stage_state: dict) -> np.ndarray:
    """prev_label = 직전 행의 (accel_raw, lane_change) 또는 None. stage_state는 같은 에피소드 안에서 이어 쓰는 dict."""
    v = (_ego(row, prev_row, ctx, prev_label)
         + [float(row["ego"]["could_change_left"]), float(row["ego"]["could_change_right"])]
         + _route(row, ctx) + _stop(row, stage_state) + _tls(row, programs) + _slots(row, ctx, ego_dims)
         + _safety(row, ctx, ego_dims) + _foes(row, ctx) + _peds(row, ctx))
    assert len(v) == len(FEATURE_NAMES)
    return np.asarray(v, dtype=np.float32)
