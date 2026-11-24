"""상호작용 지표(설계 11.3): 선행·후행 슬롯마다 THW·TTC·DRAC·여유, 50 m 안 보행자의 도달 시간."""
from __future__ import annotations

import math

from .schema import DETECT_RADIUS_M

SLOTS = ("leader", "follower", "left_leader", "left_follower", "right_leader", "right_follower")
DEFAULT_MIN_GAP = 2.5


def pair_metrics(gap: float, v_follow: float, v_lead: float) -> dict:
    closing = v_follow - v_lead
    g = max(gap, 0.1)
    return {"thw": gap / v_follow if v_follow >= 0.1 else None,
            "ttc": gap / closing if closing > 0 else None,
            "inv_ttc": max(0.0, closing) / g,
            "drac": closing * closing / (2.0 * g) if closing > 0 else 0.0}


def bumper_gap(slot: dict, kind: str, ego_min_gap: float, follower_min_gap) -> float:
    """TraCI 간격은 뒤차 minGap을 뺀 값이다. 선행 슬롯은 ego minGap, 후행 슬롯은 뒤차 minGap을 더한다."""
    add = ego_min_gap if kind == "leader" else (follower_min_gap if follower_min_gap is not None else DEFAULT_MIN_GAP)
    return slot["gap_raw"] + add


def slot_pair(row, name, ego_dims):
    """슬롯 하나: (범퍼 간격, 뒤쪽 차 속도, 앞쪽 차 속도, 상대 차량 dict 또는 None). 빈 슬롯이면 None."""
    slot = row["neighbors"].get(name)
    if slot is None:
        return None
    other = next((v for v in row["vehicles"] if v["id"] == slot["id"]), None)
    v_ego = row["ego"]["speed"]
    v_other = other["speed"] if other else v_ego
    if name.endswith("leader"):
        return bumper_gap(slot, "leader", ego_dims["min_gap"], None), v_ego, v_other, other, slot
    return bumper_gap(slot, "follower", ego_dims["min_gap"], other.get("min_gap") if other else None), v_other, v_ego, \
        other, slot


def ego_frame(row, x, y):
    ego = row["ego"]
    h = math.radians(ego["heading"])
    hx, hy = math.sin(h), math.cos(h)
    dx, dy = x - ego["x"], y - ego["y"]
    return dx * hx + dy * hy, -dx * hy + dy * hx


def row_interaction(row, ctx, ego_dims) -> dict:
    out = {}
    for name in SLOTS:
        p = slot_pair(row, name, ego_dims)
        if p is None:
            out[name] = None
            continue
        gap, v_back, v_front, other, slot = p
        m = pair_metrics(gap, v_back, v_front)
        sg = slot.get("secure_gap")
        m["margin"] = gap - sg if sg is not None else None
        m.update(id=slot["id"], gap=gap)
        out[name] = m
    v_ego = row["ego"]["speed"]
    peds = []
    for q in row["pedestrians"]:
        lon, lat = ego_frame(row, q["x"], q["y"])
        if math.hypot(lon, lat) > DETECT_RADIUS_M:
            continue
        t_ego = lon / max(v_ego, 0.1) if lon > 0 else None
        t_ped = abs(lat) / max(q["speed"], 0.1)
        peds.append({"id": q["id"], "t_ego": t_ego, "t_ped": t_ped, "dt": t_ped - t_ego if t_ego is not None else None})
    out["pedestrians"] = peds
    sc = row.get("scenario") or {}
    out["n_active"] = len(sc.get("a1") or []) + len(sc.get("s5") or []) + len(((sc.get("s6") or {}).get("peds")) or [])
    return out
