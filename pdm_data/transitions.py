"""원시 수집 폴더 -> 학습용 transition(설계 11절). data_colum.md 필수 23열과 추가 열을 만든다."""
from __future__ import annotations

import collections
import copy
import json
import math
import os

import numpy as np

from env.bus_task import REWARDS

from .features import input_design_features
from .interaction import row_interaction
from .labels import (accel_mps2, episode_end, lane_change_applied, lane_change_label, normalize_accel, reward_terms)
from .lap_context import load_lap_context
from .obs93 import obs93
from .recorder import read_steps
from .replay import replay_episode
from .schema import DETECT_RADIUS_M

STATE_BLOCKS = ("ego", "route", "tls", "stops", "neighbors", "junction_foes", "vehicles", "pedestrians")
COMFORT_LIMITS = {"acceleration_limit": 1.5, "jerk_limit": 2.0}       # env/road_config.py 버스 comfort와 같다
LANE_REL = {2: "left2", 1: "left1", 0: "current", -1: "right1", -2: "right2"}


def load_meta(run_dir):
    with open(os.path.join(run_dir, "metadata.json"), encoding="utf-8") as f:
        return json.load(f)


def episodes(rows):
    out = collections.OrderedDict()
    for r in rows:
        out.setdefault(r["meta"]["episode"], []).append(r)
    return list(out.values())


def _cut_time(ep, max_lap_s):
    """바퀴 시작에서 max_lap_s가 지난 첫 행을 final(time_limit)로 바꾸고 그 뒤를 버린다."""
    t0 = ep[0]["meta"]["sim_time"]
    for k, r in enumerate(ep):
        if r["meta"]["sim_time"] - t0 >= max_lap_s - 1e-9:
            last = copy.deepcopy(r)
            last["meta"].update(final=True, end="time_limit")
            last["window"] = None
            return ep[:k] + [last]
    return ep


def _state(row):
    return {k: row[k] for k in STATE_BLOCKS}


def _relative(ego, o):
    h = math.radians(ego["heading"])
    hx, hy = math.sin(h), math.cos(h)
    dx, dy = o["x"] - ego["x"], o["y"] - ego["y"]
    a = math.radians(o.get("heading", 0.0))
    vx, vy = o["speed"] * math.sin(a) - ego["speed"] * hx, o["speed"] * math.cos(a) - ego["speed"] * hy
    return dx * hx + dy * hy, -dx * hy + dy * hx, vx * hx + vy * hy, -vx * hy + vy * hx


def detected(row):
    ego = row["ego"]
    vehs, peds = [], []
    for v in row["vehicles"]:
        if math.hypot(v["x"] - ego["x"], v["y"] - ego["y"]) > DETECT_RADIUS_M:
            continue
        lon, lat, vlon, vlat = _relative(ego, v)
        rel = "other"
        if v.get("road_id") == ego["road_id"] and v.get("lane_index") is not None:
            rel = LANE_REL.get(v["lane_index"] - ego["lane_index"], "other")
        vehs.append({"id": v["id"], "x": v["x"], "y": v["y"], "relative_longitudinal_position": lon,
                     "relative_lateral_position": lat, "relative_longitudinal_speed": vlon,
                     "relative_lateral_speed": vlat,
                     "relative_heading": (ego["heading"] - v["heading"] + 180.0) % 360.0 - 180.0,
                     "route_distance": v.get("route_distance"), "lane_relation": rel,
                     "left_signal": bool(v.get("signals", 0) & 2), "right_signal": bool(v.get("signals", 0) & 1)})
    for p in row["pedestrians"]:
        if math.hypot(p["x"] - ego["x"], p["y"] - ego["y"]) > DETECT_RADIUS_M:
            continue
        lon, lat, vlon, vlat = _relative(ego, p)
        peds.append({"id": p["id"], "relative_longitudinal_position": lon, "relative_lateral_position": lat,
                     "relative_longitudinal_speed": vlon, "relative_lateral_speed": vlat})
    return vehs, peds


def build_transitions(run_dir: str, a_pos: float, a_neg: float, cut_on_task_failure: bool = True,
                      max_lap_s=None, with_replay: bool = False):
    ctx = load_lap_context(run_dir)
    meta = load_meta(run_dir)
    programs = meta.get("tls_programs", {})
    dims = meta["expert"]["ego_dims"]
    rows = list(read_steps(run_dir))
    out, clipped, ends, dropped, replays = [], 0, collections.Counter(), 0, []
    for ep in episodes(rows):
        if max_lap_s is not None:
            ep = _cut_time(ep, max_lap_s)
        rep = replay_episode(ep, ctx)
        replays.append((ep, rep))
        end = episode_end(ep, rep, cut_on_task_failure)
        last = end["last_index"]
        dropped += len(ep) - (last + 2 if last + 1 < len(ep) else last + 1)
        if last < 0:
            continue
        ends[end["reason"]] += 1
        n_states = min(last + 2, len(ep))
        obs = [obs93(ep[i], ctx, rep[i]).tolist() for i in range(n_states)]
        feats, stage_state, prev_label = [], {}, None
        labels = []
        for i in range(n_states):
            feats.append(input_design_features(ep[i], ep[i - 1] if i else None, ctx, programs, dims, prev_label,
                                               stage_state).tolist())
            if i <= last:
                a = accel_mps2(ep[i])
                an, was_clipped = normalize_accel(a, a_pos, a_neg)
                lc = lane_change_label(ep[i])
                labels.append((a, an, was_clipped, lc))
                prev_label = (an, lc)
        for i in range(last + 1):
            row, nxt = ep[i], ep[min(i + 1, n_states - 1)]
            a, an, was_clipped, lc = labels[i]
            clipped += int(was_clipped)
            is_last = i == last
            terminal = end["reason"] if is_last and end["terminated"] else None
            rt = reward_terms(row, rep[i], ctx, REWARDS, terminal)
            vehs, peds = detected(row)
            w = row["window"]
            pdm = row["expert"].get("pdm") or {}
            out.append({
                "episode": row["meta"]["episode"], "t": row["meta"]["t"], "sim_time": row["meta"]["sim_time"],
                "observation": obs[i], "action_raw": [an, float(lc)],
                "action": {"accel_raw": an, "lane_change_raw": float(lc), "lane_change": lc,
                           "lane_change_applied": lane_change_applied(row, lc)},
                "action_extra": {"accel_mps2": a, "accel_clipped": was_clipped},
                "reward": rt["total"], "reward_terms": rt,
                "terminated": bool(is_last and end["terminated"]), "truncated": bool(is_last and end["truncated"]),
                "state": _state(row), "detected_vehicles": vehs, "detected_pedestrians": peds,
                "event": {"collided": any(c.get("ego") for c in w["collisions"]),
                          "arrived": bool(is_last and end["reason"] == "arrived"),
                          "lane_before": w["lane_before"], "lane_after": w["lane_after"],
                          "termination_reason": end["reason"] if is_last else ""},
                "comfort": {"peak_acceleration": w["peak_acceleration"], "peak_jerk": w["peak_jerk"],
                            "acceleration_violation": w["peak_acceleration"] > COMFORT_LIMITS["acceleration_limit"],
                            "jerk_violation": w["peak_jerk"] > COMFORT_LIMITS["jerk_limit"]},
                "expert": {"phase": row["expert"]["phase"], "request_direction": row["expert"]["request_direction"],
                           "plan_kind": row["expert"]["plan_kind"], "pdm_fraction": pdm.get("fraction"),
                           "pdm_v0": pdm.get("v0")},
                "scenario": row["scenario"], "interaction": row_interaction(row, ctx, dims),
                "features": feats[i], "next_features": feats[min(i + 1, n_states - 1)],
                "next_observation": obs[min(i + 1, n_states - 1)], "next_state": _state(nxt),
                "check": row.get("check", {}),
            })
    stats = {"rows_total": len(rows), "transitions": len(out), "dropped_rows": dropped,
             "accel_clip_ratio": clipped / len(out) if out else 0.0, "end_reasons": dict(ends)}
    if with_replay:
        stats["_replays"] = replays
    return out, stats


def obs_online_check(transitions) -> dict:
    diffs = [float(np.abs(np.asarray(r["observation"]) - np.asarray(r["check"]["obs93_online"])).max())
             for r in transitions if r.get("check", {}).get("obs93_online")]
    return {"rows": len(diffs), "max_abs_diff": max(diffs) if diffs else None}
