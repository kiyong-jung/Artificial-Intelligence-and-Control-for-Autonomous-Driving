"""행동 라벨(A안: 실제 결과 기준), 보상 재계산(BusTask.outcome과 같은 식), 에피소드 끝 규칙(설계 5절, 11.1)."""
from __future__ import annotations

import numpy as np

from .schema import DECISION_STEPS, DT, STOP_IDS

TERMINAL_WEIGHT = {"collision": "collision", "red_light": "red_light", "missed_stop": "missed_stop",
                   "arrived": "arrival", "incomplete_service": "missed_stop"}
TRUNCATING_ENDS = ("time_limit", "bus_lost", "sumo_closed", "teleport")


def accel_mps2(row) -> float:
    """판단 구간 [t, t+0.5) 동안의 실제 평균 가속도."""
    return (row["window"]["substeps"][-1]["speed"] - row["ego"]["speed"]) / (DT * DECISION_STEPS)


def normalize_accel(a: float, a_pos: float, a_neg: float):
    v = a / a_pos if a >= 0 else a / a_neg
    return (float(np.clip(v, -1.0, 1.0)), abs(v) > 1.0)


def lane_change_label(row) -> int:
    """요청 단계(request)이거나 도착 보장 정차(forced) 중인 substep이 있으면 그 마지막 0 아닌 요청 방향(+1 왼쪽), 아니면 0."""
    label = 0
    for s in row["window"]["substeps"]:
        if (s["phase"] == "request" or s.get("forced")) and s["request_direction"]:
            label = int(s["request_direction"])
    return label


def lane_change_applied(row, label: int) -> bool:
    if label == 0:
        return True
    w = row["window"]
    return w["edge_before"] == w["edge_after"] and int(np.sign(w["lane_after"] - w["lane_before"])) == label


def reward_terms(row, rep: dict, ctx, weights: dict, terminal) -> dict:
    progress = weights["progress_total"] * rep["progress_m"] / ctx.route_map.length
    service = weights["service"] * rep["services"]
    terminal_value = weights[TERMINAL_WEIGHT[terminal]] if terminal in TERMINAL_WEIGHT else None
    total = terminal_value if terminal_value is not None else progress - weights["step_cost"] + service
    return {"progress": progress, "step_cost": weights["step_cost"], "service": service,
            "terminal_value": terminal_value, "total": float(total)}


def _end(i, terminated, reason):
    return {"last_index": i, "terminated": terminated, "truncated": not terminated, "reason": reason}


def episode_end(rows: list, rep: list, cut_on_task_failure: bool = True) -> dict:
    """한 에피소드 행들에서 마지막 transition 행과 끝 원인. 앞에서부터 처음 나오는 사건을 쓴다."""
    for i, row in enumerate(rows):
        w = row.get("window")
        if w is None:
            end = row["meta"].get("end")
            return _end(i - 1, False, end if end in TRUNCATING_ENDS else (end or "data_end"))
        if any(x.get("ego") for x in w["teleports"]):
            return _end(i - 1, False, "teleport")
        if any(c.get("ego") for c in w["collisions"]):
            return _end(i, True, "collision")
        if cut_on_task_failure and rep[i]["red_violation"]:
            return _end(i, True, "red_light")
        if cut_on_task_failure and rep[i]["missed_now"]:
            return _end(i, True, "missed_stop")
        if w["lap_done"]:
            served = len(rep[i]["completed"]) + rep[i]["services"]
            return _end(i, True, "arrived" if served >= len(STOP_IDS) else "incomplete_service")
    return _end(len(rows) - 1, False, "data_end")


def accel_report(values, candidates) -> dict:
    a = np.asarray(list(values), dtype=float)
    out = {"n": int(a.size)}
    if a.size == 0:
        return out
    for p in (1, 5, 50, 95, 99):
        out[f"p{p}"] = float(np.percentile(a, p))
    out["min"], out["max"] = float(a.min()), float(a.max())
    out["clip_ratio"] = {f"{ap}/{an}": float(np.mean([normalize_accel(x, ap, an)[1] for x in a])) for ap, an in candidates}
    return out
