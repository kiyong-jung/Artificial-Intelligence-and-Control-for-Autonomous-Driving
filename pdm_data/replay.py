"""정차·신호 판정 재생(설계 11.1). env/bus_task.py의 ServiceTracker와 crossed_red를 원래와 같은 0.1 s 간격으로 다시 돌린다."""
from __future__ import annotations

from env.bus_task import ServiceTracker, crossed_red

from .lap_context import LapContext
from .schema import DT, SERVICE_SECONDS


def _snapshot_pose(row):
    tls = row["tls"][0] if row["tls"] else None
    light = (tls["id"], tls["link_index"], tls["distance"], tls["state"]) if tls else None
    return {"tls": light, "edge": row["ego"]["road_id"], "distance": row["ego"]["distance"]}


def replay_episode(rows: list, ctx: LapContext) -> list:
    """행마다: 스냅숏 시점의 정차 판정 상태와 그 행 window 동안 일어난 일."""
    tracker = ServiceTracker(ctx.service_stops, SERVICE_SECONDS)
    prev, out = None, []
    for row in rows:
        if prev is None:
            prev = _snapshot_pose(row)
        state = {"tracker_index": tracker.index, "dwell": tracker.dwell, "completed": list(tracker.completed),
                 "missed": tracker.missed}
        services, red, missed_now, progress = 0, False, False, 0.0
        for s in (row.get("window") or {}).get("substeps", []):
            now = {"tls": tuple(s["tls0"]) if s["tls0"] else None, "edge": s["edge"], "distance": s["distance"]}
            progress += max(0.0, now["distance"] - prev["distance"])
            red = red or crossed_red(prev, now)
            if s["lap_position"] is not None:
                ev = tracker.update(s["lap_position"], s["lane_id"], s["speed"], DT)
                services += int(ev == "service_completed")
                missed_now = missed_now or ev == "missed_stop"
            prev = now
        state.update(services=services, red_violation=red, missed_now=missed_now, progress_m=progress)
        out.append(state)
    return out
