"""정차·신호 판정 재생(설계 11.1). env/bus_task.py의 ServiceTracker와 crossed_red를 원래와 같은 0.1 s 간격으로 다시 돌린다.

EpisodeReplay는 한 행씩 넣는 방식이라 closed-loop 평가에서 판단마다 쓸 수 있다. replay_episode는 그것을 에피소드 전체에 돌린다.
"""
from __future__ import annotations

from env.bus_task import ServiceTracker, crossed_red

from .lap_context import LapContext
from .schema import DT, SERVICE_SECONDS


def _snapshot_pose(row):
    tls = row["tls"][0] if row["tls"] else None
    light = (tls["id"], tls["link_index"], tls["distance"], tls["state"]) if tls else None
    return {"tls": light, "edge": row["ego"]["road_id"], "distance": row["ego"]["distance"]}


class EpisodeReplay:
    """한 에피소드의 정차 판정 상태. state()는 다음에 넣을 행의 스냅숏 시점 상태(관측 v2의 tracker_state)다."""

    def __init__(self, ctx: LapContext):
        self.tracker = ServiceTracker(ctx.service_stops, SERVICE_SECONDS)
        self.prev = None

    def state(self) -> dict:
        t = self.tracker
        return {"tracker_index": t.index, "dwell": t.dwell, "completed": list(t.completed), "missed": t.missed}

    def step(self, row) -> dict:
        """행 하나(스냅숏 + window)를 넣는다. 반환: 스냅숏 시점 상태 + 그 window 동안 일어난 일."""
        if self.prev is None:
            self.prev = _snapshot_pose(row)
        state = self.state()
        services, red, missed_now, progress = 0, False, False, 0.0
        for s in (row.get("window") or {}).get("substeps", []):
            now = {"tls": tuple(s["tls0"]) if s["tls0"] else None, "edge": s["edge"], "distance": s["distance"]}
            progress += max(0.0, now["distance"] - self.prev["distance"])
            red = red or crossed_red(self.prev, now)
            if s["lap_position"] is not None:
                ev = self.tracker.update(s["lap_position"], s["lane_id"], s["speed"], DT)
                services += int(ev == "service_completed")
                missed_now = missed_now or ev == "missed_stop"
            self.prev = now
        state.update(services=services, red_violation=red, missed_now=missed_now, progress_m=progress)
        return state


def replay_episode(rows: list, ctx: LapContext) -> list:
    """행마다: 스냅숏 시점의 정차 판정 상태와 그 행 window 동안 일어난 일."""
    rep = EpisodeReplay(ctx)
    return [rep.step(row) for row in rows]
