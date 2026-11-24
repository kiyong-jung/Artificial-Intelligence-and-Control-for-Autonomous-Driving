"""전처리에서 하는 S5·S6 판정(설계 8절). 수집 중에 매 스텝 전체 차량·보행자를 훑지 않도록 원시 행으로 판정한다.
규칙은 scripts/measure_s1_s6.py와 같다.

S5: 골목 도로(feeder)에서 보였던 차가 S5_WINDOW_S(15 s) 안에 버스와 같은 도로의 앞 NEAR_M(50 m) 안에 들어오면 끼어들기 한 번.
    수집 중 판정처럼 0.5 s마다 본다(원시 행 간격). 원시 행은 버스 반경 150 m 안 차량만 담는다: 끼어드는 차는 합류 직전에 이 반경 안이다.
S6: 버스가 횡단보도 50 m 앞부터 버스 뒤끝이 횡단보도를 벗어날 때까지, 그 횡단보도에 있던 보행자가 있으면 통과 한 번.
    보행자 점유 구간은 substep의 crossings(0.1 s), 버스 점유 구간은 substep의 lap_position(0.1 s)으로 잰다.
"""
from __future__ import annotations

from traffic.metrics import pet

from .lap_context import LapContext
from .schema import CROSSING_WIDTH_M, BUS_LENGTH_M, NEAR_M, S5_WINDOW_S, S6_AFTER_M, S6_BEFORE_M

S5_FOLLOW_S = 10.0          # 끼어든 뒤 이 시간 동안 버스와의 최소 간격·버스 최소 가속도를 본다


def _event(kind, lap, t_start, t_end, ids, injected, detail):
    return {"kind": kind, "lap": lap, "t_start": t_start, "t_end": t_end, "ids": list(ids), "injected": injected,
            "detail": detail}


def s5_events(rows: list, feeders, near: float = NEAR_M, window: float = S5_WINDOW_S,
              follow_s: float = S5_FOLLOW_S) -> list:
    feeders = set(feeders)
    if not feeders:
        return []
    seen, cut, found = {}, set(), []
    for i, row in enumerate(rows):
        t, ego = row["meta"]["sim_time"], row["ego"]
        for v in row["vehicles"]:
            vid = v["id"]
            if v["road_id"] in feeders:
                seen[vid] = (t, v["road_id"])
            elif vid in seen and vid not in cut and t - seen[vid][0] <= window and v["road_id"] == ego["road_id"] \
                    and 0 < v["lane_position"] - ego["lane_position"] <= near:
                cut.add(vid)
                found.append((i, vid, seen[vid], v["lane_position"] - ego["lane_position"]))
    out = []
    for i, vid, (t_seen, alley), gap in found:
        row = rows[i]
        min_gap, min_acc = gap, row["ego"]["acceleration"]
        for r in rows[i:]:
            if r["meta"]["sim_time"] - row["meta"]["sim_time"] > follow_s:
                break
            min_acc = min(min_acc, r["ego"]["acceleration"])
            v = next((x for x in r["vehicles"] if x["id"] == vid), None)
            if v is not None and v["road_id"] == r["ego"]["road_id"]:
                min_gap = min(min_gap, v["lane_position"] - r["ego"]["lane_position"])
        out.append(_event("s5_cutin", row["meta"]["episode"], t_seen, row["meta"]["sim_time"], [vid],
                          vid.lower().startswith("inj_s5"),
                          {"alley": alley, "gap": gap, "bus_speed": row["ego"]["speed"], "min_gap": min_gap,
                           "bus_min_accel": min_acc}))
    return out


def s6_events(rows: list, ctx: LapContext) -> list:
    out, state = [], {}

    def close(cid, t, lap):
        st = state.pop(cid)
        if not st["peds"]:
            return
        bus_iv = (st["bus_in"], st["bus_out"]) if st["bus_in"] is not None else None
        pets = {pid: pet((iv[0], iv[1] if iv[1] is not None else t), bus_iv) for pid, iv in st["peds"].items()}
        vals = [p for p in pets.values() if p is not None]
        out.append(_event("s6_pass", lap, st["t"], t, list(st["peds"]),
                          any(p.startswith("inj_s6_") for p in st["peds"]),
                          {"crossing": cid, "bus_interval": bus_iv, "ped_intervals": st["peds"], "pet": pets,
                           "min_pet": min(vals) if vals else None, "bus_min_speed": st["min_speed"]}))

    for row in rows:
        lap = row["meta"]["episode"]
        for s in (row.get("window") or {}).get("substeps", []):
            pos, t = s["lap_position"], s["t"]
            if pos is None:
                continue
            on_now = s.get("crossings") or {}
            for at, cid in ctx.route_map.crossings:
                st = state.get(cid)
                if st is None and -S6_BEFORE_M <= pos - at <= S6_AFTER_M and pos <= at:
                    st = state[cid] = {"t": t, "bus_in": None, "bus_out": None, "peds": {}, "min_speed": s["speed"]}
                if st is None:
                    continue
                st["min_speed"] = min(st["min_speed"], s["speed"])
                if st["bus_in"] is None and pos >= at:
                    st["bus_in"] = t
                peds = on_now.get(cid) or []
                for pid in peds:
                    st["peds"].setdefault(pid, [t, None])
                for pid, iv in st["peds"].items():
                    if iv[1] is None and pid not in peds:
                        iv[1] = t
                if pos - BUS_LENGTH_M > at + CROSSING_WIDTH_M or not (-NEAR_M - 1.0 <= pos - at <= 100.0):
                    st["bus_out"] = t
                    close(cid, t, lap)
    return out
