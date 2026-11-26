"""Closed-loop 평가: 정책 판단(관측 v2 -> 행동)과, 기록한 원시 행에서 지표를 뽑는 계산.

판단은 학습 데이터와 같은 정의를 따른다.
  관측  : pdm_data.obs_v2.obs_v2(스냅숏 행, V2Context, 정차 판정 상태, 직전 행동)
          관측 33차원 MLP 정책(run_config input "obs33")이면 pdm_data.obs33.obs33(스냅숏 행, Obs33Context, 정차 판정 상태).
          이 정책은 차로 3값을 내고, 평가 버스는 lane_values=3으로 받는다.
  직전 행동: 직전 판단 구간에서 실제로 일어난 것(labels.accel_mps2 정규화, lane_change_label5). 학습의 prev_action_raw와 같다.
  끝    : episode_end와 같은 순서(순간이동은 기록기가 끝낸다, 충돌, 신호 위반, 정류장 놓침, 바퀴 완료)
"""
from __future__ import annotations

import dataclasses
import math
import os

import numpy as np

from .closed_loop_extra import (STALL_S, StallWatch, background_collisions, failure_location, interaction_counts,
                                longest_stall, outcome, overlap_metrics)
from .interaction import ego_frame, row_interaction
from .labels import accel_mps2, episode_end, lane_change_label5, normalize_accel
from .lap_context import lane_usable, load_lap_context
from .obs33 import Obs33Context, obs33
from .obs_v2 import V2Context, obs_v2
from .recorder import CollectionDone, read_steps
from .replay import EpisodeReplay, replay_episode
from .scenario_offline import s5_events, s6_events
from .schema import BUS, DT, SERVICE_SECONDS
from .ssm import parse_ssm
from .transitions import COMFORT_LIMITS, episodes, load_meta

DEFAULT_ACCEL_SCALE = {"pos": 1.5, "neg": 3.0}
STOPPED_MPS = 0.1          # ServiceTracker와 같은 정지 기준
DEPART_MPS = 0.5           # 정차를 마친 뒤 이 속도를 넘으면 출발로 본다
HARD_BRAKE_MPS2 = -3.0     # 버스 vType decel
TTC_RISK_S = 1.5
PED_PATH_AHEAD_M, PED_PATH_HALF_WIDTH_M = 20.0, 3.0
OSCILLATION_S = 10.0       # 차선 변경 뒤 이 시간 안에 반대쪽으로 되돌아오면 갈팡질팡 한 번


def plan_dicts(pilot) -> list:
    """pilot.plans를 metadata route.lane_plans와 같은 모양으로(V2Context가 읽는다)."""
    return [{k: (sorted(v) if isinstance(v, frozenset) else v) for k, v in dataclasses.asdict(p).items()}
            for p in pilot.plans]


# ---------------------------------------------------------------- 정책 불러오기
def load_policy(model_dir: str, device: str = "cpu"):
    """scripts/train_ego_attention_bc.py 출력 폴더 -> (정책 함수 obs -> (정규화 가감속, 차로 5값), 정보).
    정책 함수는 결정적이다(가감속 구간 argmax, 차로 argmax).
    차로 5값: +2/-2 좌/우로 옮김, +1/-1 방향지시등만, 0 유지(traffic/policy_bus.PolicyBus.decide의 lane_cmd)."""
    import json

    import torch
    with open(os.path.join(model_dir, "run_config.json"), encoding="utf-8") as f:
        network = (json.load(f).get("network") or {})
    if network.get("policy_type") == "mlp":
        from utils.mlp_policy import lane_label_of as mlp_lane_label_of, load_mlp_policy_dir
        net, cfg = load_mlp_policy_dir(model_dir, device)
        if cfg.get("input") != "obs33":
            raise ValueError(f"MLP 정책은 관측 33차원(--input obs33)으로 학습한 것만 평가한다: {model_dir}")
        scale = ((cfg.get("data_metadata") or {}).get("accel_scale")) or DEFAULT_ACCEL_SCALE

        def act33(obs):
            with torch.no_grad():
                a = net.det_action(torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0))[0]
            return float(a[0]), mlp_lane_label_of(a[1])
        return act33, {"model_dir": os.path.abspath(model_dir), "accel_scale": dict(scale), "obs_dim": cfg["obs_dim"],
                       "lane_classes": net.n_lane, "accel_bins": net.n_accel, "input": "obs33"}
    from utils.ego_attention import lane_label_of, load_policy_dir
    net, cfg = load_policy_dir(model_dir, device)
    if cfg.get("input") != "v2":
        raise ValueError(f"관측 v2(--input v2)로 학습한 모델만 평가한다: {model_dir} input={cfg.get('input')}")
    scale = ((cfg.get("data_metadata") or {}).get("accel_scale")) or DEFAULT_ACCEL_SCALE

    def act(obs):
        with torch.no_grad():
            a = net.det_action(torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0))[0]
        # 행동 벡터의 차로 값은 라벨 / (클래스 수 // 2)다(5클래스면 {1, 0.5, 0, -0.5, -1}). 라벨로 되돌린다.
        return float(a[0]), lane_label_of(a[1], net.n_lane)
    return act, {"model_dir": os.path.abspath(model_dir), "accel_scale": dict(scale), "obs_dim": cfg["obs_dim"],
                 "lane_classes": net.n_lane, "accel_bins": net.n_accel, "input": "v2"}


# ---------------------------------------------------------------- 판단
class PolicyController:
    """collect_pdm_bus.collect(controller=...)에 넣는다. 판단 시각마다 on_decision(새 스냅숏, 방금 닫힌 행)을 부른다."""

    def __init__(self, act, accel_scale=None, info=None, stall_s: float = STALL_S, world: str = "hapjeong"):
        self.act = act
        self.scale = dict(accel_scale or DEFAULT_ACCEL_SCALE)
        self.info = dict(info or {})
        self.world = world
        self.obs33 = self.info.get("input") == "obs33"
        if self.obs33 and world != "sumo_lc":           # 연결성이 좌회전 차로 보정 도로망에 기대므로 학습과 같은 월드만
            raise ValueError(f"관측 33차원 정책은 --world sumo_lc에서만 평가한다(받은 값: {world})")
        self.pilot = self.v2 = self.c33 = self.ctx = self.replay = None
        self.episode, self.prev = None, None
        self.decisions = 0
        self.stall = StallWatch(stall_s)

    def make_pilot(self, tc, net, loop, stops):
        from traffic.policy_bus import PolicyBusPilot
        return PolicyBusPilot(tc, net, loop, stops, lane_values=int(self.info.get("lane_classes") or 5),
                              stop_plans_only=self.world == "sumo_lc")

    def bind(self, ctx, pilot):
        self.ctx, self.pilot = ctx, pilot
        self.v2 = V2Context(ctx, plan_dicts(pilot))
        if self.obs33:
            self.c33 = Obs33Context(ctx)

    def describe(self) -> dict:
        return {**self.info, "accel_scale": self.scale, "decisions": self.decisions, "stall_s": self.stall.seconds,
                "pilot": dict(getattr(self.pilot, "policy_stats", {}) or {})}

    def end_reason(self, closed, state):
        """닫힌 행 하나의 끝 원인(episode_end와 같은 순서, 순간이동·바퀴 완료는 기록기가 처리한다). 없으면 None."""
        w = closed["window"]
        if any(c.get("ego") for c in w["collisions"]):
            return "collision"
        if state["red_violation"]:
            return "red_light"
        if state["missed_now"]:
            return "missed_stop"
        return None

    def on_decision(self, pending, closed):
        if closed is not None and closed.get("window") is not None and self.replay is not None:
            state = self.replay.step(closed)
            reason = self.end_reason(closed, state)
            if reason is not None:
                raise CollectionDone(reason)
            if self.stall.feed(closed):
                raise CollectionDone("stuck")
            if not self.obs33:
                an, _ = normalize_accel(accel_mps2(closed), self.scale["pos"], self.scale["neg"])
                self.prev = (an, lane_change_label5(closed))
        if pending is None:
            return
        if pending["meta"]["episode"] != self.episode:
            self.episode, self.prev = pending["meta"]["episode"], None
            self.replay = EpisodeReplay(self.ctx)
            self.stall.reset()
        state = self.replay.state()
        obs = obs33(pending, self.c33, state) if self.obs33 else obs_v2(pending, self.v2, state, self.prev)
        an, lane = self.act(obs)
        a = an * self.scale["pos"] if an >= 0 else an * self.scale["neg"]
        self.decisions += 1
        extra = ({"signal_state_input": float(obs[32]), "stop_point_input": float(obs[31])} if self.obs33
                 else {"dwell_remaining_input": float(obs[13]) * SERVICE_SECONDS})
        self.pilot.decide(a, lane, pending["meta"]["sim_time"],
                          {"accel_norm": an, **extra, "tracker_index": state["tracker_index"]})


class ExpertWatch:
    """PDM 전문가가 모는 바퀴에 정책과 같은 멈춰 섬 끝을 건다(전문가 운전은 바꾸지 않는다)."""

    def __init__(self, stall_s: float = STALL_S, world: str = "hapjeong"):
        self.stall = StallWatch(stall_s)
        self.episode = None
        self.world = world

    def make_pilot(self, tc, net, loop, stops):
        if self.world == "sumo_lc":                   # 학습 데이터와 같은 전문가(SUMO 차선 변경 + PDM 종방향)
            from collect_pdm_bus_sumo_lc import DataPilot
            return DataPilot(tc, net, loop, stops)
        from traffic.pdm_bus import PDMClosedBusPilot
        return PDMClosedBusPilot(tc, net, loop, stops)

    def bind(self, ctx, pilot):
        pass

    def describe(self) -> dict:
        return {"agent": "expert", "stall_s": self.stall.seconds}

    def on_decision(self, pending, closed):
        if closed is not None and closed.get("window") is not None and self.stall.feed(closed):
            raise CollectionDone("stuck")
        if pending is not None and pending["meta"]["episode"] != self.episode:
            self.episode = pending["meta"]["episode"]
            self.stall.reset()


# ---------------------------------------------------------------- 지표
def _substeps(rows):
    for i, r in enumerate(rows):
        for s in (r.get("window") or {}).get("substeps", []):
            yield i, s


def stop_metrics(rows, ctx) -> list:
    """서는 정류장마다: 도착, 정차, 카운트다운(8초 -> 0초) 동안 버팀, 0초 뒤 출발.
    정차 판정은 ServiceTracker와 같다(앞 범퍼가 [start, end], 정류장 차로, 0.1 m/s 이하, 끊기면 0부터)."""
    out = []
    subs = [s for _, s in _substeps(rows)]
    for k, (start, end, sid, lane) in enumerate(ctx.service_stops):
        r = {"stop": sid, "order": k + 1, "reached": False, "stopped_in_area": False, "stopped_wrong_lane": False,
             "service_completed": False, "t_arrive": None, "t_first_stop": None, "t_complete": None, "t_depart": None,
             "departure_delay_s": None, "interruptions": 0, "max_partial_dwell_s": 0.0, "overshoot": False,
             "speed_at_area_entry": None}
        dwell = 0.0
        prior_done = all(x["service_completed"] for x in out)     # 앞 정류장을 다 마쳐야 이 정류장 차례다(ServiceTracker 순서)
        for s in subs:
            pos = s["lap_position"]
            if pos is None:
                continue
            in_area = start <= pos <= end
            if pos >= start and not r["reached"]:
                r["reached"], r["t_arrive"], r["speed_at_area_entry"] = True, s["t"], s["speed"]
            if r["service_completed"]:
                if r["t_depart"] is None and s["speed"] > DEPART_MPS:
                    r["t_depart"] = s["t"]
                    r["departure_delay_s"] = s["t"] - r["t_complete"]
                continue
            if not prior_done:
                continue
            stopped = s["speed"] <= STOPPED_MPS
            if in_area and stopped and s["lane_id"] != lane:
                r["stopped_wrong_lane"] = True
            if in_area and stopped and s["lane_id"] == lane:
                if r["t_first_stop"] is None:
                    r["t_first_stop"] = s["t"]
                r["stopped_in_area"] = True
                dwell += DT
                r["max_partial_dwell_s"] = max(r["max_partial_dwell_s"], dwell)
                if dwell + 1e-8 >= SERVICE_SECONDS:
                    r["service_completed"], r["t_complete"] = True, s["t"]
            else:
                if dwell > 0:
                    r["interruptions"] += 1          # 카운트다운이 0초가 되기 전에 움직이거나 구역을 벗어났다
                dwell = 0.0
                if pos > end + 1e-6:
                    r["overshoot"] = True
                    break
        r["max_partial_dwell_s"] = round(r["max_partial_dwell_s"], 3)
        out.append(r)
        if not r["service_completed"]:
            break
    return out


def _lane_changes(rows):
    """같은 도로 안에서 차로가 바뀐 구간: (시각, 방향 +-1)."""
    out = []
    for r in rows:
        w = r.get("window")
        if w and w["edge_before"] == w["edge_after"] and w["lane_after"] != w["lane_before"]:
            out.append((r["meta"]["sim_time"], 1 if w["lane_after"] > w["lane_before"] else -1))
    return out


def _policy_plans(rows):
    out = []
    for r in rows:
        for p in (r.get("window") or {}).get("pdm_plans", []):
            if p.get("policy"):
                out.append(p)
    return out


def _nanmin(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return min(xs) if xs else None


def analyze_run(run_dir: str, stall_limit: float = STALL_S) -> dict:
    """원시 수집 폴더(한 바퀴) -> 지표 dict. 첫 에피소드만 본다.
    outcome: 완주면 arrived, 아니면 실패 종류(끝 원인. 시간 한도인데 stall_limit 넘게 멈춰 있었으면 stuck)."""
    meta = load_meta(run_dir)
    ctx = load_lap_context(run_dir)
    rows_all = list(read_steps(run_dir))
    eps = episodes(rows_all)
    if not eps:
        return {"end_reason": "no_rows", "success": False}
    ep = eps[0]
    rep = replay_episode(ep, ctx)
    end = episode_end(ep, rep, True)
    last = end["last_index"]
    rows = ep[:last + 1] if last >= 0 else []
    rep = rep[:len(rows)]
    m = {"end_reason": end["reason"], "success": end["reason"] == "arrived", "decisions": len(rows)}
    if not rows:
        m["outcome"] = outcome(end["reason"], False, 0.0, stall_limit)
        return m
    t0 = rows[0]["meta"]["sim_time"]
    subs = [s for _, s in _substeps(rows)]
    t_end = subs[-1]["t"] if subs else t0
    lap_len = meta["route"].get("lap_length") or ctx.route_map.length
    pos_max = max((s["lap_position"] for s in subs if s["lap_position"] is not None), default=0.0)
    dist = sum(r["progress_m"] for r in rep)
    m.update(episode_time_s=t_end - t0, lap_time_s=(t_end - t0) if m["success"] else None,
             progress_ratio=min(1.0, pos_max / lap_len) if lap_len else None, distance_m=dist,
             mean_speed_mps=dist / (t_end - t0) if t_end > t0 else 0.0)

    # 정류장
    stops = stop_metrics(rows, ctx)
    done = [s for s in stops if s["service_completed"]]
    delays = [s["departure_delay_s"] for s in done if s["departure_delay_s"] is not None]
    m.update(stops_completed=len(done), stops_stopped_in_area=sum(s["stopped_in_area"] for s in stops),
             stop_interruptions=sum(s["interruptions"] for s in stops),
             stop_wrong_lane=sum(s["stopped_wrong_lane"] for s in stops),
             stop_overshoot=sum(s["overshoot"] for s in stops),
             departures_after_service=len(delays),
             not_departed_after_service=sum(1 for s in done if s["t_depart"] is None),
             departure_delay_mean_s=float(np.mean(delays)) if delays else None,
             departure_delay_max_s=max(delays) if delays else None)

    # 안전
    ped_ids = {p["id"] for r in rows for p in r["pedestrians"]}
    coll = [c for r in rows for c in r["window"]["collisions"] if c.get("ego")]
    other = [(c["victim"] if c["collider"] == BUS else c["collider"]) for c in coll]
    dims = meta.get("expert", {}).get("ego_dims") or {"length": 12.0, "width": 2.5, "min_gap": 1.0}
    ttc, thw = [], []
    for r in rows:
        lead = row_interaction(r, ctx, dims).get("leader")
        if lead:
            ttc.append(lead.get("ttc"))
            if r["ego"]["speed"] >= 1.0:
                thw.append(lead.get("thw"))
    ttc_v = [x for x in ttc if x is not None]
    acc = [s["acceleration"] for s in subs]
    hard = sum(1 for a, b in zip([0.0] + acc, acc) if b <= HARD_BRAKE_MPS2 < a)
    ssm = [c for c in parse_ssm(os.path.join(run_dir, "ssm.xml")) if c["ego"] == BUS or c["foe"] == BUS]
    ped_speeds = []
    for r in rows:
        for q in r["pedestrians"]:
            lon, lat = ego_frame(r, q["x"], q["y"])
            if 0.0 < lon <= PED_PATH_AHEAD_M and abs(lat) <= PED_PATH_HALF_WIDTH_M:
                ped_speeds.append(r["ego"]["speed"])
                break
    m.update(collisions=len(coll), collisions_pedestrian=sum(o in ped_ids for o in other),
             collisions_vehicle=sum(o not in ped_ids for o in other),
             red_light_violations=sum(int(r["red_violation"]) for r in rep),
             min_ttc_s=min(ttc_v) if ttc_v else None,
             ttc_risk_ratio=(sum(x < TTC_RISK_S for x in ttc_v) / len(rows)),
             min_thw_s=_nanmin(thw), hard_brakes=hard,
             emergency_stops=sum(1 for r in rows for e in r["window"]["emergency_stops"] if e.get("ego")),
             ssm_conflicts=len(ssm),
             ssm_min_ttc_s=_nanmin([(c["min_ttc"] or {}).get("value") for c in ssm]),
             ssm_max_drac=max([(c["max_drac"] or {}).get("value") or 0.0 for c in ssm], default=None),
             ssm_min_pet_s=_nanmin([(c["pet"] or {}).get("value") for c in ssm]),
             ped_in_path_max_speed_mps=max(ped_speeds) if ped_speeds else None,
             ped_in_path_mean_speed_mps=float(np.mean(ped_speeds)) if ped_speeds else None)

    # 주행 품질
    changes = _lane_changes(rows)
    osc = sum(1 for (t1, d1), (t2, d2) in zip(changes, changes[1:]) if d2 == -d1 and t2 - t1 <= OSCILLATION_S)
    plans = _policy_plans(rows)
    v2 = V2Context(ctx, meta["route"].get("lane_plans", []))
    off_route = not_usable = 0
    for r, st in zip(rows, rep):
        if r["ego"]["road_id"].startswith(":"):
            continue
        off_route += int(obs_v2(r, v2, st, None)[6] < 0.5)
        not_usable += int(not lane_usable(ctx, r["ego"]["lane_id"]))
    jerk = [(b - a) / DT for a, b in zip(acc, acc[1:])]
    m.update(lane_changes=len(changes), lane_change_oscillations=osc,
             lane_requests=sum(1 for p in plans if abs(p.get("lane_cmd", 0)) == 2),
             blink_only=sum(1 for p in plans if abs(p.get("lane_cmd", 0)) == 1),
             off_route_lane_ratio=off_route / len(rows), unusable_lane_ratio=not_usable / len(rows),
             accel_abs_mean=float(np.mean(np.abs(acc))) if acc else None,
             jerk_rms=float(np.sqrt(np.mean(np.square(jerk)))) if jerk else None,
             comfort_violation_ratio=sum(1 for r in rows
                                         if r["window"]["peak_acceleration"] > COMFORT_LIMITS["acceleration_limit"]
                                         or r["window"]["peak_jerk"] > COMFORT_LIMITS["jerk_limit"]) / len(rows))

    # 시나리오
    s5 = s5_events(rows, meta["route"].get("s5_feeders", []))
    s6 = s6_events(rows, ctx)
    m.update(s5_cutins=len(s5), s5_min_gap_m=_nanmin([e["detail"]["min_gap"] for e in s5]),
             s5_min_bus_accel=_nanmin([e["detail"]["bus_min_accel"] for e in s5]),
             s6_passes=len(s6), s6_min_pet_s=_nanmin([e["detail"]["min_pet"] for e in s6]),
             s6_no_slowdown=sum(1 for e in s6 if e["detail"]["bus_min_speed"] > 2.0))

    # 실패 분류와 위치, 상호작용 장면, 겹침, 배경 충돌
    stall = longest_stall(subs) if subs else {"duration_s": 0.0, "t_start": None}
    kind = outcome(m["end_reason"], m["success"], stall["duration_s"], stall_limit)
    m.update(outcome=kind, longest_stall_s=stall["duration_s"], stall_limit_s=stall_limit)
    m.update(interaction_counts(rows, ctx, stops))
    m.update(overlap_metrics(rows, dims))
    m.update(background_collisions(rows))
    m["failure_location"] = None if m["success"] else failure_location(rows, ctx, meta, kind, stall)
    m["stops"] = stops
    return m
