"""PDM 버스 원시 수집 폴더(scripts/collect_pdm_bus.py)를 학습용 transition과 지표로 바꾼다.

    python scripts/preprocess_pdm_bus.py data/pdm_bus/<name> --report-accel                    # 가감속 분포만 본다
    python scripts/preprocess_pdm_bus.py data/pdm_bus/<name> --accel-pos 1.5 --accel-neg 3.0    # 전처리

가감속 라벨 정규화 상수(--accel-pos, --accel-neg)는 기본값이 없다. 먼저 --report-accel로 분포와 후보별 잘림 비율을 본다.
출력 <run>/processed/: transitions.jsonl.gz, transitions.npz, events.jsonl, summary.json, metadata.json (또는 accel_report.json)
"""
import argparse
import collections
import gzip
import json
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from env.bus_task import LAYOUT, REWARDS  # noqa: E402
from env.data_schema import COLUMNS  # noqa: E402
from pdm_data.features import FEATURE_NAMES  # noqa: E402
from pdm_data.labels import accel_mps2, accel_report  # noqa: E402
from pdm_data.lap_context import load_lap_context  # noqa: E402
from pdm_data.recorder import _round, read_events, read_steps  # noqa: E402
from pdm_data.scenario_offline import s5_events, s6_events  # noqa: E402
from pdm_data.ssm import parse_ssm  # noqa: E402
from pdm_data.transitions import COMFORT_LIMITS, build_transitions, load_meta, obs_online_check  # noqa: E402

CANDIDATES = [(1.0, 2.0), (1.5, 3.0), (2.0, 4.0)]
EXTRA_COLUMNS = {
    "sim_time": "관측 시점의 SUMO 시각(초)",
    "detected_pedestrians": "관측 시점 반경 50 m 보행자(ego 기준 상대 위치·속도)",
    "event.termination_reason": "에피소드 끝 원인(arrived, incomplete_service, collision, red_light, missed_stop, teleport, "
                                "time_limit, bus_lost). 끝이 아니면 빈 값",
    "comfort.*": "구간 가속도·jerk 최댓값과 승차감 한계(1.5 m/s², 2.0 m/s³) 위반",
    "expert.*": "전문가 명령(B안 라벨): 규칙 단계, 요청 방향, 창 종류, PDM 속도 비율·v0",
    "reward_terms.*": "보상 항목: progress, step_cost, service, terminal_value, total",
    "scenario.*": "지금 진행 중인 S1~S6·A1",
    "interaction.*": "선행·후행 슬롯의 thw, ttc, inv_ttc, drac, margin과 50 m 보행자의 t_ego, t_ped, dt",
    "features / next_features": "입력 설계 특징(feature_names 순서, [-1, 1])",
    "action_extra.accel_mps2": "정규화 전 실제 평균 가속도(m/s²)",
}


def _pct(values):
    a = np.asarray([v for v in values if v is not None], dtype=float)
    if a.size == 0:
        return None
    return {"n": int(a.size), "p5": float(np.percentile(a, 5)), "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95))}


def derived_events(stats):
    out = []
    for ep, rep in stats["_replays"]:
        for row, r in zip(ep, rep):
            base = {"lap": row["meta"]["episode"], "t_start": row["meta"]["sim_time"], "t_end": None, "ids": [],
                    "injected": False}
            if r["services"]:
                out.append(dict(base, kind="service", detail={"completed_before": r["completed"]}))
            if r["missed_now"]:
                out.append(dict(base, kind="missed_stop", detail={"tracker_index": r["tracker_index"]}))
            if r["red_violation"]:
                out.append(dict(base, kind="red_violation", detail={"tls": row["tls"][:1]}))
    return out


def summarize(transitions, events, ssm):
    laps = collections.defaultdict(lambda: collections.Counter())
    for e in events:
        laps[e.get("lap")][e["kind"]] += 1
    eps = collections.defaultdict(lambda: {"reward_sum": 0.0, "transitions": 0, "end": None,
                                           "acc_violations": 0, "jerk_violations": 0})
    leader = collections.defaultdict(list)
    for r in transitions:
        s = eps[r["episode"]]
        s["reward_sum"] += r["reward"]
        s["transitions"] += 1
        s["acc_violations"] += int(r["comfort"]["acceleration_violation"])
        s["jerk_violations"] += int(r["comfort"]["jerk_violation"])
        if r["event"]["termination_reason"]:
            s["end"] = r["event"]["termination_reason"]
        for slot in ("leader", "follower"):
            m = r["interaction"].get(slot)
            if m:
                for k in ("thw", "ttc", "drac"):
                    leader[f"{slot}.{k}"].append(m[k])
    return {"episodes": {str(k): v for k, v in eps.items()},
            "events_by_lap": {str(k): dict(v) for k, v in laps.items()},
            "interaction": {k: _pct(v) for k, v in leader.items()},
            "ssm_conflicts": len(ssm),
            "ssm_min_ttc": _pct([c["min_ttc"]["value"] for c in ssm if c["min_ttc"]])}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--accel-pos", type=float)
    ap.add_argument("--accel-neg", type=float)
    ap.add_argument("--report-accel", action="store_true")
    ap.add_argument("--keep-after-task-failure", action="store_true", help="신호 위반·정류장 놓침에서 에피소드를 자르지 않는다")
    ap.add_argument("--max-lap-s", type=float, default=None)
    a = ap.parse_args(argv)
    out_dir = os.path.join(a.run_dir, "processed")
    os.makedirs(out_dir, exist_ok=True)
    if a.report_accel:
        vals = [accel_mps2(r) for r in read_steps(a.run_dir) if r.get("window")]
        rep = accel_report(vals, CANDIDATES)
        with open(os.path.join(out_dir, "accel_report.json"), "w", encoding="utf-8") as f:
            json.dump(rep, f, ensure_ascii=False, indent=1)
        print(json.dumps(rep, ensure_ascii=False, indent=1))
        return 0
    if a.accel_pos is None or a.accel_neg is None:
        print("--accel-pos와 --accel-neg가 필요합니다(먼저 --report-accel로 분포를 보세요)", file=sys.stderr)
        return 2
    tr, stats = build_transitions(a.run_dir, a.accel_pos, a.accel_neg, not a.keep_after_task_failure, a.max_lap_s,
                                  with_replay=True)
    meta = load_meta(a.run_dir)
    ssm = parse_ssm(os.path.join(a.run_dir, "ssm.xml"))
    # S5·S6은 수집 중에 판정하지 않고 여기서 원시 행으로 판정한다(pdm_data/scenario_offline.py)
    all_rows = list(read_steps(a.run_dir))
    offline = s5_events(all_rows, meta.get("route", {}).get("s5_feeders", [])) + \
        s6_events(all_rows, load_lap_context(a.run_dir))
    events = read_events(a.run_dir) + derived_events(stats) + offline + [
        {"kind": "ssm_conflict", "lap": None, "t_start": c["begin"], "t_end": c["end"], "ids": [c["foe"]],
         "injected": False, "detail": c} for c in ssm]
    stats.pop("_replays")
    with gzip.open(os.path.join(out_dir, "transitions.jsonl.gz"), "wt", encoding="utf-8", compresslevel=6) as f:
        for r in tr:
            f.write(json.dumps(_round(r), ensure_ascii=False, separators=(",", ":")) + "\n")
    np.savez_compressed(os.path.join(out_dir, "transitions.npz"),
                        observation=np.asarray([r["observation"] for r in tr], np.float32).reshape(-1, 93),
                        next_observation=np.asarray([r["next_observation"] for r in tr], np.float32).reshape(-1, 93),
                        action_raw=np.asarray([r["action_raw"] for r in tr], np.float32).reshape(-1, 2),
                        reward=np.asarray([r["reward"] for r in tr], np.float32),
                        terminated=np.asarray([r["terminated"] for r in tr], bool),
                        truncated=np.asarray([r["truncated"] for r in tr], bool),
                        episode=np.asarray([r["episode"] for r in tr], np.int32),
                        t=np.asarray([r["t"] for r in tr], np.int32),
                        features=np.asarray([r["features"] for r in tr], np.float32).reshape(-1, len(FEATURE_NAMES)),
                        next_features=np.asarray([r["next_features"] for r in tr], np.float32).reshape(-1, len(FEATURE_NAMES)))
    with open(os.path.join(out_dir, "events.jsonl"), "w", encoding="utf-8") as f:
        for e in events:
            f.write(json.dumps(_round(e), ensure_ascii=False, separators=(",", ":")) + "\n")
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(_round(summarize(tr, events, ssm)), f, ensure_ascii=False, indent=1)
    check = obs_online_check(tr)
    pmeta = {"source_schema": meta.get("schema_version"), "net_sha256": meta.get("net_sha256"),
             "accel_scale": {"pos": a.accel_pos, "neg": a.accel_neg}, "stats": stats,
             "cut_on_task_failure": not a.keep_after_task_failure, "max_lap_s": a.max_lap_s,
             "feature_names": FEATURE_NAMES, "obs93_layout": LAYOUT, "rewards": REWARDS, "comfort_limits": COMFORT_LIMITS,
             "obs93_online_check": check,
             "columns": [{"name": n, "type": k, "meaning": m} for n, k, _, m in COLUMNS] +
                        [{"name": n, "meaning": m} for n, m in EXTRA_COLUMNS.items()]}
    with open(os.path.join(out_dir, "metadata.json"), "w", encoding="utf-8") as f:
        json.dump(pmeta, f, ensure_ascii=False, indent=1)
    print(f"transition {stats['transitions']}개 | 끝 원인 {stats['end_reasons']} | 가속 라벨 잘림 {stats['accel_clip_ratio']:.3f}")
    if check["rows"]:
        print(f"온라인·오프라인 93차원 비교: {check['rows']}행, 최대 오차 {check['max_abs_diff']:.2e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
