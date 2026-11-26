"""학습한 BC 정책(관측 v2)과 PDM 전문가를 합정역 baseline 교통에서 한 바퀴씩 돌려(closed-loop) 비교한다.

    # 전부(모델 6개 + 전문가) x 평가 seed 20개, 동시에 8개
    python scripts/eval_bc_closed_loop.py run --models results/bc_v2_lane --out results/closed_loop_v2_lane --jobs 8
    # 한 에피소드만(확인용)
    python scripts/eval_bc_closed_loop.py one --agent results/bc_v2_lane/base/seed1 --seed 1000 --out results/cl_smoke
    python scripts/eval_bc_closed_loop.py one --agent expert --seed 1000 --out results/cl_smoke
    # 표만 다시
    python scripts/eval_bc_closed_loop.py summarize results/closed_loop_v2_lane

평가 조건
  - 교통: baseline, 학습 데이터(seed 0~114)와 겹치지 않는 seed(기본 1000~1019). 바퀴 하나 = 에피소드 하나, 시간 한도 1800 s.
  - 정책 버스(traffic/policy_bus.py): SUMO 안전장치 끔(speedMode 32, laneChangeMode 0), 정류장 정차 명령 없음.
    가감속과 차선 변경(+-2 요청, +-1 방향지시등)을 정책이 0.5 s마다 직접 낸다.
    정류장: 버스가 정류장 구역·차로에 멈추면 관측 dwell_remaining이 8초에서 0초까지 줄고, 0초가 되면 정차 완료다.
  - 전문가: 데이터 수집과 같은 PDM-Closed 규칙 버스(SUMO 정차 명령과 안전장치를 쓴다). 기준선이다.
  - 에피소드 끝: 충돌, 신호 위반, 정류장 놓침, 순간이동, 멈춰 섬, 시간 한도, 바퀴 완료(정류장 4곳 모두 마치면 arrived).
    멈춰 섬(stuck): --stall-seconds(기본 180 s) 동안 5 m보다 덜 움직였다. 정책과 전문가 모두 그 순간 끝낸다(0이면 끈다).

출력 <out>/
  <agent>/seed<S>/run/    원시 기록(steps.jsonl.gz, events.jsonl, ssm.xml, metadata.json, assets/) - 학습 데이터와 같은 형식
  <agent>/seed<S>/result.json   지표(pdm_data/closed_loop.analyze_run, 추가 지표는 pdm_data/closed_loop_extra.py)
  logs/  episodes.csv  stops.csv  agents.csv  failures.csv  highlights.json  summary.md
  summary.md: 지표 표(학습 seed 평균 ± 표준편차), 결과 분포(95% 신뢰구간), 실패 위치, 바퀴당 상호작용 장면,
              보조 지표(SUMO 미보고 겹침, 버스 근처 배경 차 충돌), 영상 후보(가장 좋은 바퀴, 실패 종류별 대표 바퀴)
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

EXPERT = "expert"
DEFAULT_SEEDS = "1000-1019"

# (열, 설명, 집계: rate=에피소드 비율, mean=에피소드 평균(None 제외), 좋은 방향)
END_REASONS = ("arrived", "collision", "red_light", "missed_stop", "incomplete_service", "stuck", "time_limit",
               "teleport", "bus_lost", "sumo_closed")
OUTCOME_LABEL = {"arrived": "완주", "collision": "충돌", "red_light": "신호 위반", "missed_stop": "정류장 놓침",
                 "incomplete_service": "정차 덜 함", "stuck": "멈춰 섬", "time_limit": "시간 초과",
                 "teleport": "순간이동", "bus_lost": "버스 사라짐", "sumo_closed": "SUMO 종료"}
COLUMNS = [
    ("success", "완주율", "rate", "+"),
    ("rate_collision", "충돌률", "rate", "-"),
    ("rate_red_light", "신호 위반률", "rate", "-"),
    ("rate_missed_stop", "정류장 놓침률", "rate", "-"),
    ("rate_stuck", "멈춰 섬 비율", "rate", "-"),
    ("rate_time_limit", "시간 초과률", "rate", "-"),
    ("rate_teleport", "순간이동률", "rate", "-"),
    ("longest_stall_s", "가장 길게 멈춘 시간 s", "mean", "-"),
    ("progress_ratio", "노선 진행률", "mean", "+"),
    ("lap_time_s", "한 바퀴 시간(완주만) s", "mean", "-"),
    ("mean_speed_mps", "평균 속도 m/s", "mean", "+"),
    ("stops_completed", "정차 완료 정류장(0~4)", "mean", "+"),
    ("stop_interruptions", "카운트다운 중 움직임", "mean", "-"),
    ("stop_wrong_lane", "다른 차로 정차", "mean", "-"),
    ("stop_overshoot", "정류장 지나침", "mean", "-"),
    ("departure_delay_mean_s", "0초 뒤 출발 지연 s", "mean", "-"),
    ("not_departed_after_service", "0초 뒤 미출발", "mean", "-"),
    ("min_ttc_s", "최소 TTC s", "mean", "+"),
    ("ttc_risk_ratio", "TTC<1.5s 비율", "mean", "-"),
    ("min_thw_s", "최소 차간 시간 s", "mean", "+"),
    ("hard_brakes", "급제동(<=-3) 횟수", "mean", "-"),
    ("ssm_conflicts", "SSM 상충", "mean", "-"),
    ("ped_in_path_max_speed_mps", "앞 보행자 있을 때 최고 속도", "mean", "-"),
    ("lane_changes", "차선 변경", "mean", None),
    ("lane_change_oscillations", "갈팡질팡", "mean", "-"),
    ("off_route_lane_ratio", "노선 이탈 차로 비율", "mean", "-"),
    ("unusable_lane_ratio", "버스 불가 차로 비율", "mean", "-"),
    ("jerk_rms", "저크 RMS", "mean", "-"),
    ("comfort_violation_ratio", "승차감 위반 비율", "mean", "-"),
    ("s5_min_gap_m", "S5 끼어들기 최소 간격 m", "mean", "+"),
    ("s6_no_slowdown", "S6 보행자 있는데 감속 없음", "mean", "-"),
    ("bbox_overlap_unreported_rows", "SUMO 미보고 상자 겹침(스냅숏 수)", "mean", "-"),
    ("bg_collisions_near", "버스 150 m 안 배경 차 충돌", "mean", None),
]
# 바퀴당 상호작용 장면: (열, 설명, 분모 열(그중 몇 번인지 비율을 붙인다) 또는 None)
INTERACTIONS = [
    ("ix_stop_served", "정류장 정차(완료)", None),
    ("ix_stop_follower", "  그중 같은 차로 뒤 50 m 안에 차", "ix_stop_served"),
    ("ix_lc_req_left", "왼쪽 변경 요청", None),
    ("ix_lc_req_left_follower", "  그중 옮길 차로 뒤 30 m 안에 차", "ix_lc_req_left"),
    ("ix_lc_req_right", "오른쪽 변경 요청", None),
    ("ix_lc_req_right_follower", "  그중 옮길 차로 뒤 30 m 안에 차", "ix_lc_req_right"),
    ("ix_lc_done", "실제 차로 변경", None),
    ("ix_lc_done_occupied", "  그중 옮길 차로 앞뒤 30 m 안에 차", "ix_lc_done"),
    ("ix_lc_done_follower", "  그중 옮길 차로 뒤 30 m 안에 차", "ix_lc_done"),
    ("ix_lc_done_mid", "  그중 신호 앞 60 m 밖(구간 가운데)", "ix_lc_done"),
    ("ix_crossing_approach", "횡단보도 접근", None),
    ("ix_crossing_ped", "  그중 보행자가 건너던 접근", "ix_crossing_approach"),
    ("ix_cutin", "앞차 간격이 0.5초에 10 m 넘게 줄어든 순간", None),
]


def parse_seeds(text):
    out = []
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def agent_name(agent, models_root=None):
    if agent == EXPERT:
        return EXPERT
    p = os.path.abspath(agent)
    if models_root:
        rel = os.path.relpath(p, os.path.abspath(models_root))
        if not rel.startswith(".."):
            return rel.replace(os.sep, "/")
    return "/".join(p.split(os.sep)[-2:])


# ---------------------------------------------------------------- 한 에피소드
WORLDS = {"hapjeong": ("baseline", 120.0), "sumo_lc": ("busy", 0.0)}   # 월드 -> (시나리오, 기본 warmup)


def use_world(world):
    """collect_pdm_bus의 월드·metadata 훅을 고른다. sumo_lc는 학습 데이터(data/pdm_bus_sumo_lc)를 만든 월드다
    (scripts/collect_pdm_bus_sumo_lc.py: 좌회전 차로 제한, 밀도 유지기 등). 하위 프로세스마다 에피소드 하나라 전역 교체가 안전하다.
    반환: (시나리오, 기본 warmup)."""
    import collect_pdm_bus as C
    if world == "sumo_lc":
        import collect_pdm_bus_sumo_lc as LC
        C._world, C.build_metadata = LC._world, LC._build_metadata
    elif world != "hapjeong":
        raise ValueError(f"알 수 없는 월드: {world}")
    return WORLDS[world]


def run_one(agent, seed, out_root, name=None, warmup=None, max_seconds=1800.0, keep_work=False, stall_s=None,
            world="hapjeong"):
    import collect_pdm_bus
    from pdm_data.closed_loop import STALL_S, ExpertWatch, PolicyController, analyze_run, load_policy
    stall_s = STALL_S if stall_s is None else float(stall_s)
    name = name or agent_name(agent)
    ep_dir = os.path.join(out_root, name, f"seed{seed}")
    res_path = os.path.join(ep_dir, "result.json")
    if os.path.exists(res_path):
        print(f"이미 있다: {res_path}")
        return json.load(open(res_path, encoding="utf-8"))
    run_dir = os.path.join(ep_dir, "run")
    if os.path.exists(run_dir):                     # 지난번에 기록은 끝났지만 지표 전에 멈췄다: 다시 돈다
        shutil.rmtree(run_dir)
    os.makedirs(ep_dir, exist_ok=True)
    work = tempfile.mkdtemp(prefix=f"cl_{name.replace('/', '_')}_s{seed}_")
    scenario, default_warmup = use_world(world)
    warmup = default_warmup if warmup is None else float(warmup)
    a = collect_pdm_bus.parse_args(["--scenario", scenario, "--seed", str(seed), "--laps", "1",
                                    "--warmup", str(warmup), "--max-seconds", str(max_seconds),
                                    "--name", "run", "--out-dir", ep_dir, "--work-dir", work])
    if agent == EXPERT:
        controller = ExpertWatch(stall_s, world)   # 전문가 운전은 그대로, 멈춰 섬 끝만 정책과 같게
    else:
        import torch
        torch.set_num_threads(1)
        act, info = load_policy(agent)
        controller = PolicyController(act, info["accel_scale"], info, stall_s=stall_s, world=world)
    t0 = time.time()
    try:
        r = collect_pdm_bus.collect(a, controller=controller)
    finally:
        if not keep_work:
            shutil.rmtree(work, ignore_errors=True)
    metrics = analyze_run(r["run_dir"], stall_limit=stall_s)
    result = {"agent": name, "agent_path": agent if agent == EXPERT else os.path.abspath(agent), "seed": seed,
              "scenario": scenario, "world": world, "collection_end": r["end"], "wall_s": round(time.time() - t0, 1),
              "warmup_s": warmup, "max_seconds": max_seconds, "stall_s": stall_s,
              "run_dir": os.path.relpath(r["run_dir"], out_root),
              "policy": controller.describe() if agent != EXPERT else None, "metrics": metrics}
    with open(res_path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
    os.replace(res_path + ".tmp", res_path)
    print(f"끝: {name} seed {seed} | {metrics.get('outcome', metrics['end_reason'])} | 정차 {metrics.get('stops_completed')} | "
          f"{result['wall_s']} s")
    return result


# ---------------------------------------------------------------- 여러 에피소드
def find_models(models_root, allow_unfinished=False):
    out = []
    for cfg in sorted(glob.glob(os.path.join(models_root, "**", "run_config.json"), recursive=True)):
        d = os.path.dirname(cfg)
        if not os.path.exists(os.path.join(d, "model.pt")):
            continue
        if not allow_unfinished and not os.path.exists(os.path.join(d, "audit.json")):
            print(f"학습이 안 끝나 건너뛴다(audit.json 없음): {d}")
            continue
        out.append(d)
    return out


def one_cmd(a, path, name, seed):
    """에피소드 하나를 도는 하위 프로세스 명령."""
    cmd = [sys.executable, os.path.abspath(__file__), "one", "--agent", path, "--name", name, "--seed", str(seed),
           "--out", a.out, "--max-seconds", str(a.max_seconds), "--stall-seconds", str(a.stall_seconds),
           "--world", a.world]
    if a.warmup is not None:
        cmd += ["--warmup", str(a.warmup)]
    return cmd


def run_many(a):
    models = []
    for root in a.models:
        found = find_models(root, a.allow_unfinished)
        models += [(m, agent_name(m, root)) for m in found]
    agents = models + ([] if a.no_expert else [(EXPERT, EXPERT)])
    if not agents:
        raise SystemExit("평가할 모델이 없다")
    seeds = parse_seeds(a.seeds)
    jobs = [(path, name, s) for path, name in agents for s in seeds]
    todo = [j for j in jobs if not os.path.exists(os.path.join(a.out, j[1], f"seed{j[2]}", "result.json"))]
    os.makedirs(os.path.join(a.out, "logs"), exist_ok=True)
    print(f"에이전트 {len(agents)}개 {[n for _, n in agents]} x seed {len(seeds)}개 = {len(jobs)}개, 남은 것 {len(todo)}개, "
          f"동시 {a.jobs}개", flush=True)
    if a.dry_run:
        for j in todo:
            print("  ", j[1], j[2])
        return
    lock, status = threading.Lock(), {}

    def work(job):
        path, name, seed = job
        log = os.path.join(a.out, "logs", f"{name.replace('/', '_')}_s{seed}.log")
        cmd = one_cmd(a, path, name, seed)
        t0 = time.time()
        with open(log, "a", encoding="utf-8") as f:
            f.write(" ".join(cmd) + "\n")
            f.flush()
            code = subprocess.call(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=ROOT,
                                   env={**os.environ, "CUDA_VISIBLE_DEVICES": "", "OMP_NUM_THREADS": "1"})
        with lock:
            status[f"{name}/seed{seed}"] = {"ok": code == 0, "code": code, "minutes": round((time.time() - t0) / 60, 1)}
            done = len(status)
            print(f"[{time.strftime('%H:%M:%S')}] {'끝' if code == 0 else '실패'} {name} seed {seed} "
                  f"({done}/{len(todo)}, {status[f'{name}/seed{seed}']['minutes']}분)", flush=True)

    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        list(ex.map(work, todo))
    failed = [k for k, v in status.items() if not v["ok"]]
    with open(os.path.join(a.out, "run_status.json"), "w", encoding="utf-8") as f:
        json.dump(status, f, ensure_ascii=False, indent=1)
    print(f"끝: {len(status)}개 실행, 실패 {len(failed)}개 {failed}", flush=True)
    summarize(a.out)


# ---------------------------------------------------------------- 요약
def _outcome(m):
    """예전 result.json(outcome 없음)은 끝 원인을 그대로 쓴다."""
    return m.get("outcome") or ("arrived" if m.get("success") else m.get("end_reason"))


def _flat(res):
    from pdm_data.closed_loop_extra import location_key
    m = res["metrics"]
    row = {"agent": res["agent"], "seed": res["seed"], "end_reason": m.get("end_reason"), "outcome": _outcome(m)}
    for k, v in m.items():
        if k != "stops" and not isinstance(v, (dict, list)):
            row[k] = v
    for r in END_REASONS:
        row[f"rate_{r}"] = float(row["outcome"] == r)
    row["success"] = float(bool(m.get("success")))
    row["failure_where"] = location_key(m.get("failure_location") or {}) if row["outcome"] != "arrived" else None
    return row


def wilson(k, n, z=1.96):
    """이항 비율의 Wilson 95% 신뢰구간."""
    if n == 0:
        return None, None
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def _rate_cell(k, n):
    if n == 0:
        return "-"
    lo, hi = wilson(k, n)
    return f"{k} ({100 * k / n:.1f}%, {100 * lo:.1f}~{100 * hi:.1f})"


def reanalyze(out, stall_s=None):
    """기록에서 지표를 다시 계산한다(지표 정의를 바꾼 뒤 시뮬레이션 없이). run/이 없는 result.json은 건너뛴다."""
    from pdm_data.closed_loop import STALL_S, analyze_run
    n = 0
    for p in sorted(glob.glob(os.path.join(out, "**", "result.json"), recursive=True)):
        with open(p, encoding="utf-8") as f:
            res = json.load(f)
        run_dir = os.path.join(os.path.dirname(p), "run")
        if not os.path.exists(os.path.join(run_dir, "metadata.json")):
            print(f"기록 없음, 건너뜀: {run_dir}")
            continue
        limit = stall_s if stall_s is not None else res.get("stall_s", STALL_S)
        res["metrics"], res["stall_s"] = analyze_run(run_dir, stall_limit=limit), limit
        res["run_dir"] = os.path.relpath(run_dir, out)
        with open(p + ".tmp", "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=1)
        os.replace(p + ".tmp", p)
        n += 1
    print(f"다시 계산: {n}개")
    summarize(out)


def _highlights(results, groups_of):
    """영상 후보. 실험마다: 가장 좋은 완주 바퀴(실제 차로 변경이 가장 많고, 같으면 빨리 돈 것), 실패 종류마다 대표 바퀴
    (그 종류의 실패가 가장 많이 난 위치에서 난 것 중 seed가 가장 작은 것). 시간 구간은 실패 전 30 s ~ 후 5 s."""
    from pdm_data.closed_loop_extra import location_key
    out = {}
    by_group = {}
    for r in results:
        by_group.setdefault(groups_of(r["agent"]), []).append(r)
    for g, rs in sorted(by_group.items()):
        item = {}
        ok = [r for r in rs if r["metrics"].get("success")]
        if ok:
            best = max(ok, key=lambda r: (r["metrics"].get("ix_lc_done") or 0, -(r["metrics"].get("lap_time_s") or 0)))
            item["best"] = {"agent": best["agent"], "seed": best["seed"], "run_dir": best.get("run_dir"),
                            "lap_time_s": best["metrics"].get("lap_time_s"),
                            "lane_changes": best["metrics"].get("ix_lc_done")}
        fails = {}
        for r in rs:
            k = _outcome(r["metrics"])
            if k != "arrived":
                fails.setdefault(k, []).append(r)
        for k, frs in sorted(fails.items()):
            where = collections.Counter(location_key(r["metrics"].get("failure_location") or {}) for r in frs)
            top = where.most_common(1)[0][0]
            pick = min((r for r in frs if location_key(r["metrics"].get("failure_location") or {}) == top),
                       key=lambda r: (r["agent"], r["seed"]))
            loc = pick["metrics"].get("failure_location") or {}
            t = loc.get("t")
            item[f"worst_{k}"] = {"agent": pick["agent"], "seed": pick["seed"], "run_dir": pick.get("run_dir"),
                                  "where": top, "count_here": where[top], "count_kind": len(frs), "t": t,
                                  "clip": [round(t - 30.0, 1), round(t + 5.0, 1)] if t is not None else None,
                                  "longest_stall_s": pick["metrics"].get("longest_stall_s")}
        out[g] = item
    return out


def _mean(xs):
    xs = [float(x) for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return sum(xs) / len(xs) if xs else None


def _experiment(agent):
    """'base/seed1' -> 'base'. 전문가는 그대로."""
    parts = agent.split("/")
    return "/".join(parts[:-1]) if len(parts) > 1 and parts[-1].startswith("seed") else agent


def _fmt(v, sd=None):
    if v is None:
        return "-"
    s = f"{v:.3f}" if abs(v) < 100 else f"{v:.1f}"
    if sd is not None:
        s += f" ± {sd:.3f}" if abs(sd) < 100 else f" ± {sd:.1f}"
    return s


FAIL_COLUMNS = ("collision", "red_light", "missed_stop", "stuck", "time_limit")


def _extra_sections(results, eps, groups, order, out):
    """결과 분포(95% 신뢰구간), 실패 위치, 바퀴당 상호작용 장면, 보조 지표, 영상 후보. failures.csv·highlights.json도 쓴다."""
    group_of = {name: g for g, names in groups.items() for name in names}
    by_group = {g: [e for e in eps if group_of[e["agent"]] == g] for g in order}
    lines = ["", "## 결과 분포 (실험의 모든 에피소드를 합침)", "",
             "괄호는 비율과 Wilson 95% 신뢰구간(%)이다. 멈춰 섬은 그 시간 동안 5 m보다 덜 움직인 바퀴다.", "",
             "| 실험 | 바퀴 | 완주 | 충돌 (차/보행자) | " + " | ".join(OUTCOME_LABEL[k] for k in FAIL_COLUMNS[1:])
             + " | 그 밖 | 배경 충돌 바퀴 뺀 완주 |",
             "|---|---|---|---|" + "---|" * (len(FAIL_COLUMNS) - 1) + "---|---|"]
    for g in order:
        rows = by_group[g]
        n = len(rows)
        cnt = collections.Counter(e["outcome"] for e in rows)
        cv = sum(1 for e in rows if e["outcome"] == "collision" and (e.get("collisions_vehicle") or 0) > 0)
        cp = sum(1 for e in rows if e["outcome"] == "collision" and (e.get("collisions_pedestrian") or 0) > 0)
        other = n - cnt["arrived"] - sum(cnt[k] for k in FAIL_COLUMNS)
        clean = [e for e in rows if not e.get("abnormal_traffic")]
        lines.append(f"| {g} | {n} | {_rate_cell(cnt['arrived'], n)} | {_rate_cell(cnt['collision'], n)} {cv}/{cp} | "
                     + " | ".join(_rate_cell(cnt[k], n) for k in FAIL_COLUMNS[1:])
                     + f" | {other} | {_rate_cell(sum(e['outcome'] == 'arrived' for e in clean), len(clean))} |")

    # 실패 위치
    fails = [e for e in eps if e["outcome"] != "arrived"]
    loc_of = {(r["agent"], r["seed"]): (r["metrics"].get("failure_location") or {}) for r in results}
    if fails:
        fields = ["agent", "seed", "outcome", "failure_where", "edge", "lane", "lap_position", "next_tls",
                  "next_tls_distance", "prev_stop", "next_stop", "next_turn", "next_turn_distance", "t",
                  "longest_stall_s"]
        with open(os.path.join(out, "failures.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            for e in fails:
                w.writerow({**loc_of[(e["agent"], e["seed"])], **e})
        lines += ["", "## 실패 위치 (실험마다 많은 순 10곳)", "",
                  "위치는 '다음 신호 앞'(200 m 안에 신호가 있을 때) 또는 도로 id와, 그 직전에 지난 서는 정류장이다. "
                  "멈춰 섬은 멈추기 시작한 곳이다. 전체 목록: failures.csv", "",
                  "| 실험 | 실패 | 위치 | 바퀴 |", "|---|---|---|---|"]
        for g in order:
            c = collections.Counter((e["outcome"], e["failure_where"]) for e in by_group[g] if e["outcome"] != "arrived")
            for (k, where), v in c.most_common(10):
                lines.append(f"| {g} | {OUTCOME_LABEL.get(k, k)} | {where} | {v} |")

    # 상호작용 장면
    lines += ["", "## 바퀴당 상호작용 장면 (완주한 바퀴만)", "",
              "버스가 보는 범위 안만 센다. 들여 쓴 줄의 괄호는 위 줄 중 그 비율이다. 정의: pdm_data/closed_loop_extra.py", "",
              "| 장면 | " + " | ".join(f"{g} (완주 {sum(e['outcome'] == 'arrived' for e in by_group[g])})" for g in order)
              + " |", "|---|" + "---|" * len(order)]
    for col, label, denom in INTERACTIONS:
        cells = []
        for g in order:
            ok = [e for e in by_group[g] if e["outcome"] == "arrived" and e.get(col) is not None]
            if not ok:
                cells.append("-")
                continue
            s = sum(e[col] for e in ok)
            cell = f"{s / len(ok):.2f}"
            if denom:
                d = sum(e.get(denom) or 0 for e in ok)
                cell += f" ({100 * s / d:.0f}%)" if d else ""
            cells.append(cell)
        lines.append(f"| {label} | " + " | ".join(cells) + " |")

    # 보조 지표
    lines += ["", "## 보조 지표 (바퀴 수)", "",
              "| 실험 | 바퀴 | SUMO 미보고 상자 겹침이 있던 바퀴 | 버스 150 m 안 배경 차 충돌이 있던 바퀴 |", "|---|---|---|---|"]
    for g in order:
        rows = by_group[g]
        ov = sum(1 for e in rows if (e.get("bbox_overlap_unreported_rows") or 0) > 0)
        bg = sum(1 for e in rows if e.get("abnormal_traffic"))
        lines.append(f"| {g} | {len(rows)} | {ov} | {bg} |")
    lines += ["", "상자 겹침은 0.5 s 스냅숏에서 버스 상자와 차 상자가 겹쳤는데 그 구간에 SUMO 충돌 보고가 없던 것이다. "
              "배경 차 충돌이 있던 바퀴는 교통 자체가 이상했을 수 있어 위 결과 분포에 그 바퀴를 뺀 완주를 따로 적었다."]

    # 영상 후보
    hl = _highlights(results, lambda a: group_of[a])
    with open(os.path.join(out, "highlights.json"), "w", encoding="utf-8") as f:
        json.dump(hl, f, ensure_ascii=False, indent=1)
    lines += ["", "## 영상 후보 (highlights.json)", "", "| 실험 | 장면 | 에이전트 / seed | 내용 | 구간(sim 시각) |",
              "|---|---|---|---|---|"]
    for g in order:
        for key, v in (hl.get(g) or {}).items():
            if key == "best":
                what = f"완주, 차로 변경 {v['lane_changes']}번, {_fmt(v['lap_time_s'])} s"
                clip = "바퀴 전체"
            else:
                kind = key[len("worst_"):]
                what = f"{OUTCOME_LABEL.get(kind, kind)} {v['count_kind']}번 중 가장 많은 곳({v['count_here']}번): {v['where']}"
                clip = f"{v['clip'][0]} ~ {v['clip'][1]}" if v.get("clip") else "-"
            lines.append(f"| {g} | {key} | {v['agent']} / {v['seed']} | {what} | {clip} |")
    return lines


def summarize(out):
    results = []
    for p in sorted(glob.glob(os.path.join(out, "**", "result.json"), recursive=True)):
        with open(p, encoding="utf-8") as f:
            results.append(json.load(f))
    if not results:
        print("result.json이 없다")
        return
    eps = [_flat(r) for r in results]
    keys = sorted({k for e in eps for k in e}, key=lambda k: (k not in ("agent", "seed", "end_reason"), k))
    with open(os.path.join(out, "episodes.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(eps)
    stop_rows = [{"agent": r["agent"], "seed": r["seed"], **s} for r in results for s in r["metrics"].get("stops", [])]
    if stop_rows:
        with open(os.path.join(out, "stops.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(stop_rows[0]))
            w.writeheader()
            w.writerows(stop_rows)

    # 에이전트(모델 하나)별: 에피소드 평균
    agents = {}
    for e in eps:
        agents.setdefault(e["agent"], []).append(e)
    per_agent = {}
    for name, rows in sorted(agents.items()):
        per_agent[name] = {"episodes": len(rows), **{c: _mean([r.get(c) for r in rows]) for c, *_ in COLUMNS}}
        per_agent[name]["end_reasons"] = {k: sum(r["end_reason"] == k for r in rows) for k in
                                          sorted({r["end_reason"] for r in rows})}
    with open(os.path.join(out, "agents.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["agent", "episodes", *[c for c, *_ in COLUMNS], "end_reasons"])
        for name, v in per_agent.items():
            w.writerow([name, v["episodes"], *[v[c] for c, *_ in COLUMNS], json.dumps(v["end_reasons"])])

    # 실험별: 학습 seed 평균 ± 표준편차(전문가는 에피소드 평균만)
    groups = {}
    for name in per_agent:
        groups.setdefault(_experiment(name), []).append(name)
    order = sorted(groups, key=lambda g: (g != EXPERT, g))
    lines = [f"# Closed-loop 평가 ({out})", "",
             f"에피소드 {len(eps)}개. 모델은 학습 seed별 평균의 평균 ± 표준편차(seed 수 n), 전문가는 에피소드 평균이다.", "",
             "| 지표 | 좋은 방향 | " + " | ".join(f"{g} (n={len(groups[g])})" for g in order) + " |",
             "|---|---|" + "---|" * len(order)]
    for c, label, _, better in COLUMNS:
        cells = []
        for g in order:
            vals = [per_agent[n][c] for n in groups[g] if per_agent[n][c] is not None]
            if not vals:
                cells.append("-")
            elif len(vals) == 1:
                cells.append(_fmt(vals[0]))
            else:
                cells.append(_fmt(statistics.mean(vals), statistics.stdev(vals)))
        direction = {"+": "높게", "-": "낮게"}.get(better, "")
        lines.append(f"| {label} | {direction} | " + " | ".join(cells) + " |")
    lines += ["", "## 모델별 끝 원인(에피소드 수)", "", "| 에이전트 | 에피소드 | 끝 원인 |", "|---|---|---|"]
    for name, v in per_agent.items():
        lines.append(f"| {name} | {v['episodes']} | {', '.join(f'{k} {n}' for k, n in v['end_reasons'].items())} |")
    lines += _extra_sections(results, eps, groups, order, out)
    text = "\n".join(lines) + "\n"
    with open(os.path.join(out, "summary.md"), "w", encoding="utf-8") as f:
        f.write(text)
    print(text)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="모델들 + 전문가를 seed마다 한 바퀴씩(하위 프로세스로 병렬)")
    r.add_argument("--models", nargs="+", required=True, help="train_ego_attention_bc.py 출력들이 있는 폴더(하위 폴더를 찾는다)")
    r.add_argument("--out", required=True)
    r.add_argument("--seeds", default=DEFAULT_SEEDS, help="예: 1000-1019 또는 1000,1001")
    r.add_argument("--jobs", type=int, default=min(8, os.cpu_count() or 1))
    r.add_argument("--no-expert", action="store_true")
    r.add_argument("--allow-unfinished", action="store_true", help="audit.json이 없는(학습 중인) 모델도 평가한다")
    r.add_argument("--warmup", type=float, default=None, help="기본: 월드마다(hapjeong 120, sumo_lc 0)")
    r.add_argument("--world", choices=sorted(WORLDS), default="hapjeong",
                   help="sumo_lc: 학습 데이터 data/pdm_bus_sumo_lc를 만든 월드(busy, S2·S3 없음). 관측 33차원 정책은 이것으로")
    r.add_argument("--max-seconds", type=float, default=1800.0)
    r.add_argument("--stall-seconds", type=float, default=180.0, help="이 시간 동안 5 m보다 덜 움직이면 멈춰 섬으로 끝낸다(0: 끔)")
    r.add_argument("--dry-run", action="store_true")
    o = sub.add_parser("one", help="에피소드 하나")
    o.add_argument("--agent", required=True, help="모델 폴더(model.pt, run_config.json) 또는 expert")
    o.add_argument("--seed", type=int, required=True)
    o.add_argument("--out", required=True)
    o.add_argument("--name", default=None, help="결과 폴더 이름(기본: 모델 폴더의 마지막 두 단계)")
    o.add_argument("--warmup", type=float, default=None, help="기본: 월드마다(hapjeong 120, sumo_lc 0)")
    o.add_argument("--world", choices=sorted(WORLDS), default="hapjeong")
    o.add_argument("--max-seconds", type=float, default=1800.0)
    o.add_argument("--stall-seconds", type=float, default=180.0)
    o.add_argument("--keep-work", action="store_true")
    s = sub.add_parser("summarize", help="result.json들로 표를 다시 만든다")
    s.add_argument("out")
    s2 = sub.add_parser("reanalyze", help="기록(run/)에서 지표를 다시 계산해 result.json의 metrics를 바꾼 뒤 요약한다")
    s2.add_argument("out")
    s2.add_argument("--stall-seconds", type=float, default=None, help="기본: 각 result.json에 남은 값(없으면 180)")
    a = ap.parse_args(argv)
    if a.cmd == "run":
        run_many(a)
    elif a.cmd == "one":
        run_one(a.agent, a.seed, a.out, a.name, a.warmup, a.max_seconds, a.keep_work, a.stall_seconds, a.world)
    elif a.cmd == "reanalyze":
        reanalyze(a.out, a.stall_seconds)
    else:
        summarize(a.out)


if __name__ == "__main__":
    main()
