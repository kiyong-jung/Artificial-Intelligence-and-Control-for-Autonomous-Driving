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
  - 에피소드 끝: 충돌, 신호 위반, 정류장 놓침, 순간이동, 시간 한도, 바퀴 완료(정류장 4곳 모두 마치면 arrived).

출력 <out>/
  <agent>/seed<S>/run/    원시 기록(steps.jsonl.gz, events.jsonl, ssm.xml, metadata.json, assets/) - 학습 데이터와 같은 형식
  <agent>/seed<S>/result.json   지표(pdm_data/closed_loop.analyze_run)
  logs/  episodes.csv  stops.csv  agents.csv  summary.md
"""
import argparse
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
END_REASONS = ("arrived", "collision", "red_light", "missed_stop", "incomplete_service", "time_limit", "teleport",
               "bus_lost", "sumo_closed")
COLUMNS = [
    ("success", "완주율", "rate", "+"),
    ("rate_collision", "충돌률", "rate", "-"),
    ("rate_red_light", "신호 위반률", "rate", "-"),
    ("rate_missed_stop", "정류장 놓침률", "rate", "-"),
    ("rate_time_limit", "시간 초과률", "rate", "-"),
    ("rate_teleport", "순간이동률", "rate", "-"),
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
def run_one(agent, seed, out_root, name=None, warmup=120.0, max_seconds=1800.0, keep_work=False):
    import collect_pdm_bus
    from pdm_data.closed_loop import PolicyController, analyze_run, load_policy
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
    a = collect_pdm_bus.parse_args(["--scenario", "baseline", "--seed", str(seed), "--laps", "1",
                                    "--warmup", str(warmup), "--max-seconds", str(max_seconds),
                                    "--name", "run", "--out-dir", ep_dir, "--work-dir", work])
    controller = None
    if agent != EXPERT:
        import torch
        torch.set_num_threads(1)
        act, info = load_policy(agent)
        controller = PolicyController(act, info["accel_scale"], info)
    t0 = time.time()
    try:
        r = collect_pdm_bus.collect(a, controller=controller)
    finally:
        if not keep_work:
            shutil.rmtree(work, ignore_errors=True)
    metrics = analyze_run(r["run_dir"])
    result = {"agent": name, "agent_path": agent if agent == EXPERT else os.path.abspath(agent), "seed": seed,
              "scenario": "baseline", "collection_end": r["end"], "wall_s": round(time.time() - t0, 1),
              "warmup_s": warmup, "max_seconds": max_seconds,
              "policy": controller.describe() if controller is not None else None, "metrics": metrics}
    with open(res_path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
    os.replace(res_path + ".tmp", res_path)
    print(f"끝: {name} seed {seed} | {metrics['end_reason']} | 정차 {metrics.get('stops_completed')} | "
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
        cmd = [sys.executable, os.path.abspath(__file__), "one", "--agent", path, "--name", name, "--seed", str(seed),
               "--out", a.out, "--warmup", str(a.warmup), "--max-seconds", str(a.max_seconds)]
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
def _flat(res):
    m = res["metrics"]
    row = {"agent": res["agent"], "seed": res["seed"], "end_reason": m.get("end_reason")}
    for k, v in m.items():
        if k != "stops" and not isinstance(v, (dict, list)):
            row[k] = v
    for r in END_REASONS:
        row[f"rate_{r}"] = float(m.get("end_reason") == r)
    row["success"] = float(bool(m.get("success")))
    return row


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
    r.add_argument("--warmup", type=float, default=120.0)
    r.add_argument("--max-seconds", type=float, default=1800.0)
    r.add_argument("--dry-run", action="store_true")
    o = sub.add_parser("one", help="에피소드 하나")
    o.add_argument("--agent", required=True, help="모델 폴더(model.pt, run_config.json) 또는 expert")
    o.add_argument("--seed", type=int, required=True)
    o.add_argument("--out", required=True)
    o.add_argument("--name", default=None, help="결과 폴더 이름(기본: 모델 폴더의 마지막 두 단계)")
    o.add_argument("--warmup", type=float, default=120.0)
    o.add_argument("--max-seconds", type=float, default=1800.0)
    o.add_argument("--keep-work", action="store_true")
    s = sub.add_parser("summarize")
    s.add_argument("out")
    a = ap.parse_args(argv)
    if a.cmd == "run":
        run_many(a)
    elif a.cmd == "one":
        run_one(a.agent, a.seed, a.out, a.name, a.warmup, a.max_seconds, a.keep_work)
    else:
        summarize(a.out)


if __name__ == "__main__":
    main()
