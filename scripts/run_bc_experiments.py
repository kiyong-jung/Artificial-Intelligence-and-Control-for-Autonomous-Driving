"""행동 복제 실험 여러 개를 GPU(또는 CPU)에 나눠 동시에 돌린다. 멈춰도 같은 명령을 다시 치면 이어서 한다.

    python scripts/run_bc_experiments.py --config experiments/bc_v2_sweep.json \
        --data-list data/pdm_bus/bc_data_files.txt --out-root results/bc_v2 --gpus 0,1 --jobs-per-gpu 3
    python scripts/run_bc_experiments.py ... --only base,wide128 --seeds 1     # 일부만
    python scripts/run_bc_experiments.py ... --dry-run                         # 무엇을 돌릴지만 출력
    python scripts/summarize_bc_experiments.py results/bc_v2                   # 결과 표

실험 하나 = (실험 이름, seed). 결과는 <out-root>/<이름>/seed<번호>/, 로그는 <out-root>/logs/<이름>_seed<번호>.log.
- 끝난 실행(audit.json 있음)은 건너뛴다. 중간에 멈춘 실행은 checkpoint.pt에서 이어 간다(train_ego_attention_bc.py --resume).
- 실패하면 --retries번 다시 시도한다(이어서). 그래도 실패한 것은 run_status.json에 남는다.
- 분할은 설정의 split_seed로 고정한다. 그래서 seed만 다른 실행끼리, 실험끼리 같은 train/validation/test다.
- 처음 실행할 때 설정 파일을 <out-root>/experiment_config.json으로 복사해 둔다. 같은 out-root에 다른 설정을 쓰면 멈춘다
  (같은 이름의 실험이 다른 인자로 섞이지 않게). 실험을 더하려면 설정에 항목을 추가하고 --allow-config-change.
"""
import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TRAIN = ROOT / "scripts" / "train_ego_attention_bc.py"


def build_jobs(cfg, only, seeds):
    names = list(cfg["experiments"])
    if only:
        unknown = sorted(set(only) - set(names))
        if unknown:
            raise SystemExit(f"설정에 없는 실험: {unknown}")
        names = [n for n in names if n in only]
    return [(n, s) for n in names for s in (seeds or cfg["seeds"])]


def check_config(out_root: Path, cfg: dict, allow_change: bool):
    snap = out_root / "experiment_config.json"
    if snap.is_file():
        prev = json.loads(snap.read_text(encoding="utf-8"))
        changed = [n for n in prev["experiments"] if n in cfg["experiments"] and prev["experiments"][n] != cfg["experiments"][n]]
        if prev.get("common") != cfg.get("common") or prev.get("split_seed") != cfg.get("split_seed") or changed:
            raise SystemExit(f"{snap}와 설정이 다릅니다(common/split_seed/실험 {changed}). 다른 --out-root를 쓰세요")
        if set(cfg["experiments"]) - set(prev["experiments"]) and not allow_change:
            raise SystemExit("설정에 새 실험이 있습니다. 같은 out-root에 더하려면 --allow-config-change")
    out_root.mkdir(parents=True, exist_ok=True)
    snap.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def command(a, cfg, name, seed, out):
    return ([a.python, str(TRAIN), "--data-list", str(Path(a.data_list).resolve()), "--out", str(out), "--resume",
             "--seed", str(seed), "--split-seed", str(cfg["split_seed"]), "--device", "cpu" if a.gpus == "cpu" else "cuda"]
            + list(cfg.get("common", [])) + list(cfg["experiments"][name]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--data-list", required=True, help="npz 목록 파일(scripts/pack_bc_data.sh가 만든 bc_data_files.txt)")
    ap.add_argument("--out-root", required=True)
    ap.add_argument("--gpus", default="0", help="쉼표로 구분한 GPU 번호, 또는 cpu")
    ap.add_argument("--jobs-per-gpu", type=int, default=2, help="GPU 한 장에 동시에 돌릴 실행 수")
    ap.add_argument("--only", help="쉼표로 구분한 실험 이름만")
    ap.add_argument("--seeds", help="쉼표로 구분한 seed만(설정의 seeds 대신)")
    ap.add_argument("--retries", type=int, default=2, help="실패한 실행을 이어서 다시 시도할 횟수")
    ap.add_argument("--python", default=sys.executable, help="학습에 쓸 파이썬")
    ap.add_argument("--allow-config-change", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)

    cfg = json.loads(Path(a.config).read_text(encoding="utf-8"))
    out_root = Path(a.out_root)
    jobs = build_jobs(cfg, [s for s in (a.only or "").split(",") if s],
                      [int(s) for s in (a.seeds or "").split(",") if s.strip()])
    todo = [(n, s) for n, s in jobs if not (out_root / n / f"seed{s}" / "audit.json").is_file()]
    slots = ["cpu"] * a.jobs_per_gpu if a.gpus == "cpu" else \
        [g.strip() for g in a.gpus.split(",") if g.strip()] * a.jobs_per_gpu
    print(f"실행 {len(jobs)}개 중 남은 것 {len(todo)}개, 동시 {len(slots)}개 ({a.gpus})", flush=True)
    if a.dry_run:
        for n, s in todo:
            print(" ".join(command(a, cfg, n, s, out_root / n / f"seed{s}")))
        return 0
    if not Path(a.data_list).is_file():
        raise SystemExit(f"데이터 목록이 없습니다: {a.data_list}")
    check_config(out_root, cfg, a.allow_config_change)
    (out_root / "logs").mkdir(exist_ok=True)

    free, status, procs, lock = queue.Queue(), {}, set(), threading.Lock()
    for s in slots:
        free.put(s)
    stopping = threading.Event()

    def run(name, seed):
        out = out_root / name / f"seed{seed}"
        log = out_root / "logs" / f"{name}_seed{seed}.log"
        cmd = command(a, cfg, name, seed, out)
        slot = free.get()
        try:
            env = dict(os.environ, PYTHONIOENCODING="utf-8", OMP_NUM_THREADS="1")
            if slot != "cpu":
                env["CUDA_VISIBLE_DEVICES"] = slot
            for attempt in range(a.retries + 1):
                if stopping.is_set():
                    return
                with log.open("a", encoding="utf-8") as f:
                    f.write(f"\n===== {datetime.now().isoformat(timespec='seconds')} 시도 {attempt + 1} "
                            f"(장치 {slot})\n{' '.join(cmd)}\n")
                    f.flush()
                    t0 = time.time()
                    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, cwd=str(ROOT))
                    with lock:
                        procs.add(p)
                    code = p.wait()
                    with lock:
                        procs.discard(p)
                ok = code == 0 and (out / "audit.json").is_file()
                with lock:
                    status[f"{name}/seed{seed}"] = {"ok": ok, "code": code, "attempts": attempt + 1,
                                                    "minutes": round((time.time() - t0) / 60, 1), "device": slot}
                print(f"[{datetime.now():%H:%M:%S}] {'완료' if ok else '실패'} {name}/seed{seed} "
                      f"(코드 {code}, 시도 {attempt + 1}, {status[f'{name}/seed{seed}']['minutes']}분, 장치 {slot})",
                      flush=True)
                if ok or stopping.is_set():
                    return
        finally:
            free.put(slot)

    def stop(*_):
        stopping.set()
        with lock:
            for p in procs:
                p.terminate()
        print("중단 요청: 돌던 실행을 멈춥니다. 같은 명령으로 다시 실행하면 이어서 합니다.", flush=True)

    signal.signal(signal.SIGINT, stop)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, stop)
    threads = [threading.Thread(target=run, args=job, daemon=True) for job in todo]
    for t in threads:
        t.start()
    for t in threads:
        while t.is_alive():
            t.join(timeout=1.0)

    st_path = out_root / "run_status.json"
    old = json.loads(st_path.read_text(encoding="utf-8")) if st_path.is_file() else {}
    st_path.write_text(json.dumps({**old, **status}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    failed = [k for k, v in status.items() if not v["ok"]]
    print(f"끝: 이번에 {len(status)}개 실행, 실패 {len(failed)}개 {failed}", flush=True)
    print(f"결과 표: python scripts/summarize_bc_experiments.py {out_root}", flush=True)
    return 1 if failed or stopping.is_set() else 0


if __name__ == "__main__":
    sys.exit(main())
