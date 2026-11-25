"""scripts/run_bc_experiments.py 결과를 실험별(seed 평균 ± 표준편차) 표로 모은다.

    python scripts/summarize_bc_experiments.py results/bc_v2
    python scripts/summarize_bc_experiments.py results/bc_v2 --sort val_move_recall

출력: 화면에 markdown 표, <out-root>/summary.csv(실행 하나당 한 줄), <out-root>/summary.md.
정렬 기준은 validation 지표다(test로 실험을 고르면 test가 더는 독립 평가가 아니다). test 열은 마지막 확인용.
아직 안 끝난 실행은 history.jsonl로 진행 에폭만 보여 준다.
"""
import argparse
import csv
import json
import statistics
from pathlib import Path

# (열 이름, 분할, 지표 키, 높을수록 좋은가)
COLUMNS = [
    ("val_lane_bal", "validation", "lane_balanced_acc", True),
    ("val_move_recall", "validation", "lane_move_recall", True),
    ("val_move_prec", "validation", "lane_move_precision", True),
    ("val_keep_to_move", "validation", "keep_to_move", False),
    ("val_blink_to_move", "validation", "blink_to_move", False),
    ("val_accel_mae", "validation", "accel_mae", False),
    ("val_accel_bin_acc", "validation", "accel_bin_acc", True),
    ("test_lane_bal", "test", "lane_balanced_acc", True),
    ("test_move_recall", "test", "lane_move_recall", True),
    ("test_move_prec", "test", "lane_move_precision", True),
    ("test_keep_to_move", "test", "keep_to_move", False),
    ("test_accel_mae", "test", "accel_mae", False),
]


def runs(root: Path):
    for audit in sorted(root.glob("*/seed*/audit.json")):
        a = json.loads(audit.read_text(encoding="utf-8"))
        row = {"experiment": audit.parent.parent.name, "seed": audit.parent.name[4:], "done": True,
               "selected_epoch": a["selected_epoch"], "epochs_run": a.get("epochs_run"),
               "early_stopped": a.get("early_stopped")}
        for col, split, key, _ in COLUMNS:
            row[col] = a["final"][split].get(key)
        yield row
    for hist in sorted(root.glob("*/seed*/history.jsonl")):
        if not (hist.parent / "audit.json").is_file():
            n = sum(1 for ln in hist.read_text(encoding="utf-8").splitlines() if ln.strip())
            yield {"experiment": hist.parent.parent.name, "seed": hist.parent.name[4:], "done": False, "epochs_run": n}


def fmt(values):
    v = [x for x in values if x is not None]
    if not v:
        return "-"
    if all(float(x).is_integer() for x in v) and max(v) > 1:
        return f"{statistics.mean(v):.1f}" + (f"±{statistics.stdev(v):.1f}" if len(v) > 1 else "")
    return f"{statistics.mean(v):.3f}" + (f"±{statistics.stdev(v):.3f}" if len(v) > 1 else "")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_root")
    ap.add_argument("--sort", default="val_lane_bal", choices=[c[0] for c in COLUMNS])
    a = ap.parse_args(argv)
    root = Path(a.out_root)
    rows = list(runs(root))
    done = [r for r in rows if r["done"]]
    if rows:
        keys = ["experiment", "seed", "done", "selected_epoch", "epochs_run", "early_stopped"] + [c[0] for c in COLUMNS]
        with (root / "summary.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)

    groups = {}
    for r in done:
        groups.setdefault(r["experiment"], []).append(r)
    higher = {c[0]: c[3] for c in COLUMNS}

    def key(name):
        v = [r[a.sort] for r in groups[name] if r[a.sort] is not None]
        m = statistics.mean(v) if v else float("nan")
        return -m if higher[a.sort] else m

    shown = ["val_lane_bal", "val_move_recall", "val_move_prec", "val_keep_to_move", "val_blink_to_move",
             "val_accel_mae", "test_lane_bal", "test_move_recall", "test_move_prec", "test_keep_to_move", "test_accel_mae"]
    lines = ["| 실험 | seed | 선택 에폭 | " + " | ".join(shown) + " |", "|" + "---|" * (len(shown) + 3)]
    for name in sorted(groups, key=key):
        g = groups[name]
        lines.append(f"| {name} | {len(g)} | {fmt([r['selected_epoch'] for r in g])} | "
                     + " | ".join(fmt([r[c] for r in g]) for c in shown) + " |")
    pending = [r for r in rows if not r["done"]]
    if pending:
        lines += ["", "진행 중: " + ", ".join(f"{r['experiment']}/seed{r['seed']}({r['epochs_run']}에폭)" for r in pending)]
    text = "\n".join(lines)
    (root / "summary.md").write_text(text + "\n", encoding="utf-8")
    print(text)
    print(f"\n끝난 실행 {len(done)}개, 진행 중 {len(pending)}개. 정렬: {a.sort}. 실행별: {root / 'summary.csv'}")


if __name__ == "__main__":
    main()
