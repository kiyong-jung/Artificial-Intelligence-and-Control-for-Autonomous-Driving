#!/usr/bin/env bash
# PDM 버스 원시 데이터를 seed 여러 개로 나눠 병렬 수집한다(scripts/collect_pdm_bus.py를 seed마다 한 번 실행).
#
#   scripts/collect_pdm_bus_batch.sh                  # 기본: baseline, seed 0~999, 시드당 5바퀴, 6개 동시 실행
#   scripts/collect_pdm_bus_batch.sh 0 19 6           # seed 0~19만 (시험 수집)
#   scripts/collect_pdm_bus_batch.sh count            # 지금까지 모은 transition 수만 센다
#
# 목표 5x10^6 transition: 한 바퀴 ~1,065개(baseline seed 0 실측 532.5 s) -> 약 4,700바퀴. seed 1,000 x 5바퀴 = 약 5.3x10^6.
# 모자라면 seed 범위를 늘려 다시 돌린다(끝난 seed는 건너뛴다).
#
# - 끝난 seed(<out>/<이름>/metadata.json이 있음)는 건너뛴다. 중간에 멈춰도 같은 명령으로 이어서 돌리면 된다.
# - 작업 폴더는 D 드라이브(WORK_DIR)에 두고 성공하면 지운다(C 드라이브 여유가 적다).
# - seed마다 로그를 <out>/_batch_logs/<이름>.log에 남기고, 실패한 seed는 _batch_logs/failed.txt에 적는다.
# - 환경 변수로 바꿀 수 있다: SCENARIO(baseline), LAPS(5), OUT_DIR(data/pdm_bus), WORK_DIR(D:/pdm_work),
#   EXTRA_ARGS(collect_pdm_bus.py에 더 넘길 인자, 예: "--warmup 30 --max-seconds 60").
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_EXE="/c/Users/user/anaconda3/envs/main/python.exe"
SCENARIO="${SCENARIO:-baseline}"
LAPS="${LAPS:-5}"
OUT_DIR="${OUT_DIR:-$ROOT/data/pdm_bus}"
WORK_DIR="${WORK_DIR:-D:/pdm_work}"
EXTRA_ARGS="${EXTRA_ARGS:-}"
LOG_DIR="$OUT_DIR/_batch_logs"

count() {
    PYTHONIOENCODING=utf-8 "$PYTHON_EXE" - "$OUT_DIR" "$SCENARIO" <<'PY'
import glob, json, os, sys
out, scen = sys.argv[1], sys.argv[2]
runs = laps = trans = 0
ends = {}
for meta in sorted(glob.glob(os.path.join(out, f"{scen}_s*", "metadata.json"))):
    r = json.load(open(meta, encoding="utf-8"))["result"]
    rec = r["recorder"]
    episodes = len(r["lap_times_s"]) + (0 if rec["end"] == "laps" else 1)
    runs, laps, trans = runs + 1, laps + len(r["lap_times_s"]), trans + rec["rows"] - episodes
    ends[rec["end"]] = ends.get(rec["end"], 0) + 1
print(f"{scen}: 끝난 seed {runs}개, 완주 바퀴 {laps}개, transition 약 {trans:,}개 (seed 끝 원인 {ends})")
PY
}

run_one() {
    local seed="$1" name="${SCENARIO}_s$1"
    if [ -f "$OUT_DIR/$name/metadata.json" ]; then
        echo "[skip] $name"
        return 0
    fi
    rm -rf "$OUT_DIR/.$name.partial" "$WORK_DIR/$name"
    local t0=$SECONDS
    # shellcheck disable=SC2086
    PYTHONIOENCODING=utf-8 "$PYTHON_EXE" "$ROOT/scripts/collect_pdm_bus.py" --scenario "$SCENARIO" --seed "$seed" \
        --laps "$LAPS" --name "$name" --out-dir "$OUT_DIR" --work-dir "$WORK_DIR/$name" $EXTRA_ARGS \
        > "$LOG_DIR/$name.log" 2>&1
    local rc=$?
    if [ $rc -eq 0 ] && [ -f "$OUT_DIR/$name/metadata.json" ]; then
        rm -rf "$WORK_DIR/$name"
        echo "[ok]   $name ($((SECONDS - t0)) s) $(tail -n 1 "$LOG_DIR/$name.log")"
    else
        echo "$seed" >> "$LOG_DIR/failed.txt"
        echo "[fail] $name (종료 코드 $rc, 로그 $LOG_DIR/$name.log)"
    fi
}

if [ "${1:-}" = "count" ]; then
    count
    exit 0
fi

START="${1:-0}"
END="${2:-999}"
JOBS="${3:-6}"
mkdir -p "$OUT_DIR" "$LOG_DIR" "$WORK_DIR"
: > "$LOG_DIR/failed.txt"                     # 이번 실행의 실패만 적는다(이전에 실패한 seed는 이번에 다시 시도된다)
export ROOT PYTHON_EXE SCENARIO LAPS OUT_DIR WORK_DIR EXTRA_ARGS LOG_DIR
export -f run_one

echo "수집: $SCENARIO seed $START~$END, 시드당 ${LAPS}바퀴, 동시 ${JOBS}개 -> $OUT_DIR (작업 폴더 $WORK_DIR)"
seq "$START" "$END" | xargs -P "$JOBS" -I{} bash -c 'run_one {}'
count
if [ -s "$LOG_DIR/failed.txt" ]; then
    echo "실패한 seed가 있다: $LOG_DIR/failed.txt (같은 명령을 다시 돌리면 끝난 seed는 건너뛰고 다시 시도한다)"
fi
