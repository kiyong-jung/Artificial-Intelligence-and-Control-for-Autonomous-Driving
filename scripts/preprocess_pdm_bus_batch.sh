#!/usr/bin/env bash
# 수집이 끝난 PDM 버스 seed를 병렬로 전처리한다(scripts/preprocess_pdm_bus.py를 seed마다 한 번 실행).
#
#   scripts/preprocess_pdm_bus_batch.sh          # 아직 v2로 전처리하지 않은 seed 전부, 6개 동시
#   scripts/preprocess_pdm_bus_batch.sh 4        # 4개 동시
#   scripts/preprocess_pdm_bus_batch.sh count    # 전처리 상태만 센다
#
# - 수집이 끝난 seed(<out>/<이름>/metadata.json이 있음)만 한다. .partial 폴더는 건너뛴다.
# - 이미 관측 v2로 전처리한 seed(processed/metadata.json에 obs_v2_names가 있음)는 건너뛴다. 중간에 멈춰도 같은 명령으로 이어 간다.
# - 로그는 <out>/_preprocess_logs/<이름>.log, 실패한 seed는 _preprocess_logs/failed.txt.
# - 환경 변수: PYTHON_EXE(D:/conda_envs/sumo/python.exe), OUT_DIR(data/pdm_bus), SCENARIO(baseline),
#   ACCEL_POS(1.5), ACCEL_NEG(3.0) - 가감속 라벨 정규화 상수. 모든 seed가 같은 값이어야 한다.
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_EXE="${PYTHON_EXE:-D:/conda_envs/sumo/python.exe}"
OUT_DIR="${OUT_DIR:-$ROOT/data/pdm_bus}"
SCENARIO="${SCENARIO:-baseline}"
ACCEL_POS="${ACCEL_POS:-1.5}"
ACCEL_NEG="${ACCEL_NEG:-3.0}"
LOG_DIR="$OUT_DIR/_preprocess_logs"

is_done() {   # $1 = seed 폴더
    [ -f "$1/processed/transitions.npz" ] && grep -q obs_v2_names "$1/processed/metadata.json" 2>/dev/null
}

if [ "${1:-}" = "count" ]; then
    total=0; done_n=0
    for d in "$OUT_DIR/${SCENARIO}"_s*/; do
        d="${d%/}"; [ -f "$d/metadata.json" ] || continue
        total=$((total + 1)); is_done "$d" && done_n=$((done_n + 1))
    done
    echo "${SCENARIO}: 수집 끝난 seed ${total}개 중 v2 전처리 ${done_n}개, 남은 것 $((total - done_n))개"
    exit 0
fi

JOBS="${1:-6}"
mkdir -p "$LOG_DIR"
todo=()
for d in "$OUT_DIR/${SCENARIO}"_s*/; do
    d="${d%/}"
    [ -f "$d/metadata.json" ] || continue
    is_done "$d" || todo+=("$d")
done
echo "전처리할 seed ${#todo[@]}개, 동시 ${JOBS}개 (accel ${ACCEL_POS}/${ACCEL_NEG})"
[ ${#todo[@]} -eq 0 ] && exit 0

export PYTHON_EXE ROOT ACCEL_POS ACCEL_NEG LOG_DIR PYTHONIOENCODING=utf-8
printf '%s\n' "${todo[@]}" | xargs -P "$JOBS" -I{} bash -c '
    d="{}"; name="$(basename "$d")"
    if "$PYTHON_EXE" "$ROOT/scripts/preprocess_pdm_bus.py" "$d" --accel-pos "$ACCEL_POS" --accel-neg "$ACCEL_NEG" \
            > "$LOG_DIR/$name.log" 2>&1; then
        echo "ok   $name: $(tail -1 "$LOG_DIR/$name.log")"
    else
        echo "FAIL $name (로그: $LOG_DIR/$name.log)"; echo "$name" >> "$LOG_DIR/failed.txt"
    fi'
"$0" count
