#!/usr/bin/env bash
# 도로망 3.22.0(좌회전은 가장 왼쪽 차로만)에서 입력 31개로 시범 수집부터 DAgger, 새 seed 60번 평가까지 돌린다.
# 끝난 단계와 끝난 DAgger 반복은 건너뛰므로 끊기면 같은 명령으로 다시 돌리면 된다.
#
#     bash research/av_bc_rl/run_map322.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
export SUMO_HOME="${SUMO_HOME:-/Users/ky/src/sumo-1.27.1-mesa}"
PY="${PY:-/Users/ky/miniforge3/envs/sumo-rl/bin/python}"
W="${WORKERS:-10}"; S="${SEED:-1}"; N="${ROUNDS:-12}"; D=data/research/map322; R=results/n_dagger_s$S
TRAIN="--epochs 40 --batch-size 512 --loss bins --decode local --blind compact"
mkdir -p "$D"

for pair in quiet:2000 baseline:2300 busy:2600; do
    name=demo_${pair%%:*}
    [ -f "$D/$name.npz" ] || nice -n 10 "$PY" collect_pomdp_data.py --map hapjeong --scenario "${pair%%:*}" --seed "${pair##*:}" \
        --policy expert-v7 --episodes 100 --workers "$W" --max-steps 1400 --out-dir "$D" --name "$name" > "$D/$name.log" 2>&1
done
if [ ! -f "$D/demo_clean.npz" ]; then
    "$PY" research/av_bc_rl/filter_episodes.py "$D"/demo_quiet.jsonl.gz "$D"/demo_baseline.jsonl.gz "$D"/demo_busy.jsonl.gz --report "$D/filter_report.json"
    "$PY" research/av_bc_rl/merge_datasets.py "$D/demo_clean.npz" "$D"/demo_quiet.npz "$D"/demo_baseline.npz "$D"/demo_busy.npz --filter-report "$D/filter_report.json"
fi
[ -f "$D/demo_aug.npz" ] || "$PY" research/av_bc_rl/augment_signals.py --data "$D/demo_clean.npz" --out "$D/demo_aug.npz"
[ -f "results/n_bc_s$S/model.pt" ] || OMP_NUM_THREADS=2 nice -n 10 "$PY" train_bc.py --data "$D/demo_aug.npz" --seed "$S" --device cpu $TRAIN --out-dir "results/n_bc_s$S"
[ -f "$R/bc_it$N/model.pt" ] || OMP_NUM_THREADS=2 nice -n 10 "$PY" research/av_bc_rl/dagger.py --data "$D/demo_clean.npz" --model "results/n_bc_s$S/model.pt" \
    --iterations "$N" --episodes 25 --seed "$S" --workers "$W" --out "$R" --overlap --augment-signals $TRAIN
out=data/research/verify_map322; mkdir -p "$out"
for pair in quiet:7000 baseline:7300 busy:7600; do
    [ -f "$out/${pair%%:*}.npz" ] || nice -n 10 "$PY" collect_pomdp_data.py --map hapjeong --scenario "${pair%%:*}" --seed "${pair##*:}" --policy bc \
        --model "$R/bc_it$N/model.pt" --episodes 20 --workers "$W" --out-dir "$out" --name "${pair%%:*}" > "$out/${pair%%:*}.log" 2>&1
done
"$PY" research/av_bc_rl/filter_episodes.py "$out"/quiet.jsonl.gz "$out"/baseline.jsonl.gz "$out"/busy.jsonl.gz --report "$out/filter.json"
