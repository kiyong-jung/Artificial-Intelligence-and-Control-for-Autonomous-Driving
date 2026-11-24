#!/usr/bin/env bash
# 끊긴 DAgger 추가 학습(results/g_dagger_s1)을 마지막으로 끝난 회차에서 이어 돌린다.
#
#     bash research/av_bc_rl/resume_dagger.sh          # 남은 회차를 돌리고 새 seed 60번으로 평가한다
#
# 회차가 끝날 때마다 results/g_dagger_s1*/bc_itN/model.pt와 dagger_s*_itN.npz가 남는다. 그중 가장 뒤의 것을 찾아
# 거기서 시작한다. 하나도 없으면 5회 모델(results/f_dagger_s1)에서 처음부터 다시 돈다. 목표는 모두 10회(추가 5회)다.
set -euo pipefail
cd "$(dirname "$0")/../.."
export SUMO_HOME="${SUMO_HOME:-/Users/ky/src/sumo-1.27.1-mesa}"
PY="${PY:-/Users/ky/miniforge3/envs/sumo-rl/bin/python}"
TOTAL=5

done_rounds=0; data=results/f_dagger_s1/dagger_s1_it5.npz; model=results/f_dagger_s1/bc_it5/model.pt
for dir in results/g_dagger_s1 results/g_dagger_s1_r*; do
    [ -d "$dir" ] || continue
    for k in 1 2 3 4 5; do
        npz=$(ls "$dir"/dagger_s*_it"$k".npz 2>/dev/null | head -1 || true)
        if [ -f "$dir/bc_it$k/model.pt" ] && [ -n "$npz" ]; then
            done_rounds=$((done_rounds + 1)); data=$npz; model=$dir/bc_it$k/model.pt
        fi
    done
done
left=$((TOTAL - done_rounds))
echo "끝난 추가 회차: $done_rounds / $TOTAL. 이어서 시작할 모델: $model"
if [ "$left" -gt 0 ]; then
    nice -n 19 "$PY" research/av_bc_rl/dagger.py --data "$data" --model "$model" --iterations "$left" --episodes 40 \
        --seed $((11 + done_rounds)) --workers "${WORKERS:-8}" --out "results/g_dagger_s1_r$done_rounds" \
        --augment-signals --epochs 40 --batch-size 512 --loss bins --decode local
    model=$(ls -d results/g_dagger_s1_r"$done_rounds"/bc_it* | sort -V | tail -1)/model.pt
fi
echo "평가할 모델: $model"
out=data/research/verify_10rounds; rm -rf "$out"; mkdir -p "$out"
for pair in quiet:7000 baseline:7300 busy:7600; do
    nice -n 19 "$PY" collect_pomdp_data.py --map hapjeong --scenario "${pair%%:*}" --seed "${pair##*:}" --policy bc --model "$model" \
        --episodes 20 --workers 6 --out-dir "$out" --name "${pair%%:*}" > "$out/${pair%%:*}.log" 2>&1
done
"$PY" research/av_bc_rl/filter_episodes.py "$out"/quiet.jsonl.gz "$out"/baseline.jsonl.gz "$out"/busy.jsonl.gz
