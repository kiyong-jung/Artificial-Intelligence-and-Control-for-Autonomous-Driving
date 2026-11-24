#!/usr/bin/env bash
# 신경망 크기(폭x은닉층 수)를 바꿔 같은 데이터로 학습하고 같은 새 seed 60번을 몰아 비교한다(256은 DAgger가 낸 모델과 같은 조건의 재학습).
#     DATA=<합본 npz> OUT=<결과 폴더> bash research/av_bc_rl/size_trial.sh 256x2 256x4 ffn2   # ffnN은 가운데가 4배 넓은 잔차 블록 N개
set -euo pipefail
cd "$(dirname "$0")/../.."
export SUMO_HOME="${SUMO_HOME:-/Users/ky/src/sumo-1.27.1-mesa}"
PY="${PY:-/Users/ky/miniforge3/envs/sumo-rl/bin/python}"; W="${WORKERS:-4}"
: "${DATA:?}" "${OUT:?}"
for h in "$@"; do
    m="$OUT/bc_h$h"
    case "$h" in ffn*) size="--hidden 256 --blocks ${h#ffn}";; *) size="--hidden ${h%%x*} --depth ${h##*x}";; esac
    [ -f "$m/model.pt" ] || OMP_NUM_THREADS=2 nice -n 10 "$PY" train_bc.py --data "$DATA" --seed 1 --device cpu --epochs 40 --batch-size 512 \
        --loss bins --decode local --blind compact $size --out-dir "$m" > "$OUT/train_h$h.log" 2>&1
    for pair in quiet:7000 baseline:7300 busy:7600; do
        [ -f "$OUT/h${h}_${pair%%:*}.npz" ] || nice -n 10 "$PY" collect_pomdp_data.py --map hapjeong --scenario "${pair%%:*}" --seed "${pair##*:}" --policy bc \
            --model "$m/model.pt" --episodes 20 --workers "$W" --max-steps 1400 --out-dir "$OUT" --name "h${h}_${pair%%:*}" > "$OUT/h${h}_${pair%%:*}.log" 2>&1
    done
done
