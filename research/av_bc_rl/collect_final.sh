#!/usr/bin/env bash
# 학습한 모델로 제출용 데이터를 TARGET 건(기본 1,000만) 넘을 때까지 모은다.
# 교통량 셋을 돌아가며 200바퀴씩 한 묶음으로 모으고, 끝난 묶음은 건너뛰므로 끊기면 같은 명령으로 다시 돌린다.
# 팀원이 나눠 모을 때는 SHARD를 서로 다르게 준다(seed 범위가 겹치지 않는다).
#
#     MODEL=results/n_dagger_s1/bc_it12/model.pt bash research/av_bc_rl/collect_final.sh
#     MODEL=... SHARD=1 TARGET=3000000 OUT=data/final_shard1 bash research/av_bc_rl/collect_final.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
export SUMO_HOME="${SUMO_HOME:-/Users/ky/src/sumo-1.27.1-mesa}"
PY="${PY:-/Users/ky/miniforge3/envs/sumo-rl/bin/python}"
W="${WORKERS:-13}"; OUT="${OUT:-data/final}"; TARGET="${TARGET:-10000000}"; SHARD="${SHARD:-0}"; CHUNK="${CHUNK:-200}"
: "${MODEL:?MODEL=model.pt 경로를 준다}"
mkdir -p "$OUT"
total() { "$PY" -c "import glob,json,sys; print(sum(json.load(open(f))['transitions'] for f in glob.glob(sys.argv[1]+'/*.metadata.json')))" "$OUT"; }
k=0
while [ "$(total)" -lt "$TARGET" ]; do
    for pair in ${SCENARIOS:-quiet:1 baseline:2 busy:3 heavy:4}; do
        name=${pair%%:*}_s${SHARD}_$(printf %03d "$k")
        seed=$(( ${pair##*:} * 10000000 + SHARD * 1000000 + k * CHUNK ))
        [ -f "$OUT/$name.metadata.json" ] || nice -n 10 "$PY" collect_pomdp_data.py --map hapjeong --scenario "${pair%%:*}" --seed "$seed" \
            --policy bc --model "$MODEL" --episodes "$CHUNK" --workers "$W" --max-steps 1400 --out-dir "$OUT" --name "$name" > "$OUT/$name.log" 2>&1
    done
    k=$((k + 1))
    echo "$(date '+%m-%d %H:%M') 묶음 $k 끝, 누적 $(total)건"
done
echo "끝: $(total)건"
