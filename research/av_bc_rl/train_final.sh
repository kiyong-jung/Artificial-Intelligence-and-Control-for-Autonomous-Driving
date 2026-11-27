#!/usr/bin/env bash
# 최종 모델(DAgger 24회)을 처음부터 다시 학습하고, 수집에 쓸 체크포인트를 고른다. COLLECT=1이면 이어서 제출 데이터를 모은다.
# 명령은 models/final/model.pt를 만든 실행에서 그대로 옮겼다. 끝난 단계는 건너뛰므로 끊기면 같은 명령으로 다시 돌린다.
#
#     SUMO_HOME=/path/to/sumo-1.27.1 bash research/av_bc_rl/train_final.sh
#     COLLECT=1 SUMO_HOME=/path/to/sumo-1.27.1 bash research/av_bc_rl/train_final.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
: "${SUMO_HOME:?SUMO 1.27.1을 설치한 경로를 SUMO_HOME으로 준다}"
export SUMO_HOME PYTHONPATH=.
PY="${PY:-python}"; W="${WORKERS:-12}"
TRAIN=(--epochs 40 --batch-size 512 --loss bins --decode local --blind compact)
DAG=(--episodes 25 --workers "$W" --overlap --augment-signals "${TRAIN[@]}")
D=data/research/demo; A=results/dagger_1_16; B=results/dagger_17_21; C=results/dagger_22_24
mkdir -p "$D"

# 1) 시범: 규칙 운전사(expert-v7)가 한산·보통·혼잡에서 100바퀴씩 몬 기록으로 처음 모방학습한다.
for pair in quiet:2000 baseline:2300 busy:2600; do
    name=demo_${pair%%:*}
    [ -f "$D/$name.npz" ] || "$PY" collect_pomdp_data.py --map hapjeong --scenario "${pair%%:*}" --seed "${pair##*:}" \
        --policy expert-v7 --episodes 100 --workers "$W" --max-steps 1400 --out-dir "$D" --name "$name"
done
if [ ! -f "$D/demo_clean.npz" ]; then
    "$PY" research/av_bc_rl/filter_episodes.py "$D"/demo_quiet.jsonl.gz "$D"/demo_baseline.jsonl.gz "$D"/demo_busy.jsonl.gz --report "$D/filter.json"
    "$PY" research/av_bc_rl/merge_datasets.py "$D/demo_clean.npz" "$D"/demo_quiet.npz "$D"/demo_baseline.npz "$D"/demo_busy.npz --filter-report "$D/filter.json"
fi
[ -f "$D/demo_aug.npz" ] || "$PY" research/av_bc_rl/augment_signals.py --data "$D/demo_clean.npz" --out "$D/demo_aug.npz"
[ -f results/bc/model.pt ] || "$PY" train_bc.py --data "$D/demo_aug.npz" --seed 1 --device cpu "${TRAIN[@]}" --out-dir results/bc

# 2) DAgger 1~12회는 세 교통, 13~16회는 매우 혼잡(1.5배)을 더한다.
"$PY" research/av_bc_rl/dagger.py --data "$D/demo_clean.npz" --model results/bc/model.pt --iterations 12 --seed 1 --out "$A" "${DAG[@]}"
"$PY" research/av_bc_rl/dagger.py --data "$D/demo_clean.npz" --model results/bc/model.pt --iterations 16 --seed 1 --out "$A" \
    --scenarios quiet baseline busy heavy "${DAG[@]}"

# 3) DAgger 17~19회는 택시·골목 차·보행자가 있는 교통(rich_lo)을 더하고, 20~21회는 매우 혼잡과 그 교통만 몬다.
"$PY" research/av_bc_rl/dagger.py --data "$A/dagger_s1_it16.npz" --model "$A/bc_it16/model.pt" --iterations 3 --seed 1 --out "$B" \
    --scenarios quiet baseline busy heavy rich_lo --episode-base 10000000 "${DAG[@]}"
"$PY" research/av_bc_rl/dagger.py --data "$A/dagger_s1_it16.npz" --model "$A/bc_it16/model.pt" --iterations 5 --seed 1 --out "$B" \
    --scenarios heavy rich_lo --episode-base 20000000 "${DAG[@]}"

# 4) DAgger 22~24회는 최종 다섯 교통(복합 교통 rich2 포함)을 몬다.
"$PY" research/av_bc_rl/dagger.py --data "$B/dagger_s1_it5.npz" --model "$B/bc_it5/model.pt" --iterations 3 --seed 2 --out "$C" \
    --scenarios quiet baseline busy heavy rich2 --episode-base 30000000 "${DAG[@]}"

# 5) 21~24회 네 모델을 seed 63000에서 교통마다 96바퀴씩 돌려, 평가 전에 정한 기준(안전 사건이 가장 적은 것)으로 고른다.
CANDS=("m21=$B/bc_it5/model.pt" "m22=$C/bc_it1/model.pt" "m23=$C/bc_it2/model.pt" "m24=$C/bc_it3/model.pt")
for c in "${CANDS[@]}"; do
    for s in quiet baseline busy heavy rich2; do
        [ -f "data/research/eval/${c%%=*}_s63000_$s.jsonl.gz" ] || "$PY" research/av_bc_rl/evaluate.py --policy bc --model "${c#*=}" \
            --label "${c%%=*}" --seed 63000 --scenarios "$s" --episodes 96 --workers "$W"
    done
done
"$PY" research/traffic_checks/pick_checkpoint.py --seed 63000 "${CANDS[@]}" | tee results/pick_checkpoint.txt
BEST=$(tail -1 results/pick_checkpoint.txt)

# 6) 고른 모델로 다섯 교통을 돌아가며 200바퀴씩 모은다(제출 데이터는 13,000바퀴, 1,218만 건).
if [ "${COLLECT:-0}" = 1 ]; then
    MODEL="$BEST" OUT=data/final WORKERS="$W" TARGET="${TARGET:-12000000}" SCENARIOS="quiet:1 baseline:2 busy:3 heavy:4 rich2:5" \
        bash research/av_bc_rl/collect_final.sh
fi
