#!/usr/bin/env bash
# run_map322.sh(3단계 교통량, DAgger 12회)가 끝난 뒤에 이어서 돌린다. 교통량 1.5배(heavy)를 더해 DAgger를 4회 더 하고,
# 네 교통량 새 seed 80번으로 평가하고, 통과하면 그 모델로 제출 데이터를 모은다. 끊기면 같은 명령으로 다시 돌린다.
#
#     bash research/av_bc_rl/run_heavy.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
export SUMO_HOME="${SUMO_HOME:-/Users/ky/src/sumo-1.27.1-mesa}"
PY="${PY:-/Users/ky/miniforge3/envs/sumo-rl/bin/python}"
W="${WORKERS:-13}"; S="${SEED:-1}"; N="${ROUNDS:-16}"; D=data/research/map322; R=results/n_dagger_s$S
TRAIN="--epochs 40 --batch-size 512 --loss bins --decode local --blind compact"
until [ -f data/research/verify_map322/filter.json ]; do sleep 30; done   # 앞 단계의 마지막 산출물

[ -f "$R/bc_it$N/model.pt" ] || OMP_NUM_THREADS=2 nice -n 10 "$PY" research/av_bc_rl/dagger.py --data "$D/demo_clean.npz" --model "results/n_bc_s$S/model.pt" \
    --iterations "$N" --episodes 25 --seed "$S" --workers "$W" --out "$R" --overlap --augment-signals \
    --scenarios quiet baseline busy heavy $TRAIN
out=data/research/verify_map322_heavy; mkdir -p "$out"
for pair in quiet:7000 baseline:7300 busy:7600 heavy:7900; do
    [ -f "$out/${pair%%:*}.npz" ] || nice -n 10 "$PY" collect_pomdp_data.py --map hapjeong --scenario "${pair%%:*}" --seed "${pair##*:}" --policy bc \
        --model "$R/bc_it$N/model.pt" --episodes 20 --workers "$W" --out-dir "$out" --name "${pair%%:*}" > "$out/${pair%%:*}.log" 2>&1
done
"$PY" research/av_bc_rl/filter_episodes.py "$out"/quiet.jsonl.gz "$out"/baseline.jsonl.gz "$out"/busy.jsonl.gz "$out"/heavy.jsonl.gz --report "$out/filter.json"
"$PY" research/av_bc_rl/render_best_worst.py "$out" --out "${VIDEOS:-reports/videos}" --prefix model_ --label "학습한 신경망" || true

# 판정: 80번 중 완주 95% 이상, 충돌 2번 이하면 16회 모델과 네 교통량으로 모은다. 아니면 12회 모델(세 교통량)을 본다.
verdict() { "$PY" - "$1" <<'PYEOF'
import collections, glob, gzip, json, sys
ends = collections.Counter()
for path in glob.glob(sys.argv[1] + "/*.jsonl.gz"):
    with gzip.open(path, "rt") as f:
        for line in f:
            if '"termination_reason": null' in line:
                continue
            reason = json.loads(line)["event"].get("termination_reason")
            if reason:
                ends[reason] += 1
total = sum(ends.values())
print(sys.argv[1], dict(ends))
sys.exit(0 if ends["arrived"] >= 0.95 * total and ends["collision"] + ends["pedestrian_collision"] <= total // 40 else 1)
PYEOF
}
if verdict "$out"; then
    MODEL="$R/bc_it$N/model.pt" WORKERS="$W" bash research/av_bc_rl/collect_final.sh
elif verdict data/research/verify_map322; then
    echo "16회 모델 미달, 12회 모델과 세 교통량으로 모은다"
    MODEL="$R/bc_it12/model.pt" WORKERS="$W" SCENARIOS="quiet:1 baseline:2 busy:3" bash research/av_bc_rl/collect_final.sh
else
    echo "두 모델 모두 기준 미달. 수집하지 않는다"; exit 1
fi
