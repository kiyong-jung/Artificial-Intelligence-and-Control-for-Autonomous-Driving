#!/usr/bin/env bash
# final seed(9000번대)로 모든 정책을 한 번씩 평가하고 표를 만든다. 결과는 metrics/<label>_final.json,
# 주행 기록은 data/research/eval/<label>_final_<교통량>.jsonl.gz다. 이미 있는 기록은 다시 돌리지 않는다.
#
#   bash research/av_bc_rl/run_final_eval.sh            # 교통량마다 20에피소드, 워커 12
#   EPISODES=10 WORKERS=6 bash research/av_bc_rl/run_final_eval.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
PY=${PY:-python}
EPISODES=${EPISODES:-20}
WORKERS=${WORKERS:-12}
M=research/av_bc_rl/models

run() {   # label policy [model]
	local label=$1 policy=$2 model=${3:-}
	if [ -n "$model" ]; then
		$PY research/av_bc_rl/evaluate.py --policy "$policy" --model "$model" --split final \
			--episodes "$EPISODES" --workers "$WORKERS" --label "$label"
	else
		$PY research/av_bc_rl/evaluate.py --policy "$policy" --split final \
			--episodes "$EPISODES" --workers "$WORKERS" --label "$label"
	fi
}

run rule route-follow
run expert expert
run expert_v5 expert-v5
for s in 0 1 2; do run bc_s$s bc $M/bc_s$s/model.pt; done
for s in 0 1 2; do run dagger_s$s bc $M/dagger_s$s/model.pt; done
for s in 2101 2102 2103; do run ppo_s$s ppo $M/ppo_s$s/model.pt; done
for s in 2201 2202 2203; do run residual_s$s ppo $M/residual_s$s/model.pt; done

$PY research/av_bc_rl/compare.py --split final --groups rule=rule expert=expert expert_v5=expert_v5 \
	bc=bc_s0,bc_s1,bc_s2 dagger=dagger_s0,dagger_s1,dagger_s2 ppo=ppo_s2101,ppo_s2102,ppo_s2103 \
	residual=residual_s2201,residual_s2202,residual_s2203 --out research/av_bc_rl/report/final_table.md
