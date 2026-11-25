#!/usr/bin/env bash
# 교통량마다 규칙 운전사 v7과 it16 신경망을 같은 새 seed로 돌린다(exp/snail-traffic). 끊기면 같은 명령으로 다시 돌린다.
#
#     IT16=/절대/경로/model.pt bash research/av_bc_rl/run_sweep.sh heavy x2 x2p5 x3 x4
#     POLICIES=v7 SEED=60024 bash research/av_bc_rl/run_sweep.sh snail10v5
set -euo pipefail
cd "$(dirname "$0")/../.."
export SUMO_HOME="${SUMO_HOME:-/Users/ky/src/sumo-1.27.1-mesa}" PYTHONPATH=.
PY="${PY:-/Users/ky/miniforge3/envs/sumo-rl/bin/python}"
SEED="${SEED:-60000}"; N="${EPISODES:-24}"; W="${WORKERS:-12}"
for scenario in "$@"; do
    for policy in ${POLICIES:-v7 it16}; do
        case "$policy" in
            v7) "$PY" research/av_bc_rl/evaluate.py --policy expert-v7 --label v7 --seed "$SEED" --scenarios "$scenario" --episodes "$N" --workers "$W" ;;
            it16) "$PY" research/av_bc_rl/evaluate.py --policy bc --model "$IT16" --label it16 --seed "$SEED" --scenarios "$scenario" --episodes "$N" --workers "$W" ;;
        esac
    done
done
