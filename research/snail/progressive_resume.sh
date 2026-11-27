#!/bin/bash
# 1) rd5에서 DAgger 3회 더(다섯 교통량, rich2 포함)  2) rd5와 새 체크포인트 셋을 seed 63000에서 96바퀴씩 평가해 고름
# 3) 고른 모델로 다섯 교통량을 돌아가며 1,000만 건 수집. 각 단계는 끝난 것을 건너뛰므로 같은 명령으로 다시 돌린다.
set -u
cd "/Users/ky/work/SKKU_장학_공모전/강의/자율주행인공지능및제어/Artificial-Intelligence-and-Control-for-Autonomous-Driving/.claude/worktrees/snail"
export SUMO_HOME=/Users/ky/src/sumo-1.27.1-mesa PYTHONPATH=.
PY=/Users/ky/miniforge3/envs/sumo-rl/bin/python
D=results/rich_dagger_s2
echo "$(date '+%H:%M') DAgger 시작"
$PY research/av_bc_rl/dagger.py --data results/rich_dagger_s1/dagger_s1_it5.npz \
  --model results/rich_dagger_s1/bc_it5/model.pt --iterations 3 --episodes 25 \
  --scenarios quiet baseline busy heavy rich2 --episode-base 30000000 --seed 2 --workers 12 \
  --out $D --epochs 40 --batch-size 512 --loss bins --decode local --blind compact --overlap --augment-signals || exit 1
echo "$(date '+%H:%M') DAgger 끝"
CANDS="rd5=results/rich_dagger_s1/bc_it5/model.pt r6=$D/bc_it1/model.pt r7=$D/bc_it2/model.pt r8=$D/bc_it3/model.pt"
for c in $CANDS; do
  label=${c%%=*}; model=${c#*=}
  for s in quiet baseline busy heavy rich2; do
    [ -f data/research/eval/${label}_s63000_${s}.metadata.json ] || \
      $PY research/av_bc_rl/evaluate.py --policy bc --model "$model" --label "$label" --seed 63000 --scenarios "$s" --episodes 96 --workers 12
  done
done
$PY research/snail/pick_checkpoint.py --seed 63000 $CANDS > research/snail/pick_checkpoint.txt || exit 1
cat research/snail/pick_checkpoint.txt
BEST=$(tail -1 research/snail/pick_checkpoint.txt)
echo "$(date '+%H:%M') 수집 시작: $BEST"
MODEL="$BEST" OUT=data/final_v2 WORKERS=12 SCENARIOS="quiet:1 baseline:2 busy:3 heavy:4 rich2:5" bash research/av_bc_rl/collect_final.sh
echo "$(date '+%H:%M') PIPELINE_DONE"
