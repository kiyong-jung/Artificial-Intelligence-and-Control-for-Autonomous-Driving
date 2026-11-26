#!/usr/bin/env bash
# GPU 서버로 옮길 행동 복제 학습 데이터를 tar 하나로 묶는다(기본: SUMO-LC busy 데이터, 관측 33차원 MLP).
#
#   scripts/pack_bc_data.sh                     # -> bc_data.tar (저장소 루트)
#   scripts/pack_bc_data.sh D:/transfer/bc.tar  # 출력 경로 지정
#
# 넣는 것(저장소 루트 기준 상대 경로 그대로):
#   <DATA_DIR>/<seed>/processed/transitions.npz, processed/metadata.json  (MARK가 metadata.json에 있는 seed만)
#   <DATA_DIR>/bc_data_files.txt   학습에 쓸 npz 목록(목록 파일 기준 상대 경로). seed 번호 순서로 고정한다(분할·resume이 파일 순서에 의존)
#   ref/rl-agents/rl_agents        WITH_RL_AGENTS=1일 때만. 학습 스크립트가 utils/ego_attention.py를 import하므로
#                                  서버에 아직 없으면 한 번은 넣어야 한다(이미 풀어 두었으면 필요 없다)
# transitions.jsonl.gz와 원시 데이터(steps.jsonl.gz 등)는 학습에 쓰지 않아 넣지 않는다.
# npz는 이미 압축되어 있어 tar는 압축하지 않는다.
#
# 환경 변수: DATA_DIR(data/pdm_bus_sumo_lc), SCENARIO(busy), MARK(obs33_names, 관측 33차원까지 전처리한 표시), WITH_RL_AGENTS(0)
set -eu

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/bc_data.tar}"
DATA_DIR="${DATA_DIR:-data/pdm_bus_sumo_lc}"
SCENARIO="${SCENARIO:-busy}"
MARK="${MARK:-obs33_names}"
WITH_RL_AGENTS="${WITH_RL_AGENTS:-0}"
cd "$ROOT"
DATA_DIR="${DATA_DIR%/}"

list="$DATA_DIR/bc_data_files.txt"
: > "$list.tmp"
for d in "$DATA_DIR/${SCENARIO}"_s*/; do
    d="${d%/}"
    if [ -f "$d/processed/transitions.npz" ] && grep -q "$MARK" "$d/processed/metadata.json" 2>/dev/null; then
        echo "${d#"$DATA_DIR"/}/processed/transitions.npz" >> "$list.tmp"
    fi
done
sort -V "$list.tmp" -o "$list.tmp"
mv "$list.tmp" "$list"
n=$(wc -l < "$list")
[ "$n" -gt 0 ] || { echo "$DATA_DIR에 $MARK로 전처리한 seed가 없습니다(scripts/preprocess_pdm_bus_batch.sh 먼저)"; exit 1; }

files="$(mktemp)"
echo "$list" > "$files"
sed "s#^#$DATA_DIR/#" "$list" >> "$files"
sed "s#^#$DATA_DIR/#; s#transitions.npz\$#metadata.json#" "$list" >> "$files"
[ "$WITH_RL_AGENTS" = "1" ] && echo "ref/rl-agents/rl_agents" >> "$files"
tar --force-local --exclude='__pycache__' -cf "$OUT" -T "$files"   # --force-local: C:/... 를 원격 호스트로 읽지 않게
rm -f "$files"
echo "묶음: $OUT ($(du -h "$OUT" | cut -f1), seed ${n}개$([ "$WITH_RL_AGENTS" = "1" ] && echo ', rl-agents 포함'))"
echo "목록: $list"
