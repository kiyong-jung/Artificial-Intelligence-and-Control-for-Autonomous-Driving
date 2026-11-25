#!/usr/bin/env bash
# GPU 서버로 옮길 행동 복제 학습 데이터를 tar 하나로 묶는다.
#
#   scripts/pack_bc_data.sh                     # -> bc_data.tar (저장소 루트)
#   scripts/pack_bc_data.sh D:/transfer/bc.tar  # 출력 경로 지정
#
# 넣는 것(저장소 루트 기준 상대 경로 그대로):
#   data/pdm_bus/<seed>/processed/transitions.npz, processed/metadata.json  (관측 v2로 전처리한 seed만)
#   data/pdm_bus/bc_data_files.txt   학습에 쓸 npz 목록. seed 번호 순서로 고정한다(분할·resume이 파일 순서에 의존)
#   ref/rl-agents/rl_agents          utils/ego_attention.py가 쓰는 rl-agents 코드(git에 없다)
# transitions.jsonl.gz와 원시 데이터(steps.jsonl.gz 등)는 학습에 쓰지 않아 넣지 않는다.
# npz는 이미 압축되어 있어 tar는 압축하지 않는다.
set -eu

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/bc_data.tar}"
SCENARIO="${SCENARIO:-baseline}"
cd "$ROOT"

list="data/pdm_bus/bc_data_files.txt"
: > "$list.tmp"
for d in data/pdm_bus/"${SCENARIO}"_s*/; do
    d="${d%/}"
    if [ -f "$d/processed/transitions.npz" ] && grep -q obs_v2_names "$d/processed/metadata.json" 2>/dev/null; then
        echo "${d#data/pdm_bus/}/processed/transitions.npz" >> "$list.tmp"
    fi
done
sort -V "$list.tmp" -o "$list.tmp"
mv "$list.tmp" "$list"
n=$(wc -l < "$list")
[ "$n" -gt 0 ] || { echo "관측 v2로 전처리한 seed가 없습니다(scripts/preprocess_pdm_bus_batch.sh 먼저)"; exit 1; }

files="$(mktemp)"
echo "$list" > "$files"
sed 's#^#data/pdm_bus/#' "$list" >> "$files"
sed 's#^#data/pdm_bus/#; s#transitions.npz$#metadata.json#' "$list" >> "$files"
echo "ref/rl-agents/rl_agents" >> "$files"
tar --force-local --exclude='__pycache__' -cf "$OUT" -T "$files"   # --force-local: C:/... 를 원격 호스트로 읽지 않게
rm -f "$files"
echo "묶음: $OUT ($(du -h "$OUT" | cut -f1), seed ${n}개)"
echo "목록: $list"
