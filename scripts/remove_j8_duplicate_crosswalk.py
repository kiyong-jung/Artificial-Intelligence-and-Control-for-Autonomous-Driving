"""독막로 J8의 겹친 횡단보도 하나(-R69a R69a)를 없앤다(3.9.0 -> 3.10.0).

    python scripts/remove_j8_duplicate_crosswalk.py

J8에는 횡단보도가 둘 있다. c0(R71 -R71.175)와 c1(-R69a R69a)이 3.4.0(297ea22) 도로 정리 뒤 거의 같은 자리(간격 0.1 m)로 옮겨져
겹쳤다. 원본에서는 7 m 떨어져 있었다. c1을 없앤다(사용자 결정). 나머지 처리(신호 프로그램, 보행자 경로, map.json)는
remove_cluster_crosswalks.remove_crosswalks와 같다.
"""
from __future__ import annotations

import sys
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.remove_cluster_crosswalks import remove_crosswalks  # noqa: E402

if __name__ == "__main__":
    remove_crosswalks("J8", ("-R69a R69a",), "3.10.0", "j8_duplicate_crosswalk", "scripts/remove_j8_duplicate_crosswalk.py")
