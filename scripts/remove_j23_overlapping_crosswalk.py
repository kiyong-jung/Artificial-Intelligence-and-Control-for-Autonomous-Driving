"""독막로 J23의 겹친 횡단보도 하나(-R71 R71.256)를 없앤다(3.10.0 -> 3.11.0).

    python scripts/remove_j23_overlapping_crosswalk.py

J23에는 횡단보도가 둘 있다. c0(R73 -R73, 직각, 16 m)과 c1(-R71 R71.256, 비스듬히 19.2 m)이 3.4.0(297ea22) 도로 정리 뒤 한쪽 끝에서
V자로 겹쳤다. 두 횡단보도는 같은 보행자 단계(20초, 차량 적색)를 쓰고 같은 차량 링크 여섯 개와 충돌해 신호상 차이가 없다.
더 안정적인 쪽(직각이고 더 짧고, 지나는 보행자가 더 많은 c0)을 남기고 c1을 없앤다(사용자 결정). 나머지 처리는
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
    remove_crosswalks("J23", ("-R71 R71.256",), "3.11.0", "j23_overlapping_crosswalk", "scripts/remove_j23_overlapping_crosswalk.py")
