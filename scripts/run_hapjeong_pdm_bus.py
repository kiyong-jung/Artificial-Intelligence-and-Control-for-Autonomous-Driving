"""합정역 시나리오를 PDM-Closed 기반 버스 주행으로 실행한다. 기존 run_hapjeong_scenario.py는 고치지 않고 그대로 불러 쓴다.

    python scripts/run_hapjeong_pdm_bus.py                         # sumo-gui
    python scripts/run_hapjeong_pdm_bus.py --nogui --seconds 300   # 화면 없이 요약만

run_hapjeong_scenario.py의 모든 옵션을 그대로 받는다. 달라지는 것 두 가지:
  - 버스 차량 유형이 SUMO 기본 IDM(PDM-Closed의 추종 모델, 매개변수도 같다)이 된다.
  - 시험용 버스 조종이 PDM-Closed 조종(traffic/pdm_bus.py)이 되어, 속도 비율을 제안 점수로 고른다. 차선 변경 규칙은 그대로다.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import run_hapjeong_scenario as base  # noqa: E402
import traffic.hapjeong as hapjeong  # noqa: E402
from traffic.pdm_bus import BUS_PDM_VTYPE, PDMClosedBusPilot  # noqa: E402


class _Pilot(PDMClosedBusPilot):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        _Pilot.last = self


if __name__ == "__main__":
    hapjeong.BUS_VTYPE = BUS_PDM_VTYPE            # 월드가 버스 유형을 만들 때 읽는다
    base.TestBusPilot = _Pilot
    try:
        base.main()
    finally:
        p = getattr(_Pilot, "last", None)
        if p is not None:
            print("PDM-Closed 버스:", p.summary())
