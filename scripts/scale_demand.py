"""배경 차량 흐름 전체에 같은 배율을 곱한다(3.19.0 -> 3.20.0).

    python scripts/scale_demand.py [배율]

3.17.0까지의 수요(baseline 1,061대/시간)로는 버스가 배경 차 때문에 행동을 바꾸는 일이 드물었다. 같은 baseline seed를
배경 차 없이 달려도 한 바퀴가 평균 0.5초만 짧았다(maps/hapjeong/README.md "상호작용이 충분한가"). 흐름마다 같은 배율을
곱하므로 진입로·진출로 비율, 골목 유입 비율, quiet·baseline·busy(0.5·1·1.25배, env/road_config.py)의 비율은 그대로다.

배율은 3.17.0 수요(baseline 1,061대/시간)에 대한 값이다. map.json에 적어 두므로 다른 배율로 다시 돌리면 3.17.0 수요에서 다시 계산하고, 같은
배율로 다시 돌리면 아무것도 바꾸지 않는다. scripts/build_hapjeong.py의 DEMAND_SCALE과 같아야 한다.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.build_hapjeong import DEMAND_SCALE  # noqa: E402

FOLDER = BASE / "maps" / "hapjeong"
NEW_VERSION = "3.20.0"


def main() -> None:
    target = float(sys.argv[1]) if len(sys.argv) > 1 else DEMAND_SCALE
    path = FOLDER / "map.json"
    definition = json.loads(path.read_text(encoding="utf-8"))
    patches = definition["network_source"]["patches"]
    current = patches.get("demand_scale", {}).get("factor", 1.0)
    if current == target:
        print(f"이미 {target}배다: 바꾸지 않는다")
        return
    before = sum(flow["vehs_per_hour"] for flow in definition["flows"])
    for flow in definition["flows"]:
        flow["vehs_per_hour"] = round(flow["vehs_per_hour"] / current * target, 3)
    after = sum(flow["vehs_per_hour"] for flow in definition["flows"])
    patches["demand_scale"] = {"factor": target, "baseline_vehs_per_hour": round(after, 1),
                               "script": "scripts/scale_demand.py"}
    definition["assumptions"] = [text for text in definition["assumptions"] if not text.startswith("배경 차량 baseline 합계")]
    definition["assumptions"].append(f"배경 차량 baseline 합계는 시간당 {1061 * target:,.0f}대다(3.17.0 수요 1,061대의 {target:g}배).")
    definition["version"] = NEW_VERSION
    path.write_text(json.dumps(definition, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"배경 흐름 {len(definition['flows'])}개 x{target / current:g}: baseline {before:.1f} -> {after:.1f}대/시간. map {NEW_VERSION}")


if __name__ == "__main__":
    main()
