"""traci/libsumo 선택과 SUMO 실행 공통 설정.

libsumo가 import되지 않는 환경(현재 Windows pip wheel)에서는 traci로 대체한다.
"""
import os
import sys

import shutil


def _find_sumo_home():
    """SUMO_HOME이 잘못 설정된 PC(실제 설치 위치와 다른 드라이브)에서도 PATH의 sumo로 찾는다."""
    h = os.environ.get("SUMO_HOME", "")
    if h and os.path.isdir(os.path.join(h, "bin")):
        return h
    exe = shutil.which("sumo")
    if exe:
        return os.path.dirname(os.path.dirname(exe))
    raise RuntimeError("SUMO를 찾지 못했습니다")


os.environ["SUMO_HOME"] = _find_sumo_home()
_tools = os.path.join(os.environ["SUMO_HOME"], "tools")
if _tools not in sys.path:
    sys.path.append(_tools)

if os.environ.get("TRAFFIC_BACKEND") == "libsumo":
    import libsumo as traci  # noqa: E402  (Python 3.10 이상 환경에서만 import됨)
else:
    import traci  # noqa: E402
from sumolib import checkBinary  # noqa: E402

# 고정 설정: 0.1 s 스텝 + ballistic 적분, 충돌 옵션 일괄 반영.
COMMON_ARGS = [
    "--step-length", "0.1",
    "--step-method.ballistic",
    "--collision.action", "warn",
    "--collision.check-junctions",
    "--intermodal-collision.action", "warn",
    # 순간이동을 끈다. 막힘은 SUMO가 차를 옮겨 감추지 않고 그대로 남는다(측정에서 보인다).
    "--time-to-teleport", "-1",
    "--no-step-log", "--no-warnings", "true",
    "--duration-log.disable",
]


def sumo_cmd(cfg, seed=0, extra=None, binary="sumo"):
    # 콘솔 경고만 끈다. 비상 제동·충돌·보행자 정체 경고는 실행 폴더의 sumo_warnings.log에 남는다(감사 E3).
    log = os.path.join(os.path.dirname(os.path.abspath(cfg)), "sumo_warnings.log")
    cmd = [checkBinary(binary), "-c", cfg, "--seed", str(seed)] + COMMON_ARGS + ["--error-log", log]
    return cmd + list(extra or [])
