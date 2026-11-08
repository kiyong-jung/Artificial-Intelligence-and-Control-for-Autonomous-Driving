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

import traci  # noqa: E402
from sumolib import checkBinary  # noqa: E402

# 계획 "고정 설정": 0.1 s 스텝 + ballistic 적분, 충돌 옵션 일괄 반영.
COMMON_ARGS = [
    "--step-length", "0.1",
    "--step-method.ballistic",
    "--collision.action", "warn",
    "--collision.check-junctions",
    "--intermodal-collision.action", "warn",
    "--no-step-log", "--no-warnings", "true",
    "--duration-log.disable",
]


def sumo_cmd(cfg, seed=0, extra=None, binary="sumo"):
    cmd = [checkBinary(binary), "-c", cfg, "--seed", str(seed)] + COMMON_ARGS
    return cmd + list(extra or [])
