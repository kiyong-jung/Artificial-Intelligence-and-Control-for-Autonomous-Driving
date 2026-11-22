"""교통이 자연스러운지 다시 돌려 보는 검사기.

찾는 것은 두 가지다.
  (a) 시나리오를 만들려고 배경 차·보행자·신호를 억지로 움직이는 코드와 데이터
  (b) 순간이동, 충돌, 막힘, 도로 위 출현을 일으키거나 감추는 설정

정적 검사(기본): 모든 .py의 TraCI 변경 호출과 위험한 SUMO 옵션·속성 문자열, 커밋된 지도 XML, 두 실행 경로
(학습 환경 env/, 주변 교통 traffic/)가 만들어 내는 실행 폴더의 XML, 지도의 수요·도로망 모양을 훑는다. 찾은 것은
모두 아래 RULES 표의 한 줄과 맞아야 한다. ALLOW는 정당한 것(이유 한 줄), OPEN은 알고 있는 문제(보고서의 발견 번호)다.
표에 없는 것이 하나라도 나오면 실패한다(종료 코드 1). --strict는 OPEN도 실패로 센다.

동적 검사(--dynamic): 학습 환경을 전문가 정책으로, 주변 교통 경로를 scripts/measure_s1_s6.py로 돌리면서
SUMO에서 순간이동, 충돌(위치), 비상 제동·정지, 120초 넘게 선 차·사람(위치), 버스 50m 안 도로 한가운데 출현,
환경 종료 사유를 적는다. SUMO 프로세스는 동시에 최대 4개다.

    python scripts/audit_naturalness.py                       # 정적 검사
    python scripts/audit_naturalness.py --dynamic             # 정적 + 동적(env 9회, traffic 3회)
    python scripts/audit_naturalness.py --dynamic --seeds 5 --traffic-seeds 0 3 --out results/naturalness/round2
    python scripts/audit_naturalness.py --self-check          # 검사기가 심어 둔 나쁜 예를 잡는지 확인
"""
from __future__ import annotations

import argparse
import ast
import collections
import fnmatch
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

SKIP_DIRS = {".git", ".claude", "web_page", "results", "data", "__pycache__", ".pytest_cache", ".audit", ".tools"}

# ---------------------------------------------------------------------------------------------------------------
# 무엇을 위험하다고 보는가
# ---------------------------------------------------------------------------------------------------------------
TRACI_DOMAINS = ("vehicle", "person", "trafficlight", "lane", "edge", "route", "vehicletype", "simulation",
                 "busstop", "parkingarea", "calibrator", "rerouter", "variablespeedsign", "junction")
READ_PREFIXES = ("get", "is", "subscribe", "unsubscribe", "find", "convert", "could", "wantsAnd")
# 값이 이 조건이면 위험. 조건이 None이면 적혀 있기만 해도 위험(기본값에서 벗어남).


def _num(test):
    def check(value):
        try:
            return test(float(value))
        except ValueError:
            return True                      # 숫자가 아닌 값(식, 분포)은 사람이 본다
    return check


RISK_OPTIONS = {
    "collision.action": lambda v: v != "warn",                 # remove/teleport/none은 충돌한 차를 지우거나 감춘다
    "intermodal-collision.action": lambda v: v != "warn",
    "collision.check-junctions": lambda v: v in ("false", "False", "0"),
    "collision.mingap-factor": _num(lambda x: x != -1),
    "collision.stoptime": None,
    "time-to-teleport": _num(lambda x: x >= 0),
    "time-to-teleport.highways": None, "time-to-teleport.disconnected": None, "time-to-teleport.remove": None,
    "time-to-teleport.ride": None, "time-to-teleport.bidi": None, "time-to-teleport.railsignal-deadlock": None,
    "ignore-route-errors": None, "ignore-junction-blocker": None, "max-depart-delay": None,
    "emergency-insert": None, "eager-insert": None, "tls.all-off": None, "no-internal-links": None,
    "default.emergencydecel": None, "emergencydecel.warning-threshold": None, "scale": None,
    "no-warnings": lambda v: v in ("true", "True", "1", ""),   # 비상 제동·충돌 경고가 화면에서 사라진다
    "step-method.ballistic": None,                             # 두 실행 경로의 적분 방식이 달라진다
    "pedestrian.striping.jamtime": None, "pedestrian.striping.jamtime.crossing": None,
    "random-depart-offset": None,
}
RISK_ATTRS = {
    "jmIgnoreFoeProb": _num(lambda x: x > 0), "jmIgnoreJunctionFoeProb": _num(lambda x: x > 0),
    "jmDriveAfterRedTime": _num(lambda x: x >= 0), "jmDriveAfterYellowTime": _num(lambda x: x >= 0),
    "jmDriveRedSpeed": None, "jmStoplineGap": None, "jmStoplineCrossingGap": None, "jmAdvance": None,
    "jmExtraGap": None, "jmSigmaMinor": None, "jmStopSignWait": None, "jmAllwayStopWait": None,
    "impatience": _num(lambda x: x > 0), "jmTimegapMinor": _num(lambda x: x < 1.0),
    "jmCrossingGap": _num(lambda x: x < 10.0),
    "sigma": _num(lambda x: x == 0 or x > 0.5),               # 0은 오차 없는 운전자, 큰 값은 일부러 흔드는 운전자
    "speedFactor": _num(lambda x: not 0.8 <= x <= 1.2), "minGap": _num(lambda x: x < 1.0),
    "tau": _num(lambda x: x < 0.5), "collisionMinGapFactor": None, "insertionChecks": None,
    "lcCooperative": _num(lambda x: x < 0), "lcStrategic": _num(lambda x: x < 0), "lcPushy": _num(lambda x: x > 0),
    "lcTimeToImpatience": None,
    "departPos": lambda v: v not in ("base", "0", "0.0", "0.00"),   # 도로 시작점이 아닌 곳에서 생긴다
    "departLane": lambda v: v.lstrip("-").isdigit(),                # 차로를 못 박았다
    "arrivalPos": None, "departPosLat": None,
    "onRoad": lambda v: v in ("true", "1"),                         # 차로를 막는 정차 구역
    "uncontrolled": lambda v: v in ("true", "1"), "pass": lambda v: v in ("true", "1"),
    "keepClear": lambda v: v in ("false", "0"),
}
# .py의 문자열 상수에 이 이름이 있으면 그 코드가 위험한 속성을 XML이나 TraCI 파라미터로 쓴다.
RISK_TOKENS = ("jmIgnoreFoeProb", "jmIgnoreJunctionFoeProb", "jmDriveAfterRedTime", "jmDriveAfterYellowTime",
               "jmDriveRedSpeed", "ignoreIDs", "ignoreTypes", "insertionChecks", "impatience", "jmTimegapMinor",
               "jmCrossingGap", "onRoad")
EDGE_START_M = 20.0        # 출발 위치가 이보다 뒤면 '도로 한가운데'로 본다(버스 12m + 여유)
NEAR_EGO_M = 50.0
STUCK_S = 120.0
LONGEST_CYCLE_S = 182.0    # 가장 긴 신호 주기. 이보다 오래 서 있으면 신호 대기가 아니다

# ---------------------------------------------------------------------------------------------------------------
# 허용표. (종류, 파일, 어디서, 무엇, 대상, 판정, 이유). 위에서부터 처음 맞는 줄을 쓴다. fnmatch 패턴이다.
#   call: 파일=.py, 어디서=함수, 무엇=domain.method, 대상=첫 인자
#   lit : 파일=.py, 어디서=함수, 무엇=옵션·속성 이름, 대상=값
#   xml : 파일=XML 이름(gen:env/…, gen:traffic/…는 만들어 본 실행 폴더), 어디서=태그, 무엇=속성, 대상=id(숫자는 #)
#   data: 파일=지도 id, 어디서=검사 이름, 무엇=항목
# 판정 ALLOW = 정당하다. OPEN:<번호> = research/av_bc_rl/naturalness_audit_round1.md의 발견. 고치면 이 줄을 지운다.
# ---------------------------------------------------------------------------------------------------------------
A, O = "ALLOW", "OPEN:"
RULES = [
    # ---- 학습 환경: ego만 조종한다 ----
    ("call", "env/sumo_env.py", "*", "vehicle.*", "self.ego_id", A, "ego 버스 자신의 투입·속도·차선 명령"),
    ("lit", "env/sumo_env.py", "*._start_sumo", "no-warnings", "true", O + "E3",
     "SUMO 경고(비상 제동, 충돌)를 꺼서 학습·평가 기록에 남지 않는다"),
    ("lit", "env/road_builder.py", "build", "collision.stoptime", "*", A,
     "부딪힌 두 차가 그 자리에 road_config의 collision_stop_s(120초) 동안 섰다가 비켜 간다. 지우지 않는다"),
    ("lit", "research/av_bc_rl/s6_priority_trial.py", "background_run", "no-warnings", "true", A,
     "S6 제안용 배경 측정 실행. 학습·평가에 쓰지 않고, 수치는 TraCI와 statistics로 센다"),
    ("lit", "env/road_builder.py", "build", "collision.mingap-factor", "*", A,
     "값은 road_config의 -1(SUMO 모델별 기본값)이다"),
    ("lit", "env/road_builder.py", "build", "time-to-teleport", "*", A, "-1: 순간이동 끔"),
    ("xml", "gen:*/scenario.sumocfg", "collision.stoptime", "value", "*", A,
     "위와 같다(만들어진 설정). traffic 경로도 같은 설정 파일을 읽는다"),
    # ---- 주변 교통 경로: 시험용 버스 ----
    ("call", "traffic/test_bus.py", "*", "vehicle.*", "BUS", A, "시험용 버스 자신만 조종한다(차로·정차·방향지시등)"),
    ("call", "traffic/test_bus.py", "*", "route.add", "'bus_lap'", A, "시험용 버스의 바퀴 경로 등록"),
    # ---- 주변 교통 경로: 배경 교통에 닿는 것 ----
    ("call", "traffic/yield_logic.py", "YieldManager.on_departed", "vehicle.setParameter", "vid", A,
     "투입 때 한 번, 버스와 무관한 분포에서 뽑은 운전자 성향(차선 변경 파라미터)"),
    ("call", "traffic/yield_logic.py", "YieldManager.on_departed", "vehicle.setSpeedFactor", "vid", A,
     "투입 때 한 번, 운전자마다 다른 희망 속도(0.8~1.2)"),
    ("call", "traffic/lane_route.py", "LaneRouteAdapter.update", "vehicle.setRoute", "vid", A,
     "정지선 맨 앞에 200초(가장 긴 신호 주기보다 길게) 선 차만, 회차 없이 지금 차로가 가는 길로. 운전자의 포기 행동"),
    ("call", "traffic/lane_route.py", "LaneRouteAdapter._stuck", "vehicle.<alias>", "v", A,
     "읽기 전용 별칭(getSpeed, getWaitingTime, getLanePosition)"),
    ("call", "traffic/interior_spawner.py", "*", "vehicle.add", "vid", O + "T2",
     "노선 위 임의 지점(도로 한가운데)에서 차를 만든다. 기본은 꺼져 있다(interior_share=0)"),
    ("call", "traffic/interior_spawner.py", "*", "route.add", "rid", O + "T2", "위 차의 경로 등록"),
    ("call", "traffic/right_turn.py", "apply_right_on_red", "trafficlight.setProgramLogic", "tid", O + "T3",
     "실행 중 신호 프로그램을 바꿔 적색 우회전을 양보 녹색(g)으로 연다"),
    ("call", "traffic/ped_defiance.py", "*", "person.setParameter", "pid", O + "T4",
     "보행자가 모든 차의 우선권을 무시하게 한다(junctionModel.ignoreTypes). 기본은 꺼져 있다"),
    ("lit", "traffic/ped_defiance.py", "*", "ignoreTypes", "*", O + "T4", "위와 같다"),
    ("lit", "traffic/cutin.py", "cutin_vtype_xml", "jmIgnoreFoeProb", "*", O + "T5",
     "default_bins를 쓰면 골목 차가 큰길 차를 0.2~1.0 확률로 무시한다(합정역 실행은 0)"),
    ("lit", "traffic/cutin.py", "cutin_vtype_xml", "impatience", "*", O + "T6", "골목 운전자 impatience 0~0.5"),
    ("lit", "traffic/cutin.py", "cutin_vtype_xml", "jmTimegapMinor", "*", O + "T6", "골목 운전자 최소 시간 간격 1.0~0.3초"),
    ("lit", "traffic/cutin.py", "parking_area_xml", "onRoad", "*", O + "T7", "차로를 막는 택시 정차 구역"),
    ("lit", "traffic/pedestrian.py", "ped_dist_xml", "*", "*", O + "T8",
     "보행자 분포: impatience 0~1, jmTimegapMinor. jmIgnoreFoeProb는 0으로 적는다"),
    ("lit", "traffic/pedestrian.py", "*", "*", "*", A, "합성 T자로 검증 장면의 보행자 유형(합정역 실행과 무관)"),
    ("lit", "traffic/sumo_conn.py", "", "no-warnings", "true", O + "E3", "SUMO 경고를 끈다"),
    ("lit", "traffic/sumo_conn.py", "", "step-method.ballistic", "*", O + "X1",
     "traffic 경로만 ballistic 적분이다. 학습 환경은 SUMO 기본(Euler)이라 두 경로의 물리가 다르다"),
    ("xml", "gen:traffic/traffic.rou.xml", "vType", "impatience", "cutin_#", O + "T6", "골목 운전자 6구간 impatience 0.1~0.5"),
    ("xml", "gen:traffic/traffic.rou.xml", "vType", "jmTimegapMinor", "cutin_#", O + "T6", "골목 운전자 0.3~0.86초"),
    ("xml", "gen:traffic/traffic.rou.xml", "vType", "impatience", "ped_*", O + "T8", "보행자 P1·P2 구간 impatience"),
    ("xml", "gen:traffic/traffic.rou.xml", "vType", "sigma", "bus_test", A, "시험용 버스는 규칙으로 몬다"),
    ("xml", "gen:traffic/traffic.add.xml", "parkingArea", "onRoad", "pa_curb_*", O + "T7",
     "정류장 6곳 앞 가로변 차로를 막는 택시 정차 구역(시간당 12대씩)"),
    # ---- 합성 도로 검증 장면과 시연: 각본대로 움직이는 것이 목적이다. 합정역 학습·측정에는 들어가지 않는다 ----
    ("call", "traffic/scenarios.py", "*", "*", "*", A, "합성 2차로 직선 검증 장면(각본 버스와 후행차 한 대)"),
    ("call", "traffic/scenarios_a2.py", "*", "*", "*", A, "합성 합류 교차로 검증 장면"),
    ("call", "traffic/scenarios_s1.py", "*", "*", "*", A, "합성 정류장 검증 장면"),
    ("call", "traffic/pedestrian.py", "run_*", "*", "*", A, "합성 횡단보도 T자로 검증 장면"),
    ("lit", "traffic/scenarios*.py", "*", "*", "*", A, "합성 도로 검증 장면의 vType"),
    ("call", "tests/*.py", "*", "*", "*", A, "pytest 장면(합성 도로 또는 각본 차량으로 단위 동작을 확인)"),
    ("lit", "tests/*.py", "*", "*", "*", A, "pytest 장면"),
    ("call", "scripts/verify_*.py", "*", "*", "*", A, "검증 스크립트의 각본 장면(결과는 학습·측정에 쓰이지 않는다)"),
    ("lit", "scripts/verify_*.py", "*", "*", "*", A, "검증 스크립트"),
    ("call", "scripts/view_scenarios.py", "*", "*", "*", O + "S1",
     "합정역 지도 위에 버스와 상대 차를 각본 위치·속도로 올려 S1~S6을 보여 주는 시연(적색 무시 speedMode 15, 후행차 차로 고정)"),
    ("lit", "scripts/audit_naturalness.py", "*", "*", "*", A, "이 검사기 자신의 표"),
    ("call", "traffic/presence.py", "PresenceMonitor.update", "simulation.<alias>", "sim", A,
     "측정용 읽기(출발·도착·순간이동 목록)만 한다"),
    ("xml", "gen:traffic/traffic.rou.xml", "vType", "minGap", "ped_#", A, "보행자 유형(SUMO 보행자 기본 0.25m와 비슷)"),
    ("lit", "scripts/audit_road.py", "run", "time-to-teleport", "300", O + "V1",
     "도로 점검이 순간이동 300초로 돈다. 학습 환경은 순간이동을 껐으므로 점검이 통과해도 막힘이 남을 수 있다"),
    ("lit", "scripts/validate_road.py", "run_background", "time-to-teleport", "300", O + "V1", "위와 같다"),
    ("lit", "scripts/remove_crosswalk_pedestrians.py", "pedestrians_on", "*", "*", A,
     "지도 패치 도구가 보행자만 넣고 누가 그 횡단보도를 밟는지 읽는다(실행에 쓰는 설정이 아니다)"),
    ("lit", "scripts/remove_cluster_crosswalks.py", "unroutable_persons", "ignore-route-errors", "true", A,
     "지도 패치 도구가 경로가 끊긴 보행자를 찾아내려고 오류를 경고로 받는다"),
    ("lit", "scripts/road_usage.py", "run", "no-warnings", "true", A, "도로 사용량 측정(읽기만)"),
    ("lit", "scripts/scenario_supply.py", "run", "no-warnings", "true", A, "시나리오 공급 측정(읽기만)"),
    ("xml", "maps/paldalmun/source/network.net.xml", "connection", "uncontrolled", "*", O + "P1",
     "팔달문 지도(수업 기본 지도)의 신호 없는 연결. 이번 감사는 합정역만 봤다"),
    ("call", "scripts/audit_naturalness.py", "*", "*", "*", A, "이 검사기(읽기와 자기 점검)"),
    ("lit", "maps/hapjeong/generator/*.py", "*", "*", "*", A,
     "서예인 원본 생성기. 실행에 쓰는 도로망은 scripts/build_hapjeong.py가 만든다"),
    # ---- 커밋된 지도 ----
    ("xml", "maps/hapjeong/source/network.net.xml", "connection", "uncontrolled", "G22>R94", O + "M3",
     "신호 횡단보도 J48에서 골목 G22 우회전만 신호 없이 나온다"),
    ("xml", "maps/paldalmun/source/network.full.net.xml", "*", "*", "*", A, "자르기 전 원본(실행에 쓰지 않는다)"),
    ("data", "hapjeong", "crossing-vehicle-priority", "S6_wausanro", "*", O + "M1",
     "신호 없는 횡단보도에서 차가 우선(보행자가 양보). 한국 법규와 반대"),
    ("data", "hapjeong", "persons-fixed-schedule", "*", "*", O + "M2",
     "보행자 출발 시각이 파일에 박혀 있어 seed가 달라도 같은 사람이 같은 시각에 나온다"),
    ("data", "hapjeong", "persons-single-crossing", "s6ped", "*", O + "M2",
     "S6 횡단보도만 건너는 보행자를 15초마다 한 명씩 따로 넣는다"),
    ("data", "hapjeong", "service-stop-at-edge-end", "*", "*", O + "M4",
     "정차할 정류장이 도로 끝·시작에 붙어 있다(s6 놓침 버그의 조건)"),
    ("data", "hapjeong", "short-split-edge", "*", "*", O + "M5",
     "40m보다 짧은데 차로마다 가는 길이 달라, 틀린 차로로 들어온 차가 옮길 자리가 없다"),
    ("data", "hapjeong", "flow-origin-interior", "*", "*", O + "M6", "지도 안쪽 도로에서 차가 생긴다"),
    ("data", "hapjeong", "demand-multiplier", "*", "*", O + "M8",
     "지도 빌드가 수요를 손으로 키웠다(골목 3배, S6 보행자 4배, 상수역 출구 2배)"),
]


def judge(hit):
    for kind, file, where, what, target, verdict, reason in RULES:
        if (kind == hit["kind"] and fnmatch.fnmatchcase(hit["file"], file) and fnmatch.fnmatchcase(hit["where"], where)
                and fnmatch.fnmatchcase(hit["what"], what)
                and any(fnmatch.fnmatchcase(hit["target"], t) for t in target.split("|"))):
            return verdict, reason, (kind, file, where, what, target)
    return "UNKNOWN", "허용표에 없다", None


# ---------------------------------------------------------------------------------------------------------------
# 1) .py: TraCI 변경 호출과 위험한 문자열
# ---------------------------------------------------------------------------------------------------------------
def _traci_mutators():
    """설치된 traci에서 도메인마다 '읽기가 아닌' 메서드 이름을 읽는다. 이름만 같은 다른 객체 호출을 거른다."""
    import traci
    out = {}
    for domain in TRACI_DOMAINS:
        obj = getattr(traci, domain, None)
        if obj is not None:
            out[domain] = {name for name in dir(obj) if not name.startswith("_") and callable(getattr(obj, name))
                           and not name.startswith(READ_PREFIXES)}
    return out


class _Scan(ast.NodeVisitor):
    def __init__(self, rel, mutators):
        self.rel, self.mutators, self.stack, self.hits, self.docstrings = rel, mutators, [], [], set()

    def _where(self):
        return ".".join(self.stack)

    def _hit(self, kind, node, what, target, detail=""):
        self.hits.append(dict(kind=kind, file=self.rel, where=self._where(), what=what, target=target,
                              line=node.lineno, detail=detail))

    def _scoped(self, node):
        doc = node.body[0] if node.body else None
        if isinstance(doc, ast.Expr) and isinstance(doc.value, ast.Constant) and isinstance(doc.value.value, str):
            self.docstrings.add(id(doc.value))
        if not isinstance(node, ast.Module):
            self.stack.append(node.name)
        self.generic_visit(node)
        if not isinstance(node, ast.Module):
            self.stack.pop()

    visit_Module = visit_FunctionDef = visit_AsyncFunctionDef = visit_ClassDef = _scoped

    def visit_Call(self, node):
        f = node.func
        if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Attribute) and f.value.attr in self.mutators:
            if f.attr in self.mutators[f.value.attr]:
                detail = ", ".join(f"{k.arg}={ast.unparse(k.value)}" for k in node.keywords if k.arg)
                self._hit("call", node, f"{f.value.attr}.{f.attr}",
                          ast.unparse(node.args[0]) if node.args else "", detail)
        # getattr(tc.vehicle, name)(...) 같은 돌려 부르기
        if isinstance(f, ast.Name) and f.id in ("getattr", "setattr") and node.args:
            base = node.args[0]
            if isinstance(base, ast.Attribute) and base.attr in self.mutators:
                self._hit("call", node, f"{base.attr}.<{f.id}>", ast.unparse(node.args[1]) if len(node.args) > 1 else "")
        self.generic_visit(node)

    def visit_Assign(self, node):
        # v = tc.vehicle 처럼 도메인에 별명을 붙이면 위 규칙을 빠져나간다
        if isinstance(node.value, ast.Attribute) and node.value.attr in self.mutators and isinstance(
                node.value.value, (ast.Name, ast.Attribute)) and ast.unparse(node.value.value).split(".")[-1] in (
                "traci", "tc", "libsumo", "conn"):
            self._hit("call", node, f"{node.value.attr}.<alias>", ast.unparse(node.targets[0]))
        self.generic_visit(node)

    def _sequence(self, node):
        # ["--collision.action", "warn"] 와 ("collision.action", "remove") 꼴의 옵션
        for i, elt in enumerate(node.elts):
            if isinstance(elt, ast.Constant) and isinstance(elt.value, str) and elt.value.lstrip("-") in RISK_OPTIONS:
                name = elt.value.lstrip("-")
                nxt = node.elts[i + 1] if i + 1 < len(node.elts) else None
                if isinstance(nxt, ast.Constant) and not str(nxt.value).startswith("--"):
                    value = str(nxt.value).lower() if isinstance(nxt.value, bool) else str(nxt.value)
                elif nxt is None or isinstance(nxt, ast.Constant):
                    value = "true"
                else:
                    value = "<expr>"
                check = RISK_OPTIONS[name]
                if check is None or value == "<expr>" or check(value):
                    self._hit("lit", elt, name, value)
        self.generic_visit(node)

    visit_List = visit_Tuple = _sequence

    def visit_Constant(self, node):
        if isinstance(node.value, str) and id(node) not in self.docstrings:
            for token in RISK_TOKENS:
                if re.search(rf"(?<![A-Za-z]){token}(?![A-Za-z])", node.value):
                    self._hit("lit", node, token, "")


def scan_python(root=REPO, files=None):
    mutators = _traci_mutators()
    hits = []
    paths = files or sorted(p for p in root.rglob("*.py") if not SKIP_DIRS & set(p.relative_to(root).parts))
    for path in paths:
        rel = path.relative_to(root).as_posix() if path.is_absolute() and root in path.parents else path.name
        scan = _Scan(rel, mutators)
        scan.visit(ast.parse(path.read_text(encoding="utf-8"), filename=str(path)))
        seen = set()
        for hit in scan.hits:      # 같은 함수 안 같은 호출은 한 줄로(행 번호는 모은다)
            key = (hit["kind"], hit["where"], hit["what"], hit["target"])
            if key in seen:
                next(h for h in hits if h["file"] == rel and (h["kind"], h["where"], h["what"], h["target"]) == key)[
                    "lines"].append(hit["line"])
                continue
            seen.add(key)
            hit["lines"] = [hit.pop("line")]
            hits.append(hit)
    return hits


# ---------------------------------------------------------------------------------------------------------------
# 2) XML: 커밋된 지도 파일과 두 경로가 만드는 실행 폴더
# ---------------------------------------------------------------------------------------------------------------
def _id_pattern(elem):
    if elem.tag == "connection":
        return f"{elem.get('from')}>{elem.get('to')}"
    return re.sub(r"\d+", "#", elem.get("id") or elem.get("parkingArea") or elem.get("busStop") or "")


def scan_xml(path, label):
    hits = collections.OrderedDict()
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        return []
    if root.tag == "osm":
        return []
    option_file = root.tag == "configuration"
    for elem in root.iter():
        pairs = [(elem.tag, elem.get("value", ""))] if option_file else list(elem.attrib.items())
        for name, value in pairs:
            check = (RISK_OPTIONS if option_file else RISK_ATTRS).get(name, False)
            if check is False or (check is not None and not check(value)):
                continue
            if elem.tag in ("lane", "edge", "junction", "net", "location", "poly", "request"):
                continue
            where, what = (name, "value") if option_file else (elem.tag, name)
            target = "" if option_file else _id_pattern(elem)
            hit = hits.setdefault((where, what, target), dict(kind="xml", file=label, where=where, what=what,
                                                                target=target, values=collections.Counter()))
            hit["values"][value] += 1
    out = []
    for hit in hits.values():
        hit["detail"] = ", ".join(f"{v}×{n}" for v, n in list(hit.pop("values").items())[:8])
        out.append(hit)
    return out


def build_run_folders(tmp):
    """두 실행 경로가 실제로 SUMO에 넘기는 파일을 만들어 본다. 반환: [(이름표 접두, 폴더)]"""
    from env import road_builder
    from env.road_config import ROAD, EGO, TRAFFIC, configure_map
    from traffic.hapjeong import HapjeongTraffic
    folders = []
    for map_id in ("hapjeong", "paldalmun"):
        configure_map("busy", 0, None, map_id)
        out = Path(tmp) / f"env_{map_id}"
        road_builder.build(ROAD, EGO, TRAFFIC, str(out))
        folders.append((f"gen:env/{map_id}", out))
    out = Path(tmp) / "traffic"
    HapjeongTraffic(str(out), scenario="busy", seed=0, bus_traffic=True).build()     # measure_s1_s6.py와 같은 인자
    folders.append(("gen:traffic", out))
    return folders


def scan_all_xml(tmp):
    hits = []
    for path in sorted(REPO.glob("maps/*/source/*.xml")):
        hits += scan_xml(path, path.relative_to(REPO).as_posix())
    for prefix, folder in build_run_folders(tmp):
        for path in sorted(folder.iterdir()):
            if path.suffix in (".xml", ".sumocfg") and path.name != "network.net.xml":
                hits += scan_xml(path, f"{prefix}/{path.name}")
    return hits


# ---------------------------------------------------------------------------------------------------------------
# 3) 지도의 수요와 도로망 모양
# ---------------------------------------------------------------------------------------------------------------
def scan_map(map_id="hapjeong"):
    from sumolib.net import readNet
    from maps import load_map
    m = load_map(map_id)
    net = readNet(m["net_path"], withInternal=True, withPedestrianConnections=True)
    hits = []

    def add(where, what, detail):
        hits.append(dict(kind="data", file=map_id, where=where, what=what, target="", detail=detail))

    def car_edges(edges):
        return [e for e in edges if e.allows("passenger") and e.getFunction() in ("", "normal")]

    # 차가 지도 안쪽에서 생기거나 사라지는 흐름
    for key, where in (("from", "flow-origin-interior"), ("to", "flow-destination-interior")):
        volume = collections.Counter()
        for flow in m["flows"]:
            volume[flow[key]] += float(flow["vehs_per_hour"])
        for edge_id, per_hour in sorted(volume.items()):
            edge = net.getEdge(edge_id)
            beyond = car_edges(edge.getIncoming() if key == "from" else edge.getOutgoing())
            beyond = [e for e in beyond if e.getID().lstrip("-") != edge_id.lstrip("-")]
            if beyond:
                on_route = edge_id in m["routes"][m["ego_route"]]
                add(where, edge_id, f"{per_hour:.0f}대/시간(baseline), 이어진 도로 {len(beyond)}개"
                    + (", 버스 경로 위" if on_route else ""))
    # 다음 도로로 이어지지 않는 차로, 짧은데 차로마다 가는 길이 다른 도로
    for edge in car_edges(net.getEdges()):
        lanes = [lane for lane in edge.getLanes() if lane.allows("passenger")]
        outs = [frozenset(c.getTo().getID() for c in lane.getOutgoing()) for lane in lanes]
        if [e for e in car_edges(edge.getOutgoing()) if e.getID().lstrip("-") != edge.getID().lstrip("-")]:
            for lane, out in zip(lanes, outs):
                if not out:
                    add("dead-end-lane", lane.getID(), f"길이 {lane.getLength():.0f}m")
        if len(lanes) >= 2 and edge.getLength() < 40.0 and len(set(outs)) > 1:
            add("short-split-edge", edge.getID(), f"길이 {edge.getLength():.0f}m, 차로별 다음 도로 "
                + " / ".join(",".join(sorted(o)) or "없음" for o in outs))
    # 신호 없는 횡단보도에서 보행자가 양보하는 곳
    root = ET.parse(m["net_path"]).getroot()
    for conn in root.iter("connection"):
        if "_c" in (conn.get("to") or "") and conn.get("from", "").startswith(":") and conn.get("tl") is None \
                and conn.get("state") == "m":
            add("crossing-vehicle-priority", conn.get("to").lstrip(":").rsplit("_c", 1)[0], "보행자 링크 state=m")
    # 보행자: 파일에 박힌 출발 시각, 횡단보도 하나만 건너는 사람들
    route = set(m["routes"][m["ego_route"]])
    for path in m.get("extra_route_paths", []):
        persons = list(ET.parse(path).getroot().iter("person"))
        groups = collections.defaultdict(list)
        for person in persons:
            groups[re.sub(r"\d+$", "", person.get("id"))].append(person)
        for prefix, people in sorted(groups.items()):
            departs = sorted(float(p.get("depart")) for p in people)
            gaps = collections.Counter(round(b - a, 2) for a, b in zip(departs, departs[1:]))
            add("persons-fixed-schedule", prefix, f"{len(people)}명, 출발 간격 {gaps.most_common(1)[0][0]}초가 "
                f"{gaps.most_common(1)[0][1]}번, 마지막 출발 {departs[-1]:.0f}초")
            walks = collections.Counter(p.find("walk").get("edges") for p in people if p.find("walk") is not None)
            if len(walks) <= 2 and all(len(w.split()) <= 2 for w in walks):
                add("persons-single-crossing", prefix, f"{len(people)}명이 {sorted(walks)}만 걷는다"
                    + (" (버스 경로 위)" if any(e in route for w in walks for e in w.split()) else ""))
    # 정차할 정류장이 도로 끝이나 시작에 붙어 있는가(정류장을 넘으면 곧바로 교차로 안)
    from env.road_config import _SERVICE
    for stop in m.get("bus_stops", []):
        if stop["id"] not in _SERVICE.get(map_id, {}).get("stops", ()):
            continue
        length = net.getLane(stop["lane"]).getLength()
        if length - float(stop["endPos"]) < 5.0 or float(stop["startPos"]) < 5.0:
            add("service-stop-at-edge-end", stop["name"], f"{stop['lane']} {stop['startPos']}~{stop['endPos']}m, "
                f"차로 길이 {length:.1f}m")
    # 지도 빌드가 손으로 키운 수요
    build = REPO / "scripts" / f"build_{map_id}.py"
    if build.exists():
        text = build.read_text(encoding="utf-8")
        for name in re.findall(r'"(\w+_x\d+)"\s*:', text):
            add("demand-multiplier", name, f"scripts/build_{map_id}.py SHORTCUTS")
    return hits


# ---------------------------------------------------------------------------------------------------------------
# 정적 검사 묶음
# ---------------------------------------------------------------------------------------------------------------
def run_static():
    with tempfile.TemporaryDirectory(prefix="naturalness_static_") as tmp:
        hits = scan_python() + scan_all_xml(tmp) + scan_map("hapjeong")
    used = set()
    for hit in hits:
        hit["verdict"], hit["reason"], rule = judge(hit)
        used.add(rule)
    stale = [rule for rule in (r[:5] for r in RULES) if rule not in used]
    return hits, stale


def print_static(hits, stale, verbose):
    by = collections.Counter(h["verdict"].split(":")[0] for h in hits)
    for hit in hits:
        verdict = hit["verdict"]
        if verdict == "ALLOW" and not verbose:
            continue
        lines = ":" + ",".join(map(str, hit["lines"][:6])) if hit.get("lines") else ""
        print(f"{verdict:<9} {hit['kind']:<4} {hit['file']}{lines} [{hit['where']}] {hit['what']}"
              f"({hit['target']}) {hit.get('detail', '')}\n          -> {hit['reason']}")
    for rule in stale:
        print(f"STALE     허용표의 이 줄은 맞는 것이 없다(고쳤으면 지운다): {rule}")
    print(f"\n정적 검사: 전체 {len(hits)}, ALLOW {by['ALLOW']}, OPEN {by['OPEN']}, UNKNOWN {by['UNKNOWN']}, "
          f"안 쓰인 허용 줄 {len(stale)}")


# ---------------------------------------------------------------------------------------------------------------
# 자기 점검: 나쁜 예를 심어 검사기가 잡는지 본다
# ---------------------------------------------------------------------------------------------------------------
BAD_PY = '''
def nudge(tc, bus, other):
    tc.vehicle.setSpeed(other, 0.0)
    tc.vehicle.setLaneChangeMode(other, 0)
    tc.vehicle.moveToXY(other, "", 0, 1.0, 2.0)
    tc.person.add("p", "e", 0.0)
    v = tc.vehicle
    getattr(tc.vehicle, "remove")(other)
    cmd = ["sumo", "--time-to-teleport", "300", "--ignore-route-errors", "--collision.action", "remove"]
    return '<vType id="x" jmIgnoreFoeProb="1" jmDriveAfterRedTime="5"/>'
'''
BAD_XML = ('<routes><vType id="rude" jmIgnoreFoeProb="0.5" jmDriveAfterRedTime="3" impatience="1" sigma="0"/>'
           '<vehicle id="blocker" depart="10" departPos="120" departLane="1"/></routes>')
BAD_CFG = ('<configuration><processing><time-to-teleport value="300"/><collision.action value="remove"/>'
           '<ignore-route-errors value="true"/></processing></configuration>')


def self_check():
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        (tmp / "bad.py").write_text(BAD_PY)
        (tmp / "bad.rou.xml").write_text(BAD_XML)
        (tmp / "bad.sumocfg").write_text(BAD_CFG)
        hits = scan_python(tmp, [tmp / "bad.py"]) + scan_xml(tmp / "bad.rou.xml", "bad.rou.xml") \
            + scan_xml(tmp / "bad.sumocfg", "bad.sumocfg")
    found = {(h["kind"], h["what"]) for h in hits}
    expected = {("call", "vehicle.setSpeed"), ("call", "vehicle.setLaneChangeMode"), ("call", "vehicle.moveToXY"),
                ("call", "person.add"), ("call", "vehicle.<alias>"), ("call", "vehicle.<getattr>"),
                ("lit", "time-to-teleport"), ("lit", "ignore-route-errors"), ("lit", "collision.action"),
                ("lit", "jmIgnoreFoeProb"), ("lit", "jmDriveAfterRedTime"),
                ("xml", "jmIgnoreFoeProb"), ("xml", "jmDriveAfterRedTime"), ("xml", "impatience"), ("xml", "sigma"),
                ("xml", "departPos"), ("xml", "departLane"), ("xml", "value")}
    missing = expected - found
    unknown = [h for h in hits if judge(h)[0] != "UNKNOWN"]
    return missing, unknown, hits


# ---------------------------------------------------------------------------------------------------------------
# 동적 검사: SUMO에 붙어 한 계산마다 사건을 적는다
# ---------------------------------------------------------------------------------------------------------------
class SumoProbe:
    """traci.start/load/simulationStep/close를 감싸 SUMO 사건을 모은다. 시뮬레이션에는 명령을 보내지 않는다."""

    def __init__(self, out_dir, ego_id):
        import traci
        self.tc, self.out, self.ego = traci, Path(out_dir), ego_id
        self.out.mkdir(parents=True, exist_ok=True)
        self.net = None
        self.data = dict(collisions=[], teleports=[], emergency_stops=[], near_departures=[], near_person_departures=0,
                         departed=0, stuck={}, stuck_persons={}, sim_end=None, ego_first_seen=None)
        self._orig = {name: getattr(traci, name) for name in ("start", "load", "simulationStep")}
        traci.start = lambda cmd, *a, **k: self._orig["start"](self._rewrite(cmd), *a, **k)
        traci.load = lambda args, *a, **k: self._orig["load"](self._rewrite(args), *a, **k)
        traci.simulationStep = self._step

    def _rewrite(self, cmd):
        cmd = list(cmd)
        if "--no-warnings" in cmd:
            i = cmd.index("--no-warnings")
            has_value = i + 1 < len(cmd) and not cmd[i + 1].startswith("--")
            del cmd[i:i + (2 if has_value else 1)]
        cfg = cmd[cmd.index("-c") + 1]
        if self.net is None:
            from sumolib.net import readNet
            self.net = readNet(str(Path(cfg).parent / "network.net.xml"))
        return cmd + ["--no-warnings", "false", "--log", str(self.out / "sumo.log"),
                      "--collision-output", str(self.out / "collisions.xml"),
                      "--statistic-output", str(self.out / "statistics.xml")]

    def _where(self, vid):
        v = self.tc.vehicle
        return dict(id=vid, type=v.getTypeID(vid), lane=v.getLaneID(vid), pos=round(v.getLanePosition(vid), 1))

    def _step(self, *a, **k):
        result = self._orig["simulationStep"](*a, **k)
        tc, sim, d = self.tc, self.tc.simulation, self.data
        t = sim.getTime()
        d["sim_end"] = t
        for c in sim.getCollisions():
            d["collisions"].append(dict(t=t, collider=c.collider, victim=c.victim, collider_type=c.colliderType,
                                        victim_type=c.victimType, lane=c.lane, pos=round(c.pos, 1), kind=c.type,
                                        collider_speed=round(c.colliderSpeed, 2), victim_speed=round(c.victimSpeed, 2)))
        for vid in sim.getStartingTeleportIDList():
            d["teleports"].append(dict(t=t, id=vid))
        for vid in sim.getEmergencyStoppingVehiclesIDList():
            d["emergency_stops"].append(dict(t=t, **self._where(vid)))
        departed = sim.getDepartedIDList()
        d["departed"] += len(departed)
        ego_here = self.ego in tc.vehicle.getIDList() if (departed or sim.getDepartedPersonIDList()) else False
        if ego_here:
            if d["ego_first_seen"] is None:
                d["ego_first_seen"] = t
            ex, ey = tc.vehicle.getPosition(self.ego)
            for vid in departed:
                if vid == self.ego:
                    continue
                x, y = tc.vehicle.getPosition(vid)
                dist = math.hypot(x - ex, y - ey)
                if dist <= NEAR_EGO_M:
                    w = self._where(vid)
                    edge = self.net.getEdge(tc.vehicle.getRoadID(vid)) if not w["lane"].startswith(":") else None
                    boundary = edge is not None and not [e for e in edge.getIncoming().keys()
                                                         if e.allows("passenger") and e.getFunction() != "internal"]
                    place = ("mid_road" if w["pos"] > EDGE_START_M else "boundary_edge_start" if boundary
                             else "interior_edge_start")
                    d["near_departures"].append(dict(t=t, dist=round(dist, 1), place=place, **w))
            for pid in sim.getDepartedPersonIDList():
                x, y = tc.person.getPosition(pid)
                if math.hypot(x - ex, y - ey) <= NEAR_EGO_M:
                    d["near_person_departures"] += 1
        if round(t * 10) % 50 == 0:        # 5초마다 오래 서 있는 차와 사람
            for vid in tc.vehicle.getIDList():
                wait = tc.vehicle.getWaitingTime(vid)
                if wait >= STUCK_S:
                    rec = d["stuck"].setdefault(vid, dict(first_t=t, **self._where(vid)))
                    rec["max_wait"] = round(max(rec.get("max_wait", 0.0), wait), 1)
                    rec["last_t"] = t
            for pid in tc.person.getIDList():
                wait = tc.person.getWaitingTime(pid)
                if wait >= STUCK_S:
                    rec = d["stuck_persons"].setdefault(pid, dict(first_t=t, id=pid, edge=tc.person.getRoadID(pid)))
                    rec["max_wait"] = round(max(rec.get("max_wait", 0.0), wait), 1)
                    rec["last_t"] = t
        return result

    def restore(self):
        for name, fn in self._orig.items():
            setattr(self.tc, name, fn)

    def summary(self):
        d = self.data
        log = (self.out / "sumo.log").read_text(encoding="utf-8", errors="replace") if (self.out / "sumo.log").exists() else ""
        braking = re.findall(r"Vehicle '([^']+)' performs emergency braking on lane '([^']+)' with decel=([\d.]+)"
                             r".*?time=(\d+(?:\.\d+)?)", log)
        stats = {}
        if (self.out / "statistics.xml").exists():
            try:
                root = ET.parse(self.out / "statistics.xml").getroot()
                stats = {tag: dict(root.find(tag).attrib) for tag in ("vehicles", "teleports", "safety")
                         if root.find(tag) is not None}
            except ET.ParseError:
                pass
        end = d["sim_end"] or 0.0
        stuck = sorted(d["stuck"].values(), key=lambda r: -r["max_wait"])
        pairs = {tuple(sorted((c["collider"], c["victim"]))) for c in d["collisions"]}
        first = {}
        for c in d["collisions"]:
            first.setdefault(tuple(sorted((c["collider"], c["victim"]))), c)
        places = collections.Counter(n["place"] for n in d["near_departures"])
        return dict(
            sim_seconds=end, ego_first_seen=d["ego_first_seen"], departed=d["departed"],
            teleports=len(d["teleports"]), teleport_examples=d["teleports"][:5],
            collision_events=len(pairs), collision_examples=list(first.values())[:8],
            collisions_ego=sum(1 for p in pairs if self.ego in p),
            emergency_stops=len({e["id"] for e in d["emergency_stops"]}), emergency_stop_examples=d["emergency_stops"][:5],
            emergency_braking=len(braking), emergency_braking_ego=sum(1 for b in braking if b[0] == self.ego),
            emergency_braking_examples=[dict(id=b[0], lane=b[1], decel=float(b[2]), t=float(b[3])) for b in braking[:8]],
            stuck_over_120s=len(stuck), stuck_over_cycle=sum(1 for r in stuck if r["max_wait"] > LONGEST_CYCLE_S),
            stuck_at_end=sum(1 for r in stuck if end - r["last_t"] <= 5.0),
            stuck_examples=stuck[:8],
            stuck_lanes=collections.Counter(r["lane"] for r in stuck).most_common(6),
            stuck_persons_over_120s=len(d["stuck_persons"]),
            stuck_person_examples=sorted(d["stuck_persons"].values(), key=lambda r: -r["max_wait"])[:5],
            near_departures=len(d["near_departures"]), near_departures_by_place=dict(places),
            near_departure_examples=[n for n in d["near_departures"] if n["place"] != "boundary_edge_start"][:8],
            near_person_departures=d["near_person_departures"], sumo_statistics=stats,
            log_warnings=collections.Counter(re.sub(r"'[^']*'|[\d.]+", "#", line)[:70]
                                             for line in log.splitlines() if line.startswith("Warning")).most_common(8))


def run_env_episode(scenario, seed, out_dir, max_steps):
    """학습·평가와 같은 환경(collect_pomdp_data.py와 같은 구성)을 전문가 정책으로 한 에피소드 돈다."""
    from env import road_builder
    from env.mdp_config import ACTION, OBSERVATION, SIMULATION
    from env.road_config import EGO, ROAD, TRAFFIC, configure_map
    from env.route_policy import make_expert
    from env.sumo_env import SumoHighwayEnv
    from train import REWARD
    configure_map(scenario, seed, None, "hapjeong")
    SIMULATION["max_steps"] = max_steps
    cfg = road_builder.build(ROAD, EGO, TRAFFIC, os.path.join(out_dir, "sumo"))
    env = SumoHighwayEnv(cfg_path=cfg, road=ROAD, ego=EGO, mdp_sim=SIMULATION, mdp_obs=OBSERVATION, action=ACTION,
                         reward=REWARD)
    probe = SumoProbe(out_dir, env.ego_id)
    started = time.time()
    obs, reset_info = env.reset(seed=seed)
    act = make_expert(env)
    done, steps, info, intervened = False, 0, {}, 0
    while not done:
        obs, _reward, terminated, truncated, info = env.step(act(obs))
        intervened += bool(info["speed_intervened"])
        steps += 1
        done = terminated or truncated
    env.close()
    probe.restore()
    return dict(path="env", scenario=scenario, seed=seed, wall_seconds=round(time.time() - started, 1), steps=steps,
                termination=info.get("termination_reason"), stops_served=info.get("stops_served"),
                stops_missed=info.get("stops_missed"), speed_intervened_rate=round(intervened / max(steps, 1), 4),
                warmup_seconds=reset_info["warmup_seconds"], ego_insert_wait_seconds=reset_info["ego_insert_wait_seconds"],
                **probe.summary())


def run_traffic_measure(scenario, seed, out_dir):
    """scripts/measure_s1_s6.py를 이 프로세스 안에서 그대로 돌리며 같은 사건을 적는다."""
    import runpy
    os.environ["TMPDIR"] = str(Path(out_dir) / "tmp")      # 다른 측정과 /tmp/occ_* 폴더가 겹치지 않게
    Path(os.environ["TMPDIR"]).mkdir(parents=True, exist_ok=True)
    tempfile.tempdir = None
    probe = SumoProbe(out_dir, "bus0")
    sys.argv = ["measure_s1_s6.py", scenario, str(seed), "audit"]
    started = time.time()
    stdout, sys.stdout = sys.stdout, open(os.devnull, "w")
    try:
        runpy.run_path(str(REPO / "scripts" / "measure_s1_s6.py"), run_name="__main__")
    finally:
        sys.stdout = stdout
    probe.restore()
    result = json.loads((REPO / "results" / "traffic_verification" / f"s1_s6_{scenario}_{seed}_audit.json").read_text())
    return dict(path="traffic", scenario=scenario, seed=seed, wall_seconds=round(time.time() - started, 1),
                laps_done=result.get("laps_done"), lap_seconds=result.get("lap_seconds"),
                measure_safety={k: result["safety"].get(k) for k in ("collision_events", "teleports")},
                bus=result.get("bus"), **probe.summary())


def _worker(job):
    kind, scenario, seed, out_dir, max_steps = job
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    os.dup2(os.open(os.path.join(out_dir, "stderr.log"), os.O_WRONLY | os.O_CREAT | os.O_TRUNC), 2)   # SUMO 경고는 파일로
    try:
        if kind == "env":
            return run_env_episode(scenario, seed, out_dir, max_steps)
        return run_traffic_measure(scenario, seed, out_dir)
    except Exception as exc:                # 한 실행이 죽어도 나머지 결과는 남긴다
        import traceback
        return dict(path=kind, scenario=scenario, seed=seed, error=f"{type(exc).__name__}: {exc}",
                    traceback=traceback.format_exc()[-1500:])


EVAL_SEED = {"quiet": 1000, "baseline": 1300, "busy": 1600}      # research/av_bc_rl/evaluate.py의 dev seed


def run_dynamic(args):
    import multiprocessing
    out = Path(args.out).resolve()
    jobs = [("env", scenario, EVAL_SEED[scenario] + k, str(out / f"env_{scenario}_{EVAL_SEED[scenario] + k}"),
             args.max_steps) for scenario in args.scenarios for k in range(args.seeds)]
    jobs += [("traffic", scenario, seed, str(out / f"traffic_{scenario}_{seed}"), 0)
             for scenario in args.traffic_scenarios for seed in args.traffic_seeds]
    with multiprocessing.get_context("spawn").Pool(min(args.jobs, 4), maxtasksperchild=1) as pool:
        runs = []
        for run in pool.imap_unordered(_worker, jobs):
            runs.append(run)
            print(f"  {run['path']} {run['scenario']} {run['seed']}: "
                  + (run.get("error") or f"{run.get('termination') or run.get('laps_done')} "
                     f"충돌 {run['collision_events']} 순간이동 {run['teleports']} 비상제동 {run['emergency_braking']} "
                     f"120초 넘게 섬 {run['stuck_over_120s']} ({run['wall_seconds']}s)"), flush=True)
    runs.sort(key=lambda r: (r["path"], ["quiet", "baseline", "busy"].index(r["scenario"]), r["seed"]))
    return runs


COLUMNS = (("경로", lambda r: r["path"]), ("교통량", lambda r: r["scenario"]), ("seed", lambda r: r["seed"]),
           ("끝난 사유", lambda r: r.get("termination") or f"{r.get('laps_done')}바퀴"),
           ("시뮬 초", lambda r: f"{r['sim_seconds']:.0f}"), ("순간이동", lambda r: r["teleports"]),
           ("충돌(쌍)", lambda r: r["collision_events"]), ("버스 충돌", lambda r: r["collisions_ego"]),
           ("비상 제동", lambda r: r["emergency_braking"]), ("버스 비상 제동", lambda r: r["emergency_braking_ego"]),
           ("비상 정지", lambda r: r["emergency_stops"]), ("120초 넘게 선 차", lambda r: r["stuck_over_120s"]),
           ("182초 넘게", lambda r: r["stuck_over_cycle"]), ("끝까지 서 있음", lambda r: r["stuck_at_end"]),
           ("120초 넘게 선 사람", lambda r: r["stuck_persons_over_120s"]),
           ("버스 50m 안 출현(도로 한가운데/안쪽 도로 시작/경계)", lambda r: "/".join(
               str(r["near_departures_by_place"].get(k, 0)) for k in ("mid_road", "interior_edge_start", "boundary_edge_start"))),
           ("버스 50m 안 보행자 출현", lambda r: r["near_person_departures"]))


def dynamic_markdown(runs):
    ok = [r for r in runs if "error" not in r]
    lines = ["| " + " | ".join(name for name, _ in COLUMNS) + " |", "|" + "---|" * len(COLUMNS)]
    lines += ["| " + " | ".join(str(get(r)) for _, get in COLUMNS) + " |" for r in ok]
    lines.append("")
    for r in runs:
        if "error" in r:
            lines.append(f"- 실패: {r['path']} {r['scenario']} {r['seed']}: {r['error']}")
    for r in ok:
        notes = []
        if r["collision_examples"]:
            notes.append("충돌 " + "; ".join(f"t={c['t']:.0f} {c['collider']}({c['collider_type']})→{c['victim']}"
                                           f"({c['victim_type']}) {c['lane']} {c['kind']}" for c in r["collision_examples"][:4]))
        if r["stuck_lanes"]:
            notes.append("오래 선 곳 " + ", ".join(f"{lane}×{n}" for lane, n in r["stuck_lanes"]))
        if r["stuck_examples"]:
            s = r["stuck_examples"][0]
            notes.append(f"가장 오래 선 차 {s['id']}({s['type']}) {s['lane']} {s['pos']}m {s['max_wait']}초")
        if r["near_departure_examples"]:
            notes.append("버스 근처 출현 " + "; ".join(f"{n['id']} {n['lane']} {n['pos']}m({n['place']}, {n['dist']}m)"
                                                    for n in r["near_departure_examples"][:4]))
        if r["emergency_braking_examples"]:
            notes.append("비상 제동 " + "; ".join(f"{b['id']} {b['lane']} {b['decel']}m/s²" for b in r["emergency_braking_examples"][:4]))
        if notes:
            lines.append(f"- {r['path']} {r['scenario']} {r['seed']}: " + " / ".join(notes))
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dynamic", action="store_true", help="SUMO를 돌려 사건을 센다(몇 분~수십 분)")
    parser.add_argument("--self-check", action="store_true", help="심어 둔 나쁜 예를 검사기가 잡는지 확인")
    parser.add_argument("--strict", action="store_true", help="OPEN(알려진 문제)도 실패로 센다")
    parser.add_argument("--verbose", action="store_true", help="ALLOW도 모두 출력")
    parser.add_argument("--scenarios", nargs="+", default=["quiet", "baseline", "busy"])
    parser.add_argument("--seeds", type=int, default=3, help="env 경로에서 교통량마다 돌릴 에피소드 수")
    parser.add_argument("--max-steps", type=int, default=1400, help="env 에피소드 길이(evaluate.py와 같은 1400)")
    parser.add_argument("--traffic-scenarios", nargs="*", default=["busy"])
    parser.add_argument("--traffic-seeds", nargs="*", type=int, default=[0, 1, 3])
    parser.add_argument("--jobs", type=int, default=4, help="동시에 띄울 SUMO 수(최대 4)")
    parser.add_argument("--out", default=str(REPO / "results" / "naturalness" / "latest"))
    args = parser.parse_args()

    if args.self_check:
        missing, wrongly_allowed, hits = self_check()
        print(f"자기 점검: 심은 나쁜 예 {len(hits)}건을 찾음, 못 찾은 종류 {sorted(missing)}, 허용표가 통과시킨 것 {len(wrongly_allowed)}")
        return 1 if missing or wrongly_allowed else 0

    hits, stale = run_static()
    print_static(hits, stale, args.verbose)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "static.json").write_text(json.dumps(dict(hits=hits, stale=stale), ensure_ascii=False, indent=1))
    verdicts = collections.Counter(h["verdict"].split(":")[0] for h in hits)
    failed = verdicts["UNKNOWN"] > 0 or (args.strict and verdicts["OPEN"] > 0)

    if args.dynamic:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO, capture_output=True, text=True).stdout.strip()
        print(f"\n동적 검사 시작(commit {commit}, 결과 폴더 {out})")
        runs = run_dynamic(args)
        (out / "dynamic.json").write_text(json.dumps(dict(commit=commit, runs=runs), ensure_ascii=False, indent=1))
        (out / "dynamic.md").write_text(dynamic_markdown(runs), encoding="utf-8")
        print("\n" + dynamic_markdown(runs))
        print(out / "dynamic.json")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
