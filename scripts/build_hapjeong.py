"""합정역 실행 도로망과 부속 파일, map.json을 만든다.

    python scripts/build_hapjeong.py

입력은 서예인이 maps/hapjeong/generator/의 생성기로 만든 원본(source/network.base.net.xml,
source/signals.base.add.xml)이다. 임시 폴더에서 다음을 모두 마친 뒤에만 결과를 바꿔 넣는다.
1. 제한속도를 실제 기준으로 낮춘다(간선 50km/h, 연결로·"~길" 이면도로 30km/h, 더 낮은 값은 그대로).
2. 11m 간격으로 붙어 차가 교차로 안에 갇히던 J52·J56·J58을 한 교차로로 합치고 횡단보도를 다시 놓는다.
3. 신호 프로그램에서 합친 교차로의 J56 프로그램을 빼고(기본 신호 사용), J1에서 중앙버스차로가
   일반 차로로 합류하는 연결을 양보 녹색(g)으로 바꾼다. 같은 차로에 우선 녹색 두 개가 들어가던 곳이다.
4. 바뀐 도로망에서 보행자 경로를 원본과 같은 설정(시간당 600명, 최소 150m)과 고정 seed로 다시 만든다.
5. OSM의 실제 골목을 일방통행 1차로로 넣는다. 골목은 버스 경로에 신호 없이 합류한다(S5). 차가 다니지 않는
   원본의 30m 골목 토막은 뺀다.
6. 연구 시나리오(S1~S3, S6)용으로 OSM 위치의 가로변 정류장 6곳과 와우산로의 신호 없는 횡단보도 1곳을 둔다.
7. 배경 흐름(팔달문과 같은 규칙을 합계 900대/시간으로 줄임)과 ego 경로를 계산하고 validate_structure로 확인한다.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import sumolib

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.build_road import ALLEY, ARTERIAL_PER_LANE, publish, shortest_route
from scripts.patch_network import digest
from scripts.prune_network import OPTIONS
from scripts.validate_road import validate_structure

FOLDER = BASE / "maps" / "hapjeong"
SOURCE = FOLDER / "source"
VERSION = "3.5.0"
ARTERIAL_KMH, LINK_KMH, ALLEY_KMH = 50, 30, 30
LINK_EDGES = {"R65", "R65.c", "R66", "R66.c", "R67", "R67.c"}   # 생성기 설정에서 primary_link인 연결로
JOIN = [("J52", "J56", "J58")]
# 연결로가 양화로에 지퍼로 합류하던 J3은 busy에서 정체가 풀리지 않아, 연결로가 양보하는 합류로 바꾼다.
NODE_TYPES = {"J3": "priority"}
JOINED = "cluster_J52_J56_J58"
# 독막로를 건너는 두 곳만 둔다. 어울마당로(R120·R121·R122) 골목 쪽 횡단보도는 두지 않는다
JOINED_CROSSINGS = [("R73", "-R73"), ("R77", "-R77")]
# 중앙버스차로가 일반 차로로 합류하는 연결(신호 교차로 → [(나가는 도로, 들어가는 도로)])
YIELD_CONNECTIONS = {"J1": [("B2", "R40"), ("B1", "R41")]}
DROP_PROGRAMS = {"J56"}
# 양화로를 건너는 긴 횡단보도(36m, 39m)는 잔다리로 차량 신호와 같이 켜져 있었다. 잔다리로에서 좌회전·우회전한 차가
# 교차로를 빠져나가는 자리가 바로 이 횡단보도라, 건너는 보행자와 부딪혔다(버스 9회 실행 중 5번).
# 이 횡단보도는 차량이 모두 멈춘 보행자 전용 단계로 옮긴다(신호 교차로 → [건너는 도로들]).
EXCLUSIVE_CROSSINGS = {"J42": [("R53", "R55"), ("R54a", "R50")]}
EXCLUSIVE_CLEARANCE_S = 5
# 단계 뒤에 넣는 전적색(신호 교차로 → {원본 단계 번호: 초}).
# 상수역 사거리 J82는 황색 3초 뒤 바로 다음 방향이 출발했다. 교차로 안에서 맞은편을 기다리던 비보호 좌회전 차가 빠져나가기
# 전에 독막로 좌회전(버스 경로)이 들어와 부딪혔다(300회 점검의 충돌 5번이 모두 독막로 녹색 뒤 1.8~4.9초).
# 양화로 중앙정류장 횡단보도 J106은 보행 녹색 30초 뒤 바로 차량 녹색이라, 늦게 들어선 보행자가 남아 있었다.
ALL_RED_S: dict[str, dict[int, int]] = {"J82": {1: 3, 4: 3, 7: 5}, "J106": {2: 8}}
# 단계 길이를 바꾸는 곳(신호 교차로 → {원본 단계 번호: 초}). J106은 보행 녹색을 22초로 줄여 주기를 그대로 둔다.
# J48(잔다리로 횡단보도)은 보행 녹색 5초 뒤의 전적색을 5초에서 10초로 늘리고 차량 녹색을 그만큼 줄인다.
# J82는 버스가 들어오는 독막로 동행 녹색을 25초에서 35초로 늘린다. 줄이 짧아야 가로변 차로의 버스가 좌회전 차로로 옮긴다.
PHASE_SECONDS: dict[str, dict[int, int]] = {"J42": {8: 35, 9: 5, 11: 15}, "J82": {0: 35}, "J106": {2: 22}, "J48": {0: 70, 3: 10}}
# 한 단계를 한 진입로가 통째로 쓰게 하는 곳(신호 교차로 → {진입 도로: [원본 단계 번호]}). 그 단계에서 이 진입로의
# 모든 연결이 녹색(황색 단계면 황색)이고 다른 차량 연결은 적색이다.
# 서교동사거리 J42에서 잔다리로는 직진·우회전 39초 뒤 좌회전 20초였다. 버스가 가는 R94는 1차로가 직진·우회전,
# 2차로가 좌회전인데 우회전과 좌회전이 모두 많아, 줄이 88m 구간을 채우고 뒤의 횡단보도 J48과 골목까지 막았다.
# R94가 40초 동안 모든 방향으로 나가고, 맞은편 -R92가 그 뒤 15초를 쓴다.
APPROACH_PHASES: dict[str, dict[str, list[int]]] = {"J42": {"R94": [8, 9, 10], "-R92": [11, 12]}}
# 도로망의 북서쪽(월드컵로와 잔다리로 너머)은 버스 경로와 멀어 잘라낸다. 자른 점이 새 경계가 된다.
# 자르는 점 → (안쪽에서 나가는 도로, 들어오는 도로, 안쪽 교차로에서 자르는 점까지 m, 버리는 쪽의 교차로)
CUTS = {"CUT_worldcup": ("R61", "-R61", 200.0, "J49"),    # 월드컵로, 합정역 사거리 북쪽
        "CUT_jandari": ("R92", "-R92", 130.0, "J64")}     # 잔다리로, 서교동사거리 북서쪽
# (나가는 도로, 차로, 들어가는 도로, 차로). 합정역 R42 1·2번 차로가 둘 다 R40 1번 차로로 직진해 부딪히던 것은
# 생성기에서 R40을 일반 3차로로 넓혀 R42 직진 세 차로를 1:1로 이어 풀었다.
REMOVED_CONNECTIONS: list[tuple[str, str, str, str]] = []
# 차도 차로 수를 바꾸는 도로(도로 → 차도 차로 수)와, 그 때문에 통째로 다시 적는 연결(나가는 도로 → [(차로, 들어가는 도로, 차로)]).
# MOVED_LINKS는 새 연결이 원본의 어느 연결의 신호를 받는지 적는다.
# 상수역 사거리 앞 27m 구간 R111b는 차도가 3차로에서 2차로로 줄었다. 정지선 바로 앞에서 두 차가 한 차로에 나란히 끼인 채
# 우회전하다 교차로 안에서 서로를 막아 300초 뒤 순간이동했다. 3차로로 넓혀 차로를 1:1로 잇는다. 2차로는 직진과 좌회전,
# 3차로는 좌회전이다. 2차로에서도 좌회전이 되어야 가로변 차로를 달리던 버스가 한 번만 옮기고 좌회전한다.
WIDENED_EDGES: dict[str, int] = {"R111b": 3}
REPLACED_CONNECTIONS: dict[str, list[tuple[str, str, str]]] = {
    "R111": [("1", "R111b", "1"), ("2", "R111b", "2"), ("3", "R111b", "3")],
    "R111b": [("1", "R129", "1"), ("1", "R128", "1"), ("2", "R128", "2"), ("2", "R112", "1"), ("3", "R112", "2")],
    # 삼거리포차 앞과 서교동사거리의 좌회전은 2차로에서만 됐다. 가로변 차로를 달리는 버스가 좌회전 줄에 끼어들지 못하고
    # 300초를 서 있다 순간이동했다(규칙 버스 12회 중 1회). 가로변 차로에서도 좌회전해 가로변 차로로 들어가게 한다.
    "R112.221": [("1", "R130", "1"), ("1", "R113", "1"), ("2", "R113", "2")],
    "R94": [("1", "R53", "1"), ("1", "R92", "1"), ("1", "R54a", "1"), ("2", "R54a", "2"), ("2", "R54a", "4")]}
MOVED_LINKS: dict[tuple[str, str, str, str], tuple[str, str, str, str]] = {
    ("R111b", "3", "R112", "2"): ("R111b", "2", "R112", "2"), ("R112", "1", "R113", "1"): ("R112", "2", "R113", "1"),
    ("R94", "1", "R54a", "1"): ("R94", "2", "R54a", "2")}
# 배경 차량 baseline 합계. 생성기 설정(generator/road_config.py OSM_ROAD traffic)의 값이다.
BASELINE_VEHS_PER_HOUR = 900
PEDESTRIANS = {"persons_per_hour": 600, "min_distance": 150, "end": 3600, "seed": 42}
# 무작위 보행자 경로는 도로망이 바뀔 때마다 달라져, 신호 없는 횡단보도(S6)를 건너는 사람이 없을 수도 있다.
# S6를 건너는 보행자를 따로 넣는다. 와우산로 양쪽 인도를 번갈아 출발한다. 시간당 인원은 가정값이다.
S6_WALKERS_PER_HOUR = 60
# 신호 없는 횡단보도(S6). OSM 13236303911(crossing=uncontrolled)은 와우산로 차도 위의 점이다.
MIDBLOCK_PRIORITY = False      # S6 횡단보도를 비우선으로 둔다(주변 교통 A3)
MIDBLOCK_CROSSINGS = {"S6_wausanro": ("R112", "-R112", 37.5498511, 126.9231277)}
# 연구 경로의 가로변 정류장(OSM 노드, 이름, ARS 번호, 위도, 경도, 놓을 도로). 위치는 Overpass에서 2026-11-03에 받았다.
CURB_STOPS = [
    ("357920065", "합정역7번출구", "14-119", 37.5486897, 126.9142951, "R69"),
    ("357920024", "성산중학교", "", 37.5480489, 126.9174405, "R71"),
    ("357916587", "상수역", "", 37.5476100, 126.9212700, "R111"),
    ("5930628870", "극동방송", "", 37.5495471, 126.9231077, "R112"),
    ("3446267493", "서교동교회", "14-984", 37.5517944, 126.9193584, "-R96"),
    ("3424857165", "합정역", "14-219", 37.5510384, 126.9157425, "R54a"),
]
# 골목은 차가 실제로 다니는 방향만 둔다. busy 교통을 한 시간 흘려 차가 한 대도 지나지 않은 차도(골목의 반대 방향,
# 독막로 남쪽 골목망, 양화로6길 동쪽 구간, 원본의 30m 토막)는 뺐다. 남은 골목은 버스 경로로 차를 내보내거나
# (S5) 두 간선을 잇는다. 골목의 안쪽 끝은 건물과 주차장에서 차가 나오는 출발점이다.
# 골목이 독막로에 새로 닿는 점(OSM 노드)에서는 독막로를 나눠 신호 없는 교차로를 만든다.
ALLEY_SPLITS = {"X_dongmak3": ("R71", "-R71", "1834932797"),   # 독막로3길
                "X_dongmak4": ("R71", "-R71", "3182282262"),   # 독막로4길(성산중학교 정류장 15m 앞)
                "X_dongmak6": ("R71", "-R71", "1834932795")}   # 독막로6길
# 골목끼리 만나는 점과 골목의 안쪽 끝. 도로망 노드 id → OSM 노드 id
ALLEY_NODES = {"Y2": "3066579587", "D4": "3182251264", "S2": "2323311243"}
STUB_ENDS: list[str] = []   # 원본 골목 토막의 끝 가운데 골목이 이어지는 점
# 원본의 골목 토막. R120(어울마당로에서 독막로로 나오는 길)만 남긴다. J8의 토막(R116)으로는 독막로 동쪽 방향 차가
# 골목으로 좌회전해 되돌아 나왔는데, 이 좌회전 차가 횡단보도 위에서 맞은편을 기다리다 보행자와 부딪혔다.
REMOVED_EDGES = ["R116", "-R116", "R117", "-R117", "R118", "-R118", "R119", "-R119", "R121", "R122", "R123", "-R123"]
# 토막을 떼어 냈거나 차로를 바꾼 신호 교차로. netconvert는 읽어 온 신호의 연결 번호를 그대로 두어 번호에 빈칸이 생기고, 다시 놓는
# 횡단보도가 남은 번호와 겹친다. 첫 단계에서 신호를 떼었다가 둘째 단계에서 다시 달아 번호를 새로 매기게 한다.
RESIGNALED = ["J8", "J23", "J82", "J83", "J42"]
SERVICE_KMH = 20   # maps/paldalmun/source/speed_defaults.json의 service
# (도로 id, 이름, OSM 길 id, 시작, 끝, km/h, 일방통행). 시작·끝은 위의 노드 id다. 굽은 점은 source/area.osm.xml의
# 그 길에서 시작과 끝 사이의 점을 쓴다. OSM 길이 area.osm.xml에 없으면(None) 직선이다. 양방향이면 "-" + id도 만든다.
ALLEYS = [
    ("G3", "독막로3길", "172603989", "Y2", "X_dongmak3", ALLEY_KMH, True),
    # 독막로 남쪽의 고리. 성산중학교 정류장 앞에서 독막로4길로 들어가 성지1길과 독막로6길을 거쳐 정류장 끝에서 다시 나온다.
    # 신호 교차로 J8의 독막로2길로 들어가게 하면 우회전 차가 횡단보도에 남은 보행자와 부딪혔다.
    ("G30", "독막로4길", "312368845", "X_dongmak4", "D4", SERVICE_KMH, True),
    ("G11", "성지1길", "223433265", "D4", "S2", SERVICE_KMH, True),
    ("G12", "독막로6길", "223433270", "S2", "X_dongmak6", SERVICE_KMH, True),
]
# 골목 안(주차, 건물)에서 나와 간선에 신호 없이 합류하는 차(S5)를 이 도로에서 출발시킨다.
# 유입량은 경계 골목과 같은 ALLEY다.
ALLEY_ORIGINS = ["G3"]
# 간선에서 들어가 간선으로 다시 나오는 골목(이름 → 지나는 도로들). 골목 안에서 차를 출발시키지 않는다. 입구와 출구 사이의
# 간선을 지나는 배경 차량 가운데 골목 출발지 하나만큼(ALLEY × ALLEY_INFLOW)이 이 길로 돌아간다. SUMO는 가장 빠른 길만 고르므로
# 흐름에 경유 도로(via)를 적어야 골목으로 간다.
THROUGH_ALLEYS = {"dongmak6": ["G30", "G11", "G12"]}
# 지름길 후보. 버스 경로가 둘러싼 구역 안의 실제 OSM 길이고, 골목과 마찬가지로 신호가 없다.
# splits는 버스 경로 도로를 나눠 만드는 신호 없는 합류점(나가는 방향 도로, 반대 방향 도로, OSM 노드),
# nodes는 골목끼리 만나는 점, attach는 신호를 그대로 둔 채 골목을 붙이는 기존 신호 교차로, origins는 골목 안에서 차가 출발하는 도로,
# exits는 골목에서 나갈 수 있는 도로를 좁힐 때 쓴다(골목 도로 → 나갈 도로들. 없으면 모든 방향).
# 무엇을 켤지는 scripts/shortcut_experiment.py의 측정으로 정하고 ACTIVE_SHORTCUTS에 적는다.
SHORTCUTS = {
    "dongmak7": {   # 독막로7길: 양화로6길에서 북동쪽으로 잔다리로까지
        "splits": {"X_jandari7": ("-R96", "R96", "3038710046")},
        "nodes": {"Y3": "1834932799"},
        # 양화로6길 서쪽 구간이 독막로3길(Y2)과 독막로7길을 잇는다. 차는 이 구간에서 출발한다.
        "alleys": [("G4", "양화로6길", "172603991", "Y2", "Y3", ALLEY_KMH, True),
                   ("G16", "독막로7길", "172115945", "Y3", "X_jandari7", ALLEY_KMH, True)],
        "origins": ["G4"]},
    "jandari3": {   # 독막로3길 북쪽과 잔다리로3길: 양화로6길에서 잔다리로의 서교동교회 정류장 앞까지
        "splits": {"X_jandari3": ("-R96", "R96", "436869371")},
        "nodes": {"Z1": "1834932810"},
        "alleys": [("G17", "독막로3길", "172603989", "Y2", "Z1", ALLEY_KMH, True),
                   ("G18", "잔다리로3길", "172603995", "Z1", "X_jandari3", ALLEY_KMH, True)],
        "origins": ["G17"]},
    # 어울마당로: 독막로에서 잔다리로의 신호 횡단보도 J74까지. 방향별 일방통행이다.
    "eoulmadang": {
        "nodes": {"E1": "4175978557", "Y4": "1834932796", "Y5": "3743682656"},
        "attach": ["J74"],
        "stubs": ["J92"],
        "alleys": [("G19", "어울마당로", "-11", "Y5", "J74", ALLEY_KMH, True),
                   ("G20", "어울마당로", "-50", "J74", "E1", ALLEY_KMH, True),
                   ("G21", "어울마당로", "168229931", "E1", "Y4", ALLEY_KMH, True),
                   ("G8", "어울마당로", "168229931", "Y4", "J92", ALLEY_KMH, True)],
        "origins": ["G19"]},
    # 잔다리로6길: 버스 경로 바깥(북동쪽)에서 잔다리로의 신호 횡단보도 J48로 들어오는 일방통행. 서교동사거리 좌회전 대기 구간 앞이다.
    "jandari6": {
        "nodes": {"N6": "1830469500"},
        "attach": ["J48"],
        "alleys": [("G22", "잔다리로6길", "-7", "N6", "J48", ALLEY_KMH, True)],
        "exits": {"G22": ["R94"]},   # 우회전만. 좌회전으로 4차로를 건너던 골목 차가 직진 차와 부딪혔다
        "origins": []},
    # 잔다리로2길: 버스 경로 바깥(북동쪽)에서 잔다리로로 나오는 골목. 독막로7길과 잔다리로3길 합류점 사이다.
    "jandari2": {
        "splits": {"X_jandari2": ("-R96", "R96", "3038710044")},
        "nodes": {"N2": "1830469519"},   # 잔다리로6길과 만나는 점
        "alleys": [("G23", "잔다리로2길", "-8", "N2", "X_jandari2", SERVICE_KMH, True)],
        "origins": []},
    # 와우산로11길: 버스 경로 안쪽에서 와우산로로 나오는 골목. 극동방송 정류장 63m 앞, 신호 없는 횡단보도 107m 앞이다.
    # 나오는 방향만 만든다. 양방향으로 두거나 어울마당로까지 이으면 와우산로 차가 정류장 앞을 지나지 않고 빠진다.
    "wausan11": {
        "splits": {"X_wausan11": ("R112", "-R112", "1831696284")},
        "nodes": {"W11": "1831696285"},
        "alleys": [("G24", "와우산로11길", "172245595", "W11", "X_wausan11", ALLEY_KMH, True)],
        "exits": {"G24": ["R112"]},   # 버스와 같은 북쪽 방향으로만 나온다
        "origins": []},
    # 와우산로 동쪽의 이름 없는 출입로(OSM 1442563730, 18m). 극동방송 정류장 약 90m 앞에서 차가 버스와 같은 북쪽 방향으로
    # 가장자리 차로에 나온다. 바로 옆 독막로19길(정류장 40m 앞)은 실제로 와우산로에서 들어가기만 하는 일방통행인데, 들어가게 하면
    # 남쪽 방향 좌회전 차가 북쪽 두 차로를 가로지르다 300회 점검에서 112번 부딪혔다. 출입로는 사건을 늘리지 않았지만 버스 80회
    # 주행 두 번에 이 자리 끼어들기가 2번씩뿐이라 넣지 않았다. 와우산로11길 합류점과 18m 거리라 효과가 겹친다.
    "febc_drive": {
        "splits": {"X_febc": ("R112", "-R112", "13236318512")},
        "nodes": {"F0": "13236303922"},
        "alleys": [("G28", "와우산로 출입로", "1442563730", "F0", "X_febc", SERVICE_KMH, True)],
        "exits": {"G28": ["R112"]},
        "origins": []},
    # 아래 둘은 막다른 끝으로 남은 골목을 실제 길로 이어 준다.
    # 잔다리로6길의 남쪽 구간: 잔다리로2길과 잔다리로6길을 잇는다. 버스 경로 북동쪽 바깥에 고리가 생긴다.
    "jandari26_link": {
        "alleys": [("G25", "잔다리로6길", "172111342", "N2", "N6", SERVICE_KMH, False)],
        "origins": ["G25", "-G25"]},
    # 와우산로13길과 독막로15길: 와우산로11길을 어울마당로·양화로6길이 만나는 Y5까지 잇는다. 와우산로13길의 나머지
    # (정류장과 횡단보도를 지난 뒤 와우산로에 닿는 구간)는 넣지 않는다.
    "wausan13_link": {
        "nodes": {"W13": "1831674381"},
        "alleys": [("G26", "와우산로13길", "172243759", "Y5", "W13", ALLEY_KMH, True),
                   ("G27", "독막로15길", "172245594", "W13", "W11", ALLEY_KMH, True)],
        "origins": ["G26"]},   # Y5 쪽에서 출발해야 와우산로13길 구간에도 차가 다닌다
    # 아래는 길을 더하지 않는 비교 실험용이다.
    # 양화로 중앙버스전용차로(OSM highway=busway)를 일반 차도 다니게 연다. 전용차로가 없다면 어떻게 되는지 볼 때 쓴다.
    "open_bus_lanes": {"alleys": [], "origins": [], "open_bus_lanes": True},
    # 골목에서 출발하는 차를 세 배로(간선 유입은 그대로 두고 그 위에 더한다), 신호 없는 횡단보도(S6) 보행자를 네 배로 늘린다.
    "alley_inflow_x3": {"alleys": [], "origins": [], "alley_inflow": 3.0},
    # 배경 차량 합계를 1.5배로, 상수역 사거리와 삼거리포차 앞으로 나가는 차의 몫을 두 배로 한다.
    "more_traffic": {"alleys": [], "origins": [], "baseline_vehs_per_hour": 1350},
    # 배경 차량을 고정 간격이 아니라 무작위 간격으로 넣는다(env/road_builder.py).
    "poisson_arrivals": {"alleys": [], "origins": [], "flow_arrivals": "poisson"},
    "sangsu_exits_x2": {"alleys": [], "origins": [], "exit_weights": {"R128": 2.0, "R129": 2.0, "R130": 2.0}},
    "s6_walkers_x4": {"alleys": [], "origins": [], "s6_walkers_per_hour": 240},
}
# 측정으로 고른 것(maps/hapjeong/README.md). 길 여덟 개와 교통 설정 네 개다.
ACTIVE_SHORTCUTS: list[str] = ["dongmak7", "jandari3", "eoulmadang", "jandari6", "jandari2", "wausan11", "jandari26_link",
                               "wausan13_link", "alley_inflow_x3", "poisson_arrivals", "s6_walkers_x4", "sangsu_exits_x2"]
BASE_ALLEYS = (dict(ALLEY_SPLITS), dict(ALLEY_NODES), list(ALLEYS), list(ALLEY_ORIGINS), list(STUB_ENDS))
ATTACHED: list[str] = []
ALLEY_IDS: set[str] = set()
ALLEY_EXITS: dict[str, list[str]] = {}
OPEN_BUS_LANES = False
# area.osm.xml은 와우산로 서쪽 130m에서 끊긴다. 그 동쪽(와우산로 일대)은 2026-11-13에 Overpass에서 따로 받았다.
OSM_FILES = ["area.osm.xml", "area.east.osm.xml"]
ALLEY_INFLOW = 1.0   # 합계를 맞춘 뒤 골목 유입에만 곱하는 값. 간선 유입은 줄지 않는다
BASE_VEHS_PER_HOUR = BASELINE_VEHS_PER_HOUR
EXIT_WEIGHTS: dict[str, float] = {}   # 진출로 → 그 진출로로 가는 차의 몫에 곱하는 값
FLOW_ARRIVALS = "fixed"   # "poisson"이면 배경 차량이 무작위 간격으로 들어온다
BASE_S6_WALKERS_PER_HOUR = S6_WALKERS_PER_HOUR


def use_shortcuts(names: list[str]) -> None:
    """골목 설정을 기본 골목에 names의 지름길을 더한 것으로 바꾼다."""
    global ALLEY_SPLITS, ALLEY_NODES, ALLEYS, ALLEY_ORIGINS, ATTACHED, ACTIVE_SHORTCUTS, ALLEY_IDS, ALLEY_EXITS, OPEN_BUS_LANES
    global ALLEY_INFLOW, S6_WALKERS_PER_HOUR, BASELINE_VEHS_PER_HOUR, EXIT_WEIGHTS, FLOW_ARRIVALS, STUB_ENDS
    splits, nodes, alleys, origins, stubs = (type(part)(part) for part in BASE_ALLEYS)
    attached, exits, open_bus_lanes, alley_inflow, walkers = [], {}, False, 1.0, BASE_S6_WALKERS_PER_HOUR
    baseline, weights, arrivals = BASE_VEHS_PER_HOUR, {}, "fixed"
    for name in names:
        shortcut = SHORTCUTS[name]
        splits.update(shortcut.get("splits", {}))
        nodes.update(shortcut.get("nodes", {}))
        alleys += shortcut["alleys"]
        origins += shortcut["origins"]
        attached += shortcut.get("attach", [])
        stubs += shortcut.get("stubs", [])
        exits.update(shortcut.get("exits", {}))
        open_bus_lanes = open_bus_lanes or shortcut.get("open_bus_lanes", False)
        alley_inflow = shortcut.get("alley_inflow", alley_inflow)
        walkers = shortcut.get("s6_walkers_per_hour", walkers)
        baseline = shortcut.get("baseline_vehs_per_hour", baseline)
        arrivals = shortcut.get("flow_arrivals", arrivals)
        weights.update(shortcut.get("exit_weights", {}))
    ALLEY_SPLITS, ALLEY_NODES, ALLEYS, ALLEY_ORIGINS, ATTACHED, ACTIVE_SHORTCUTS = splits, nodes, alleys, origins, attached, list(names)
    ALLEY_IDS = {direction + alley[0] for alley in alleys for direction in ("", "-")}
    ALLEY_EXITS, OPEN_BUS_LANES, ALLEY_INFLOW, S6_WALKERS_PER_HOUR = exits, open_bus_lanes, alley_inflow, walkers
    BASELINE_VEHS_PER_HOUR, EXIT_WEIGHTS, FLOW_ARRIVALS, STUB_ENDS = baseline, weights, arrivals, stubs


EGO_ROUTE = "bus_loop"
# (설명, 경유 도로). 서예인이 제안한 순환 경로다. 상수역 정차 뒤 약 140m 안에 와우산로로 좌회전한다(S2).
ROUTES = {EGO_ROUTE: ("[연구] 합정역 → 성산중학교 → 상수역 → 와우산로 좌회전 → 극동방송 → 잔다리로 → 서교동교회 → "
                      "서교동사거리 좌회전 → 양화로 → 합정역",
                      ["R69", "R71", "R111", "R112", "R113", "-R96", "R94", "R54a", "R54", "R42"])}
EXTRA_FILES = {
    "additional": ["source/signals.add.xml", "source/islands.add.xml", "source/seams.add.xml", "source/stops.add.xml"],
    "routes": ["source/pedestrians.rou.xml"],
}


def netconvert(args: list[str], output: Path) -> None:
    completed = subprocess.run([sumolib.checkBinary("netconvert"), *args, *OPTIONS, "--output-file", str(output)],
                               capture_output=True, text=True, timeout=300)
    if completed.returncode:
        raise RuntimeError("netconvert 실패: " + completed.stderr[-3000:])
    # 머리 주석의 생성 시각과 임시 경로를 지워 빌드마다 같은 바이트가 되게 한다.
    ET.ElementTree(ET.parse(output).getroot()).write(output, encoding="utf-8", xml_declaration=True)


def build_network(temporary: Path) -> Path:
    base = sumolib.net.readNet(str(SOURCE / "network.base.net.xml"))
    edges = ET.Element("edges")
    patches = {}
    for edge in base.getEdges():
        if edge.getID() in REMOVED_EDGES:
            continue
        # 이름이 "~길"인 이면도로는 30km/h다. 50km/h 골목 토막 끝에서 출발한 차가 바로 앞 횡단보도에서 멈추지 못했다.
        kmh = LINK_KMH if edge.getID() in LINK_EDGES else ALLEY_KMH if edge.getName().endswith("길") else ARTERIAL_KMH
        target = kmh / 3.6
        if max(lane.getSpeed() for lane in edge.getLanes()) > target + 0.01:
            patches[edge.getID()] = ET.SubElement(edges, "edge", id=edge.getID(), speed=f"{target:.2f}")
        for lane in edge.getLanes() if OPEN_BUS_LANES else []:
            if lane.allows("bus") and not lane.allows("passenger") and not lane.allows("pedestrian"):
                element = patches.setdefault(edge.getID(), patches.get(edge.getID()) or ET.SubElement(edges, "edge", id=edge.getID()))
                ET.SubElement(element, "lane", index=str(lane.getIndex()), disallow="pedestrian")
    for edge_id, count in WIDENED_EDGES.items():
        element = patches.setdefault(edge_id, patches.get(edge_id) or ET.SubElement(edges, "edge", id=edge_id))
        walk = [lane for lane in base.getEdge(edge_id).getLanes() if lane.allows("pedestrian")]
        element.set("numLanes", str(count + len(walk)))
        for lane in walk:
            ET.SubElement(element, "lane", index=str(lane.getIndex()), allow="pedestrian", width=f"{lane.getWidth():.2f}")
        for index in range(len(walk), count + len(walk)):
            ET.SubElement(element, "lane", index=str(index), disallow="pedestrian")
    osm = osm_nodes()
    splits = {**MIDBLOCK_CROSSINGS, **{node_id: (forward, backward, *osm[osm_id][::-1])
                                       for node_id, (forward, backward, osm_id) in ALLEY_SPLITS.items()}}
    for node_id, (forward, backward, lat, lon) in splits.items():
        point = base.convertLonLat2XY(lon, lat)
        for edge_id in (forward, backward):
            position, _ = sumolib.geomhelper.polygonOffsetAndDistanceToPoint(point, base.getEdge(edge_id).getShape())
            element = patches.setdefault(edge_id, patches.get(edge_id) or ET.SubElement(edges, "edge", id=edge_id))
            ET.SubElement(element, "split", pos=f"{position:.2f}", id=node_id)
    for node_id, (outward, inward, distance, _) in CUTS.items():
        for edge_id, position in ((outward, distance), (inward, base.getEdge(inward).getLength() - distance)):
            element = patches.setdefault(edge_id, patches.get(edge_id) or ET.SubElement(edges, "edge", id=edge_id))
            ET.SubElement(element, "split", pos=f"{position:.2f}", id=node_id)
    nodes = ET.Element("nodes")
    for group in JOIN:
        ET.SubElement(nodes, "join", nodes=" ".join(group))
    for node_id, node_type in {**NODE_TYPES, **dict.fromkeys(RESIGNALED, "priority")}.items():
        ET.SubElement(nodes, "node", id=node_id, type=node_type)
    ET.ElementTree(edges).write(temporary / "speed.edg.xml", encoding="utf-8")
    ET.ElementTree(nodes).write(temporary / "join.nod.xml", encoding="utf-8")
    joined = temporary / "joined.net.xml"
    netconvert(["--sumo-net-file", str(SOURCE / "network.base.net.xml"), "--edge-files", str(temporary / "speed.edg.xml"),
                "--node-files", str(temporary / "join.nod.xml"), "--remove-edges.explicit", ",".join(REMOVED_EDGES)], joined)
    crossings = ET.Element("connections")
    for group in JOINED_CROSSINGS:
        ET.SubElement(crossings, "crossing", node=JOINED, edges=" ".join(group))
    split_net = sumolib.net.readNet(str(joined))
    discarded = discarded_edges(split_net)
    # 도로를 나누면 조각의 id가 바뀌어(R71 → R71.256) 그 도로를 건너던 횡단보도를 netconvert가 버린다.
    # 원본 횡단보도가 건너던 도로의 조각 중 그 교차로에 닿는 것으로 다시 놓는다.
    kept = {(node, frozenset(edges)) for node, edges in crossing_edges(joined)}
    for node_id, edges in crossing_edges(SOURCE / "network.base.net.xml"):
        if not split_net.hasNode(node_id) or (node_id, frozenset(edges)) in kept:
            continue
        node = split_net.getNode(node_id)
        pieces = [edge.getID() for edge in node.getIncoming() + node.getOutgoing() if original_id(edge.getID()) in edges]
        if {original_id(piece) for piece in pieces} == set(edges) and not set(pieces) & set(discarded):
            ET.SubElement(crossings, "crossing", node=node_id, edges=" ".join(pieces))
    for node_id, (forward, _, _, _) in MIDBLOCK_CROSSINGS.items():
        node = split_net.getNode(node_id)
        incoming = next(edge for edge in node.getIncoming() if original_id(edge.getID()) == forward)
        outgoing = next(edge for edge in node.getOutgoing() if edge.getToNode() == incoming.getFromNode())
        # 주변 교통 A3는 비우선 횡단보도(priority="false")를 전제한다. 우선이면 보행자 P1·P2의
        # jmIgnoreFoe*·impatience가 아무 영향이 없고 보행자가 차 속도와 무관하게 건넌다. 한국 법규상
        # 운전자 일시정지 의무와는 다르며 한계에 적는다.
        ET.SubElement(crossings, "crossing", node=node_id, edges=f"{incoming.getID()} {outgoing.getID()}",
                      priority="true" if MIDBLOCK_PRIORITY else "false")
    for source, source_lane, target, target_lane in REMOVED_CONNECTIONS:
        ET.SubElement(crossings, "delete", {"from": source, "to": target, "fromLane": source_lane, "toLane": target_lane})
    for source, targets in REPLACED_CONNECTIONS.items():
        for target, links in split_net.getEdge(source).getOutgoing().items():
            for link in links:
                ET.SubElement(crossings, "delete", {"from": source, "to": target.getID(),
                                                    "fromLane": str(link.getFromLane().getIndex()),
                                                    "toLane": str(link.getToLane().getIndex())})
        for source_lane, target, target_lane in targets:
            ET.SubElement(crossings, "connection", {"from": source, "to": target, "fromLane": source_lane, "toLane": target_lane})
    alley_nodes, alley_edges = add_alleys(split_net, crossings)
    ET.ElementTree(crossings).write(temporary / "crossings.con.xml", encoding="utf-8")
    ET.ElementTree(alley_nodes).write(temporary / "alleys.nod.xml", encoding="utf-8")
    ET.ElementTree(alley_edges).write(temporary / "alleys.edg.xml", encoding="utf-8")
    output = temporary / "network.net.xml"
    netconvert(["--sumo-net-file", str(joined), "--node-files", str(temporary / "alleys.nod.xml"),
                "--edge-files", str(temporary / "alleys.edg.xml"), "--connection-files", str(temporary / "crossings.con.xml"),
                "--remove-edges.explicit", ",".join(discarded)],
               output)
    keep_attached_signals(output)
    return output


def keep_attached_signals(net_path: Path) -> None:
    """골목을 붙인 신호 교차로(ATTACHED)의 신호 프로그램을 늘어난 연결에 맞춰 다시 적는다.

    netconvert는 연결 번호만 새로 매기고 프로그램은 원본 길이 그대로 둔다. 원래 있던 연결은 원본의 신호를 그대로 받는다.
    골목에서 나오는 연결은 간선 차량이 녹색인 단계에서 양보 녹색(g), 황색 단계에서 황색이고, 보행 신호와 그 뒤 전적색 동안은
    적색이다. 황색 내내 양보 녹색이면 그 끝에 들어간 골목 차가 우회전을 마치기 전에 보행 신호가 켜져 보행자를 쳤다.
    황색 없이 적색으로 바꾸면 정지선 앞의 골목 차가 비상 정지한다. 간선에서 골목으로 들어가는 연결은 같은 차로의
    신호를 양보로 받는다.
    """
    if not ATTACHED:
        return
    base_root = ET.parse(SOURCE / "network.base.net.xml").getroot()
    base, built = signal_links(SOURCE / "network.base.net.xml"), signal_links(net_path)
    tree = ET.parse(net_path)
    for tls in ATTACHED:
        original = {key: index for index, key in base[tls].items()}
        source = next(logic for logic in base_root.iter("tlLogic") if logic.get("id") == tls)
        logic = next(logic for logic in tree.getroot().iter("tlLogic") if logic.get("id") == tls)
        for phase in list(logic):
            logic.remove(phase)
        for phase in source.iter("phase"):
            old, state = phase.get("state"), []
            cars = [old[index] for key, index in original.items() if key[0] != "crossing"]
            alley = "g" if any(mark in "Gg" for mark in cars) else "y" if "y" in cars else "r"
            for index in sorted(built[tls]):
                key = built[tls][index]
                if key in original:
                    state.append(old[original[key]])
                elif key[0] in ALLEY_IDS:
                    state.append(alley)
                else:
                    state.append(next(old[i] for k, i in original.items() if k[:2] == key[:2]).replace("G", "g"))
            ET.SubElement(logic, "phase", duration=phase.get("duration"), state="".join(state))
        phases = logic.findall("phase")
        for index, seconds in PHASE_SECONDS.get(tls, {}).items():
            phases[index].set("duration", str(seconds))
        for index, seconds in ALL_RED_S.get(tls, {}).items():
            logic.insert(list(logic).index(phases[index]) + 1,
                         ET.Element("phase", duration=str(seconds), state="r" * len(phases[0].get("state"))))
    ET.indent(tree, "    ")
    tree.write(net_path, encoding="utf-8", xml_declaration=True)


def discarded_edges(net) -> list[str]:
    """자르는 점 너머의 도로. 버리는 쪽 교차로에서 자르는 점을 넘지 않고 닿는 도로 전부다."""
    reached, queue = set(), [far for *_, far in CUTS.values()]
    while queue:
        node_id = queue.pop()
        if node_id in reached or node_id in CUTS:
            continue
        reached.add(node_id)
        node = net.getNode(node_id)
        queue += [edge.getToNode().getID() for edge in node.getOutgoing()]
        queue += [edge.getFromNode().getID() for edge in node.getIncoming()]
    edges = sorted(edge.getID() for edge in net.getEdges()
                   if edge.getFromNode().getID() in reached or edge.getToNode().getID() in reached)
    kept_nodes = {node.getID() for node in net.getNodes()} - reached
    if "J1" not in kept_nodes or len(edges) > len(net.getEdges()) / 2:
        raise ValueError("자르는 점이 도로망을 둘로 나누지 못했습니다. 버리는 쪽이 남길 쪽과 다른 길로 이어져 있습니다.")
    return edges


def original_id(edge_id: str) -> str:
    """나뉜 도로 조각(R71.256)의 원래 id(R71)."""
    return edge_id.split(".")[0]


def crossing_edges(net_path: Path) -> list[tuple[str, list[str]]]:
    """도로망의 횡단보도마다 (교차로, 건너는 도로들)."""
    return [(edge.get("id")[1:].rsplit("_c", 1)[0], edge.get("crossingEdges").split())
            for edge in ET.parse(net_path).getroot().iter("edge") if edge.get("function") == "crossing"]


def signal_links(net_path: Path) -> dict[str, dict[int, tuple]]:
    """신호 교차로 → {연결 번호: 그 번호가 가리키는 연결}. 도로 조각은 원래 id로 적어 원본과 견줄 수 있게 한다."""
    root = ET.parse(net_path).getroot()
    crossed = {edge.get("id"): frozenset(map(original_id, edge.get("crossingEdges").split()))
               for edge in root.iter("edge") if edge.get("function") == "crossing"}
    links: dict[str, dict[int, tuple]] = {}
    for connection in root.iter("connection"):
        if connection.get("tl") is None:
            continue
        target = connection.get("to")
        key = (("crossing", crossed[target]) if target in crossed else
               (original_id(connection.get("from")), connection.get("fromLane"), original_id(target), connection.get("toLane")))
        key = MOVED_LINKS.get(key, key)
        links.setdefault(connection.get("tl"), {})[int(connection.get("linkIndex"))] = key
    return links


def osm_nodes() -> dict[str, tuple[float, float]]:
    """source/area.osm.xml의 노드 id → (경도, 위도)."""
    return {node.get("id"): (float(node.get("lon")), float(node.get("lat")))
            for name in OSM_FILES for node in ET.parse(SOURCE / name).getroot().iter("node")}


def add_alleys(net, connections: ET.Element) -> tuple[ET.Element, ET.Element]:
    """골목 노드·도로를 만들고, 이미 있는 노드(토막 끝, 독막로를 나눈 점)에서 새 골목과의 연결을 차로까지 적는다.

    도로망 파일에서 읽은 노드의 연결은 고정값이라, 적지 않으면 새 골목과 이어지지 않는다.
    """
    osm = osm_nodes()
    way_nodes = {way.get("id"): [nd.get("ref") for nd in way.findall("nd")]
                 for name in OSM_FILES for way in ET.parse(SOURCE / name).getroot().iter("way")}
    nodes, edges = ET.Element("nodes"), ET.Element("edges")
    coords = {node_id: net.convertLonLat2XY(*osm[osm_id]) for node_id, osm_id in ALLEY_NODES.items()}
    for node_id, (x, y) in coords.items():
        ET.SubElement(nodes, "node", id=node_id, x=f"{x:.3f}", y=f"{y:.3f}", type="priority")
    attached = STUB_ENDS + list(ALLEY_SPLITS) + ATTACHED
    for node_id in STUB_ENDS:
        ET.SubElement(nodes, "node", id=node_id, type="priority")
    for node_id in RESIGNALED:
        ET.SubElement(nodes, "node", id=node_id, type="traffic_light", tl=node_id)
    for node_id in attached:
        coords[node_id] = net.getNode(node_id).getCoord()

    def shape(way_id, start, end):
        # OSM 길 위에서 시작점과 끝점 사이에 있는 점을 순서대로 넣는다.
        a, b = coords[start], coords[end]
        if way_id is None:
            return [a, b]
        points = [net.convertLonLat2XY(*osm[node]) for node in way_nodes[way_id] if node in osm]
        offset_a = sumolib.geomhelper.polygonOffsetAndDistanceToPoint(a, points)[0]
        offset_b = sumolib.geomhelper.polygonOffsetAndDistanceToPoint(b, points)[0]
        low, high = sorted((offset_a, offset_b))
        offsets, inner, length = [], [], 0.0
        for index, point in enumerate(points):
            if index:
                length += sumolib.geomhelper.distance(points[index - 1], point)
            if low + 0.5 < length < high - 0.5:
                inner.append(point)
        return [a, *(inner if offset_a < offset_b else inner[::-1]), b]
    new = {}
    for edge_id, name, way_id, start, end, kmh, oneway in ALLEYS:
        points = shape(way_id, start, end)
        directions = {edge_id: (start, end, points)}
        if not oneway:
            directions["-" + edge_id] = (end, start, points[::-1])
        for direction, (a, b, line) in directions.items():
            new[direction] = (a, b)
            ET.SubElement(edges, "edge", {"id": direction, "from": a, "to": b, "name": name, "numLanes": "1",
                                          "speed": f"{kmh / 3.6:.2f}", "priority": "1", "disallow": "pedestrian",
                                          "shape": " ".join(f"{x:.3f},{y:.3f}" for x, y in line)})

    def drivable(edge, outermost=True):
        lanes = [lane.getIndex() for lane in edge.getLanes() if lane.allows("passenger")]
        return str(lanes[0] if outermost else lanes[-1])
    for node_id in attached:
        node = net.getNode(node_id)
        for target, (a, b) in new.items():
            if a == node_id:
                for source in node.getIncoming():
                    if source.getFromNode().getID() != b:
                        # 우회전은 바깥 차로, 좌회전·직진은 안쪽 차로에서 골목으로 들어간다.
                        (x0, y0), (x1, y1) = source.getShape()[-2:]
                        x2, y2 = coords[b]
                        right = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1) < 0
                        ET.SubElement(connections, "connection", {"from": source.getID(), "to": target,
                                                                  "fromLane": drivable(source, right), "toLane": "0"})
            if b == node_id:
                for sink in node.getOutgoing():
                    if sink.getToNode().getID() != a and original_id(sink.getID()) in ALLEY_EXITS.get(target, [original_id(sink.getID())]):
                        ET.SubElement(connections, "connection", {"from": target, "to": sink.getID(),
                                                                  "fromLane": "0", "toLane": drivable(sink)})
    return nodes, edges


def build_signals(net_path: Path, output: Path) -> dict[str, list[int]]:
    """원본 신호 프로그램을 실행 도로망에 맞춰 쓴다. 양보 녹색으로 바꾼 연결 번호를 반환한다.

    신호 프로그램은 연결 번호 순서로 적혀 있다. 도로를 고치다 번호가 밀리면 다른 연결에 녹색이 가므로,
    칸마다 원본에서 같은 연결이 받던 신호를 찾아 옮겨 적는다. 원본에 없는 연결이 생겼거나, 일부러 지운
    연결(REMOVED_CONNECTIONS) 말고 다른 연결이 사라졌으면 멈춘다.
    """
    root = ET.parse(SOURCE / "signals.base.add.xml").getroot()
    base, built = signal_links(SOURCE / "network.base.net.xml"), signal_links(net_path)
    yielded = {}
    for logic in list(root.findall("tlLogic")):
        tls = logic.get("id")
        if tls in DROP_PROGRAMS:
            root.remove(logic)
            continue
        if tls not in built:
            raise ValueError(f"신호 프로그램 {tls}의 교차로가 도로망에 없습니다.")
        original = {key: index for index, key in base[tls].items()}
        gone = {key for key in original if key[0] != "crossing" and {key[0], key[2]} & set(REMOVED_EDGES)}
        changed = (set(original) ^ set(built[tls].values())) - set(REMOVED_CONNECTIONS) - gone
        if changed or len(original) != len(base[tls]):
            raise ValueError(f"신호 교차로 {tls}의 연결이 원본과 다릅니다: {sorted(map(str, changed))}")
        order = [original[built[tls][index]] for index in sorted(built[tls])]
        yielded[tls] = sorted(index for index, key in built[tls].items() if key[0] != "crossing"
                              and (key[0], key[2]) in YIELD_CONNECTIONS.get(tls, []))
        exclusive = sorted(index for index, key in built[tls].items() if key[0] == "crossing"
                           and key[1] in map(frozenset, EXCLUSIVE_CROSSINGS.get(tls, [])))
        walk_s = 0
        owner = {number: approach for approach, numbers in APPROACH_PHASES.get(tls, {}).items() for number in numbers}
        for number, phase in enumerate(logic.findall("phase")):
            state = [phase.get("state")[index] for index in order]
            if number in owner:
                mark = "y" if "y" in state else "G"
                state = [signal if key[0] == "crossing" else mark if key[0] == owner[number] else "r"
                         for signal, key in zip(state, (built[tls][index] for index in sorted(built[tls])))]
            for index in yielded[tls]:
                state[index] = "g" if state[index] == "G" else state[index]
            if any(state[index] == "G" for index in exclusive):
                walk_s = max(walk_s, int(float(phase.get("duration"))))
            for index in exclusive:
                state[index] = "r"
            phase.set("state", "".join(state))
        phases = logic.findall("phase")
        width = len(phases[0].get("state"))
        for index, seconds in PHASE_SECONDS.get(tls, {}).items():
            phases[index].set("duration", str(seconds))
        for index, seconds in ALL_RED_S.get(tls, {}).items():
            logic.insert(list(logic).index(phases[index]) + 1, ET.Element("phase", duration=str(seconds), state="r" * width))
        if exclusive:
            if not walk_s:
                raise ValueError(f"신호 교차로 {tls}에서 보행자 전용 단계로 옮길 횡단보도 녹색을 찾지 못했습니다.")
            # 교차로 안에서 기다리던 좌회전 차가 황색 뒤에 빠져나간다. 그 차가 나갈 때까지 보행 신호를 늦춘다.
            ET.SubElement(logic, "phase", duration=str(EXCLUSIVE_CLEARANCE_S), state="r" * width)
            ET.SubElement(logic, "phase", duration=str(walk_s),
                          state="".join("G" if index in exclusive else "r" for index in range(width)))
            ET.SubElement(logic, "phase", duration=str(EXCLUSIVE_CLEARANCE_S), state="r" * width)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return {tls: indexes for tls, indexes in yielded.items() if indexes}


def build_pedestrians(net_path: Path, output: Path, temporary: Path) -> None:
    tool = Path(os.environ["SUMO_HOME"]) / "tools" / "randomTrips.py"
    period = 3600 / PEDESTRIANS["persons_per_hour"]
    completed = subprocess.run(
        [sys.executable, str(tool), "-n", str(net_path), "--pedestrians", "-b", "0", "-e", str(PEDESTRIANS["end"]),
         "-p", f"{period:.4f}", "--min-distance", str(PEDESTRIANS["min_distance"]), "--seed", str(PEDESTRIANS["seed"]),
         "--prefix", "ped", "-o", str(temporary / "pedestrians.trips.xml"), "-r", str(output)],
        capture_output=True, text=True, timeout=600, cwd=temporary)
    if completed.returncode:
        raise RuntimeError("randomTrips 실패: " + completed.stderr[-3000:])
    root = ET.parse(output).getroot()
    net = sumolib.net.readNet(str(net_path))
    (forward, backward, *_), = MIDBLOCK_CROSSINGS.values()
    # 횡단보도 노드에서 끝나는 두 도로(양쪽 인도)를 잇는 걸음은 그 횡단보도를 건넌다.
    sides = [edge.getID() for edge in net.getNode(next(iter(MIDBLOCK_CROSSINGS))).getIncoming() if original_id(edge.getID()) in (forward, backward)]
    walkers = []
    for index in range(int(PEDESTRIANS["end"] * S6_WALKERS_PER_HOUR / 3600)):
        person = ET.Element("person", id=f"s6ped{index}", depart=f"{index * 3600 / S6_WALKERS_PER_HOUR:.2f}")
        ET.SubElement(person, "walk", edges=" ".join(sides if index % 2 == 0 else sides[::-1]))
        walkers.append(person)
    # SUMO는 출발 시각 순서로 읽는다.
    people = sorted([*root.findall("person"), *walkers], key=lambda person: float(person.get("depart")))
    for person in root.findall("person"):
        root.remove(person)
    root.extend(people)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def curb_stops(net) -> list[dict]:
    """OSM 정류장을 지정한 도로(나뉜 경우 그 조각 포함)의 가장 바깥 차도 차로에 25m 길이로 놓는다."""
    stops = []
    for osm_id, name, ref, lat, lon, edge_id in CURB_STOPS:
        point = net.convertLonLat2XY(lon, lat)
        candidates = []
        for edge in net.getEdges():
            if edge.getID() != edge_id and not edge.getID().startswith(edge_id + "."):
                continue
            lane = next(lane for lane in edge.getLanes() if lane.allows("bus"))
            offset, distance = sumolib.geomhelper.polygonOffsetAndDistanceToPoint(point, lane.getShape())
            candidates.append((distance, offset, lane))
        distance, offset, lane = min(candidates, key=lambda item: item[0])
        if distance > 15:  # OSM 정류장 점은 인도 위 승강장에 찍혀 차로에서 10m 넘게 떨어지기도 한다
            raise ValueError(f"정류장 {name}({osm_id})이 {edge_id}에서 {distance:.1f}m 떨어져 있습니다.")
        end = min(max(offset + 12.5, 25.5), lane.getLength() - 0.5)
        stops.append({"id": f"osm_stop_{osm_id}", "name": name, "ref": ref or None, "lane": lane.getID(),
                      "startPos": round(end - 25, 2), "endPos": round(end, 2),
                      "lanes_on_edge": sum(item.allows("bus") for item in lane.getEdge().getLanes()),
                      "source_osm_node": osm_id, "latitude": lat, "longitude": lon,
                      "snap_distance_m": round(distance, 3)})
    return stops


def scaled_flows(net) -> list[dict]:
    """경계 진입로와 ALLEY_ORIGINS마다 유입량을 정해 되돌아가는 진출로를 뺀 진출로에 같은 비율로 나누고, 합계를 줄인다.

    규칙은 scripts/build_road.py의 boundary_flows와 같다. 다만 경계는 노드 종류(dead_end)가 아니라 차가 다니는
    이웃 노드가 하나뿐인 도로 끝으로 본다. 원본 도로망은 월드컵로·동교로·잔다리로 등의 끝이 dead_end가 아니다.
    """
    def neighbours(node):
        return ({edge.getToNode() for edge in node.getOutgoing() if edge.allows("passenger")}
                | {edge.getFromNode() for edge in node.getIncoming() if edge.allows("passenger")})
    boundary = {node.getID() for node in net.getNodes() if len(neighbours(node)) == 1}
    drivable = [edge for edge in net.getEdges() if edge.allows("passenger")]
    entries = [edge for edge in drivable if edge.getFromNode().getID() in boundary]
    # 안쪽 끝에서 시작하는 골목은 경계 진입로로 이미 들어 있다.
    entries += [net.getEdge(edge) for edge in ALLEY_ORIGINS if edge not in {entry.getID() for entry in entries}]
    exits = [edge for edge in drivable if edge.getToNode().getID() in boundary]
    flows = []
    for entry in entries:
        lanes = sum(lane.allows("passenger") for lane in entry.getLanes())
        alley = entry.getSpeed() <= 10
        rate = ALLEY if alley else ARTERIAL_PER_LANE * min(lanes, 2)
        targets = [exit for exit in exits if exit.getToNode() != entry.getFromNode()
                   and net.getShortestPath(entry, exit, vClass="passenger")[0] is not None]
        share = {exit.getID(): EXIT_WEIGHTS.get(original_id(exit.getID()), 1.0) for exit in targets}
        flows += [{"id": f"f_{entry.getID()}_to_{exit.getID()}", "from": entry.getID(), "to": exit.getID(),
                   "vehs_per_hour": rate * share[exit.getID()] / sum(share.values()),
                   "boost": ALLEY_INFLOW if alley else 1.0} for exit in targets]
    for name, via in THROUGH_ALLEYS.items():
        entrances = {edge.getID() for edge in net.getEdge(via[0]).getIncoming()}
        leavings = {edge.getID() for edge in net.getEdge(via[-1]).getOutgoing()}

        def passes(flow):
            path = [edge.getID() for edge in net.getShortestPath(net.getEdge(flow["from"]), net.getEdge(flow["to"]),
                                                                 vClass="passenger")[0]]
            return any(path.index(a) < path.index(b) for a in entrances & set(path) for b in leavings & set(path))
        passing = [flow for flow in flows if "via" not in flow and passes(flow)]
        share = ALLEY * ALLEY_INFLOW / sum(flow["vehs_per_hour"] for flow in passing)
        for flow in passing:
            flows.append({**flow, "id": f"{flow['id']}_via_{name}", "via": " ".join(via),
                          "vehs_per_hour": flow["vehs_per_hour"] * share})
            flow["vehs_per_hour"] *= 1 - share
    scale = BASELINE_VEHS_PER_HOUR / sum(flow["vehs_per_hour"] for flow in flows)
    return [{"id": flow["id"], "from": flow["from"], "to": flow["to"], **({"via": flow["via"]} if "via" in flow else {}),
             "vehs_per_hour": round(flow["vehs_per_hour"] * scale * flow["boost"], 3)} for flow in flows]


def build() -> dict:
    with tempfile.TemporaryDirectory(prefix="hapjeong_build_") as name:
        temporary = Path(name)
        net_path = build_network(temporary)
        yield_links = build_signals(net_path, temporary / "signals.add.xml")
        build_pedestrians(net_path, temporary / "pedestrians.rou.xml", temporary)
        net = sumolib.net.readNet(str(net_path))
        routes, route_design = {}, {}
        for route_id, (description, anchors) in ROUTES.items():
            edges = routes[route_id] = shortest_route(net, anchors, route_id)
            lanes = [lane for edge in edges for lane in net.getEdge(edge).getLanes() if lane.allows("passenger")]
            route_design[route_id] = {
                "description": description,
                "length_m": round(sum(net.getEdge(edge).getLength() for edge in edges), 2),
                "max_speed_mps": max(lane.getSpeed() for lane in lanes),
                "first_edge_lanes": sum(lane.allows("passenger") for lane in net.getEdge(edges[0]).getLanes()),
            }
        (west, south), (east, north) = (net.convertXY2LonLat(*corner) for corner in
                                        (net.getBoundary()[:2], net.getBoundary()[2:]))
        center = net.convertLonLat2XY((west + east) / 2, (south + north) / 2)
        definition = {
            "id": "hapjeong", "name": "합정역 · 양화로와 독막로", "version": VERSION,
            "description": "합정역 일대 실제 도로. 신호 교차로, 인도·횡단보도·보행자, 중앙버스차로, 연구 경로의 가로변 정류장 6곳과 신호 없는 횡단보도가 있다.",
            "source_status": "osm_generated",
            "network_source": {"path": "source/network.net.xml", "sha256": digest(net_path),
                               "base_path": "source/network.base.net.xml", "base_sha256": digest(SOURCE / "network.base.net.xml"),
                               "generator": "generator/ (서예인, seogyo-hongdae 066e5b1)",
                               "patches": {"speed_kmh": {"arterial": ARTERIAL_KMH, "link": LINK_KMH, "alley": ALLEY_KMH},
                                           "joined_junctions": [list(group) for group in JOIN],
                                           "signal_yield_links": yield_links, "dropped_programs": sorted(DROP_PROGRAMS),
                                           "exclusive_crossings": {tls: [list(edges) for edges in crossings]
                                                                   for tls, crossings in EXCLUSIVE_CROSSINGS.items()},
                                           "shortcuts": [{"id": name, "edges": [alley[0] for alley in SHORTCUTS[name]["alleys"]],
                                                          "names": [alley[1] for alley in SHORTCUTS[name]["alleys"]],
                                                          "merges": list(SHORTCUTS[name].get("splits", {})) + SHORTCUTS[name].get("attach", [])}
                                                         for name in ACTIVE_SHORTCUTS],
                                           "cuts": {node: {"edges": [outward, inward], "distance_m": distance}
                                                    for node, (outward, inward, distance, _) in CUTS.items()},
                                           "removed_connections": [list(item) for item in REMOVED_CONNECTIONS],
                                           "through_alleys": THROUGH_ALLEYS},
                               "pedestrians": {**PEDESTRIANS, "s6_walkers_per_hour": S6_WALKERS_PER_HOUR}},
            "extra_files": EXTRA_FILES,
            "ego_vehicle": "bus",
            "focus_bbox": [round(west, 7), round(south, 7), round(east, 7), round(north, 7)],
            "gui_center": {"x": round(center[0], 3), "y": round(center[1], 3)},
            "sources_license": {"road_data": "ODbL 1.0", "attribution": "© OpenStreetMap contributors",
                                "url": "https://www.openstreetmap.org/copyright"},
            "assumptions": ["제한속도는 안전속도 5030 기준이다. 간선 50km/h, 연결로와 이름이 ~길인 이면도로 30km/h이고 OSM의 독막로 50km/h와 같다.",
                            "배경 차량 유입량은 팔달문과 같은 규칙으로 나눈 합성 수요다. quiet·baseline·busy는 0.5·1·1.25배다.",
                            "배경 차량 baseline 합계는 생성기 설정의 시간당 900대다."],
            "routes": routes, "route_design": route_design, "flows": scaled_flows(net), "flow_arrivals": FLOW_ARRIVALS,
            "max_lane_speed_mps": max(lane.getSpeed() for edge in net.getEdges() for lane in edge.getLanes()
                                      if lane.allows("passenger")),
            "ego_route": EGO_ROUTE, "bus_stops": curb_stops(net),
            "design": {"measure_lanes": [], "zones": {}},
        }
        validate_structure(definition, net_path)
        publish({SOURCE / "network.net.xml": net_path.read_bytes(),
                 SOURCE / "signals.add.xml": (temporary / "signals.add.xml").read_bytes(),
                 SOURCE / "pedestrians.rou.xml": (temporary / "pedestrians.rou.xml").read_bytes(),
                 FOLDER / "map.json": (json.dumps(definition, ensure_ascii=False, indent=2) + "\n").encode("utf-8")})
    return definition


use_shortcuts(ACTIVE_SHORTCUTS)

if __name__ == "__main__":
    result = build()
    print(f"hapjeong {result['version']}: 배경 흐름 {len(result['flows'])}개, "
          f"baseline {round(sum(flow['vehs_per_hour'] for flow in result['flows']))}대/시간, "
          f"실행망 sha256 {result['network_source']['sha256'][:12]}")
