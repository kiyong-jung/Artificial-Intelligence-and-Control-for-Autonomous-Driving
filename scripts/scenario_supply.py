"""배경 교통만 30분 돌려 S5(골목 합류 차량)와 S6(횡단보도 보행자)가 얼마나 공급되는지 센다.

    python scripts/scenario_supply.py
"""
import tempfile
import sys, collections
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
from env.road_config import ROAD, EGO, TRAFFIC, configure_map
from env import road_builder
from sumolib import checkBinary
import traci
configure_map("busy", 0, None, "hapjeong")
cfg = road_builder.build(ROAD, EGO, TRAFFIC, tempfile.mkdtemp())
traci.start([checkBinary("sumo"), "-c", cfg, "--no-warnings", "true", "--no-step-log", "true"])
on, alley, merged = set(), set(), collections.Counter()
for step in range(18000):
    traci.simulationStep()
    if step % 5: continue
    for p in traci.person.getIDList():
        if traci.person.getRoadID(p).startswith(":S6_wausanro_c"): on.add(p)
    for v in traci.vehicle.getIDList():
        if v.startswith(("f_G3_", "f_G12_")):
            alley.add(v); r = traci.vehicle.getRoadID(v)
            if r.startswith(("R71", "-R71")): merged[(v, r.split(".")[0])] += 1
traci.close()
into = collections.Counter(road for (_, road) in merged)
print("1800초 동안 S6 횡단보도를 건넌 보행자:", len(on)); print("골목(G3, G12)에서 나온 차:", len(alley), "합류한 방향:", dict(into))
