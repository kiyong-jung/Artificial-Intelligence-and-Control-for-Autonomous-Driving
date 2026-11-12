"""수집 데이터에서 연구 시나리오(S1~S6)가 실제로 일어난 에피소드를 센다.

    python3 scripts/scenario_occurrence.py data/hapjeong/run.jsonl.gz

시나리오의 뜻은 maps/hapjeong/README.md의 표에 있다. 판정 기준은 SCENARIOS의 설명과 같다.
표준 라이브러리만 쓴다. 결과는 JSON으로 출력한다.
"""
import gzip
import json
import math
import sys

NEAR_M = 50.0          # ego와 상대가 이 거리 안이면 마주친 것으로 본다
ALLEY_FLOWS = ("f_G3_", "f_G12_")
ALLEY_ROADS = ("G3", "G12", ":X_dongmak")
S6_CROSSING = ":S6_wausanro_c"


def near(ego, other):
    return math.hypot(other["x"] - ego["x"], other["y"] - ego["y"]) <= NEAR_M


def s1(row):
    navigation, ego = row["state"]["navigation"], row["state"]["ego"]
    return navigation["distance_to_stop"] == 0 and ego["speed"] < 0.1


def s2(row):
    return row["state"]["ego"]["road_id"] in ("-R96", "R94") and row["action"]["lane_change_actual"] > 0


def s2_contested(row):
    return s2(row) and any(d["lane_relation"] == "left" for d in row["detected_vehicles"])


def s3(row):
    distance = row["state"]["navigation"]["distance_to_stop"]
    return distance is not None and 0 < distance <= 100 and row["action"]["lane_change_actual"] < 0


def s4(row):
    return any(v["id"].startswith("taxi") and near(row["state"]["ego"], v) for v in row["state"]["vehicles"])


def s5(row):
    ego = row["state"]["ego"]
    return ego["road_id"].startswith("R71") and any(
        v["id"].startswith(ALLEY_FLOWS) and v["road_id"].startswith(ALLEY_ROADS) and near(ego, v)
        for v in row["state"]["vehicles"])


def s6(row):
    navigation = row["state"]["navigation"]
    return (str(navigation["crossing_id"]).startswith(S6_CROSSING) and navigation["distance_to_crossing"] <= NEAR_M
            and any(p["road_id"].startswith(S6_CROSSING) for p in row["state"]["pedestrians"]))


SCENARIOS = [
    ("S1", "정류장 출발 후 재합류", "버스가 정류장 안에서 멈춤", s1),
    ("S2", "정류장 직후 좌회전 차로 변경", "서교동교회를 지나 왼쪽 차로로 옮김", s2),
    ("S2+", "그때 옮겨 갈 차로에 차가 있음", "왼쪽 차로에 감지 차량", s2_contested),
    ("S3", "정류장 접근 차로 변경", "정류장 100m 앞에서 오른쪽으로 옮김", s3),
    ("S4", "정류장 앞 정차 차량", "50m 안에 정차 택시", s4),
    ("S5", "골목 차량 끼어들기", "독막로에서 50m 안에 골목 합류 차량", s5),
    ("S6", "신호 없는 횡단보도", "50m 앞 횡단보도 위에 보행자", s6),
]


def main(path):
    episodes, hits, steps = set(), {key: set() for key, *_ in SCENARIOS}, {key: 0 for key, *_ in SCENARIOS}
    total = 0
    with gzip.open(path, "rt", encoding="utf-8") as lines:
        for line in lines:
            row = json.loads(line)
            total += 1
            episodes.add(row["episode"])
            if not row["state"]["ego"]["alive"]:
                continue
            for key, _, _, test in SCENARIOS:
                if test(row):
                    hits[key].add(row["episode"])
                    steps[key] += 1
    return {"episodes": len(episodes), "steps": total, "scenarios": [
        {"id": key, "name": name, "criterion": criterion, "episodes": len(hits[key]), "steps": steps[key]}
        for key, name, criterion, _ in SCENARIOS]}


if __name__ == "__main__":
    print(json.dumps(main(sys.argv[1]), ensure_ascii=False, indent=1))
