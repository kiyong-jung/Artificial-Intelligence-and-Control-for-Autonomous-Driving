"""수집 데이터에서 연구 시나리오(S1~S6)가 실제로 일어난 횟수를 센다.

    python3 scripts/scenario_occurrence.py data/hapjeong/run.jsonl.gz

시나리오의 뜻은 maps/hapjeong/README.md의 표에 있다. 판정 기준은 아래 SCENARIOS의 설명과 같다.
도로망은 수집 때 함께 저장된 <이름>.assets/sumo/network.net.xml에서 읽는다. 골목(id가 G로 시작)이
신호 없이 다른 도로와 만나는 교차로를 골목 합류점으로 본다. 표준 라이브러리만 쓴다.
"""
from __future__ import annotations

import gzip
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

NEAR_M = 50.0          # ego와 상대가 이 거리 안이면 마주친 것으로 본다
CUT_IN_WINDOW_S = 15.0  # 골목 합류점을 지난 뒤 이 시간 안에 버스 앞에 나타나면 끼어들기로 센다
ALLEY = re.compile(r"^-?G\d+$")
S6_CROSSING = ":S6_wausanro_c"
S6_WAITING = ":S6_wausanro_w"   # 횡단보도 양끝의 대기 공간. 인도를 따라 지나가는 보행자도 여기를 밟는다


def alley_merges(net_path: Path) -> tuple[dict[str, str], set[str]]:
    """(골목 도로 → 그 도로가 끝나는 골목 합류점, 합류점들). 합류점은 신호가 없고 골목 아닌 도로도 닿는 교차로다."""
    root = ET.parse(net_path).getroot()
    signalised = {junction.get("id") for junction in root.findall("junction") if junction.get("type", "").startswith("traffic_light")}
    ends, alleys_at, through = {}, set(), {}
    for edge in root.findall("edge"):
        if edge.get("function"):
            continue
        if ALLEY.match(edge.get("id")):
            ends[edge.get("id")] = edge.get("to")
            alleys_at |= {edge.get("from"), edge.get("to")}
        else:
            through.setdefault(edge.get("from"), set()).add(edge.get("to"))
            through.setdefault(edge.get("to"), set()).add(edge.get("from"))
    # 골목이 지나가는 도로(양쪽으로 이어진 도로)에 닿아야 합류점이다. 신호 교차로로 가는 토막의 끝은 뺀다.
    merges = {node for node in alleys_at if len(through.get(node, ())) >= 2 and node not in signalised}
    return {edge: node for edge, node in ends.items() if node in merges}, merges


def near(ego, other):
    return math.hypot(other["x"] - ego["x"], other["y"] - ego["y"]) <= NEAR_M


def ahead(ego, other):
    radians = math.radians(ego["heading"])
    return (other["x"] - ego["x"]) * math.sin(radians) + (other["y"] - ego["y"]) * math.cos(radians) > 0


class Counter:
    """에피소드를 순서대로 읽으며 시나리오별로 일어난 에피소드와 횟수를 모은다."""

    def __init__(self, alley_end: dict[str, str], merges: set[str]):
        self.alley_end, self.merges = alley_end, merges
        self.by_merge = {node: {"cut_ins": 0, "waiting_steps": 0} for node in sorted(merges)}
        self.episodes, self.steps = set(), 0
        self.hit = {key: set() for key, *_ in SCENARIOS}
        self.count = {key: 0 for key, *_ in SCENARIOS}
        self.stop_passes, self.stop_passes_shared = set(), set()
        self.in_lane_steps = 0
        self._episode, self._merged, self._cut_in = None, {}, set()

    def add(self, row: dict) -> None:
        episode = row["episode"]
        if episode != self._episode:
            self._episode, self._merged, self._cut_in = episode, {}, set()
        self.episodes.add(episode)
        self.steps += 1
        state = row["state"]
        ego = state["ego"]
        if not ego["alive"]:
            return
        now = state["sim_time"]
        in_lane = [d for d in row["detected_vehicles"] if d["lane_relation"] != "other"]
        self.in_lane_steps += bool(in_lane)
        events = {key: 0 for key, *_ in SCENARIOS}

        navigation = state["navigation"]
        if navigation["distance_to_stop"] == 0:
            stop = (episode, navigation["next_stop_id"])
            self.stop_passes.add(stop)
            if in_lane:
                self.stop_passes_shared.add(stop)
            events["S1"] += ego["speed"] < 0.1
        distance = navigation["distance_to_stop"]
        changed = row["action"]["lane_change_actual"]
        if navigation.get("next_turn") == "left" and changed > 0:
            events["S2"] += 1
            events["S2+"] += any(d["lane_relation"] == "left" for d in in_lane)
        events["S3"] += distance is not None and 0 < distance <= 100 and changed < 0
        events["S4"] += any(v["id"].startswith("taxi") and near(ego, v) for v in state["vehicles"])

        waiting = False
        for vehicle in state["vehicles"]:
            road = vehicle["road_id"]
            seen, origin = self._merged.get(vehicle["id"], (-1e9, None))
            recent = origin is not None and now - seen <= CUT_IN_WINDOW_S
            # 합류점 교차로 안의 차는 방금 그 골목에서 나온 차만 잇는다. 버스 경로를 따라 교차로를 지나가는 차는 끼어든 차가 아니다.
            merge = self.alley_end.get(road) or (origin if recent and road.startswith(f":{origin}_") else None)
            if merge:
                self._merged[vehicle["id"]] = (now, merge)
                if near(ego, vehicle) and ahead(ego, vehicle):
                    waiting = True
                    self.by_merge[merge]["waiting_steps"] += 1
                continue
            if (recent and vehicle["id"] not in self._cut_in
                    and road == ego["road_id"] and 0 < vehicle["lane_position"] - ego["lane_position"] <= NEAR_M):
                self._cut_in.add(vehicle["id"])
                self.by_merge[origin]["cut_ins"] += 1
                events["S5"] += 1
        events["S5~"] += waiting
        if str(navigation["crossing_id"]).startswith(S6_CROSSING) and navigation["distance_to_crossing"] <= NEAR_M:
            roads = [p["road_id"] for p in state["pedestrians"]]
            events["S6"] += any(road.startswith(S6_CROSSING) for road in roads)
            events["S6~"] += any(road.startswith(S6_WAITING) for road in roads)
        for key, value in events.items():
            if value:
                self.hit[key].add(episode)
                self.count[key] += int(value)

    def result(self) -> dict:
        episodes = len(self.episodes)
        return {
            "episodes": episodes, "steps": self.steps,
            "scenarios": [{"id": key, "name": name, "criterion": criterion, "episodes": len(self.hit[key]),
                           "count": self.count[key]} for key, name, criterion in SCENARIOS],
            "s5_cut_ins_per_episode": round(self.count["S5"] / episodes, 3) if episodes else None,
            "stop_passes": len(self.stop_passes), "stop_passes_with_vehicle_in_lane": len(self.stop_passes_shared),
            "in_lane_step_share": round(self.in_lane_steps / self.steps, 4) if self.steps else None,
            "alley_merges": self.by_merge,
        }


# (id, 이름, 판정 기준). count는 S5가 끼어든 차량 수이고 나머지는 스텝 수다.
SCENARIOS = [
    ("S1", "정류장 출발 후 재합류", "버스가 정류장 안에서 멈춤"),
    ("S2", "정류장 직후 좌회전 차로 변경", "다음 회전이 좌회전인 구간에서 왼쪽 차로로 옮김"),
    ("S2+", "그때 옮겨 갈 차로에 차가 있음", "왼쪽 차로에 감지 차량"),
    ("S3", "정류장 접근 차로 변경", "정류장 100m 앞에서 오른쪽으로 옮김"),
    ("S4", "정류장 앞 정차 차량", "50m 안에 정차 택시"),
    ("S5", "골목 차량 끼어들기", "신호 없는 골목 합류점에서 나온 차가 15초 안에 버스와 같은 도로의 50m 앞에 들어옴"),
    ("S5~", "골목 차량이 합류점에서 대기", "버스 앞 50m 안의 골목 합류점에 골목 차량이 있음"),
    ("S6", "신호 없는 횡단보도", "50m 앞 횡단보도 위에 보행자"),
    ("S6~", "횡단보도 끝에 보행자", "50m 앞 횡단보도의 양끝 대기 공간에 보행자(건너려고 기다리거나 인도를 지나감)"),
]


def main(path: str, net_path: str | None = None) -> dict:
    net = Path(net_path) if net_path else Path(path.replace(".jsonl.gz", ".assets")) / "sumo" / "network.net.xml"
    counter = Counter(*alley_merges(net))
    with gzip.open(path, "rt", encoding="utf-8") as lines:
        for line in lines:
            counter.add(json.loads(line))
    return counter.result()


if __name__ == "__main__":
    print(json.dumps(main(*sys.argv[1:3]), ensure_ascii=False, indent=1))
