"""시나리오 발생 측정기가 끼어들기를 세고, 반대 방향 합류는 세지 않는지 확인한다."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

from scenario_occurrence import Counter  # noqa: E402

ALLEY_END, MERGES = {"G3": "X_dongmak3"}, {"X_dongmak3"}


def row(t, vehicle_road, vehicle_position, ego_position=100.0):
    """버스는 R71.135를 동쪽(90도)으로 달리고, 차량 한 대가 주어진 도로에 있다."""
    return {
        "episode": 0, "action": {"lane_change_actual": 0}, "detected_vehicles": [],
        "state": {
            "sim_time": t,
            "ego": {"alive": True, "x": ego_position, "y": 0.0, "heading": 90.0, "speed": 5.0,
                    "road_id": "R71.135", "lane_position": ego_position},
            "navigation": {"distance_to_stop": 80.0, "next_stop_id": "s", "crossing_id": None, "distance_to_crossing": None},
            "vehicles": [{"id": "car", "x": ego_position + 30.0, "y": 5.0, "road_id": vehicle_road,
                          "lane_position": vehicle_position}],
            "pedestrians": [],
        },
    }


def count(rows):
    counter = Counter(ALLEY_END, MERGES)
    for item in rows:
        counter.add(item)
    return {scenario["id"]: scenario["count"] for scenario in counter.result()["scenarios"]}


def test_alley_car_entering_ahead_of_the_bus_is_one_cut_in():
    counts = count([row(0.0, "G3", 50.0), row(0.5, ":X_dongmak3_1", 2.0), row(1.0, "R71.135", 130.0),
                    row(1.5, "R71.135", 135.0)])
    assert counts["S5"] == 1
    assert counts["S5~"] == 2


def test_alley_car_joining_the_opposite_direction_is_not_a_cut_in():
    assert count([row(0.0, "G3", 50.0), row(0.5, ":X_dongmak3_0", 2.0), row(1.0, "-R71.175", 10.0)])["S5"] == 0


def test_car_that_never_used_an_alley_is_not_a_cut_in():
    assert count([row(0.0, "R71", 90.0), row(0.5, "R71.135", 130.0)])["S5"] == 0


def test_alley_car_entering_behind_the_bus_is_not_a_cut_in():
    assert count([row(0.0, "G3", 50.0), row(1.0, "R71.135", 60.0)])["S5"] == 0
