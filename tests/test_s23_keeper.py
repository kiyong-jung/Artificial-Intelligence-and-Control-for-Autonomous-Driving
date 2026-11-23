"""S2·S3와 버스 주변 키퍼: 키퍼가 있으면 주입기는 차를 직접 넣지 않고, 키퍼가 넣은 차 중에서 버스 목표 차로 뒤 가장 가까운 차를 상대로 고른다."""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import test_injector_plan as T  # noqa: E402
from traffic.scenario_injector import ScenarioInjector  # noqa: E402
from traffic.test_bus import STOP_AT, lane_plans  # noqa: E402


class _FakeKeeper:
    def __init__(self, cars=()):
        self.cars, self.spawned = set(cars), []

    def is_keeper_car(self, vid):
        return vid in self.cars

    def spawn_car(self, path, lane, pos):
        vid = f"win{len(self.spawned) + 1}"
        self.spawned.append((list(path), lane, pos))
        self.cars.add(vid)
        return vid


def _inj(tc, keeper):
    plans = lane_plans(T._net(), T._route(), T._stops())
    return ScenarioInjector(tc, T._net(), "bus0", T._stops(), [], "carDist", lap_fn=lambda: 1, plans=plans,
                            rng=random.Random(0), stop_ids=STOP_AT, keeper=keeper)


def _in_s3_window():
    return T._TC2(bus_road="R71", bus_pos=70.0)               # S3 창 앞(J8 신호를 지난 뒤)


def test_without_a_keeper_car_the_injector_asks_the_keeper_for_one_and_injects_nothing_itself():
    tc = _in_s3_window()
    kp = _FakeKeeper()
    inj = _inj(tc, kp)
    inj.update(0.0)
    assert not any(a["id"].startswith("inj_lane") for a in tc.f.added)            # 주입기가 직접 넣지 않는다
    assert len(kp.spawned) == 1 and kp.spawned[0][0][0] == "R71" and kp.spawned[0][1] == 1
    assert inj.tracked_ids() == {"win1"}                                            # 키퍼 차가 이번 창의 상대다


def test_selects_the_nearest_follower_whether_keeper_car_or_natural():
    tc = _in_s3_window()
    tc.f.right_followers = [("natural", 12.0), ("win7", 25.0), ("win9", 40.0)]     # S3는 오른쪽 차로
    kp = _FakeKeeper(cars={"win7", "win9"})
    inj = _inj(tc, kp)
    inj.update(0.0)
    assert kp.spawned == [] and inj.tracked_ids() == {"natural"}                    # 키퍼 차든 자연 차량이든 가장 가까운 후행차를 고른다
    assert inj.log[-1]["ok"] == "natural" and inj.log[-1]["id"] == "natural"


def test_no_platoon_at_red_when_a_keeper_is_present():
    # 키퍼가 있으면 신호 대기 때 줄 세우기(_queue_platoon)를 하지 않는다. 키퍼가 창에서 차로마다 유지한다
    tc = T._TC2(bus_road="R69", bus_pos=T._net().getEdge("R69").getLength() - 3.0, bus_speed=0.0)
    kp = _FakeKeeper()
    inj = _inj(tc, kp)
    inj.update(0.0)
    assert not any(a["id"].startswith("inj_lane") for a in tc.f.added) and kp.spawned == []


def test_a_keeper_car_being_tracked_is_not_a_removal_candidate():
    tc = _in_s3_window()
    kp = _FakeKeeper(cars={"win7"})
    tc.f.right_followers = [("win7", 25.0)]
    inj = _inj(tc, kp)
    inj.update(0.0)
    assert "win7" in inj.tracked_ids()
