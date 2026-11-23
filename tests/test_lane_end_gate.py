"""끝나는 차로에서 서 있거나 기어가는 버스의 차선변경 요청은 환경 게이트를 지나 SUMO에 간다. 달리는 중에는 그대로 막는다."""
import os
import sys

import pytest
import traci

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from env import sumo_env  # noqa: E402
from env.mdp_config import ACTION, OBSERVATION, REWARD, SIMULATION  # noqa: E402
from env.road_config import EGO, ROAD, configure_map  # noqa: E402

EGO_ID, FOLLOWER = "ego", "f"


class _Vehicle:
    """R111b_1(27m)의 16m 지점에 선 버스와, 왼쪽 차로에서 버스 뒤범퍼 2m 뒤에 선 승용차."""

    def __init__(self, speed, best_lane_offset):
        self.speed, self.best_lane_offset, self.changes = speed, best_lane_offset, []

    def getIDList(self):
        return [EGO_ID, FOLLOWER]

    def getRoadID(self, _vid):
        return "R111b"

    def getLaneID(self, vid):
        return "R111b_1" if vid == EGO_ID else "R111b_2"

    def getLaneIndex(self, vid):
        return 1 if vid == EGO_ID else 2

    def getLanePosition(self, vid):
        return 16.0 if vid == EGO_ID else 2.0

    def getSpeed(self, vid):
        return self.speed if vid == EGO_ID else 0.0

    def getLength(self, vid):
        return 12.0 if vid == EGO_ID else 5.0

    def getDistance(self, _vid):
        return 900.0

    def getBestLanes(self, _vid):
        return [("R111b_1", 27.0, 0.0, self.best_lane_offset, False, ()), ("R111b_2", 27.0, 0.0, 0, True, ())]

    def getNeighbors(self, _vid, _mode):
        return []

    def getRoute(self, _vid):
        return ("R111b",)

    def getRouteIndex(self, _vid):
        return 0

    def changeLane(self, vid, lane_index, duration):
        self.changes.append((vid, lane_index, duration))


class _Lane:
    def getAllowed(self, _lane):
        return []

    def getLastStepVehicleIDs(self, lane):
        return [FOLLOWER] if lane == "R111b_2" else [EGO_ID]

    def getLength(self, _lane):
        return 27.0

    def getLinks(self, _lane):
        return []


class _Edge:
    def getLaneNumber(self, _edge):
        return 4


class _FakeTraci:
    TraCIException = traci.TraCIException

    def __init__(self, speed, best_lane_offset):
        self.vehicle, self.lane, self.edge = _Vehicle(speed, best_lane_offset), _Lane(), _Edge()


@pytest.fixture(scope="module")
def env():
    configure_map("busy", 0, None, "hapjeong")
    cfg = os.path.join(os.path.dirname(ROAD["network"]["net_path"]), "scenario.sumocfg")
    return sumo_env.SumoHighwayEnv(cfg_path=cfg, road=ROAD, ego=EGO, mdp_sim=SIMULATION, mdp_obs=OBSERVATION,
                                   action=ACTION, reward=REWARD)


def _request_left(env, monkeypatch, speed, best_lane_offset=1):
    fake = _FakeTraci(speed, best_lane_offset)
    monkeypatch.setattr(sumo_env, "traci", fake)
    env._segment_cache.clear()
    return env._apply_lane_change(1), fake.vehicle.changes


def test_creeping_at_the_lane_end_sends_the_request_despite_a_car_2m_behind(env, monkeypatch):
    applied, changes = _request_left(env, monkeypatch, speed=0.5)
    assert applied and changes == [(EGO_ID, 2, ACTION["lane_change_duration"])]


def test_moving_bus_is_still_refused_with_the_same_gap(env, monkeypatch):
    applied, changes = _request_left(env, monkeypatch, speed=5.0)
    assert not applied and changes == []


def test_a_change_the_route_does_not_need_is_still_refused(env, monkeypatch):
    """지금 차로가 경로로 이어지면(정류장 차로 맞추기 같은 변경) 서 있어도 간격 게이트를 지나야 한다."""
    applied, changes = _request_left(env, monkeypatch, speed=0.5, best_lane_offset=0)
    assert not applied and changes == []
