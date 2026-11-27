"""차선변경 요청은 수업 원본의 간격 게이트를 지나야 SUMO에 간다. 끝나는 차로에서 기어가는 버스에게도 예외가 없다.

게이트를 지난 요청은 SUMO가 옆 차와 무관하게 실행한다(laneChangeMode 0). 게이트가 측면충돌을 막는 유일한 환경 장치다."""
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
    """R111b_1(27m)의 16m 지점에 선 버스와, 왼쪽 차로에서 버스 뒤범퍼 2m 뒤에 선 승용차(follower가 거짓이면 없다)."""

    def __init__(self, speed, follower):
        self.speed, self.follower, self.changes = speed, follower, []

    def getIDList(self):
        return [EGO_ID, FOLLOWER] if self.follower else [EGO_ID]

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

    def getNeighbors(self, _vid, _mode):
        return []

    def getRoute(self, _vid):
        return ("R111b",)

    def getRouteIndex(self, _vid):
        return 0

    def changeLane(self, vid, lane_index, duration):
        self.changes.append((vid, lane_index, duration))


class _Lane:
    def __init__(self, follower):
        self.follower = follower

    def getAllowed(self, _lane):
        return []

    def getLastStepVehicleIDs(self, lane):
        if lane == "R111b_2":
            return [FOLLOWER] if self.follower else []
        return [EGO_ID]

    def getLength(self, _lane):
        return 27.0

    def getLinks(self, _lane):
        return []


class _Edge:
    def getLaneNumber(self, _edge):
        return 4


class _Simulation:
    def getTime(self):
        return 100.0


class _FakeTraci:
    TraCIException = traci.TraCIException

    def __init__(self, speed, follower):
        self.vehicle, self.lane, self.edge = _Vehicle(speed, follower), _Lane(follower), _Edge()
        self.simulation = _Simulation()


@pytest.fixture(scope="module")
def env():
    configure_map("busy", 0, None, "hapjeong")
    cfg = os.path.join(os.path.dirname(ROAD["network"]["net_path"]), "scenario.sumocfg")
    return sumo_env.SumoHighwayEnv(cfg_path=cfg, road=ROAD, ego=EGO, mdp_sim=SIMULATION, mdp_obs=OBSERVATION,
                                   action=ACTION, reward=REWARD)


def _request_left(env, monkeypatch, speed, follower=True):
    fake = _FakeTraci(speed, follower)
    monkeypatch.setattr(sumo_env, "traci", fake)
    env._segment_cache.clear()
    return env._apply_lane_change(1), fake.vehicle.changes


@pytest.mark.parametrize("speed", [0.0, 0.5, 5.0])
def test_a_car_2m_behind_refuses_the_request_at_any_speed(env, monkeypatch, speed):
    """끝나는 차로 끝에서 서 있거나 기어가도 게이트를 건너뛰지 않는다."""
    applied, changes = _request_left(env, monkeypatch, speed)
    assert not applied and changes == []


def test_a_clear_lane_sends_the_request(env, monkeypatch):
    applied, changes = _request_left(env, monkeypatch, speed=0.5, follower=False)
    assert applied and changes == [(EGO_ID, 2, ACTION["lane_change_duration"])]


def test_the_gate_has_no_lane_end_exception():
    assert not [key for key in ACTION["lane_change_safety"] if key.startswith("lane_end")]
