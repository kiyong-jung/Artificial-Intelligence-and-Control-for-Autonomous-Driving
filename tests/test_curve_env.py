"""환경이 곡선 속도 제한을 관측에 넣어, 전문가가 좌회전 전에 줄이고 횡가속도 한계 안에서 돈다(실제 SUMO).

환경은 수업 원본처럼 최고 속도(vmax)로만 자른다. 곡선 제한을 지키는 것은 정책이다."""
import math
import os
import subprocess
import sys

import numpy as np
import pytest
import traci
from sumolib import checkBinary

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from env.data_schema import OBSERVATION_FEATURES  # noqa: E402
from env.mdp_config import ACTION, OBSERVATION, REWARD, SIMULATION  # noqa: E402
from env.road_config import EGO, configure_map  # noqa: E402
from env.route_policy import make_expert  # noqa: E402
from env.sumo_env import SumoHighwayEnv  # noqa: E402

VMAX, DT = 13.89, SIMULATION["decision_interval"]
INDEX = {name: index for index, name in enumerate(OBSERVATION_FEATURES)}


@pytest.fixture(scope="module")
def env(tmp_path_factory):
    """250m 직선 뒤 90도 좌회전하는 한 차로 도로와 버스 한 대."""
    d = tmp_path_factory.mktemp("left_turn")
    (d / "n.nod.xml").write_text('<nodes><node id="W" x="-250" y="0"/><node id="C" x="0" y="0" type="priority"/>'
                                 '<node id="N" x="0" y="250"/></nodes>')
    (d / "e.edg.xml").write_text('<edges><edge id="in" from="W" to="C" numLanes="1" speed="13.89"/>'
                                 '<edge id="out" from="C" to="N" numLanes="1" speed="13.89"/></edges>')
    subprocess.run([checkBinary("netconvert"), "-n", str(d / "n.nod.xml"), "-e", str(d / "e.edg.xml"),
                    "-o", str(d / "network.net.xml"), "--no-warnings"], check=True, capture_output=True)
    (d / "r.rou.xml").write_text('<routes><vType id="ego_type" vClass="bus" length="12" width="2.5" accel="1.0" decel="2.0" '
                                 'minGap="3" tau="1.5" laneChangeModel="SL2015"/><route id="r0" edges="in out"/></routes>')
    (d / "scenario.sumocfg").write_text(
        '<configuration><input><net-file value="network.net.xml"/><route-files value="r.rou.xml"/></input>'
        '<processing><lateral-resolution value="0.8"/></processing><time><step-length value="0.1"/></time></configuration>')
    configure_map("quiet", 0, None, "hapjeong")
    road = {"seed": 0, "speed_limit": VMAX, "network": {"routes": {"r0": ["in", "out"]}, "bus_stops": []}}
    ego = {**EGO, "route_id": "r0", "service": {"stops": (), "dwell_s": 0.0}}
    simulation = {**SIMULATION, "warmup_steps": 0, "warmup_jitter_steps": 0, "max_steps": 300}
    if traci.isLoaded():
        traci.close()
    environment = SumoHighwayEnv(str(d / "scenario.sumocfg"), road, ego, simulation, OBSERVATION, ACTION, REWARD)
    yield environment
    environment.close()


def _drive(env, policy):
    """out 도로 40m까지 달리며 판단마다 (도로, 차로 위치, 속도, 방향, 관측)을 모은다."""
    obs, _info = env.reset(seed=0)
    rows = []
    for _ in range(300):
        rows.append((traci.vehicle.getRoadID("ego"), traci.vehicle.getLanePosition("ego"), traci.vehicle.getSpeed("ego"),
                     traci.vehicle.getAngle("ego"), obs))
        if rows[-1][0] == "out" and rows[-1][1] > 40.0:
            return rows
        obs, _reward, terminated, truncated, _info = env.step(policy(obs))
        assert not (terminated or truncated)
    raise AssertionError("버스가 좌회전을 마치지 못했다")


def _turn_speed(env, rows):
    """버스가 지나간 자리들의 곡선 속도 제한 중 가장 낮은 값."""
    return min(env.route_map.curve_speed(f"{row[0]}_0", row[1]) for row in rows)


def _lateral_peak(rows):
    return max(abs(math.radians((b[3] - a[3] + 180.0) % 360.0 - 180.0)) / DT * 0.5 * (a[2] + b[2])
               for a, b in zip(rows, rows[1:]))


def test_expert_slows_before_the_turn_and_stays_within_the_lateral_limit(env):
    rows = _drive(env, make_expert(env, version=5))
    limit = EGO["comfort"]["lateral_acceleration_limit"]
    assert max(row[2] for row in rows) > 12.0                       # 직선에서는 제한속도 가까이 달린다
    assert _lateral_peak(rows) <= limit * 1.1
    turn_speed = _turn_speed(env, rows)
    entering = next(row for row in rows if row[0].startswith(":"))
    assert entering[2] <= turn_speed + 2.0                           # 교차로에 들어설 때 이미 줄였다


def test_the_turn_is_visible_ahead_and_inside_the_junction(env):
    rows = _drive(env, make_expert(env, version=5))
    turn_speed = _turn_speed(env, rows)
    assert 2.0 < turn_speed < 7.0
    length = max(row[1] for row in rows if row[0] == "in")
    approach = next(row for row in rows if row[0] == "in" and length - row[1] < 60.0)
    assert approach[4][INDEX["speed_limit"]] * VMAX == pytest.approx(VMAX, abs=0.01)
    assert approach[4][INDEX["next_speed_limit"]] * VMAX == pytest.approx(turn_speed, abs=0.01)
    assert approach[4][INDEX["next_speed_limit_distance"]] * 100.0 < 75.0
    for row in rows:
        if row[0].startswith(":"):   # 교차로 안에서도 앞의 가장 낮은 제한이 보인다
            assert row[4][INDEX["next_speed_limit"]] * VMAX <= env.route_map.curve_speed("out_0", 0.0) + 0.01


def test_the_environment_does_not_hold_full_throttle_to_the_curve_speed(env):
    """관측을 보지 않고 늘 가속하는 정책은 곡선 제한을 넘겨 돈다. 환경은 vmax로만 자르고, 제한은 관측에 그대로 보인다."""
    rows = _drive(env, lambda _obs: np.array([1.0, 0.0], dtype=np.float32))
    leaving = next(row for row in rows if row[0] == "out")
    limit_here = env.route_map.curve_speed("out_0", leaving[1])
    assert leaving[2] > limit_here + 3.0
    assert max(row[2] for row in rows) <= VMAX + 1e-6
    assert leaving[4][INDEX["speed_limit"]] * VMAX == pytest.approx(limit_here, abs=0.01)
    assert _lateral_peak(rows) > EGO["comfort"]["lateral_acceleration_limit"]
