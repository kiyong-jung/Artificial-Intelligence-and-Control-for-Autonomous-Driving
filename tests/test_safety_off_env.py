"""SUMO 안전 보조를 끈 환경(수업 원본의 speedMode 0, laneChangeMode 0)의 행동을 실제 SUMO로 본다.

적색 통과와 보행자 충돌은 SUMO가 막아 주지 않고, 환경이 찾아 에피소드를 끝낸다. 전문가 규칙은 스스로 선다.
"""
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
from env.sumo_env import CROSSING_RANGE, SumoHighwayEnv  # noqa: E402

VMAX = 13.89
INDEX = {name: index for index, name in enumerate(OBSERVATION_FEATURES)}
FULL_THROTTLE = np.array([1.0, 0.0], dtype=np.float32)
BRAKE = np.array([-1.0, 0.0], dtype=np.float32)
WALKER_SPEED = 0.2


@pytest.fixture(scope="module")
def env(tmp_path_factory):
    """두 차로 직선 도로가 신호 교차로를 지난다. 교차로를 나가는 쪽에 횡단보도가 있다."""
    d = tmp_path_factory.mktemp("crossroad")
    (d / "n.nod.xml").write_text('<nodes><node id="W" x="-300" y="0"/><node id="C" x="0" y="0" type="traffic_light"/>'
                                 '<node id="E" x="200" y="0"/><node id="S" x="0" y="-100"/><node id="N" x="0" y="100"/></nodes>')
    (d / "e.edg.xml").write_text('<edges><edge id="in" from="W" to="C" numLanes="2" speed="13.89"/>'
                                 '<edge id="out" from="C" to="E" numLanes="2" speed="13.89"/>'
                                 '<edge id="south" from="S" to="C" numLanes="1" speed="13.89"/>'
                                 '<edge id="north" from="C" to="N" numLanes="1" speed="13.89"/></edges>')
    subprocess.run([checkBinary("netconvert"), "-n", str(d / "n.nod.xml"), "-e", str(d / "e.edg.xml"),
                    "--sidewalks.guess", "true", "--sidewalks.guess.min-speed", "1", "--crossings.guess", "true",
                    "-o", str(d / "network.net.xml"), "--no-warnings"], check=True, capture_output=True)
    (d / "r.rou.xml").write_text(
        '<routes><vType id="ego_type" vClass="bus" length="12" width="2.5" accel="1.0" decel="2.0" minGap="3" tau="1.5" '
        'laneChangeModel="SL2015" maxSpeedLat="1.0"/>'
        '<vType id="car" vClass="passenger" laneChangeModel="SL2015" maxSpeedLat="1.0"/>'
        f'<vType id="slow_walker" vClass="pedestrian" maxSpeed="{WALKER_SPEED}"/><route id="r0" edges="in out"/></routes>')
    (d / "scenario.sumocfg").write_text(
        '<configuration><input><net-file value="network.net.xml"/><route-files value="r.rou.xml"/></input>'
        '<processing><lateral-resolution value="0.8"/><collision.check-junctions value="true"/>'
        '<collision.action value="remove"/><collision.stoptime value="120"/>'
        '<time-to-teleport value="-1"/></processing><time><step-length value="0.1"/></time></configuration>')
    configure_map("quiet", 0, None, "hapjeong")
    road = {"seed": 0, "speed_limit": VMAX, "network": {"routes": {"r0": ["in", "out"]}, "bus_stops": []}}
    ego = {**EGO, "route_id": "r0", "service": {"stops": (), "dwell_s": 0.0}}
    simulation = {**SIMULATION, "warmup_steps": 0, "warmup_jitter_steps": 0, "max_steps": 600}
    if traci.isLoaded():
        traci.close()
    environment = SumoHighwayEnv(str(d / "scenario.sumocfg"), road, ego, simulation, OBSERVATION, ACTION, REWARD)
    yield environment
    environment.close()


def _set_lights(letter):
    tls = traci.trafficlight.getIDList()[0]
    traci.trafficlight.setRedYellowGreenState(tls, letter * len(traci.trafficlight.getRedYellowGreenState(tls)))


def _run(env, obs, policy, steps):
    """끝나거나 steps번 판단할 때까지 달린다. (마지막 관측, 보상, 끝났는지, info, 판단마다의 속도)를 돌려준다."""
    speeds, reward, done, info = [], 0.0, False, {}
    for _ in range(steps):
        obs, reward, terminated, truncated, info = env.step(policy(obs))
        speeds.append(float(obs[INDEX["ego_speed"]]) * VMAX)
        done = terminated or truncated
        if done:
            break
    return obs, reward, done, info, speeds


def _walker_onto_the_crossing():
    """느린 보행자를 횡단보도 앞 인도에 놓는다. 횡단보도는 out 도로를 가로지른다."""
    traci.person.add("walker", "south", traci.lane.getLength("south_0") - 1.0, typeID="slow_walker")
    traci.person.appendWalkingStage("walker", ["south", "north"], 50.0)


def test_ego_has_no_sumo_assistance(env):
    env.reset(seed=0)
    assert traci.vehicle.getSpeedMode("ego") == 0
    assert traci.vehicle.getLaneChangeMode("ego") == 0


def test_running_a_red_light_ends_the_episode(env):
    obs, _info = env.reset(seed=0)
    _set_lights("r")
    obs, reward, done, info, speeds = _run(env, obs, lambda _obs: FULL_THROTTLE, 200)
    assert done and info["termination_reason"] == "red_light"
    assert info["red_light_violation"] and not info["collided"] and not info["arrived"]
    assert reward == -REWARD["red_light_penalty"]
    assert speeds[-2] > 10.0                      # SUMO는 세우지 않았다
    assert traci.vehicle.getRoadID("ego").startswith(":C")


def test_expert_waits_at_red_and_goes_on_green(env):
    obs, _info = env.reset(seed=0)
    _set_lights("r")
    expert = make_expert(env, version=5)
    obs, _reward, done, info, speeds = _run(env, obs, expert, 160)
    assert not done and speeds[-1] < 0.05
    assert traci.vehicle.getRoadID("ego") == "in"
    assert 0.0 < obs[INDEX["tl_distance"]] * 100.0 < 4.0     # 정지선 앞에 섰다
    _set_lights("G")
    obs, _reward, done, info, _speeds = _run(env, obs, expert, 300)
    assert done and info["termination_reason"] == "arrived" and not info["red_light_violation"]


def test_a_green_light_is_not_a_violation(env):
    obs, _info = env.reset(seed=0)
    _set_lights("G")
    _obs, _reward, done, info, _speeds = _run(env, obs, lambda _obs: FULL_THROTTLE, 300)
    assert done and info["termination_reason"] == "arrived" and not info["red_light_violation"]


def test_hitting_a_pedestrian_ends_the_episode(env):
    """버스가 횡단보도 앞에 서 있다가, 보행자가 버스 앞길에 들어올 때 닿도록 맞춰 최대 가속으로 지나간다."""
    obs, _info = env.reset(seed=0)
    _set_lights("G")
    _walker_onto_the_crossing()
    launched = False
    for _ in range(600):
        distance = float(obs[INDEX["crosswalk_distance"]]) * CROSSING_RANGE
        speed = float(obs[INDEX["ego_speed"]]) * VMAX
        if not launched and distance < 40.0 and speed < 0.05 and traci.person.getRoadID("walker").startswith(":C_c"):
            lateral = abs(traci.person.getPosition("walker")[1] - traci.vehicle.getPosition("ego")[1])
            launched = lateral <= WALKER_SPEED * math.sqrt(2.0 * distance / EGO["accel"])
        action = FULL_THROTTLE if launched or distance > 60.0 else BRAKE
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break
    assert launched and info["termination_reason"] == "pedestrian_collision"
    assert info["collided"] and info["pedestrian_collision"] and not info["red_light_violation"]
    assert reward == -REWARD["collision_penalty"]


def test_expert_waits_for_a_pedestrian_on_the_crossing(env):
    obs, _info = env.reset(seed=0)
    _set_lights("G")
    _walker_onto_the_crossing()
    expert = make_expert(env, version=5)
    waited = on_crossing = False
    for _ in range(600):
        on_crossing = on_crossing or traci.person.getRoadID("walker").startswith(":C_c")
        occupied = obs[INDEX["crosswalk_occupied"]] > 0.5
        waited = waited or (occupied and obs[INDEX["ego_speed"]] * VMAX < 0.5
                            and 1.0 < obs[INDEX["crosswalk_distance"]] * CROSSING_RANGE < 10.0)
        # 보행자가 횡단보도에 들어선 뒤에 출발한다. 다가오는 차가 있으면 보행자가 들어서지 않고 기다린다.
        obs, _reward, terminated, truncated, info = env.step(expert(obs) if on_crossing else BRAKE)
        if terminated or truncated:
            break
    assert waited and info["termination_reason"] == "arrived" and not info["collided"]


def _stopped_car_ahead(metres):
    lane = traci.vehicle.getLaneIndex("ego")
    traci.vehicle.add("blocker", "r0", typeID="car", departLane=str(lane), departPos=str(metres), departSpeed="0")
    traci.vehicle.setSpeedMode("blocker", 0)
    traci.vehicle.setSpeed("blocker", 0.0)


def test_running_into_a_stopped_car_ends_the_episode(env):
    obs, _info = env.reset(seed=0)
    _stopped_car_ahead(150.0)
    _obs, reward, done, info, speeds = _run(env, obs, lambda _obs: FULL_THROTTLE, 200)
    assert done and info["termination_reason"] == "collision"
    assert info["collided"] and not info["pedestrian_collision"]
    assert reward == -REWARD["collision_penalty"]
    assert speeds[-2] > 10.0                      # SUMO는 세우지 않았다


def test_expert_stops_behind_a_stopped_car(env):
    obs, _info = env.reset(seed=0)
    _stopped_car_ahead(150.0)
    # 옆 차로가 비어 있어도 앞지르지 않는다. 이 정책은 경로와 정류장 때문에만 차로를 바꾼다.
    obs, _reward, done, _info, speeds = _run(env, obs, make_expert(env, version=5), 120)
    assert not done and speeds[-1] < 0.05 and max(speeds) > 8.0
    gap = obs[INDEX["current_leader_gap"]] * OBSERVATION["visibility"]
    assert 0.5 < gap < 6.0


def test_a_lane_change_ends_in_the_lane_centre(env):
    """laneChangeMode 0에서 SUMO는 차선변경을 차로 가운데 0.35m 앞에서 끝낸다. 환경이 가운데까지 옮긴다."""
    obs, _info = env.reset(seed=0)
    _set_lights("G")
    for _ in range(40):
        obs, *_ = env.step(np.array([0.5, 0.0], dtype=np.float32))
    start = traci.vehicle.getLaneIndex("ego")
    direction = 1.0 if obs[INDEX["left1_route_connectivity"]] >= 0.0 else -1.0
    applied = [env.step(np.array([0.0, direction], dtype=np.float32))[4]["lane_change_applied"] for _ in range(2)]
    for _ in range(14):
        env.step(np.array([0.0, 0.0], dtype=np.float32))
    assert all(applied)
    assert traci.vehicle.getLaneIndex("ego") == start + int(direction)
    assert abs(traci.vehicle.getLateralLanePosition("ego")) < 0.02


def test_a_lane_change_is_undone_when_a_car_comes_alongside(env):
    """게이트를 지나 옮기기 시작한 뒤, 옆 차로 뒤차가 줄이지 않고 다가온다. 차로 경계를 넘는 데 1.6초가 걸리므로 환경은
    옮기는 동안 게이트를 계속 보고, 깨지면 제 차로 가운데로 되돌린다. 되돌리지 않으면 그 차 바로 앞으로 들어가 부딪힌다."""
    obs, _info = env.reset(seed=0)
    _set_lights("G")
    for _ in range(50):
        obs, *_ = env.step(np.array([0.3, 0.0], dtype=np.float32))
    start = traci.vehicle.getLaneIndex("ego")
    direction = 1 if obs[INDEX["left1_route_connectivity"]] >= 0.0 else -1
    behind = traci.vehicle.getLanePosition("ego") - traci.vehicle.getLength("ego") - 14.0
    traci.vehicle.add("chaser", "r0", typeID="car", departLane=str(start + direction), departPos=str(behind),
                      departSpeed="13.8")
    traci.vehicle.setSpeedMode("chaser", 0)
    traci.vehicle.setLaneChangeMode("chaser", 0)
    traci.vehicle.setSpeed("chaser", 13.8)
    infos = []
    for step in range(16):
        command = float(direction) if step < 2 else 0.0
        obs, _reward, terminated, _truncated, info = env.step(np.array([0.0, command], dtype=np.float32))
        infos.append(info)
        assert not terminated, info["termination_reason"]
    assert infos[0]["lane_change_applied"]                           # 명령 때는 간격이 충분했다
    assert any(info["lane_change_aborted"] for info in infos)
    assert traci.vehicle.getLaneIndex("ego") == start
    assert abs(traci.vehicle.getLateralLanePosition("ego")) < 0.02
    assert traci.vehicle.getLanePosition("chaser") > traci.vehicle.getLanePosition("ego")   # 그 차는 지나갔다


def test_a_car_drifting_into_the_bus_lane_is_seen_as_the_leader(env):
    """옆 차로에 속한 채 버스 앞길로 1.3m 넘어온 차는 SUMO의 getLeader에 없다. 관측의 현재 차로 앞차로 잡는다."""
    obs, _info = env.reset(seed=0)
    ego_lane = traci.vehicle.getLaneIndex("ego")
    side = ego_lane + (1 if obs[INDEX["left1_route_connectivity"]] >= 0.0 else -1)
    traci.vehicle.add("drifter", "r0", typeID="car", departLane=str(side), departPos="45", departSpeed="0")
    traci.vehicle.setSpeedMode("drifter", 0)
    traci.vehicle.setLaneChangeMode("drifter", 0)
    traci.vehicle.setSpeed("drifter", 1.0)
    obs, *_ = env.step(BRAKE)
    assert obs[INDEX["current_leader_gap"]] == 1.0
    toward_bus = -1.3 if side > ego_lane else 1.3
    for _ in range(8):
        traci.vehicle.changeSublane("drifter", toward_bus - traci.vehicle.getLateralLanePosition("drifter"))
        obs, *_ = env.step(BRAKE)
    assert traci.vehicle.getLaneIndex("drifter") == side          # 여전히 옆 차로 소속이다
    side_name = "left1" if side > ego_lane else "right1"
    assert obs[INDEX["current_leader_gap"]] < 1.0
    assert obs[INDEX["current_leader_gap"]] == pytest.approx(obs[INDEX[f"{side_name}_leader_gap"]], abs=0.02)
