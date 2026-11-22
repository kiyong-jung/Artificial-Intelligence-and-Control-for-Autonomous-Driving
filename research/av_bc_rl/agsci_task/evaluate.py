"""AI-Scientist-v3 evaluator for the Hapjeong bus task. Writes metrics.json and correct.json.

    python evaluate.py --program_path program_0/src/program.py --results_dir /tmp/eval

The Hapjeong repository is found through HAPJEONG_REPO, or three folders above this file
(research/av_bc_rl/agsci_task/). SUMO and the fixed seeds make the score deterministic.
"""
import argparse
import importlib.util
import json
import os
import sys
import traceback

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.environ.get("HAPJEONG_REPO") or os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SCENARIO, FIRST_SEED, EPISODES, MAX_STEPS = "busy", 1600, 12, 1400
HARD_BRAKE = 3.0   # m/s²


def load_program(path):
    sys.path.insert(0, os.path.dirname(os.path.abspath(path)))
    spec = importlib.util.spec_from_file_location("candidate_program", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.make_policy


def run_episode(env, act, seed):
    obs, _ = env.reset(seed=seed)
    speed = None
    hard = interventions = steps = 0
    info = {}
    done = False
    while not done:
        action = np.clip(np.asarray(act(obs), dtype=np.float32).reshape(-1)[:2], -1.0, 1.0)
        obs, _reward, terminated, truncated, info = env.step(action)
        steps += 1
        interventions += bool(info.get("speed_intervened"))
        new = info.get("speed")
        if speed is not None and new is not None and (speed - new) / env.decision_interval > HARD_BRAKE:
            hard += 1
        speed = new if new is not None else speed
        done = terminated or truncated
    reason = info.get("termination_reason")
    return {"seed": seed, "steps": steps, "termination": reason, "arrived": reason == "arrived",
            "lap_time_s": steps * env.decision_interval if reason == "arrived" else None,
            "hard_brakes": hard, "intervention_rate": interventions / max(steps, 1),
            "stops_served": int(info.get("stops_served", 0)), "stops_missed": int(info.get("stops_missed", 0))}


def score(episodes):
    n = len(episodes)
    arrived = [e for e in episodes if e["arrived"]]
    completion = len(arrived) / n
    crash = sum(e["termination"] in ("collision", "teleport") for e in episodes) / n
    hard = float(np.mean([e["hard_brakes"] for e in episodes]))
    intervention = float(np.mean([e["intervention_rate"] for e in episodes]))
    missed = float(np.mean([e["stops_missed"] + (4 - e["stops_served"] - e["stops_missed"]) for e in episodes]))
    lap = float(np.mean([e["lap_time_s"] for e in arrived])) if arrived else 700.0
    combined = (100 * completion - 50 * crash - 10 * hard - 100 * intervention - 10 * missed
                - 0.05 * max(0.0, lap - 350.0))
    public = {"completion": completion, "collision_or_teleport_rate": crash, "hard_brakes_per_lap": hard,
              "intervention_rate": intervention, "missed_stops_per_lap": missed, "lap_time_s": lap}
    return combined, public, completion >= 0.9 and crash == 0.0


def main(program_path, results_dir):
    os.makedirs(results_dir, exist_ok=True)
    sys.path.insert(0, REPO)
    metrics, correct, message = {"combined_score": 0.0}, False, None
    try:
        from env.road_config import ROAD, EGO, TRAFFIC, configure_map
        from env.mdp_config import SIMULATION, OBSERVATION, ACTION, REWARD
        from env import road_builder
        from env.sumo_env import SumoHighwayEnv
        configure_map(SCENARIO, FIRST_SEED, None, "hapjeong")
        SIMULATION["max_steps"] = MAX_STEPS
        cfg = road_builder.build(ROAD, EGO, TRAFFIC, os.path.join(results_dir, "sumo"))
        env = SumoHighwayEnv(cfg_path=cfg, road=ROAD, ego=EGO, mdp_sim=SIMULATION, mdp_obs=OBSERVATION,
                             action=ACTION, reward=REWARD, gui=False)
        try:
            act = load_program(program_path)(env)
            episodes = [run_episode(env, act, FIRST_SEED + i) for i in range(EPISODES)]
        finally:
            env.close()
        combined, public, correct = score(episodes)
        metrics = {"combined_score": combined, "public": public, "private": {"episodes": episodes}}
        message = "ok" if correct else "collision/teleport or completion below 90%"
    except Exception as exc:   # 후보 프로그램의 오류는 점수 0과 메시지로 남긴다
        message = f"{type(exc).__name__}: {exc}"
        metrics = {"combined_score": 0.0, "error": message, "traceback": traceback.format_exc()}
    with open(os.path.join(results_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    with open(os.path.join(results_dir, "correct.json"), "w") as f:
        json.dump({"correct": correct, "message": message}, f, indent=2)
    print(json.dumps({"combined_score": metrics["combined_score"], "correct": correct,
                      **metrics.get("public", {})}, indent=1))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hapjeong bus evaluator (metrics.json + correct.json)")
    parser.add_argument("--program_path", default=os.path.join(HERE, "program_0", "src", "program.py"))
    parser.add_argument("--results_dir", default="results")
    args = parser.parse_args()
    main(args.program_path, args.results_dir)
