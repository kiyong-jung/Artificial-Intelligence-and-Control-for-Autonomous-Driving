# Goal
- Improve the driving policy of an autonomous city bus on the Hapjeong (Seoul) loop in SUMO.
- The bus must complete the 1,949 m loop, serve 4 roadside stops for 8 s each, and drive smoothly and lawfully.
- The SUMO safety assist (speedMode 31) stays on as a last line of defense, but a good policy should almost never need it.

# What you may change
- Only `src/program.py` (and new files under `src/`). It must define `make_policy(env)` that returns a function
  `act(observation) -> np.ndarray([accel_raw, lane_change_raw], float32)`, both in [-1, 1].
  `accel_raw` maps to acceleration (+1 = 1.0 m/s², -1 = -2.0 m/s²); `lane_change_raw` is quantized to
  {-1 right, 0 keep, +1 left} at |x| >= 0.5.
- `env` is `env.sumo_env.SumoHighwayEnv` of the Hapjeong repository (`HAPJEONG_REPO`). You may read its attributes
  (`vmax`, `visibility`, `max_accel`, `max_decel`) and the observation layout in `env/data_schema.py`
  (`OBSERVATION_FEATURES`, 46 values: 31 course features + 15 route features: traffic light, stop, route lane,
  crosswalk, speed limits, acceleration).
- You may load trained weights you produce yourself (BC, PPO, residual RL) from files under `src/`.

# What you may not do
- Do not read SUMO/TraCI state directly in the policy; act only on the observation (the policy must be
  deployable from the bus's own sensors and map).
- Do not change the environment, the road network, the traffic, or the evaluator.

# Evaluation
`evaluate.py` drives the policy closed-loop for 12 episodes (busy traffic, fixed dev seeds 1600–1611, 700 s limit)
and writes `metrics.json` and `correct.json`.

combined_score = 100·completion − 50·(collision or teleport rate) − 10·(hard brakes > 3 m/s² per lap)
                 − 100·(SUMO speed-intervention rate) − 10·(missed stops per lap) − 0.05·max(0, lap time − 350 s)

`correct` is true when no episode ends in a collision or teleport and completion ≥ 90%.

Reference values measured with this evaluator on a 14-core Apple M4 Pro (about 3 minutes per evaluation):
- rule expert `ExpertPolicy` v4 (`program_0`): 96.9, correct (intervention 1.1%, 0 hard brakes, 0 missed stops,
  lap 390 s);
- the old constant-acceleration rule (`route_following_action`): -1.5, correct (2.6 hard brakes per lap, 32%
  interventions, misses all 4 stops, lap 420 s). SUMO stops it hard at red lights because it cannot see them.
