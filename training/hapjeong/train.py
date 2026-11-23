"""Four explicit training methods on the team's SUMO environment and school PPO."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch
from algorithms.ppo import PPO
from env.road_config import ROAD, EGO, TRAFFIC
from env.mdp_config import SIMULATION, OBSERVATION, ACTION
from env.road_builder import build
from env.sumo_env import SumoHighwayEnv
from env.experiment import write_run_config
from train import REWARD
from training.hapjeong.task import install, configure_execution, check_model_config
from training.hapjeong.residual_env import ResidualEnv, initialize_zero

PRESETS = json.loads(Path(__file__).with_name('presets.json').read_text(encoding='utf8'))

class Logger:
    def __init__(self, out):
        self.out, self.run_dir = out, str(out)

    def log_episode(self, steps, ret, length):
        with (self.out / 'episodes.jsonl').open('a', encoding='utf8') as f:
            f.write(json.dumps(dict(steps=steps, return_sum=ret, length=length)) + '\n')

    def log_train(self, steps, metrics, verbose=False):
        with (self.out / 'training.jsonl').open('a', encoding='utf8') as f:
            f.write(json.dumps(dict(steps=steps, **metrics)) + '\n')
        (self.out / 'progress.json').write_text(json.dumps(dict(phase='training', steps=steps)), encoding='utf8')

def train_stage(method, preset, out, base_path=None, steps=None):
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    hp = dict(preset['hparams'])
    steps = preset['steps'] if steps is None else steps
    if steps < 1:
        raise ValueError('steps must be positive')
    if steps == 64:
        hp.update(n_steps=64, minibatch_size=64)
    configure_execution(SimpleNamespace(map='hapjeong', scenario='baseline', seed=hp['seed'],
        ego_route='bus_loop', max_steps=2400, learning_profile='bus_service_v1'))
    EGO['task_rewards'] = dict(preset['reward'])
    base = None
    manifest = dict(method=method, initialization='random', optimizer='fresh; weights-only loading is not exact resume')
    if base_path is not None:
        base_path = Path(base_path).resolve()
        config = json.loads(base_path.with_name('run_config.json').read_text(encoding='utf8'))
        check_model_config(config)
        if config.get('kind') == 'frozen_bc_residual_ppo':
            raise ValueError('Base must be a 93D BC/PPO policy, not a residual policy')
        base = PPO(93, 2, **config['hparams'])
        base.load(str(base_path))
        manifest.update(initialization='saved weights', base_sha256=hashlib.sha256(base_path.read_bytes()).hexdigest())
        shutil.copy2(base_path, out / 'base_model.pt')
        shutil.copy2(base_path.with_name('run_config.json'), out / 'base_run_config.json')
    if method == 'ppo' and base is not None:
        raise ValueError('Pure PPO must not load a checkpoint')
    if method != 'ppo' and base is None:
        raise ValueError('This method requires --base')
    agent = PPO(95 if method == 'residual' else 93, 2, **hp)
    if method == 'residual':
        initialize_zero(agent)
        manifest.update(obs_dim=95, acceleration_scale=.5, actor_base_frozen=True,
            lane_mapping='cyclic category offset; not a spatial lane direction')
        (out / 'residual_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf8')
    elif base is not None:
        agent.load(str(base_path))
        assert all(torch.equal(v, agent.policy.state_dict()[k]) for k, v in base.policy.state_dict().items())
    initial = {k: v.clone() for k, v in agent.policy.state_dict().items()}
    kind = 'frozen_bc_residual_ppo' if method == 'residual' else method
    write_run_config(out, hparams=hp, reward=REWARD, kind=kind)
    (out / 'initialization.json').write_text(json.dumps(manifest, indent=2), encoding='utf8')
    for path in ['training/hapjeong/train.py', 'training/hapjeong/task.py', 'training/hapjeong/presets.json',
                 'training/hapjeong/residual_env.py', 'algorithms/ppo.py', 'utils/networks.py', 'env/bus_task.py']:
        target = out / 'source' / path
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / path, target)
    cfg = build(ROAD, EGO, TRAFFIC, str(out / 'sumo'))
    env = SumoHighwayEnv(cfg, ROAD, EGO, SIMULATION, OBSERVATION, ACTION, REWARD)
    if method == 'residual':
        env = ResidualEnv(env, base, scale=.5)
    try:
        agent.learn(env, total_timesteps=steps, eval_interval=0, logger=Logger(out))
    finally:
        env.close()
    agent.save(str(out / 'model.pt'))
    restored = PPO(95 if method == 'residual' else 93, 2, **hp)
    restored.load(str(out / 'model.pt'))
    rng = np.random.default_rng(1234)
    for obs in rng.uniform(-1, 1, (32, 95 if method == 'residual' else 93)).astype(np.float32):
        assert np.array_equal(agent.predict(obs, True), restored.predict(obs, True))
    assert all(torch.isfinite(v).all() for v in agent.policy.state_dict().values())
    assert any(not torch.equal(v, initial[k]) for k, v in agent.policy.state_dict().items())
    if base is not None:
        assert hashlib.sha256(base_path.read_bytes()).hexdigest() == manifest['base_sha256']
    if method == 'residual':
        # Frozen base is also checked through its output parity after rollout.
        check = PPO(93, 2, **config['hparams']); check.load(str(out / 'base_model.pt'))
        for obs in rng.uniform(-1, 1, (32, 93)).astype(np.float32):
            assert np.array_equal(base.predict(obs, True), check.predict(obs, True))
    (out / 'audit.json').write_text(json.dumps(dict(passed=True, steps=agent.num_timesteps,
        finite=True, updated=True, save_load_parity=True)), encoding='utf8')
    (out / 'progress.json').write_text(json.dumps(dict(phase='complete', steps=agent.num_timesteps)), encoding='utf8')
    return out / 'model.pt'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('method', choices=['bc', 'bc-ppo', 'ppo', 'residual'])
    parser.add_argument('--out', required=True)
    parser.add_argument('--base', help='93D BC model; required for bc-ppo and residual')
    parser.add_argument('--steps', type=int, help='Override each PPO stage budget; 64 for smoke checks')
    parser.add_argument('--data'); parser.add_argument('--split-file'); parser.add_argument('--config')
    args = parser.parse_args()
    if args.method == 'ppo' and args.base:
        parser.error('ppo is random initialization; use bc-ppo to load a base')
    if args.method in ('bc-ppo', 'residual') and not args.base:
        parser.error('--base is required')
    if args.method == 'bc':
        if not all((args.data, args.split_file, args.config)):
            parser.error('bc requires --data, --split-file and --config')
        if args.base or args.steps:
            parser.error('BC uses supervised epochs, not PPO steps or a base checkpoint')
        command = [sys.executable, str(ROOT / 'scripts/train_bus_hybrid_bc.py'), '--data', args.data,
            '--out', args.out, '--split-file', args.split_file, '--config', args.config,
            '--epochs', '100', '--brake-weight', '4', '--valid-key', 'action_valid']
        subprocess.run(command, check=True)
        return
    torch.set_num_threads(1)
    install()
    if args.method == 'bc-ppo':
        out = Path(args.out).resolve(); out.mkdir(parents=True, exist_ok=False)
        first = train_stage('bc-ppo', PRESETS['bc_ppo_initial'], out / 'initial', args.base, args.steps)
        train_stage('bc-ppo', PRESETS['bc_ppo_refine'], out / 'refine', first, args.steps)
    else:
        train_stage(args.method, PRESETS[args.method], args.out, args.base, args.steps)

if __name__ == '__main__':
    main()
