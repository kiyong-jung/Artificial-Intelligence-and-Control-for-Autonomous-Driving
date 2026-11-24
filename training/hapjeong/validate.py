"""Fixed validation with stop outcomes and diagnostic traces, no policy assistance."""
import argparse
import gzip
import hashlib
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace
import torch
from contextlib import nullcontext

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from env.experiment import configure_execution, write_run_config
from env.road_config import ROAD, EGO, TRAFFIC
from env.mdp_config import SIMULATION, OBSERVATION, ACTION
from env.road_builder import build
from env.sumo_env import SumoHighwayEnv, traci
from train import HPARAMS, REWARD
from test import load_agent
from training.hapjeong.task import configure_execution,install,check_model_config
install()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('model'); p.add_argument('output'); p.add_argument('protocol')
    p.add_argument('--compact',action='store_true',help='Summary metrics without expensive privileged-state trace')
    args = p.parse_args()
    torch.set_num_threads(1)
    model = Path(args.model).resolve(); out = Path(args.output)
    out.mkdir(exist_ok=False, parents=True)
    config_path=model.parent/'run_config.json'
    if not config_path.exists() and model.parent.name=='checkpoints':
        config_path=model.parent.parent/'run_config.json'
    saved = json.loads(config_path.read_text(encoding='utf8'))
    protocol = json.loads(Path(args.protocol).read_text(encoding='utf8'))
    assert saved['ego']['learning_profile'] == 'bus_service_v1'
    check_model_config(saved)
    HPARAMS.update(saved['hparams'])
    manifest = dict(pid=os.getpid(), model=str(model), sha256=hashlib.sha256(model.read_bytes()).hexdigest(), protocol=protocol)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    rows=[]
    for seed in protocol['validation_seeds']:
        run = out/str(seed); run.mkdir()
        configure_execution(SimpleNamespace(map='hapjeong',scenario='baseline',seed=seed,ego_route='bus_loop',max_steps=1800,learning_profile='bus_service_v1'))
        EGO['task_rewards'] = dict(saved['ego']['task_rewards'])
        write_run_config(run,hparams=HPARAMS,reward=REWARD,kind='fixed_validation')
        cfg=build(ROAD,EGO,TRAFFIC,str(run/'sumo'))
        env=SumoHighwayEnv(cfg,ROAD,EGO,SIMULATION,OBSERVATION,ACTION,REWARD)
        try:
            obs,reset=env.reset(seed=seed)
            agent=load_agent(str(model),93,2)
            (run/'scope_audit.json').write_text(json.dumps(dict(stops=env.bus_task.stops,progress_per_m=env.bus_task.progress_per_m,reference_distance=env.bus_task.reference_distance,observation_dim=len(obs))))
            begin=traci.simulation.getTime(); ret=0.; speeds=[]; stopped=0; streak=0; longest=0
            start_distance=env.bus_task.pose()['distance']; max_distance=start_distance
            with (nullcontext(None) if args.compact else gzip.open(run/'trace.jsonl.gz','wt',encoding='utf8')) as f:
                for step in range(2400):
                    before=obs.copy(); action=agent.predict(obs,deterministic=True)
                    obs,reward,term,trunc,info=env.step(action); ret+=reward
                    pose=env.bus_task.pose()
                    if pose is not None: max_distance=max(max_distance,pose['distance'])
                    speed=info.get('speed')
                    if speed is not None:
                        speeds.append(float(speed)); stopped+=int(speed<=.1)
                        streak=streak+1 if speed<=.1 else 0; longest=max(longest,streak)
                    if f is not None:
                        f.write(json.dumps(dict(step=step,observation=before.tolist(),action=action.tolist(),reward=reward,
                                               state=env.get_privileged_state(),reason=info.get('termination_reason')))+'\n')
                    if term or trunc:break
            row=dict(seed=seed,steps=step+1,seconds=traci.simulation.getTime()-begin,
                     reason=info.get('termination_reason'),success=bool(info.get('arrived')),
                     route_arrived=bool(info.get('route_arrived')),collision=bool(info.get('collided')),
                     red_violation=bool(info.get('red_violation')),service=info['bus_service'],
                     timeout=bool(trunc),return_sum=ret,mean_speed=sum(speeds)/max(1,len(speeds)),
                     observed_travel_m=max_distance-start_distance,max_speed=max(speeds,default=0.),
                     stopped_fraction=stopped/max(1,len(speeds)),longest_stopped_seconds=longest*.5)
        finally:env.close()
        rows.append(row)
        (run/'result.json').write_text(json.dumps(row,indent=2),encoding='utf8')
        summary=dict(complete=len(rows)==len(protocol['validation_seeds']),runs=rows,
                     success=sum(x['success'] for x in rows),collisions=sum(x['collision'] for x in rows),
                     red_violations=sum(x['red_violation'] for x in rows),
                     completed_stops=sum(len(x['service']['completed_stops']) for x in rows))
        (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
        print(json.dumps(row),flush=True)


if __name__=='__main__':main()
