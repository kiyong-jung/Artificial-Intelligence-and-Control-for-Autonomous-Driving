"""BC on school's existing HybridActorCritic; directly loadable by school PPO.

Uses school train_bc loader. Actor loss: acceleration MSE plus lane CE.
Critic and exploration std are not supervised. No rule used at inference.
"""
import sys,json,argparse,copy,shutil
from pathlib import Path
import numpy as np
import torch
import torch.nn.functional as F
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from train_bc import load_data
from algorithms.ppo import PPO

def masked_lane_loss(logits, classes, weights, mask):
 if not mask.any():return logits.sum()*0.0
 if weights[classes[mask]].sum()<=0:return logits.sum()*0.0
 return F.cross_entropy(logits[mask],classes[mask],weight=weights)

def main():
 p=argparse.ArgumentParser();p.add_argument('--data',required=True);p.add_argument('--out',required=True);p.add_argument('--epochs',type=int,default=100);p.add_argument('--brake-weight',type=float,default=1.0);p.add_argument('--action-key',default='action_raw');p.add_argument('--valid-key');p.add_argument('--split-file');p.add_argument('--config');p.add_argument('--lane-mask-key');a=p.parse_args()
 assert np.isfinite(a.brake_weight) and a.brake_weight>=1
 torch.set_num_threads(1);torch.manual_seed(38001);rng=np.random.default_rng(38001)
 out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
 data=Path(a.data);x,y,ep=load_data(data,action_key=a.action_key,valid_key=a.valid_key)
 if x.shape[1] != 93 or y.shape[1] != 2:
  raise ValueError('Requires 93D bus observations and 2D actions')
 if not np.isin(y[:,1],[-1,0,1]).all() or (np.abs(y[:,0])>1).any():
  raise ValueError('Requires executed normalized acceleration and discrete lane commands')
 with np.load(data) as raw:
  if not a.valid_key:assert raw['action_valid'].all()
  lane_mask=np.ones(len(x),bool)
  if a.lane_mask_key:
   lm=raw[a.lane_mask_key];assert lm.dtype==np.bool_ and lm.shape==(len(raw['observation']),)
   lane_mask=lm[raw[a.valid_key]] if a.valid_key else lm
 ids=np.unique(ep);assert len(ids)>=20
 ids=rng.permutation(ids);n=len(ids);ntrain=int(n*.7);nval=int(n*.15)
 split={k:sorted(v.tolist()) for k,v in [('train',ids[:ntrain]),('validation',ids[ntrain:ntrain+nval]),('test',ids[ntrain+nval:])]}
 if a.split_file:
  split=json.loads(Path(a.split_file).read_text(encoding='utf8'))['split']
  assert sorted(sum(split.values(),[]))==sorted(np.unique(ep).tolist())
 idx={k:np.flatnonzero(np.isin(ep,v)) for k,v in split.items()}
 if any(len(v)==0 for v in idx.values()):
  raise ValueError('Every episode split must contain valid samples')
 config_path=Path(a.config) if a.config else data.parent/str(json.loads((data.parent/'protocol.json').read_text())['seeds'][0])/'run_config.json'
 config=json.loads(config_path.read_text(encoding='utf8'))
 from training.hapjeong.task import check_model_config
 check_model_config(config)
 hp=config['hparams'];hp['seed']=38001;hp['device']='cpu'
 agent=PPO(93,2,**hp);net=agent.policy
 xx=torch.tensor(x);yy=torch.tensor(y);classes=(1-yy[:,1]).long()
 lane_mask=torch.tensor(lane_mask)
 counts=torch.bincount(classes[idx['train']][lane_mask[idx['train']]],minlength=3).float()
 assert counts.sum()>0
 weights=torch.where(counts>0,counts.sum()/(3*counts.clamp_min(1)),torch.zeros_like(counts));weights=weights.clamp(max=20)
 optimizer=torch.optim.Adam([v for k,v in net.named_parameters() if not k.startswith('v_head') and k!='log_std'],lr=3e-4)
 best=float('inf');history=[]
 def loss(ii):
  mu,_,logits,_=net(xx[ii]);err=(mu[:,0]-yy[ii,0]).square()
  brake_weights=torch.where(yy[ii,0]<-.05,a.brake_weight,1.0)
  return (err*brake_weights).sum()/brake_weights.sum()+masked_lane_loss(logits,classes[ii],weights,lane_mask[ii])
 for epoch in range(a.epochs):
  net.train()
  for ii in np.array_split(rng.permutation(idx['train']),max(1,int(np.ceil(len(idx['train'])/256)))):
   optimizer.zero_grad();l=loss(ii);l.backward();torch.nn.utils.clip_grad_norm_(net.parameters(),.5);optimizer.step()
  net.eval()
  with torch.no_grad():vl=float(loss(idx['validation']));tl=float(loss(idx['train']))
  assert np.isfinite(vl) and np.isfinite(tl)
  if vl<best:best=vl;agent.save(str(out/'model.pt'));best_epoch=epoch+1
  history.append(dict(epoch=epoch+1,train_loss=tl,val_loss=vl))
  if (epoch+1)%10==0:print(history[-1],flush=True)
 config['kind']='hybrid_bc_pretraining';config['hparams']=hp
 (out/'run_config.json').write_text(json.dumps(config,indent=2),encoding='utf8')
 agent.load(str(out/'model.pt'))
 restored=PPO(93,2,**hp);restored.load(str(out/'model.pt'))
 with torch.no_grad():assert torch.equal(agent.policy.det_action(xx[:32]),restored.policy.det_action(xx[:32]))
 report=dict(split=split,action_key=a.action_key,valid_key=a.valid_key,lane_mask_key=a.lane_mask_key,selected_epoch=best_epoch,history=history,lane_counts_train=counts.tolist(),lane_weights=weights.tolist(),algorithm='BC on school PPO hybrid actor; critic/std unchanged',exact_ppo_load_parity=True,test_split_used_for_selection=False)
 (out/'bc_audit.json').write_text(json.dumps(report,indent=2),encoding='utf8');shutil.copy2(__file__,out/Path(__file__).name)
 (out/'loss_config.json').write_text(json.dumps(dict(brake_weight=a.brake_weight,brake_threshold=-.05,normalization='weighted acceleration MSE divided by sum of weights; unchanged lane CE',selection='minimum validation same weighted objective; test unused')),encoding='utf8')
 print('BC complete, exact PPO loading verified',flush=True)

if __name__=='__main__':main()
