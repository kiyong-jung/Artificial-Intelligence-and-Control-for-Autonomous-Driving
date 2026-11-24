import sys,json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from training.hapjeong import validate
from training.hapjeong.residual_env import ResidualEnv
from algorithms.ppo import PPO
class CombinedPolicy:
 def __init__(self,path):
  path=Path(path);m=json.loads(path.with_name('residual_manifest.json').read_text(encoding='utf8'));c=json.loads(path.with_name('run_config.json').read_text(encoding='utf8'))
  bp=path.parent/'base_model.pt';assert hashlib.sha256(bp.read_bytes()).hexdigest()==m['base_sha256']
  bc=json.loads(path.with_name('base_run_config.json').read_text(encoding='utf8'));self.base=PPO(93,2,**bc['hparams']);self.base.load(str(bp));self.base.policy.eval()
  for p in self.base.policy.parameters():p.requires_grad_(False)
  self.res=PPO(95,2,**c['hparams']);self.res.load(str(path));self.scale=m['acceleration_scale']
 def predict(self,obs,deterministic=True):
  b=np.clip(self.base.predict(obs,True),-1,1);delta=self.res.predict(np.concatenate([obs,b]).astype(np.float32),deterministic)
  return ResidualEnv.compose(b,delta,self.scale)
if __name__=='__main__':
 validate.load_agent=lambda path,obs_dim,act_dim:CombinedPolicy(path)
 validate.main()
