"""Frozen BC + school PPO residual action adapter. No teacher at runtime."""
import numpy as np
import torch
from gymnasium import spaces
class ResidualEnv:
 def __init__(self,env,base,scale=.5,trace=None):
  self.env=env;self.base=base;self.scale=scale;self.trace=trace
  self.observation_space=spaces.Box(-np.inf,np.inf,shape=(95,),dtype=np.float32)
  self.action_space=env.action_space
  self.base.policy.eval()
  for p in self.base.policy.parameters():p.requires_grad_(False)
 def __getattr__(self,k):return getattr(self.env,k)
 def augment(self,obs):
  self.base_action=np.clip(self.base.predict(obs,deterministic=True),-1,1).astype(np.float32)
  return np.concatenate([obs,self.base_action]).astype(np.float32)
 @staticmethod
 def compose(base,residual,scale=.5):
  # Residual lane code is a cyclic categorical offset, NOT a spatial lane command.
  # Base categories:0left,1keep,2right. Offset0 preserves; +/-1 covers other2.
  offset=int(round(float(residual[1])));assert offset in (-1,0,1)
  category=(1-int(round(float(base[1])))+offset)%3
  return np.array([np.clip(base[0]+scale*np.tanh(residual[0]),-1,1),1-category],dtype=np.float32)
 def reset(self,**kw):
  obs,info=self.env.reset(**kw);return self.augment(obs),info
 def step(self,residual):
  base=self.base_action.copy();applied=self.compose(base,residual,self.scale)
  obs,reward,term,trunc,info=self.env.step(applied)
  info['residual_control']={'base_action':base.tolist(),'residual_sample':np.asarray(residual).tolist(),'applied_policy_action':applied.tolist()}
  if self.trace:self.trace(info)
  return self.augment(obs),reward,term,trunc,info
 def close(self):return self.env.close()
def initialize_zero(agent):
 with torch.no_grad():
  agent.policy.mu_head.weight.zero_();agent.policy.mu_head.bias.zero_()
  agent.policy.lane_head.weight.zero_();agent.policy.lane_head.bias.zero_();agent.policy.lane_head.bias[1]=4.
