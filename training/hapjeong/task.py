import numpy as np
import env.bus_task as school_task
from env.experiment import configure_execution as base_configure
from env.road_config import EGO
from env.mdp_config import OBSERVATION
REWARDS=dict(progress_total=10.,step_cost=.001,service=5.,arrival=10.,collision=-10.,red_light=-10.,missed_stop=-10.,incomplete_end=-10.,service_shaping_weight=2.,service_shaping_gamma=.999)
STOP_ID='osm_stop_357920024'
BaseTask=school_task.BusTask
class FullTask(BaseTask):
 def __init__(self,env,config,tc):
  super().__init__(env,config,tc)
  assert len(self.stops)==4 and self.stops[0][2]==STOP_ID
  self.reference_distance=max(1.,self.stops[0][1])
  self.progress_per_m=10./self.reference_distance
 def observation(self):
  obs=super().observation();obs[1:6]=0;pose=self.pose()
  if pose is not None:
   goal=self.stops[self.tracker.index][1] if self.tracker.index<len(self.stops) else self.env.route_map.length
   remaining=max(0.,goal-pose['position']);nav=self.env.route_map.ahead(pose['position'])
   if nav['distance_to_turn'] is None or nav['distance_to_turn']>remaining:obs[76:79]=0;obs[79]=1
   obs[80]=np.clip(remaining/max(1.,goal),0,1)
  return obs
 def outcome(self,*args):
  reward,term,trunc,info=super().outcome(*args)
  if not term and not trunc:reward+=self.step_progress*(self.progress_per_m-self.config['task_rewards']['progress_total']/self.env.route_map.length)
  info.update(progress_per_m=self.progress_per_m,reference_first_stop_distance=self.reference_distance)
  return float(reward),term,trunc,info

def configure_execution(args):
 args.max_steps=2400;base_configure(args)
 EGO.update(task_scope='full_four_stops_exploratory_v1',task_rewards=dict(REWARDS),observation_mask_indices=[1,2,3,4,5])
 OBSERVATION['masked_indices']=[1,2,3,4,5]
def install():school_task.BusTask=FullTask
def check_model_config(config):
 if config['ego'].get('observation_mask_indices')!=[1,2,3,4,5] or config['observation'].get('masked_indices')!=[1,2,3,4,5]:
  raise ValueError('Requires masked 93D full-route model')
