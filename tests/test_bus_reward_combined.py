import unittest
from types import SimpleNamespace
from env.bus_task import BusTask, ServiceTracker, REWARDS

class CombinedRewardTests(unittest.TestCase):
    def task(self, combined=True):
        task=object.__new__(BusTask)
        weights=dict(REWARDS)
        if combined:
            weights.update(incomplete_end=-10.,service_shaping_weight=2.,service_shaping_gamma=.999)
        task.config={'task_rewards':weights}
        task.env=SimpleNamespace(route_map=SimpleNamespace(length=2000.))
        task.tracker=ServiceTracker([(10,30,'stop','road_1')],8.)
        task.stops=task.tracker.stops
        task.red_violation=False
        task.step_progress=task.step_services=0
        task.previous_service_potential=0.
        return task
    def outcome(self, task, term=False, trunc=False, **kw):
        info=dict(collided=False,arrived=False,termination_reason=None)
        info.update(kw)
        return task.outcome(999,term,trunc,info)
    def test_normal_wait_has_no_extra_stationary_penalty(self):
        self.assertAlmostEqual(self.outcome(self.task())[0],-.001)
    def test_progress_increase_does_not_override_failure(self):
        task=self.task();task.config['task_rewards']['progress_total']=20.
        task.step_progress=10.
        self.assertAlmostEqual(self.outcome(task)[0],.099)
        self.assertEqual(self.outcome(task,term=True,collided=True)[0],-10.)
        task.tracker.missed=True
        self.assertEqual(self.outcome(task)[0],-5.)
    def test_legacy_terminal_unchanged(self):
        self.assertAlmostEqual(self.outcome(self.task(False),term=True,termination_reason='teleport')[0],-.001)
    def test_unsuccessful_end_penalized(self):
        for term,trunc,reason in [(True,False,'teleport'),(False,True,'time_limit')]:
            reward,t,u,_=self.outcome(self.task(),term,trunc,termination_reason=reason)
            self.assertEqual(reward,-10.)
            self.assertEqual((t,u),(term,trunc))
    def test_collision_priority_and_terminal_potential(self):
        task=self.task();task.previous_service_potential=.5
        self.assertEqual(self.outcome(task,term=True,collided=True)[0],-11.)
    def test_dwell_reset_cancels_discounted_shaping(self):
        task=self.task();g=.999
        potentials=[0.,.1,.2,.3,0.]
        total=0.
        for i,(before,after) in enumerate(zip(potentials,potentials[1:])):
            task.previous_service_potential=before;task.tracker.dwell=after*8
            total+=g**i*(self.outcome(task)[0]+.001)
        self.assertAlmostEqual(total,0.)

if __name__=='__main__':unittest.main()
