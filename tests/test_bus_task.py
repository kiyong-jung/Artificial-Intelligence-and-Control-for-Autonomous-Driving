import unittest
from types import SimpleNamespace
from env.bus_task import ServiceTracker, BusTask, REWARDS, crossed_red


class ServiceTests(unittest.TestCase):
    def test_red_boundary_not_approach_or_other_intersection(self):
        prev=dict(tls=('tls',0,0.3,'r'),edge='incoming',distance=10)
        self.assertTrue(crossed_red(prev,dict(edge=':junction',distance=10.4)))
        self.assertFalse(crossed_red(prev,dict(edge='incoming',distance=10.2)))
        self.assertFalse(crossed_red(dict(prev,tls=('tls',0,50,'r')),dict(edge='next',distance=10.4)))
        self.assertFalse(crossed_red(dict(prev,tls=('tls',0,.3,'G')),dict(edge=':junction',distance=10.4)))
    def tracker(self):
        return ServiceTracker([(10, 30, 'a', 'road_1')], dwell=1)

    def test_wrong_lane_and_moving_do_not_service(self):
        tr = self.tracker()
        for _ in range(20):
            tr.update(20, 'road_2', 0, .1)
            tr.update(20, 'road_1', 1, .1)
        self.assertEqual(tr.completed, [])

    def test_service_requires_continuous_dwell_and_rewards_once(self):
        tr = self.tracker()
        for _ in range(9):
            tr.update(20, 'road_1', 0, .1)
        tr.update(20, 'road_1', .2, .1)
        for _ in range(9):
            self.assertIsNone(tr.update(20, 'road_1', 0, .1))
        self.assertEqual(tr.update(20, 'road_1', 0, .1), 'service_completed')
        self.assertIsNone(tr.update(20, 'road_1', 0, .1))
        self.assertEqual(tr.completed, ['a'])

    def test_passed_stop_is_failure(self):
        tr = self.tracker()
        self.assertEqual(tr.update(31, 'road_1', 2, .1), 'missed_stop')

    def test_arrival_without_service_not_success(self):
        task = object.__new__(BusTask)
        task.config = {'task_rewards':REWARDS}
        task.env = SimpleNamespace(route_map=SimpleNamespace(length=100))
        task.tracker = self.tracker()
        task.stops = task.tracker.stops
        task.red_violation = False
        task.step_progress = task.step_services = 0
        _, term, trunc, info = task.outcome(2, True, False,
            dict(arrived=True,collided=False,termination_reason='arrived'))
        self.assertTrue(term)
        self.assertFalse(info['arrived'])
        self.assertEqual(info['termination_reason'], 'incomplete_service')

if __name__ == '__main__':
    unittest.main()
