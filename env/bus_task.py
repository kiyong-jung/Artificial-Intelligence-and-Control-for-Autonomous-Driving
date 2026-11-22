"""Policy-only bus task: observation and goal scoring, never vehicle commands.

The default four service stops match the previous demo itinerary, not its controls.
All coordinates are simulator-visible ideal sensing (no occlusion model).
"""
import math
from copy import deepcopy
import numpy as np

PROFILE = "bus_service_v1"
STOP_IDS = ["osm_stop_357920024", "osm_stop_5930628870",
            "osm_stop_3446267493", "osm_stop_3424857165"]
LAYOUT = {"speed": [0, 1], "drivable_offsets": [1, 6],
          "nearest_vehicles_8_x_mask_xy_vxy": [6, 46],
          "nearest_pedestrians_6_x_mask_xy_vxy": [46, 76],
          "navigation_left_right_uturn_distance_remaining": [76, 81],
          "tls_present_distance_red_yellow_green_other": [81, 87],
          "stop_present_xy_dwell_completed_distance": [87, 93]}
_BASE = None
REWARDS = dict(progress_total=10.0, step_cost=0.001, service=2.0,
               arrival=10.0, collision=-10.0, red_light=-10.0, missed_stop=-5.0)


def crossed_red(prev, now):
    light = prev["tls"]
    return bool(light and light[3] in "rR" and not prev["edge"].startswith(":")
                and now["edge"] != prev["edge"]
                and light[2] <= now["distance"] - prev["distance"] + 0.5)


def apply_profile(road, ego, observation, action, profile):
    global _BASE
    if _BASE is None:
        _BASE = (deepcopy(observation), deepcopy(action))
    observation.clear(); observation.update(deepcopy(_BASE[0]))
    action.clear(); action.update(deepcopy(_BASE[1]))
    if profile == "legacy":
        return
    if profile != PROFILE or road["map_id"] != "hapjeong":
        raise ValueError("bus_service_v1 requires the Hapjeong map")
    ego.update(learning_profile=PROFILE, speed_mode=38, lane_change_mode=0,
               depart_lane="1", service_stop_ids=list(STOP_IDS), service_seconds=8.0,
               task_rewards=deepcopy(REWARDS), background_stop_period=600.0,
               background_stop_duration=20.0)
    # Speed mode 38: physical accel/decel and speed limit retained; no safe-speed,
    # red-light braking or junction priority assistance. LC0: no autonomous LC.
    action["lane_change_safety"]["enabled"] = False
    observation.update(profile=PROFILE, dim=93, layout=LAYOUT,
                       sensing="ideal_radius_no_occlusion", vehicle_slots=8, pedestrian_slots=6)


class ServiceTracker:
    """Continuous stationary service; crossing a stop without service is a failure."""
    def __init__(self, stops, dwell=8.0):
        self.stops, self.required = list(stops), float(dwell)
        self.index, self.dwell, self.missed = 0, 0.0, False
        self.completed = []

    def update(self, position, lane, speed, dt):
        if self.index >= len(self.stops) or self.missed:
            return None
        start, end, sid, stop_lane = self.stops[self.index]
        if position > end + 1e-6:
            self.missed = True
            return "missed_stop"
        # Front bumper must be in the stop area; criterion is recorded explicitly.
        if start <= position <= end and lane == stop_lane and speed <= 0.1:
            self.dwell += dt
            if self.dwell + 1e-8 >= self.required:
                self.completed.append(sid)
                self.index += 1
                self.dwell = 0.0
                return "service_completed"
        else:
            self.dwell = 0.0
        return None


class BusTask:
    def __init__(self, env, config, tc):
        self.env, self.config, self.tc = env, config, tc
        ids = config["service_stop_ids"]
        stops = [s for s in env.route_map.stops if s[2] in ids]
        if {s[2] for s in stops} != set(ids):
            raise ValueError("Required service stops are not on the ego route")
        self.stops = stops
        self.reset()

    def reset(self):
        self.tracker = ServiceTracker(self.stops, self.config["service_seconds"])
        self.red_violation = False
        self.step_services = 0
        self.previous = None
        self.progress = 0.0
        self.step_progress = 0.0

    def pose(self):
        tc, eid = self.tc, self.env.ego_id
        if eid not in tc.vehicle.getIDList():
            return None
        lane = tc.vehicle.getLaneID(eid)
        edge = tc.vehicle.getRoadID(eid)
        pos = self.env.route_map.position(tc.vehicle.getRouteIndex(eid), lane,
                                          tc.vehicle.getLanePosition(eid))
        tls = tc.vehicle.getNextTLS(eid)
        return dict(position=pos, lane=lane, edge=edge, speed=tc.vehicle.getSpeed(eid),
                    distance=tc.vehicle.getDistance(eid), tls=tls[0] if tls else None)

    def before_step(self):
        self.previous_service_potential = self.tracker.dwell / self.tracker.required
        self.step_services = 0
        self.step_progress = 0.0
        self.previous = self.pose()

    def tick(self, dt):
        now = self.pose()
        if now is None:
            return
        prev = self.previous
        if prev:
            self.step_progress += max(0.0, now["distance"] - prev["distance"])
            # Only count crossing the incoming edge boundary, not merely approaching.
            if crossed_red(prev, now):
                self.red_violation = True
        event = self.tracker.update(now["position"], now["lane"], now["speed"], dt)
        self.step_services += int(event == "service_completed")
        self.previous = now

    def state(self):
        return {"required_stops": [s[2] for s in self.stops],
                "completed_stops": list(self.tracker.completed), "dwell_seconds": self.tracker.dwell,
                "missed_stop": self.tracker.missed, "red_violation": self.red_violation,
                "all_serviced": self.tracker.index == len(self.stops)}

    def outcome(self, reward, terminated, truncated, info):
        # No timing/lane-choice reward. Progress + service + completion, with failures.
        weights = self.config["task_rewards"]
        reward = weights["progress_total"] * self.step_progress / self.env.route_map.length - weights["step_cost"]
        reward += weights["service"] * self.step_services
        if info["collided"]:
            reward, terminated = weights["collision"], True
        elif self.red_violation:
            reward, terminated = weights["red_light"], True
            info["termination_reason"] = "red_light"
        elif self.tracker.missed:
            reward, terminated = weights["missed_stop"], True
            info["termination_reason"] = "missed_stop"
        elif info["arrived"]:
            if self.tracker.index == len(self.stops):
                reward = weights["arrival"]
            else:
                reward = weights["missed_stop"]
                info["termination_reason"] = "incomplete_service"
        elif (terminated or truncated) and weights.get("incomplete_end") is not None:
            reward = weights["incomplete_end"]
        # Potential difference gives transient feedback for continuous service.
        # Leaving the stop resets dwell, cancelling the earlier shaping gain.
        # Terminal potential is zero; no change to action or termination rules.
        shaping_weight = weights.get("service_shaping_weight", 0.0)
        if shaping_weight:
            potential = 0.0 if terminated or truncated else self.tracker.dwell / self.tracker.required
            shaping = shaping_weight * (weights["service_shaping_gamma"] * potential
                                        - self.previous_service_potential)
            reward += shaping
            info["service_shaping_reward"] = float(shaping)
        info["route_arrived"] = info["arrived"]
        info["arrived"] = bool(info["arrived"] and self.state()["all_serviced"]
                               and not self.red_violation and not info["collided"])
        info["bus_service"] = self.state()
        info["red_violation"] = self.red_violation
        return float(reward), terminated, bool(truncated and not terminated), info

    def observation(self):
        env, tc = self.env, self.tc
        pose = self.pose()
        if pose is None:
            return np.zeros(93, np.float32)
        ex, ey, ev, heading = env._pose(env.ego_id)
        h = math.radians(heading)
        hx, hy = math.sin(h), math.cos(h)
        def relative(x, y, speed=0.0, angle=0.0):
            a = math.radians(angle)
            dx, dy = x-ex, y-ey
            vx, vy = speed*math.sin(a)-ev*hx, speed*math.cos(a)-ev*hy
            return [dx*hx+dy*hy, -dx*hy+dy*hx, vx*hx+vy*hy, -vx*hy+vy*hx]
        obs = [ev/env.vmax] + [float(env._offset_lane(k) is not None) for k in env.obs_offsets]
        for domain, count in [(tc.vehicle, 8), (tc.person, 6)]:
            items = []
            for vid in domain.getIDList():
                if domain is tc.vehicle and vid == env.ego_id:
                    continue
                x, y = domain.getPosition(vid)
                d2 = (x-ex)**2+(y-ey)**2
                if d2 <= env.visibility**2:
                    vals = relative(x,y,domain.getSpeed(vid),domain.getAngle(vid))
                    items.append((d2,vid,[1.0,vals[0]/env.visibility,vals[1]/env.visibility,
                                          vals[2]/env.vmax,vals[3]/env.vmax]))
            items.sort(key=lambda item:(item[0],item[1]))
            obs.extend(v for item in items[:count] for v in item[2])
            obs.extend([0.0]*5*max(0,count-len(items)))
        nav = env.route_map.ahead(pose["position"])
        obs += [float(nav['next_turn']==k) for k in ('left','right','turnaround')]
        obs += [(nav['distance_to_turn'] if nav['distance_to_turn'] is not None else 200.0)/200.0,
                nav['distance_to_destination']/env.route_map.length]
        tls = pose['tls']
        if tls and tls[2] <= env.visibility:
            ch=tls[3].lower()
            obs += [1.0,tls[2]/env.visibility,float(ch=='r'),float(ch=='y'),float(ch=='g'),float(ch not in 'ryg')]
        else:
            obs += [0.0]*6
        tr=self.tracker
        if tr.index < len(self.stops):
            start,end,sid,lane=self.stops[tr.index]
            from sumolib.geomhelper import positionAtShapeOffset
            x,y=positionAtShapeOffset(tc.lane.getShape(lane), (start+end)/2-env.route_map.start[
                env.route_map.edge_ids.index(tc.lane.getEdgeID(lane))])
            r=relative(x,y)
            obs += [1.0,r[0]/200.0,r[1]/200.0,tr.dwell/tr.required,
                    tr.index/len(self.stops),(start-pose['position'])/200.0]
        else:
            obs += [0.0,0.0,0.0,0.0,1.0,0.0]
        assert len(obs)==93
        return np.clip(np.asarray(obs,dtype=np.float32),-1,1)
