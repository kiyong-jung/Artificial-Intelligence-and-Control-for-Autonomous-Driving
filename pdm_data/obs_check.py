"""수집 중 93차원 관측을 BusTask.observation으로 직접 계산해 두는 확인 모드(설계 12절).

노선 위치는 한 바퀴 노선(R68 + bus_loop) 기준이다. SUMO 안의 버스 노선은 바퀴마다 이어 붙어 getRouteIndex가 한 바퀴 번호와
다르므로 pose()만 바꾼다. 나머지 식은 env/bus_task.py 그대로다.
"""
from __future__ import annotations

from env.bus_task import BusTask

from .lap_context import LapContext, lap_route_index, lane_usable
from .schema import BUS, DETECT_RADIUS_M, DT, SERVICE_SECONDS, STOP_IDS


class ObsEnvAdapter:
    """BusTask가 env에서 읽는 것만 가진다."""
    obs_offsets = (2, 1, 0, -1, -2)

    def __init__(self, ctx: LapContext, tc):
        self.ctx, self.tc = ctx, tc
        self.ego_id, self.vmax, self.visibility, self.route_map = BUS, ctx.vmax, DETECT_RADIUS_M, ctx.route_map

    def _pose(self, vid):
        x, y = self.tc.vehicle.getPosition(vid)
        return float(x), float(y), float(self.tc.vehicle.getSpeed(vid)), float(self.tc.vehicle.getAngle(vid))

    def _offset_lane(self, offset):
        """env/sumo_env.py의 _offset_lane과 같은 규칙."""
        tc = self.tc
        edge = tc.vehicle.getRoadID(self.ego_id)
        if edge.startswith(":"):
            return tc.vehicle.getLaneID(self.ego_id) if offset == 0 else None
        index = tc.vehicle.getLaneIndex(self.ego_id) + offset
        lane = f"{edge}_{index}"
        if 0 <= index < tc.edge.getLaneNumber(edge) and lane_usable(self.ctx, lane):
            return lane
        return None


class LapBusTask(BusTask):
    def __init__(self, env, config, tc):
        self._last_road = None
        super().__init__(env, config, tc)

    def pose(self):
        tc, eid, ctx = self.tc, self.env.ego_id, self.env.ctx
        if eid not in tc.vehicle.getIDList():
            return None
        lane, edge = tc.vehicle.getLaneID(eid), tc.vehicle.getRoadID(eid)
        if not edge.startswith(":"):
            self._last_road = edge
        pos = ctx.route_map.position(lap_route_index(edge, self._last_road, ctx.lap_edges), lane,
                                     tc.vehicle.getLanePosition(eid))
        tls = tc.vehicle.getNextTLS(eid)
        return dict(position=pos, lane=lane, edge=edge, speed=tc.vehicle.getSpeed(eid),
                    distance=tc.vehicle.getDistance(eid), tls=tls[0] if tls else None)


class OnlineObsCheck:
    """tick()은 0.1 s마다(기록 시작 뒤), observe()는 스냅숏마다, reset()은 바퀴가 바뀔 때."""

    def __init__(self, ctx: LapContext, tc, n_rows: int):
        self.env = ObsEnvAdapter(ctx, tc)
        self.cfg = {"service_stop_ids": list(STOP_IDS), "service_seconds": SERVICE_SECONDS}
        self.left = int(n_rows)
        self.task = LapBusTask(self.env, self.cfg, tc)
        self.task.before_step()

    def reset(self):
        last = self.task._last_road
        self.task.reset()
        self.task._last_road = last
        self.task.before_step()

    def tick(self):
        self.task.tick(DT)

    def observe(self):
        if self.left <= 0:
            return None
        self.left -= 1
        return [float(x) for x in self.task.observation()]
