"""주변 교통 지표: 유발 급제동, 충돌·비상정지, 승객 가중 지연, 보행자 PET.

버스 기준 반경 안의 주변 차량만 센다. 한 스텝의 속도 변화로 감속을 구하므로(직전 스텝 값 사용)
버스 행동보다 최소 한 스텝 늦게 나타난다.
"""


def pet(ped_interval, bus_interval):
    """Post-Encroachment Time [s]. 구간은 (진입 시각, 이탈 시각).

    겹치면 0, 한쪽이 나간 뒤 다른 쪽이 들어오면 그 사이 시간. 어느 한쪽이 없으면 None.
    """
    if ped_interval is None or bus_interval is None:
        return None
    (pi, po), (bi, bo) = ped_interval, bus_interval
    if bi >= po:
        return bi - po
    if pi >= bo:
        return pi - bo
    return 0.0


def passenger_weighted_delay(delays, occupancies):
    """차량별 지연[s]을 탑승 인원으로 가중한 합 [인·s]."""
    return float(sum(d * o for d, o in zip(delays, occupancies)))


class MetricsRecorder:
    def __init__(self, traci, bus_id, radius=50.0, brake_thresh=3.0, dt=0.1,
                 crossing_edge=None, bus_zone_roads=None, occupancy=None):
        self.tc, self.bus, self.radius, self.thresh, self.dt = traci, bus_id, radius, brake_thresh, dt
        self.prev = {}
        self.hard_brakes = {}        # 차량 id -> 최대 감속(임계 초과분만)
        self.collisions = 0
        self.emergency_stops = set()
        self.crossing_edge = crossing_edge
        self.bus_zone_roads = bus_zone_roads    # 버스가 횡단 구역에 있는 도로 id 집합
        # 차량 종류별 기본 탑승 인원. getPersonNumber가 0보다 크면 그 값을 쓴다(버스 승객 등).
        self.occupancy = {"passenger": 1.3, "taxi": 1.5, "truck": 1.0, "bus": 20.0, **(occupancy or {})}
        self.delay = {}              # 차량 id -> 속도 손실 지연 [s]
        self.occ = {}                # 차량 id -> 탑승 인원
        self.ped_iv = {}             # 보행자 id -> [in, out]
        self.bus_iv = None

    def step(self, t):
        tc = self.tc
        ids = tc.vehicle.getIDList()
        self.collisions += len(tc.simulation.getCollisions())
        self.emergency_stops.update(tc.simulation.getEmergencyStoppingVehiclesIDList())
        if self.bus in ids:
            bx, by = tc.vehicle.getPosition(self.bus)
        for vid in ids:
            sp = tc.vehicle.getSpeed(vid)
            if vid != self.bus and self.bus in ids and vid in self.prev:
                x, y = tc.vehicle.getPosition(vid)
                if ((x - bx) ** 2 + (y - by) ** 2) ** 0.5 <= self.radius:
                    dec = (self.prev[vid] - sp) / self.dt
                    if dec >= self.thresh:
                        self.hard_brakes[vid] = max(self.hard_brakes.get(vid, 0.0), dec)
            self.prev[vid] = sp
            if vid != self.bus:
                vmax = min(tc.vehicle.getAllowedSpeed(vid), tc.vehicle.getMaxSpeed(vid))
                if vmax > 0:
                    self.delay[vid] = self.delay.get(vid, 0.0) + self.dt * max(0.0, 1.0 - sp / vmax)
                if vid not in self.occ:
                    n = tc.vehicle.getPersonNumber(vid)
                    self.occ[vid] = float(n) if n > 0 else self.occupancy.get(
                        tc.vehicle.getVehicleClass(vid), 1.3)
        if self.crossing_edge:
            for pid in tc.person.getIDList():
                on = tc.person.getRoadID(pid) == self.crossing_edge
                iv = self.ped_iv.get(pid)
                if on and iv is None:
                    self.ped_iv[pid] = [t, None]
                elif not on and iv is not None and iv[1] is None:
                    iv[1] = t
        if self.bus_zone_roads and self.bus in ids:
            on = tc.vehicle.getRoadID(self.bus) in self.bus_zone_roads
            if on and self.bus_iv is None:
                self.bus_iv = [t, None]
            elif not on and self.bus_iv is not None and self.bus_iv[1] is None:
                self.bus_iv[1] = t

    def summary(self):
        pets = {}
        for pid, iv in self.ped_iv.items():
            if iv[1] is not None and self.bus_iv and self.bus_iv[1] is not None:
                pets[pid] = pet(tuple(iv), tuple(self.bus_iv))
        return {"hard_brake_vehicles": len(self.hard_brakes),
                "max_induced_decel": max(self.hard_brakes.values(), default=0.0),
                "collisions": self.collisions,
                "emergency_stops": len(self.emergency_stops), "pet": pets,
                "weighted_delay_person_s": passenger_weighted_delay(
                    [self.delay[v] for v in self.delay], [self.occ[v] for v in self.delay])}
