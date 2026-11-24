"""PDM 데이터 시험용 가짜 TraCI. 도메인별 값은 dict로 채우고, 호출은 FakeTC.calls에 (이름, 인자)로 쌓는다."""
import functools
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from traffic.sumo_conn import traci  # noqa: E402

TraCIException = traci.exceptions.TraCIException

VEH_KEYS = {
    "getPosition": "pos", "getAngle": "angle", "getSpeed": "speed", "getAcceleration": "accel",
    "getLateralSpeed": "lat_speed", "getSpeedWithoutTraCI": "speed_wo", "getAllowedSpeed": "allowed",
    "getMaxSpeed": "max_speed", "getRoadID": "road", "getLaneID": "lane", "getLaneIndex": "lane_index",
    "getLanePosition": "lane_pos", "getLateralLanePosition": "lat_pos", "getRouteIndex": "route_index",
    "getDistance": "distance", "getSignals": "signals", "getStopState": "stop_state", "getWaitingTime": "waiting",
    "getAccumulatedWaitingTime": "acc_waiting", "getLaneChangeState": "lc_state", "couldChangeLane": "could",
    "getTypeID": "type", "getVehicleClass": "vclass", "getLength": "length", "getWidth": "width",
    "getMinGap": "min_gap", "getDecel": "decel", "getParameter": "params", "getLeader": "leader",
    "getFollower": "follower", "getNeighbors": "neighbors", "getBestLanes": "best_lanes", "getNextLinks": "next_links",
    "getNextTLS": "next_tls", "getStops": "stops", "getJunctionFoes": "foes", "getDrivingDistance": "driving",
    "getRoute": "route",
}
VEH_DEFAULTS = dict(pos=(0.0, 0.0), angle=0.0, speed=0.0, accel=0.0, lat_speed=0.0, speed_wo=0.0, allowed=13.89,
                    max_speed=13.89, road="R69", lane="R69_1", lane_index=1, lane_pos=10.0, lat_pos=0.0, route_index=1,
                    distance=0.0, signals=0, stop_state=0, waiting=0.0, acc_waiting=0.0,
                    lc_state=lambda d: (0, 0), could=lambda d: True, type="car_idm", vclass="passenger", length=4.5,
                    width=1.8, min_gap=2.5, decel=4.5, params=lambda k: "", leader=lambda d: None,
                    follower=lambda d: ("", -1.0), neighbors=lambda mode: (), best_lanes=(), next_links=(), next_tls=(),
                    stops=lambda n=4: (), foes=lambda d: (), driving=lambda e, p: 100.0, route=("R68", "R69"))

PER_KEYS = {"getPosition": "pos", "getAngle": "angle", "getSpeed": "speed", "getRoadID": "road",
            "getLanePosition": "lane_pos", "getNextEdge": "next_edge", "getWaitingTime": "waiting", "getStage": "stage"}
PER_DEFAULTS = dict(pos=(0.0, 0.0), angle=0.0, speed=1.2, road="R69", lane_pos=0.0, next_edge="", waiting=0.0,
                    stage=lambda *a: type("Stage", (), {"type": 2})())


class _Dom:
    """keys: 메서드 이름 -> 값 키. 값이 callable이면 첫 인자(id) 뒤의 인자로 부른다."""

    def __init__(self, tc, name, items, keys, defaults):
        self._tc, self._name, self.items, self._keys, self._defaults = tc, name, items, keys, defaults
        self.const = {}

    def getIDList(self):
        return list(self.items)

    def _get(self, meth, oid, *rest):
        if oid not in self.items or (self.items[oid].get("gone") and meth != "getPosition"):
            raise TraCIException(f"{oid} 없음")
        key = self._keys[meth]
        val = self.items[oid].get(key, self._defaults[key])
        return val(*rest) if callable(val) else val

    def __getattr__(self, meth):
        if meth.startswith("_"):
            raise AttributeError(meth)

        def call(*args):
            self._tc.calls.append((f"{self._name}.{meth}", args))
            if meth in self._keys:
                return self._get(meth, *args)
            if meth in self.const:
                c = self.const[meth]
                return c(*args) if callable(c) else c
            return None
        return call


class _Simple:
    def __init__(self, tc, name, table):
        self._tc, self._name, self.table = tc, name, table

    def __getattr__(self, meth):
        if meth.startswith("_"):
            raise AttributeError(meth)

        def call(*args):
            self._tc.calls.append((f"{self._name}.{meth}", args))
            v = self.table.get(meth)
            if callable(v):
                return v(*args)
            if isinstance(v, dict):
                if args and args[0] in v:
                    return v[args[0]]
                raise TraCIException(f"{meth}{args}")
            return v
        return call


class FakeTC:
    def __init__(self, vehicles=None, persons=None, now=10.0):
        self.calls = []
        self.vehicle = _Dom(self, "vehicle", dict(vehicles or {}), VEH_KEYS, VEH_DEFAULTS)
        self.vehicle.const.update(getSecureGap=7.0, getFollowSpeed=9.0, getStopSpeed=5.0)
        self.person = _Dom(self, "person", dict(persons or {}), PER_KEYS, PER_DEFAULTS)
        self.lane = _Simple(self, "lane", {"getMaxSpeed": lambda l: 13.89, "getAllowed": lambda l: [],
                                           "getLength": lambda l: 100.0})
        self.edge = _Simple(self, "edge", {"getLaneNumber": lambda e: 3})
        self.trafficlight = _Simple(self, "trafficlight", {"getNextSwitch": lambda i: 37.0,
                                                           "getSpentDuration": lambda i: 5.0, "getPhase": lambda i: 2,
                                                           "getProgram": lambda i: "0"})
        self.now = now
        self.collisions, self.teleports, self.emergency = [], [], []
        self.simulation = _Simple(self, "simulation", {"getTime": lambda: self.now,
                                                       "getCollisions": lambda: list(self.collisions),
                                                       "getStartingTeleportIDList": lambda: list(self.teleports),
                                                       "getEmergencyStoppingVehiclesIDList": lambda: list(self.emergency)})


@functools.lru_cache(maxsize=1)
def _ctx_args():
    from maps import load_map
    from traffic.alleys import read_net
    from traffic.test_bus import lap_route
    m = load_map("hapjeong")
    edges = lap_route(read_net(m["net_path"]), m["routes"]["bus_loop"])
    return m["net_path"], tuple(edges), m["bus_stops"], m["route_design"]["bus_loop"]["max_speed_mps"]


def hapjeong_ctx(lane_allowed=None):
    """합정역 한 바퀴 노선 문맥(새 객체). lane_allowed를 안 주면 노선 도로의 모든 차로를 버스 허용으로 둔다."""
    from pdm_data.lap_context import build_lap_context
    from traffic.alleys import read_net
    net_path, edges, stops, vmax = _ctx_args()
    if lane_allowed is None:
        net = read_net(net_path)
        lane_allowed = {ln.getID(): [] for e in edges for ln in net.getEdge(e).getLanes()}
    return build_lap_context(net_path, list(edges), stops, vmax, lane_allowed)
