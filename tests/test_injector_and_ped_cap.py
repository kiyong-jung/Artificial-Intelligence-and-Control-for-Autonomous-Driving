"""시나리오 주입기의 경로 계산·기록과, 보행자의 차량 대기 상한(veh_wait_hard)."""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from traffic.ped_defiance import DefianceCfg, PedDefianceManager, _Ped  # noqa: E402
from traffic.scenario_injector import ScenarioInjector  # noqa: E402


class _Lane:
    LENGTHS = {"A": 100.0, "B": 60.0, "C": 80.0}

    def getLength(self, lane): return self.LENGTHS[lane.rsplit("_", 1)[0]]


class _Tc:
    lane = _Lane()


class _Net:
    def __init__(self, tls=()):
        self.tls = set(tls)

    def getEdge(self, eid):
        net = self

        class _E:
            def getToNode(self):
                class _N:
                    def getType(self_):
                        return "traffic_light" if eid in net.tls else "priority"
                return _N()
        return _E()


def _inj(tls=()):
    return ScenarioInjector(_Tc(), _Net(tls), "bus0", [], [], "cutin_0", lap_fn=lambda: 1, plans=[],
                            rng=random.Random(0))


def test_back_point_walks_backwards_along_the_route():
    inj, route = _inj(), ["A", "B", "C"]
    assert inj._back_point(route, 2, 50.0, 30.0) == (2, 20.0)             # 같은 도로 안
    k, p = inj._back_point(route, 2, 10.0, 70.0)                          # C 10 m 뒤로 가서 B로 60 m 더 -> B 시작점
    assert (k, round(p)) == (1, 0) or (k, round(p)) == (0, 100)
    assert inj._back_point(route, 1, 10.0, 200.0) is None                  # 경로 시작보다 더 뒤: 투입 못 한다
    assert inj._gap(route, 0, 40.0, 2, 10.0) == 60.0 + 60.0 + 10.0


def test_back_point_stops_after_a_signalized_junction():
    inj, route = _inj(tls=("B",)), ["A", "B", "C"]                         # B 끝이 신호 교차로
    assert inj._back_point(route, 2, 10.0, 70.0) == (2, 1.0)               # 신호 뒤 C 시작에서 출발(적색에 걸리지 않게)
    assert inj._back_point(route, 2, 50.0, 30.0) == (2, 20.0)              # 같은 도로 안이면 그대로


def test_summary_separates_natural_from_injected():
    inj = _inj()
    inj._record("S1", 1.0, "natural")
    inj._record("S1", 2.0, "injected", "x")
    inj._record("S5", 3.0, False)
    s = inj.summary()["by_kind"]
    assert s["S1"] == {"natural": 1, "injected": 1, "failed": 0} and s["S5"]["failed"] == 1


class _Veh:
    def __init__(self, wait): self.wait = wait
    def getWaitingTime(self, v): return self.wait
    def getLanePosition(self, v): return 95.0
    def getSpeed(self, v): return 0.0
    def getWidth(self, v): return 1.8
    def getHeight(self, v): return 1.5
    def getTypeID(self, v): return "car"


class _Person:
    def __init__(self, road): self.road, self.params = road, []
    def getRoadID(self, p): return self.road
    def setParameter(self, p, k, v): self.params.append((p, k, v))
    def getSpeed(self, p): return 0.0


class _L:
    def getLastStepVehicleIDs(self, lane): return ["v"]
    def getLength(self, lane): return 100.0


def _mgr(wait, road):
    m = object.__new__(PedDefianceManager)
    class T: vehicle, lane, person = _Veh(wait), _L(), _Person(road)
    m.traci, m.cfg, m.rng = T(), DefianceCfg(), random.Random(0)
    m.incoming = {"cross": {"lane"}}
    m.peds, m.log, m.watch, m.revoked, m.ignore_scope = {}, [], {}, 0, "all"
    return m, T


def test_ped_does_not_start_crossing_when_the_lead_car_has_waited_long():
    m, T = _mgr(wait=6.0, road=":J_w0")
    st = _Ped(age=30, distracted=False, t_wait0=0.0, crossing="cross")
    m._decide("p", st, 10.0)
    assert not st.decided and T.person.params == [] and m.log[-1]["p"] == 0.0     # 차를 먼저 보낸다


def test_pending_decision_is_revoked_but_a_crossing_ped_finishes():
    m, T = _mgr(wait=6.0, road=":J_w0")                                           # 아직 대기 공간
    st = _Ped(age=30, distracted=False, t_wait0=0.0, crossing="cross", decided=True)
    m._maybe_revoke("p", st)
    assert not st.decided and m.revoked == 1 and T.person.params[-1][2] == ""
    m2, T2 = _mgr(wait=6.0, road=":J_c0")                                         # 이미 건너는 중
    st2 = _Ped(age=30, distracted=False, t_wait0=0.0, crossing="cross", decided=True)
    m2._maybe_revoke("p", st2)
    assert st2.decided and m2.revoked == 0


def test_ped_defiance_ignores_only_the_test_bus_and_never_a_background_car():
    # 배경 차(선두)가 막고 있으면 보행자는 기본 규칙대로 기다린다. 무시하고 건너는 것은 시험용 버스에 대해서만이다
    m, T = _mgr(wait=0.0, road=":J_w0")
    m.ignore_scope, m.bus = "bus", "bus0"
    m.cfg.base, m.cfg.use = 1.0, []                                                # 건너기 확률 1: 결과가 우연에 좌우되지 않게
    st = _Ped(age=30, distracted=False, t_wait0=0.0, crossing="cross")
    m._decide("p", st, 10.0)                                                       # 선두는 id 'v'(배경 차)
    assert not st.decided and T.person.params == []
    m.bus = "v"                                                                    # 선두가 시험용 버스
    m._decide("p", st, 11.0)
    assert st.decided and T.person.params[-1][2] == "car"
