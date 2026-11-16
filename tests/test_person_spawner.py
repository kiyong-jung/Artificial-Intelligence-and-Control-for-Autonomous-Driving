"""보행자를 시험용 버스 반경 밖에서만 생기고 사라지게 한다(traffic/person_spawner.py)."""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sumolib.geomhelper import positionAtShapeOffset  # noqa: E402

from traffic.alleys import read_net  # noqa: E402
from traffic.person_spawner import PersonSpawner, extract_persons  # noqa: E402
from traffic.presence import PRESENCE_RADIUS_M  # noqa: E402

NET = os.path.join(os.path.dirname(__file__), "..", "maps", "hapjeong", "source", "network.net.xml")
R = PRESENCE_RADIUS_M
_cache = {}


def _net():
    if "net" not in _cache:
        _cache["net"] = read_net(NET)
    return _cache["net"]


def _start_xy(edge):
    return positionAtShapeOffset(_net().getEdge(edge).getShape(), 0.0)


class _Stage:
    def __init__(self, type_):
        self.type = type_


class _Person:
    def __init__(self, tc):
        self.tc = tc

    def add(self, pid, edge, pos, depart=-3, typeID="DEFAULT_PEDTYPE"):
        self.tc.added.append(pid)
        self.tc.alive[pid] = {"stages": [], "type": typeID, "xy": _start_xy(edge)}

    def appendWalkingStage(self, pid, edges, arrivalPos, *a, **k):
        self.tc.alive[pid]["stages"].append(("walk", list(edges)))

    def appendWaitingStage(self, pid, duration, *a, **k):
        self.tc.alive[pid]["stages"].append(("wait", duration))

    def getIDList(self):
        return list(self.tc.alive)

    def getRemainingStages(self, pid):
        return len(self.tc.alive[pid]["stages"])

    def getStage(self, pid, nextStageIndex=0):
        return _Stage(1 if self.tc.alive[pid]["stages"][0][0] == "wait" else 2)

    def getPosition(self, pid):
        return self.tc.alive[pid]["xy"]

    def remove(self, pid, *a):
        self.tc.removed.append(pid)
        del self.tc.alive[pid]


class _Vehicle:
    def __init__(self, tc):
        self.tc = tc

    def getIDList(self):
        return ["bus0"] if self.tc.bus_xy is not None else []

    def getPosition(self, vid):
        return self.tc.bus_xy


class _FakeTC:
    def __init__(self, bus_xy):
        self.bus_xy = bus_xy
        self.added, self.removed, self.alive = [], [], {}
        self.person, self.vehicle = _Person(self), _Vehicle(self)


def _walk_done(tc, pid):
    tc.alive[pid]["stages"].pop(0)            # 걷기 단계가 끝나 마지막 기다림 단계만 남았다


def _person(pid="p0", edge="R71.206", depart=0.0, ptype=None):
    return {"id": pid, "depart": depart, "type": ptype, "edges": [edge, "R71.256"]}


def test_extract_persons_moves_them_out_of_the_route_file(tmp_path):
    p = tmp_path / "ped.rou.xml"
    p.write_text('<routes><person id="a" depart="6.00" type="ped_p1"><walk edges="R1 R2"/></person>'
                 '<person id="b" depart="0.00"><walk edges="R3"/></person></routes>', encoding="utf-8")
    out = extract_persons(str(p))
    assert out == [{"id": "b", "depart": 0.0, "type": None, "edges": ["R3"]},
                   {"id": "a", "depart": 6.0, "type": "ped_p1", "edges": ["R1", "R2"]}]
    assert list(ET.parse(str(p)).getroot().iter("person")) == []


def test_person_departure_waits_until_bus_is_far():
    x, y = _start_xy("R71.206")
    tc = _FakeTC(bus_xy=(x + 0.5 * R, y))
    sp = PersonSpawner(tc, _net(), "bus0", [_person()])
    sp.update(0.0)
    assert tc.added == [] and sp.summary()["deferred"] >= 1
    tc.bus_xy = (x + 5 * R, y)
    sp.update(1.0)
    assert tc.added == ["p0"]
    assert [s[0] for s in tc.alive["p0"]["stages"]] == ["walk", "wait"]   # 걷고 나서 기다리며 서 있는다


def test_spawned_person_keeps_its_type():
    tc = _FakeTC(bus_xy=None)
    PersonSpawner(tc, _net(), "bus0", [_person(ptype="ped_p1")]).update(0.0)
    assert tc.alive["p0"]["type"] == "ped_p1"


def test_lingering_person_removed_only_when_bus_far():
    tc = _FakeTC(bus_xy=None)
    sp = PersonSpawner(tc, _net(), "bus0", [_person()])
    sp.update(0.0)
    _walk_done(tc, "p0")
    x, y = tc.alive["p0"]["xy"]
    tc.bus_xy = (x + 0.3 * R, y)
    sp.update(1.0)
    assert tc.removed == []
    tc.bus_xy = (x + 3 * R, y)
    sp.update(2.0)
    assert tc.removed == ["p0"] and sp.summary()["forced_vanish"] == 0


def test_lingering_person_removed_after_cap_even_if_bus_near():
    tc = _FakeTC(bus_xy=None)
    sp = PersonSpawner(tc, _net(), "bus0", [_person()], linger_max=120.0)
    sp.update(0.0)
    _walk_done(tc, "p0")
    x, y = tc.alive["p0"]["xy"]
    tc.bus_xy = (x + 0.3 * R, y)
    sp.update(1.0)
    sp.update(121.5)
    assert tc.removed == ["p0"] and sp.summary()["forced_vanish"] == 1
