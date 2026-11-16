"""보행자를 시험용 버스 반경(`presence.PRESENCE_RADIUS_M`) 밖에서만 생기고 사라지게 한다.

경로 파일(pedestrians.rou.xml)의 보행자는 인도 한가운데에서 갑자기 생기고 걷기가 끝나면 그 자리에서 사라진다. 버스 바로 옆에서
그러면 순간이동으로 보인다. 그래서 보행자를 경로 파일에서 떼어(`extract_persons`) 이 클래스가 TraCI로 넣는다.

- 출발: 출발 시각이 되었어도 출발 지점이 버스 반경 안이면 `retry`초 뒤에 다시 본다(버스가 지나가면 출발한다).
- 도착: 걷기 뒤에 긴 기다림 단계를 붙여, 걷기가 끝나면 그 자리에 서 있게 한다. 버스가 반경 밖에 있으면 바로 지운다.
  버스가 옆 정류장에 오래 서 있는 경우를 위해 `linger_max`초가 지나면 버스가 가까워도 지우고 `forced_vanish`로 센다.
"""
import xml.etree.ElementTree as ET

from .presence import PERSON_MARGIN_M, PRESENCE_RADIUS_M, far_from_bus, sidewalk_xy

STAGE_WAITING = 1


def extract_persons(rou_path):
    """경로 파일에서 보행자를 떼어 목록으로 돌려준다(출발 시각 순). 파일에서는 지운다.
    반환: [{"id", "depart", "type", "edges"}]. type은 없으면 None."""
    tree = ET.parse(rou_path)
    root = tree.getroot()
    out = []
    for person in list(root.iter("person")):
        walk = person.find("walk")
        edges = (walk.get("edges") or "").split() if walk is not None else []
        if edges:
            out.append({"id": person.get("id"), "depart": float(person.get("depart", 0.0)),
                        "type": person.get("type"), "edges": edges})
        root.remove(person)
    tree.write(rou_path, encoding="utf-8", xml_declaration=True)
    return sorted(out, key=lambda p: p["depart"])


class PersonSpawner:
    """update(t)를 매 스텝 simulationStep() 앞에서 호출한다(warmup 중에도)."""

    def __init__(self, tc, net, bus_id, persons, radius=PRESENCE_RADIUS_M, linger_max=120.0, retry=1.0,
                 check_period=0.5, linger_s=100000.0):
        self.tc, self.net, self.bus = tc, net, bus_id
        self.radius, self.linger_max, self.retry = radius, linger_max, retry
        self.check_period, self.linger_s = check_period, linger_s
        self.persons = sorted(persons, key=lambda p: p["depart"])
        self._i = 0
        self._waiting = []            # [다시 볼 시각, 사람] 출발을 미룬 사람
        self._alive = {}              # 넣은 사람 id -> 서 있기 시작한 시각(None이면 아직 걷는 중)
        self._unseen = set()          # 넣었지만 아직 SUMO 목록에 나타나지 않은 사람
        self._t_check = -1e9
        self.n = {"spawned": 0, "deferred": 0, "removed": 0, "forced_vanish": 0, "failed": 0}

    def _try_add(self, p):
        if not self.net.hasEdge(p["edges"][0]) or not self.net.hasEdge(p["edges"][-1]):
            self.n["failed"] += 1
            return True
        # 인도 위치로 재고 여유를 둔다(도로 중심선으로 재면 반경 바로 안 28~29 m에서 생겼다)
        if not far_from_bus(self.tc, self.bus, sidewalk_xy(self.net, p["edges"][0]), self.radius + PERSON_MARGIN_M):
            return False
        tc = self.tc
        last = self.net.getEdge(p["edges"][-1]).getLength()
        try:
            if p.get("type"):
                tc.person.add(p["id"], p["edges"][0], 0.0, typeID=p["type"])
            else:
                tc.person.add(p["id"], p["edges"][0], 0.0)
            tc.person.appendWalkingStage(p["id"], p["edges"], max(0.1, last - 1.0))
            tc.person.appendWaitingStage(p["id"], self.linger_s)       # 걷기가 끝나면 그 자리에 서 있는다
        except Exception:
            self.n["failed"] += 1
            return True
        self._alive[p["id"]] = None
        self._unseen.add(p["id"])          # SUMO는 다음 스텝에야 목록에 올린다
        self.n["spawned"] += 1
        return True

    def update(self, t):
        while self._i < len(self.persons) and self.persons[self._i]["depart"] <= t:
            self._waiting.append([t, self.persons[self._i]])
            self._i += 1
        if self._waiting:
            keep = []
            for item in self._waiting:
                if item[0] > t:
                    keep.append(item)
                elif not self._try_add(item[1]):
                    self.n["deferred"] += 1
                    keep.append([t + self.retry, item[1]])
            self._waiting = keep
        if t - self._t_check < self.check_period or not self._alive:
            return
        self._t_check = t
        tc = self.tc
        ids = set(tc.person.getIDList())
        for pid in list(self._alive):
            if pid not in ids:
                if pid not in self._unseen:
                    del self._alive[pid]                  # 한 번 나타났다가 없어졌다(SUMO가 지웠다)
                continue
            self._unseen.discard(pid)
            if self._alive[pid] is None:
                if tc.person.getRemainingStages(pid) <= 1 and tc.person.getStage(pid).type == STAGE_WAITING:
                    self._alive[pid] = t                                  # 걷기가 끝나 서 있기 시작했다
                else:
                    continue
            far = far_from_bus(tc, self.bus, tc.person.getPosition(pid), self.radius)
            over = t - self._alive[pid] > self.linger_max
            if far or over:
                tc.person.remove(pid)
                del self._alive[pid]
                self.n["removed"] += 1
                if not far:
                    self.n["forced_vanish"] += 1

    def summary(self):
        return dict(self.n)
