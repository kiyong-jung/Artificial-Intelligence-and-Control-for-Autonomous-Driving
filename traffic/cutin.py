"""A2. 끼어들기형(S5) vType 분포와 정차 차량(S4) 정의.

끼어들기는 차선변경이 아니라 우선 교차로(priority junction)의 우선권 위반으로 구현한다.
jm* 파라미터는 vType 단위로만 설정되므로 분포를 구간으로 나눈 vTypeDistribution을 쓴다.
"""

from dataclasses import dataclass


def cutin_vtype_xml(dist_id="cutin", bins=None, v_ign=15.0, base_attrs=None):
    """bins: [(p_ignore, timegap_minor, impatience, weight)] — 구간마다 vType 하나.

    v_ign은 접근하는 차량 속도 범위(예: 8~14 m/s)보다 커야 한다. 기본값 0이면 멈춘 차량만 무시한다.
    """
    bins = bins or default_bins()
    attrs = base_attrs or 'length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0"'
    out = []
    for i, (p, tg, imp, w, *_) in enumerate(bins):
        out.append(f'<vType id="{dist_id}_{i}" {attrs} jmIgnoreFoeProb="{p}" '
                   f'jmIgnoreFoeSpeed="{v_ign}" jmTimegapMinor="{tg}" impatience="{imp}"/>')
    ids = " ".join(f'{dist_id}_{i}' for i in range(len(bins)))
    probs = " ".join(str(b[3]) for b in bins)
    out.append(f'<vTypeDistribution id="{dist_id}" vTypes="{ids}" probabilities="{probs}"/>')
    return "\n".join(out)


def default_bins(n=6):
    """INTERACTION 합류 간격 분포 자리표시: 공격성이 커지는 n개 구간 (균등 가중)."""
    return [(round(0.2 + 0.8 * i / (n - 1), 3), round(1.0 - 0.7 * i / (n - 1), 3),
             round(i / (n - 1), 3), round(1.0 / n, 4)) for i in range(n)]


def cutin_bins_once(n=6, aggression=None):
    """끼어들기 진입 판정을 차량당 한 번(= 합류 상호작용당 한 번)으로 만드는 구간.

    SUMO의 jmIgnoreFoeProb는 접근 중 여러 번 뽑혀 실제 진입 확률이 p가 아니라 1-(1-p)^k가 된다(검증으로 확인, k≈5).
    그래서 vType의 무시 확률을 0 또는 1로만 두고, '무시형' vType의 비율을 시나리오 확률 p_ignore(a)로 맞춘다.
    구간(공격성 a_i)마다 vType 두 개(무시형, 비무시형)가 생기고, 투입 때 vTypeDistribution이 한 번 뽑는다.
    5번째 값은 그 구간의 공격성 a다.
    """
    from .common import AGGRESSION
    ag = aggression or AGGRESSION
    bins = []
    for i in range(n):
        a = i / (n - 1)
        p = min(1.0, max(0.0, ag.p_ignore(a)))
        tg, imp = round(1.0 - 0.7 * a, 3), round(a, 3)
        if p > 0:
            bins.append((1.0, tg, imp, round(p / n, 5), a))
        if p < 1:
            bins.append((0.0, tg, imp, round((1 - p) / n, 5), a))
    return bins


def parking_area_xml(pid, lane, start, end, capacity=1):
    """S4: 도로 위에 남은 채 정차하는 주차 구역 (onRoad=true, roadsideCapacity만 사용)."""
    return (f'<parkingArea id="{pid}" lane="{lane}" startPos="{start}" endPos="{end}" '
            f'onRoad="true" roadsideCapacity="{capacity}"/>')


def taxi_flow_xml(fid, route, parking_id, duration, prob, vtype="taxi", begin=0, end=3600):
    return (f'<flow id="{fid}" type="{vtype}" route="{route}" begin="{begin}" end="{end}" '
            f'probability="{prob}"><stop parkingArea="{parking_id}" duration="{duration}"/></flow>')


# ---- S4 일반화: 정차 차량(택시·버스·일반 차량), 위치·시각·정차 시간·확률을 설정으로 받는다 ----------------------------------

# 정차 차량 유형 -> (vType id, vType XML, 길이 m). "car"는 일반 배경 차량 분포(carDist)를 쓴다(vType XML 없음).
STOPPED_KINDS = {
    "taxi": ("stop_taxi", '<vType id="stop_taxi" vClass="taxi" length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0"/>', 4.5),
    "bus": ("stop_bus", '<vType id="stop_bus" vClass="bus" length="12" width="2.5" accel="1.2" decel="4.0" tau="1.0" '
                        'sigma="0" guiShape="bus" color="0,120,255"/>', 12.0),
    "car": ("stop_car", None, 4.5),     # 일반 승용차. vType은 호출 쪽이 배경 차량 속성(car_attrs)으로 만든다
}
DEFAULT_CAR_ATTRS = 'length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0" vClass="passenger"'


@dataclass
class StoppedSpec:
    """도로(차로) 위에 서는 정차 차량 한 대. parkingArea onRoad=true라 서 있는 동안 그 차로가 막힌다.

    kind     : "taxi" | "bus" | "car"(일반 배경 차량 분포)
    edge/lane: 서는 도로 id와 차로 번호
    pos      : 정차 구역 시작 위치 [m]. 차 길이에 맞춰 구역 끝을 정한다
    depart   : 투입 시각 [s]. 서는 시각은 그 뒤 도착하는 즉시다
    duration : 정차 시간 [s]
    prob     : 이 정차가 일어날 확률(빌드 때 한 번 뽑는다). 1이면 항상
    """
    kind: str
    edge: str
    lane: int
    pos: float
    depart: float
    duration: float = 30.0
    prob: float = 1.0
    sid: str = None


def stopped_vehicle_xml(spec, index=0, car_attrs=None):
    """(vType XML 조각 목록, 차량·경로 XML 조각 목록, parkingArea XML)을 돌려준다.

    car_attrs: "car" 유형의 vType 속성 문자열(배경 차량과 같은 물리 특성을 주려고 쓴다).
    """
    if spec.kind not in STOPPED_KINDS:
        raise ValueError(f"정차 차량 유형은 {sorted(STOPPED_KINDS)} 중 하나여야 한다: {spec.kind}")
    vt_id, vt_xml, length = STOPPED_KINDS[spec.kind]
    sid = spec.sid or f"stopped{index}"
    pa_id, rid = f"pa_{sid}", f"r_{sid}"
    start = float(spec.pos)
    end = start + max(10.0, length + 4.0)
    pa = parking_area_xml(pa_id, f"{spec.edge}_{spec.lane}", start, end, 1)
    veh = (f'<route id="{rid}" edges="{spec.edge}"/>'
           f'<vehicle id="{sid}" type="{vt_id}" route="{rid}" depart="{spec.depart}" departLane="{spec.lane}" '
           f'departSpeed="max"><stop parkingArea="{pa_id}" duration="{spec.duration}"/></vehicle>')
    if spec.kind == "car":
        vt_xml = f'<vType id="{vt_id}" {car_attrs or DEFAULT_CAR_ATTRS} color="0.3,0.5,0.9"/>'
    return [vt_xml], [veh], pa


class CutinTypeSwitch:
    """끼어들기 vType(앞차 무시 확률 1 등)은 합류 골목 안에서만 쓴다. 그 밖에서는 보통 차 vType으로 바꾼다.

    jmIgnoreFoeProb는 차뿐 아니라 보행자도 무시하게 한다. 그래서 끼어들기 vType으로 계속 달리면 골목에서 나온 뒤 지나는 신호
    교차로 횡단보도에서도 보행자를 무시해 부딪혔다(baseline 3바퀴 측정 2건, cluster_J52_J56_J58). 골목(feeders) 안에 있거나
    합류 중(교차로 내부)일 때만 원래 끼어들기 vType이고, 합류를 마치거나 아직 골목에 닿지 않았으면 normal_type이다.
    update(t)를 매 스텝 부른다(runner의 extras).
    """

    def __init__(self, traci, feeders, normal_type="car_krauss", prefix="cutin_", period=0.2):
        self.tc, self.feeders, self.normal = traci, set(feeders), normal_type
        self.prefix, self.period = prefix, period
        self.cut = {}            # 차 id -> 원래 끼어들기 vType
        self.state = {}          # 차 id -> "out"(골목 밖) | "in"(골목 안·합류 중)
        self._started, self._t_next = False, -1e9
        self.switches = 0

    def _set(self, vid, vtype):
        try:
            self.tc.vehicle.setType(vid, vtype)
            self.switches += 1
        except Exception:
            pass

    def _register(self, vid):
        vt = self.tc.vehicle.getTypeID(vid).split("@")[0]
        if not vt.startswith(self.prefix):
            return
        self.cut[vid] = vt
        if self.tc.vehicle.getRoadID(vid) in self.feeders:
            self.state[vid] = "in"
        else:
            self.state[vid] = "out"
            self._set(vid, self.normal)

    def update(self, t):
        tc = self.tc
        if not self._started:                      # 예열 중에 나온 차도 맡는다
            self._started = True
            for vid in tc.vehicle.getIDList():
                self._register(vid)
        for vid in tc.simulation.getDepartedIDList():
            if vid not in self.state:
                self._register(vid)
        if t < self._t_next:
            return
        self._t_next = t + self.period
        ids = set(tc.vehicle.getIDList())
        for vid in list(self.state):
            if vid not in ids:
                del self.state[vid], self.cut[vid]
                continue
            road = tc.vehicle.getRoadID(vid)
            if self.state[vid] == "out" and road in self.feeders:
                self._set(vid, self.cut[vid])                   # 골목에 들어왔다: 버스 앞으로 끼어든다
                self.state[vid] = "in"
            elif self.state[vid] == "in" and not road.startswith(":") and road not in self.feeders:
                self._set(vid, self.normal)                     # 합류를 마쳤다: 다시 보통 차
                del self.state[vid], self.cut[vid]
