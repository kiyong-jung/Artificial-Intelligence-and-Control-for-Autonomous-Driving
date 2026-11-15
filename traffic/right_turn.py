"""신호 교차로의 우회전: 직진·좌회전은 신호를 따르지만, 우회전은 신호와 무관하게 갈 수 있게 한다. 단 우회전 차가 지나는
횡단보도에 사람이 없을 때만 연다.

확인 결과(합정역 3.6.0 도로망의 신호 프로그램을 직접 읽음): 신호 교차로의 우회전 링크가 주기의 54~69% 동안 적색(r)이다.
즉 우회전도 신호를 기다린다. 그래서 시작할 때 신호 프로그램에서 우회전 링크를 적색·황색 위상에서도 'g'(녹색, 양보하며 진행)로
바꾼다. 어느 위상에서 열지는 **신호 프로그램만으로** 정한다(런타임에 링크 상태를 바꾸지 않는다).

우회전 링크를 위상에서 'g'로 여는 조건 (둘 다 만족해야 한다):
1. 겹치는(`getInternalFoes`) 다른 차량 링크가 모두 적색 또는 황색이다. 이 도로망의 신호 교차로 링크는 서로 양보하도록 만들어져
   있지 않아, 직진·좌회전 녹색과 겹치게 열면 교차로 안에서 부딪힌다(처음 시도에서 J74·J82 충돌 13건).
2. 우회전 차가 지나는 횡단보도의 보행자 신호가 적색이다. 보행자 신호가 녹색인 위상에는 사람이 건너고 있을 수 있으므로 열지
   않는다. 그래서 "횡단보도에 사람이 없을 때만" 열린다.

처음에는 런타임 보호 장치(`setLinkState`로 보행자가 있으면 그 링크만 적색으로 되돌림)를 썼으나 합정역에서 교차로가 막혀
(순간이동 30건, 시험용 버스 R73 정지) 없앴다.

- 도로망 파일은 건드리지 않는다. TraCI로 신호 프로그램만 바꾼다(주기·위상 길이는 그대로).
- `exclude`의 교차로(기본 `J1`, 합정역 사거리)는 바꾸지 않는다. J1은 중앙 정류장 우회 차로 B1·B2가 닿는 합정역 앞 교차로다.
"""
import re

CROSSING = re.compile(r"^:.+_c\d+_\d+$")


def right_turn_links(tls, directions=("r", "R")):
    """sumolib 신호(TLS)에서 우회전 링크의 신호 인덱스 집합."""
    out = set()
    for in_lane, out_lane, idx in tls.getConnections():
        for c in in_lane.getEdge().getConnections(out_lane.getEdge()):
            if c.getFromLane() == in_lane and c.getToLane() == out_lane and c.getDirection() in directions:
                out.add(idx)
    return out


def _foes(traci, tid, links):
    """우회전 링크 인덱스 -> (겹치는 차량 링크 인덱스 집합, 겹치는 횡단보도 링크 인덱스 집합)."""
    ctrl = traci.trafficlight.getControlledLinks(tid)
    via_veh, via_cross = {}, {}
    for i, group in enumerate(ctrl):
        if not group:
            continue
        if CROSSING.match(group[0][1]):               # 횡단보도 링크: via가 비어 있고 두 번째 칸이 횡단보도 차로(:J96_c1_0)다
            via_cross[group[0][1]] = i
        elif group[0][2]:
            via_veh[group[0][2]] = i
    out = {}
    for i in links:
        if i >= len(ctrl) or not ctrl[i]:
            continue
        v = ctrl[i][0][2]
        foes = traci.lane.getInternalFoes(v) if v else ()
        out[i] = ({via_veh[f] for f in foes if f in via_veh and via_veh[f] != i},
                  {via_cross[f] for f in foes if f in via_cross})
    return out


def apply_right_on_red(traci, net, exclude=("J1",), foe_ok="ry"):
    """exclude 말고 신호가 있는 모든 교차로에서 우회전을 가능한 위상에서 'g'로 바꾼다.

    foe_ok: 겹치는 차량 링크가 이 상태들뿐일 때 연다("r" 적색, "y" 황색). 직진·좌회전 녹색("G", "g")이 겹치면 열지 않는다.
    겹치는 횡단보도의 보행자 신호는 반드시 적색이어야 한다.

    반환: {신호 id: {"links": 우회전 링크 수, "opened_share": 원래 적색·황색이던 시간 중 열린 비율,
                    "blocked_by_ped": 보행자 신호 때문에 열지 못한 시간 비율}}.
    """
    changed = {}
    for tls in net.getTrafficLights():
        tid = tls.getID()
        if tid in exclude:
            continue
        links = right_turn_links(tls)
        if not links:
            continue
        foes = _foes(traci, tid, links)
        total = opened = by_ped = 0.0
        for logic in traci.trafficlight.getAllProgramLogics(tid):
            origs = [p.state for p in logic.phases]            # 바꾸기 전 상태(직전 위상 확인용)
            for pi, phase in enumerate(logic.phases):
                orig = origs[pi]
                prev = origs[pi - 1]                           # 직전 위상(처음이면 마지막 위상 = 주기가 이어진다)
                s = list(orig)
                for i in links:
                    if i >= len(orig) or orig[i] not in "ryu":
                        continue
                    total += phase.duration
                    veh, cross = foes.get(i, (set(), set()))
                    if not all(orig[j] in foe_ok for j in veh):
                        continue                                     # 직진·좌회전 녹색과 겹친다
                    # 보행자 신호가 이 위상과 직전 위상 모두 적색이어야 한다. 막 적색이 된 직후에는 사람이 아직 건너고 있다.
                    if not all(orig[c] == "r" and prev[c] == "r" for c in cross if c < len(orig)):
                        by_ped += phase.duration
                        continue
                    s[i] = "g"
                    opened += phase.duration
                phase.state = "".join(s)
            traci.trafficlight.setProgramLogic(tid, logic)
        changed[tid] = {"links": len(links), "opened_share": round(opened / total, 2) if total else 0.0,
                        "blocked_by_ped": round(by_ped / total, 2) if total else 0.0,
                        "orig": origs}               # 바꾸기 전 위상별 상태(충돌이 우리가 연 위상에서 났는지 가리는 데 쓴다)
    return changed
