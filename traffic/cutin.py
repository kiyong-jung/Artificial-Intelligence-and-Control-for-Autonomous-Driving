"""A2. 끼어들기형(S5) vType 분포와 정류장 앞 정차 차량(S4) 정의.

끼어들기는 차선변경이 아니라 우선 교차로(priority junction)의 우선권 위반으로 구현한다.
jm* 파라미터는 vType 단위로만 설정되므로 분포를 구간으로 나눈 vTypeDistribution을 쓴다.
"""


def cutin_vtype_xml(dist_id="cutin", bins=None, v_ign=15.0, base_attrs=None):
    """bins: [(p_ignore, timegap_minor, impatience, weight)] — 구간마다 vType 하나.

    v_ign은 접근하는 차량 속도 범위(예: 8~14 m/s)보다 커야 한다. 기본값 0이면 멈춘 차량만 무시한다.
    """
    bins = bins or default_bins()
    attrs = base_attrs or 'length="4.5" accel="2.6" decel="4.5" tau="1.0" sigma="0"'
    out = []
    for i, (p, tg, imp, w) in enumerate(bins):
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


def parking_area_xml(pid, lane, start, end, capacity=1):
    """S4: 도로 위에 남은 채 정차하는 주차 구역 (onRoad=true, roadsideCapacity만 사용)."""
    return (f'<parkingArea id="{pid}" lane="{lane}" startPos="{start}" endPos="{end}" '
            f'onRoad="true" roadsideCapacity="{capacity}"/>')


def taxi_flow_xml(fid, route, parking_id, duration, prob, vtype="taxi", begin=0, end=3600):
    return (f'<flow id="{fid}" type="{vtype}" route="{route}" begin="{begin}" end="{end}" '
            f'probability="{prob}"><stop parkingArea="{parking_id}" duration="{duration}"/></flow>')
