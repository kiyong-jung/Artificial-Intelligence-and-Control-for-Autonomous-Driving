"""공통 실행 루프 (계획 "공통 기반: 실행 구조").

한 스텝(0.1 s)의 순서: 1 관측 수집 -> 2 버스 훅 -> 3 주변 교통 갱신(A1) -> 4 simulationStep -> 5 지표 기록.
환경은 버스 차량에 명령을 내리지 않는다. 버스 제어 측은 bus_hook(traci, t)로만 끼어든다.
"""


def run(traci, t_end, bus_id, bus_hook=None, yield_mgr=None, metrics=None, dt=0.1,
        on_step=None):
    """bus_hook: callable(traci, t) 또는 None. 반환: 마지막 시각."""
    t, step = 0.0, 0
    order = []
    while t < t_end - 1e-9:
        if yield_mgr is not None:
            yield_mgr.on_departed(traci.simulation.getDepartedIDList())   # 1 관측: 투입 차량 유형 확정
        if bus_hook is not None:
            bus_hook(traci, t)                                             # 2 버스 훅
            order.append("hook")
        if yield_mgr is not None:
            yield_mgr.update(t)                                            # 3 A1 갱신
            order.append("update")
        traci.simulationStep()                                             # 4
        step += 1
        t = round(step * dt, 6)
        if metrics is not None:
            metrics.step(t)                                                # 5 지표
            order.append("metrics")
        if on_step is not None:
            on_step(t)
    return t
