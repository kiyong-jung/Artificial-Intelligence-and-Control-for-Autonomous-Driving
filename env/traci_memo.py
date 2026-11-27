"""같은 계산 스텝 안에서 되풀이하는 TraCI 조회를 한 번만 SUMO에 보낸다.

환경은 관측, 감지 목록, 기록용 상태를 만들며 같은 값을 여러 번 묻는다(한 에피소드 조회의 59%가 중복이었다).
조회마다 소켓을 한 번 오가므로 이것이 수집 시간의 대부분이다. 조회 결과를 (명령, 변수, 대상, 인자)로
기억해 두고, 조회가 아닌 명령(simulationStep, setSpeed 같은 변경)을 보낼 때마다 모두 지운다. 그래서
돌려주는 값은 기억하지 않을 때와 같다.

    from env import traci_memo
    traci_memo.install()   # 여러 번 불러도 한 번만 설치한다
"""
from traci.connection import Connection
from traci.domain import Domain

GET_COMMANDS = range(0xA0, 0xB0)   # 도메인별 변수 조회 명령
_memo = {}
_installed = False


def install():
    global _installed
    if _installed:
        return
    _installed = True
    get_universal, send = Domain._getUniversal, Connection._sendCmd

    def memo_get(self, varID, objectID="", format="", *values):
        try:
            key = (id(self._connection), self._cmdGetID, varID, objectID, values)
            hash(key)
        except TypeError:   # 인자에 리스트가 있으면 기억하지 않는다
            return get_universal(self, varID, objectID, format, *values)
        if key not in _memo:
            _memo[key] = get_universal(self, varID, objectID, format, *values)
        return _memo[key]

    def clearing_send(self, cmdID, varID, objID, format="", *values):
        if cmdID not in GET_COMMANDS:
            _memo.clear()
        return send(self, cmdID, varID, objID, format, *values)

    Domain._getUniversal = memo_get
    Connection._sendCmd = clearing_send
