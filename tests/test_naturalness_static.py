"""배경 교통을 억지로 움직이거나 순간이동·충돌을 감추는 코드와 설정이 새로 들어오면 실패한다.

scripts/audit_naturalness.py의 정적 검사만 돈다(SUMO를 돌리지 않는다). 새로 찾은 것이 정당하면 그 스크립트의
RULES 표에 이유와 함께 ALLOW로 적고, 문제면 고친다.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import audit_naturalness as audit  # noqa: E402


@pytest.fixture(scope="module")
def static():
    return audit.run_static()


def test_every_risky_command_and_setting_is_in_the_table(static):
    hits, _stale = static
    unknown = [f"{h['file']}:{h.get('lines', '')} [{h['where']}] {h['what']}({h['target']}) {h.get('detail', '')}"
               for h in hits if h["verdict"] == "UNKNOWN"]
    assert unknown == [], "허용표(scripts/audit_naturalness.py RULES)에 없는 것:\n" + "\n".join(unknown)


def test_the_table_has_no_line_that_matches_nothing(static):
    _hits, stale = static
    assert stale == [], "고쳐서 없어진 것은 RULES에서 지운다"


def test_the_scanner_catches_planted_bad_code_and_settings():
    missing, wrongly_allowed, hits = audit.self_check()
    assert missing == set()
    assert wrongly_allowed == []
    assert len(hits) >= 18


def test_a_command_to_a_background_vehicle_in_the_training_env_is_not_allowed():
    hit = dict(kind="call", file="env/sumo_env.py", where="SumoHighwayEnv.step", what="vehicle.setSpeed", target="vid")
    assert audit.judge(hit)[0] == "UNKNOWN"
    ego = dict(hit, target="self.ego_id")
    assert audit.judge(ego)[0] == "ALLOW"
