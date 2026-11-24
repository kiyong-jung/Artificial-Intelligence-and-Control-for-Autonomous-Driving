"""46차원 관측(스키마 v3)으로 학습한 모델은 지금 환경에서 조용히 돌지 않고, 이유를 알리며 멈춘다."""
import glob
import os
import subprocess
import sys

import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, REPO)
torch = pytest.importorskip("torch")
from collect_pomdp_data import check_model_observation  # noqa: E402
from env.data_schema import OBSERVATION_FEATURES, ROUTE_FEATURES  # noqa: E402

def _observation_dim(path):
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    return int(checkpoint["state_dim"]) if checkpoint.get("algorithm") == "bc" else int(checkpoint["trunk.0.weight"].shape[1])


MODELS = sorted(glob.glob(os.path.join(REPO, "research", "av_bc_rl", "models", "*", "model.pt")))
OLD_MODELS = [path for path in MODELS if _observation_dim(path) == 46]


def test_the_observation_has_no_lane_instruction():
    assert len(OBSERVATION_FEATURES) == 43 and len(ROUTE_FEATURES) == 12
    assert not {"route_lane_offset", "route_lane_continuation", "stop_lane_offset"} & set(OBSERVATION_FEATURES)


@pytest.mark.parametrize("model", OLD_MODELS, ids=[os.path.basename(os.path.dirname(path)) for path in OLD_MODELS])
def test_models_trained_on_the_old_observation_are_refused(model):
    with pytest.raises(ValueError, match="46차원으로 학습한 모델"):
        check_model_observation(model, len(OBSERVATION_FEATURES))


def test_checkpoints_of_both_kinds_are_checked(tmp_path):
    bc, ppo = str(tmp_path / "bc.pt"), str(tmp_path / "ppo.pt")
    torch.save({"algorithm": "bc", "state_dim": 46, "action_dim": 2, "state_dict": {}}, bc)
    torch.save({"trunk.0.weight": torch.zeros(64, 46), "mu_head.weight": torch.zeros(1, 64)}, ppo)
    for path in (bc, ppo):
        with pytest.raises(ValueError, match="46차원"):
            check_model_observation(path, 43)
        check_model_observation(path, 46)


@pytest.mark.skipif(not OLD_MODELS, reason="저장소에 옛 모델이 없다")
def test_the_collector_stops_before_starting_sumo(tmp_path):
    result = subprocess.run([sys.executable, os.path.join(REPO, "collect_pomdp_data.py"), "--map", "hapjeong",
                             "--model", OLD_MODELS[0], "--episodes", "1", "--out-dir", str(tmp_path)],
                            cwd=REPO, capture_output=True, text=True)
    assert result.returncode != 0
    assert "46차원으로 학습한 모델" in result.stderr and "다시 학습" in result.stderr
    assert not any(name.endswith(".jsonl.gz") for name in os.listdir(tmp_path))
