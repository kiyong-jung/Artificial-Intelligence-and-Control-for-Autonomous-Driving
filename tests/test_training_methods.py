"""Cross-method invariants; no trained checkpoint or SUMO process required."""
import numpy as np
import pytest
import torch
from algorithms.ppo import PPO
from training.hapjeong.residual_env import ResidualEnv, initialize_zero
from training.hapjeong.train import PRESETS
from training.hapjeong.task import check_model_config

def test_zero_residual_and_all_lane_categories():
    agent = PPO(95, 2, **PRESETS['residual']['hparams'])
    initialize_zero(agent)
    for lane in (-1, 0, 1):
        base = np.array([.25, lane], dtype=np.float32)
        action = agent.predict(np.zeros(95, dtype=np.float32), True)
        np.testing.assert_array_equal(ResidualEnv.compose(base, action), base)
        assert {int(ResidualEnv.compose(base, [0, offset])[1]) for offset in (-1, 0, 1)} == {-1, 0, 1}

def test_residual_logprob_uses_sampled_action():
    agent = PPO(95, 2, **PRESETS['residual']['hparams'])
    initialize_zero(agent)
    obs = np.zeros(95, dtype=np.float32)
    action, logp, _ = agent.act(obs)
    value, _, _ = agent.policy.evaluate_actions(torch.tensor(obs)[None], torch.tensor(action)[None])
    assert value.item() == pytest.approx(logp, abs=1e-5)

def test_wrong_observation_profile_rejected():
    with pytest.raises(ValueError):
        check_model_config({'ego': {}, 'observation': {}})

def test_selected_schedule_not_one_stage_substitution():
    assert PRESETS['bc_ppo_initial']['steps'] == 32768
    assert PRESETS['bc_ppo_refine']['steps'] == 65536
    assert PRESETS['bc_ppo_initial']['hparams']['lr'] == 1e-4
    assert PRESETS['bc_ppo_refine']['hparams']['lr'] == 3e-5
    assert PRESETS['bc_ppo_refine']['hparams']['n_epochs'] == 5
