"""3값 차로 MLP 하이브리드 정책(utils/mlp_policy.py)."""
import json
import os
import sys

import pytest
import torch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from utils.mlp_policy import (MLPHybridPolicy, build_mlp_policy, lane_class_of, lane_label_of,  # noqa: E402
                              load_mlp_policy_dir)


def test_mlp_shapes_and_det_action():
    net = MLPHybridPolicy(33, {"lane_classes": 3, "accel_bins": 21})
    mu, std, logits = net(torch.zeros(4, 33))
    assert mu.shape == (4, 1) and logits.shape == (4, 3) and torch.all(std > 0)
    a = net.det_action(torch.zeros(4, 33))
    assert a.shape == (4, 2)
    assert set(a[:, 1].tolist()) <= {1.0, 0.0, -1.0}
    assert torch.all(a[:, 1] == 0.0)                       # 초기에는 유지(keep 편향)
    assert net.n_lane == 3 and net.keep == 1 and net.n_accel == 21


@pytest.mark.parametrize("bins", [0, 21])
def test_mlp_evaluate_actions_finite(bins):
    net = MLPHybridPolicy(33, {"accel_bins": bins})
    obs = torch.randn(2, 33).clamp(-1, 1)
    logp, ent = net.evaluate_actions(obs, torch.tensor([[0.3, 1.0], [-0.5, -1.0]]))
    assert logp.shape == (2,) and ent.shape == (2,)
    assert torch.isfinite(logp).all() and torch.isfinite(ent).all()
    act, lp = net.sample(obs)
    assert act.shape == (2, 2) and torch.isfinite(lp).all()


def test_build_and_load_roundtrip(tmp_path):
    network = {"policy_type": "mlp", "mlp": {"hidden_layers": [32, 32], "lane_classes": 3, "accel_bins": 21},
               "init_log_std": -2.0, "lane_keep_bias": 0.5}
    torch.manual_seed(0)
    net = build_mlp_policy(33, network)
    torch.nn.init.normal_(net.lane_head.weight)            # 결정적 출력이 초기값만이 아니게
    (tmp_path / "run_config.json").write_text(json.dumps({"obs_dim": 33, "network": network}), encoding="utf-8")
    torch.save(net.state_dict(), tmp_path / "model.pt")
    loaded, cfg = load_mlp_policy_dir(str(tmp_path))
    obs = torch.rand(5, 33) * 2 - 1
    assert torch.equal(net.eval().det_action(obs), loaded.det_action(obs))
    assert cfg["obs_dim"] == 33 and not loaded.training


def test_load_rejects_non_mlp(tmp_path):
    (tmp_path / "run_config.json").write_text(json.dumps({"obs_dim": 33, "network": {"attention": {}}}),
                                              encoding="utf-8")
    with pytest.raises(ValueError):
        load_mlp_policy_dir(str(tmp_path))


def test_lane_label_roundtrip():
    assert lane_label_of(1.0) == 1 and lane_label_of(-1.0) == -1 and lane_label_of(0.0) == 0
    assert lane_class_of(torch.tensor([1.0, 0.0, -1.0])).tolist() == [0, 1, 2]
