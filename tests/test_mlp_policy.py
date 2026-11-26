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


def _fake_npz_33(path, episodes, rows=40, seed=7):
    import numpy as np
    rng = np.random.default_rng(seed)
    n = episodes * rows
    obs = rng.uniform(-1, 1, (n, 33)).astype(np.float32)
    lane = rng.choice([-1.0, 0.0, 1.0], n, p=[0.1, 0.8, 0.1]).astype(np.float32)
    act = np.stack([np.clip(obs[:, 0] * 0.5, -1, 1), lane], 1).astype(np.float32)
    ep = np.repeat(np.arange(episodes), rows).astype(np.int32)
    t = np.tile(np.arange(rows), episodes).astype(np.int32)
    raw = np.stack([act[:, 0], lane * 2], 1).astype(np.float32)     # 5값 열은 학습에 쓰이면 안 된다
    np.savez_compressed(path, observation_33=obs, action_33=act, action_raw=raw, prev_action_raw=np.roll(raw, 1, 0),
                        episode=ep, t=t)


def test_train_obs33_mlp_smoke(tmp_path):
    from scripts.train_ego_attention_bc import main
    files = []
    for i in range(2):
        f = tmp_path / f"run{i}" / "processed" / "transitions.npz"
        f.parent.mkdir(parents=True)
        _fake_npz_33(f, 3, seed=i)
        files.append(str(f))
    out = tmp_path / "out"
    main(["--data", *files, "--out", str(out), "--input", "obs33", "--policy", "mlp", "--hidden-layers", "32,32",
          "--epochs", "1", "--no-tensorboard"])
    cfg = json.loads((out / "run_config.json").read_text(encoding="utf-8"))
    audit = json.loads((out / "audit.json").read_text(encoding="utf-8"))
    assert cfg["input"] == "obs33" and cfg["obs_dim"] == 33 and cfg["network"]["policy_type"] == "mlp"
    assert len(audit["lane_counts_train"]) == 3
    for split in ("validation", "test"):                   # 3클래스도 이동 지표를 낸다(±1이 곧 이동)
        m = audit["final"][split]
        assert m["lane_move_recall"] == m["lane_change_recall"]
        assert m["lane_move_precision"] == m["lane_change_precision"]
        assert isinstance(m["keep_to_move"], int) and isinstance(m["wrong_side"], int)
    net, _ = load_mlp_policy_dir(str(out))
    assert net.n_lane == 3 and net.det_action(torch.zeros(1, 33)).shape == (1, 2)


def test_focal_loss_matches_weighted_ce_at_gamma0_and_downweights_easy():
    import torch.nn.functional as F
    from scripts.train_ego_attention_bc import focal_cross_entropy
    torch.manual_seed(0)
    logits, y = torch.randn(64, 3) * 3, torch.randint(0, 3, (64,))
    w = torch.tensor([5.0, 1.0, 5.0])
    assert torch.allclose(focal_cross_entropy(logits, y, w, 0.0), F.cross_entropy(logits, y, weight=w), atol=1e-6)
    easy = torch.tensor([[0.0, 8.0, 0.0]])                 # 맞게 확신한 표본은 gamma가 클수록 손실이 작다
    one = torch.tensor([1])
    assert focal_cross_entropy(easy, one, w, 2.0) < 0.01 * focal_cross_entropy(easy, one, w, 0.0)


def test_train_focal_reports_both_losses(tmp_path):
    from scripts.train_ego_attention_bc import main
    f = tmp_path / "run0" / "processed" / "transitions.npz"
    f.parent.mkdir(parents=True)
    _fake_npz_33(f, 3)
    out = tmp_path / "out"
    main(["--data", str(f), "--out", str(out), "--input", "obs33", "--policy", "mlp", "--hidden-layers", "16,16",
          "--lane-loss", "focal", "--focal-gamma", "2", "--epochs", "1", "--no-tensorboard"])
    m = json.loads((out / "audit.json").read_text(encoding="utf-8"))["final"]["validation"]
    assert m["lane_loss"] < m["lane_ce"]                   # focal 값은 같은 가중 CE보다 작다
    assert abs(m["bc_loss"] - (m["accel_loss"] + m["lane_loss"])) < 1e-6


def test_resume_accepts_args_added_later(tmp_path):
    from scripts.train_ego_attention_bc import main
    f = tmp_path / "t.npz"
    _fake_npz_33(f, 3)
    args = ["--data", str(f), "--out", str(tmp_path / "o"), "--input", "obs33", "--policy", "mlp",
            "--hidden-layers", "16,16", "--no-tensorboard", "--resume"]
    main(args + ["--epochs", "1"])
    ra = tmp_path / "o" / "run_args.json"
    saved = json.loads(ra.read_text(encoding="utf-8"))
    for k in ("lane_loss", "focal_gamma"):                 # 이 인자가 생기기 전에 시작한 실행
        saved["args"].pop(k)
    ra.write_text(json.dumps(saved), encoding="utf-8")
    (tmp_path / "o" / "audit.json").unlink()
    main(args + ["--epochs", "2"])


def test_obs33_requires_mlp(tmp_path):
    from scripts.train_ego_attention_bc import main
    f = tmp_path / "t.npz"
    _fake_npz_33(f, 2)
    with pytest.raises(SystemExit):
        main(["--data", str(f), "--out", str(tmp_path / "o"), "--input", "obs33", "--epochs", "1"])
