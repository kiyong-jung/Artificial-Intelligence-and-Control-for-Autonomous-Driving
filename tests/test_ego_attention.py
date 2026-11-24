"""Ego-attention 정책: layout, 마스크, 순서 불변, PPO 연결, BC 스크립트(SUMO 없이)."""
import json
import os
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from algorithms.ppo import PPO  # noqa: E402
from env.bus_task import LAYOUT  # noqa: E402
from utils.ego_attention import LAYOUTS, EgoAttentionHybridActorCritic  # noqa: E402


def _obs(layout, rng, n=4):
    """layout에 맞는 무작위 관측. 각 묶음의 앞쪽 절반 token만 present."""
    lay = LAYOUTS[layout]
    x = rng.uniform(-1, 1, (n, lay["dim"])).astype(np.float32)
    for g in lay["groups"]:
        for k in range(g["count"]):
            s = g["start"] + k * g["size"]
            x[:, [s + i for i in g["presence"]]] = 1.0 if k < g["count"] // 2 else 0.0
    return x


def test_obs93_layout_matches_bus_task():
    lay = LAYOUTS["obs93"]
    assert lay["groups"][0]["start"] == LAYOUT["nearest_vehicles_8_x_mask_xy_vxy"][0]
    assert lay["groups"][1]["start"] == LAYOUT["nearest_pedestrians_6_x_mask_xy_vxy"][0]
    assert lay["ego"] == [[0, 6], [LAYOUT["navigation_left_right_uturn_distance_remaining"][0], 93]]


def test_features130_layout_matches_feature_names():
    from pdm_data.features import FEATURE_NAMES
    lay = LAYOUTS["features130"]
    assert lay["dim"] == len(FEATURE_NAMES)
    covered = sorted(sum([list(range(a, b)) for a, b in lay["ego"]], []) +
                     [g["start"] + i for g in lay["groups"] for i in range(g["count"] * g["size"])])
    assert covered == list(range(len(FEATURE_NAMES)))
    names = {g["name"]: FEATURE_NAMES[g["start"]] for g in lay["groups"]}
    assert names == {"lane_slot": "slot_leader_present", "foe": "foe0_present", "pedestrian_feat": "ped0_present"}
    assert FEATURE_NAMES[76 + 1] == "foe0_phantom"
    assert [FEATURE_NAMES[i] for i in lay["lane_flags"]] == ["lc_could_left", "lc_could_right"]


@pytest.mark.parametrize("layout", ["obs93", "features130", "both"])
def test_shapes_and_interface(layout):
    rng = np.random.default_rng(0)
    net = EgoAttentionHybridActorCritic(LAYOUTS[layout]["dim"], {"layout": layout, "self_attention": True})
    x = torch.as_tensor(_obs(layout, rng))
    mu, std, logits, v = net(x)
    assert mu.shape == (4, 1) and logits.shape == (4, 3) and v.shape == (4,) and std.shape == (1,)
    a, logp, _ = net.sample(x)
    logp2, ent, _ = net.evaluate_actions(x, a)
    torch.testing.assert_close(logp, logp2)
    assert set(net.det_action(x)[:, 1].tolist()) <= {-1.0, 0.0, 1.0}
    att = net.attention_matrix(x)
    assert att.shape == (4, len(net.entities))
    torch.testing.assert_close(att.sum(-1), torch.ones(4))


def test_absent_entities_are_ignored_and_order_invariant():
    torch.manual_seed(0)
    rng = np.random.default_rng(1)
    net = EgoAttentionHybridActorCritic(93).eval()
    x = _obs("obs93", rng, 1)
    base = net(torch.as_tensor(x))
    y = x.copy()
    y[0, 6 + 7 * 5 + 1:6 + 8 * 5] = 0.3          # 없는 차량(마지막 slot)의 내용을 바꾼다
    z = x.copy()
    v = z[0, 6:46].reshape(8, 5)
    z[0, 6:46] = v[[2, 0, 3, 1, 4, 5, 6, 7]].reshape(-1)   # present 차량 순서를 섞는다
    for other in (y, z):
        out = net(torch.as_tensor(other))
        for p, q in zip(base, out):
            torch.testing.assert_close(p, q, atol=1e-5, rtol=1e-5)
    att = net.attention_matrix(torch.as_tensor(x))[0]
    absent = [i for i, n in enumerate(net.entities) if n in {f"vehicle{k}" for k in range(4, 8)}]
    assert float(att[absent].sum()) < 1e-6


def test_lane_action_mask_blocks_unavailable_side():
    net = EgoAttentionHybridActorCritic(93, {"lane_action_mask": True}, lane_keep_bias=-5.0)
    x = torch.as_tensor(_obs("obs93", np.random.default_rng(2), 64))
    x[:, 2] = 0.0   # 왼쪽 차로 없음
    x[:, 4] = 1.0
    for _ in range(5):
        a, _, _ = net.sample(x)
        assert (a[:, 1] != 1.0).all()


def test_ppo_ego_attention_save_load(tmp_path):
    hp = dict(policy_type="ego_attention", attention={"heads": 2, "feature_size": 32, "embed_layers": [32]},
              device="cpu", seed=3)
    agent = PPO(93, 2, **hp)
    agent.save(str(tmp_path / "m.pt"))
    other = PPO(93, 2, **json.loads(json.dumps(hp)))
    other.load(str(tmp_path / "m.pt"))
    obs = _obs("obs93", np.random.default_rng(3), 1)[0]
    np.testing.assert_array_equal(agent.predict(obs, True), other.predict(obs, True))
    action, logp, value = agent.act(obs)
    assert action.shape == (2,) and np.isfinite(logp) and np.isfinite(value)


def _fake_npz(path, rng, episodes, rows=40):
    n = episodes * rows
    obs = _obs("obs93", rng, n)
    obs[:, 1:6] = rng.integers(0, 2, (n, 5))
    feats = _obs("features130", rng, n)
    lane = rng.choice([-1.0, 0.0, 1.0], n, p=[0.1, 0.8, 0.1]).astype(np.float32)
    act = np.stack([np.clip(obs[:, 0] * 0.5, -1, 1), lane], 1).astype(np.float32)
    ep = np.repeat(np.arange(episodes), rows).astype(np.int32)
    t = np.tile(np.arange(rows), episodes).astype(np.int32)
    term = t == rows - 1
    np.savez_compressed(path, observation=obs, next_observation=np.roll(obs, -1, 0), action_raw=act,
                        reward=rng.normal(0, 0.1, n).astype(np.float32), terminated=term,
                        truncated=np.zeros(n, bool), episode=ep, t=t, features=feats,
                        next_features=np.roll(feats, -1, 0))


@pytest.mark.parametrize("extra,episodes", [([], 4), (["--input", "both", "--mask-obs-indices", "1,2,3,4,5"], 1)])
def test_bc_script_smoke(tmp_path, extra, episodes):
    from scripts.train_ego_attention_bc import main
    rng = np.random.default_rng(4)
    files = []
    for i in range(2):
        files.append(tmp_path / f"run{i}" / "processed" / "transitions.npz")
        files[-1].parent.mkdir(parents=True)
        _fake_npz(files[-1], rng, episodes)
    out = tmp_path / "out"
    main(["--data", str(files[0].parents[1]), str(files[1]), "--out", str(out), "--epochs", "2",
          "--feature-size", "16", "--heads", "2", "--embed-layers", "16", "--output-layers", "16"] + extra)
    cfg = json.loads((out / "run_config.json").read_text(encoding="utf-8"))
    audit = json.loads((out / "audit.json").read_text(encoding="utf-8"))
    assert audit["split"]["mode"] == ("episode" if episodes >= 2 else "time_block")
    assert audit["final"]["test"]["rows"] > 0
    agent = PPO(cfg["obs_dim"], 2, **cfg["hparams"])
    agent.load(str(out / "model.pt"))
    if extra:
        assert cfg["ego"]["observation_mask_indices"] == [1, 2, 3, 4, 5]
