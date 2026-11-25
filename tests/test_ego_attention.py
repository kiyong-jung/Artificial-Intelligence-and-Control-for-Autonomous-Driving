"""Ego-attention 정책: layout, 마스크, 순서 불변, 저장·불러오기, BC 스크립트(SUMO 없이)."""
import json
import os
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from env.bus_task import LAYOUT  # noqa: E402
from utils.ego_attention import LAYOUTS, EgoAttentionHybridPolicy, load_policy_dir  # noqa: E402


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
    net = EgoAttentionHybridPolicy(LAYOUTS[layout]["dim"], {"layout": layout, "self_attention": True})
    x = torch.as_tensor(_obs(layout, rng))
    mu, std, logits = net(x)
    assert mu.shape == (4, 1) and logits.shape == (4, 3) and std.shape == (1,)
    assert not any(n.startswith("v_head") for n in net.state_dict())
    a, logp = net.sample(x)
    logp2, ent = net.evaluate_actions(x, a)
    torch.testing.assert_close(logp, logp2)
    assert set(net.det_action(x)[:, 1].tolist()) <= {-1.0, 0.0, 1.0}
    att = net.attention_matrix(x)
    assert att.shape == (4, len(net.entities))
    torch.testing.assert_close(att.sum(-1), torch.ones(4))


def test_absent_entities_are_ignored_and_order_invariant():
    torch.manual_seed(0)
    rng = np.random.default_rng(1)
    net = EgoAttentionHybridPolicy(93).eval()
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
    net = EgoAttentionHybridPolicy(93, {"lane_action_mask": True}, lane_keep_bias=-5.0)
    x = torch.as_tensor(_obs("obs93", np.random.default_rng(2), 64))
    x[:, 2] = 0.0   # 왼쪽 차로 없음
    x[:, 4] = 1.0
    for _ in range(5):
        a, _ = net.sample(x)
        assert (a[:, 1] != 1.0).all()


def test_load_policy_dir_reads_legacy_output_with_v_head(tmp_path):
    """critic을 두던 때의 출력(run_config "hparams", model.pt에 v_head)도 읽는다."""
    attention = {"heads": 2, "feature_size": 32, "embed_layers": [32]}
    net = EgoAttentionHybridPolicy(93, attention).eval()
    state = dict(net.state_dict(), **{"v_head.weight": torch.zeros(1, 64), "v_head.bias": torch.zeros(1)})
    torch.save(state, tmp_path / "model.pt")
    (tmp_path / "run_config.json").write_text(json.dumps(
        {"obs_dim": 93, "hparams": {"policy_type": "ego_attention", "attention": attention, "lr": 1e-4}}))
    loaded, _ = load_policy_dir(tmp_path)
    x = torch.as_tensor(_obs("obs93", np.random.default_rng(3), 4))
    torch.testing.assert_close(net.det_action(x), loaded.det_action(x))


def test_lane_label_of():
    from utils.ego_attention import LANE_CMDS, lane_label_of
    assert [lane_label_of(c, 5) for c in LANE_CMDS[5]] == [2, 1, 0, -1, -2]
    assert [lane_label_of(c, 3) for c in LANE_CMDS[3]] == [1, 0, -1]


def _fake_npz(path, rng, episodes, rows=40):
    n = episodes * rows
    obs = _obs("obs93", rng, n)
    obs[:, 1:6] = rng.integers(0, 2, (n, 5))
    feats = _obs("features130", rng, n)
    lane = rng.choice([-1.0, 0.0, 1.0], n, p=[0.1, 0.8, 0.1]).astype(np.float32)
    act = np.stack([np.clip(obs[:, 0] * 0.5, -1, 1), lane], 1).astype(np.float32)
    ep = np.repeat(np.arange(episodes), rows).astype(np.int32)
    t = np.tile(np.arange(rows), episodes).astype(np.int32)
    np.savez_compressed(path, observation=obs, action_raw=act, episode=ep, t=t, features=feats)


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
    assert "hparams" not in cfg and cfg["network"]["attention"]["heads"] == 2
    load_policy_dir(out)
    if extra:
        assert cfg["ego"]["observation_mask_indices"] == [1, 2, 3, 4, 5]


def test_v2_layout_matches_obs_v2_names():
    from pdm_data.obs_v2 import FEATURE_NAMES, OBS_V2_DIM
    lay = LAYOUTS["v2"]
    assert lay["dim"] == OBS_V2_DIM
    covered = sorted(sum([list(range(a, b)) for a, b in lay["ego"]], []) +
                     [g["start"] + i for g in lay["groups"] for i in range(g["count"] * g["size"])])
    assert covered == list(range(OBS_V2_DIM))
    assert {g["name"]: FEATURE_NAMES[g["start"]] for g in lay["groups"]} == {"vehicle": "veh0_presence",
                                                                              "pedestrian": "ped0_presence"}
    assert [FEATURE_NAMES[i] for i in lay["lane_flags"]] == ["left_lane_exists", "right_lane_exists"]


def test_v2_five_lane_classes_interface_and_mask():
    net = EgoAttentionHybridPolicy(166, {"layout": "v2", "lane_classes": 5, "lane_action_mask": True},
                                        lane_keep_bias=-5.0)
    x = torch.as_tensor(_obs("v2", np.random.default_rng(5), 64))
    x[:, 4], x[:, 5] = 0.0, 1.0          # 왼쪽 차로 없음, 오른쪽 있음
    mu, _, logits = net(x)
    assert logits.shape == (64, 5)
    for _ in range(5):
        a, logp = net.sample(x)
        assert set(a[:, 1].tolist()) <= {0.0, -0.5, -1.0}     # 왼쪽 방향지시등·변경(+0.5, +1)은 막힌다
        logp2, _ = net.evaluate_actions(x, a)
        torch.testing.assert_close(logp, logp2)
    assert set(net.det_action(x)[:, 1].tolist()) <= {1.0, 0.5, 0.0, -0.5, -1.0}


def _fake_npz_v2(path, rng, episodes, rows=40):
    n = episodes * rows
    obs = _obs("v2", rng, n)
    lane = rng.choice([-2.0, -1.0, 0.0, 1.0, 2.0], n, p=[0.05, 0.1, 0.7, 0.1, 0.05]).astype(np.float32)
    act = np.stack([np.clip(obs[:, 0] * 0.5, -1, 1), lane], 1).astype(np.float32)
    ep = np.repeat(np.arange(episodes), rows).astype(np.int32)
    t = np.tile(np.arange(rows), episodes).astype(np.int32)
    np.savez_compressed(path, observation_v2=obs, action_raw=act, prev_action_raw=np.roll(act, 1, 0), episode=ep, t=t)


def test_bc_script_v2_five_classes(tmp_path):
    from scripts.train_ego_attention_bc import main
    rng = np.random.default_rng(6)
    f = tmp_path / "run" / "processed" / "transitions.npz"
    f.parent.mkdir(parents=True)
    _fake_npz_v2(f, rng, 4)
    out = tmp_path / "out"
    main(["--data", str(f), "--out", str(out), "--input", "v2", "--epochs", "2", "--feature-size", "16", "--heads", "2",
          "--embed-layers", "16", "--output-layers", "16"])
    cfg = json.loads((out / "run_config.json").read_text(encoding="utf-8"))
    audit = json.loads((out / "audit.json").read_text(encoding="utf-8"))
    assert cfg["obs_dim"] == 166 and cfg["network"]["attention"]["lane_classes"] == 5
    assert len(audit["lane_counts_train"]) == 5 and "lane_move_recall" in audit["final"]["test"]
    net, _ = load_policy_dir(out)
    assert net.n_lane == 5


def test_accel_bins_interface_roundtrip():
    from utils.ego_attention import accel_bin_of
    net = EgoAttentionHybridPolicy(166, {"layout": "v2", "lane_classes": 5, "accel_bins": 11})
    x = torch.as_tensor(_obs("v2", np.random.default_rng(7), 16))
    a, logp = net.sample(x)
    grid = set(np.round(np.linspace(-1, 1, 11), 6).tolist())
    assert set(np.round(a[:, 0].numpy().astype(np.float64), 6).tolist()) <= grid
    logp2, ent = net.evaluate_actions(x, a)
    torch.testing.assert_close(logp, logp2)
    assert net.current_std == 0.0 and not hasattr(net, "mu_head")
    assert accel_bin_of(torch.tensor([-1.0, -0.04, 0.06, 1.0, 1.7]), 11).tolist() == [0, 5, 5, 10, 10]


def test_class_weights_schemes():
    from scripts.train_ego_attention_bc import class_weights
    c = torch.tensor([10.0, 1000.0, 0.0, 40.0])
    w = class_weights(c, "inverse", 50.0)
    assert w.tolist() == pytest.approx([50.0, 1.0, 0.0, 25.0])          # 100배는 상한 50
    assert class_weights(c, "sqrt", 50.0).tolist() == pytest.approx([10.0, 1.0, 0.0, 5.0])
    e = class_weights(c, "effective", 1e9, beta=0.999)
    assert e[1] == 1.0 and e[0] > e[3] > 1.0 and e[2] == 0.0
    assert class_weights(c, "none", 50.0).tolist() == [1.0, 1.0, 0.0, 1.0]


def _bc_args(f, out, epochs, *extra):
    return ["--data", str(f), "--out", str(out), "--input", "v2", "--epochs", str(epochs), "--feature-size", "16",
            "--heads", "2", "--embed-layers", "16", "--output-layers", "16", "--batch-size", "64", "--resume", *extra]


def test_bc_resume_matches_uninterrupted(tmp_path, monkeypatch):
    """4 epoch 실행이 3번째 epoch 학습 중에 죽었다가 --resume으로 이어 간 결과가 한 번에 끝낸 것과 같다."""
    import scripts.train_ego_attention_bc as bc
    f = tmp_path / "d" / "processed" / "transitions.npz"
    f.parent.mkdir(parents=True)
    _fake_npz_v2(f, np.random.default_rng(8), 4)
    extra = ("--lr-schedule", "cosine", "--weight-decay", "1e-4")
    bc.main(_bc_args(f, tmp_path / "full", 4, *extra))

    real, calls = bc.Trainer.train_epoch, []

    def dies_on_third(self, *args):
        calls.append(1)
        if len(calls) == 3:
            raise KeyboardInterrupt("중단 흉내")
        return real(self, *args)
    monkeypatch.setattr(bc.Trainer, "train_epoch", dies_on_third)
    with pytest.raises(KeyboardInterrupt):
        bc.main(_bc_args(f, tmp_path / "part", 4, *extra))
    monkeypatch.setattr(bc.Trainer, "train_epoch", real)
    assert not (tmp_path / "part" / "audit.json").exists()
    assert len((tmp_path / "part" / "history.jsonl").read_text().splitlines()) == 2
    bc.main(_bc_args(f, tmp_path / "part", 4, *extra))
    a = torch.load(tmp_path / "full" / "model.pt")
    b = torch.load(tmp_path / "part" / "model.pt")
    assert a.keys() == b.keys() and all(torch.equal(a[k], b[k]) for k in a)
    ha = [json.loads(x)["validation"]["bc_loss"] for x in (tmp_path / "full" / "history.jsonl").read_text().splitlines()]
    hb = [json.loads(x)["validation"]["bc_loss"] for x in (tmp_path / "part" / "history.jsonl").read_text().splitlines()]
    assert ha == pytest.approx(hb) and len(hb) == 4
    # TensorBoard: 멈췄다 이어 간 실행도 에폭마다 점 하나씩(1~4), 값은 한 번에 끝낸 것과 같다
    ea_mod = pytest.importorskip("tensorboard.backend.event_processing.event_accumulator")

    def scalars(run):
        acc = ea_mod.EventAccumulator(str(tmp_path / run / "tb"))
        acc.Reload()
        return [(e.step, e.value) for e in acc.Scalars("bc_loss/validation")]
    sa, sb = scalars("full"), scalars("part")
    assert [s for s, _ in sb] == [1, 2, 3, 4] and [v for _, v in sa] == pytest.approx([v for _, v in sb])


def test_bc_resume_rejects_changed_args_and_skips_done(tmp_path, capsys):
    from scripts.train_ego_attention_bc import main
    f = tmp_path / "d" / "processed" / "transitions.npz"
    f.parent.mkdir(parents=True)
    _fake_npz_v2(f, np.random.default_rng(9), 4)
    main(_bc_args(f, tmp_path / "o", 1))
    main(_bc_args(f, tmp_path / "o", 1))                       # 끝난 실행은 건너뛴다
    assert "건너뜁니다" in capsys.readouterr().out
    (tmp_path / "o" / "audit.json").unlink()
    with pytest.raises(ValueError, match="lr"):
        main(_bc_args(f, tmp_path / "o", 2, "--lr", "1e-3"))
    with pytest.raises(FileExistsError):
        main([a for a in _bc_args(f, tmp_path / "o", 2) if a != "--resume"])
    main(_bc_args(f, tmp_path / "o", 2))                       # --epochs는 늘려서 이어 갈 수 있다
    assert len((tmp_path / "o" / "history.jsonl").read_text().splitlines()) == 2
    main(_bc_args(f, tmp_path / "c", 1, "--lr-schedule", "cosine"))
    (tmp_path / "c" / "audit.json").unlink()
    with pytest.raises(ValueError, match="cosine"):             # cosine은 길이가 바뀌면 스케줄이 달라진다
        main(_bc_args(f, tmp_path / "c", 3, "--lr-schedule", "cosine"))
