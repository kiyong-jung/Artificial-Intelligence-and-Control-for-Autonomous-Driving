"""BC 정책의 두 출력 방식. bins는 갈리는 정답 중 한쪽을 고르고, mse는 그 중간값을 낸다."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
torch = pytest.importorskip("torch")
from algorithms.bc import BCPolicy  # noqa: E402

STATE_DIM = 4


def fit(model, states, actions, steps=400):
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-2)
    for _ in range(steps):
        optimizer.zero_grad()
        model.loss(states, actions).backward()
        optimizer.step()
    return model


def conflicting_labels():
    """같은 관측에 정답이 둘이다: 열 번 중 일곱 번은 세게 줄이고(-1), 세 번은 가속한다(+0.6). 황색 신호 앞과 같다."""
    states = torch.zeros(100, STATE_DIM)
    actions = torch.tensor([[-1.0, 0.0]] * 70 + [[0.6, 0.0]] * 30)
    return states, actions


def test_mse_answers_between_two_conflicting_labels():
    torch.manual_seed(0)
    model = fit(BCPolicy(STATE_DIM), *conflicting_labels())
    assert model.predict(np.zeros(STATE_DIM))[0] == pytest.approx(-0.52, abs=0.05)


@pytest.mark.parametrize("decode", ["mode", "local"])
def test_bins_pick_the_more_common_of_two_conflicting_labels(decode):
    torch.manual_seed(0)
    model = fit(BCPolicy(STATE_DIM, head="bins", decode=decode), *conflicting_labels())
    assert model.predict(np.zeros(STATE_DIM))[0] == pytest.approx(-1.0, abs=0.02)


def test_bins_with_mean_decoding_still_average():
    torch.manual_seed(0)
    model = fit(BCPolicy(STATE_DIM, head="bins", decode="mean"), *conflicting_labels())
    assert model.predict(np.zeros(STATE_DIM))[0] == pytest.approx(-0.52, abs=0.05)


@pytest.mark.parametrize("decode", ["mean", "local"])
def test_bins_reproduce_a_value_between_two_bin_centres(decode):
    """정답 -0.37은 칸 중심(-0.4, -0.2) 사이다. 양옆 두 칸에 나눠 배우므로 -0.4로 뭉개지지 않는다."""
    torch.manual_seed(0)
    states, actions = torch.zeros(64, STATE_DIM), torch.tensor([[-0.37, 0.0]] * 64)
    model = fit(BCPolicy(STATE_DIM, head="bins", decode=decode), states, actions)
    assert model.predict(np.zeros(STATE_DIM))[0] == pytest.approx(-0.37, abs=0.02)


def test_bins_learn_the_lane_change_as_one_of_three_choices():
    """관측 첫 값이 차선변경 방향을 정한다. 출력은 정확히 -1, 0, +1 중 하나다."""
    torch.manual_seed(0)
    lanes = torch.tensor([-1.0, 0.0, 1.0]).repeat(40)
    states = torch.zeros(120, STATE_DIM)
    states[:, 0] = lanes
    model = fit(BCPolicy(STATE_DIM, head="bins"), states, torch.stack([torch.zeros(120), lanes], dim=1))
    for lane in (-1.0, 0.0, 1.0):
        action = model.predict(np.array([lane, 0.0, 0.0, 0.0]))
        assert action.shape == (2,) and action[1] == lane and -1.0 <= action[0] <= 1.0


def test_a_saved_bins_model_drives_the_same_after_loading(tmp_path):
    torch.manual_seed(0)
    model = fit(BCPolicy(STATE_DIM, head="bins", accel_bins=9, decode="local"), *conflicting_labels(), steps=50)
    model.save(tmp_path / "model.pt")
    loaded = BCPolicy.load(tmp_path / "model.pt")
    obs = np.array([0.3, -0.2, 0.1, 0.9], dtype=np.float32)
    assert (loaded.head, loaded.accel_bins, loaded.decode) == ("bins", 9, "local")
    assert np.array_equal(loaded.predict(obs), model.predict(obs))


def test_a_checkpoint_from_before_the_bins_head_loads_as_mse(tmp_path):
    torch.manual_seed(0)
    model = BCPolicy(STATE_DIM)
    torch.save({"algorithm": "bc", "state_dim": STATE_DIM, "action_dim": 2, "state_dict": model.state_dict()},
               tmp_path / "old.pt")
    loaded = BCPolicy.load(tmp_path / "old.pt")
    obs = np.array([0.3, -0.2, 0.1, 0.9], dtype=np.float32)
    assert loaded.head == "mse" and np.array_equal(loaded.predict(obs), model.predict(obs))


def test_an_unknown_head_is_refused(tmp_path):
    model = BCPolicy(STATE_DIM)
    torch.save({"algorithm": "bc", "state_dim": STATE_DIM, "action_dim": 2, "head": "mixture",
                "state_dict": model.state_dict()}, tmp_path / "model.pt")
    with pytest.raises(ValueError, match="head"):
        BCPolicy.load(tmp_path / "model.pt")


def test_thinning_keeps_the_start_and_end_of_each_standstill():
    """에피소드 0은 2판단 달리고 6판단 서고 1판단 달린다. 에피소드 1은 서서 시작한다(3판단). 처음과 마지막 2개씩을
    남기면 긴 정지의 가운데 2개만 빠지고, 에피소드가 바뀌면 정지 구간도 새로 센다."""
    from train_bc import standstill_middles
    speed = np.array([0.5, 0.4, 0, 0, 0, 0, 0, 0, 0.3, 0, 0, 0], dtype=np.float32)
    obs = np.stack([speed, np.zeros_like(speed)], axis=1)
    episodes = np.array([0] * 9 + [1] * 3)
    assert np.flatnonzero(standstill_middles(obs, episodes, keep=2)).tolist() == [4, 5]
    assert np.flatnonzero(standstill_middles(obs, episodes, keep=1)).tolist() == [3, 4, 5, 6, 10]


def test_a_feature_with_infinite_spread_does_not_reach_the_network():
    """train_bc.py --blind는 그 항목의 정규화 표준편차를 무한대로 둔다. 그 값이 달라져도 행동이 같다."""
    torch.manual_seed(0)
    model = BCPolicy(STATE_DIM, head="bins")
    model.obs_std[3] = float("inf")
    seen = model.predict(np.array([0.3, -0.2, 0.1, 0.9], dtype=np.float32))
    assert np.array_equal(seen, model.predict(np.array([0.3, -0.2, 0.1, -0.5], dtype=np.float32)))
    assert not np.array_equal(model(torch.tensor([0.3, -0.2, 0.1, 0.9])), model(torch.tensor([0.9, -0.2, 0.1, 0.9])))
