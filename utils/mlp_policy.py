"""3값 차로 MLP 하이브리드 정책(행동 복제 전용). 관측 33차원(pdm_data/obs33.py)용.

utils/ego_attention.py의 EgoAttentionHybridPolicy와 같은 공개 메서드·속성을 갖지만 그 모듈을 import하지 않는다(rl-agents 의존 없음).
출력: 가감속(accel_bins 0이면 tanh 평균 + 학습 log_std Normal, K면 [-1, 1]을 K개 중심으로 나눈 Categorical)과 차로 3클래스.
차로 클래스 0 = 왼쪽(+1), 1 = 유지(0), 2 = 오른쪽(-1). 행동 벡터의 차로 값은 LANE_CMDS_3이다.
"""
from __future__ import annotations

import json
import os

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical, Normal

LANE_CMDS_3 = (1.0, 0.0, -1.0)
DEFAULT_MLP = {"hidden_layers": [256, 256], "activation": "RELU", "lane_classes": 3, "accel_bins": 0}
ACTIVATIONS = {"RELU": nn.ReLU, "TANH": nn.Tanh, "ELU": nn.ELU}


def accel_centers(k):
    return torch.linspace(-1.0, 1.0, k)


def accel_bin_of(value, k):
    """정규화 가감속 [-1, 1] -> 가장 가까운 구간 번호(0 = -1)."""
    return torch.round((value.clamp(-1.0, 1.0) + 1.0) * (k - 1) / 2.0).long()


def lane_class_of(cmd):
    """행동 벡터의 차로 값(+1, 0, -1) -> 클래스 번호(0, 1, 2)."""
    return torch.round(1.0 - cmd).long().clamp(0, 2)


def lane_label_of(cmd) -> int:
    """행동 벡터의 차로 값 -> 차로 라벨 {+1, 0, -1}."""
    return int(round(float(cmd)))


def _orthogonal(module, gain):
    nn.init.orthogonal_(module.weight, gain=gain)
    nn.init.constant_(module.bias, 0.0)
    return module


class MLPHybridPolicy(nn.Module):
    def __init__(self, obs_dim: int, mlp: dict = None, init_log_std=-0.5, lane_keep_bias: float = 1.0):
        super().__init__()
        cfg = {**DEFAULT_MLP, **(mlp or {})}
        self.cfg = cfg
        self.n_lane = int(cfg["lane_classes"])
        if self.n_lane != 3:
            raise ValueError("MLP 정책은 3값 차로(lane_classes=3)만 쓴다")
        self.keep = 1
        layers, last = [], int(obs_dim)
        for h in cfg["hidden_layers"]:
            layers += [nn.Linear(last, int(h)), ACTIVATIONS[cfg["activation"]]()]
            last = int(h)
        self.body = nn.Sequential(*layers)
        self.n_accel = int(cfg.get("accel_bins") or 0)
        if self.n_accel == 1:
            raise ValueError("accel_bins는 0(연속) 또는 2 이상")
        if self.n_accel:
            self.accel_head = _orthogonal(nn.Linear(last, self.n_accel), 0.01)
            self.register_buffer("accel_values", accel_centers(self.n_accel))
        else:
            self.mu_head = _orthogonal(nn.Linear(last, 1), 0.01)
        self.lane_head = _orthogonal(nn.Linear(last, self.n_lane), 0.01)
        with torch.no_grad():
            self.lane_head.bias[self.keep] = lane_keep_bias
        if not np.isscalar(init_log_std):
            init_log_std = float(init_log_std[0])
        self.log_std = nn.Parameter(torch.ones(1) * float(init_log_std))
        self.register_buffer("lane_cmds", torch.tensor(LANE_CMDS_3))

    def encode(self, obs):
        return self.body(obs)

    def lane_logits(self, z, obs=None):
        return self.lane_head(z)

    def accel_logits(self, z):
        return self.accel_head(z)

    def forward(self, obs):
        z = self.encode(obs)
        if self.n_accel:
            mu = (torch.softmax(self.accel_head(z), dim=-1) * self.accel_values).sum(-1, keepdim=True)
        else:
            mu = torch.tanh(self.mu_head(z))
        return mu, torch.exp(self.log_std), self.lane_logits(z, obs)

    def _dists(self, obs):
        z = self.encode(obs)
        lane = Categorical(logits=self.lane_logits(z, obs))
        if self.n_accel:
            return Categorical(logits=self.accel_head(z)), lane
        return Normal(torch.tanh(self.mu_head(z)), torch.exp(self.log_std)), lane

    def sample(self, obs):
        ad, cd = self._dists(obs)
        lane_idx = cd.sample()
        if self.n_accel:
            a_idx = ad.sample()
            accel, log_a = self.accel_values[a_idx].unsqueeze(-1), ad.log_prob(a_idx)
        else:
            accel = ad.sample()
            log_a = ad.log_prob(accel).sum(-1)
        return torch.cat([accel, self.lane_cmds[lane_idx].unsqueeze(-1)], dim=-1), log_a + cd.log_prob(lane_idx)

    def det_action(self, obs):
        """결정적 행동 [정규화 가감속, 차로 명령(+1, 0, -1)]."""
        ad, cd = self._dists(obs)
        accel = self.accel_values[ad.logits.argmax(-1)].unsqueeze(-1) if self.n_accel else ad.mean
        return torch.cat([accel, self.lane_cmds[torch.argmax(cd.logits, dim=-1)].unsqueeze(-1)], dim=-1)

    def evaluate_actions(self, obs, actions):
        ad, cd = self._dists(obs)
        lane_idx = lane_class_of(actions[:, 1])
        if self.n_accel:
            log_a, ent_a = ad.log_prob(accel_bin_of(actions[:, 0], self.n_accel)), ad.entropy()
        else:
            log_a, ent_a = ad.log_prob(actions[:, :1]).sum(-1), ad.entropy().sum(-1)
        return log_a + cd.log_prob(lane_idx), ent_a + cd.entropy()

    def lane_entropy(self, obs):
        return Categorical(logits=self.forward(obs)[2]).entropy()

    def lane_change_prob(self, obs):
        return 1.0 - torch.softmax(self.forward(obs)[2], dim=-1)[:, self.keep]

    @property
    def current_std(self) -> float:
        return 0.0 if self.n_accel else float(torch.exp(self.log_std).mean().item())


NETWORK_KEYS = ("mlp", "init_log_std", "lane_keep_bias")


def build_mlp_policy(obs_dim: int, network: dict, device="cpu") -> MLPHybridPolicy:
    """network = run_config.json의 "network"(policy_type "mlp", mlp, init_log_std, lane_keep_bias)."""
    return MLPHybridPolicy(obs_dim, **{k: network[k] for k in NETWORK_KEYS if k in network}).to(device)


def load_mlp_policy_dir(model_dir, device="cpu"):
    """학습 출력 폴더(run_config.json + model.pt) -> (eval 모드 정책, run_config)."""
    with open(os.path.join(model_dir, "run_config.json"), encoding="utf-8") as f:
        cfg = json.load(f)
    network = cfg.get("network") or {}
    if network.get("policy_type") != "mlp":
        raise ValueError(f"MLP 정책이 아닙니다: {model_dir}")
    net = build_mlp_policy(cfg["obs_dim"], network, device)
    net.load_state_dict(torch.load(os.path.join(model_dir, "model.pt"), map_location=device, weights_only=True))
    return net.eval(), cfg
