"""Ego-attention 하이브리드 Actor-Critic (rl-agents EgoAttention 재사용).

ref/rl-agents의 EgoAttention·SelfAttention·MultiLayerPerceptron을 그대로 쓰고, 버스 관측 벡터를
엔티티 타입별 token으로 나누는 부분과 하이브리드 출력(가감속 Normal + 차선 Categorical + V)만 이 파일에 둔다.
rl-agents의 EgoAttentionNetwork는 ego와 다른 엔티티가 같은 특징 차원이어야 하지만, 버스 관측은 타입마다
특징이 다르다(차량 5개, 보행자 11개 등). 그래서 타입마다 embedding MLP를 따로 두고 같은 feature_size로 맞춘다
(SUMO_AV_input_설계.md "엔티티 타입별 token에 ego-attention" 권고).

입력 layout
    obs93       : pdm_data/obs93.py, env/bus_task.py의 93차원 관측(data_colum.md observation)
                  ego = 속도·주행 가능 차로(6) + 경로(5) + 신호(6) + 정류장(6) = 23
                  token = 차량 8개 x [presence, x, y, vx, vy], 보행자 6개 x 같은 5개
    features130 : pdm_data/features.py의 입력 설계 특징(SUMO_AV_input_설계.md 권장 벡터)
                  ego = ego(7) + 차선 변경 가능(2) + 경로(9) + 정류장(7) + 신호(10) + 안전 여유(5) = 40
                  token = lane-slot 6개 x 6, 골목·교차 차량 3개 x 7(phantom 포함), 보행자 3개 x 11
    both        : obs93 뒤에 features130을 이어 붙인 223차원

출력 인터페이스는 utils/networks.py의 HybridActorCritic과 같다. 그래서 algorithms/ppo.py가 그대로 쓴다.
    forward(obs) -> (mu, std, lane_logits, value)
    sample / det_action / evaluate_actions / lane_entropy / lane_change_prob / current_std
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical, Normal

_RL_AGENTS = Path(__file__).resolve().parents[1] / "ref" / "rl-agents"
try:
    from rl_agents.agents.common.models import EgoAttention, MultiLayerPerceptron, SelfAttention
except ImportError:  # 설치하지 않았으면 저장소 안의 ref/rl-agents를 쓴다
    sys.path.insert(0, str(_RL_AGENTS))
    from rl_agents.agents.common.models import EgoAttention, MultiLayerPerceptron, SelfAttention


def _group(name, start, count, size, presence=(0,)):
    return {"name": name, "start": start, "count": count, "size": size, "presence": list(presence)}


# ego: [start, end) 구간 목록. groups: 같은 크기 token이 연속으로 놓인 묶음. presence: 하나라도 0.5 이상이면 있는 token.
# lane_flags: (좌, 우) 차선 변경 가능 비트의 위치. lane_action_mask를 켤 때만 쓴다.
LAYOUTS = {
    "obs93": {
        "dim": 93,
        "ego": [[0, 6], [76, 93]],
        "groups": [_group("vehicle", 6, 8, 5), _group("pedestrian", 46, 6, 5)],
        "lane_flags": [2, 4],       # 주행 가능 차로 offset (+2, +1, 0, -1, -2) 중 +1(좌), -1(우)
    },
    "features130": {
        "dim": 130,
        "ego": [[0, 35], [71, 76]],
        "groups": [_group("lane_slot", 35, 6, 6), _group("foe", 76, 3, 7, presence=(0, 1)),
                   _group("pedestrian_feat", 97, 3, 11)],
        "lane_flags": [7, 8],       # lc_could_left, lc_could_right
    },
}
LAYOUTS["both"] = {
    "dim": 93 + 130,
    "ego": LAYOUTS["obs93"]["ego"] + [[a + 93, b + 93] for a, b in LAYOUTS["features130"]["ego"]],
    "groups": LAYOUTS["obs93"]["groups"] + [dict(g, start=g["start"] + 93) for g in LAYOUTS["features130"]["groups"]],
    "lane_flags": LAYOUTS["obs93"]["lane_flags"],
}
INPUT_LAYOUT = {"observation": "obs93", "features": "features130", "both": "both"}

DEFAULT_ATTENTION = {
    "layout": "obs93",
    "feature_size": 64,
    "heads": 4,
    "embed_layers": [64],          # 타입별 embedding: in -> embed_layers -> feature_size
    "output_layers": [64, 64],     # attention 결과 -> output_layers -> 각 head
    "self_attention": False,       # True면 ego-attention 앞에 SelfAttention 한 층
    "dropout": 0.0,
    "activation": "RELU",
    "lane_action_mask": False,     # True면 차선 변경 불가 쪽 로짓을 막는다(hapjeong FullTask는 그 비트를 0으로 지우므로 끈다)
}


def layout_entities(layout: dict) -> list:
    """attention 행렬 열 순서의 엔티티 이름. [ego, vehicle0, vehicle1, ...]"""
    return ["ego"] + [f"{g['name']}{k}" for g in layout["groups"] for k in range(g["count"])]


def _mlp(in_dim, layers, activation):
    return MultiLayerPerceptron({"in": in_dim, "layers": list(layers), "activation": activation, "reshape": False,
                                 "out": None})


def _orthogonal(module, gain):
    nn.init.orthogonal_(module.weight, gain=gain)
    nn.init.constant_(module.bias, 0.0)
    return module


class EgoAttentionHybridActorCritic(nn.Module):
    # 차선 인덱스 ↔ 명령: 0=왼쪽(+1), 1=유지(0), 2=오른쪽(-1). HybridActorCritic과 같다.
    LANE_CMDS = (1.0, 0.0, -1.0)

    def __init__(self, obs_dim: int, attention: dict = None, init_log_std=-0.5, lane_keep_bias: float = 1.0):
        super().__init__()
        cfg = {**DEFAULT_ATTENTION, **(attention or {})}
        self.cfg = cfg
        self.layout = LAYOUTS[cfg["layout"]]
        if obs_dim != self.layout["dim"]:
            raise ValueError(f"layout {cfg['layout']}는 {self.layout['dim']}차원 입력이 필요한데 obs_dim={obs_dim}")
        f = int(cfg["feature_size"])
        if f % int(cfg["heads"]):
            raise ValueError("feature_size는 heads로 나누어떨어져야 한다")
        act = cfg["activation"]
        self.register_buffer("ego_index", torch.as_tensor(
            np.concatenate([np.arange(a, b) for a, b in self.layout["ego"]]), dtype=torch.long))
        self.ego_embedding = _mlp(len(self.ego_index), list(cfg["embed_layers"]) + [f], act)
        self.embeddings = nn.ModuleList(_mlp(g["size"], list(cfg["embed_layers"]) + [f], act)
                                        for g in self.layout["groups"])
        att = {"feature_size": f, "heads": int(cfg["heads"]), "dropout_factor": float(cfg["dropout"])}
        self.self_attention = SelfAttention(dict(att, type="SelfAttention")) if cfg["self_attention"] else None
        self.ego_attention = EgoAttention(dict(att, type="EgoAttention"))
        self.output = _mlp(f, cfg["output_layers"], act)
        last = cfg["output_layers"][-1] if cfg["output_layers"] else f
        self.mu_head = _orthogonal(nn.Linear(last, 1), 0.01)
        self.lane_head = _orthogonal(nn.Linear(last, 3), 0.01)
        self.v_head = _orthogonal(nn.Linear(last, 1), 1.0)
        with torch.no_grad():
            self.lane_head.bias[1] = lane_keep_bias
        if not np.isscalar(init_log_std):
            init_log_std = float(init_log_std[0])
        self.log_std = nn.Parameter(torch.ones(1) * float(init_log_std))
        self.register_buffer("lane_cmds", torch.tensor(self.LANE_CMDS))
        self.entities = layout_entities(self.layout)

    # ---- token 만들기 --------------------------------------------------
    def tokens(self, obs):
        """(ego (B,1,F), others (B,N,F), mask (B,1+N) — True면 없는 엔티티)"""
        b = obs.shape[0]
        ego = self.ego_embedding(obs[:, self.ego_index].unsqueeze(1))
        others, absent = [], []
        for g, emb in zip(self.layout["groups"], self.embeddings):
            raw = obs[:, g["start"]:g["start"] + g["count"] * g["size"]].reshape(b, g["count"], g["size"])
            others.append(emb(raw))
            absent.append(raw[:, :, g["presence"]].max(dim=-1).values < 0.5)
        mask = torch.cat([torch.zeros(b, 1, dtype=torch.bool, device=obs.device)] + absent, dim=1)
        return ego, torch.cat(others, dim=1), mask

    def attend(self, obs):
        """(attention 결과 (B,F), attention 행렬 (B,heads,1,1+N))"""
        ego, others, mask = self.tokens(obs)
        if self.self_attention is not None:
            mixed, _ = self.self_attention(ego, others, mask)
            ego, others = mixed[:, :1], mixed[:, 1:]
        return self.ego_attention(ego, others, mask)

    def encode(self, obs):
        z, _ = self.attend(obs)
        return self.output(z)

    def lane_logits(self, z, obs):
        logits = self.lane_head(z)
        if self.cfg["lane_action_mask"]:
            left, right = self.layout["lane_flags"]
            blocked = torch.stack([obs[:, left] < 0.5, torch.zeros_like(obs[:, left], dtype=torch.bool),
                                   obs[:, right] < 0.5], dim=-1)
            logits = logits.masked_fill(blocked, -1e9)
        return logits

    def heads(self, z, obs):
        mu = torch.tanh(self.mu_head(z))
        return mu, torch.exp(self.log_std), self.lane_logits(z, obs), self.v_head(z).squeeze(-1)

    def forward(self, obs):
        return self.heads(self.encode(obs), obs)

    # ---- algorithms/ppo.py가 쓰는 인터페이스 ----------------------------
    def sample(self, obs):
        mu, std, logits, value = self.forward(obs)
        nd, cd = Normal(mu, std), Categorical(logits=logits)
        accel, lane_idx = nd.sample(), cd.sample()
        log_prob = nd.log_prob(accel).sum(-1) + cd.log_prob(lane_idx)
        return torch.cat([accel, self.lane_cmds[lane_idx].unsqueeze(-1)], dim=-1), log_prob, value

    def det_action(self, obs):
        mu, _, logits, _ = self.forward(obs)
        return torch.cat([mu, self.lane_cmds[torch.argmax(logits, dim=-1)].unsqueeze(-1)], dim=-1)

    def evaluate_actions(self, obs, actions):
        mu, std, logits, value = self.forward(obs)
        nd, cd = Normal(mu, std), Categorical(logits=logits)
        lane_idx = (1.0 - torch.round(actions[:, 1])).long().clamp(0, 2)
        log_prob = nd.log_prob(actions[:, :1]).sum(-1) + cd.log_prob(lane_idx)
        return log_prob, nd.entropy().sum(-1) + cd.entropy(), value

    def lane_entropy(self, obs):
        return Categorical(logits=self.forward(obs)[2]).entropy()

    def lane_change_prob(self, obs):
        return 1.0 - torch.softmax(self.forward(obs)[2], dim=-1)[:, 1]

    @property
    def current_std(self) -> float:
        return float(torch.exp(self.log_std).mean().item())

    @torch.no_grad()
    def attention_matrix(self, obs):
        """진단용: 헤드 평균 attention (B, 1+N). 열 순서는 self.entities."""
        _, att = self.attend(obs)
        return att.mean(dim=1).squeeze(1)
