"""Residual RL: 전문가 규칙(env/route_policy.ExpertPolicy)의 행동에 학습한 가감속 보정을 더한다.

최종 행동 = [clip(전문가 가감속 + scale × 정책 가감속, -1, 1), 전문가 차선변경]

차선변경은 전문가가 정한다. 정책의 차선 출력은 쓰지 않는다. 학습 초반의 무작위 차선변경이 주행을 망가뜨리지 않고,
정책은 신호·정류장·앞차 사이의 가감속만 다듬는다. 합성은 관측만으로 정해지므로 학습 환경과 평가에서 같다.
"""
import numpy as np

from env.route_policy import make_expert


def compose(expert_action, residual_action, scale):
    action = np.asarray(expert_action, dtype=np.float32).copy()
    action[0] = np.clip(action[0] + scale * float(np.asarray(residual_action).reshape(-1)[0]), -1.0, 1.0)
    return action


class ResidualActionEnv:
    """SumoHighwayEnv를 감싸 정책 행동을 전문가 행동의 보정으로 바꾼다. 나머지 속성은 안쪽 환경에 넘긴다."""

    def __init__(self, env, scale):
        self.env = env
        self.scale = float(scale)
        self.expert = make_expert(env)
        self._obs = None

    def __getattr__(self, name):
        return getattr(self.__dict__["env"], name)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self._obs = obs
        return obs, info

    def step(self, action):
        expert_action = self.expert(self._obs)
        obs, reward, terminated, truncated, info = self.env.step(compose(expert_action, action, self.scale))
        info["expert_accel_raw"] = float(expert_action[0])
        info["residual_accel_raw"] = float(np.asarray(action).reshape(-1)[0])
        self._obs = obs
        return obs, reward, terminated, truncated, info

    def close(self):
        self.env.close()


class ResidualPolicy:
    """학습한 보정 정책(predict(obs, deterministic))을 평가용 정책으로 바꾼다."""

    def __init__(self, agent, env, scale):
        self.agent, self.scale = agent, float(scale)
        self.expert = make_expert(env)

    def predict(self, obs, deterministic=True):
        return compose(self.expert(obs), self.agent.predict(obs, deterministic=deterministic), self.scale)
