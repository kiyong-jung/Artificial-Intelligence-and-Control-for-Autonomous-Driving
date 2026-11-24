"""단일 시점의 부분관측으로 행동 벡터를 직접 회귀하는 BC 정책.

Behavior Cloning(행동 모방)은 수집한 관측과 행동의 짝을 정답 데이터로 삼아
관측 → 행동 관계를 지도학습하는 방법이다. 이 파일은 신경망과 추론 및 저장을
담당하고, 손실 계산과 가중치 갱신은 외부 학습 코드에서 수행한다.

기본 구조: 관측 → 정규화 → 은닉층 256 → 은닉층 256 → 출력
기본 행동: [가감속 raw, 차선변경 raw]
출력은 두 가지다(head).
- "mse": 두 행동을 연속값 2개로 낸다(수업 원본). 정답이 "세게 줄인다"와 "그대로 간다"로 갈리는 곳에서는
  그 중간값을 낸다.
- "bins": 가감속은 [-1, 1]을 같은 간격으로 나눈 칸 중 하나를, 차선변경은 {-1, 0, +1} 중 하나를 고른다.
  칸마다 점수(logit)를 내므로 갈리는 정답을 섞지 않고 한쪽을 고를 수 있다.
관측 이력을 기억하는 구조가 아니므로 현재 시점의 관측만으로 행동을 결정한다.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

HEADS = ("mse", "bins")
DECODES = ("mean", "mode", "local")
LANE_CLASSES = 3   # 차선변경 {-1, 0, +1}


class BCPolicy(nn.Module):
    """PyTorch가 가중치를 관리하고 역전파할 수 있는 행동 예측 신경망."""

    def __init__(self, state_dim, action_dim=2, head="mse", accel_bins=11, decode="mean"):
        """state_dim은 관측 길이, action_dim은 출력 행동 벡터의 길이이다.

        head="bins"이면 가감속 칸 accel_bins개와 차선변경 3개의 점수를 낸다. decode는 칸 점수를 가감속 값으로
        바꾸는 방법이다. "mean"은 확률로 가중한 평균, "mode"는 확률이 가장 높은 칸, "local"은 그 칸과 양옆
        칸만으로 낸 평균이다.
        """
        if head not in HEADS or decode not in DECODES:
            raise ValueError(f"head는 {HEADS}, decode는 {DECODES} 중 하나여야 합니다: head={head!r}, decode={decode!r}")
        if head == "bins" and (action_dim != 2 or accel_bins < 2):
            raise ValueError("bins head는 행동 [가감속, 차선변경] 2개와 칸 2개 이상이 필요합니다.")
        # nn.Module 초기화: 아래에서 정의하는 층과 buffer를 등록할 준비를 한다.
        super().__init__()

        # 여기서 state는 SUMO의 전체 내부 상태가 아니라 에이전트의 부분관측이다.
        # 입력 검증과 모델 복원에 사용하므로 입출력 차원을 보관한다.
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.head, self.accel_bins, self.decode = head, int(accel_bins), decode
        out_dim = action_dim if head == "mse" else self.accel_bins + LANE_CLASSES

        # 완전연결층(Linear)은 입력에 학습 가능한 가중치와 편향을 적용한다.
        # fc1: 관측을 256개 특징으로 변환.
        # fc2: 특징 사이의 관계를 조합하여 행동 결정에 필요한 표현을 학습.
        # fc3: 최종 특징을 action_dim개의 행동 값으로 변환.
        self.fc1 = nn.Linear(state_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, out_dim)

        # 관측 항목별 평균과 표준편차. 학습 코드에서 학습 데이터의 통계로 설정한다.
        # 초기값 0과 1은 정규화를 하더라도 입력이 그대로 유지되도록 한다.
        # buffer는 학습으로 갱신되는 파라미터가 아니지만 state_dict에 저장되고,
        # model.to(device)를 호출하면 신경망 가중치와 함께 장치를 이동한다.
        self.register_buffer("obs_mean", torch.zeros(state_dim))
        self.register_buffer("obs_std", torch.ones(state_dim))
        # 가감속 칸의 중심값. 양 끝 칸이 -1과 +1이다.
        self.register_buffer("bin_centers", torch.linspace(-1.0, 1.0, self.accel_bins), persistent=False)

    def forward(self, state):
        """단일 관측 (state_dim,) 또는 관측 배치 (N, state_dim)의 출력을 계산.

        head="mse"이면 행동 값, "bins"이면 [가감속 칸 점수, 차선변경 점수]다. 행동 값은 act()가 낸다."""
        # 학습과 추론에 동일한 통계를 사용하여 입력 항목의 스케일을 맞춘다.
        # 배치 입력에서는 관측 항목별 통계가 각 행에 자동으로 적용된다.
        # obs_std가 0이 되지 않도록 통계를 설정하는 학습 코드에서 처리해야 한다.
        state = (state - self.obs_mean) / self.obs_std

        # ReLU는 음수를 0으로 바꾸는 비선형 함수이다.
        # 층 사이에 비선형성을 넣어 단순한 선형 관계보다 복잡한 행동을 학습한다.
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))

        # 출력에는 tanh나 clipping이 없으므로 [-1, 1] 범위가 보장되지 않는다.
        # 환경에 행동을 전달할 때 필요한 범위 제한과 차선 명령 양자화를 적용한다.
        return self.fc3(x)

    def _two_hot(self, accel):
        """가감속 값을 양옆 두 칸에 거리 비율로 나눈 정답 분포 (N, accel_bins). 칸 중심값이면 그 칸 하나가 1이다.
        이 분포의 평균은 원래 값과 같아, 칸으로 나눠도 값이 뭉개지지 않는다."""
        position = (accel.clamp(-1.0, 1.0) + 1.0) * 0.5 * (self.accel_bins - 1)
        low = position.floor().clamp(max=self.accel_bins - 2).long()
        weight = (position - low).unsqueeze(1)
        target = torch.zeros(accel.shape[0], self.accel_bins, device=accel.device)
        target.scatter_(1, low.unsqueeze(1), 1.0 - weight)
        target.scatter_(1, low.unsqueeze(1) + 1, weight)
        return target

    def loss(self, state, target_action):
        """관측 배치와 정답 행동 (N, 2)의 학습 손실. mse는 평균제곱오차, bins는 가감속 칸과 차선변경의
        cross-entropy 합이다. 드문 행동에 가중치를 주지 않는다."""
        output = self(state)
        if self.head == "mse":
            return F.mse_loss(output, target_action)
        accel_logits, lane_logits = output[:, :self.accel_bins], output[:, self.accel_bins:]
        accel_loss = -(self._two_hot(target_action[:, 0]) * F.log_softmax(accel_logits, dim=1)).sum(dim=1).mean()
        lane_class = (torch.round(target_action[:, 1]).long() + 1).clamp(0, LANE_CLASSES - 1)
        return accel_loss + F.cross_entropy(lane_logits, lane_class)

    def act(self, state):
        """관측 배치 (N, state_dim)의 행동 (N, 2). bins는 칸 점수를 decode 방법으로 값으로 바꾼다."""
        output = self(state)
        if self.head == "mse":
            return output
        probs = torch.softmax(output[:, :self.accel_bins], dim=1)
        if self.decode == "mode":
            accel = self.bin_centers[probs.argmax(dim=1)]
        else:
            if self.decode == "local":
                near = (torch.arange(self.accel_bins, device=probs.device) - probs.argmax(dim=1, keepdim=True)).abs() <= 1
                probs = probs * near
                probs = probs / probs.sum(dim=1, keepdim=True)
            accel = (probs * self.bin_centers).sum(dim=1)
        lane = output[:, self.accel_bins:].argmax(dim=1).float() - 1.0
        return torch.stack([accel, lane], dim=1)

    # 추론에서는 역전파가 필요 없으므로 gradient 기록을 꺼 메모리 사용을 줄인다.
    @torch.no_grad()
    def predict(self, state, deterministic=True):
        """단일 관측을 받아 환경에 전달할 NumPy 행동 벡터를 반환한다.

        deterministic은 기존 정책 평가 코드와 인터페이스를 맞추기 위한 인자다.
        이 모델은 확률분포에서 행동을 뽑지 않으므로 False여도 같은 방식으로 예측한다.
        """
        # 평가 모드로 전환한다. eval() 자체가 gradient 계산을 끄는 것은 아니다.
        # 현재 구조에는 Dropout/BatchNorm이 없지만 추론 의도를 명확히 표시한다.
        self.eval()

        # 리스트나 NumPy 입력을 신경망 가중치와 같은 float32 자료형으로 통일한다.
        state = np.asarray(state, dtype=np.float32)

        # predict는 단일 관측 전용이다. 배치 학습은 forward를 직접 사용한다.
        # 관측 차원이 달라지면 엉뚱한 입력으로 추론하지 않도록 즉시 오류를 낸다.
        if state.shape != (self.state_dim,):
            raise ValueError(
                f"observation shape은 ({self.state_dim},)이어야 합니다."
            )

        # 입력도 모델과 같은 CPU/GPU 장치에 올려 장치 불일치 오류를 방지한다.
        state = torch.as_tensor(
            state,
            dtype=torch.float32,
            device=self.obs_mean.device,
        )

        action = self.act(state.unsqueeze(0))[0]

        # 환경은 NumPy 배열을 받으므로 CPU로 옮긴 뒤 변환한다.
        # 여기서는 출력값을 제한하지 않고 신경망의 회귀 결과를 그대로 반환한다.
        return action.cpu().numpy()

    def save(self, path):
        """모델 구조를 복원할 차원 정보, 가중치, 정규화 통계를 함께 저장한다."""
        # state_dict에는 fc1~fc3의 가중치/편향뿐 아니라 obs_mean/obs_std도 포함된다.
        # optimizer 상태는 저장하지 않으므로 학습 재개용 전체 상태 저장은 아니다.
        torch.save(
            {
                "algorithm": "bc",
                "state_dim": self.state_dim,
                "action_dim": self.action_dim,
                "head": self.head,
                "accel_bins": self.accel_bins,
                "decode": self.decode,
                "state_dict": self.state_dict(),
            },
            path,
        )

    @classmethod
    def load(cls, path, device="cpu"):
        """저장 파일로부터 새 BCPolicy 객체를 만들어 추론 가능한 상태로 반환한다."""
        # classmethod이므로 기존 객체 없이 BCPolicy.load(path)로 호출할 수 있다.
        # map_location을 지정하면 GPU에서 저장한 가중치도 CPU 등으로 읽을 수 있다.
        # weights_only=True는 텐서와 기본 자료형 중심으로 역직렬화를 제한한다.
        checkpoint = torch.load(
            path,
            map_location=device,
            weights_only=True,
        )

        # 저장 당시의 입력/출력 차원으로 동일한 신경망 구조를 먼저 만든다. head가 없는 파일은 수업 원본(mse)이다.
        # 모르는 head나 decode는 생성자가 거절한다.
        model = cls(
            checkpoint["state_dim"],
            checkpoint["action_dim"],
            head=checkpoint.get("head", "mse"),
            accel_bins=checkpoint.get("accel_bins", 11),
            decode=checkpoint.get("decode", "mean"),
        ).to(device)

        # 초기 무작위 가중치를 학습된 값으로 교체하고 정규화 통계도 복원한다.
        model.load_state_dict(checkpoint["state_dict"])
        # 불러온 직후 바로 추론할 수 있도록 평가 모드로 설정한다.
        model.eval()

        return model
