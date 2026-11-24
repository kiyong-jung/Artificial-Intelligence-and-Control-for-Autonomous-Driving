"""Ego-attention 정책을 hapjeong SUMO 온라인 환경에서 PPO로 학습한다.

training/hapjeong/train.py의 train_stage와 presets.json을 그대로 쓰고, 네트워크 hparams만 ego_attention으로 바꾼다.

    # BC 사전학습에서 이어서(initial -> refine 두 단계, presets bc_ppo_initial / bc_ppo_refine)
    python scripts/train_ego_attention_ppo.py bc-ppo --base results/ego_att_bc_masked/model.pt --out results/ego_att_bcppo
    # 무작위 초기화부터(preset ppo)
    python scripts/train_ego_attention_ppo.py ppo --out results/ego_att_ppo --heads 2
    # 짧은 확인: --steps 64

--base 모델은 scripts/train_ego_attention_bc.py를 --input observation --mask-obs-indices 1,2,3,4,5로 학습한 것이어야 한다.
hapjeong FullTask는 관측 1~5열(주행 가능 차로)을 0으로 지우고, train_stage가 base run_config에서 그 표시를 확인한다.
"""
import argparse
import copy
import json
import sys
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from training.hapjeong.task import install  # noqa: E402
from training.hapjeong.train import PRESETS, train_stage  # noqa: E402
from utils.ego_attention import DEFAULT_ATTENTION  # noqa: E402

NET_KEYS = ("policy_type", "attention", "init_log_std", "lane_keep_bias")


def preset_with(name, net):
    preset = copy.deepcopy(PRESETS[name])
    preset["hparams"].update(copy.deepcopy(net))
    return preset


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("method", choices=["bc-ppo", "ppo"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--base", help="scripts/train_ego_attention_bc.py가 만든 model.pt (bc-ppo 필수)")
    ap.add_argument("--steps", type=int, help="단계별 PPO 스텝 수를 덮어쓴다(64면 짧은 확인)")
    ap.add_argument("--feature-size", type=int, default=DEFAULT_ATTENTION["feature_size"], help="ppo 전용")
    ap.add_argument("--heads", type=int, default=DEFAULT_ATTENTION["heads"], help="ppo 전용")
    ap.add_argument("--self-attention", action="store_true", help="ppo 전용")
    a = ap.parse_args(argv)
    if a.method == "bc-ppo":
        if not a.base:
            ap.error("bc-ppo에는 --base가 필요합니다")
        config = json.loads(Path(a.base).with_name("run_config.json").read_text(encoding="utf-8"))
        hp = config["hparams"]
        if hp.get("policy_type") != "ego_attention" or config.get("obs_dim", 93) != 93:
            ap.error("--base는 93차원 observation 입력의 ego_attention 모델이어야 합니다")
        net = {k: hp[k] for k in NET_KEYS}
    else:
        if a.base:
            ap.error("ppo는 무작위 초기화입니다. 이어 학습은 bc-ppo")
        net = {"policy_type": "ego_attention",
               "attention": dict(DEFAULT_ATTENTION, feature_size=a.feature_size, heads=a.heads,
                                 self_attention=a.self_attention)}
    torch.set_num_threads(1)
    install()
    if a.method == "bc-ppo":
        out = Path(a.out).resolve()
        out.mkdir(parents=True, exist_ok=False)
        first = train_stage("bc-ppo", preset_with("bc_ppo_initial", net), out / "initial", a.base, a.steps)
        train_stage("bc-ppo", preset_with("bc_ppo_refine", net), out / "refine", first, a.steps)
    else:
        train_stage("ppo", preset_with("ppo", net), a.out, None, a.steps)


if __name__ == "__main__":
    main()
