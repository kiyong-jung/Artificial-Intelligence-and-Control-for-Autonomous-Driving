"""학습한 BC 모델이 관측 43개 중 무엇에 기대는지 잰다(순열 중요도).

관측 한 항목(또는 한 묶음)의 값을 행끼리 섞어 넣고 모델 출력이 얼마나 달라지는지 본다. 섞어도 출력이 그대로면
모델이 그 항목을 쓰지 않는다. 묶음으로도 재는 까닭은 서로 겹치는 항목(예: 신호 거리와 적색 여부)이 하나씩
섞을 때는 작게 나오기 때문이다.

    python research/av_bc_rl/feature_importance.py --model models/final/model.pt \
        --data results/g_dagger_s1/dagger_s11_it1_busy.npz --out importance.json

재는 값
  loss_increase  섞었을 때 손실이 늘어난 양. 기준은 NPZ에 적힌 행동이다(모델이 직접 몬 기록이면 모델 자신의 행동)
  accel_change   가감속 출력이 평균 얼마나 달라졌는지(-1~1 눈금)
  lane_flip      차선변경 결정이 바뀐 행의 비율
"""
import argparse
import json
import os
import sys

import numpy as np
import torch

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, REPO)
from algorithms.bc import BCPolicy  # noqa: E402
from env.data_schema import OBSERVATION_FEATURES as NAMES  # noqa: E402

LANES = ("left2", "left1", "current", "right1", "right2")
GROUPS = {
    "두 칸 옆 차로(left2, right2) 전부": [n for n in NAMES if n.startswith(("left2_", "right2_"))],
    "두 칸 옆 차로의 앞뒤 차": [n for n in NAMES if n.startswith(("left2_", "right2_")) and ("gap" in n or "speed" in n)],
    "옆 차로(left1, right1)의 앞뒤 차": [n for n in NAMES if n.startswith(("left1_", "right1_")) and ("gap" in n or "speed" in n)],
    "내 차로 앞차": ["current_leader_gap", "current_leader_relative_speed"],
    "내 차로 뒤차": ["current_follower_gap", "current_follower_relative_speed"],
    "차로 연결성 5개": [f"{lane}_route_connectivity" for lane in LANES],
    "차로 밀도 5개": [f"{lane}_density" for lane in LANES],
    "신호 4개": ["tl_distance", "tl_red", "tl_yellow", "tl_yield"],
    "정류장 2개": ["stop_distance", "stop_dwell_remaining"],
    "횡단보도 2개": ["crosswalk_distance", "crosswalk_occupied"],
    "제한속도 3개": ["speed_limit", "next_speed_limit", "next_speed_limit_distance"],
}


def measure(model, obs, target, columns, repeats, rng, base):
    loss, accel, lane = [], [], []
    for _ in range(repeats):
        shuffled = obs.clone()
        order = torch.as_tensor(rng.permutation(len(obs)))
        shuffled[:, columns] = obs[order][:, columns]
        with torch.no_grad():
            action = model.act(shuffled)
            loss.append(float(model.loss(shuffled, target)) - base["loss"])
        accel.append(float((action[:, 0] - base["action"][:, 0]).abs().mean()))
        lane.append(float((action[:, 1] != base["action"][:, 1]).float().mean()))
    return {"loss_increase": float(np.mean(loss)), "accel_change": float(np.mean(accel)), "lane_flip": float(np.mean(lane))}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True)
    parser.add_argument("--data", nargs="+", required=True, help="모델이 학습에 쓰지 않은 NPZ")
    parser.add_argument("--rows", type=int, default=200000)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    torch.set_num_threads(2)
    rng = np.random.default_rng(0)
    obs, act = [], []
    for path in args.data:
        with np.load(path) as data:
            obs.append(data["observation"])
            act.append(np.stack([data["action_raw"][:, 0], data["lane_change"]], axis=1))
    obs, act = np.concatenate(obs), np.concatenate(act)
    pick = rng.choice(len(obs), min(args.rows, len(obs)), replace=False)
    obs = torch.as_tensor(obs[pick], dtype=torch.float32)
    target = torch.as_tensor(act[pick], dtype=torch.float32)
    model = BCPolicy.load(args.model)
    model.eval()
    with torch.no_grad():
        base = {"loss": float(model.loss(obs, target)), "action": model.act(obs)}
    index = {name: i for i, name in enumerate(NAMES)}
    single = {name: measure(model, obs, target, [index[name]], args.repeats, rng, base) for name in NAMES}
    grouped = {label: measure(model, obs, target, [index[n] for n in names], args.repeats, rng, base)
               for label, names in GROUPS.items()}
    print(f"행 {len(obs)}개, 기준 손실 {base['loss']:.4f}")
    for title, table in (("항목 하나씩", single), ("묶음", grouped)):
        print(f"\n{title}: 손실 증가 / 가감속 변화 / 차선 결정 바뀜")
        for name, m in sorted(table.items(), key=lambda item: -item[1]["loss_increase"]):
            print(f"  {name:<34} {m['loss_increase']:8.4f} {m['accel_change']:7.4f} {m['lane_flip'] * 100:6.2f}%")
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump({"model": args.model, "data": args.data, "rows": len(obs), "base_loss": base["loss"],
                       "single": single, "grouped": grouped}, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
