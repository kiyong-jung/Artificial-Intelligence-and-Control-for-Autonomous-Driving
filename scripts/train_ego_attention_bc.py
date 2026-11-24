"""PDM 버스 전문가 데이터로 Ego-attention 정책을 오프라인 학습한다(행동 복제 + critic TD 사전학습).

    python scripts/train_ego_attention_bc.py --data data/pdm_bus/lap_baseline_s0 data/pdm_bus/baseline_s1 \
        --out results/ego_att_bc
    python scripts/train_ego_attention_bc.py --data data/pdm_bus/*/processed/transitions.npz --input both \
        --self-attention --heads 2 --epochs 300 --out results/ego_att_both
    # hapjeong 온라인 환경(FullTask가 obs[1:6]을 0으로 지움)에서 이어 학습할 모델:
    python scripts/train_ego_attention_bc.py --data ... --mask-obs-indices 1,2,3,4,5 --out results/ego_att_bc_masked

입력 데이터: scripts/preprocess_pdm_bus.py가 만든 <run>/processed/transitions.npz (폴더를 주면 알아서 찾는다).
data_colum.md 열과 쓰임:
    observation / next_observation (93) : 정책 입력(--input observation, 기본). 온라인 BusTask와 같은 관측
    features / next_features (130)      : SUMO_AV_input_설계.md 입력 설계 특징(--input features), 둘 다는 --input both
    action_raw [가감속, 차선변경]        : 행동 복제 라벨. 가감속은 MSE(제동에 --brake-weight), 차선은 클래스 가중 CE
    reward, terminated, truncated       : critic TD(0) 타깃 r + γ(1-terminated)V'(s'). truncated는 끝이 아니므로 부트스트랩
    episode, t                          : 에피소드 단위 train/validation/test 분할(에피소드가 3개 미만이면 시간 블록 분할)

critic은 몸통 특징을 detach한 뒤 v_head만 학습한다. 행동 복제가 critic 손실에 흔들리지 않게 하고,
온라인 PPO를 시작할 때 쓸 만한 V(s)를 미리 맞춰 둔다.

출력 <out>/: model.pt(algorithms.ppo.PPO(obs_dim, 2, **hparams).load로 읽힌다), run_config.json, history.jsonl,
audit.json(분할, 클래스 가중치, 최종 지표, 엔티티 타입별 평균 attention), 이 스크립트 사본.
온라인 이어 학습: scripts/train_ego_attention_ppo.py --base <out>/model.pt
"""
import argparse
import copy
import hashlib
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from algorithms.ppo import PPO  # noqa: E402
from utils.ego_attention import DEFAULT_ATTENTION, INPUT_LAYOUT, LAYOUTS  # noqa: E402

OBS_DIM = 93
LANE_WEIGHT_CAP = 20.0
BRAKE_THRESHOLD = -0.05
PPO_DEFAULTS = dict(lr=1e-4, n_steps=1024, n_epochs=10, minibatch_size=64, gamma=0.999, gae_lambda=0.95,
                    clip_eps=0.2, value_coef=0.005, entropy_coef=0.0, lane_entropy_coef=0.0, max_grad_norm=0.5,
                    normalize_advantage=True)


def resolve(path) -> Path:
    p = Path(path)
    if p.is_dir():
        p = p / "processed" / "transitions.npz" if (p / "processed").is_dir() else p / "transitions.npz"
    if not p.is_file():
        raise FileNotFoundError(f"transitions.npz가 없습니다: {path}")
    return p


def _inputs(d, kind, prefix, mask_idx):
    obs = d[f"{prefix}observation"].astype(np.float32).copy()
    if mask_idx:
        obs[:, mask_idx] = 0.0
    if kind == "observation":
        return obs
    feats = d[f"{prefix}features"].astype(np.float32)
    return feats if kind == "features" else np.concatenate([obs, feats], axis=1)


def load(paths, kind, mask_idx):
    """여러 수집 run을 이어 붙인다. 에피소드 키 = 파일 번호 * 1e6 + episode."""
    cols = {k: [] for k in ("x", "nx", "y", "r", "term", "trunc", "key", "t")}
    sources = []
    for i, p in enumerate(paths):
        with np.load(p, allow_pickle=False) as d:
            need = ["observation", "next_observation", "action_raw", "reward", "terminated", "truncated", "episode", "t"]
            if kind != "observation":
                need += ["features", "next_features"]
            missing = [k for k in need if k not in d]
            if missing:
                raise ValueError(f"{p}: 필요한 열이 없습니다 {missing}")
            if d["observation"].shape[1] != OBS_DIM or d["action_raw"].shape[1] != 2:
                raise ValueError(f"{p}: 93차원 observation과 2차원 action_raw가 필요합니다")
            cols["x"].append(_inputs(d, kind, "", mask_idx))
            cols["nx"].append(_inputs(d, kind, "next_", mask_idx))
            cols["y"].append(d["action_raw"].astype(np.float32))
            cols["r"].append(d["reward"].astype(np.float32))
            cols["term"].append(d["terminated"].astype(bool))
            cols["trunc"].append(d["truncated"].astype(bool))
            cols["key"].append(i * 1_000_000 + d["episode"].astype(np.int64))
            cols["t"].append(d["t"].astype(np.int64))
            sources.append({"path": str(p), "rows": int(len(d["reward"])),
                            "sha256": hashlib.sha256(p.read_bytes()).hexdigest()})
    data = {k: np.concatenate(v) for k, v in cols.items()}
    x, y = data["x"], data["y"]
    if not (np.isfinite(x).all() and np.isfinite(data["nx"]).all() and np.isfinite(data["r"]).all()):
        raise ValueError("입력이나 보상에 NaN/inf가 있습니다")
    if np.abs(x).max() > 1.0 + 1e-5:
        raise ValueError("입력은 [-1, 1]로 정규화되어 있어야 합니다")
    if not np.isin(y[:, 1], [-1, 0, 1]).all() or (np.abs(y[:, 0]) > 1 + 1e-6).any():
        raise ValueError("action_raw는 [정규화 가감속 ∈ [-1,1], 차선 ∈ {-1,0,1}]이어야 합니다")
    return data, sources


def make_split(keys, t, rng, split_file=None):
    """에피소드 단위 70/15/15 분할. 에피소드가 3개 미만이면 에피소드마다 시간순 70/15/15 블록으로 자른다."""
    ids = np.unique(keys)
    if split_file:
        split = json.loads(Path(split_file).read_text(encoding="utf-8"))["split"]
        if sorted(sum(split.values(), [])) != sorted(ids.tolist()):
            raise ValueError("split 파일의 에피소드 키가 데이터와 다릅니다")
        return {k: np.flatnonzero(np.isin(keys, v)) for k, v in split.items()}, {"mode": "file", "split": split}
    if len(ids) >= 3:
        perm = rng.permutation(ids)
        n_hold = max(1, int(round(len(ids) * 0.15)))
        split = {"train": perm[2 * n_hold:], "validation": perm[:n_hold], "test": perm[n_hold:2 * n_hold]}
        idx = {k: np.flatnonzero(np.isin(keys, v)) for k, v in split.items()}
        return idx, {"mode": "episode", "split": {k: sorted(int(e) for e in v) for k, v in split.items()}}
    idx = {"train": [], "validation": [], "test": []}
    for e in ids:
        rows = np.flatnonzero(keys == e)
        rows = rows[np.argsort(t[rows], kind="stable")]
        a, b = int(len(rows) * 0.70), int(len(rows) * 0.85)
        idx["train"].append(rows[:a]), idx["validation"].append(rows[a:b]), idx["test"].append(rows[b:])
    idx = {k: np.concatenate(v) for k, v in idx.items()}
    return idx, {"mode": "time_block", "episodes": [int(e) for e in ids],
                 "note": "에피소드가 3개 미만이라 각 에피소드를 시간순 70/15/15로 잘랐다(경계 근처 상관 있음)"}


class Trainer:
    def __init__(self, agent, data, args, device):
        self.net, self.args, self.device = agent.policy, args, device
        self.x = torch.as_tensor(data["x"], device=device)
        self.nx = torch.as_tensor(data["nx"], device=device)
        self.accel = torch.as_tensor(data["y"][:, 0], device=device)
        self.lane = torch.as_tensor(1 - data["y"][:, 1], device=device).long()  # +1→0(좌), 0→1, -1→2(우)
        self.r = torch.as_tensor(data["r"], device=device)
        self.done = torch.as_tensor(data["term"], device=device).float()
        self.target = copy.deepcopy(self.net).eval()
        self.optimizer = torch.optim.Adam([p for n, p in self.net.named_parameters() if n != "log_std"], lr=args.lr)
        self.weights = torch.ones(3, device=device)

    def set_class_weights(self, train_idx):
        counts = torch.bincount(self.lane[train_idx], minlength=3).float()
        w = torch.where(counts > 0, counts.sum() / (3 * counts.clamp_min(1)), torch.zeros_like(counts))
        self.weights = w.clamp(max=LANE_WEIGHT_CAP)
        return counts.tolist(), self.weights.tolist()

    def losses(self, ii):
        x = self.x[ii]
        z = self.net.encode(x)
        mu, _, logits, _ = self.net.heads(z, x)
        w = torch.where(self.accel[ii] < BRAKE_THRESHOLD, self.args.brake_weight, 1.0)
        accel = ((mu[:, 0] - self.accel[ii]).square() * w).sum() / w.sum()
        cls = self.lane[ii]
        # lane_action_mask가 막은 쪽 라벨은 CE에서 뺀다(로짓 -1e9). 개수는 audit에 남긴다.
        ok = logits.gather(1, cls[:, None]).squeeze(1) > -1e8
        ok = ok & (self.weights[cls] > 0)
        lane = F.cross_entropy(logits[ok], cls[ok], weight=self.weights) if ok.any() else logits.sum() * 0.0
        v = self.net.v_head(z.detach()).squeeze(-1)
        with torch.no_grad():
            target = self.r[ii] + self.args.gamma * (1.0 - self.done[ii]) * self.target(self.nx[ii])[3]
        value = F.smooth_l1_loss(v, target)
        return accel, lane, value, (mu, logits, v, target, ok)

    def train_epoch(self, train_idx, rng):
        self.net.train()
        for ii in np.array_split(rng.permutation(train_idx), max(1, int(np.ceil(len(train_idx) / self.args.batch_size)))):
            accel, lane, value, _ = self.losses(torch.as_tensor(ii, device=self.device))
            loss = accel + self.args.lane_weight * lane + self.args.value_coef * value
            self.optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.net.parameters(), self.args.max_grad_norm)
            self.optimizer.step()
        self.target.load_state_dict(self.net.state_dict())

    @torch.no_grad()
    def evaluate(self, idx):
        self.net.eval()
        if len(idx) == 0:
            return None
        sums, n, preds, labels, masked = {"accel": 0.0, "lane": 0.0, "value": 0.0, "accel_mae": 0.0}, 0, [], [], 0
        for ii in np.array_split(idx, max(1, int(np.ceil(len(idx) / 4096)))):
            ii_t = torch.as_tensor(ii, device=self.device)
            accel, lane, value, (mu, logits, _, _, ok) = self.losses(ii_t)
            k = len(ii)
            sums["accel"] += float(accel) * k
            sums["lane"] += float(lane) * k
            sums["value"] += float(value) * k
            sums["accel_mae"] += float((mu[:, 0] - self.accel[ii_t]).abs().sum())
            preds.append(logits.argmax(-1).cpu().numpy())
            labels.append(self.lane[ii_t].cpu().numpy())
            masked += int((~ok).sum())
            n += k
        p, y = np.concatenate(preds), np.concatenate(labels)
        change = y != 1
        out = {"accel_mse": sums["accel"] / n, "lane_ce": sums["lane"] / n, "value_td": sums["value"] / n,
               "accel_mae": sums["accel_mae"] / n, "lane_acc": float((p == y).mean()),
               "lane_change_recall": float((p[change] == y[change]).mean()) if change.any() else None,
               "lane_change_precision": float((p[p != 1] == y[p != 1]).mean()) if (p != 1).any() else None,
               "lane_labels_excluded": masked, "rows": n}
        out["bc_loss"] = out["accel_mse"] + self.args.lane_weight * out["lane_ce"]
        return out

    @torch.no_grad()
    def attention_summary(self, idx):
        """엔티티 타입별 평균 attention 합(ego, vehicle, pedestrian, ...)."""
        if len(idx) == 0:
            return None
        self.net.eval()
        att = torch.cat([self.net.attention_matrix(self.x[torch.as_tensor(ii, device=self.device)])
                         for ii in np.array_split(idx, max(1, int(np.ceil(len(idx) / 4096))))]).mean(0).cpu().numpy()
        out = {}
        for name, a in zip(self.net.entities, att):
            group = name.rstrip("0123456789")
            out[group] = out.get(group, 0.0) + float(a)
        return out


def parse_layers(text):
    return [int(v) for v in text.split(",") if v.strip()] if text else []


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", nargs="+", required=True, help="수집 run 폴더나 transitions.npz (여러 개 가능)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--input", choices=sorted(INPUT_LAYOUT), default="observation")
    ap.add_argument("--mask-obs-indices", default="", help="observation에서 0으로 지울 열(예: 1,2,3,4,5)")
    ap.add_argument("--split-file", help='{"split": {"train": [...], "validation": [...], "test": [...]}} 에피소드 키')
    ap.add_argument("--config", help="온라인 run_config.json. 주면 그 설정(ego, observation, PPO hparams)을 이어받는다")
    # 네트워크
    ap.add_argument("--feature-size", type=int, default=DEFAULT_ATTENTION["feature_size"])
    ap.add_argument("--heads", type=int, default=DEFAULT_ATTENTION["heads"])
    ap.add_argument("--embed-layers", default=",".join(map(str, DEFAULT_ATTENTION["embed_layers"])))
    ap.add_argument("--output-layers", default=",".join(map(str, DEFAULT_ATTENTION["output_layers"])))
    ap.add_argument("--self-attention", action="store_true")
    ap.add_argument("--dropout", type=float, default=0.0)
    ap.add_argument("--lane-action-mask", action="store_true", help="차선 변경 불가 쪽 로짓을 막는다")
    ap.add_argument("--init-log-std", type=float, default=-2.0)
    ap.add_argument("--lane-keep-bias", type=float, default=0.5)
    # 학습
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch-size", type=int, default=256)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--brake-weight", type=float, default=4.0)
    ap.add_argument("--lane-weight", type=float, default=1.0)
    ap.add_argument("--value-coef", type=float, default=0.5, help="0이면 critic을 학습하지 않는다")
    ap.add_argument("--gamma", type=float, default=0.999)
    ap.add_argument("--max-grad-norm", type=float, default=0.5)
    ap.add_argument("--seed", type=int, default=38001)
    ap.add_argument("--device", default="cpu")
    a = ap.parse_args(argv)
    if not (np.isfinite(a.brake_weight) and a.brake_weight >= 1):
        ap.error("--brake-weight는 1 이상이어야 합니다")
    mask_idx = [int(v) for v in a.mask_obs_indices.split(",") if v.strip()]
    if mask_idx and a.input == "features":
        ap.error("--mask-obs-indices는 observation 입력(observation, both)에만 쓴다")
    if any(not 0 <= i < OBS_DIM for i in mask_idx):
        ap.error("--mask-obs-indices는 0~92")

    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    paths = [resolve(p) for p in a.data]
    data, sources = load(paths, a.input, mask_idx)
    idx, split_info = make_split(data["key"], data["t"], rng, a.split_file)
    if len(idx["train"]) == 0 or len(idx["validation"]) == 0:
        raise ValueError("train/validation 분할이 비었습니다")

    layout = INPUT_LAYOUT[a.input]
    obs_dim = LAYOUTS[layout]["dim"]
    attention = dict(DEFAULT_ATTENTION, layout=layout, feature_size=a.feature_size, heads=a.heads,
                     embed_layers=parse_layers(a.embed_layers), output_layers=parse_layers(a.output_layers),
                     self_attention=a.self_attention, dropout=a.dropout, lane_action_mask=a.lane_action_mask)
    base_config = json.loads(Path(a.config).read_text(encoding="utf-8")) if a.config else {}
    hp = {**PPO_DEFAULTS, **{k: v for k, v in (base_config.get("hparams") or {}).items()
                             if k in PPO_DEFAULTS}}
    hp.update(policy_type="ego_attention", attention=attention, init_log_std=a.init_log_std,
              lane_keep_bias=a.lane_keep_bias, seed=a.seed, device=a.device)
    agent = PPO(obs_dim, 2, **hp)
    trainer = Trainer(agent, data, a, agent.device)
    counts, weights = trainer.set_class_weights(torch.as_tensor(idx["train"], device=agent.device))
    n_params = sum(p.numel() for p in agent.policy.parameters() if p.requires_grad)
    print(f"rows={len(data['key'])} train/val/test={[len(idx[k]) for k in ('train', 'validation', 'test')]} "
          f"split={split_info['mode']} input={a.input}({obs_dim}) params={n_params} lane_counts={counts}", flush=True)

    best, best_epoch, history = float("inf"), 0, []
    with (out / "history.jsonl").open("w", encoding="utf-8") as hist:
        for epoch in range(1, a.epochs + 1):
            trainer.train_epoch(idx["train"], rng)
            tr, va = trainer.evaluate(idx["train"]), trainer.evaluate(idx["validation"])
            if not (np.isfinite(tr["bc_loss"]) and np.isfinite(va["bc_loss"])):
                raise FloatingPointError(f"epoch {epoch}: 손실이 발산했습니다")
            row = {"epoch": epoch, "train": tr, "validation": va}
            hist.write(json.dumps(row, ensure_ascii=False) + "\n")
            history.append(row)
            if va["bc_loss"] < best:
                best, best_epoch = va["bc_loss"], epoch
                agent.save(str(out / "model.pt"))
            if epoch == 1 or epoch % 10 == 0 or epoch == a.epochs:
                print(f"epoch {epoch:4d} train bc={tr['bc_loss']:.4f} val bc={va['bc_loss']:.4f} "
                      f"acc_mae={va['accel_mae']:.3f} lane_acc={va['lane_acc']:.3f} "
                      f"lc_recall={va['lane_change_recall']} V_td={va['value_td']:.4f}", flush=True)

    agent.load(str(out / "model.pt"))
    trainer.net = agent.policy
    final = {k: trainer.evaluate(v) for k, v in idx.items()}
    restored = PPO(obs_dim, 2, **hp)
    restored.load(str(out / "model.pt"))
    probe = torch.as_tensor(data["x"][:32], device=agent.device)
    with torch.no_grad():
        assert torch.equal(agent.policy.det_action(probe), restored.policy.det_action(probe))

    settings = {k: base_config[k] for k in ("road", "ego", "traffic", "simulation", "observation", "action")
                if k in base_config}
    if mask_idx:  # 학습 입력에서 지운 열을 기록한다(hapjeong check_model_config가 읽는 키)
        settings["ego"] = {**settings.get("ego", {}), "observation_mask_indices": mask_idx}
        settings["observation"] = {**settings.get("observation", {}), "masked_indices": mask_idx}
    processed_meta = resolve(a.data[0]).with_name("metadata.json")
    config = {**settings, "kind": "ego_attention_bc_pretraining", "hparams": hp, "input": a.input, "obs_dim": obs_dim,
              "data": sources, "reward": base_config.get("reward"),
              "data_metadata": json.loads(processed_meta.read_text(encoding="utf-8")) if processed_meta.is_file() else None,
              "created_at": datetime.now().astimezone().isoformat()}
    (out / "run_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    audit = {"split": split_info, "selected_epoch": best_epoch, "final": final,
             "attention_by_type_validation": trainer.attention_summary(idx["validation"]),
             "lane_counts_train": counts, "lane_weights": weights, "trainable_parameters": n_params,
             "loss": {"accel": f"MSE, 제동(<{BRAKE_THRESHOLD}) 가중 {a.brake_weight}",
                      "lane": f"클래스 가중 CE x {a.lane_weight}(가중 상한 {LANE_WEIGHT_CAP})",
                      "value": f"smooth L1 TD(0) x {a.value_coef}, gamma {a.gamma}, 몸통 detach, 에폭마다 target 갱신"},
             "selection": "validation bc_loss 최소; test는 선택에 쓰지 않음", "save_load_parity": True,
             "args": vars(a)}
    (out / "audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    shutil.copy2(__file__, out / Path(__file__).name)
    print(f"done: best epoch {best_epoch}, test={final['test']}", flush=True)
    print(f"attention by type (validation): {audit['attention_by_type_validation']}", flush=True)


if __name__ == "__main__":
    main()
