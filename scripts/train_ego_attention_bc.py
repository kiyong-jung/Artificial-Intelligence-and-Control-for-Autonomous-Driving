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
    observation_v2 / next_observation_v2 (166) : 관측 v2(docs/observation_features_v2.md, --input v2)
    action_raw [가감속, 차선변경]        : 행동 복제 라벨. 가감속은 [-1, 1]을 --accel-bins개 구간으로 나눈 CE
                                          (0이면 MSE, 제동 표본에 --brake-weight), 차선은 클래스 가중 CE(--lane-weighting).
                                          prev_action_raw 열이 있는 데이터는 5값 차로 라벨(5클래스), 없으면 예전 3값(3클래스)
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
import os
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from algorithms.ppo import PPO  # noqa: E402
from utils.ego_attention import DEFAULT_ATTENTION, INPUT_LAYOUT, LAYOUTS, accel_bin_of  # noqa: E402

OBS_DIM = 93
WEIGHTINGS = ("none", "inverse", "sqrt", "effective")


def class_weights(counts, scheme, cap, beta=0.999):
    """클래스 빈도 -> CE 가중치. 가장 많은 클래스가 1이고, 드문 클래스는 cap배까지 올라간다. train에 없는 클래스는 0.

    inverse: 1/n, sqrt: 1/sqrt(n), effective: (1-beta)/(1-beta^n) (Cui et al. 2019, 유효 표본 수), none: 모두 1.
    CE가 가중치 합으로 나눠 평균을 내므로 절대 크기는 상관없고 클래스 사이 비율만 뜻이 있다.
    """
    n = counts.clamp_min(1.0)
    raw = {"none": torch.ones_like(n), "inverse": 1.0 / n, "sqrt": n.rsqrt(),
           "effective": (1.0 - beta) / (1.0 - torch.pow(torch.full_like(n, beta), n))}[scheme]
    w = (raw / raw[counts.argmax()]).clamp(max=cap)
    return torch.where(counts > 0, w, torch.zeros_like(w))
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
    if kind == "v2":
        return d[f"{prefix}observation_v2"].astype(np.float32)
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
    sources, kinds = [], set()
    for i, p in enumerate(paths):
        with np.load(p, allow_pickle=False) as d:
            need = ["action_raw", "reward", "terminated", "truncated", "episode", "t"]
            need += {"v2": ["observation_v2", "next_observation_v2"],
                     "features": ["features", "next_features"],
                     "both": ["observation", "next_observation", "features", "next_features"]}.get(
                         kind, ["observation", "next_observation"])
            missing = [k for k in need if k not in d]
            if missing:
                raise ValueError(f"{p}: 필요한 열이 없습니다 {missing}")
            if kind in ("observation", "both") and d["observation"].shape[1] != OBS_DIM:
                raise ValueError(f"{p}: 93차원 observation이 필요합니다")
            if d["action_raw"].shape[1] != 2:
                raise ValueError(f"{p}: 2차원 action_raw가 필요합니다")
            # prev_action_raw가 있으면 5값 차로 라벨(pdm_data/labels.lane_change_label5)로 만든 데이터다
            kinds.add(5 if "prev_action_raw" in d else 3)
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
    if len(kinds) != 1:
        raise ValueError("3값 차로 라벨 데이터와 5값 차로 라벨 데이터를 섞을 수 없습니다(전처리를 다시 하세요)")
    n_lane = kinds.pop()
    data = {k: np.concatenate(v) for k, v in cols.items()}
    x, y = data["x"], data["y"]
    if not (np.isfinite(x).all() and np.isfinite(data["nx"]).all() and np.isfinite(data["r"]).all()):
        raise ValueError("입력이나 보상에 NaN/inf가 있습니다")
    if np.abs(x).max() > 1.0 + 1e-5:
        raise ValueError("입력은 [-1, 1]로 정규화되어 있어야 합니다")
    half = n_lane // 2
    if not np.isin(y[:, 1], np.arange(-half, half + 1)).all() or (np.abs(y[:, 0]) > 1 + 1e-6).any():
        raise ValueError(f"action_raw는 [정규화 가감속 ∈ [-1,1], 차선 ∈ {{-{half}..{half}}}]이어야 합니다")
    data["n_lane"] = n_lane
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
        self.n_lane = int(data["n_lane"])
        self.keep = self.n_lane // 2
        # 라벨 -> 클래스: keep - 라벨. 3값 +1→0(좌), 0→1, -1→2(우). 5값 +2→0, +1→1, 0→2, -1→3, -2→4
        self.lane = torch.as_tensor(self.keep - data["y"][:, 1], device=device).long()
        self.r = torch.as_tensor(data["r"], device=device)
        self.done = torch.as_tensor(data["term"], device=device).float()
        self.target = copy.deepcopy(self.net).eval()
        params = [p for n, p in self.net.named_parameters() if n != "log_std"]
        wd = getattr(args, "weight_decay", 0.0)
        self.optimizer = (torch.optim.AdamW(params, lr=args.lr, weight_decay=wd) if wd > 0
                          else torch.optim.Adam(params, lr=args.lr))
        self.weights = torch.ones(self.n_lane, device=device)
        self.n_accel = self.net.n_accel
        if self.n_accel:
            self.accel_cls = accel_bin_of(self.accel, self.n_accel)
            self.accel_weights = torch.ones(self.n_accel, device=device)

    def set_class_weights(self, train_idx):
        """차로(와 가감속 구간 분류면 가감속) 클래스 가중치를 train 분할의 빈도로 정한다."""
        a = self.args
        counts = torch.bincount(self.lane[train_idx], minlength=self.n_lane).float()
        self.weights = class_weights(counts, a.lane_weighting, a.lane_weight_cap, a.cb_beta)
        out = {"lane_counts": counts.tolist(), "lane_weights": self.weights.tolist()}
        if self.n_accel:
            ac = torch.bincount(self.accel_cls[train_idx], minlength=self.n_accel).float()
            self.accel_weights = class_weights(ac, a.accel_weighting, a.accel_weight_cap, a.cb_beta)
            out.update(accel_counts=ac.tolist(), accel_weights=self.accel_weights.tolist())
        return out

    def _accel_loss(self, z, ii, w):
        """MSE(연속) 또는 구간 CE. w = 제동 표본 가중치. 반환: (손실, 결정적 예측 가감속)."""
        if not self.n_accel:
            mu = torch.tanh(self.net.mu_head(z))[:, 0]
            return ((mu - self.accel[ii]).square() * w).sum() / w.sum(), mu
        logits = self.net.accel_logits(z)
        cls = self.accel_cls[ii]
        cw = self.accel_weights[cls] * w
        loss = (F.cross_entropy(logits, cls, reduction="none") * cw).sum() / cw.sum().clamp_min(1e-8)
        return loss, self.net.accel_values[logits.argmax(-1)]

    def losses(self, ii):
        x = self.x[ii]
        z = self.net.encode(x)
        logits = self.net.lane_logits(z, x)
        w = torch.where(self.accel[ii] < BRAKE_THRESHOLD, self.args.brake_weight, 1.0)
        accel, mu = self._accel_loss(z, ii, w)
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
            sums["accel_mae"] += float((mu - self.accel[ii_t]).abs().sum())
            sums["accel_mse_pred"] = sums.get("accel_mse_pred", 0.0) + float((mu - self.accel[ii_t]).square().sum())
            preds.append(logits.argmax(-1).cpu().numpy())
            labels.append(self.lane[ii_t].cpu().numpy())
            if self.n_accel:
                a_hit = (accel_bin_of(mu, self.n_accel) == self.accel_cls[ii_t])
                sums["accel_acc"] = sums.get("accel_acc", 0.0) + float(a_hit.sum())
            masked += int((~ok).sum())
            n += k
        p, y = np.concatenate(preds), np.concatenate(labels)
        k = self.keep
        change = y != k
        per_class = [float((p[y == c] == c).mean()) if (y == c).any() else None for c in range(self.n_lane)]
        seen = [r for r in per_class if r is not None]
        out = {"accel_loss": sums["accel"] / n, "accel_mse": sums["accel_mse_pred"] / n,
               "lane_ce": sums["lane"] / n, "value_td": sums["value"] / n,
               "accel_mae": sums["accel_mae"] / n, "lane_acc": float((p == y).mean()),
               "lane_recall_per_class": per_class, "lane_balanced_acc": float(np.mean(seen)) if seen else None,
               "lane_change_recall": float((p[change] == y[change]).mean()) if change.any() else None,
               "lane_change_precision": float((p[p != k] == y[p != k]).mean()) if (p != k).any() else None,
               "lane_labels_excluded": masked, "rows": n}
        if self.n_accel:
            out["accel_bin_acc"] = sums["accel_acc"] / n
        if self.n_lane == 5:   # 실제로 옮기는 클래스(±2)만 따로
            move, pm = np.isin(y, (0, 4)), np.isin(p, (0, 4))
            out["lane_move_recall"] = float((p[move] == y[move]).mean()) if move.any() else None
            out["lane_move_precision"] = float((p[pm] == y[pm]).mean()) if pm.any() else None
            # 위험한 오예측: 유지해야 할 때 깜빡이 없이 옮기기, 깜빡이 중(전문가는 기다림) 먼저 옮기기, 반대 방향
            out["keep_to_move"] = int(((y == k) & pm).sum())
            out["blink_to_move"] = int((np.isin(y, (1, 3)) & pm).sum())
            out["wrong_side"] = int((((y < k) & (p > k)) | ((y > k) & (p < k))).sum())
        out["bc_loss"] = out["accel_loss"] + self.args.lane_weight * out["lane_ce"]
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


RESUME_FREE_ARGS = {"resume", "epochs", "device", "data", "data_list", "out", "patience", "tensorboard"}
TB_KEYS = ("bc_loss", "accel_loss", "lane_ce", "value_td", "accel_mae", "accel_bin_acc", "lane_acc", "lane_balanced_acc",
           "lane_change_recall", "lane_change_precision", "lane_move_recall", "lane_move_precision", "keep_to_move",
           "blink_to_move", "wrong_side")
LANE_CLASS_NAMES = {3: ("left", "keep", "right"),
                    5: ("left_move", "left_blink", "keep", "right_blink", "right_move")}


def open_tensorboard(out, start_epoch):
    """<out>/tb에 기록한다. 이어서 할 때는 체크포인트 뒤에 쓰인 기록(start_epoch 이후)을 지운다."""
    try:
        from torch.utils.tensorboard import SummaryWriter
    except ImportError:
        print("tensorboard가 없어 기록하지 않습니다(pip install tensorboard)", flush=True)
        return None
    return SummaryWriter(str(out / "tb"), purge_step=start_epoch + 1 if start_epoch else None)


def log_tensorboard(w, epoch, row, n_lane):
    """지표/분할 이름으로 기록해서 train과 validation이 같은 그래프에 겹쳐 보이게 한다."""
    for split in ("train", "validation"):
        m = row[split]
        for k in TB_KEYS:
            if m.get(k) is not None:
                w.add_scalar(f"{k}/{split}", m[k], epoch)
        for name, r in zip(LANE_CLASS_NAMES[n_lane], m["lane_recall_per_class"]):
            if r is not None:
                w.add_scalar(f"lane_recall_{name}/{split}", r, epoch)
    w.add_scalar("lr", row["lr"], epoch)
    w.add_scalar("time_s", row["time_s"], epoch)
    w.flush()


def read_data_list(path):
    """한 줄에 경로 하나. 빈 줄과 #으로 시작하는 줄은 건너뛴다. 상대 경로는 목록 파일 기준."""
    if not path:
        return []
    base = Path(path).resolve().parent
    out = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            p = Path(line)
            out.append(str(p if p.is_absolute() else base / p))
    return out


def atomic_save(obj, path):
    """임시 파일에 쓰고 바꿔 넣는다. 저장 중에 멈춰도 이전 파일이 깨지지 않는다."""
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    torch.save(obj, tmp)
    os.replace(tmp, path)


def check_resume(out, a, sources):
    """새 실행이면 run_args.json을 쓰고 None. 이어서 하는 실행이면 인자·데이터가 같은지 확인하고 체크포인트를 돌려준다.

    데이터는 경로가 아니라 sha256과 순서로 비교한다(서버로 옮겨 경로가 바뀌어도 된다).
    --epochs, --patience, --device, 경로 인자는 바꿔도 된다(예: 더 오래 학습).
    """
    args_path, ckpt = out / "run_args.json", out / "checkpoint.pt"
    now = {"args": {k: v for k, v in vars(a).items() if k not in RESUME_FREE_ARGS},
           "epochs": a.epochs, "data_sha256": [s["sha256"] for s in sources]}
    if not ckpt.is_file():
        args_path.write_text(json.dumps(now, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return None
    prev = json.loads(args_path.read_text(encoding="utf-8"))
    diff = sorted(k for k in set(prev["args"]) | set(now["args"]) if prev["args"].get(k) != now["args"].get(k))
    if diff:
        raise ValueError(f"이어서 학습할 수 없습니다: 인자가 다릅니다 {diff}")
    if a.lr_schedule == "cosine" and prev.get("epochs") != a.epochs:
        raise ValueError("이어서 학습할 수 없습니다: cosine 스케줄은 --epochs가 같아야 합니다"
                         f"(처음 {prev.get('epochs')}, 지금 {a.epochs})")
    if prev["data_sha256"] != now["data_sha256"]:
        raise ValueError("이어서 학습할 수 없습니다: 데이터 파일(내용 또는 순서)이 다릅니다")
    return torch.load(ckpt, map_location=a.device, weights_only=False)


def make_scheduler(optimizer, a):
    if a.lr_schedule == "cosine":
        return torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=a.epochs, eta_min=a.min_lr)
    if a.lr_schedule == "plateau":
        return torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, factor=0.5, patience=5, min_lr=a.min_lr)
    return None


def parse_layers(text):
    return [int(v) for v in text.split(",") if v.strip()] if text else []


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", nargs="+", help="수집 run 폴더나 transitions.npz (여러 개 가능)")
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
    # 기본값 근거: seed 29개, 40 epoch 비교에서 effective(beta 0.9999)가 시험 분할 클래스 평균 recall이 가장 높고
    # 깜빡이 없이 바로 옮기는 오예측(정답 0 -> 예측 ±2)이 0건이었다(sqrt 9건, inverse 1건).
    ap.add_argument("--lane-weighting", choices=WEIGHTINGS, default="effective", help="차로 클래스 가중 방식")
    ap.add_argument("--lane-weight-cap", type=float, default=100.0, help="가장 많은 클래스 대비 최대 배수")
    ap.add_argument("--accel-bins", type=int, default=21,
                    help="가감속을 [-1, 1]의 구간 K개로 나눠 CE로 학습(0이면 MSE 회귀)")
    ap.add_argument("--accel-weighting", choices=WEIGHTINGS, default="none", help="가감속 구간 클래스 가중 방식")
    ap.add_argument("--accel-weight-cap", type=float, default=10.0)
    ap.add_argument("--cb-beta", type=float, default=0.9999, help="effective 가중의 beta")
    ap.add_argument("--value-coef", type=float, default=0.5, help="0이면 critic을 학습하지 않는다")
    ap.add_argument("--gamma", type=float, default=0.999)
    ap.add_argument("--max-grad-norm", type=float, default=0.5)
    ap.add_argument("--weight-decay", type=float, default=0.0, help="0보다 크면 AdamW")
    ap.add_argument("--lr-schedule", choices=("none", "cosine", "plateau"), default="none")
    ap.add_argument("--min-lr", type=float, default=1e-5, help="cosine의 마지막 학습률")
    ap.add_argument("--patience", type=int, default=0, help="validation bc_loss가 이 에폭 동안 안 좋아지면 멈춘다(0이면 끝까지)")
    ap.add_argument("--seed", type=int, default=38001, help="초기화·섞기 난수")
    ap.add_argument("--split-seed", type=int, help="train/validation/test 분할 난수(없으면 --seed). 실험끼리 같게 고정한다")
    ap.add_argument("--device", default="cpu", help="cpu, cuda, cuda:1, auto(있으면 cuda)")
    ap.add_argument("--resume", action="store_true",
                    help="--out에 checkpoint.pt가 있으면 이어서 학습한다. 이미 끝난 실행(audit.json)은 건너뛴다")
    ap.add_argument("--tensorboard", action=argparse.BooleanOptionalAction, default=True,
                    help="<out>/tb에 TensorBoard 기록(기본 켜짐, --no-tensorboard로 끈다)")
    ap.add_argument("--data-list", help="npz 경로를 한 줄에 하나씩 적은 파일(상대 경로는 이 파일 기준). --data와 함께 써도 된다")
    a = ap.parse_args(argv)
    if not (np.isfinite(a.brake_weight) and a.brake_weight >= 1):
        ap.error("--brake-weight는 1 이상이어야 합니다")
    mask_idx = [int(v) for v in a.mask_obs_indices.split(",") if v.strip()]
    if mask_idx and a.input in ("features", "v2"):
        ap.error("--mask-obs-indices는 observation 입력(observation, both)에만 쓴다")
    if any(not 0 <= i < OBS_DIM for i in mask_idx):
        ap.error("--mask-obs-indices는 0~92")
    if a.device == "auto":
        a.device = "cuda" if torch.cuda.is_available() else "cpu"
    data_args = list(a.data or []) + read_data_list(a.data_list)
    if not data_args:
        ap.error("--data나 --data-list가 필요합니다")

    out = Path(a.out)
    if out.is_dir() and any(out.iterdir()):
        if not a.resume:
            raise FileExistsError(f"{out}가 이미 있습니다. 이어서 학습하려면 --resume")
        if (out / "audit.json").is_file():
            print(f"이미 끝난 실행이라 건너뜁니다: {out}", flush=True)
            return
    out.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)
    split_rng = np.random.default_rng(a.seed if a.split_seed is None else a.split_seed)
    paths = [resolve(p) for p in data_args]
    data, sources = load(paths, a.input, mask_idx)
    state = check_resume(out, a, sources)
    idx, split_info = make_split(data["key"], data["t"], split_rng, a.split_file)
    if len(idx["train"]) == 0 or len(idx["validation"]) == 0:
        raise ValueError("train/validation 분할이 비었습니다")

    layout = INPUT_LAYOUT[a.input]
    obs_dim = LAYOUTS[layout]["dim"]
    attention = dict(DEFAULT_ATTENTION, layout=layout, feature_size=a.feature_size, heads=a.heads,
                     embed_layers=parse_layers(a.embed_layers), output_layers=parse_layers(a.output_layers),
                     self_attention=a.self_attention, dropout=a.dropout, lane_action_mask=a.lane_action_mask,
                     lane_classes=data["n_lane"], accel_bins=a.accel_bins)
    base_config = json.loads(Path(a.config).read_text(encoding="utf-8")) if a.config else {}
    hp = {**PPO_DEFAULTS, **{k: v for k, v in (base_config.get("hparams") or {}).items()
                             if k in PPO_DEFAULTS}}
    hp.update(policy_type="ego_attention", attention=attention, init_log_std=a.init_log_std,
              lane_keep_bias=a.lane_keep_bias, seed=a.seed, device=a.device)
    agent = PPO(obs_dim, 2, **hp)
    trainer = Trainer(agent, data, a, agent.device)
    cw = trainer.set_class_weights(torch.as_tensor(idx["train"], device=agent.device))
    counts = cw["lane_counts"]
    n_params = sum(p.numel() for p in agent.policy.parameters() if p.requires_grad)
    print(f"rows={len(data['key'])} train/val/test={[len(idx[k]) for k in ('train', 'validation', 'test')]} "
          f"split={split_info['mode']} input={a.input}({obs_dim}) params={n_params} lane_counts={counts}", flush=True)

    sched = make_scheduler(trainer.optimizer, a)
    ck = {"epoch": 0, "best": float("inf"), "best_epoch": 0, "bad": 0, "stopped": False, "best_state": None}
    if state is not None:
        agent.policy.load_state_dict(state["net"])
        trainer.target.load_state_dict(state["target"])
        trainer.optimizer.load_state_dict(state["optimizer"])
        if sched is not None and state.get("scheduler") is not None:
            sched.load_state_dict(state["scheduler"])
        rng.bit_generator.state = state["np_rng"]
        torch.set_rng_state(state["torch_rng"].cpu())
        if state.get("cuda_rng") is not None and torch.cuda.is_available():
            torch.cuda.set_rng_state_all([s.cpu() for s in state["cuda_rng"]])
        ck = {k: state[k] for k in ck}
        if ck["best_state"] is not None:   # model.pt는 장치와 상관없이 CPU 텐서로 둔다
            ck["best_state"] = {k: v.cpu() for k, v in ck["best_state"].items()}
        print(f"resume: epoch {ck['epoch']}부터 이어서 (best epoch {ck['best_epoch']})", flush=True)
    hist_path = out / "history.jsonl"
    keep = [ln for ln in (hist_path.read_text(encoding="utf-8").splitlines() if hist_path.is_file() else [])
            if ln.strip() and json.loads(ln)["epoch"] <= ck["epoch"]]   # 체크포인트 뒤에 쓰인 줄은 버린다
    hist_path.write_text("".join(ln + "\n" for ln in keep), encoding="utf-8")
    tb = open_tensorboard(out, ck["epoch"]) if a.tensorboard else None

    with hist_path.open("a", encoding="utf-8") as hist:
        for epoch in range(ck["epoch"] + 1, a.epochs + 1):
            if ck["stopped"]:
                break
            t0 = time.time()
            trainer.train_epoch(idx["train"], rng)
            tr, va = trainer.evaluate(idx["train"]), trainer.evaluate(idx["validation"])
            if not (np.isfinite(tr["bc_loss"]) and np.isfinite(va["bc_loss"])):
                raise FloatingPointError(f"epoch {epoch}: 손실이 발산했습니다")
            lr = trainer.optimizer.param_groups[0]["lr"]
            row = {"epoch": epoch, "lr": lr, "time_s": time.time() - t0, "train": tr, "validation": va}
            hist.write(json.dumps(row, ensure_ascii=False) + "\n")
            hist.flush()
            if tb is not None:
                log_tensorboard(tb, epoch, row, trainer.n_lane)
            if va["bc_loss"] < ck["best"]:
                ck.update(best=va["bc_loss"], best_epoch=epoch, bad=0,
                          best_state={k: v.detach().cpu().clone() for k, v in agent.policy.state_dict().items()})
                atomic_save(ck["best_state"], out / "model.pt")
            else:
                ck["bad"] += 1
            if sched is not None:
                sched.step(va["bc_loss"]) if a.lr_schedule == "plateau" else sched.step()
            ck["epoch"] = epoch
            ck["stopped"] = bool(a.patience and ck["bad"] >= a.patience)
            atomic_save({**ck, "net": agent.policy.state_dict(), "target": trainer.target.state_dict(),
                         "optimizer": trainer.optimizer.state_dict(),
                         "scheduler": sched.state_dict() if sched is not None else None,
                         "np_rng": rng.bit_generator.state, "torch_rng": torch.get_rng_state(),
                         "cuda_rng": torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None},
                        out / "checkpoint.pt")
            if epoch == 1 or epoch % 10 == 0 or epoch == a.epochs or ck["stopped"]:
                print(f"epoch {epoch:4d} lr={lr:.2e} train bc={tr['bc_loss']:.4f} val bc={va['bc_loss']:.4f} "
                      f"acc_mae={va['accel_mae']:.3f} lane_acc={va['lane_acc']:.3f} "
                      f"lane_bal={va['lane_balanced_acc']} move_recall={va.get('lane_move_recall')} "
                      f"V_td={va['value_td']:.4f} ({row['time_s']:.1f}s)", flush=True)
            if ck["stopped"]:
                print(f"early stop: {a.patience} 에폭 동안 validation bc_loss가 좋아지지 않았다", flush=True)
    if ck["best_state"] is None:
        raise RuntimeError("학습한 에폭이 없습니다(--epochs 확인)")
    best_epoch = ck["best_epoch"]

    agent.policy.load_state_dict(ck["best_state"])
    atomic_save(ck["best_state"], out / "model.pt")
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
    processed_meta = paths[0].with_name("metadata.json")
    config = {**settings, "kind": "ego_attention_bc_pretraining", "hparams": hp, "input": a.input, "obs_dim": obs_dim,
              "data": sources, "reward": base_config.get("reward"),
              "data_metadata": json.loads(processed_meta.read_text(encoding="utf-8")) if processed_meta.is_file() else None,
              "created_at": datetime.now().astimezone().isoformat()}
    (out / "run_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    audit = {"split": split_info, "selected_epoch": best_epoch, "final": final,
             "attention_by_type_validation": trainer.attention_summary(idx["validation"]),
             "lane_counts_train": counts, "lane_weights": cw["lane_weights"], "class_weights": cw,
             "trainable_parameters": n_params,
             "loss": {"accel": (f"구간 {a.accel_bins}개 CE({a.accel_weighting} 가중, 상한 {a.accel_weight_cap})"
                                if a.accel_bins else "MSE") + f", 제동(<{BRAKE_THRESHOLD}) 표본 가중 {a.brake_weight}",
                      "lane": f"{a.lane_weighting} 클래스 가중 CE x {a.lane_weight}(최다 클래스 대비 상한 {a.lane_weight_cap})",
                      "value": f"smooth L1 TD(0) x {a.value_coef}, gamma {a.gamma}, 몸통 detach, 에폭마다 target 갱신"},
             "selection": "validation bc_loss 최소; test는 선택에 쓰지 않음", "save_load_parity": True,
             "epochs_run": ck["epoch"], "early_stopped": ck["stopped"],
             "args": vars(a)}
    if tb is not None:   # 선택한 모델의 최종 지표(선택 에폭 위치에 점 하나)
        for split in ("validation", "test"):
            for k in TB_KEYS:
                if final[split] and final[split].get(k) is not None:
                    tb.add_scalar(f"final_{split}/{k}", final[split][k], best_epoch)
        tb.close()
    (out / "audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    shutil.copy2(__file__, out / Path(__file__).name)
    print(f"done: best epoch {best_epoch}, test={final['test']}", flush=True)
    print(f"attention by type (validation): {audit['attention_by_type_validation']}", flush=True)


if __name__ == "__main__":
    main()
