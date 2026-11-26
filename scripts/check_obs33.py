"""관측 33차원·3값 차로 라벨 실데이터 점검(docs/superpowers/plans/2026-11-26-obs33-mlp-bc.md Task 8).

    D:/conda_envs/sumo/python.exe scripts/check_obs33.py data/pdm_bus_sumo_lc/busy_s0 [다른 run ...]

run마다 processed/obs33_check.json과 표준 출력에 다음을 남긴다.
  - features: 33개 특징의 min, max, 기본값(없음·포화) 비율
  - lane: ±1 구간 수와 길이(행) 분포, 원시 행에서 센 차로 변경 횟수(같은 도로 번호 변화 또는 옆 위치가 차로 폭 절반 넘게 뜀),
          라벨이 0인데 차로가 바뀐 행 수(unlabeled_changes, 0이어야 한다), 차로가 두 번 바뀐 ±1 구간 수(설계 6절의 드문 경우)
  - stops: 정차를 마친 구간(reward_terms.service > 0) 수와, 그 행의 신호가 0이고 다음 행의 신호가 1인 수(같아야 한다)
  - zero_connectivity: 현재 차로 연결성이 0인 행 수와 도로별 개수
"""
import collections
import gzip
import json
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from pdm_data.labels import lane_change_labels3  # noqa: E402
from pdm_data.lap_context import load_lap_context  # noqa: E402
from pdm_data.obs33 import FEATURE_NAMES_33, Obs33Context  # noqa: E402
from pdm_data.recorder import read_steps  # noqa: E402
from pdm_data.transitions import episodes  # noqa: E402

DEFAULTS = {"ego_speed": 0.0, "stop_point_distance": 1.0, "signal_state": 1.0}
for _n in FEATURE_NAMES_33:
    for suffix, v in (("leader_gap", 1.0), ("leader_relative_speed", 1.0), ("follower_gap", -1.0),
                      ("follower_relative_speed", -1.0), ("route_connectivity", -1.0), ("density", 1.0)):
        if _n.endswith(suffix):
            DEFAULTS[_n] = v


def runs_of(labels, episode):
    """같은 에피소드 안 같은 0 아닌 값의 연속 구간 길이 목록."""
    out, cur, prev = [], 0, None
    for v, e in zip(labels, episode):
        key = (int(e), int(v)) if v else None
        if key is not None and key == prev:
            cur += 1
        else:
            if prev is not None:
                out.append(cur)
            cur = 1 if key is not None else 0
        prev = key
    if prev is not None:
        out.append(cur)
    return out


def _changed(ep, r, width):
    w = ep[r].get("window")
    if w is None:
        return False
    if w["edge_before"] == w["edge_after"] and not w["edge_before"].startswith(":") and w["lane_before"] != w["lane_after"]:
        return True
    if r + 1 < len(ep):
        a, b = ep[r]["ego"].get("lateral_position"), ep[r + 1]["ego"].get("lateral_position")
        return a is not None and b is not None and abs(b - a) > width(ep[r + 1]["ego"]["lane_id"]) / 2
    return False


def raw_lane_changes(run_dir, net):
    """(차로 변경 수, 라벨 0인 변경 수, 두 번 바뀐 ±1 구간 수). 라벨은 원시 에피소드로 다시 계산한다(전처리와 같은 함수)."""
    width = lambda lid: net.getLane(lid).getWidth()  # noqa: E731
    total = unlabeled = multi = 0
    for ep in episodes(list(read_steps(run_dir))):
        lab = lane_change_labels3(ep, width)
        changed = [_changed(ep, r, width) for r in range(len(ep))]
        total += sum(changed)
        unlabeled += sum(1 for r, c in enumerate(changed) if c and lab[r] == 0)
        r = 0
        while r < len(lab):
            if lab[r] == 0:
                r += 1
                continue
            s0 = r
            while r + 1 < len(lab) and lab[r + 1] == lab[s0]:
                r += 1
            multi += int(sum(changed[s0:r + 1]) > 1)
            r += 1
    return total, unlabeled, multi


def check(run_dir):
    z = np.load(os.path.join(run_dir, "processed", "transitions.npz"))
    obs, act, ep = z["observation_33"], z["action_33"], z["episode"]
    feats = {}
    for i, name in enumerate(FEATURE_NAMES_33):
        col = obs[:, i]
        feats[name] = {"min": float(col.min()), "max": float(col.max()),
                       "default_ratio": float(np.mean(np.isclose(col, DEFAULTS[name])))}
    seg = runs_of(act[:, 1], ep)
    c33 = Obs33Context(load_lap_context(run_dir))
    changes, unlabeled, multi = raw_lane_changes(run_dir, c33.net)
    I = {n: i for i, n in enumerate(FEATURE_NAMES_33)}
    sig = obs[:, I["signal_state"]]
    zero = np.where(obs[:, I["current_route_connectivity"]] == 0.0)[0]
    zs, roads, served = set(zero.tolist()), collections.Counter(), []
    with gzip.open(os.path.join(run_dir, "processed", "transitions.jsonl.gz"), "rt", encoding="utf-8") as f:
        for k, line in enumerate(f):
            r = json.loads(line)
            if k in zs:
                roads[r["state"]["ego"]["road_id"]] += 1
            if r["reward_terms"]["service"] > 0:
                served.append(k)
    green = sum(1 for k in served if sig[k] == 0.0 and (k + 1 >= len(obs) or ep[k + 1] != ep[k] or sig[k + 1] == 1.0))
    out = {"rows": int(len(obs)), "episodes": int(len(set(ep.tolist()))),
           "in_range": bool(np.abs(obs).max() <= 1.0 + 1e-6),
           "lane": {"segments": len(seg), "raw_lane_changes": changes, "unlabeled_changes": unlabeled,
                    "multi_change_segments": multi, "segment_rows": dict(sorted(collections.Counter(seg).items())),
                    "label_counts": {str(k): int(v) for k, v in zip(*np.unique(act[:, 1], return_counts=True))}},
           "stops": {"services": len(served), "red_then_green": green},
           "zero_connectivity": {"rows": int(len(zero)), "roads": dict(roads)},
           "features": feats}
    with open(os.path.join(run_dir, "processed", "obs33_check.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    return out


def main(argv=None):
    for run_dir in (argv if argv is not None else sys.argv[1:]):
        r = check(run_dir)
        print(f"== {run_dir}: 행 {r['rows']}, 에피소드 {r['episodes']}, 범위 안 {r['in_range']}")
        print(f"   차로: ±1 구간 {r['lane']['segments']}, 원시 차로 변경 {r['lane']['raw_lane_changes']}, "
              f"라벨 없는 변경 {r['lane']['unlabeled_changes']}, 두 번 바뀐 구간 {r['lane']['multi_change_segments']}, "
              f"구간 길이(행) {r['lane']['segment_rows']}, 라벨 {r['lane']['label_counts']}")
        print(f"   정차 완료 {r['stops']['services']}, 그때 적색 -> 다음 행 녹색 {r['stops']['red_then_green']}")
        print(f"   현재 차로 연결성 0인 행 {r['zero_connectivity']['rows']} {r['zero_connectivity']['roads']}")
        for name, f in r["features"].items():
            print(f"   {name:40s} min {f['min']:+.3f} max {f['max']:+.3f} 기본값 {f['default_ratio']:.3f}")


if __name__ == "__main__":
    main()
