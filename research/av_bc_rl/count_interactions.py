"""수집한 NPZ에서 버스가 다른 차·보행자와 얽힌 장면을 교통량별 바퀴당 횟수로 센다. 관측값(50m 범위)만 쓴다.

    python research/av_bc_rl/count_interactions.py data/final

  stop            정류장 정차 시작            stop_follower    그때 같은 차로 뒤 50m 안에 차
  left, right     차로 변경 요청 시작(환경이 거절한 요청 포함)
  *_follower30    그때 옮길 차로 뒤 30m 안에 차
  crosswalk       횡단보도 50m 안 접근(신호 있는 곳 포함)   crosswalk_busy  그 접근 중 보행자가 건넘
  cut_in          요청 없이 달리는 중 내 차로 앞차 간격이 한 판단(0.5초)에 10m 넘게 줄어든 순간
"""
import collections
import glob
import sys

import numpy as np

sys.path.insert(0, ".")
from env.data_schema import OBSERVATION_FEATURES as F  # noqa: E402

VISIBILITY = 50.0
COLUMN = {name: F.index(name) for name in (
    "ego_speed", "current_leader_gap", "current_follower_gap", "left1_follower_gap", "right1_follower_gap",
    "stop_dwell_remaining", "crosswalk_distance", "crosswalk_occupied")}


def count(path, c):
    z = np.load(path)
    o, episode, lane = z["observation"], z["episode"], z["lane_change"].astype(int)
    col = {name: o[:, index] for name, index in COLUMN.items()}
    new = np.r_[True, episode[1:] != episode[:-1]]
    previous_lane = np.r_[0, lane[:-1]]
    c["laps"] += len(np.unique(episode))
    dwell = col["stop_dwell_remaining"] > 0
    start = dwell & (~np.r_[False, dwell[:-1]] | new)
    c["stop"] += start.sum()
    c["stop_follower"] += (start & (col["current_follower_gap"] > -0.999)).sum()   # 뒤차 없음은 -1
    for name, direction, follower in (("left", 1, "left1_follower_gap"), ("right", -1, "right1_follower_gap")):
        onset = (lane == direction) & ((previous_lane != direction) | new)
        c[name] += onset.sum()
        c[f"{name}_follower30"] += (onset & (col[follower] > -30.0 / VISIBILITY)).sum()
    near = col["crosswalk_distance"] < 0.5
    first = near & (~np.r_[False, near[:-1]] | new)
    ids = np.cumsum(first) * near
    busy = np.zeros(ids.max() + 1, bool)
    np.logical_or.at(busy, ids, col["crosswalk_occupied"] > 0.5)
    busy[0] = False
    c["crosswalk"] += first.sum()
    c["crosswalk_busy"] += busy.sum()
    gap = col["current_leader_gap"] * VISIBILITY
    c["cut_in"] += (~new & (np.r_[0, gap[:-1] - gap[1:]] > 10) & (lane == 0) & (previous_lane == 0)
                    & (col["ego_speed"] > 0.05)).sum()


def main():
    totals = collections.defaultdict(collections.Counter)
    for path in sorted(glob.glob(f"{sys.argv[1]}/*.npz")):
        count(path, totals[path.split("/")[-1].split("_")[0]])
    for scenario, c in totals.items():
        laps = c.pop("laps")
        print(scenario, laps, {key: round(float(value) / laps, 2) for key, value in c.items()})


if __name__ == "__main__":
    main()
