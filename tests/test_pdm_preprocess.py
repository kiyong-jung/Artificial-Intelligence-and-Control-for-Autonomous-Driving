"""PDM 버스 데이터 전처리(pdm_data/transitions.py, scripts/preprocess_pdm_bus.py)."""
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(__file__)
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)

from env.data_schema import lookup  # noqa: E402
from pdm_fixtures import make_run_dir  # noqa: E402
from pdm_data.transitions import build_transitions  # noqa: E402

REQUIRED = [line.split("|")[1].strip() for line in open(os.path.join(ROOT, "scripts", "data_colum.md"), encoding="utf-8")
            if line.startswith("| ") and "포함 여부" not in line and "---" not in line]
SCRIPT = os.path.join(ROOT, "scripts", "preprocess_pdm_bus.py")


def test_required_columns_present(tmp_path):
    tr, _ = build_transitions(make_run_dir(tmp_path), 1.5, 3.0)
    assert len(REQUIRED) == 19 and tr          # data_colum.md 표의 행 수(설계 문서의 "23"은 셈 실수)
    for rec in tr:
        for col in REQUIRED:
            lookup(rec, col)
        assert len(rec["observation"]) == 93 and len(rec["action_raw"]) == 2


def test_next_observation_is_next_row(tmp_path):
    tr, _ = build_transitions(make_run_dir(tmp_path), 1.5, 3.0)
    assert all(a["next_observation"] == b["observation"] for a, b in zip(tr, tr[1:]) if a["episode"] == b["episode"])
    assert all(a["next_state"]["ego"] == b["state"]["ego"] for a, b in zip(tr, tr[1:]))


def test_episode_t_contiguous_and_terminal(tmp_path):
    tr, stats = build_transitions(make_run_dir(tmp_path), 1.5, 3.0)
    assert [r["t"] for r in tr] == list(range(len(tr))) and len(tr) == 11
    assert tr[-1]["terminated"] and tr[-1]["event"]["termination_reason"] == "incomplete_service"
    assert tr[-1]["reward"] == -5.0 and not any(r["terminated"] for r in tr[:-1])
    assert stats["end_reasons"] == {"incomplete_service": 1}


def test_accel_label_uses_given_scale(tmp_path):
    tr, stats = build_transitions(make_run_dir(tmp_path), 1.5, 3.0)
    assert tr[0]["action"]["accel_raw"] == pytest.approx(0.4 / 1.5) and tr[0]["action_extra"]["accel_mps2"] == pytest.approx(0.4)
    assert stats["accel_clip_ratio"] == 0.0


def test_expert_columns_from_window(tmp_path):
    import gzip
    import json
    d = make_run_dir(tmp_path)
    path = os.path.join(d, "steps.jsonl.gz")
    rows = [json.loads(x) for x in gzip.open(path, "rt", encoding="utf-8")]
    rows[0]["expert"]["pdm"] = {"sim_time": 99.9, "fraction": 0.4, "v0": 5.6}
    rows[0]["window"]["pdm_plans"] = [{"sim_time": 100.2, "fraction": 0.6, "v0": 8.3}]
    for s in rows[0]["window"]["substeps"]:
        s["phase"] = "request"
    with gzip.open(path, "wt", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
    tr, _ = build_transitions(d, 1.5, 3.0)
    assert tr[0]["expert"]["pdm_fraction"] == 0.6 and tr[0]["expert"]["phase"] == "request"
    assert tr[1]["expert"]["pdm_fraction"] is None


def test_preprocess_adds_offline_s5_events(tmp_path):
    import gzip
    import json
    d = make_run_dir(tmp_path)
    path = os.path.join(d, "steps.jsonl.gz")
    rows = [json.loads(x) for x in gzip.open(path, "rt", encoding="utf-8")]
    car = {"id": "inj_S51", "x": 0.0, "y": 0.0, "speed": 5.0, "heading": 0.0}
    rows[0]["vehicles"] = [dict(car, road_id="G23", lane_position=30.0)]
    rows[1]["vehicles"] = [dict(car, road_id="R69", lane_position=rows[1]["ego"]["lane_position"] + 20.0)]
    with gzip.open(path, "wt", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
    meta_path = os.path.join(d, "metadata.json")
    meta = json.load(open(meta_path, encoding="utf-8"))
    meta["route"]["s5_feeders"] = ["G23"]
    json.dump(meta, open(meta_path, "w", encoding="utf-8"))
    subprocess.run([sys.executable, SCRIPT, d, "--accel-pos", "1.5", "--accel-neg", "3.0"], check=True, capture_output=True)
    events = [json.loads(x) for x in open(os.path.join(d, "processed", "events.jsonl"), encoding="utf-8")]
    assert [e["ids"] for e in events if e["kind"] == "s5_cutin"] == [["inj_S51"]]


def test_max_lap_s_truncates(tmp_path):
    tr, stats = build_transitions(make_run_dir(tmp_path), 1.5, 3.0, max_lap_s=2.0)
    assert len(tr) == 4 and tr[-1]["truncated"] and tr[-1]["event"]["termination_reason"] == "time_limit"


def test_missing_scale_exits_2(tmp_path):
    p = subprocess.run([sys.executable, SCRIPT, make_run_dir(tmp_path)], capture_output=True, text=True)
    assert p.returncode == 2 and "--report-accel" in p.stderr


def test_report_accel_writes_only_report(tmp_path):
    d = make_run_dir(tmp_path)
    subprocess.run([sys.executable, SCRIPT, d, "--report-accel"], check=True, capture_output=True)
    assert os.path.exists(os.path.join(d, "processed", "accel_report.json"))
    assert not os.path.exists(os.path.join(d, "processed", "transitions.jsonl.gz"))


def test_full_preprocess_writes_outputs(tmp_path):
    import numpy as np
    d = make_run_dir(tmp_path)
    subprocess.run([sys.executable, SCRIPT, d, "--accel-pos", "1.5", "--accel-neg", "3.0"], check=True, capture_output=True)
    for f in ("transitions.jsonl.gz", "transitions.npz", "events.jsonl", "summary.json", "metadata.json"):
        assert os.path.exists(os.path.join(d, "processed", f)), f
    z = np.load(os.path.join(d, "processed", "transitions.npz"))
    assert z["observation"].shape == (11, 93) and z["features"].shape[0] == 11 and z["action_raw"].shape == (11, 2)


def test_preprocess_writes_v2_columns(tmp_path):
    import numpy as np
    d = make_run_dir(tmp_path)
    subprocess.run([sys.executable, SCRIPT, d, "--accel-pos", "1.5", "--accel-neg", "3.0"], check=True, capture_output=True)
    z = np.load(os.path.join(d, "processed", "transitions.npz"))
    assert z["observation_v2"].shape == (11, 166) and z["next_observation_v2"].shape == (11, 166)
    assert z["prev_action_raw"].shape == (11, 2) and list(z["prev_action_raw"][0]) == [0.0, 0.0]
    np.testing.assert_array_equal(z["prev_action_raw"][1:], z["action_raw"][:-1])
    np.testing.assert_array_equal(z["next_observation_v2"][:-1], z["observation_v2"][1:])
    assert np.abs(z["observation_v2"]).max() <= 1.0


def test_npz_has_obs33_columns(tmp_path):
    import json
    import numpy as np
    d = make_run_dir(tmp_path)
    subprocess.run([sys.executable, SCRIPT, d, "--accel-pos", "1.5", "--accel-neg", "3.0"], check=True, capture_output=True)
    z = np.load(os.path.join(d, "processed", "transitions.npz"))
    assert z["observation_33"].shape == (11, 33) and z["next_observation_33"].shape == (11, 33)
    assert z["action_33"].shape == (11, 2)
    np.testing.assert_array_equal(z["action_33"][:, 0], z["action_raw"][:, 0])
    assert set(z["action_33"][:, 1].tolist()) <= {-1.0, 0.0, 1.0}
    np.testing.assert_array_equal(z["next_observation_33"][:-1], z["observation_33"][1:])
    assert np.abs(z["observation_33"]).max() <= 1.0
    with open(os.path.join(d, "processed", "metadata.json"), encoding="utf-8") as f:
        assert len(json.load(f)["obs33_names"]) == 33
