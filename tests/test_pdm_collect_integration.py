"""PDM 버스 수집기 통합 시험(SUMO 필요). PDM_INTEGRATION=1일 때만 돈다. 짧게: warmup 30 s, 60 s 실행."""
import hashlib
import json
import os
import sys
import tempfile

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts"))

pytestmark = pytest.mark.skipif(os.environ.get("PDM_INTEGRATION") != "1", reason="SUMO 통합 시험")


def _sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def _run(name, *extra):
    import collect_pdm_bus
    out = tempfile.mkdtemp(prefix="pdm_it_")
    args = collect_pdm_bus.parse_args(["--warmup", "30", "--max-seconds", "60", "--name", name, "--out-dir", out,
                                       "--work-dir", os.path.join(out, "work"), *extra])
    return collect_pdm_bus.collect(args)


@pytest.fixture(scope="module")
def short_run():
    return _run("it_rec", "--check-obs", "100")


@pytest.fixture(scope="module")
def short_run_no_record():
    return _run("it_norec", "--no-record")


def test_short_run_writes_all_files(short_run):
    from pdm_data.recorder import read_steps
    d = short_run["run_dir"]
    for f in ("steps.jsonl.gz", "events.jsonl", "ssm.xml", "metadata.json", "assets/network.net.xml"):
        assert os.path.exists(os.path.join(d, f)), f
    meta = json.load(open(os.path.join(d, "metadata.json"), encoding="utf-8"))
    assert meta["schema_version"] == "pdm_bus_raw_v1"
    assert meta["net_sha256"] == _sha(os.path.join(d, "assets/network.net.xml"))
    rows = list(read_steps(d))
    assert len(rows) >= 100 and rows[-1]["meta"]["final"] and rows[-1]["meta"]["end"] == "time_limit"
    assert any(r["expert"]["pdm"] and len(r["expert"]["pdm"]["scores"]) == 5 for r in rows)
    assert sum(1 for r in rows if r.get("check", {}).get("obs93_online")) == 100


def test_recording_does_not_change_bus(short_run, short_run_no_record):
    a, b = short_run["bus_trace"], short_run_no_record["bus_trace"]
    n = min(len(a), len(b))
    assert n >= 100
    assert all(abs(p[3] - q[3]) < 1e-9 and abs(p[1] - q[1]) < 1e-6 and abs(p[2] - q[2]) < 1e-6
               for p, q in zip(a[:n], b[:n]))
