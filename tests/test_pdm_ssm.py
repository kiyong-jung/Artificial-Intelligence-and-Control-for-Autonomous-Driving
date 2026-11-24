"""PDM 버스 데이터: SSM 장치 vType과 출력 파서(pdm_data/ssm.py)."""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pdm_data.ssm import parse_ssm, ssm_vtype  # noqa: E402
from traffic.pdm_bus import BUS_PDM_VTYPE  # noqa: E402

SAMPLE_SSM = """<?xml version="1.0" encoding="UTF-8"?>
<SSMLog>
    <conflict begin="100.00" end="103.50" ego="bus0" foe="car7">
        <conflictTypes values="2 2 2"/>
        <minTTC time="101.20" position="1.00,2.00" type="2" value="1.80" speed="5.00"/>
        <maxDRAC time="101.10" position="1.00,2.00" type="2" value="2.40" speed="5.00"/>
        <PET time="NA" position="NA" type="NA" value="NA" speed="NA"/>
    </conflict>
    <conflict begin="200.00" end="201.00" ego="bus0" foe="car9">
        <conflictTypes values="5"/>
        <minTTC time="NA" position="NA" type="NA" value="NA" speed="NA"/>
        <maxDRAC time="NA" position="NA" type="NA" value="NA" speed="NA"/>
        <PET time="200.50" position="3.00,4.00" type="5" value="0.90" speed="4.00"/>
    </conflict>
</SSMLog>
"""


def test_ssm_vtype_is_valid_xml_with_params():
    s = ssm_vtype(BUS_PDM_VTYPE, "D:/x/ssm.xml")
    root = ET.fromstring(s)
    params = {p.get("key"): p.get("value") for p in root.findall("param")}
    assert root.get("id") == "bus_test" and root.get("carFollowModel") == "IDM"
    assert params["has.ssm.device"] == "true" and params["device.ssm.file"] == "D:/x/ssm.xml"
    assert params["device.ssm.measures"] == "TTC DRAC PET BR SGAP TGAP"
    assert params["device.ssm.thresholds"] == "4.0 3.0 2.0 0.0 0.2 0.5"


def test_parse_ssm_reads_min_ttc_and_na(tmp_path):
    p = tmp_path / "ssm.xml"
    p.write_text(SAMPLE_SSM, encoding="utf-8")
    c = parse_ssm(str(p))
    assert c[0]["ego"] == "bus0" and c[0]["foe"] == "car7" and c[0]["min_ttc"]["value"] == 1.8
    assert c[0]["pet"] is None and c[0]["types"] == ["2", "2", "2"] and c[0]["begin"] == 100.0
    assert c[1]["min_ttc"] is None and c[1]["pet"]["value"] == 0.9 and c[1]["pet"]["time"] == 200.5


def test_parse_ssm_missing_file_is_empty(tmp_path):
    assert parse_ssm(str(tmp_path / "none.xml")) == []
    (tmp_path / "empty.xml").write_text("", encoding="utf-8")
    assert parse_ssm(str(tmp_path / "empty.xml")) == []
