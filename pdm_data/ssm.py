"""SUMO SSM 장치(설계 9절): 시험 버스 vType에 파라미터로 붙이고, 출력 파일의 상충을 읽는다."""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from xml.sax.saxutils import quoteattr

SSM_PARAMS = {
    "has.ssm.device": "true",
    "device.ssm.measures": "TTC DRAC PET BR SGAP TGAP",
    "device.ssm.thresholds": "4.0 3.0 2.0 0.0 0.2 0.5",
    "device.ssm.range": "50",
    "device.ssm.extratime": "5",
    "device.ssm.trajectories": "false",
}


def ssm_vtype(base_vtype: str, out_file: str) -> str:
    """자기 닫힘 <vType .../>에 SSM 파라미터를 넣는다. 다른 속성은 그대로 둔다."""
    head = base_vtype.strip()
    if not head.endswith("/>"):
        raise ValueError("자기 닫힘 vType 문자열이어야 한다")
    params = dict(SSM_PARAMS, **{"device.ssm.file": out_file})
    body = "".join(f"<param key={quoteattr(k)} value={quoteattr(v)}/>" for k, v in params.items())
    return head[:-2].rstrip() + ">" + body + "</vType>"


def _measure(el):
    if el is None or el.get("value") in (None, "NA"):
        return None
    return {"time": float(el.get("time")), "value": float(el.get("value"))}


def parse_ssm(path: str) -> list:
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return []
    out = []
    for c in ET.parse(path).getroot().iter("conflict"):
        types = c.find("conflictTypes")
        out.append({"begin": float(c.get("begin")), "end": float(c.get("end")), "ego": c.get("ego"), "foe": c.get("foe"),
                    "min_ttc": _measure(c.find("minTTC")), "max_drac": _measure(c.find("maxDRAC")),
                    "pet": _measure(c.find("PET")),
                    "types": types.get("values").split() if types is not None and types.get("values") else []})
    return out
