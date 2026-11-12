"""수업 양식(00조_칼럼설명.pptx)에 컬럼 표(env/data_schema.py)를 채운다.

    python scripts/fill_column_slide.py 00조_칼럼설명.pptx data/submission/00조_칼럼설명.pptx

필수 항목은 표에 있으면 포함 여부에 o를 적고, 팀 추가 항목은 빈 행을 복사해 칼럼명·형태·의미를 적는다.
관측 차원(양식의 19-dim)은 실제 차원으로 바꾼다. 필수 항목이 표에 없으면 x를 적고 끝낼 때 알린다.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path
from xml.sax.saxutils import escape
import zipfile

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from env.data_schema import COLUMNS, OBSERVATION_FEATURES, TEAM

SLIDE = "ppt/slides/slide1.xml"
ROW = re.compile(r"<a:tr\b.*?</a:tr>", re.S)
CELL = re.compile(r"<a:tc\b.*?</a:tc>", re.S)
EMPTY = re.compile(r"<a:p><a:endParaRPr[^>]*>.*?</a:endParaRPr></a:p>", re.S)
RUN = ('<a:p><a:r><a:rPr lang="en-US" altLang="ko-KR" sz="1000" b="0" dirty="0"><a:solidFill>'
       '<a:srgbClr val="23272E"/></a:solidFill></a:rPr><a:t>{}</a:t></a:r></a:p>')


def text(cell: str) -> str:
    return "".join(re.findall(r"<a:t>(.*?)</a:t>", cell))


def write(row: str, values: dict[int, str]) -> str:
    cells = CELL.findall(row)
    for index, value in values.items():
        filled, count = EMPTY.subn(lambda _: RUN.format(escape(value)), cells[index], count=1)
        if not count:
            raise ValueError(f"양식의 칸이 비어 있지 않습니다: {text(cells[index])!r}")
        row = row.replace(cells[index], filled, 1)
    return row


def main(template: str, output: str) -> int:
    with zipfile.ZipFile(template) as source:
        parts = {name: source.read(name) for name in source.namelist()}
    slide = parts[SLIDE].decode("utf-8").replace("19-dim", f"{len(OBSERVATION_FEATURES)}-dim")
    rows = ROW.findall(slide)
    known = {name for name, *_ in COLUMNS}
    missing = []
    blank = next(row for row in rows if not text(row))
    for row in rows:
        name = text(CELL.findall(row)[0])
        if not name or "항목" in name:
            continue
        missing += [] if name in known else [name]
        slide = slide.replace(row, write(row, {3: "o" if name in known else "x"}), 1)
    team = "".join(write(blank, {0: name, 1: kind, 2: meaning})
                   for name, kind, origin, meaning in COLUMNS if origin == TEAM)
    first = slide.index(blank)
    slide = slide[:first] + team + slide[first:].replace(blank, "")
    parts[SLIDE] = slide.encode("utf-8")
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as target:
        for name, content in parts.items():
            target.writestr(name, content)
    print(f"{output}: 필수 항목 {len(rows) - 4 - len(missing)}개 포함, 팀 추가 항목 "
          f"{sum(origin == TEAM for *_, origin, _ in COLUMNS)}개")
    if missing:
        print("표에 없는 필수 항목:", ", ".join(missing))
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main(*sys.argv[1:3]))
