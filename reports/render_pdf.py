"""reports/ 아래 Markdown 보고서를 A4 PDF로 만든다. Chrome(또는 Chromium)이 있어야 한다.

    python reports/render_pdf.py reports/bus_av_learning_summary.md

같은 이름의 .pdf를 옆에 쓰고 쪽수를 출력한다. 표와 글꼴 크기는 아래 CSS가 정한다.
"""
import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import markdown

CSS = """
@page { size: A4; margin: 13mm 14mm 12mm 14mm; }
body { font-family: "Apple SD Gothic Neo", "Malgun Gothic", "Noto Sans CJK KR", sans-serif;
       font-size: 9.1pt; line-height: 1.42; color: #1b1b1b; }
h1 { font-size: 14.5pt; margin: 0 0 2pt; }
h2 { font-size: 11pt; margin: 9pt 0 3pt; padding-bottom: 1pt; border-bottom: 1px solid #c8c8c8; }
h3 { font-size: 9.8pt; margin: 6pt 0 2pt; }
p, ul, ol { margin: 2pt 0 4pt; }
ul, ol { padding-left: 15pt; }
li { margin: 1pt 0; }
table { border-collapse: collapse; width: 100%; margin: 3pt 0 5pt; font-size: 8.3pt; }
th, td { border: 1px solid #c8c8c8; padding: 1.5pt 4pt; text-align: left; vertical-align: top; }
th { background: #eef1f4; }
code { font-size: 8.3pt; background: #f2f2f2; padding: 0 2pt; }
"""
CHROME_CANDIDATES = ("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
                     "google-chrome", "chromium", "chromium-browser", "chrome")


def find_chrome():
    for candidate in CHROME_CANDIDATES:
        path = shutil.which(candidate) or (candidate if Path(candidate).exists() else None)
        if path:
            return path
    sys.exit("Chrome을 찾지 못했다.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source")
    args = parser.parse_args()
    source = Path(args.source).resolve()
    body = markdown.markdown(source.read_text(encoding="utf-8"), extensions=["tables"])
    title = re.search(r"<h1>(.*?)</h1>", body)
    html = (f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><title>{title.group(1) if title else source.stem}"
            f"</title><style>{CSS}</style></head><body>{body}</body></html>")
    out = source.with_suffix(".pdf")
    with tempfile.TemporaryDirectory() as tmp:
        page = Path(tmp) / "report.html"
        page.write_text(html, encoding="utf-8")
        subprocess.run([find_chrome(), "--headless", "--disable-gpu", "--no-pdf-header-footer",
                        f"--print-to-pdf={out}", page.as_uri()], check=True, capture_output=True)
    pages = len(re.findall(rb"/Type\s*/Page[^s]", out.read_bytes()))
    print(f"{out} ({pages}쪽)")


if __name__ == "__main__":
    main()
