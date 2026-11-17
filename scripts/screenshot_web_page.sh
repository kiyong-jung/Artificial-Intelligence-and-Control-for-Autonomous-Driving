#!/bin/zsh
# web_page/index.html을 헤드리스 Chrome으로 찍는다.  scripts/screenshot_web_page.sh <출력 폴더>  → desktop.png(1440 폭), mobile.png(390 폭)
out=${1:-.}; mkdir -p $out
page="file://${0:A:h:h}/web_page/index.html"
chrome="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
"$chrome" --headless=new --disable-gpu --hide-scrollbars --window-size=1440,5200 --screenshot=$out/desktop.png $page 2>/dev/null
"$chrome" --headless=new --disable-gpu --hide-scrollbars --window-size=390,7600 --screenshot=$out/mobile.png $page 2>/dev/null
ls -la $out/desktop.png $out/mobile.png
