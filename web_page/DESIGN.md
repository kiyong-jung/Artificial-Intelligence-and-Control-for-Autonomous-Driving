---
version: alpha
name: hapjeong-bus-project-page
description: 합정역 자율주행버스 팀 프로젝트 페이지(index.html)의 디자인 시스템. 레이아웃은 Stanford OpenWAM 같은 논문 페이지(고정 상단 바, 가운데 정렬 히어로, 테두리 그림 상자, 요약, 왼쪽 목차 + 본문)를 따른다. 색은 버스 노선도에서 가져왔다. 파랑은 노선, 노랑은 정류장이다. 이 파일의 값이 index.html 의 :root 와 어긋나면 index.html 을 고친다.

colors:
  paper: "#f3f6f8"
  surface: "#ffffff"
  ink: "#12202e"
  body: "#1f2d3b"
  muted: "#556577"
  rule: "#d4dce4"
  rule-soft: "#e4e9ee"
  rule-strong: "#9fb0c0"
  tone: "#e6ecf2"
  tone-deep: "#d6dfe8"
  accent: "#0a58b5"
  accent-deep: "#08458f"
  accent-faint: "#0a58b540"
  stop: "#f5b800"
  on-accent: "#ffffff"

typography:
  font: "Pretendard Variable (1.3.9, CDN), Apple SD Gothic Neo, Noto Sans KR, system-ui"
  display: { size: "clamp(2.8rem, 7vw, 5.4rem)", weight: 800, lineHeight: 1.15, letterSpacing: -.04em }
  title: { size: "clamp(1.15rem, 2.2vw, 1.5rem)", weight: 500, lineHeight: 1.5, letterSpacing: -.02em }
  h2: { size: "clamp(1.6rem, 2.6vw, 2.1rem)", weight: 800, lineHeight: 1.3, letterSpacing: -.03em }
  xl: { size: 1.5rem, weight: 700, lineHeight: 1.5, use: "pull quote, 요약 제목, 극장 제목(800)" }
  xl-m: { size: 1.25rem, use: "860px 이하에서 xl 대신 (당김 글, 극장 제목)" }
  stat: { size: 1.625rem, weight: 700, lineHeight: 1.15, letterSpacing: -.03em }
  lead: { size: 1.0625rem, weight: 400, lineHeight: 1.75, use: "lede, 요약 본문, 골목 카드 수치" }
  body: { size: 1rem, weight: 400, lineHeight: 1.75 }
  ui: { size: .9375rem, weight: 500, use: "상단 링크, 목차, 통계 라벨" }
  small: { size: .875rem, use: "캡션, 표 머리글, 푸터" }
  weights: [400, 500, 600, 700, 800]

spacing: { s-1: 4px, s-2: 8px, s-3: 12px, s-4: 16px, s-5: 24px, s-6: 32px, s-7: 48px, s-8: 64px, s-9: 96px }
radius: { xs: 4px, sm: 8px, lg: 12px, pill: 999px }
layout:
  wide: 1200px
  figure: 1000px
  measure: 720px
  toc: 200px
  nav: 64px
  tap: 44px
  mark: 12px
  gutter: "32px, 860px 이하 16px"
  breakpoints: [1100px, 860px]
motion: { fast: 150ms, med: 300ms, slow: 700ms }
---

## Overview

논문 프로젝트 페이지다. 읽는 사람은 같은 수업의 학생과 교수이고, 목적은 하루 안에 훑어 읽는 것이다. 그래서 장식을 줄이고 수치, 표, 그림이 눈에 먼저 들어오게 한다.

시각 주제는 버스 노선도 하나다. 파랑 선이 노선이고 노랑 네모가 정류장이다. 이 두 색 말고는 색을 쓰지 않는다. 구분은 색 대신 굵기, 크기, 여백으로 한다.

Stanford OpenWAM 의 골격은 유지한다. 순서는 상단 바, 가운데 정렬 히어로, 테두리 그림 상자, 요약, 왼쪽 목차와 본문이다.

## Colors

색은 `:root` 변수로만 쓴다. 규칙, CSS 안에 16진수 값이 직접 나오면 안 된다.

- 바탕: `paper` 는 페이지 바탕, `surface` 는 그림 상자, 표, 카드의 바탕이다.
- 글: `ink` 는 제목과 강조, `body` 는 본문, `muted` 는 캡션과 보조 글이다. `muted` 는 `paper` 위에서 대비 5:1 이상이다.
- 선: `rule` 은 상자 테두리와 구획선, `rule-soft` 는 표 행 사이, `rule-strong` 은 호버 테두리와 스크롤바다.
- `tone`, `tone-deep` 은 보조 버튼과 인라인 코드 바탕이다.
- `accent` 는 노선이다. 주 버튼, 링크 호버, 경로선, 목차 점과 줄에 쓴다. `accent-deep` 은 호버, `accent-faint` 는 목차 줄이다.
- `stop` 은 정류장이다. 정사각형 표식과 글 선택 배경에만 쓴다. 글자색이나 면 배경으로 쓰지 않는다.

## Typography

글꼴은 Pretendard Variable 하나다. 한국어 글은 `word-break: keep-all` 이다. 제목은 `text-wrap: balance`, 본문은 `pretty` 로 한다.

본문, 목록, 캡션은 모두 왼쪽 정렬이다. 양쪽 정렬(`text-align: justify`)은 쓰지 않는다. `keep-all` 은 줄을 공백에서만 끊기 때문에, 양쪽 정렬을 함께 쓰면 줄 끝을 맞추려고 낱말 사이가 크게 벌어진다(390px 에서 특히 심하다). 줄 끝은 `pretty` 가 고르게 한다. 숫자와 단위, 숫자 뒤 조사가 줄 머리와 줄 끝으로 갈라지면 안 되는 곳(히어로 리드)은 `.nobr`(`white-space: nowrap`)로 묶는다.

크기는 위 표의 값만 쓴다. 최소 크기는 .875rem 이다. 굵기는 400, 500, 600, 700, 800 다섯 가지다. 큰 제목일수록 자간을 좁히고(-.04em, -.03em, -.02em), 본문은 자간을 건드리지 않는다. 본문 줄 높이는 1.75, 한 줄 길이는 `measure`(720px) 를 넘기지 않는다. 숫자가 들어가는 표와 통계는 `tabular-nums` 를 쓴다.

## Layout

- 간격은 4px 기준 `s-1`부터 `s-9`까지만 쓴다. 섹션 사이는 `s-9`(모바일 `s-8`)이고 선을 긋지 않는다. 구분은 여백이 한다. 선은 요약과 본문 사이 한 줄, 푸터 위 한 줄뿐이다. 두 선은 화면 끝이나 안쪽 여백까지 칠하지 않고 내용 폭(좌우 `gutter` 안쪽)에 맞춘다. 그래서 테두리가 아니라 `left/right: var(--gutter)` 인 1px `rule` 가상 요소로 긋는다.
- 본문은 2열 격자다. 왼쪽 `toc` 폭 200px, 열 사이 `s-8`, 오른쪽 본문이다. 요약도 같은 열 선에 맞춘다.
- 본문 글 블록은 `measure` 안에 두고, 그림, 표, 카드 묶음은 본문 열 전체 폭을 쓴다. 가장자리가 어긋나면 안 된다.
- 히어로 그림 상자는 `figure`(1000px) 폭이다.
- 모서리 둥글기는 상자와 카드 `lg`, 상자 안 이미지 `sm`, 인라인 코드 `xs`, 버튼 `pill` 이다. 이 밖의 값은 쓰지 않는다.
- 그림자와 그라데이션은 쓰지 않는다.

## Components

**상단 바.** 높이 `nav`, 불투명 `paper` 바탕, 아래 1px `rule`. 왼쪽에 정류장 표식과 영문 이름(800), 오른쪽에 섹션 링크 여섯 개(도로, 교통, 학습, 결과, 영상, 데이터. `ui`, `muted`). 현재 섹션 링크는 `ink` 색과 2px 밑줄이다. 860px 이하에서 링크를 숨긴다.

**히어로.** 가운데 정렬. 위 여백 `s-7`. `display` 제목, `title` 부제(최대 `measure`), 경로선, 저자 줄, 소속 줄, `lede`, 버튼 세 개 순서다. 경로선은 4px `accent` 선 사이에 `stop` 표식 4개(버스가 서는 정류장 수)를 둔 노선 그림이다. `lede` 는 `text-wrap: balance` 이고 숫자 묶음은 `.nobr` 로 감싼다. 버튼은 주 버튼 하나(GitHub 코드)와 보조 버튼 둘(Hugging Face 데이터, 주행 영상)이다. 골목 보고서처럼 본문에 이미 있는 자료 링크는 히어로가 아니라 푸터에 둔다. 첫 화면 등장 애니메이션은 아래에서 12px 올라오는 한 가지뿐이고, 한 번만 돈다.

**버튼.** 높이 `tap`, 알약 모양, 글 `body`/700. 주 버튼은 `accent` 바탕에 `on-accent` 글, 호버에서 `accent-deep`. 보조 버튼(`.tonal`)은 `tone` 바탕에 `ink` 글, 호버에서 `tone-deep`. 한 화면에 주 버튼은 하나만 둔다. 860px 이하에서 히어로 버튼은 2열 격자(간격 `s-2`, 좌우 안쪽 여백 `s-2`)로 칸을 꽉 채우고, 주 버튼이 첫 줄 전체를 쓴다. 버튼 두 줄이 첫 화면 영상을 밀어내지 않게 하려는 것이다.

**그림 상자.** `surface` 바탕, 1px `rule`, 반지름 `lg`, 안쪽 여백 `s-3`. 이미지는 `sm` 모서리. 아래 캡션은 첫 줄이 `body`/700 `ink` 인 제목이고, 둘째 줄부터 `small` `muted` 다. 상자 전체가 원본 이미지 링크이고, 호버(마우스 환경)에서 2px 뜨고 테두리가 `rule-strong` 이 된다.

**섹션 제목(h2).** `h2` 크기와 굵기. 앞에 `mark` 크기의 정류장 표식(노랑 네모, 2px `ink` 테두리, 모서리 2px)을 제목 왼쪽 바깥 `s-4` 에 둔다. 1100px 이하에서는 표식을 숨긴다. 요약 제목은 `xl` 크기이고 표식이 없다.

**요약.** 왼쪽 열에 제목, 오른쪽 열에 `lead` 크기 본문. 본문 열 선과 맞춘다.

**목차.** 왼쪽에서 스크롤을 따라온다(sticky). 세로 3px `accent-faint` 선 위에 항목마다 원 점(`mark`, 2px `accent` 테두리)이 있다. 현재 섹션은 점이 `stop` 네모(테두리 `ink`)로 바뀌고 글이 700 `ink` 가 된다. JS 없이도 링크로 동작한다. 860px 이하에서는 본문 위 2열 격자가 되고 항목 높이는 `tap` 이다.

**표.** `table-scroll` 상자(`surface`, 1px `rule`, `lg`)가 감싸고 좁은 화면에서 가로로 민다. 상자에는 `role="region" tabindex="0" aria-label="… 표"` 를 달아 키보드로도 밀 수 있게 한다. 머리글은 `small`/500 `muted`, 아래 2px `ink` 선. 행 사이는 1px `rule-soft`, 마지막 행은 선 없음. 첫 열은 700이고 넓은 화면에서는 줄바꿈하지 않는다. 숫자 열(`.num`)은 오른쪽 정렬, 700. 머리글 칸은 `scope="col"` 을 쓴다. 860px 이하에서는 글자를 `small` 로 줄이고 칸 여백을 `s-3`/`s-2` 로 좁힌다. 첫 열은 줄바꿈을 허용하고(최소 6em) 왼쪽에 고정(`position: sticky`, `surface` 바탕, 오른쪽 1px `rule-soft`)하며, 글 칸은 최소 8em 이다. 숫자 칸이 화면 밖으로 밀려 숨거나 글 칸이 한두 글자 폭으로 쪼개지지 않게 하려는 것이다.

**통계 줄.** 4칸 격자. 위 1px `ink`, 아래 1px `rule`, 칸 사이 1px `rule` 세로선. 수치는 `stat` 크기, 라벨은 `ui` `muted`. 카드 상자를 씌우지 않는다. 860px 이하에서 2×2 가 된다. 라벨이 짧은 사실(길이, 곳 수, 건수)일 때만 쓴다. 라벨이 문장이라 두 줄 넘게 접히면 같은 숫자를 문장 목록(`ul`)으로 쓴다(차량 interaction 절).

**눈치싸움 극장.** 페이지에 하나뿐인 영상 묶음이다(`#nunchi`, 차량 interaction 절 끝). 위에서부터 제목(`h3`, `xl`/800, 860px 이하 `xl-m`), 소개 문단, 본문 열 전체 폭의 `.video-box` 영상 하나, 바로 아래 고른 장면의 캡션, 그 아래 장면 고르기 노선 순서다.
- 캡션은 `장면 N · 교통`(`small` `muted`, 번호는 700 `ink`), 장면 제목(`lead`/700 `ink`), 설명(`body` 크기, `body` 색, 최대 `measure`), 버스 색 줄 순서다. 영상을 보며 읽는 글이라 설명만 캡션 기본값(`small` `muted`)보다 크게 쓴다.
- 장면 고르기는 4px `accent` 노선 위 정류장 다섯 개다. 정류장마다 장면 이름이 보이는 글자로 붙는다(`ui`/500 `muted`). 고른 정류장은 목차의 현재 표식과 같이 `stop` 네모(2px `ink` 테두리)와 700 `ink` 글이 된다. 색만이 아니라 모양과 굵기로도 구분한다. 860px 이하에서는 노선이 세로가 되고 정류장 한 줄이 높이 `tap` 이다.
- 접근성은 WAI-ARIA 탭 패턴이다. 노선은 `role="tablist"`(`aria-label`, 폭에 따라 `aria-orientation`), 정류장은 `<button role="tab" aria-selected aria-controls>`, 장면마다 `role="tabpanel" aria-labelledby`. 고른 탭만 `tabindex="0"` 이라 Tab 은 노선에 한 번만 들른다. 좌우 화살표(세로일 때 위아래)로 옮기고 끝에서 처음으로 돈다. Home, End 는 처음과 끝. 포커스는 전역 2px `accent` 외곽선이 보인다. 고르면 바로 그 장면으로 바뀐다(자동 활성화).
- 다섯 장면은 한 격자 칸에 겹쳐 두고 고르지 않은 장면은 `visibility: hidden` 으로 숨긴다. 칸 높이가 가장 긴 캡션에 맞춰지므로 장면을 바꿔도 노선과 아래 글이 움직이지 않는다. 영상에는 원본 `width`/`height`(960×540)를 쓴다.
- JS 가 없으면 노선(`hidden`)이 숨고 다섯 장면이 컨트롤이 있는 영상과 캡션으로 모두 펼쳐진다.
- 저절로 다음 장면으로 넘기지 않는다.

**버스 색 줄(`.video-key`).** 모든 영상 캡션의 끝에 한 줄로 버스 색과 화면 종류를 적는다. 버스 색은 700 `ink`, 나머지는 캡션 색이다. 눈치싸움 장면은 SUMO 화면 녹화라 "빨간 차가 버스 · SUMO 화면 녹화", 나머지 영상은 기록으로 다시 그린 화면이라 "노란 차가 버스 · 기록으로 다시 그린 화면"이다. 색 견본(색 네모)은 쓰지 않는다. 노랑은 정류장 표식에만 쓰고 빨강은 팔레트에 없기 때문이다.

**지름길 카드.** `surface` 바탕, 1px `rule`, `lg`. 위에 높이 140px(모바일 110px) 지도 이미지, 1px `rule` 선, 아래 `s-3`/`s-4` 여백의 글. 이름은 `h3`, 수치 줄은 `lead`/700 `ink`, 보조 줄은 `body`. 격자는 최소 200px 칸 자동 채움, 간격 `s-4`, 모바일 2열.

**당김 글.** `xl` 크기(860px 이하 `xl-m`), 700, `ink`. 화면에 들어오면 `muted` 에서 `ink` 로 한 번 바뀐다(JS 없으면 처음부터 `ink`).

**인라인 코드.** 본문 글꼴 그대로, .9em, 600, `tone` 바탕, `xs` 모서리. 고정폭 글꼴을 쓰지 않는다.

**푸터.** 위 1px `rule`(내용 폭), `small` `muted`, 세 항목(팀 프로젝트와 연도, 자료 링크, 지도 출처)을 양끝 정렬하고 좁으면 줄바꿈한다. 가운데 자료 링크는 코드, 데이터, 골목 보고서 순이고 링크마다 높이 `tap` 이다. 발표자료는 파일이 저장소에 들어온 뒤에만 넣는다.

## Do's and Don'ts

### Do
- 새 값이 필요하면 먼저 이 파일과 `:root` 에 토큰을 추가하고 나서 쓴다.
- 강조는 굵기, 크기, `ink` 색으로 한다.
- 노랑은 정류장 표식으로만, 파랑은 노선과 누를 수 있는 것에만 쓴다.
- 이미지에는 의미 있는 `alt` 를 쓴다. 키보드 포커스는 2px `accent` 외곽선을 항상 보인다.
- 움직임은 `prefers-reduced-motion: reduce` 에서 끈다. 숫자와 사실은 디자인 때문에 바꾸지 않는다.
- 390px 폭에서 가로 스크롤이 없어야 한다.

### Don't
- CSS 에 16진수 색, 4 배수가 아닌 px 간격, 정해진 크기 밖의 글자 크기를 직접 쓰지 않는다.
- 그림자, 그라데이션, 장식 아이콘, 이모지를 쓰지 않는다.
- 섹션 사이에 구분선을 긋지 않는다. 여백으로 가른다.
- 노랑을 글자색이나 큰 면으로 쓰지 않는다.
- 통계나 표를 카드 상자로 다시 감싸지 않는다.
- 이 페이지에 다른 브랜드의 색, 글꼴, 로고를 가져오지 않는다.

## Responsive Behavior

- 1100px 이하. 제목 표식을 숨긴다.
- 860px 이하. 본문과 요약이 1열, 목차가 본문 위 2열 격자, 상단 링크 숨김, 좌우 여백 `s-4`, 통계 2×2, 지름길 카드 2열, 히어로 버튼 2열 격자(주 버튼 첫 줄 전체), 당김 글과 극장 제목 `xl-m`, 표 글자 `small`·첫 열 고정, 영상 격자 1열, 극장 노선 세로.
- 터치 대상은 높이 `tap`(44px) 이상. 목차 항목, 버튼, 극장 정류장, 푸터 링크가 해당한다.
- 이미지는 `max-width: 100%` 이고 표는 상자 안에서 가로 스크롤한다.

## Performance

- 모든 `<img>` 에 원본 `width`/`height` 를 쓴다. CSS 의 `height: auto` 가 비율을 지켜 레이아웃이 밀리지 않는다.
- 히어로 그림(그림 1)만 `fetchpriority="high"` 이고 나머지 이미지는 `loading="lazy"` 다. 모든 이미지에 `decoding="async"` 를 쓴다.
- 글꼴 CSS 가 있는 CDN 에 `preconnect` 를 건다. 링크와 버튼에는 `touch-action: manipulation` 을 쓴다.
- PNG 원본은 압축하거나 바꾸지 않는다(총 약 3MB, 지름길 지도 제외).

## Iteration Guide

1. 색, 간격, 글자 크기, 굵기가 필요하면 위 토큰 중에서 고른다.
2. 컴포넌트는 위 목록에서 가장 가까운 것을 따라 만든다.
3. 바꾼 뒤 `scripts/screenshot_web_page.sh` 로 1440px 과 390px 을 찍어 페이지 전체를 위에서 아래까지 본다. 단, headless Chrome 은 창 폭을 약 500px 아래로 줄이지 못해 `--window-size=390,…` 로 찍으면 넓은 화면의 왼쪽만 잘려 나온다. 390px 은 폭 390px `iframe` 에 페이지를 담아 찍거나 DevTools 의 기기 크기 흉내(mobile 390×844)로 찍는다.

## Known Gaps

- 지름길 카드 이미지 높이(140px, 110px), 경로선 최대 폭(360px), 지연 애니메이션 시간(60ms 단위)은 토큰 없이 컴포넌트 안에 있다.
- 그림 PNG 가 크다(그림 1 약 1.1MB, 그림 2 약 1.4MB). WebP 변환과 `srcset` 은 파일 교체가 필요해 하지 않았다.
- 다크 모드는 없다(`color-scheme: light`).

## Video

영상은 `.video-box`(그림 상자와 같은 테두리와 여백) 안에 `<video controls muted loop playsinline>`로 넣는다. `autoplay` 속성은 어디에도 쓰지 않는다. 히어로 영상은 `preload="metadata"`, 나머지는 `preload="none"`에 포스터 그림을 둔다. 영상마다 굵은 제목, 한두 줄 설명, 버스 색 줄(`.video-key`)을 캡션으로 단다. 영상 파일은 `assets/videos/`에 H.264 mp4로 둔다. 잘한 장면만 싣지 않고 실패 장면을 유형별로 함께 싣는다.

**저절로 재생(예외).** 저절로 재생하는 영상은 히어로 영상과 눈치싸움 극장의 고른 장면 둘뿐이고, `data-play-in-view` 속성으로 표시한다. 페이지 스크립트가 다음 규칙으로 튼다.
- 영상이 상단 바 아래 화면에 절반 넘게 보일 때만 소리 없이 반복 재생하고, 절반 아래로 벗어나면 멈춘다. 그래서 한 화면에서 도는 영상은 많아야 둘이고, 보이지 않는 영상은 데이터와 CPU 를 쓰지 않는다.
- `prefers-reduced-motion: reduce` 와 데이터 절약(`navigator.connection.saveData`)에서는 저절로 틀지 않는다. 재생 중에 움직임 줄이기를 켜면 멈춘다. JS 가 없으면 포스터와 컨트롤만 보인다.
- 사용자가 컨트롤로 멈춘 영상은 화면을 나갔다 들어와도 다시 저절로 틀지 않는다. 사용자가 다시 틀면 이 기록은 지워진다. 스크립트가 멈춘 것은 사용자가 멈춘 것으로 치지 않는다.
- 극장에서 장면을 바꾸면 이전 장면은 멈춘다. 이전 장면이 돌고 있었으면 새 장면을 이어서 틀고, 사용자가 멈춰 두었으면 새 장면도 멈춘 채로 둔다.
- 컨트롤(`controls`)은 언제나 남긴다. 움직이는 내용을 멈출 수단이다.

**영상 격자.** 실패 장면 세 개는 `.video-grid`(3열, 860px 이하 1열)로 한 줄에 놓는다. 실패가 아닌 보조 장면(옆에서 나란히 가는 차)은 격자 아래에 `.video-aside` 로 따로 둔다. 영상은 격자 한 칸과 같은 폭(1열)이고 캡션이 옆 두 칸을 쓴다. 실패 장면보다 크게 보이지 않게 하려는 것이다. 860px 이하에서는 영상 아래에 캡션이 온다.
