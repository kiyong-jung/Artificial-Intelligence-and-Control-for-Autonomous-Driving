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
  xl: { size: 1.5rem, weight: 700, lineHeight: 1.5, use: "pull quote, 요약 제목" }
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

크기는 위 표의 값만 쓴다. 최소 크기는 .875rem 이다. 굵기는 400, 500, 600, 700, 800 다섯 가지다. 큰 제목일수록 자간을 좁히고(-.04em, -.03em, -.02em), 본문은 자간을 건드리지 않는다. 본문 줄 높이는 1.75, 한 줄 길이는 `measure`(720px) 를 넘기지 않는다. 숫자가 들어가는 표와 통계는 `tabular-nums` 를 쓴다.

## Layout

- 간격은 4px 기준 `s-1`부터 `s-9`까지만 쓴다. 섹션 사이는 `s-9`(모바일 `s-8`)이고 선을 긋지 않는다. 구분은 여백이 한다. 선은 요약과 본문 사이 한 줄, 푸터 위 한 줄뿐이다.
- 본문은 2열 격자다. 왼쪽 `toc` 폭 200px, 열 사이 `s-8`, 오른쪽 본문이다. 요약도 같은 열 선에 맞춘다.
- 본문 글 블록은 `measure` 안에 두고, 그림, 표, 카드 묶음은 본문 열 전체 폭을 쓴다. 가장자리가 어긋나면 안 된다.
- 히어로 그림 상자는 `figure`(1000px) 폭이다.
- 모서리 둥글기는 상자와 카드 `lg`, 상자 안 이미지 `sm`, 인라인 코드 `xs`, 버튼 `pill` 이다. 이 밖의 값은 쓰지 않는다.
- 그림자와 그라데이션은 쓰지 않는다.

## Components

**상단 바.** 높이 `nav`, 불투명 `paper` 바탕, 아래 1px `rule`. 왼쪽에 정류장 표식과 영문 이름(800), 오른쪽에 섹션 링크 다섯 개(`ui`, `muted`). 현재 섹션 링크는 `ink` 색과 2px 밑줄이다. 860px 이하에서 링크를 숨긴다.

**히어로.** 가운데 정렬. `display` 제목, `title` 부제(최대 `measure`), 경로선, 저자 줄, 소속 줄, `lede`, 버튼 두 개 순서다. 경로선은 4px `accent` 선 사이에 `stop` 표식 6개를 둔 노선 그림이다. 첫 화면 등장 애니메이션은 아래에서 12px 올라오는 한 가지뿐이고, 한 번만 돈다.

**버튼.** 높이 `tap`, 알약 모양, 글 `body`/700. 주 버튼은 `accent` 바탕에 `on-accent` 글, 호버에서 `accent-deep`. 보조 버튼(`.tonal`)은 `tone` 바탕에 `ink` 글, 호버에서 `tone-deep`. 한 화면에 주 버튼은 하나만 둔다. 모바일에서는 가로 꽉 채운다.

**그림 상자.** `surface` 바탕, 1px `rule`, 반지름 `lg`, 안쪽 여백 `s-3`. 이미지는 `sm` 모서리. 아래 캡션은 첫 줄이 `body`/700 `ink` 인 제목이고, 둘째 줄부터 `small` `muted` 다. 상자 전체가 원본 이미지 링크이고, 호버(마우스 환경)에서 2px 뜨고 테두리가 `rule-strong` 이 된다.

**섹션 제목(h2).** `h2` 크기와 굵기. 앞에 `mark` 크기의 정류장 표식(노랑 네모, 2px `ink` 테두리, 모서리 2px)을 제목 왼쪽 바깥 `s-4` 에 둔다. 1100px 이하에서는 표식을 숨긴다. 요약 제목은 `xl` 크기이고 표식이 없다.

**요약.** 왼쪽 열에 제목, 오른쪽 열에 `lead` 크기 본문. 본문 열 선과 맞춘다.

**목차.** 왼쪽에서 스크롤을 따라온다(sticky). 세로 3px `accent-faint` 선 위에 항목마다 원 점(`mark`, 2px `accent` 테두리)이 있다. 현재 섹션은 점이 `stop` 네모(테두리 `ink`)로 바뀌고 글이 700 `ink` 가 된다. JS 없이도 링크로 동작한다. 860px 이하에서는 본문 위 2열 격자가 되고 항목 높이는 `tap` 이다.

**표.** `table-scroll` 상자(`surface`, 1px `rule`, `lg`)가 감싸고 좁은 화면에서 가로로 민다. 머리글은 `small`/500 `muted`, 아래 2px `ink` 선. 행 사이는 1px `rule-soft`, 마지막 행은 선 없음. 첫 열은 700이고 줄바꿈하지 않는다. 숫자 열(`.num`)은 오른쪽 정렬, 700. 머리글 칸은 `scope="col"` 을 쓴다.

**통계 줄.** 4칸 격자. 위 1px `ink`, 아래 1px `rule`, 칸 사이 1px `rule` 세로선. 수치는 `stat` 크기, 라벨은 `ui` `muted`. 카드 상자를 씌우지 않는다. 860px 이하에서 2×2 가 된다.

**지름길 카드.** `surface` 바탕, 1px `rule`, `lg`. 위에 높이 140px(모바일 110px) 지도 이미지, 1px `rule` 선, 아래 `s-3`/`s-4` 여백의 글. 이름은 `h3`, 수치 줄은 `lead`/700 `ink`, 보조 줄은 `body`. 격자는 최소 200px 칸 자동 채움, 간격 `s-4`, 모바일 2열.

**당김 글.** `xl` 크기, 700, `ink`. 화면에 들어오면 `muted` 에서 `ink` 로 한 번 바뀐다(JS 없으면 처음부터 `ink`).

**인라인 코드.** 본문 글꼴 그대로, .9em, 600, `tone` 바탕, `xs` 모서리. 고정폭 글꼴을 쓰지 않는다.

**푸터.** 위 1px `rule`, `small` `muted`, 세 항목을 양끝 정렬하고 좁으면 줄바꿈한다.

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
- 860px 이하. 본문과 요약이 1열, 목차가 본문 위 2열 격자, 상단 링크 숨김, 좌우 여백 `s-4`, 통계 2×2, 지름길 카드 2열, 주 버튼 꽉 채움, 당김 글 1.25rem.
- 터치 대상은 높이 `tap`(44px) 이상. 목차 항목과 버튼이 해당한다.
- 이미지는 `max-width: 100%` 이고 표는 상자 안에서 가로 스크롤한다.

## Performance

- 모든 `<img>` 에 원본 `width`/`height` 를 쓴다. CSS 의 `height: auto` 가 비율을 지켜 레이아웃이 밀리지 않는다.
- 히어로 그림(그림 1)만 `fetchpriority="high"` 이고 나머지 이미지는 `loading="lazy"` 다. 모든 이미지에 `decoding="async"` 를 쓴다.
- 글꼴 CSS 가 있는 CDN 에 `preconnect` 를 건다. 링크와 버튼에는 `touch-action: manipulation` 을 쓴다.
- PNG 원본은 압축하거나 바꾸지 않는다(총 약 3MB, 지름길 지도 제외).

## Iteration Guide

1. 색, 간격, 글자 크기, 굵기가 필요하면 위 토큰 중에서 고른다.
2. 컴포넌트는 위 목록에서 가장 가까운 것을 따라 만든다.
3. 바꾼 뒤 `scripts/screenshot_web_page.sh` 로 1440px 과 390px 을 찍어 페이지 전체를 위에서 아래까지 본다.

## Known Gaps

- `.pull` 의 모바일 크기 1.25rem 은 토큰이 아닌 예외 값이다.
- 지름길 카드 이미지 높이(140px, 110px), 경로선 최대 폭(360px), 지연 애니메이션 시간(60ms 단위)은 토큰 없이 컴포넌트 안에 있다.
- 그림 PNG 가 크다(그림 1 약 1.1MB, 그림 2 약 1.4MB). WebP 변환과 `srcset` 은 파일 교체가 필요해 하지 않았다.
- 다크 모드는 없다(`color-scheme: light`).
