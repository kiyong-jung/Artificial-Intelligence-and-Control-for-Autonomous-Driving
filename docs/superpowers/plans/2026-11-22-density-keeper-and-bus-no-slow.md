# 구간 밀도 유지(DensityKeeper)와 시험용 버스 감속 제거 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 양화로·독막로·잔다리로·와우산로에 목표 대수의 차량이 있도록 코드가 직접 유지하고(SUMO 위임 없음), 시험용 버스 주변 50 m 안에서는 순간이동 없이 밀도를 맞추며, 시험용 버스가 S2·S3 상호작용을 위해 속도를 줄이는 코드를 없앤다.

**Architecture:** (1) `DensityKeeper`가 구간(도로 이름) 단위로 차량 수를 세어 부족하면 넣고 남으면 지운다. 시험용 버스 반경 + 여유 밖에서만 한다. (2) `NearBusDensity`가 버스 주변 50 m 안에 차가 모자라면 뒤차는 빨리 달려 들어오게, 앞차는 느리게 달리게 한 뒤 반경에 들어오면 SUMO 기본 알고리즘에 돌려준다. (3) `TestBusPilot`에서 후행차를 기다리는 감속과 비양보 차 보내기 감속을 지운다(정류장·좌회전 차로 도착을 위한 강제 정차는 남긴다). (4) `InteriorSpawner`의 투입 금지 거리를 200 m에서 50 m(+여유)로 줄인다.

**Tech Stack:** Python 3.10(`D:\conda_envs\faster\python.exe`), SUMO 1.27.1 TraCI, sumolib, pytest. 기존 `traffic/` 패키지의 가짜 TraCI 테스트 방식을 따른다.

**Spec:** 별도 명세 문서는 없다. 이 계획의 "요구사항" 절이 사용자 요청(2026-11-22)을 그대로 옮긴 명세다. 관련 기존 문서: `docs/superpowers/specs/2026-11-16-test-bus-route-and-scenarios.md`, `traffic/README.md`.

## 요구사항 (사용자 요청 원문 요약)

1. 양화로·독막로·잔다리로·와우산로에 일정 대수의 차량이 있게 유지한다. 목표 유량을 SUMO(calibrator 등)에 맡기지 않고 TraCI 코드로 직접 조작한다.
2. 신호등 쪽에 차량이 몰려 정체가 생기면 안 된다.
3. 시험용 버스 주변 `PRESENCE_RADIUS_M`(50 m) 안에서는 차량 순간이동(생성·제거·정체 순간이동)이 절대 없어야 한다. 50 m 밖에서는 목표 대수를 맞추려고 순간이동해도 된다.
4. 밀도 분산 투입 금지 거리를 200 m에서 50 m 범위로 수정한다.
5. 그러면 버스 주변이 비므로, 버스 주변에 차가 없으면 버스 뒤차는 속도를 높여 50 m 안으로 빨리 들어오고 그 뒤부터는 기본 차량 알고리즘에 맡긴다. 앞차는 속도를 줄여 버스 주변 밀도를 맞춘다.
6. 시험용 버스는 정류장 진입·좌회전 차로 진입을 위해서는 줄이거나 설 수 있지만, 주변 차량과의 상호작용(S2·S3 상대 기다리기)을 위해 줄이면 안 된다. 그 감속 코드를 제거한다.
7. 작업은 새 브랜치 `scenario-개선`(git 브랜치 이름에는 공백을 쓸 수 없어 `scenario 개선`을 `scenario-개선`으로 둔다)에서 한다. 커밋 메시지에 Co-Authored-By 등 추가 줄을 넣지 않는다. 커밋은 큰 작업 뒤 한 번만 한다.

## Global Constraints

- 반경 상수는 `traffic/presence.py`의 `PRESENCE_RADIUS_M = 50.0`만 쓴다. 새 숫자 50을 다른 파일에 적지 않는다.
- 새 상수 `SPAWN_MARGIN_M = 5.0`을 `traffic/presence.py`에 둔다. 한 스텝(0.1 s)에 버스와 새 차가 합쳐 3 m 가까이 움직여 검사 직후 반경 안으로 들어오는 것을 막는 여유다. 투입·제거 검사 거리는 `PRESENCE_RADIUS_M + SPAWN_MARGIN_M`(55 m, 직선거리)이다.
- 새로 넣는 차의 vType은 `carDist`, 경로는 지도 경계(`presence.nearest_boundary_exit`)나 도로망 나가는 곳에서 끝난다(`VehicleArrivalGuard`와 같은 전제).
- 속도를 건드리는 새 코드는 `traffic/common.py`의 `Ownership`으로 차량을 `"density"` 이름으로 claim한 뒤에만 `setSpeed`를 쓴다. claim이 실패한 차(다른 추가 행동이 다루는 차)는 건드리지 않는다. 끝낼 때는 `setSpeed(vid, -1)`과 `release`를 한다.
- 신호 교차로 안(도로 id가 `:`로 시작)과 정지선 앞 금지 구간(`traffic/signal_areas.py`의 `forbidden_intervals`가 주는 구간)에서는 속도 개입을 하지 않는다.
- 파일 편집은 Write·Edit 도구로 하고 허락을 구하지 않는다. 파이썬은 전체 경로로 직접 호출하고 `cd`를 쓰지 않는다. 검증 실행은 필요한 만큼만 하고 돌리지 않은 것은 보고에 명시한다.
- 커밋은 Task 8에서 한 번만 한다. 메시지는 한국어 한 줄 제목 + 본문, 끝에 attribution 줄을 붙이지 않는다.

## Review Focus

1. 버스 반경 55 m 안에서 투입·제거·정체 순간이동이 한 번이라도 나면 안 된다 → Task 2·7에서 `PresenceMonitor`로 0을 확인한다.
2. 목표 대수를 채우려고 신호 정지선 앞에 차를 넣어 대기 줄이 길어지면 안 된다 → Task 2의 정체 가드 테스트.
3. 버스가 없을 때(출발 전 예열)도 구간은 채워져야 한다 → Task 2의 `bus_id` 부재 테스트.
4. 다른 추가 행동(A1 양보, S4, 주입기)이 잡고 있는 차를 밀도 유지가 건드리면 안 된다 → Task 2·3의 보호·claim 테스트.
5. 버스가 정류장에 서 있거나 신호 구간에 있을 때 뒤차 가속·앞차 감속을 걸면 안 된다 → Task 3 테스트.

---

## File Structure

| 파일 | 일 | 변경 |
|---|---|---|
| `traffic/presence.py` | `SPAWN_MARGIN_M` 상수 | 수정 |
| `traffic/interior_spawner.py` | 투입 금지 거리 기본값 50 m 범위로 | 수정 |
| `traffic/density_keeper.py` | 구간별 목표 대수 유지(투입·제거) | 새 파일 |
| `traffic/near_bus_density.py` | 버스 주변 50 m 밀도 맞추기(뒤차 가속·앞차 감속) | 새 파일 |
| `traffic/test_bus.py` | S2·S3 상호작용 감속 제거 | 수정 |
| `traffic/scenario_injector.py` | `follower_ready` 제거 | 수정 |
| `traffic/hapjeong.py` | 두 새 부품 연결, `follower_ready` 연결 제거 | 수정 |
| `scripts/measure_section_density.py` | 구간별 밀도·대기·순간이동 측정 | 새 파일 |
| `scripts/run_hapjeong_scenario.py`, `scripts/measure_s1_s6.py` | `late`·`slowed` 출력 제거 | 수정 |
| `tests/test_density_keeper.py`, `tests/test_near_bus_density.py` | 새 부품 테스트 | 새 파일 |
| `tests/test_test_bus.py`, `tests/test_density_and_right_turn.py` | 감속 테스트 교체, 투입 금지 거리 테스트 | 수정 |
| `traffic/README.md`, `traffic/VERIFICATION.md` | 변경 내용 반영 | 수정 |

---

### Task 0: 브랜치 만들기

**Files:** 없음

- [ ] **Step 1:** 현재 브랜치가 `main`이고 pull이 끝났는지 확인한다. Run: `git status -sb` → 첫 줄이 `## main...origin/main`.
- [ ] **Step 2:** `git checkout -b scenario-개선`. untracked 파일(`docs/`, `.ua/` 등)은 그대로 따라온다.

---

### Task 1: 투입 금지 거리를 50 m 범위로 (`InteriorSpawner`)

**Files:**
- Modify: `traffic/presence.py` (상수 추가)
- Modify: `traffic/interior_spawner.py:18` (`min_dist` 기본값), docstring 3~6행
- Modify: `traffic/hapjeong.py:115,118,315` (`interior_min_dist` 기본값)
- Test: `tests/test_density_and_right_turn.py::test_spawner_never_places_cars_near_the_test_bus`

**Interfaces:**
- Produces: `presence.SPAWN_MARGIN_M: float = 5.0`; `InteriorSpawner(..., min_dist: float = PRESENCE_RADIUS_M + SPAWN_MARGIN_M, ...)`; `HapjeongTraffic(..., interior_min_dist: float = PRESENCE_RADIUS_M + SPAWN_MARGIN_M)`.

- [ ] **Step 1: 테스트를 고친다.** 기존 테스트의 `min_dist=200.0`, `>= 200.0 - 1e-6`을 `min_dist` 기본값 사용과 `>= PRESENCE_RADIUS_M + SPAWN_MARGIN_M - 1e-6`로 바꾼다. 가짜 버스 위치 (250, 150) 근처가 50~55 m 안에도 노선 지점이 있어 `n_skipped_near > 0`이 유지되는지 본다(안 되면 버스 좌표를 노선 위 한 점으로 옮긴다). 투입 수는 200 m일 때보다 많아야 하므로 `sp.n_spawned > 100`은 유지한다.
- [ ] **Step 2:** Run `D:\conda_envs\faster\python.exe -m pytest tests/test_density_and_right_turn.py::test_spawner_never_places_cars_near_the_test_bus -q` → FAIL(기본값이 아직 200 m라 `SPAWN_MARGIN_M` import 실패).
- [ ] **Step 3: 구현.** `presence.py`에 `SPAWN_MARGIN_M = 5.0`과 한 줄 주석을 둔다. `interior_spawner.py`와 `hapjeong.py`의 기본값을 `PRESENCE_RADIUS_M + SPAWN_MARGIN_M`로 바꾸고, 모듈 docstring의 "200 m"를 "버스 반경 50 m(+여유 5 m)"로 고친다. `hapjeong.py` 주석의 `interior_min_dist m 안` 설명도 맞춘다.
- [ ] **Step 4:** 같은 테스트와 `tests/test_density_and_right_turn.py` 전체 → PASS.

---

### Task 2: `DensityKeeper` (구간별 목표 대수 유지)

**Files:**
- Create: `traffic/density_keeper.py`
- Test: `tests/test_density_keeper.py`

**Interfaces:**
- Consumes: `presence.far_from_bus(tc, bus_id, xy, radius)`, `presence.PRESENCE_RADIUS_M`, `presence.SPAWN_MARGIN_M`, `signal_areas.forbidden_intervals`·`is_signalized`, `common.Ownership`, `InteriorSpawner._route`와 같은 방식의 최단 경로 계산(경로 캐시는 키퍼 안에 둔다).
- Produces:
  - `SECTION_ROADS = ("양화로", "독막로", "잔다리로", "와우산로")`
  - `DEFAULT_DENSITY_PER_100M = {"양화로": 2.0, "독막로": 2.0, "잔다리로": 2.0, "와우산로": 1.5}` (시작값이다. Task 7에서 측정으로 조정한다.)
  - `@dataclass Section: name: str; edge: str; length: float; signalized_end: bool`
  - `sections_of(net, route_edges, min_len=60.0) -> list[Section]`: 노선 도로 중 `net.getEdge(e).getName()`이 `SECTION_ROADS`에 속하고 길이가 `min_len` 이상인 것.
  - `class DensityKeeper(tc, net, bus_id, route_edges, exit_edges, density=DEFAULT_DENSITY_PER_100M, *, rng=None, period=1.0, dt=0.1, stop_zone_m=60.0, halt_max=3, surplus_ratio=1.5, protected=None)`
    - `update(t) -> None`: 매 스텝 호출. `period`초마다 한 번만 일한다(한 번에 구간당 차 한 대만 넣거나 지운다).
    - `target(section) -> int` = `max(1, round(density[name] * length / 100))`.
    - `summary() -> dict` = `{"spawned": int, "removed": int, "skipped_near_bus": int, "skipped_queue": int, "failed_insert": int}`.
    - `protected`: `Callable[[str], bool] | None`. True인 차는 지우지 않는다.

**동작 규칙 (이 값들이 명세다):**
- 구간 차량 수는 `tc.edge.getLastStepVehicleNumber(edge)`.
- 부족(수 < target): 투입 위치를 `uniform(5, length - 25)`에서 뽑되, 끝이 신호 교차로(`signalized_end`)인 구간은 `length - stop_zone_m`(60 m) 아래에서만 뽑는다. 위치의 직선거리가 버스에서 `PRESENCE_RADIUS_M + SPAWN_MARGIN_M` 미만이면 `skipped_near_bus`를 세고 건너뛴다(`far_from_bus` 사용, 시도 6회). 버스가 없으면(`bus_id` 미등록) 거리 검사는 통과한다.
- 정체 가드: `tc.edge.getLastStepHaltingNumber(edge) >= halt_max`이거나 신호 끝 구간의 다음 도로 대기 수가 같은 값 이상이면 그 구간은 투입하지 않고 `skipped_queue`를 센다.
- 투입은 `InteriorSpawner.update`와 같은 호출(`route.add` + `vehicle.add(typeID="carDist", departLane="best", departPos, departSpeed="max")`)이다. 실패하면 `failed_insert`.
- 남음(수 > `ceil(target * surplus_ratio)`): 구간 안 차 중 버스에서 `PRESENCE_RADIUS_M + SPAWN_MARGIN_M` 이상이고 `protected`가 아니고 `Ownership` 주인이 없는 차를 하나 골라 `tc.vehicle.remove(vid)`. 정지 중(속도 < 0.5)인 차를 먼저 지운다.

- [ ] **Step 1: 실패하는 테스트를 쓴다** (`tests/test_density_keeper.py`; 가짜 TraCI는 `tests/test_density_and_right_turn.py`의 `_FakeTc` 방식을 따라 `edge.getLastStepVehicleNumber/HaltingNumber`, `vehicle.getIDList/getPosition/getSpeed/getRoadID/add/remove`, `route.add`를 갖춘다. 실제 망은 `maps/hapjeong/source/network.net.xml`을 `read_net`으로 읽는다.)
  - `test_sections_cover_the_four_named_roads`: `sections_of(net, bus_loop)`의 도로 이름 집합이 `SECTION_ROADS`와 같고, 각 구간 길이 ≥ 60.
  - `test_fills_shortfall_only_where_short`: 한 구간만 대수 0, 나머지는 target일 때 `update`를 `period` 간격으로 돌리면 투입이 그 구간에만 일어난다.
  - `test_never_inserts_within_bus_radius_plus_margin`: 가짜 버스를 노선 위 한 점에 두고 10 000 스텝 돌린다. 모든 투입 위치의 직선거리 ≥ 55 m, `skipped_near_bus > 0`.
  - `test_fills_when_bus_absent`: 버스가 없어도(`getIDList`에 없음) 투입된다.
  - `test_no_insert_near_signal_stop_line`: `signalized_end` 구간 투입 위치가 모두 `length - 60` 이하.
  - `test_no_insert_when_queue_is_long`: `getLastStepHaltingNumber`가 3이면 투입 0, `skipped_queue > 0`.
  - `test_removes_surplus_far_from_bus_only`: 수가 `ceil(target*1.5)`를 넘고 일부 차가 버스 55 m 안일 때, 지운 차는 모두 55 m 밖이고 `protected`·claim된 차는 남는다.
  - `test_target_rounds_with_minimum_one`.
- [ ] **Step 2:** Run `D:\conda_envs\faster\python.exe -m pytest tests/test_density_keeper.py -q` → FAIL(모듈 없음).
- [ ] **Step 3: 구현.** 위 Interfaces·규칙대로 `traffic/density_keeper.py`를 쓴다. 한 번에 하나만 넣고 지우는 이유(급격한 순간이동 방지)를 모듈 docstring 첫 단락에 쓴다. 최단 경로는 `net.getOptimalPath(from, to, vClass="passenger")`, 노선 도로를 둘 이상 지나지 않아도 된다(키퍼의 차는 구간 안에서 시작하므로 `exit_edges` 무작위 선택 + 경로가 비면 다음 후보).
- [ ] **Step 4:** Run 같은 명령 → PASS.

---

### Task 3: `NearBusDensity` (버스 주변 50 m 밀도 맞추기)

**Files:**
- Create: `traffic/near_bus_density.py`
- Test: `tests/test_near_bus_density.py`

**Interfaces:**
- Consumes: `common.Ownership`, `presence.PRESENCE_RADIUS_M`, `signal_areas.forbidden_intervals`, `density_keeper.DEFAULT_DENSITY_PER_100M`, `density_keeper.SECTION_ROADS`.
- Produces:
  - `route_gap(tc, lengths: Callable[[str], float], bus_id: str, vid: str) -> float | None`: 버스 앞끝에서 `vid` 앞끝까지 버스 노선(`tc.vehicle.getRoute(bus_id)`) 거리. 앞이면 양수, 뒤면 음수, 노선 밖이거나 교차로 안이면 None. (`TestBusPilot._route_ahead`의 도로 비교 방식을 일반화한 것이다. Task 4에서 원본을 지운다.)
  - `class NearBusDensity(tc, net, bus_id, owner, density=DEFAULT_DENSITY_PER_100M, *, radius=PRESENCE_RADIUS_M, reach=150.0, enter_gap=None, pull_dv=4.0, slow_dv=4.0, min_speed=3.0, timeout=20.0, period=0.5)`
    - `update(t) -> None`
    - `summary() -> dict` = `{"pulled": int, "slowed": int, "released_in_range": int, "timeout": int}`
    - `enter_gap` 기본값은 `radius - 10.0`(40 m): 이 안에 들어오면 놓는다.

**동작 규칙:**
- 버스가 없거나, 버스가 교차로 안·신호 금지 구간 안이거나, 버스가 정류장에 서 있는 동안(`getStopState & 1`)에는 새로 걸지 않는다(이미 걸린 것은 아래 풀기 규칙대로 푼다).
- 필요 수 `need = max(1, round(density[버스가 있는 도로 이름] * radius / 100))`. 도로 이름이 `SECTION_ROADS`가 아니면 `need = 1`. 버스 노선 앞(gap > 0)과 뒤(gap < 0)를 따로 센다. 센 범위는 `|gap| < radius`인 모든 차로의 차(`route_gap`이 None이 아닌 차).
- 뒤가 모자라면: gap이 `[-reach, -radius]`인 차 중 버스에 가장 가까운 것을 골라 `owner.claim(vid, "density")`에 성공하면 `setSpeed(vid, min(v_bus + pull_dv, 그 차로 제한 속도 * 1.1))`. 부족한 수만큼만 동시에 건다.
- 앞이 모자라면: gap이 `[radius, reach]`인 차 중 가장 가까운 것을 골라 claim에 성공하면 `setSpeed(vid, max(v_bus - slow_dv, min_speed))`.
- 풀기: `|gap| <= enter_gap`가 되면(반경 안으로 들어왔다) `setSpeed(vid, -1)` + `owner.release`, 그 뒤는 기본 알고리즘이다. `timeout`초가 지났거나, 차가 사라졌거나, `route_gap`이 None이거나, 앞차가 버스 뒤로 가거나(gap 부호가 바뀜), 신호 금지 구간·교차로에 들어서도 푼다.
- 이 클래스는 **시험용 버스에는 아무 명령도 내리지 않는다.**

- [ ] **Step 1: 실패하는 테스트를 쓴다.** (가짜 TraCI: 버스 + 도로 위 차 몇 대, `setSpeed`·`getStopState` 기록. 노선은 `maps/hapjeong/map.json`의 `bus_loop`.)
  - `test_route_gap_sign_and_none_in_junction`: 버스 앞 차는 양수, 뒤 차는 음수, `:`로 시작하는 도로의 차는 None, 노선에 없는 도로의 차는 None. 다음 짧은 도로(R111b 27 m)로 넘어간 앞차도 거리가 맞다.
  - `test_follower_behind_range_is_sped_up_until_it_enters_radius`: 버스 뒤 100 m에 차 하나, 반경 안에 뒤차가 없을 때 `setSpeed(vid, v_bus + 4)`. 차를 gap -40 m로 옮기면 `setSpeed(vid, -1)`과 owner 해제, `summary()["released_in_range"] == 1`.
  - `test_leader_ahead_is_slowed_when_nothing_ahead_in_radius`: 앞 100 m에 차 하나일 때 `setSpeed(vid, max(v_bus - 4, 3))`.
  - `test_no_action_when_radius_already_has_enough_cars`.
  - `test_never_touches_vehicle_owned_by_another_behavior`: owner가 `"A1"`인 차는 `setSpeed` 호출이 없다.
  - `test_no_new_action_while_bus_is_stopped_or_in_signal_zone`.
  - `test_released_on_timeout_and_when_vehicle_leaves`.
  - `test_never_commands_the_bus`: 모든 시나리오에서 `setSpeed`의 대상 id에 버스가 없다.
- [ ] **Step 2:** Run `D:\conda_envs\faster\python.exe -m pytest tests/test_near_bus_density.py -q` → FAIL.
- [ ] **Step 3: 구현.** `route_gap`은 `TestBusPilot._route_ahead`(`traffic/test_bus.py`)의 비교 규칙(버스 노선의 `i-3..i+3` 도로 범위에서 같은 도로를 찾고, 뒤쪽은 음수 거리)을 그대로 일반화한다. 후보 차량 목록은 버스 컨텍스트 구독 대신 `tc.vehicle.getIDList()`에서 직선거리 `reach*1.5` 안만 고른다(`PresenceMonitor`의 구독 방식은 쓰지 않는다. 구독은 하나뿐이라 충돌한다).
- [ ] **Step 4:** Run 같은 명령 → PASS.

---

### Task 4: 시험용 버스의 S2·S3 상호작용 감속 제거

**Files:**
- Modify: `traffic/test_bus.py` (모듈 docstring 13~14행, 상수 `MIN_SLOW`·`SLOW_FRAC`·`SLOW_UNTIL_FRAC`, `BLOCKER_*`·`PASS_MARGIN` 상수, `__init__`의 `follower_ready`·`late`·`slowed`·`_slow_t0`·`_late_noted`·`_blocker*`·`_fast_t0`, `_pilot`의 signal 단계 감속 분기, `_let_blocker_pass`, `_route_ahead`, `_release_speed`의 `slowed` 기록)
- Modify: `traffic/scenario_injector.py` (`follower_ready` 메서드와 docstring 13행 문구 삭제)
- Modify: `traffic/hapjeong.py:421` (`pilot.follower_ready = ...` 줄 삭제)
- Test: `tests/test_test_bus.py`

**Interfaces:**
- Produces: `TestBusPilot(tc, net, loop, stops, stop_s=8.0)`(`follower_ready` 인자 삭제). 속성 `late`, `slowed` 삭제. 남기는 속성: `missed`, `forced`, `changes`, `lap`, `lap_times`, `why`, `phase`, `plans`.
- 버스가 `setSpeed`를 쓰는 곳은 **정차**(`_force_arrival`의 0.0, `STOP_AT` 정류장 정차)뿐이다.

- [ ] **Step 1: 테스트를 교체한다.** 아래 다섯 테스트를 지우고(`test_bus_slows_and_lets_blocker_pass_then_changes`, `test_blocker_on_next_short_road_is_still_tracked`, `test_pilot_slows_but_never_waits_for_follower`, `test_pilot_keeps_slowing_until_the_last_30_percent_of_the_window`, `test_pilot_gives_up_slowing_after_half_window`), `_pilot` 헬퍼에서 `follower_ready` 인자를 뺀다. 새 테스트:
  - `test_pilot_never_sets_speed_while_waiting_for_a_changing_partner`: S3 창 앞 절반에서 목표 차로 후행차가 가까이 없고 아주 빠른 차가 옆에 있어도 `tc.set_speed == []`이며, 방향지시등 3 s 뒤 `change_req`가 나온다.
  - `test_pilot_requests_change_even_if_fast_blocker_is_alongside`: 위와 같은 상황에서 요청이 지연되지 않는다.
  - `test_forced_arrival_still_stops_near_window_end`: 기존 `test_bus_stops_to_guarantee_arrival_in_target_lane_before_window_end`가 그대로 PASS(정지 0.0은 허용).
  - `test_pilot_has_no_follower_ready_parameter`: `TestBusPilot.__init__`의 `inspect.signature`에 `follower_ready`가 없다.
- [ ] **Step 2:** Run `D:\conda_envs\faster\python.exe -m pytest tests/test_test_bus.py -q` → 새 테스트 FAIL(감속 코드가 아직 있음).
- [ ] **Step 3: 구현.** 위 Files의 항목을 모두 지운다. signal 단계는 `SIGNAL_S`(또는 이어지는 변경 `NEXT_SIGNAL_S`)가 지나면 곧바로 `changeLane(BUS, target, REQUEST_DURATION)`을 요청하고 `request` 단계로 간다. `request` 단계는 `_let_blocker_pass` 분기만 빼고 그대로 둔다(성공·시간 초과·금지 구간 처리 유지). 모듈 docstring의 "상대를 기다리지 않는다: ... 속도를 줄여 달리며" 문장을 "상대 때문에 속도를 줄이지 않는다. 차선 변경이 늦으면 도착 보장(`_force_arrival`)이 정지선·정류장 앞에서만 세운다"로 바꾼다. 지운 뒤 `grep -n "follower_ready\|slowed\|\.late\|BLOCKER\|_slow_t0\|_blocker\|SLOW_FRAC\|MIN_SLOW" traffic scripts tests` 결과가 비어야 한다(스크립트 두 곳은 Task 6에서 지운다. 그 전에는 이 둘만 남는다).
- [ ] **Step 4:** Run `D:\conda_envs\faster\python.exe -m pytest tests/test_test_bus.py tests/test_injector_and_ped_cap.py tests/test_injector_plan.py -q` → PASS.

---

### Task 5: 연결 (`HapjeongTraffic`)

**Files:**
- Modify: `traffic/hapjeong.py` (생성자, `start`의 예열 루프, `run`의 `extras`, `summary`)
- Test: `tests/test_hapjeong_integration.py` (한 테스트 추가)

**Interfaces:**
- Consumes: `DensityKeeper`, `NearBusDensity`, `sections_of`, `DEFAULT_DENSITY_PER_100M`.
- Produces: `HapjeongTraffic(..., density=DEFAULT_DENSITY_PER_100M, near_bus_density=True)`. `density=None`이면 `DensityKeeper`를 끈다. 속성 `self.keeper: DensityKeeper | None`, `self.near_density: NearBusDensity | None`. `self.summary["density"] = {"keeper": keeper.summary(), "near_bus": near_density.summary()}`는 `run` 끝에 채운다.

- [ ] **Step 1: 실패하는 테스트.** `test_density_parts_are_wired_and_can_be_disabled`: `HapjeongTraffic(tmp, density=None, near_bus_density=False)`는 `start` 뒤 `keeper is None and near_density is None`, 기본값은 둘 다 객체.
- [ ] **Step 2:** Run 그 테스트 → FAIL.
- [ ] **Step 3: 구현.**
  - `start()`: `self.interior` 만든 직후 `self.keeper = DensityKeeper(traci, net, "bus0", m["routes"]["bus_loop"], exits, density, rng=random.Random(self.seed + 7), protected=lambda vid: False)`를 만들고, 예열 루프에서 `interior.update` 다음에 `keeper.update`도 부른다(버스가 없으므로 구간이 먼저 채워진다).
  - `run()`: `extras`에 `keeper`를 `interior` 다음에, `NearBusDensity(traci, net, bus_id, self.owner, density)`를 `arrival_guard` 다음에 넣는다. `keeper.protected`는 `lambda vid: vid in mgr.inter or self.owner.owner(vid) is not None`로 `run`에서 갱신한다.
  - `NearBusDensity`는 `pilot`이 있을 때만 만든다(버스가 시험용 버스일 때만 뜻이 있다).
- [ ] **Step 4:** Run `D:\conda_envs\faster\python.exe -m pytest tests/test_hapjeong_integration.py -q` → PASS.

---

### Task 6: 스크립트 정리와 측정 스크립트

**Files:**
- Modify: `scripts/run_hapjeong_scenario.py:167`, `scripts/measure_s1_s6.py:380` (`late`·`slowed` 키 제거)
- Create: `scripts/measure_section_density.py`

**Interfaces:**
- Produces: `scripts/measure_section_density.py --scenario {quiet,baseline,busy} --seed N --laps 3 [--no-density] [--out results/density_{scenario}_{tag}.json]`. 출력 JSON 키: `per_road`(도로 이름별 `mean_per_100m`, `p10_per_100m`), `queue`(신호 끝 구간별 `max_halting`, `mean_halting`), `presence`(`PresenceMonitor.summary()`의 `appear`·`vanish`·`teleport`), `bus`(`lap_times`, `forced`, `missed`), `density`(`HapjeongTraffic.summary["density"]`).

- [ ] **Step 1:** `measure_s1_s6.py`의 실행 방식(`HapjeongTraffic` 생성, `TestBusPilot`, `PresenceMonitor` 연결, `--laps`)을 읽고 같은 뼈대로 새 스크립트를 쓴다. 매 1초(`on_step`에서 `t % 1 == 0`)마다 `tc.edge.getLastStepVehicleNumber`로 `sections_of` 구간의 대수와 `getLastStepHaltingNumber`를 모은다. 구간 대수는 길이로 나눠 100 m당으로 환산한다. `late`·`slowed`는 쓰지 않는다.
- [ ] **Step 2:** 두 기존 스크립트에서 `bus.late`·`bus.slowed` 참조를 지운다. Run `D:\conda_envs\faster\python.exe -m scripts.measure_section_density --help` → 사용법 출력(오류 없음).

---

### Task 7: 측정과 목표값 조정

**Files:**
- Modify: `traffic/density_keeper.py` (`DEFAULT_DENSITY_PER_100M` 값만)
- Output: `results/density_*.json` (커밋하지 않는다)

수락 기준(모두 만족해야 끝난다):
1. `presence`의 `appear`, `vanish`, `teleport`가 baseline·busy 각 3바퀴에서 모두 0.
2. 신호 끝 구간의 `max_halting`이 `--no-density`일 때의 값 + 2대를 넘지 않는다.
3. 네 도로의 `mean_per_100m`이 목표값의 ±30 % 안.
4. 버스 바퀴 시간이 `--no-density` 대비 +10 % 이내이고, 충돌·`missed`가 늘지 않는다.
5. `bus.slowed`·`late`가 없고(Task 4), 버스 `setSpeed` 호출은 정차뿐이다.

- [ ] **Step 1:** `--no-density` 기준선을 baseline seed 0, 3바퀴로 측정한다. Run `D:\conda_envs\faster\python.exe -m scripts.measure_section_density --scenario baseline --seed 0 --laps 3 --no-density --out results/density_baseline_off.json`.
- [ ] **Step 2:** 켠 상태로 같은 조건을 측정한다(`--out results/density_baseline_on.json`). 기준 1~4를 비교한다.
- [ ] **Step 3:** 기준 2·4를 넘으면 목표값을 한 단계(0.5 veh/100 m)씩 낮추고, 기준 3을 못 맞추면 `halt_max`·`stop_zone_m`을 확인한다. 값을 바꿀 때마다 Step 2를 다시 한다. 최종 값은 `DEFAULT_DENSITY_PER_100M`에 넣는다.
- [ ] **Step 4:** busy 3바퀴(seed 0)로 기준 1·2를 확인한다. quiet는 시간이 되면 1바퀴만 본다. 돌리지 않은 것은 보고서에 "미실행"으로 적는다.
- [ ] **Step 5:** 같은 seed로 `D:\conda_envs\faster\python.exe -m pytest tests -q`를 돌려 전체 회귀를 본다. 실패는 원인을 보고하고 계획 범위 밖이면 사용자에게 알린다.

---

### Task 8: 문서와 커밋

**Files:**
- Modify: `traffic/README.md` (밀도 분산 절: 200 m → 50 m 범위, `DensityKeeper`·`NearBusDensity` 설명, 버스 감속 제거), `traffic/VERIFICATION.md` (Task 7 측정 결과 표)
- Modify: `docs/UA_ONBOARDING.md`는 건드리지 않는다.

- [ ] **Step 1:** 두 문서를 Write·Edit로 갱신한다. 결과 숫자는 Task 7의 JSON에서 옮기고, 돌리지 않은 시나리오는 "미측정"이라고 쓴다.
- [ ] **Step 2: 커밋(한 번).** 변경한 소스·테스트·문서·스크립트만 add한다(`results/`, `.ua/`, `docs/report_draft/` 등 기존 untracked 제외).
```bash
git add traffic tests scripts docs/superpowers/plans/2026-11-22-density-keeper-and-bus-no-slow.md
git commit -m "feat(traffic): 구간 밀도 유지와 버스 주변 50 m 밀도 맞추기, 시험용 버스 상호작용 감속 제거" -m "양화로·독막로·잔다리로·와우산로 목표 대수를 DensityKeeper가 직접 유지하고, 버스 반경 55 m 안에서는 투입·제거하지 않는다. 버스 주변이 비면 뒤차 가속·앞차 감속으로 채운다. 밀도 분산 투입 금지 거리를 200 m에서 50 m 범위로 줄이고, S2·S3 상대를 기다리던 시험용 버스 감속을 없앴다."
```
  메시지 끝에 attribution 줄을 붙이지 않는다.

---

## Self-Review

- **요구사항 대응:** 1 → Task 2·5, 2 → Task 2(정체 가드)·7(기준 2), 3 → Task 1·2·3·7(기준 1), 4 → Task 1, 5 → Task 3, 6 → Task 4, 7 → Task 0·8.
- **타입 일관성:** `DEFAULT_DENSITY_PER_100M`, `SECTION_ROADS`, `sections_of`는 Task 2가 정의하고 Task 3·5가 쓴다. `route_gap`은 Task 3에서 정의하고 Task 4가 원본 `_route_ahead`를 지운다. `SPAWN_MARGIN_M`은 Task 1에서 정의한다.
- **알려진 위험:** (a) 앞차 감속이 같은 차로 버스를 뒤따르게 만들어 버스가 자연 감속할 수 있다. 감속 목표를 `v_bus - 4`, 풀기 40 m로 제한했고 Task 7 기준 4가 바퀴 시간으로 잡는다. (b) 도로 이름 `getName()`은 망 파일 인코딩이 UTF-8이라 한글 비교가 되지만, 이름이 빈 도로 조각은 구간에서 빠진다(`sections_of` 테스트가 네 도로가 모두 나오는지 확인한다). (c) 목표 대수 시작값 2.0/1.5는 가정이다. Task 7에서 측정으로 정한다.
