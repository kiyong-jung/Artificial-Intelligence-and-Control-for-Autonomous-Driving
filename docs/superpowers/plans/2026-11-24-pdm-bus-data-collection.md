# PDM 버스 전문가 데이터 수집 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 합정역 시나리오에서 PDM-Closed 버스(전문가)의 원시 주행 데이터를 수집하는 수집기를 만든다. 그 원시 데이터에서 `data_colum.md` 필수 열, 93차원 관측, 입력 설계 특징, 상호작용·시나리오 지표를 만드는 전처리기도 만든다.

**Architecture:** 새 패키지 `pdm_data/`에 순수 계산(스냅숏, 기록기, 관측, 라벨, 특징)을 둔다. `scripts/collect_pdm_bus.py`는 `HapjeongTraffic.run(on_step=...)`에 기록기를 걸어 원시 값만 쓴다. `scripts/preprocess_pdm_bus.py`는 그 원시 값으로 학습용 transition과 지표를 만든다. SUMO SSM 장치는 시험 버스 vType 파라미터로 붙인다.

**Tech Stack:** Python 3.9, SUMO 1.27.1 TraCI, sumolib, numpy, pytest.

**Spec:** `docs/superpowers/specs/2026-11-24-pdm-bus-data-collection-design.md`

## Global Constraints

- Python 3.9에서 돌아야 한다. 모든 새 모듈 첫 줄은 `from __future__ import annotations`이고, 실행 시점에 `X | Y` 타입과 `match`를 쓰지 않는다.
- 파이썬은 `/c/Users/user/anaconda3/envs/main/python.exe`로 직접 부른다. 명령에 `cd`를 붙이지 않는다(프로젝트 루트에서 실행).
- 고치지 않는 파일: `scripts/run_hapjeong_scenario.py`, `scripts/run_hapjeong_pdm_bus.py`, `scripts/collect_bus_lap.py`, `collect_pomdp_data.py`, `env/` 아래 전부. `traffic/pdm_bus.py`에는 `self.last_plan` 기록만 더한다(Task 4).
- 기록 코드는 TraCI 조회만 한다. `bus0`에 `subscribeContext`를 걸지 않는다(PDM 구독을 덮어쓴다). set 계열은 `CallTap`이 그대로 전달할 때만 지난다.
- 계산 간격 0.1 s, 판단 간격 0.5 s(5스텝). 범위: state 150 m, detected 50 m, 선행 160 m, 후행 80 m, 신호 200 m, junction foes 100 m, 정류장 4개.
- 에피소드는 한 바퀴(R68 → R68)이고, 노선은 `TestBusPilot.lap_route`(R68 + `bus_loop`)다. vmax는 `map.json`의 `route_design.bus_loop.max_speed_mps`(13.89)다.
- 값이 없으면 `null`(파이썬 `None`)이다. 0으로 채우지 않는다.
- 가감속 정규화 상수는 수집기에서 쓰지 않는다. 전처리는 `--accel-pos`와 `--accel-neg`를 반드시 받고 기본값이 없다.
- 보상 가중치는 `env.bus_task.REWARDS`를 그대로 읽는다.
- 출력: `data/pdm_bus/<run_name>/`(steps.jsonl.gz, events.jsonl, ssm.xml, metadata.json, assets/). `metadata.json`은 마지막에 쓴다. 전처리 결과는 `<run>/processed/`다.
- 커밋 메시지에 `Co-Authored-By` 줄을 붙이지 않는다.
- 검증은 짧게 한다. SUMO 통합 시험은 warmup 30 s, 실행 60~120 s를 넘기지 않는다.

## Review Focus

1. **버스가 아직 투입되지 않은 시점**(출발 자리가 막힘): 행을 쓰지 않고, 버스가 나타난 판단 시점부터 `t=0`이어야 한다. → Task 4 `test_no_rows_until_bus_present`
2. **버스가 교차로 내부 차로에 있을 때**: 노선 번호는 직전 일반 도로 기준이고, 93차원 관측의 drivable 마스크는 현재 차로(오프셋 0)만 1이어야 한다. → Task 1 `test_lap_route_index_internal_uses_last_road`, Task 6 `test_obs93_internal_lane_mask`
3. **바퀴 중간에 실행이 끝날 때**(시간 한도): 마지막 스냅숏 행이 `final: true, end: "time_limit"`으로 남고, 전처리는 truncated로 처리해야 한다. → Task 4 `test_time_limit_writes_final_row`, Task 7 `test_time_limit_truncates`
4. **조회 도중 차량·보행자가 사라질 때**(TraCIException): 그 물체만 건너뛰고 수집은 계속돼야 한다. → Task 2 `test_vehicles_block_skips_vanished`
5. **버스가 순간이동하거나 충돌할 때**: 순간이동이면 직전 행에서 truncated(`teleport`), 충돌이면 그 행에서 terminated(`collision`)여야 한다. → Task 7 `test_teleport_cuts_before`, `test_collision_terminates`

---

### Task 1: 공통 상수와 한 바퀴 노선 문맥

**Files:**
- Create: `pdm_data/__init__.py`(빈 파일), `pdm_data/schema.py`, `pdm_data/lap_context.py`
- Test: `tests/test_pdm_lap_context.py`

**Interfaces:**
- Produces:
  - `pdm_data.schema`에 상수 `RAW_SCHEMA_VERSION = "pdm_bus_raw_v1"`, `BUS = "bus0"`, `DT = 0.1`, `DECISION_STEPS = 5`, `STATE_RADIUS_M = 150.0`, `DETECT_RADIUS_M = 50.0`, `LEADER_LOOKAHEAD_M = 160.0`, `FOLLOWER_LOOKBACK_M = 80.0`, `TLS_LOOKAHEAD_M = 200.0`, `FOES_LOOKAHEAD_M = 100.0`, `STOPS_AHEAD = 4`, `INDUCED_BRAKE_MPS2 = 3.0`, `S5_WINDOW_S = 15.0`, `NEAR_M = 50.0`, `DEFAULT_MAX_LAP_S = 1800.0`, `STOP_IDS = traffic.test_bus.STOP_AT`, `SERVICE_SECONDS = 8.0`, 그리고 `COLUMN_DOC: dict[str, str]`(스펙 7~9절 블록 이름 → 한 줄 설명)
  - `LapContext` dataclass: `lap_edges: list[str]`, `route_map: env.route_map.RouteMap`, `vmax: float`, `lane_allowed: dict[str, list[str]]`, `crossing_ids: frozenset[str]`, `service_stops: list[tuple[float, float, str, str]]`(start, end, stop_id, lane_id; 노선 순서), `net_path: str`
  - `build_lap_context(net_path: str, lap_edges: list[str], bus_stops: list[dict], vmax: float, lane_allowed: dict[str, list[str]]) -> LapContext`
  - `lap_route_index(road: str, last_road: str | None, lap_edges: list[str]) -> int`
  - `lane_usable(ctx: LapContext, lane_id: str, vclass: str = "bus") -> bool`

- [ ] **Step 1: 실패하는 시험을 쓴다.** 고정 데이터로 `load_map("hapjeong")`(net_path, bus_stops, routes["bus_loop"], route_design.bus_loop.max_speed_mps), `traffic.alleys.read_net(net_path)`, `traffic.test_bus.lap_route(net, loop)`를 쓴다.

```python
def test_lap_edges_unique_and_start_with_R68(ctx):
    assert ctx.lap_edges[0] == "R68" and len(set(ctx.lap_edges)) == len(ctx.lap_edges)

def test_route_map_covers_R68(ctx):
    assert ctx.route_map.edge_ids[0] == "R68" and ctx.route_map.length > 1949.44

def test_service_stops_are_four_required(ctx):
    assert [s[2] for s in ctx.service_stops] == list(STOP_IDS)
    assert all(a[0] < b[0] for a, b in zip(ctx.service_stops, ctx.service_stops[1:]))

def test_lap_route_index_internal_uses_last_road(ctx):
    e = ctx.lap_edges
    assert lap_route_index("R71", "R69", e) == e.index("R71")
    assert lap_route_index(":J1_3_0", "R42", e) == e.index("R42")

def test_crossing_ids_from_net(ctx):
    assert ctx.crossing_ids and all(c.startswith(":") and "_c" in c for c in ctx.crossing_ids)

def test_lane_usable_empty_allowed_means_all(ctx):
    ctx.lane_allowed.update({"X_0": [], "X_1": ["pedestrian"], "X_2": ["bus", "taxi"]})
    assert lane_usable(ctx, "X_0") and not lane_usable(ctx, "X_1") and lane_usable(ctx, "X_2")
```

- [ ] **Step 2: 시험이 실패하는지 확인한다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_lap_context.py -v` → Expected: FAIL (ModuleNotFoundError: pdm_data)
- [ ] **Step 3: 구현한다.** `RouteMap(net_path, "lap", lap_edges, bus_stops, "bus")`를 쓴다. `service_stops`는 `route_map.stops` 중 id가 `STOP_IDS`에 있는 것이다. `crossing_ids`는 net.xml의 `function="crossing"` edge id다(`RouteMap`과 같은 방식). `lane_usable`은 `env/sumo_env.py:_usable`과 같은 규칙(`not allowed or vclass in allowed`)이다. 표에 없는 차로는 False다.
- [ ] **Step 4: 시험이 통과하는지 확인한다.** 같은 명령 → PASS
- [ ] **Step 5: 커밋한다.** `git add pdm_data tests/test_pdm_lap_context.py && git commit -m "feat(pdm_data): 공통 상수와 한 바퀴 노선 문맥"`

---

### Task 2: TraCI 스냅숏

**Files:**
- Create: `pdm_data/snapshot.py`, `tests/pdm_fakes.py`
- Test: `tests/test_pdm_snapshot.py`

**Interfaces:**
- Consumes: `LapContext`, `lap_route_index`, `pdm_data.schema` 상수
- Produces:
  - `take_snapshot(tc, ctx: LapContext, last_road: str | None, roles: dict, now: float) -> dict`. 반환 키는 정확히 `{"ego", "route", "tls", "stops", "neighbors", "junction_foes", "vehicles", "pedestrians"}`다. `roles`는 `{"vehicles": {id: dict}, "pedestrians": {id: dict}}`다(Task 4 `RoleBook`가 만든다).
  - 블록 함수: `ego_block(tc, ctx, last_road) -> dict`, `route_block(tc, ctx, lap_idx) -> dict`, `tls_block(tc, now) -> list[dict]`, `stops_block(tc) -> dict`, `neighbors_block(tc) -> dict`, `junction_foes_block(tc) -> list[dict]`, `vehicles_block(tc, ctx, ego_xy, ego_pos, roles) -> list[dict]`, `pedestrians_block(tc, ctx, ego_xy, roles) -> list[dict]`
  - `tests/pdm_fakes.py`: `FakeTC`. `vehicle`, `person`, `lane`, `edge`, `trafficlight`, `simulation` 도메인을 dict로 채운다. 없는 id를 조회하면 `traci.exceptions.TraCIException`을 던진다. 호출 기록은 `FakeTC.calls`에 쌓는다.

블록별 키는 스펙 7.2, 7.4~7.10을 그대로 따른다. 정할 것:
- `ego`: 스펙 7.2의 키와 `lap_route_index`, `lap_position`(`route_map.position(lap_idx, lane_id, lane_position)`)이다. `length`, `width`, `min_gap`은 넣지 않는다(메타데이터).
- `neighbors`: 키는 `leader`, `follower`, `left_leader`, `right_leader`, `left_follower`, `right_follower`다. 값은 `None` 또는 `{"id", "gap_raw", "secure_gap", "follow_speed"?, "stop_speed"?}`다.
  - getNeighbors 모드는 left_leader=3, right_leader=2, left_follower=1, right_follower=0이고, 결과가 여럿이면 `abs(dist)`가 가장 작은 것을 고른다.
  - 선행 슬롯의 `secure_gap`은 `getSecureGap(BUS, v_ego, v_lead, getDecel(lead), lead)`, `follow_speed`는 `getFollowSpeed(BUS, v_ego, gap, v_lead, getDecel(lead), lead)`, `stop_speed`는 `getStopSpeed(BUS, v_ego, gap)`다.
  - 후행 슬롯의 `secure_gap`은 `getSecureGap(fol, v_fol, v_ego, getDecel(BUS), BUS)`다.
- `tls`: `getNextTLS` 중 거리 ≤ 200 m. 항목은 `{"id", "link_index", "distance", "state", "time_to_switch": getNextSwitch − now, "spent", "phase", "program"}`이다.
- `stops`: `{"stop_state": int, "stops": [StopData 필드 dict], "distances": [노선 거리 또는 None]}`이다. StopData 필드는 lane, startPos, endPos, stoppingPlaceID, stopFlags, duration, until, intendedArrival, arrival, depart다. 남은 정차 시간은 Task 4가 `stopped_since`로 따로 남긴다.
- `vehicles`: `getIDList` 전체에서 bus를 빼고, ego `getPosition`과의 직선 거리 ≤ 150 m인 것이다.
  - 키는 스펙 7.9 기본 키 + `driver_type`, `p_yield`(`getParameter`, 빈 문자열이면 None) + `roles`에서 온 `injected`, `role` + `route_distance`다. `route_distance`는 그 차 도로가 `lap_edges`에 있을 때만 `lap_position(그 차) − ego.lap_position`이고, 아니면 None이다.
  - 조회 중 TraCIException이 나면 그 차만 뺀다.
- `pedestrians`: 반경 150 m다. 키는 스펙 7.10이고, `on_crossing = road_id in ctx.crossing_ids`다.

- [ ] **Step 1: 실패하는 시험을 쓴다.**

```python
def test_vehicles_block_filters_radius_150(tc_two_cars):          # 하나는 100 m, 하나는 200 m
    v = take_snapshot(tc_two_cars, ctx, "R69", ROLES, 10.0)["vehicles"]
    assert [x["id"] for x in v] == ["near"]

def test_vehicles_block_skips_vanished(tc_vanishing):              # getSpeed("gone")가 TraCIException
    v = take_snapshot(tc_vanishing, ctx, "R69", ROLES, 10.0)["vehicles"]
    assert "gone" not in [x["id"] for x in v] and len(v) == 1

def test_neighbors_null_when_empty(tc_alone):
    n = take_snapshot(tc_alone, ctx, "R69", ROLES, 10.0)["neighbors"]
    assert set(n) == {"leader", "follower", "left_leader", "right_leader", "left_follower", "right_follower"}
    assert all(v is None for v in n.values())

def test_leader_secure_gap_uses_leader_decel(tc_leader):            # leader decel 4.5
    snap = take_snapshot(tc_leader, ctx, "R69", ROLES, 10.0)
    assert ("vehicle.getSecureGap", ("bus0", 8.0, 6.0, 4.5, "lead")) in tc_leader.calls
    assert snap["neighbors"]["leader"]["gap_raw"] == 20.0

def test_tls_time_to_switch_relative(tc_tls):                       # getNextSwitch = 37.0, now = 10.0
    t = take_snapshot(tc_tls, ctx, "R69", ROLES, 10.0)["tls"][0]
    assert t["time_to_switch"] == 27.0 and t["state"] == "r"

def test_pedestrian_on_crossing(tc_ped):                            # road_id가 ctx.crossing_ids 중 하나
    p = take_snapshot(tc_ped, ctx, "R69", ROLES, 10.0)["pedestrians"][0]
    assert p["on_crossing"] is True

def test_no_context_subscription(tc_two_cars):
    take_snapshot(tc_two_cars, ctx, "R69", ROLES, 10.0)
    assert not any(name.endswith("subscribeContext") for name, _ in tc_two_cars.calls)
```

- [ ] **Step 2: 시험이 실패하는지 확인한다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_snapshot.py -v` → Expected: FAIL (ImportError)
- [ ] **Step 3: `pdm_data/snapshot.py`와 `tests/pdm_fakes.py`를 구현한다.**
- [ ] **Step 4: 시험이 통과하는지 확인한다.** 같은 명령 → PASS
- [ ] **Step 5: 커밋한다.** `git add pdm_data/snapshot.py tests/pdm_fakes.py tests/test_pdm_snapshot.py && git commit -m "feat(pdm_data): TraCI 스냅숏 블록"`

---

### Task 3: SSM 장치 vType과 출력 파서

**Files:**
- Create: `pdm_data/ssm.py`
- Test: `tests/test_pdm_ssm.py`

**Interfaces:**
- Produces:
  - `SSM_PARAMS: dict[str, str]`: `has.ssm.device="true"`, `device.ssm.measures="TTC DRAC PET BR SGAP TGAP"`, `device.ssm.thresholds="4.0 3.0 2.0 0.0 0.2 0.5"`, `device.ssm.range="50"`, `device.ssm.extratime="5"`, `device.ssm.trajectories="false"`
  - `ssm_vtype(base_vtype: str, out_file: str) -> str`: 자기 닫힘 `<vType .../>`을 `<vType ...>` + `<param .../>`들(+`device.ssm.file=out_file`) + `</vType>`으로 바꾼다.
  - `parse_ssm(path: str) -> list[dict]`: `<conflict>`마다 `{"begin", "end", "ego", "foe", "min_ttc": {"time", "value"} | None, "max_drac": ... | None, "pet": ... | None, "types": list[str]}`를 만든다. 값이 `NA`면 None이다. 파일이 없거나 비면 `[]`다.

- [ ] **Step 1: 실패하는 시험을 쓴다.**

```python
def test_ssm_vtype_is_valid_xml_with_params():
    s = ssm_vtype(BUS_PDM_VTYPE, "D:/x/ssm.xml")
    root = ET.fromstring(s)
    params = {p.get("key"): p.get("value") for p in root.findall("param")}
    assert root.get("id") == "bus_test" and root.get("carFollowModel") == "IDM"
    assert params["has.ssm.device"] == "true" and params["device.ssm.file"] == "D:/x/ssm.xml"
    assert params["device.ssm.measures"] == "TTC DRAC PET BR SGAP TGAP"

def test_parse_ssm_reads_min_ttc_and_na(tmp_path):
    p = tmp_path / "ssm.xml"
    p.write_text(SAMPLE_SSM)   # conflict 2개: 하나는 minTTC 1.8, 다른 하나는 PET만 있고 minTTC value="NA"
    c = parse_ssm(str(p))
    assert c[0]["ego"] == "bus0" and c[0]["min_ttc"]["value"] == 1.8
    assert c[1]["min_ttc"] is None and c[1]["pet"]["value"] == 0.9

def test_parse_ssm_missing_file_is_empty(tmp_path):
    assert parse_ssm(str(tmp_path / "none.xml")) == []
```

`SAMPLE_SSM`은 SUMO 문서 형식(`<SSMLog><conflict begin end ego foe><conflictTypes .../><minTTC time value .../><maxDRAC .../><PET .../></conflict></SSMLog>`)으로 쓴다. Task 5의 짧은 실행에서 나온 실제 ssm.xml로 다시 확인하고, 형식이 다르면 파서와 이 고정 데이터를 함께 고친다.

- [ ] **Step 2: 시험이 실패하는지 확인한다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_ssm.py -v` → FAIL
- [ ] **Step 3: 구현한다.** `xml.etree.ElementTree`를 쓴다.
- [ ] **Step 4: 시험이 통과하는지 확인한다.** → PASS
- [ ] **Step 5: 커밋한다.** `git add pdm_data/ssm.py tests/test_pdm_ssm.py && git commit -m "feat(pdm_data): SSM 장치 vType과 출력 파서"`

---

### Task 4: 기록기(판단 구간, 바퀴 나누기, 역할·시나리오·사건)

**Files:**
- Create: `pdm_data/recorder.py`, `pdm_data/scenario_tags.py`, `tests/pdm_fixtures.py`
- Modify: `traffic/pdm_bus.py`의 `PDMClosedBusPilot.__init__`(`self.last_plan = None`)과 `_pdm_step`(점수 계산 뒤 `self.last_plan` 설정)
- Test: `tests/test_pdm_recorder.py`

**Interfaces:**
- Consumes: `take_snapshot`, `LapContext`, `lap_route_index`, 스키마 상수
- Produces:
  - `traffic/pdm_bus.py`: `PDMClosedBusPilot.last_plan: dict | None`. 키는 `sim_time, fraction, v0, scores, mult, progress, ttc, comfort`(각 길이 5 list[float]), `collide_t`, `emergency_brake: bool`이다. 동작은 바꾸지 않는다.
  - `CallTap(tc, bus_id)`: `install()`은 `tc.vehicle.changeLane/setMaxSpeed/setSpeed`를 감싸 bus 호출만 `{"t", "call", "args"}`로 쌓고 원래 함수를 그대로 부른다. `drain() -> list[dict]`. `uninstall()`.
  - `RoleBook(world)`: `update()`는 `world.injector.log`, `world.mgr.inter`/`world.mgr.log`, `world.ped.log`, `world.ahead`(있으면)를 읽는다. 결과는 `roles() -> {"vehicles": {...}, "pedestrians": {...}}`, `new_events() -> list[dict]`(kind `a1`, `a3`, `injection`, `s4_block`; 로그에 새로 생긴 줄만)이다.
    - 차량 역할: id 접두어 `inj_s5` → `"S5"`, `stopahead`·`stopped`·`taxi0` → `"S4"`; 주입기 로그의 S2/S3 상대 id → `"S2"`/`"S3"`; `mgr.inter`에 있으면 `"A1_yield"`/`"A1_block"`(`inter[vid].yields`).
    - 보행자 역할: `inj_s6_` → `"S6_injected"`, `s6ped` → `"S6_background"`, `a3_decision`은 `ped.log`에서 그 `ped`의 마지막 `cross` 값이다.
    - `injected`는 주입기 로그나 접두어 `inj_`에서 정한다.
  - `ScenarioTracker(tc, world, pilot, ctx, feeders: set[str])`: `step(t)`는 0.1 s마다 부른다. `tags() -> dict`는 스펙 7.11의 키(`s1, s2, s3, s4, s5, s6, a1, injector_plan`)를 낸다. `new_events() -> list[dict]`는 kind `lane_window`, `s1_stop`, `s5_cutin`, `s6_pass`, `induced_brake`, `collision`, `teleport`, `emergency_stop`이다.
    - 판정 규칙은 `scripts/measure_s1_s6.py:on_step`과 같다: S5 15 s / 50 m, S6은 횡단보도 50 m 앞, induced_brake는 50 m 안 차량의 감속 ≥ 3.0 m/s².
    - `s6_pass`의 PET는 `traffic.metrics.pet`로 계산한다.
  - `RunWriter(run_dir: str)`: `write_step(row: dict)`, `write_event(ev: dict)`, `close()`. 출력은 gzip JSONL과 JSONL이고 float는 6자리로 반올림한다.
  - `Recorder(tc, world, pilot, ctx, writer, max_laps: int, max_seconds: float, check=None)`: `on_step(t)`는 runner가 0.1 s마다 부른다. `stopped_since`를 추적한다. 끝나면 `CollectionDone(reason)`을 던진다(`"laps"`, `"time_limit"`, `"bus_lost"`). `finish(reason)`은 마지막 행을 쓴다. `summary() -> dict`. `tls_seen: set[str]`.
  - 행 형식: `{"meta": {run_id, episode, t, sim_time, sim_step, final, end}, "expert": {...}, "scenario": {...}, "stopped_since": float | None, **take_snapshot(...), "window": {...} | None}`.
    - `window` 키: `substeps`(5개, 각 `{t, edge, lane_id, lane_index, lane_position, lap_index, lap_position, distance, speed, acceleration, tls0, phase, request_direction}`), `peak_acceleration`, `peak_jerk`, `calls`(CallTap), `lane_before`, `lane_after`, `edge_before`, `edge_after`, `collisions`(`{collider, victim, type, ego}`), `teleports`(`{id, ego}`), `emergency_stops`, `lap_done`, `pdm_plans`(구간 안의 `last_plan` 변화).
    - `expert` 키: `phase, why, plan_kind, target_lane, request_direction, forced_stop, speed_limited, blocker_active, pdm`(`last_plan`)다.
  - `tests/pdm_fixtures.py`: `make_row(episode=1, t=0, speed=5.0, edge="R69", lane_index=1, x=0.0, y=0.0, phase="idle", **over) -> dict`. 위 형식을 빠짐없이 채운 최소 행이다. Task 6~9가 쓴다.

순서 규칙(스펙 7절):
- runner는 `bus_hook → extras → simulationStep → on_step(t)` 순서로 돈다. `on_step`은 매번 substep 하나를 현재 구간에 더한다.
- 버스가 처음 보인 뒤 `sim_step % 5 == 0`인 시점마다 다음 순서로 처리한다: 직전 행에 `window`를 채워 쓴다 → 새 스냅숏을 찍는다.
- 바퀴 번호(`pilot.lap`)가 바뀌었으면 새 스냅숏을 두 번 쓴다. 먼저 이전 episode의 `final: true, end: "lap"` 행(window 없음)으로, 그다음 새 episode의 `t=0` 행으로 쓴다.

- [ ] **Step 1: 실패하는 시험을 쓴다.** `FakeTC`와 가짜 pilot·world(`tests/pdm_fakes.py`에 `FakePilot(lap, phase, ...)`, `FakeWorld(mgr, injector, ped, ahead)` 추가)를 쓴다.

```python
def test_no_rows_until_bus_present():
    rec, w = make_recorder(bus_appears_at_step=7)
    run_steps(rec, 21)
    rows = w.steps
    assert rows[0]["meta"]["t"] == 0 and rows[0]["meta"]["sim_step"] == 10

def test_rows_every_5_steps_window_has_5_substeps():
    rec, w = make_recorder()
    run_steps(rec, 16)
    assert [r["meta"]["t"] for r in w.steps] == [0, 1, 2]
    assert all(len(r["window"]["substeps"]) == 5 for r in w.steps)

def test_lap_change_writes_final_then_t0():
    rec, w = make_recorder(lap_changes_at_step=10)
    run_steps(rec, 16)
    eps = [(r["meta"]["episode"], r["meta"]["t"], r["meta"]["final"]) for r in w.steps]
    assert (1, 2, True) in eps and (2, 0, False) in eps

def test_time_limit_writes_final_row():
    rec, w = make_recorder(max_seconds=1.0)
    with pytest.raises(CollectionDone):
        run_steps(rec, 30)
    rec.finish("time_limit")
    assert w.steps[-1]["meta"]["final"] and w.steps[-1]["meta"]["end"] == "time_limit"

def test_calltap_forwards_and_records():
    tc = FakeTC(); tap = CallTap(tc, "bus0"); tap.install()
    tc.vehicle.changeLane("bus0", 2, 5.0); tc.vehicle.changeLane("other", 1, 5.0)
    assert [c["args"] for c in tap.drain()] == [("bus0", 2, 5.0)]
    assert ("vehicle.changeLane", ("other", 1, 5.0)) in tc.calls

def test_lane_window_event_on_signal_request_idle():
    rec, w = make_recorder(phases=["idle"] * 5 + ["signal"] * 5 + ["request"] * 5 + ["idle"] * 5, lane_after_step=14)
    run_steps(rec, 21)
    ev = [e for e in w.events if e["kind"] == "lane_window"]
    assert len(ev) == 1 and ev[0]["detail"]["success"] is True

def test_rolebook_a1_and_injected():
    rb = RoleBook(FakeWorld(inter={"car7": yields(True)}, injector_log=[{"kind": "S5", "id": "inj_s5_1", "ok": True}]))
    rb.update()
    r = rb.roles()["vehicles"]
    assert r["car7"]["role"] == "A1_yield" and r["inj_s5_1"]["injected"] is True

def test_fixture_matches_recorder_row_keys():
    rec, w = make_recorder(); run_steps(rec, 6)
    assert set(w.steps[0]) == set(make_row())
```

- [ ] **Step 2: 시험이 실패하는지 확인한다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_recorder.py -v` → FAIL
- [ ] **Step 3: `pdm_bus.py`에 `last_plan`을 더하고, `scenario_tags.py`, `recorder.py`, 고정 데이터를 구현한다.**
- [ ] **Step 4: 새 시험과 기존 PDM 시험이 통과하는지 확인한다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_recorder.py tests/test_pdm_bus.py -v` → PASS
- [ ] **Step 5: 커밋한다.** `git add pdm_data traffic/pdm_bus.py tests/pdm_fakes.py tests/pdm_fixtures.py tests/test_pdm_recorder.py && git commit -m "feat(pdm_data): 판단 구간 기록기, 역할·시나리오·사건, PDM 마지막 계획 기록"`

---

### Task 5: 수집 스크립트, 메타데이터, 온라인 관측 확인 모드

**Files:**
- Create: `scripts/collect_pdm_bus.py`, `pdm_data/obs_check.py`, `pdm_data/metadata.py`
- Test: `tests/test_pdm_collect_integration.py` (SUMO 필요. `PDM_INTEGRATION=1`일 때만 돈다)

**Interfaces:**
- Consumes: Task 1~4 전부, `traffic.hapjeong`(모듈 변수 `BUS_VTYPE`), `HapjeongTraffic`, `PDMClosedBusPilot`, `BUS_PDM_VTYPE`, `run_hapjeong_scenario.map_stops`, `env.bus_task.BusTask`
- Produces:
  - `collect(args: argparse.Namespace) -> dict` (`scripts/collect_pdm_bus.py`). 반환은 `{"run_dir", "rows", "laps_completed", "end", "bus_trace"}`다. `bus_trace`는 0.5 s마다 `(sim_time, x, y, speed)`다.
  - CLI 인자(기본값은 `run_hapjeong_scenario.py`와 같다): `--scenario {quiet,baseline,busy}`(baseline), `--seed`(0), `--warmup`(120), `--laps`(1), `--max-seconds`(1800), `--lane-buses`(30), `--taxis`(40), `--route-boost`(1.3), `--stopped`(여러 번), `--no-inject`, `--no-stop-ahead`, `--no-ped-defiance`, `--no-density`, `--check-obs N`(0), `--no-record`, `--name`(`pdm_<scenario>_s<seed>`), `--out-dir`(`data/pdm_bus`), `--work-dir`(임시 폴더).
  - `--no-record`: 기록기·SSM·CallTap 없이 `BUS_PDM_VTYPE` 그대로 돌리고 `bus_trace`만 낸다(동작 불변 비교용).
  - `pdm_data/obs_check.py`: `LapBusTask(BusTask)`는 `pose()`만 바꾼다(`getRouteIndex` 대신 `lap_route_index`). `ObsEnvAdapter(ctx, tc)`는 BusTask가 쓰는 `ego_id`, `vmax`, `visibility=50.0`, `obs_offsets=(2, 1, 0, -1, -2)`, `route_map`, `_pose`, `_offset_lane`(`sumo_env._offset_lane`과 같은 규칙, `lane_usable` 사용)을 가진다. `OnlineObsCheck(ctx, tc, n_rows)`는 `tick()`을 0.1 s마다, `observe() -> list[float] | None`을 스냅숏마다 부르고 바퀴가 바뀌면 `reset()`한다. 결과는 행의 `check.obs93_online`에 넣는다.
  - `build_metadata(args, world, pilot, ctx, recorder, tc) -> dict`: 스펙 10절 전체다. 포함하는 것: git `rev-parse HEAD`와 `status --porcelain` 유무, SUMO 버전(`traci.getVersion()`), `PDMCfg` asdict, vType 문자열, `lane_allowed` 표, `tls_programs`(`recorder.tls_seen`의 `getAllProgramLogics`: phases의 duration, state, minDur, maxDur), `lane_plans`, `forbidden_intervals`, `net_sha256`, `ego_dims`(length 12, width 2.5, min_gap 1.0), `service_seconds`, `column_doc`, 결과 요약(`recorder.summary()`, `pilot.summary()`).
  - 수집 순서:
    1. `hapjeong.BUS_VTYPE = ssm_vtype(BUS_PDM_VTYPE, <run_dir>/ssm.xml)`
    2. `world.build()`
    3. `assets/`에 `network.net.xml`과 `traffic.add.xml`을 복사한다.
    4. `start(warmup)` → pilot → `ctx`. `lane_allowed`는 lap 도로 모든 차로의 `tc.lane.getAllowed`다.
    5. `world.run(..., on_step=recorder.on_step, stop_bypass=False, ped_defiance=not ..., stop_ahead=not ..., scenario_inject=not ..., pilot=pilot, triggers="scenario")`
    6. `CollectionDone`을 받으면 `finish` → `world.close()` → `metadata.json`을 마지막에 쓴다.
  - 수집 중에는 `<out>/.<name>.partial/`에 쓰고, 끝나면 `<out>/<name>/`으로 옮긴다(`collect_pomdp_data.py` 관례).

- [ ] **Step 1: 실패하는 통합 시험을 쓴다.** 공통 인자: `--warmup 30 --max-seconds 60 --check-obs 100`

```python
pytestmark = pytest.mark.skipif(os.environ.get("PDM_INTEGRATION") != "1", reason="SUMO 통합 시험")

def test_short_run_writes_all_files(short_run):
    d = short_run["run_dir"]
    for f in ("steps.jsonl.gz", "events.jsonl", "ssm.xml", "metadata.json", "assets/network.net.xml"):
        assert os.path.exists(os.path.join(d, f))
    meta = json.load(open(os.path.join(d, "metadata.json"), encoding="utf-8"))
    assert meta["schema_version"] == "pdm_bus_raw_v1" and meta["net_sha256"] == sha256(os.path.join(d, "assets/network.net.xml"))
    rows = list(read_steps(d))
    assert len(rows) >= 100 and rows[-1]["meta"]["final"] and rows[-1]["meta"]["end"] == "time_limit"
    assert any(r["expert"]["pdm"] and len(r["expert"]["pdm"]["scores"]) == 5 for r in rows)
    assert sum(1 for r in rows if r.get("check", {}).get("obs93_online")) == 100

def test_recording_does_not_change_bus(short_run, short_run_no_record):
    a, b = short_run["bus_trace"], short_run_no_record["bus_trace"]
    n = min(len(a), len(b))
    assert n >= 100 and all(abs(p[3] - q[3]) < 1e-9 and abs(p[1] - q[1]) < 1e-6 for p, q in zip(a[:n], b[:n]))
```

`read_steps(run_dir) -> Iterator[dict]`는 이 Task에서 `pdm_data/recorder.py`에 더한다.

- [ ] **Step 2: 시험이 실패하는지 확인한다.** Run: `PDM_INTEGRATION=1 /c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_collect_integration.py -v` → FAIL (스크립트 없음)
- [ ] **Step 3: `collect_pdm_bus.py`, `obs_check.py`, `metadata.py`를 구현한다.**
- [ ] **Step 4: 시험이 통과하는지 확인한다.** 같은 명령 → PASS. 나온 `ssm.xml`이 Task 3의 `SAMPLE_SSM` 형식과 같은지 확인하고, 다르면 Task 3 파서와 고정 데이터를 고친다. 동작 불변 시험이 실패하면 기록 코드가 시뮬레이션을 건드린 것이므로 원인을 찾아 고친다(허용 오차를 늘리지 않는다).
- [ ] **Step 5: 커밋한다.** `git add scripts/collect_pdm_bus.py pdm_data tests/test_pdm_collect_integration.py && git commit -m "feat(scripts): PDM 버스 원시 데이터 수집기, 메타데이터, 온라인 관측 확인 모드"`

---

### Task 6: 정차·신호 재생과 오프라인 93차원 관측

**Files:**
- Create: `pdm_data/replay.py`, `pdm_data/obs93.py`
- Test: `tests/test_pdm_obs93.py`

**Interfaces:**
- Consumes: `LapContext`, `lane_usable`, `make_row`, `env.bus_task.ServiceTracker`, `env.bus_task.crossed_red`
- Produces:
  - `replay_episode(rows: list[dict], ctx: LapContext) -> list[dict]`: 행마다 그 행 스냅숏 시점의 `{"tracker_index", "dwell", "completed": list[str], "missed": bool}`과 그 행 window 동안의 `{"services": int, "red_violation": bool, "missed_now": bool, "progress_m": float}`를 낸다.
    - 정차 판정은 `ServiceTracker(ctx.service_stops, SERVICE_SECONDS)`에 substep마다 `update(lap_position, lane_id, speed, 0.1)`을 부른다.
    - 신호 위반은 직전 substep과 지금 substep에 `crossed_red({"tls", "edge", "distance"}, ...)`를 쓴다(window 사이에도 이어 간다).
    - 진행 거리는 substep `distance` 증가분 중 0 이상인 값의 합이다.
  - `obs93(row: dict, ctx: LapContext, tracker_state: dict) -> np.ndarray`: shape (93,), float32, [-1,1]로 자른다. 식은 `env/bus_task.py:BusTask.observation`과 줄마다 같다. 바꿔 읽는 곳은 다음과 같다.
    - ego pose: `row["ego"]`
    - drivable: `ctx`와 `lane_usable`, 내부 차로면 오프셋 0만
    - 차량·보행자: `row["vehicles"]`/`row["pedestrians"]` 중 직선 50 m, `(d², id)` 순
    - 내비게이션: `ctx.route_map.ahead(ego.lap_position)`
    - 신호: `row["tls"][0]`
    - 정류장 중심점: sumolib `net.getLane(lane).getShape()`에 `positionAtShapeOffset` 적용

- [ ] **Step 1: 실패하는 시험을 쓴다.**

```python
def test_obs93_free_road_shape_and_speed():
    o = obs93(make_row(speed=6.945), CTX, fresh_state())
    assert o.shape == (93,) and o.dtype == np.float32 and abs(o[0] - 0.5) < 1e-6

def test_obs93_vehicle_slot_ahead():
    row = make_row(heading=0.0, vehicles=[veh("a", x=0.0, y=10.0, speed=0.0, heading=0.0)])
    o = obs93(row, CTX, fresh_state())
    assert list(o[6:11]) == pytest.approx([1.0, 0.2, 0.0, -row["ego"]["speed"] / 13.89, 0.0], abs=1e-6)

def test_obs93_internal_lane_mask():
    o = obs93(make_row(edge=":J1_3_0", lane_id=":J1_3_0"), CTX, fresh_state())
    assert list(o[1:6]) == [0.0, 0.0, 1.0, 0.0, 0.0]

def test_replay_service_after_8s_in_stop():
    rows = rows_stopped_in(CTX.service_stops[0], seconds=8.5)
    rep = replay_episode(rows, CTX)
    assert sum(r["services"] for r in rep) == 1 and rep[-1]["completed"] == [STOP_IDS[0]]

def test_replay_missed_when_passing_stop():
    rep = replay_episode(rows_passing(CTX.service_stops[0]), CTX)
    assert rep[-1]["missed"] is True

def test_replay_red_violation():
    rep = replay_episode(rows_crossing_red(), CTX)
    assert any(r["red_violation"] for r in rep)
```

- [ ] **Step 2: 시험이 실패하는지 확인한다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_obs93.py -v` → FAIL
- [ ] **Step 3: 구현한다.**
- [ ] **Step 4: 시험이 통과하는지 확인한다.** 같은 명령 → PASS
- [ ] **Step 5: 온라인 값과 비교하는 통합 시험을 더한다.** `tests/test_pdm_collect_integration.py`에 아래 시험을 넣는다. Run: `PDM_INTEGRATION=1 /c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_collect_integration.py::test_offline_obs93_matches_online -v` → PASS

```python
def test_offline_obs93_matches_online(short_run):
    rows = [r for r in read_steps(short_run["run_dir"]) if r["meta"]["episode"] == 1]
    ctx = load_lap_context(short_run["run_dir"])        # metadata + assets로 다시 만든다(Task 9에서도 쓴다)
    rep = replay_episode(rows, ctx)
    diffs = [np.abs(obs93(r, ctx, s) - np.asarray(r["check"]["obs93_online"])).max()
             for r, s in zip(rows, rep) if r.get("check", {}).get("obs93_online")]
    assert len(diffs) >= 100 and max(diffs) < 1e-4
```

  `load_lap_context(run_dir: str) -> LapContext`는 `pdm_data/lap_context.py`에 더한다(`metadata.json`의 lap_edges, bus_stops, vmax, lane_allowed와 `assets/network.net.xml` 사용). 100행만 있지만 스펙의 200행 기준은 Task 9의 수동 확인에서 채운다(아래 Task 9 Step 6).
- [ ] **Step 6: 커밋한다.** `git add pdm_data tests/test_pdm_obs93.py tests/test_pdm_collect_integration.py && git commit -m "feat(pdm_data): 정차·신호 재생과 오프라인 93차원 관측"`

---

### Task 7: 행동 라벨, 보상, 에피소드 끝 규칙

**Files:**
- Create: `pdm_data/labels.py`
- Test: `tests/test_pdm_labels.py`

**Interfaces:**
- Consumes: `replay_episode` 결과, `make_row`, `env.bus_task.REWARDS`
- Produces:
  - `accel_mps2(row) -> float`: `(window.substeps[-1].speed − ego.speed) / 0.5`
  - `normalize_accel(a: float, a_pos: float, a_neg: float) -> tuple[float, bool]`: (자른 값, 잘렸는가)
  - `lane_change_label(row) -> int`: substep 중 `phase == "request"`가 하나라도 있으면 그 substep들의 마지막 0 아닌 `request_direction`, 아니면 0
  - `lane_change_applied(row, label: int) -> bool`: label 0이면 True. 아니면 `edge_before == edge_after`이고 `sign(lane_after − lane_before) == label`이다.
  - `reward_terms(row, rep: dict, ctx, weights: dict, terminal: str | None) -> dict`: 키는 `progress, step_cost, service, terminal_value, total`이다. `total`은 `env/bus_task.py:BusTask.outcome`과 같다. 종료 사건이 있으면 그 값으로 대체하고(collision −10, red_light −10, missed_stop −5, arrived 10, incomplete_service −5), 아니면 `10 × progress_m / route_length − 0.001 + 2 × services`다.
  - `episode_end(rows: list[dict], rep: list[dict], cut_on_task_failure: bool = True) -> dict`: `{"last_index", "terminated", "truncated", "reason"}`. 앞에서부터 처음 나오는 사건을 쓴다.
    - window에 ego 순간이동 → 그 행 직전에서 끊는다. truncated, `"teleport"`
    - ego 충돌 → 그 행에서 terminated, `"collision"`
    - `cut_on_task_failure`가 켜져 있을 때 신호 위반·정류장 놓침 → terminated, `"red_light"`/`"missed_stop"`
    - `window.lap_done`이고 4곳 모두 정차 → terminated, `"arrived"`. 일부만 정차 → terminated, `"incomplete_service"`
    - 마지막 행이 `end == "time_limit"`이거나 `end == "bus_lost"` → truncated, 그 이름
  - `accel_report(values: list[float], candidates: list[tuple[float, float]]) -> dict`: 백분위 p1, p5, p50, p95, p99, min, max, 후보별 잘림 비율

- [ ] **Step 1: 실패하는 시험을 쓴다.**

```python
def test_accel_mps2_and_normalize_clip():
    row = make_row(speed=5.0, window_speeds=[5.2, 5.4, 5.6, 5.8, 6.0])
    assert accel_mps2(row) == pytest.approx(2.0)
    assert normalize_accel(2.0, 1.5, 3.0) == (1.0, True)
    assert normalize_accel(-1.5, 1.5, 3.0) == (-0.5, False)

def test_lane_change_label_whole_request_phase():
    assert lane_change_label(make_row(window_phases=["request"] * 5, window_dirs=[1] * 5)) == 1
    assert lane_change_label(make_row(window_phases=["signal"] * 5, window_dirs=[1] * 5)) == 0

def test_lane_change_applied():
    assert lane_change_applied(make_row(lane_before=1, lane_after=2, edge_before="R69", edge_after="R69"), 1)
    assert not lane_change_applied(make_row(lane_before=1, lane_after=1), 1)
    assert lane_change_applied(make_row(), 0)

def test_reward_matches_bus_task_formula():
    r = reward_terms(make_row(), {"progress_m": 5.0, "services": 1}, CTX, REWARDS, None)
    assert r["total"] == pytest.approx(10 * 5.0 / CTX.route_map.length - 0.001 + 2.0)
    assert reward_terms(make_row(), {"progress_m": 5.0, "services": 0}, CTX, REWARDS, "collision")["total"] == -10.0

def test_collision_terminates():
    rows = [make_row(t=i) for i in range(5)]; rows[3]["window"]["collisions"] = [{"ego": True}]
    assert episode_end(rows, fresh_rep(rows)) == {"last_index": 3, "terminated": True, "truncated": False, "reason": "collision"}

def test_teleport_cuts_before():
    rows = [make_row(t=i) for i in range(5)]; rows[3]["window"]["teleports"] = [{"id": "bus0", "ego": True}]
    assert episode_end(rows, fresh_rep(rows))["last_index"] == 2

def test_time_limit_truncates():
    rows = [make_row(t=i) for i in range(4)] + [make_row(t=4, final=True, end="time_limit")]
    assert episode_end(rows, fresh_rep(rows))["reason"] == "time_limit"

def test_missed_stop_kept_when_cut_disabled():
    rows, rep = rows_with_missed_stop()
    assert episode_end(rows, rep, cut_on_task_failure=False)["reason"] != "missed_stop"
```

- [ ] **Step 2: 시험이 실패하는지 확인한다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_labels.py -v` → FAIL
- [ ] **Step 3: 구현한다.**
- [ ] **Step 4: 시험이 통과하는지 확인한다.** → PASS
- [ ] **Step 5: 커밋한다.** `git add pdm_data/labels.py tests/test_pdm_labels.py tests/pdm_fixtures.py && git commit -m "feat(pdm_data): 행동 라벨, 보상 재계산, 에피소드 끝 규칙"`

---

### Task 8: 상호작용 지표와 입력 설계 특징

**Files:**
- Create: `pdm_data/interaction.py`, `pdm_data/features.py`
- Test: `tests/test_pdm_features.py`

**Interfaces:**
- Consumes: `make_row`, `LapContext`, 메타데이터의 `tls_programs`, `ego_dims`
- Produces:
  - `pair_metrics(gap: float, v_follow: float, v_lead: float) -> dict`. 키는 `thw, ttc, inv_ttc, drac`다. closing = v_follow − v_lead.
    - `thw = gap / v_follow`(v_follow < 0.1이면 None)
    - `ttc = gap / closing`(closing > 0일 때만, 아니면 None)
    - `inv_ttc = max(0, closing) / max(gap, 0.1)`
    - `drac = closing² / (2·max(gap, 0.1))`(closing > 0, 아니면 0.0)
  - `bumper_gap(slot: dict, kind: str, ego_min_gap: float, follower_min_gap: float | None) -> float`: 선행은 `gap_raw + ego_min_gap`, 후행은 `gap_raw + follower_min_gap`
  - `row_interaction(row, ctx, ego_dims) -> dict`(스펙 11.3)
    - 슬롯마다 `<slot>.thw/ttc/inv_ttc/drac/margin`(margin = 범퍼 간격 − secure_gap)
    - 50 m 안 보행자마다 `{"id", "t_ego", "t_ped", "dt"}`
      - `t_ego` = ego에서 그 보행자 종방향 위치까지 / max(v_ego, 0.1)
      - `t_ped` = 보행자와 ego 차로 중심선 사이 횡거리 / max(v_ped, 0.1)
      - 둘 다 ego 차체 좌표 기준이다.
    - `n_active` = A1 + S5 + S6 상대 수
  - `time_to_green_red(phases: list[dict], phase: int, time_to_switch: float, link_index: int) -> tuple[float, float]`
    - 현재 phase에서 시작해 순환하며 duration을 더한다(현재 phase는 time_to_switch만 남았다고 본다).
    - ttg: link 문자가 `G`/`g`인 첫 phase까지 남은 시간(지금 녹색이면 0)
    - ttr: `r`/`R`인 첫 phase까지(지금 적색이면 0)
    - 한 바퀴 안에 없으면 `inf`
  - `stage(row, prev_stage: str | None, since_dep_s: float) -> str`: `"dwell"`(stop_state의 1과 16 비트가 모두 켜짐) / `"depart"`(dwell 뒤 10 s 안이고 정류장 끝에서 30 m 안) / `"approach"`(다음 서는 정류장까지 노선 거리 ≤ 100 m) / `"cruise"`
  - `FEATURE_NAMES: list[str]`와 `input_design_features(row, prev_row, ctx, programs, ego_dims, prev_label: tuple[float, int] | None, stage_state: dict) -> np.ndarray`(길이 = `len(FEATURE_NAMES)`, [-1,1])
    - 정규화: 전방 160 m, 후방 80 m, 속도 ctx.vmax, 가속 3.0 m/s², 정차 60 s, 신호 120 s. 빈 슬롯은 전방 +1·후방 −1, 속도 0, presence 0이다(Huegle·presence 관례).
    - 묶음과 순서는 스펙 11.2 표 순서다: ego(7) · lane-change(2) · route(7) · stop(7) · tls(8) · lane-slot(6×6) · safety(5) · foes(3×6) · peds(3×10). 묶음 안 항목은 `SUMO_AV_input_설계.md`의 권장 표 순서를 따른다.
    - dilemma 특징은 `d/max(v,0.1) − ttr`와 `v²/(2·3.0) − d`다.
    - phantom 슬롯은 foes와 peds의 빈 칸을 (present=0, phantom=1, 전방 끝 거리)로 채운다.

- [ ] **Step 1: 실패하는 시험을 쓴다.**

```python
def test_pair_metrics_closing():
    m = pair_metrics(20.0, 10.0, 5.0)
    assert m["thw"] == 2.0 and m["ttc"] == 4.0 and m["inv_ttc"] == 0.25 and m["drac"] == pytest.approx(0.625)

def test_pair_metrics_opening_has_no_ttc():
    m = pair_metrics(20.0, 5.0, 10.0)
    assert m["ttc"] is None and m["inv_ttc"] == 0.0 and m["drac"] == 0.0

def test_bumper_gap_adds_min_gap():
    assert bumper_gap({"gap_raw": 9.0}, "leader", 1.0, None) == 10.0

def test_time_to_green_red_cycle():
    phases = [{"duration": 30, "state": "Gr"}, {"duration": 3, "state": "yr"}, {"duration": 30, "state": "rG"}]
    assert time_to_green_red(phases, 2, 10.0, 0) == (10.0, 0.0)
    assert time_to_green_red(phases, 0, 5.0, 0) == (0.0, 8.0)

def test_stage_rules():
    assert stage(make_row(stop_state=1 | 16), None, 0.0) == "dwell"
    assert stage(make_row(next_stop_distance=80.0), "cruise", 99.0) == "approach"

def test_features_length_and_range():
    f = input_design_features(make_row(), None, CTX, PROGRAMS, DIMS, None, {})
    assert f.shape == (len(FEATURE_NAMES),) and np.all(np.isfinite(f)) and np.all(np.abs(f) <= 1.0)
    assert len(FEATURE_NAMES) == len(set(FEATURE_NAMES)) and 100 <= len(FEATURE_NAMES) <= 130
```

- [ ] **Step 2: 시험이 실패하는지 확인한다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_features.py -v` → FAIL
- [ ] **Step 3: 구현한다.**
- [ ] **Step 4: 시험이 통과하는지 확인한다.** → PASS
- [ ] **Step 5: 커밋한다.** `git add pdm_data/interaction.py pdm_data/features.py tests/test_pdm_features.py && git commit -m "feat(pdm_data): 상호작용 지표와 입력 설계 특징"`

---

### Task 9: 전처리 스크립트(transition, 사건, 요약, 필수 열 검사)

**Files:**
- Create: `pdm_data/transitions.py`, `scripts/preprocess_pdm_bus.py`
- Test: `tests/test_pdm_preprocess.py`

**Interfaces:**
- Consumes: Task 1~8 전부(`load_lap_context`, `read_steps`, `replay_episode`, `obs93`, labels, features, `parse_ssm`)
- Produces:
  - `build_transitions(run_dir: str, a_pos: float, a_neg: float, cut_on_task_failure: bool = True, max_lap_s: float | None = None) -> tuple[list[dict], dict]`: (transition 목록, 처리 통계).
    - transition의 키는 `data_colum.md` 23열 + 추가 열(스펙 11.1 끝: `sim_time, detected_pedestrians, event.termination_reason, comfort.*, expert.*, reward_terms.*, scenario.*, interaction.*, features, action_extra.accel_mps2`)이다. 점(.) 이름은 중첩 dict다(`data_schema.lookup` 규칙).
    - `max_lap_s`가 주어지면 그 시간을 넘는 행부터 잘라 truncated(`"time_limit"`)로 둔다.
    - 처리 통계: 행 수, 잘린 비율(accel), 종료 원인별 수, 버린 행 수
  - `scripts/preprocess_pdm_bus.py` CLI:
    - `run_dir`(위치 인자), `--accel-pos`, `--accel-neg`, `--report-accel`, `--keep-after-task-failure`, `--max-lap-s`
    - `--report-accel`이면 `processed/accel_report.json`만 쓰고 끝낸다(후보 `(1.0,2.0) (1.5,3.0) (2.0,4.0)`).
    - 그렇지 않은데 두 상수가 없으면 stderr에 "--accel-pos와 --accel-neg가 필요합니다(먼저 --report-accel로 분포를 보세요)"를 쓰고 종료 코드 2로 끝낸다.
    - 출력:
      - `processed/transitions.jsonl.gz`
      - `processed/transitions.npz`: `observation`, `next_observation`, `action_raw`, `reward`, `terminated`, `truncated`, `episode`, `t`, `features`, `next_features`
      - `processed/events.jsonl`: 원 events에 `service`/`missed_stop`/`red_violation`(replay)과 `ssm_conflict`(parse_ssm)를 더한다.
      - `processed/summary.json`: 바퀴별 시나리오 사건 수, 상호작용 지표 분포(p5·p50·p95), SSM 상충 수, 승차감 위반 수, 정차 완료 수, 보상 합
      - `processed/metadata.json`: 상수, 잘림 비율, `FEATURE_NAMES`, 93차원 레이아웃(`env.bus_task.LAYOUT`), 열 목록과 설명, 원 실행의 `schema_version`·`net_sha256`

- [ ] **Step 1: 실패하는 시험을 쓴다.** 고정 데이터로 `make_row`를 이어 붙인 작은 실행 폴더(행 12개, 마지막 행 `final, end="lap"`, 정류장 4곳 정차 replay가 나오게)를 `tmp_path`에 쓰는 `make_run_dir(tmp_path)`를 `tests/pdm_fixtures.py`에 더한다.

```python
REQUIRED = [line.split("|")[1].strip() for line in open("scripts/data_colum.md", encoding="utf-8")
            if line.startswith("| ") and "포함 여부" not in line and "---" not in line]

def test_required_columns_present(tmp_path):
    tr, _ = build_transitions(make_run_dir(tmp_path), 1.5, 3.0)
    assert len(REQUIRED) == 23
    for rec in tr:
        for col in REQUIRED:
            lookup(rec, col)                      # KeyError면 실패
        assert len(rec["observation"]) == 93 and len(rec["action_raw"]) == 2

def test_next_observation_is_next_row(tmp_path):
    tr, _ = build_transitions(make_run_dir(tmp_path), 1.5, 3.0)
    assert all(a["next_observation"] == b["observation"] for a, b in zip(tr, tr[1:]) if a["episode"] == b["episode"])

def test_episode_t_contiguous(tmp_path):
    tr, _ = build_transitions(make_run_dir(tmp_path), 1.5, 3.0)
    assert [r["t"] for r in tr] == list(range(len(tr)))

def test_missing_scale_exits_2(tmp_path):
    p = subprocess.run([sys.executable, "scripts/preprocess_pdm_bus.py", make_run_dir(tmp_path)], capture_output=True, text=True)
    assert p.returncode == 2 and "--report-accel" in p.stderr

def test_report_accel_writes_only_report(tmp_path):
    d = make_run_dir(tmp_path)
    subprocess.run([sys.executable, "scripts/preprocess_pdm_bus.py", d, "--report-accel"], check=True)
    assert os.path.exists(os.path.join(d, "processed/accel_report.json"))
    assert not os.path.exists(os.path.join(d, "processed/transitions.jsonl.gz"))
```

- [ ] **Step 2: 시험이 실패하는지 확인한다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_preprocess.py -v` → FAIL
- [ ] **Step 3: 구현한다.**
- [ ] **Step 4: 시험이 통과하는지 확인한다.** → PASS
- [ ] **Step 5: 전체 단위 시험을 돌린다.** Run: `/c/Users/user/anaconda3/envs/main/python.exe -m pytest tests/test_pdm_lap_context.py tests/test_pdm_snapshot.py tests/test_pdm_ssm.py tests/test_pdm_recorder.py tests/test_pdm_obs93.py tests/test_pdm_labels.py tests/test_pdm_features.py tests/test_pdm_preprocess.py tests/test_pdm_bus.py -q` → 모두 PASS
- [ ] **Step 6: 짧은 실제 확인을 한 번 한다(긴 실행 금지).**
  1. Run: `/c/Users/user/anaconda3/envs/main/python.exe scripts/collect_pdm_bus.py --warmup 30 --max-seconds 120 --check-obs 200 --name smoke`
  2. Run: `/c/Users/user/anaconda3/envs/main/python.exe scripts/preprocess_pdm_bus.py data/pdm_bus/smoke --report-accel`
  3. Run: `/c/Users/user/anaconda3/envs/main/python.exe scripts/preprocess_pdm_bus.py data/pdm_bus/smoke --accel-pos 1.5 --accel-neg 3.0`
  4. 기대 결과: 두 번째 명령은 `accel_report.json`을 만든다. 세 번째 명령은 `processed/` 파일 다섯 개를 만든다. 온라인과 오프라인 93차원 비교(200행)의 최대 오차가 1e-4 미만이라고 출력한다. 전처리 스크립트는 `check.obs93_online`이 있는 행을 만나면 이 비교를 출력한다.
  5. 결과 숫자를 보고에 적는다. 한 바퀴 전체 실행은 하지 않았다고 적는다.
- [ ] **Step 7: 커밋한다.** `git add pdm_data/transitions.py scripts/preprocess_pdm_bus.py tests/test_pdm_preprocess.py tests/pdm_fixtures.py && git commit -m "feat(scripts): PDM 버스 데이터 전처리(transition, 사건, 요약, 필수 열 검사)"`
