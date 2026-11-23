"""수집한 데이터에서 이상한 에피소드를 찾아 뺀다. 저장된 JSONL(state, event)만 읽는다.

에피소드 단위로 거른다. 한 에피소드 안의 행은 이어진 주행이라 일부만 빼면 다음 관측과 보상이 어긋난다.

거르는 기준은 셋으로 나눈다.
  버스 실패  완주 못 함, 차·보행자 충돌, 적색 통과, 순간이동, 정류장 놓침
  시뮬레이션 이상  버스 주변(state에 기록된 범위)의 배경 충돌, 보행자·차량 위치 점프, 버스 차체와 겹친 차나 보행자
  어색한 장면(--strict일 때만 뺌)  버스 50m 안에서 차로 가운데를 벗어나 10초 넘게 선 차, 6m/s² 넘는 급제동

    python research/av_bc_rl/filter_episodes.py data/research/v4/demo_busy.jsonl.gz --report out.json
    python research/av_bc_rl/filter_episodes.py a.jsonl.gz b.jsonl.gz --strict --write-clean data/research/v4/clean
"""
import argparse
import gzip
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

DT = 0.5
EGO_SIZE, CAR_SIZE = (12.0, 2.5), (5.0, 1.8)   # 길이, 폭(m). state의 x, y는 앞범퍼 가운데다
SHRINK = 0.3            # m. 상자를 이만큼 줄여 겹침을 본다(스치는 정도는 세지 않는다)
PED_JUMP, VEH_JUMP = 2.0, 3.0   # m. 속도 × 시간보다 이만큼 넘게 움직이면 점프
NEAR = 50.0             # m. 어색한 장면을 세는 버스 주변 거리
OFF_CENTRE, OFF_CENTRE_S = 0.5, 10.0
EMERGENCY = 6.0         # m/s²
FAIL = ("not_arrived", "ego_collision", "pedestrian_collision", "red_light", "teleport", "stop_missed")
ARTIFACT = ("background_collision", "pedestrian_jump", "vehicle_jump", "vehicle_in_ego", "pedestrian_in_ego")
AWKWARD = ("off_centre_standing", "emergency_braking_near")


def box(x, y, heading, size, shrink=SHRINK):
    """앞범퍼 가운데 (x, y)와 SUMO 방향(북쪽 0도, 시계 방향)으로 차체 네 모서리를 만든다."""
    length, width = size[0] - 2 * shrink, size[1] - 2 * shrink
    a = math.radians(90.0 - heading)
    fx, fy = math.cos(a), math.sin(a)
    x, y = x - fx * shrink, y - fy * shrink
    lx, ly = -fy * width / 2, fx * width / 2
    return [(x + lx, y + ly), (x - lx, y - ly), (x - fx * length - lx, y - fy * length - ly),
            (x - fx * length + lx, y - fy * length + ly)]


def overlap(a, b):
    """볼록 사각형 둘이 겹치는지(분리축)."""
    for poly in (a, b):
        for i in range(4):
            ex, ey = poly[(i + 1) % 4][0] - poly[i][0], poly[(i + 1) % 4][1] - poly[i][1]
            pa = [p[0] * -ey + p[1] * ex for p in a]
            pb = [p[0] * -ey + p[1] * ex for p in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def inside(point, poly):
    sign = None
    for i in range(4):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % 4]
        cross = (x2 - x1) * (point[1] - y1) - (y2 - y1) * (point[0] - x1)
        if sign is None:
            sign = cross > 0
        elif (cross > 0) != sign:
            return False
    return True


def episode_flags(rows):
    flags = Counter()
    last = rows[-1]["event"]
    if last.get("termination_reason") != "arrived":
        flags["not_arrived"] += 1
    previous_v, previous_p, off = {}, {}, defaultdict(float)
    counted_off = set()
    for row in rows:
        event, state = row["event"], row["state"]
        flags["ego_collision"] += bool(event.get("collided")) and not event.get("pedestrian_collision")
        flags["pedestrian_collision"] += bool(event.get("pedestrian_collision"))
        flags["red_light"] += bool(event.get("red_light_violation"))
        flags["teleport"] += bool(event.get("teleporting_ids"))
        flags["background_collision"] += int(event.get("background_collisions") or 0)
        ego = state["ego"]
        if "x" not in ego:
            continue
        ego_box = box(ego["x"], ego["y"], ego["heading"], EGO_SIZE)
        now_v, now_p = {}, {}
        for v in state["vehicles"]:
            now_v[v["id"]] = v
            distance = math.dist((v["x"], v["y"]), (ego["x"], ego["y"]))
            old = previous_v.get(v["id"])
            if old is not None:
                moved = math.dist((old["x"], old["y"]), (v["x"], v["y"]))
                if moved - max(old["speed"], v["speed"]) * DT > VEH_JUMP:
                    flags["vehicle_jump"] += 1
                if distance < NEAR and (old["speed"] - v["speed"]) / DT > EMERGENCY:
                    flags["emergency_braking_near"] += 1
            if distance < 20.0 and overlap(ego_box, box(v["x"], v["y"], v["heading"], CAR_SIZE)):
                flags["vehicle_in_ego"] += 1
            standing = v["speed"] < 0.5 and abs(v.get("lateral_position", 0.0)) > OFF_CENTRE and distance < NEAR
            off[v["id"]] = off[v["id"]] + DT if standing else 0.0
            if off[v["id"]] >= OFF_CENTRE_S and v["id"] not in counted_off:
                counted_off.add(v["id"])
                flags["off_centre_standing"] += 1
        for p in state["pedestrians"]:
            now_p[p["id"]] = p
            old = previous_p.get(p["id"])
            if old is not None and math.dist((old["x"], old["y"]), (p["x"], p["y"])) - max(old["speed"], p["speed"]) * DT > PED_JUMP:
                flags["pedestrian_jump"] += 1
            if inside((p["x"], p["y"]), ego_box):
                flags["pedestrian_in_ego"] += 1
        previous_v, previous_p = now_v, now_p
    if int(last.get("stops_missed") or 0) > 0:
        flags["stop_missed"] += int(last["stops_missed"])
    return {key: int(value) for key, value in flags.items() if value}


def read_episodes(path):
    rows, current = [], None
    with gzip.open(path, "rt", encoding="utf-8") as f:
        for line in f:
            row = json.loads(line)
            if current is not None and row["episode"] != current:
                yield rows
                rows = []
            current = row["episode"]
            rows.append(row)
    if rows:
        yield rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("jsonl", nargs="+")
    parser.add_argument("--strict", action="store_true", help="어색한 장면이 있는 에피소드도 뺀다")
    parser.add_argument("--report", default=None, help="에피소드별 표시를 JSON으로 쓴다")
    parser.add_argument("--write-clean", default=None, help="남긴 에피소드만 <폴더>/<이름>.jsonl.gz로 다시 쓴다")
    args = parser.parse_args()
    drop_on = FAIL + ARTIFACT + (AWKWARD if args.strict else ())
    report, totals, kept_rows, all_rows = {}, Counter(), 0, 0
    for path in args.jsonl:
        out = None
        if args.write_clean:
            Path(args.write_clean).mkdir(parents=True, exist_ok=True)
            out = gzip.open(Path(args.write_clean) / Path(path).name, "wt", encoding="utf-8")
        episodes = {}
        for rows in read_episodes(path):
            flags = episode_flags(rows)
            keep = not any(flags.get(key) for key in drop_on)
            episodes[int(rows[0]["episode"])] = {"rows": len(rows), "keep": keep, "flags": flags}
            all_rows += len(rows)
            for key in flags:
                totals[key] += 1
            if keep:
                kept_rows += len(rows)
                if out:
                    for row in rows:
                        out.write(json.dumps(row, ensure_ascii=False) + "\n")
        if out:
            out.close()
        report[path] = episodes
        kept = sum(e["keep"] for e in episodes.values())
        print(f"{path}: 에피소드 {len(episodes)}개 중 {kept}개 남김")
    print(f"행 {all_rows}개 중 {kept_rows}개 남김({100 * kept_rows / max(all_rows, 1):.1f}%). 표시가 붙은 에피소드 수: {dict(totals)}")
    if args.report:
        Path(args.report).write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
