"""PDM-Closed(tuPlan garage) 기반 버스 종방향 주행. 기존 `TestBusPilot`은 그대로 두고 상속해 덧붙인다.

PDM-Closed(autonomousvision/tuplan_garage)의 구조를 SUMO에 맞게 옮겼다.
  1. 제안 생성: 속도 비율 {0.2, 0.4, 0.6, 0.8, 1.0} x 제한속도를 목표 속도 v0로 하는 IDM 전개(4 s, 0.1 s 간격).
     IDM 매개변수는 PDM-Closed 설정값(s0 1 m, T 1.5 s, a 1.5, b 3.0, 지수 10)이다.
  2. 예측: 주변 물체는 등속 직선 운동으로 4 s 내다본다. 앞차는 같은 차로 선행차(SUMO getLeader)와 빨간 신호(정지 물체)다.
  3. 점수: mult(충돌 없음) x (5 progress + 5 TTC + 2 comfort) / 12. progress는 제안 중 최댓값으로 나눈다.
  4. 선택: 점수가 가장 높은 제안(동점이면 낮은 속도 비율)의 속도 비율을 쓴다.
  5. 긴급 제동: 최선 제안도 2 s 안에 과실 충돌이고 속도가 5 m/s 이하이면 PDM의 긴급 제동식으로 멈춘다.

SUMO 기본 알고리즘 활용: PDM-Closed의 추종 모델이 IDM이고 SUMO에 IDM이 있으므로, 버스 유형을 SUMO IDM(`BUS_PDM_VTYPE`)으로 두고
실제 가감속은 SUMO IDM이 한다. 이 모듈은 PDM이 고른 속도 비율을 `setMaxSpeed`(= IDM의 v0)로만 전한다.
신호·정류장·충돌 회피 같은 SUMO 안전 규칙은 그대로 위에 남는다.

옮기지 않은 것: 횡방향 제안(±1 m). 이 시뮬레이션은 차로 단위라 차선 변경은 기존 규칙(`LanePlan`)이 맡는다.
DRIVABLE_AREA·DRIVING_DIRECTION은 차로 중심선 주행이라 항상 1이다. LQR 추종기와 자전거 모델은 SUMO가 대신한다.
"""
import math
from dataclasses import dataclass

import numpy as np

from .test_bus import BUS, TestBusPilot

BUS_PDM_VTYPE = ('<vType id="bus_test" vClass="bus" length="12" width="2.5" carFollowModel="IDM" accel="1.5" decel="3.0" '
                 'emergencyDecel="9.0" tau="1.5" minGap="1.0" delta="10" guiShape="bus" color="255,0,0"/>')


@dataclass
class PDMCfg:
    horizon: int = 40                      # 제안 길이(스텝). PDM-Closed: 40
    dt: float = 0.1
    fractions: tuple = (0.2, 0.4, 0.6, 0.8, 1.0)
    fallback_speed: float = 15.0           # 제한속도를 모를 때(PDM: fallback_target_velocity)
    s0: float = 1.0                        # 최소 간격 [m]
    headway: float = 1.5                   # T [s]
    accel: float = 1.5
    decel: float = 3.0
    exponent: float = 10.0
    ttc_offsets: tuple = (0.0, 0.3, 0.6, 0.9)
    w_progress: float = 5.0
    w_ttc: float = 5.0
    w_comfort: float = 2.0
    lon_accel: tuple = (-4.05, 2.40)       # 편안함 한계(PDM 기본값)
    lat_accel: float = 4.89
    lon_jerk: float = 4.13
    yaw_rate: float = 0.95
    brake_ttc_s: float = 2.0               # 긴급 제동: 최선 제안의 과실 충돌까지 시간 / 속도 한계
    brake_speed: float = 5.0
    radius: float = 50.0                   # 주변 물체 범위 [m]
    lookahead: float = 90.0                # 중심선 길이 [m]
    ego_length: float = 12.0
    ego_width: float = 2.5
    replan_steps: int = 2                  # 몇 스텝마다 다시 계획하나(2 = 0.2 s). 1이면 PDM 원래처럼 매 스텝
    margin: float = 0.1                    # 충돌 판정 여유(원 반지름에 더한다) [m]


# ---- 순수 계산(SUMO 없이 시험 가능) -------------------------------------------------------------------
def idm_rollout_batch(v_ego, v0, gap, v_lead, cfg=None):
    """IDM 제안 여러 개(목표 속도 v0 배열)를 한꺼번에 전개한다. gap은 선행차까지 범퍼 간격(없으면 None), v_lead는 그 속도(등속 가정).
    반환: 진행 거리 s (P,H+1), 속도 v (P,H+1), 가속도 acc (P,H)."""
    c = cfg or PDMCfg()
    H, dt = c.horizon, c.dt
    v0 = np.maximum(np.asarray(v0, dtype=float), 0.1)
    P = len(v0)
    s, v, acc = np.zeros((P, H + 1)), np.zeros((P, H + 1)), np.zeros((P, H))
    v[:, 0] = v_ego
    inv = 1.0 / (2.0 * math.sqrt(c.accel * c.decel))
    for k in range(H):
        a = 1.0 - (v[:, k] / v0) ** c.exponent
        if gap is not None:
            g = np.maximum(gap + v_lead * k * dt - s[:, k], c.s0)
            s_star = c.s0 + v[:, k] * c.headway + v[:, k] * (v[:, k] - v_lead) * inv
            a = a - (s_star / g) ** 2
        a = np.clip(c.accel * a, -c.decel, c.accel)
        acc[:, k] = a
        v[:, k + 1] = np.maximum(0.0, v[:, k] + a * dt)                 # 후진은 없다
        s[:, k + 1] = s[:, k] + 0.5 * (v[:, k] + v[:, k + 1]) * dt
    return s, v, acc


def idm_rollout(v_ego, v0, gap, v_lead, cfg=None, a_ego=0.0):
    """IDM 제안 하나. 반환: s[0..H], v[0..H], acc[0..H-1]."""
    s, v, acc = idm_rollout_batch(v_ego, [v0], gap, v_lead, cfg)
    return s[0], v[0], acc[0]


class Centerline:
    """차로 중심선(점열). s는 시작점(버스 앞 범퍼)에서 잰 거리. 시작 앞쪽(s<0)은 처음 방향으로 연장한다."""

    def __init__(self, pts):
        p = np.asarray(pts, dtype=float)
        keep = np.r_[True, np.hypot(*np.diff(p, axis=0).T) > 1e-6] if len(p) > 1 else np.array([True])
        self.p = p[keep]
        seg = np.hypot(*np.diff(self.p, axis=0).T) if len(self.p) > 1 else np.zeros(0)
        self.s = np.r_[0.0, np.cumsum(seg)]
        self.length = float(self.s[-1])
        d = np.diff(self.p, axis=0) if len(self.p) > 1 else np.array([[0.0, 1.0]])
        self.dir = d / np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-9)[:, None]

    def pose(self, s):
        """s(배열)에서 (xy, 단위 방향). 범위 밖은 양 끝 방향으로 연장한다."""
        s = np.asarray(s, dtype=float)
        idx = np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.dir) - 1)
        base = self.p[idx] + self.dir[idx] * (s - self.s[idx])[:, None] if s.ndim else None
        return base, self.dir[idx]

    def curvature(self, s):
        """s에서 곡률 근사(방향 변화 / 거리)."""
        s = np.asarray(s, dtype=float)
        idx = np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.dir) - 1)
        nxt = np.minimum(idx + 1, len(self.dir) - 1)
        prv = np.maximum(idx - 1, 0)
        a1, a0 = np.arctan2(self.dir[nxt, 1], self.dir[nxt, 0]), np.arctan2(self.dir[prv, 1], self.dir[prv, 0])
        dang = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
        ds = np.maximum(self.s[np.minimum(nxt + 1, len(self.s) - 1)] - self.s[prv], 1.0)
        return dang / ds


def _circles(center, direction, length, width, margin):
    """몸체를 지나는 원들의 중심. center (...,2), direction (...,2) -> (..., n, 2). 반지름은 width/2 + margin."""
    n = max(1, int(math.ceil(length / max(width, 0.1))))
    off = (np.arange(n) + 0.5) / n * length - length / 2.0
    return center[..., None, :] + direction[..., None, :] * off[:, None], width / 2.0 + margin


def score_proposals(cl, v_ego, v_limit, lead, objects, cfg=None, a_ego=0.0):
    """모든 속도 비율 제안을 점수 매긴다.

    cl: Centerline, lead: (gap, v) 또는 None, objects: 딕셔너리 목록(xy, dir(단위), v, length, width).
    반환: dict(best 인덱스, scores, mult, progress, ttc, comfort, collide_t(과실 충돌 시각 또는 None), s, v)."""
    c = cfg or PDMCfg()
    H, nP = c.horizon, len(c.fractions)
    S, V, ACC = idm_rollout_batch(v_ego, np.array(c.fractions) * v_limit, None if lead is None else lead[0],
                                  0.0 if lead is None else lead[1], c)
    t_idx = np.arange(H + 1)
    xy, dr = cl.pose(S.ravel())
    xy, dr = xy.reshape(nP, H + 1, 2), dr.reshape(nP, H + 1, 2)
    # 몸체의 중심은 앞 범퍼에서 length/2 뒤
    ego_c = xy - dr * (c.ego_length / 2.0)
    mult = np.ones(nP)
    ttc = np.ones(nP)
    first_hit = np.full(nP, np.inf)
    if objects:
        o_xy = np.array([o["xy"] for o in objects], dtype=float)            # (N,2) 몸체 중심
        o_dr = np.array([o["dir"] for o in objects], dtype=float)
        o_v = np.array([o["v"] for o in objects], dtype=float)
        o_len = np.array([o["length"] for o in objects], dtype=float)
        o_wid = np.array([o["width"] for o in objects], dtype=float)
        tt = t_idx * c.dt
        o_pos = o_xy[None] + o_dr[None] * (o_v[None, :, None] * tt[:, None, None])   # (T,N,2) 등속 직선
        d0, _ = cl.pose(np.array([0.0]))
        ego_dir0 = cl.dir[0]
        ego0 = d0[0] - ego_dir0 * (c.ego_length / 2.0)
        behind = ((o_xy - ego0) @ ego_dir0) < -(c.ego_length / 2.0)           # 처음부터 뒤에 있는 물체(추돌은 과실이 아니다)
        keep = ~behind
        # 넓은 거름: 4초 동안 한 번도 경로 근처(몸체 반치수 합 + 여유)에 오지 않는 물체는 뺀다. 시각은 무시해 보수적이다
        path_pts = cl.pose(np.linspace(-c.ego_length, float(S.max()), 24))[0]
        d_min = np.hypot(*(path_pts[:, None, None, :] - o_pos[None, ::10]).transpose(3, 0, 1, 2)).min(axis=(0, 1))
        keep &= d_min < (c.ego_length + o_len) / 2.0 + (c.ego_width + o_wid) / 2.0 + 3.0
        if keep.any():
            sel = np.flatnonzero(keep)
            o_pos, o_dr, o_len, o_wid, keep = o_pos[:, sel], o_dr[sel], o_len[sel], o_wid[sel], keep[sel]
            ec, er = _circles(ego_c, dr, c.ego_length, c.ego_width, c.margin)          # (P,T+1,Ne,2)
            oc, orad = _obj_circles(o_pos, o_dr, o_len, o_wid, c.margin)               # (T+1,N,Nm,2), (N,Nm)
            # 충돌: 시각마다 같은 시각의 물체와 몸체 원이 겹치는가 (P,T+1,Ne,N,Nm)
            hit = _overlap(ec, er, oc, orad, keep[None, None, :])                       # (P,T+1)
            first_hit = np.where(hit.any(axis=1), hit.argmax(axis=1).astype(float), np.inf)
            # TTC: 앞 범퍼 기준 박스를 등속 외삽(0, 0.3, 0.6, 0.9 s)한 뒤 그 시각의 물체와 겹치면 위반
            moving = V > 5e-3
            for off in c.ttc_offsets:
                k = int(round(off / c.dt))
                n = H + 1 - k                                                           # t + k <= H 인 t만
                ep = ego_c[:, :n] + dr[:, :n] * (V[:, :n, None] * off)
                ec2, er2 = _circles(ep, dr[:, :n], c.ego_length, c.ego_width, c.margin)
                ahead = (((o_pos[None, k:k + n] - ep[:, :, None, :]) * dr[:, :n, None, :]).sum(-1)) > 0   # (P,n,N)
                bad = _overlap(ec2, er2, oc[k:k + n], orad, keep[None, None, :] & ahead) & moving[:, :n]
                ttc = np.where(bad.any(axis=1), 0.0, ttc)
        mult = np.where(np.isfinite(first_hit), 0.0, 1.0)
    # 편안함: 종방향 가속·저크, 횡가속·요레이트(곡률에서)
    kappa = cl.curvature(S[:, 1:])
    jerk = np.diff(np.concatenate([np.full((nP, 1), a_ego), ACC], axis=1), axis=1) / c.dt
    ok = ((ACC > c.lon_accel[0]) & (ACC < c.lon_accel[1])).all(axis=1) & (np.abs(jerk) < c.lon_jerk).all(axis=1) \
        & (np.abs(V[:, 1:] ** 2 * kappa) < c.lat_accel).all(axis=1) & (np.abs(V[:, 1:] * kappa) < c.yaw_rate).all(axis=1)
    comfort = ok.astype(float)
    prog = np.maximum(0.0, S[:, -1]) * mult
    prog = prog / prog.max() if prog.max() > 0.1 else (mult != 0).astype(float)
    total = mult * (c.w_progress * prog + c.w_ttc * ttc + c.w_comfort * comfort) / (c.w_progress + c.w_ttc + c.w_comfort)
    best = int(np.argmax(total))                                           # 동점이면 낮은 속도 비율(PDM과 같다)
    hit_t = None if not np.isfinite(first_hit[best]) else float(first_hit[best] * c.dt)
    return {"best": best, "scores": total, "mult": mult, "progress": prog, "ttc": ttc, "comfort": comfort,
            "collide_t": hit_t, "s": S, "v": V}


def _obj_circles(o_pos, o_dr, o_len, o_wid, margin):
    """모든 물체·시각의 몸체 원. o_pos (T,N,2) -> 중심 (T,N,Nm,2), 반지름 (N,Nm). 원이 모자란 물체의 빈 칸은 NaN(겹침 거짓)."""
    n = np.maximum(1, np.ceil(o_len / np.maximum(o_wid, 0.1)).astype(int))
    nm = int(n.max())
    k = np.arange(nm)[None, :]
    off = (k + 0.5) / n[:, None] * o_len[:, None] - o_len[:, None] / 2.0
    off = np.where(k < n[:, None], off, np.nan)                                       # (N,Nm)
    centers = o_pos[:, :, None, :] + o_dr[None, :, None, :] * off[None, :, :, None]
    rad = np.broadcast_to((o_wid / 2.0 + margin)[:, None], off.shape)
    return centers, rad


def _overlap(ec, er, oc, orad, mask):
    """ec (P,T,Ne,2) 자기 원 / oc (T,N,Nm,2) 물체 원 -> (P,T): 마스크된 물체와 한 원이라도 겹치는가."""
    dx = ec[:, :, :, None, None, 0] - oc[None, :, None, :, :, 0]
    dy = ec[:, :, :, None, None, 1] - oc[None, :, None, :, :, 1]
    r = er + orad[None, None, None]
    hit = (dx * dx + dy * dy < r * r).any(axis=(2, 4))                                  # (P,T,N)
    return (hit & mask).any(axis=2)


def emergency_brake_speed(v, cfg=None):
    """PDM-Closed 긴급 제동의 이번 스텝 목표 속도. v>0.2: u = clip(10*(-v), -4.05, 2.40), 속도 += u*dt."""
    c = cfg or PDMCfg()
    if v > 0.2:
        u = float(np.clip(10.0 * (-v), c.lon_accel[0], c.lon_accel[1]))
        return max(0.0, v + u * c.dt)
    return 0.0


# ---- SUMO 연결 ----------------------------------------------------------------------------------
def _sub_vars():
    from traci import constants as k
    return [k.VAR_POSITION, k.VAR_ANGLE, k.VAR_SPEED, k.VAR_LENGTH, k.VAR_WIDTH], k


class PDMClosedBusPilot(TestBusPilot):
    """차선 변경·정류장·시나리오는 `TestBusPilot` 규칙 그대로. 종방향 속도 비율만 PDM-Closed 점수로 고른다."""

    def __init__(self, tc, net, loop, stops, stop_s=8.0, cfg=None):
        super().__init__(tc, net, loop, stops, stop_s=stop_s)
        self.cfg = cfg or PDMCfg()
        self._subscribed = False
        self.pdm = {"plans": 0, "brakes": 0, "fractions": {f: 0 for f in self.cfg.fractions}}
        self._last_frac = None
        self._shapes, self._lane_len, self._n = {}, {}, 0
        self.last_plan = None                                 # 마지막 계획 결과(데이터 기록용, 동작에는 쓰지 않는다)

    # 매 스텝: 기존 규칙(차선 변경 등) 다음에 PDM 종방향
    def tick(self, tc, t):
        super().tick(tc, t)
        if not self._added or BUS not in tc.vehicle.getIDList():
            return
        try:
            self._pdm_step(tc)
        except Exception as e:                                # 계획 실패가 시뮬레이션을 멈추면 안 된다: SUMO IDM에 맡긴다
            self.why = f"pdm 오류: {e}"

    def _subscribe(self, tc):
        ids, k = _sub_vars()
        tc.vehicle.subscribeContext(BUS, k.CMD_GET_VEHICLE_VARIABLE, self.cfg.radius, ids)
        try:
            tc.vehicle.subscribeContext(BUS, k.CMD_GET_PERSON_VARIABLE, self.cfg.radius, ids)
        except Exception:
            pass
        self._subscribed = True

    def _objects(self, tc):
        ids, k = _sub_vars()
        res = tc.vehicle.getContextSubscriptionResults(BUS) or {}
        objs = []
        for oid, r in res.items():
            if oid == BUS or k.VAR_POSITION not in r:
                continue
            ang = math.radians(r[k.VAR_ANGLE])
            d = np.array([math.sin(ang), math.cos(ang)])              # SUMO 각도: 북쪽 기준 시계 방향
            length, width = r.get(k.VAR_LENGTH, 4.5), r.get(k.VAR_WIDTH, 1.8)
            front = np.array(r[k.VAR_POSITION])
            objs.append({"id": oid, "xy": front - d * (length / 2.0), "dir": d, "v": r.get(k.VAR_SPEED, 0.0),
                         "length": length, "width": width})
        return objs

    def _shape(self, tc, lane):
        """차로 모양(캐시: 도로망은 변하지 않는다)."""
        if lane not in self._shapes:
            self._shapes[lane] = np.array(tc.lane.getShape(lane), dtype=float)
        return self._shapes[lane]

    def _lane_length(self, tc, lane):
        if lane not in self._lane_len:
            self._lane_len[lane] = tc.lane.getLength(lane)
        return self._lane_len[lane]

    def _centerline(self, tc, road):
        """버스 앞 범퍼부터 노선을 따라 lookahead m의 차로 중심선."""
        net, c = self.net, self.cfg
        lane = tc.vehicle.getLaneID(BUS)
        shape = self._shape(tc, lane)
        seg = np.hypot(*np.diff(shape, axis=0).T)
        total = seg.sum()
        pos = tc.vehicle.getLanePosition(BUS) * (total / max(self._lane_length(tc, lane), 1e-6))
        cum = np.r_[0.0, np.cumsum(seg)]
        k = int(np.clip(np.searchsorted(cum, pos, side="right") - 1, 0, len(seg) - 1))
        start = shape[k] + (shape[k + 1] - shape[k]) * ((pos - cum[k]) / max(seg[k], 1e-9))
        pts = [start, *shape[k + 1:]]
        length = total - pos
        route, i = list(tc.vehicle.getRoute(BUS)), tc.vehicle.getRouteIndex(BUS)
        idx, cur = tc.vehicle.getLaneIndex(BUS), road
        while length < c.lookahead and i + 1 < len(route):
            nxt = route[i + 1]
            if not (net.hasEdge(cur) and net.hasEdge(nxt)):
                break
            conns = net.getEdge(cur).getConnections(net.getEdge(nxt))
            if not conns:
                break
            conn = next((cc for cc in conns if cc.getFromLane().getIndex() == idx), None) \
                or min(conns, key=lambda cc: abs(cc.getFromLane().getIndex() - idx))
            via = conn.getViaLaneID()
            to_lane = f"{nxt}_{conn.getToLane().getIndex()}"
            for ln in ([via] if via else []) + [to_lane]:
                sh = self._shape(tc, ln)
                pts.extend(sh)
                length += float(np.hypot(*np.diff(sh, axis=0).T).sum())
            cur, idx, i = nxt, conn.getToLane().getIndex(), i + 1
        return Centerline(pts)

    def _lead(self, tc):
        """가장 가까운 선행 물체: SUMO 같은 경로 선행차, 빨간 신호(정지 물체). (범퍼 간격, 속도) 또는 None."""
        best = None
        ld = tc.vehicle.getLeader(BUS, self.cfg.lookahead)
        if ld and ld[0]:
            best = (float(ld[1]), float(tc.vehicle.getSpeed(ld[0])))
        for tls in tc.vehicle.getNextTLS(BUS):
            _, _, dist, state = tls
            if state in ("r", "R", "u") and dist < self.cfg.lookahead and (best is None or dist < best[0]):
                best = (float(dist), 0.0)
        return best

    def _pdm_step(self, tc):
        c = self.cfg
        road = tc.vehicle.getRoadID(BUS)
        if road.startswith(":"):
            return                                            # 교차로 안: 직전 속도 비율 유지
        if not self._subscribed:
            self._subscribe(tc)
        if tc.vehicle.getStopState(BUS) & 1:
            return                                            # 정류장 정차 중
        self._n += 1
        if self._n % c.replan_steps:
            return                                            # 재계획 사이에는 직전 결정을 유지한다
        v = tc.vehicle.getSpeed(BUS)
        vlim = tc.lane.getMaxSpeed(tc.vehicle.getLaneID(BUS))
        vlim = vlim if vlim > 0.1 else c.fallback_speed
        res = score_proposals(self._centerline(tc, road), v, vlim, self._lead(tc), self._objects(tc), c,
                              a_ego=tc.vehicle.getAcceleration(BUS))
        self.pdm["plans"] += 1
        if res["collide_t"] is not None and res["collide_t"] <= c.brake_ttc_s and v <= c.brake_speed \
                and self._forced_p is None:
            tc.vehicle.setSpeed(BUS, emergency_brake_speed(v, c))
            self.pdm["brakes"] += 1
            self._braking = True
        elif getattr(self, "_braking", False):
            if not self._limited:                              # 기존 규칙(비양보형·S5 감속)이 속도를 제한하는 중이면 그 제한을 지우지 않는다
                tc.vehicle.setSpeed(BUS, -1)
            self._braking = False
        frac = c.fractions[res["best"]]
        self.pdm["fractions"][frac] += 1
        self.last_plan = {"sim_time": tc.simulation.getTime(), "fraction": frac, "v0": frac * vlim,
                          "scores": [float(x) for x in res["scores"]], "mult": [float(x) for x in res["mult"]],
                          "progress": [float(x) for x in res["progress"]], "ttc": [float(x) for x in res["ttc"]],
                          "comfort": [float(x) for x in res["comfort"]], "collide_t": res["collide_t"],
                          "emergency_brake": bool(getattr(self, "_braking", False))}
        if self._last_frac != (frac, vlim):                    # 속도 비율이나 제한속도가 바뀔 때만 IDM의 v0를 갱신
            tc.vehicle.setMaxSpeed(BUS, frac * vlim)
            self._last_frac = (frac, vlim)

    def summary(self):
        n = max(1, self.pdm["plans"])
        fr = ", ".join(f"{f:.1f}:{100 * k / n:.0f}%" for f, k in self.pdm["fractions"].items())
        return f"계획 {self.pdm['plans']}회, 긴급 제동 {self.pdm['brakes']}회, 선택한 속도 비율 {fr}"
