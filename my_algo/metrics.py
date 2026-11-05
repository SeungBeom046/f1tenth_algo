"""회차 지표 계산 (ROS 비의존, numpy). analysis/extract_run.py가 bag에서 꺼낸 시계열로 호출한다.

위치 좌표 s: AUTO 시작부터 odom 주행 거리 [m] = 트랙 좌표 (scenario_*.yaml의 x 값과 같은 기준).
지표 (2.11절):
    loss_delta_t       절대 손실 ΔT = ∫(1/v − 1/v_ref) ds [s]. 명령 기준(v_raw)과 실측 기준(odom)을 따로 계산
    window_loss_pct    고정 창(주차열 ±1 m) 손실 % (보조)
    min_speed          시험 구간 최저 속도
    decel_length       v < 0.95 v_ref 인 구간 총 길이
    jerk_rms           IMU 종방향 가속도 → 2차 Butterworth 저역통과(JERK_CUTOFF_HZ, 양방향) → 미분 → RMS
    footprint_distance 장애물 점과 차체 다각형 최소 거리 → 최소 거리, 충돌 여부, 충돌 시 속도, 최소 TTC
    vref_reached       가속 구간 끝에서 v_ref 도달 여부 (미도달 회차는 폐기, 폐기 수 보고)
"""

import math

import numpy as np

from my_algo.geometry import point_polygon_distance

JERK_CUTOFF_HZ = 5.0     # [EXP-FIXED] jerk 저역통과 차단주파수. 방법 간 상대 비교 전용, 문서에 명시
V_FLOOR = 0.05           # 손실 적분에서 1/v 발산을 막는 하한 [m/s]. 이 값이 쓰이면 floored 플래그
DECEL_RATIO = 0.95       # [EXP-FIXED] 감속 구간 판정 v < 0.95 v_ref
VREF_REACH_RATIO = 0.95  # [EXP-FIXED] 가속 구간 끝 v_ref 도달 판정
DS = 0.01                # 위치 격자 간격 [m]


def travel_distance(x, y):
    """odom 위치 → 누적 주행 거리 s."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) == 0:
        return np.empty(0)
    return np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])


def series_on_s(t_s, s, t_v, v):
    """시간 시계열 v(t_v)를 위치 s로 옮긴다. 정지 중(s가 안 늘어남)은 그 위치의 최소값을 쓴다."""
    s_at_v = np.interp(t_v, t_s, s)
    order = np.argsort(s_at_v, kind='stable')
    s_sorted, v_sorted = s_at_v[order], np.asarray(v, float)[order]
    uniq, inv = np.unique(np.round(s_sorted / DS).astype(np.int64), return_inverse=True)
    v_min = np.full(len(uniq), np.inf)
    np.minimum.at(v_min, inv, v_sorted)
    return uniq * DS, v_min


def _grid(s, v, s0, s1):
    grid = np.arange(s0, s1, DS) + 0.5 * DS
    return grid, np.interp(grid, s, v)


def loss_delta_t(s, v, v_ref, s0, s1):
    """반환 (ΔT [s] 또는 None(구간 미완주), floored)."""
    s, v = np.asarray(s, float), np.asarray(v, float)
    if len(s) == 0 or s.max() < s1:
        return None, False
    _, vg = _grid(s, v, s0, s1)
    floored = bool((vg < V_FLOOR).any())
    vg = np.maximum(vg, V_FLOOR)
    return float(np.sum(1.0 / vg - 1.0 / v_ref) * DS), floored


def window_loss_pct(s, v, v_ref, center_start, center_end, margin=1.0):
    s0, s1 = center_start - margin, center_end + margin
    dt, _ = loss_delta_t(s, v, v_ref, s0, s1)
    return None if dt is None else 100.0 * dt / ((s1 - s0) / v_ref)


def min_speed(s, v, s0, s1):
    s, v = np.asarray(s, float), np.asarray(v, float)
    m = (s >= s0) & (s <= s1)
    return float(v[m].min()) if m.any() else None


def decel_length(s, v, v_ref, s0, s1, ratio=DECEL_RATIO):
    if len(s) == 0:
        return None
    _, vg = _grid(s, v, s0, min(s1, float(np.max(s))))
    return float(np.count_nonzero(vg < ratio * v_ref) * DS)


def vref_reached(s, v, accel_end, v_ref, ratio=VREF_REACH_RATIO):
    s, v = np.asarray(s, float), np.asarray(v, float)
    if len(s) == 0 or s.max() < accel_end:
        return False
    return bool(np.interp(accel_end, s, v) >= ratio * v_ref)


def butter2_lowpass(x, fs, fc):
    """2차 Butterworth 저역통과, 앞뒤 두 번 적용 (위상 지연 없음)."""
    x = np.asarray(x, float)
    if len(x) < 4:
        return x.copy()
    k = math.tan(math.pi * fc / fs)
    q = math.sqrt(2.0)
    norm = 1.0 / (1.0 + q * k + k * k)
    b = np.array([k * k, 2 * k * k, k * k]) * norm
    a = np.array([1.0, 2 * (k * k - 1) * norm, (1 - q * k + k * k) * norm])

    def run(sig):
        y = np.zeros_like(sig)
        y[0] = y[1] = sig[0]
        for i in range(2, len(sig)):
            y[i] = (b[0] * sig[i] + b[1] * sig[i - 1] + b[2] * sig[i - 2]
                    - a[1] * y[i - 1] - a[2] * y[i - 2])
        return y

    return run(run(x)[::-1])[::-1]


def jerk_rms(t, a, t0=None, t1=None, cutoff=JERK_CUTOFF_HZ):
    """종방향 가속도 a(t) [m/s^2] → jerk RMS [m/s^3]. 균일 시간 격자로 다시 표본화한다."""
    t, a = np.asarray(t, float), np.asarray(a, float)
    if len(t) < 8:
        return None
    dt = float(np.median(np.diff(t)))
    tg = np.arange(t[0], t[-1], dt)
    af = butter2_lowpass(np.interp(tg, t, a), 1.0 / dt, cutoff)
    jerk = np.gradient(af, dt)
    m = np.ones_like(tg, dtype=bool)
    if t0 is not None:
        m &= tg >= t0
    if t1 is not None:
        m &= tg <= t1
    return float(np.sqrt(np.mean(jerk[m] ** 2))) if m.any() else None


def footprint_distance(points_base, footprint):
    """base_link 좌표 장애물 점들 (N,2)과 차체 다각형의 최소 거리."""
    if len(points_base) == 0:
        return math.inf
    return min(point_polygon_distance(float(x), float(y), footprint) for x, y in points_base)


def safety_metrics(t, dist, ego_speed, collision_tol=0.02):
    """시간별 최소 거리 dist(t)로 최소 거리, 충돌, 충돌 시 속도, 최소 TTC를 낸다.

    TTC = 거리 / 접근 속도(−d dist/dt, 양수일 때). 장애물 상대속도를 쓰므로 AEB의 에고 속도 TTC와 다르다.
    ego_ttc = 거리 / 에고 속도 (AEB와 같은 정의)도 함께 낸다.
    """
    t, dist, v = (np.asarray(x, float) for x in (t, dist, ego_speed))
    finite = np.isfinite(dist)
    if not finite.any():
        return {'min_distance': None, 'collision': False, 'impact_speed': None,
                'min_ttc': None, 'min_ego_ttc': None}
    i = int(np.nanargmin(np.where(finite, dist, np.nan)))
    collision = bool(dist[i] <= collision_tol)
    closing = -np.gradient(np.where(finite, dist, np.nan), t)
    with np.errstate(divide='ignore', invalid='ignore'):
        ttc = np.where((closing > 1e-3) & finite, dist / closing, np.inf)
        ego_ttc = np.where((v > 1e-3) & finite, dist / v, np.inf)
    return {'min_distance': float(dist[i]), 'collision': collision,
            'impact_speed': float(v[i]) if collision else None,
            'min_ttc': float(np.min(ttc)) if np.isfinite(ttc).any() else None,
            'min_ego_ttc': float(np.min(ego_ttc)) if np.isfinite(ego_ttc).any() else None}


T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262,
       10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110,
       18: 2.101, 19: 2.093, 20: 2.086, 25: 2.060, 30: 2.042}


def mean_ci95(values):
    """평균과 95% 신뢰구간 반폭 (t 분포). None은 제외."""
    x = np.array([v for v in values if v is not None and np.isfinite(v)], float)
    n = len(x)
    if n == 0:
        return None, None, 0
    if n == 1:
        return float(x[0]), None, 1
    df = n - 1
    tcrit = T95.get(df) or (T95[max(k for k in T95 if k <= df)] if df < 30 else 1.96)
    return float(x.mean()), float(tcrit * x.std(ddof=1) / math.sqrt(n)), n


def boundary_jitter(frames_xy, bin_size=0.1, min_frames=5):
    """RQ4: 프레임별 가림 경계 위치의 흔들림.

    frames_xy: 프레임마다 경계 칸 중심 (N, 2) 목록. x를 bin_size로 묶고, 각 bin에서 차선에 가장 가까운
    (|y| 최소) 경계 y를 프레임마다 뽑아 그 표준편차를 bin 평균한다. 반환 (평균 표준편차 [m] 또는 None, bin 수).
    """
    per_bin = {}
    for xy in frames_xy:
        if len(xy) == 0:
            continue
        xy = np.asarray(xy, float)
        bins = np.round(xy[:, 0] / bin_size).astype(int)
        for b in np.unique(bins):
            ys = xy[bins == b, 1]
            per_bin.setdefault(int(b), []).append(float(ys[np.argmin(np.abs(ys))]))
    stds = [np.std(v) for v in per_bin.values() if len(v) >= min_frames]
    return (float(np.mean(stds)) if stds else None), len(stds)


def phantom_flicker(counts):
    """phantom 수 변화 횟수와 있음/없음 전환 횟수."""
    counts = np.asarray(counts)
    if len(counts) < 2:
        return 0, 0
    return int(np.count_nonzero(np.diff(counts))), int(np.count_nonzero(np.diff((counts > 0).astype(int))))

