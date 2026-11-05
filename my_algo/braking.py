"""Exp0 지연·제동 분석 (ROS 비의존).

t_r 정의 (config/common.yaml): 관측 → 실제 감속 시작까지 전체 지연 = 인지 + 통신 + 제어 + 구동.
v_cap 식은 '순수 지연 t_r 뒤 일정 감속 a_b'를 가정하므로, 속도 제어의 1차 지연은 같은 효과를 내는 등가 지연으로 넣는다.
    tracking_lag : 명령 속도가 a_b보다 완만하게 내려갈 때(v_cap을 따라갈 때) 실측 속도가 명령보다 늦는 시간.
                   순수 지연 t_d + 1차 지연 τ인 계는 기울기 일정한 명령을 정확히 t_d + τ 늦게 따라간다 → 등가 지연.
    equivalent_delay : 급제동 정지거리를 '순수 지연 + 일정 감속 a_b'로 맞추는 지연 (v0 t + v0²/(2 a_b) = 정지거리).
                   계단 명령에서는 감속이 a_b로 바로 포화되어 1차 지연이 거의 드러나지 않는다 (참고값).
    common.yaml t_r = 인지 지연 + tracking_lag (CI 상한). 인지 지연 = 라이다 주기 평균(50 ms) + 격자 처리(ms).

급제동 지표 (계단 0 명령):

    t_cmd    : 0 명령이 나간 시각 (/exp/stop_reason이 stop_line이 된 첫 시각)
    t_onset  : 종방향 가속도(저역통과 후)가 -0.5 a_b 아래로 처음 내려간 시각 (감속도 50% 도달).
               양방향 저역통과는 계단을 앞뒤로 번지게 하므로 고정 임계값은 시작을 일찍 잡는다.
               50% 지점은 대칭 필터에서도 제자리라 이것을 기준으로 정의한다. a_b를 못 구하면 onset_threshold
    t_r      = t_onset - t_cmd  (계단 응답의 구동 지연. 참고값 — common.yaml t_r에는 tracking_lag을 쓴다)
    a_b      : v가 v0의 90%에서 10%로 떨어지는 동안의 평균 감속도 (odom 속도 기울기) [m/s^2]
    stop_dist: t_cmd 위치부터 정지(v < 0.05)까지 이동 거리
"""
import math

import numpy as np

from my_algo.metrics import JERK_CUTOFF_HZ, butter2_lowpass


def braking_metrics(t_cmd, t_v, v, s, t_a=None, a=None, onset_threshold=-0.5):
    t_v, v, s = (np.asarray(x, float) for x in (t_v, v, s))
    after = t_v >= t_cmd
    if not after.any():
        return None
    v0 = float(np.interp(t_cmd, t_v, v))
    s0 = float(np.interp(t_cmd, t_v, s))
    tv, vv, ss = t_v[after], v[after], s[after]
    hi = np.flatnonzero(vv <= 0.9 * v0)
    lo = np.flatnonzero(vv <= 0.1 * v0)
    a_b = None
    if len(hi) and len(lo) and lo[0] > hi[0]:
        a_b = float((vv[hi[0]] - vv[lo[0]]) / (tv[lo[0]] - tv[hi[0]]))
    stopped = np.flatnonzero(vv < 0.05)
    stop_dist = float(ss[stopped[0]] - s0) if len(stopped) else None
    t_r = None
    if t_a is not None and len(t_a) > 8:
        t_a, a = np.asarray(t_a, float), np.asarray(a, float)
        dt = float(np.median(np.diff(t_a)))
        af = butter2_lowpass(a, 1.0 / dt, JERK_CUTOFF_HZ)
        thr = -0.5 * a_b if a_b else onset_threshold
        onset = np.flatnonzero((t_a >= t_cmd) & (af < thr))
        if len(onset):
            t_r = float(t_a[onset[0]] - t_cmd)
    return {'v0': v0, 'a_b': a_b, 't_r': t_r, 'stop_distance': stop_dist,
            't_r_equiv': equivalent_delay(v0, stop_dist, a_b)}


def equivalent_delay(v0, stop_distance, a_b):
    """정지거리를 '순수 지연 + 일정 감속 a_b'로 맞추는 지연 [s]: v0 t + v0²/(2 a_b) = stop_distance."""
    if not v0 or v0 <= 0 or stop_distance is None or not a_b:
        return None
    return float((stop_distance - v0 * v0 / (2.0 * a_b)) / v0)


def _first_order(u, dt, tau):
    """1차 지연 필터 (정확 이산화). tau가 0이면 그대로."""
    if tau <= 0:
        return u.copy()
    alpha = 1.0 - math.exp(-dt / tau)
    y = np.empty_like(u)
    y[0] = u[0]
    for k in range(1, len(u)):
        y[k] = y[k - 1] + alpha * (u[k] - y[k - 1])
    return y


def tracking_lag(t_cmd, v_cmd, t_v, v, a_b, max_dead=0.4, max_tau=0.4, dt=0.01, min_slope=0.3, sat_ratio=0.9,
                 smooth_s=0.2):
    """명령 속도 하강 구간에서 실측 속도의 등가 지연 [s] = 순수 지연 t_d + 1차 지연 τ.

    명령 v_cmd(t)를 'v(t) = 1차지연_τ(v_cmd(t - t_d))'로 맞추는 (t_d, τ)를 격자로 찾는다 (0.01 → 0.002 s).
    오차는 지연된 명령이 하강 중인 시각에서만 잰다: 명령 기울기가 -sat_ratio a_b보다 완만하고 -min_slope보다 가파른 곳
    (감속 포화·정속 제외). 기울기 일정한 명령에서 이 계는 정확히 t_d + τ 늦게 따라간다 → v_cap 식의 등가 지연.
    반환 {'lag': t_d + τ, 'dead': t_d, 'tau': τ, 'n': 쓴 표본 수, 'rmse': 오차} 또는 None.
    """
    t_cmd, v_cmd, t_v, v = (np.asarray(x, float) for x in (t_cmd, v_cmd, t_v, v))
    if len(t_cmd) < 5 or len(t_v) < 5:
        return None
    tg = np.arange(t_cmd[0], t_cmd[-1], dt)
    ug = np.interp(tg, t_cmd, v_cmd)
    # 명령은 계단 모양이다 (d가 10 Hz로 갱신). 하강 구간 선택만 smooth_s 이동 평균 기울기로 한다 (모델에는 원 명령)
    w = max(1, int(round(smooth_s / dt)))
    slope = np.gradient(np.convolve(np.pad(ug, (w // 2, w - 1 - w // 2), mode='edge'), np.ones(w) / w, 'valid'), dt)
    use_g = (slope < -min_slope) & (slope > -sat_ratio * a_b)
    vg = np.interp(tg, t_v, v)
    inside = (tg >= t_v[0]) & (tg <= t_v[-1])

    def err(t_d, tau):
        shift = np.interp(tg - t_d, tg, ug, left=ug[0])
        y = _first_order(shift, dt, tau)
        m = inside & (np.interp(tg - t_d, tg, use_g.astype(float), left=0.0) > 0.5)
        if m.sum() < 5:
            return np.inf, 0
        return float(np.mean((y[m] - vg[m]) ** 2)), int(m.sum())

    best = (np.inf, None, None, 0)
    for t_d in np.arange(0.0, max_dead + 1e-9, 0.01):
        for tau in np.arange(0.0, max_tau + 1e-9, 0.01):
            e, n = err(t_d, tau)
            if e < best[0]:
                best = (e, t_d, tau, n)
    if best[1] is None:
        return None
    _, d0, tau0, _ = best
    for t_d in np.arange(max(0.0, d0 - 0.01), d0 + 0.0101, 0.002):
        for tau in np.arange(max(0.0, tau0 - 0.01), tau0 + 0.0101, 0.002):
            e, n = err(t_d, tau)
            if e < best[0]:
                best = (e, t_d, tau, n)
    e, t_d, tau, n = best
    return {'lag': float(t_d + tau), 'dead': float(t_d), 'tau': float(tau), 'n': n, 'rmse': math.sqrt(e)}
