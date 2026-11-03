"""Exp0 급제동 분석 (ROS 비의존): 0 명령 시각부터 감속 시작(t_r)과 평균 감속도(a_b), 정지거리.

    t_cmd    : 0 명령이 나간 시각 (/exp/stop_reason이 stop_line이 된 첫 시각)
    t_onset  : 종방향 가속도(저역통과 후)가 -0.5 a_b 아래로 처음 내려간 시각 (감속도 50% 도달).
               양방향 저역통과는 계단을 앞뒤로 번지게 하므로 고정 임계값은 시작을 일찍 잡는다.
               50% 지점은 대칭 필터에서도 제자리라 이것을 기준으로 정의한다. a_b를 못 구하면 onset_threshold
    t_r      = t_onset - t_cmd  (인지 제외: 명령 → VESC → 감속 시작 지연. 인지 지연은 별도 더함)
    a_b      : v가 v0의 90%에서 10%로 떨어지는 동안의 평균 감속도 (odom 속도 기울기) [m/s^2]
    stop_dist: t_cmd 위치부터 정지(v < 0.05)까지 이동 거리
"""
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
    return {'v0': v0, 'a_b': a_b, 't_r': t_r, 'stop_distance': stop_dist}
