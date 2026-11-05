#!/usr/bin/env python3
"""Exp0 지연·제동 → a_b, t_r(전체 지연), 정지거리 (평균 ± 95% CI)와 common.yaml 권장값.

    python3 analysis/exp0_braking.py ~/exp_logs/20261110                                  # c. 급제동 (E0_*)
    python3 analysis/exp0_braking.py ~/exp_logs/20261110 --tracking S2_B2_dena_run01 ...   # + i. 추종 지연

t_r 정의 (common.yaml): 관측 → 실제 감속 시작 전체 지연 = 인지 + 통신 + 제어 + 구동 (1차 지연은 등가 지연으로 포함).
    c. 급제동 (E0): a_b, 정지거리, 계단 구동 지연(t_r_step, 참고), 정지거리 등가 지연(t_r_equiv, 참고).
       0 명령 시각 = /exp/stop_reason이 처음 'stop_line'이 된 시각. 계단 명령은 감속이 a_b로 바로 포화돼
       속도 제어의 1차 지연이 거의 드러나지 않으므로 t_r에는 쓰지 않는다.
    i. 추종 지연 (B2, S2 배치): 명령(/exp/final_cmd)이 v_cap 곡선을 따라 완만히 내려가는 구간에서
       odom 속도의 등가 지연 = 순수 지연 + 1차 지연 (my_algo.braking.tracking_lag). 통신 + 제어 + 구동.
    인지 지연 = 라이다 주기 평균(0.5 / 10 Hz = 50 ms) + 격자 처리 시간(/exp/grid_debug ms 중앙값, 추종 회차에서).
권장값 (보수적): a_b = 급제동 95% CI 하한, t_r = 인지 지연 + 추종 지연 95% CI 상한.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import read_bag, stamp  # noqa: E402

from my_algo.braking import braking_metrics, tracking_lag  # noqa: E402
from my_algo.metrics import mean_ci95, travel_distance  # noqa: E402

LIDAR_HZ = 10.0     # Mid-360 점군 주기 (common.yaml 격자 누적과 같은 가정)


def _meta(run_dir):
    with open(os.path.join(run_dir, 'meta.json'), encoding='utf-8') as f:
        return json.load(f)


def one(run_dir):
    meta = _meta(run_dir)
    odom_topic = meta.get('odom_topic', '/vesc/odom')
    bag = read_bag(os.path.join(run_dir, 'bag'),
                   [odom_topic, '/exp/stop_reason', '/imu/linear_accel_base', '/autonomous_mode'])
    t_cmd = next((t for t, m in bag.get('/exp/stop_reason', []) if m.data == 'stop_line'), None)
    t_auto = next((t for t, m in bag.get('/autonomous_mode', []) if m.data), None)
    if t_cmd is None or t_auto is None:
        return None
    odom = [(stamp(m), m) for _, m in bag[odom_topic] if stamp(m) >= t_auto]
    t_v = np.array([t for t, _ in odom])
    v = np.array([m.twist.twist.linear.x for _, m in odom])
    s = travel_distance([m.pose.pose.position.x for _, m in odom], [m.pose.pose.position.y for _, m in odom])
    imu = bag.get('/imu/linear_accel_base', [])
    t_a = np.array([stamp(m) for _, m in imu])
    a = np.array([m.vector.x for _, m in imu])
    r = braking_metrics(t_cmd, t_v, v, s, t_a if len(t_a) else None, a if len(a) else None)
    if r:
        r['run'] = os.path.basename(run_dir)
        r['t_r_step'] = r.pop('t_r')
    return r


def one_tracking(run_dir, a_b):
    meta = _meta(run_dir)
    odom_topic = meta.get('odom_topic', '/vesc/odom')
    bag = read_bag(os.path.join(run_dir, 'bag'), [odom_topic, '/exp/final_cmd', '/exp/grid_debug',
                                                  '/autonomous_mode'])
    t_auto = next((t for t, m in bag.get('/autonomous_mode', []) if m.data), None)
    if t_auto is None:
        return None
    odom = [(stamp(m), m) for _, m in bag[odom_topic] if stamp(m) >= t_auto]
    cmd = [(t, m.data) for t, m in bag.get('/exp/final_cmd', []) if t >= t_auto]
    if len(odom) < 5 or len(cmd) < 5:
        return None
    r = tracking_lag(np.array([t for t, _ in cmd]), np.array([c for _, c in cmd]),
                     np.array([t for t, _ in odom]), np.array([m.twist.twist.linear.x for _, m in odom]), a_b)
    if r is None:
        return None
    ms = [json.loads(m.data).get('ms') for _, m in bag.get('/exp/grid_debug', [])]
    ms = [x for x in ms if x is not None]
    r.update(run=os.path.basename(run_dir), grid_ms_median=float(np.median(ms)) if ms else None)
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('session_dir')
    ap.add_argument('--tracking', nargs='*', default=[], help='i. 추종 지연 회차 폴더 (session_dir 기준 또는 경로)')
    args = ap.parse_args()
    root = args.session_dir
    runs = [os.path.join(root, d) for d in sorted(os.listdir(root)) if d.startswith('E0_')]
    results = [r for r in (one(d) for d in runs) if r]
    for r in results:
        print(f'{r["run"]}: v0={r["v0"]:.2f} a_b={r["a_b"]} t_r_step={r["t_r_step"]} '
              f't_r_equiv={r["t_r_equiv"]} stop={r["stop_distance"]}')
    summary = {}
    for k in ('v0', 'a_b', 't_r_step', 't_r_equiv', 'stop_distance'):
        mean, half, n = mean_ci95([r[k] for r in results])
        summary[k] = {'mean': mean, 'ci95': half, 'n': n}
    rec = {}
    if summary['a_b']['mean'] is not None:
        rec['a_b'] = summary['a_b']['mean'] - (summary['a_b']['ci95'] or 0.0)
    a_b = rec.get('a_b', 3.0)
    tracking = []
    for d in args.tracking:
        path = d if os.path.isdir(d) else os.path.join(root, d)
        r = one_tracking(path, a_b)
        if r:
            tracking.append(r)
            print(f'{r["run"]}: 추종 지연 {r["lag"]:.3f} s (순수 {r["dead"]:.3f} + 1차 {r["tau"]:.3f}, '
                  f'표본 {r["n"]}, rmse {r["rmse"]:.3f}) 격자 {r["grid_ms_median"]} ms')
    mean, half, n = mean_ci95([r['lag'] for r in tracking])
    summary['tracking_lag'] = {'mean': mean, 'ci95': half, 'n': n}
    ms = [r['grid_ms_median'] for r in tracking if r['grid_ms_median'] is not None]
    perception = 0.5 / LIDAR_HZ + (float(np.median(ms)) / 1000.0 if ms else 0.0)
    summary['perception_delay'] = {'value': perception, 'grid_ms_known': bool(ms)}
    if mean is not None:
        rec['t_r'] = perception + mean + (half or 0.0)
    out = {'runs': results, 'tracking_runs': tracking, 'summary': summary, 'recommended': rec,
           'note': 't_r = 인지 지연 + 추종 지연(CI 상한). 추종 회차가 없으면 t_r 권장값을 내지 않는다'}
    path = os.path.join(root, 'exp0_braking.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f'권장 (보수적): {rec}  → {path}')
    if 't_r' not in rec:
        print('추종 지연 회차(--tracking)가 없어 t_r 권장값 없음 (EXPERIMENT_PLAN Exp0 i)', file=sys.stderr)


if __name__ == '__main__':
    main()
