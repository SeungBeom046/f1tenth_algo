#!/usr/bin/env python3
"""Exp0 급제동 회차(scenario E0)들 → a_b, t_r, 정지거리 (평균 ± 95% CI)와 common.yaml 권장값.

    python3 analysis/exp0_braking.py ~/exp_logs/20261110

0 명령 시각 = /exp/stop_reason이 처음 'stop_line'이 된 시각 (bag 수신 시각).
t_r에는 인지 지연(점군 주기 10 Hz → 평균 50 ms + 처리 시간 /exp/grid_debug ms)이 들어 있지 않다.
common.yaml t_r = (측정 t_r 상한) + 인지 지연 으로 정한다 (EXPERIMENT_PLAN Exp0).
권장값은 보수적으로 a_b = 95% CI 하한, t_r = 95% CI 상한.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import read_bag, stamp  # noqa: E402

from my_algo.braking import braking_metrics  # noqa: E402
from my_algo.metrics import mean_ci95, travel_distance  # noqa: E402


def one(run_dir):
    with open(os.path.join(run_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
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
    return r


def main():
    root = sys.argv[1]
    runs = [os.path.join(root, d) for d in sorted(os.listdir(root)) if d.startswith('E0_')]
    results = [r for r in (one(d) for d in runs) if r]
    for r in results:
        print(f'{r["run"]}: v0={r["v0"]:.2f} a_b={r["a_b"]} t_r={r["t_r"]} stop={r["stop_distance"]}')
    summary = {}
    for k in ('v0', 'a_b', 't_r', 'stop_distance'):
        mean, half, n = mean_ci95([r[k] for r in results])
        summary[k] = {'mean': mean, 'ci95': half, 'n': n}
    rec = {}
    if summary['a_b']['mean'] is not None:
        rec['a_b'] = summary['a_b']['mean'] - (summary['a_b']['ci95'] or 0.0)
    if summary['t_r']['mean'] is not None:
        rec['t_r_actuation'] = summary['t_r']['mean'] + (summary['t_r']['ci95'] or 0.0)
    out = {'runs': results, 'summary': summary, 'recommended': rec,
           'note': 't_r_actuation에 인지 지연을 더해 common.yaml t_r로 쓴다'}
    path = os.path.join(root, 'exp0_braking.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(json.dumps(summary, indent=2))
    print(f'권장 (보수적): {rec}  → {path}')


if __name__ == '__main__':
    main()
