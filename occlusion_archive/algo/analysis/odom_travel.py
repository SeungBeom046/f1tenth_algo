#!/usr/bin/env python3
"""Exp0 odom 스케일 보정: 회차별 AUTO 시작부터 정지까지 odom 주행 거리.

    python3 analysis/odom_travel.py <run_dir> [<run_dir> ...] --tape 5.02 5.01 4.99

줄자로 잰 실제 거리(--tape, 회차 순서대로)를 주면 비율 = 줄자 / odom 과 평균을 낸다.
보정은 speed_to_erpm_gain(odom)과 ERPM_GAIN(명령)을 같은 값으로 맞추는 방향으로 한다 (docs/OPEN_ITEMS.md 2절 경고).
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import read_bag, stamp  # noqa: E402

from my_algo.metrics import mean_ci95, travel_distance  # noqa: E402


def travel(run_dir):
    with open(os.path.join(run_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    topic = meta.get('odom_topic', '/vesc/odom')
    bag = read_bag(os.path.join(run_dir, 'bag'), [topic, '/autonomous_mode'])
    t_auto = next(t for t, m in bag['/autonomous_mode'] if m.data)
    odom = [m for _, m in bag[topic] if stamp(m) >= t_auto]
    s = travel_distance([m.pose.pose.position.x for m in odom], [m.pose.pose.position.y for m in odom])
    return float(s[-1]) if len(s) else None, meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--tape', nargs='*', type=float, default=[])
    args = ap.parse_args()
    ratios = []
    for i, r in enumerate(args.runs):
        d, meta = travel(r)
        tape = args.tape[i] if i < len(args.tape) else None
        ratio = tape / d if tape and d else None
        ratios.append(ratio)
        print(f'{os.path.basename(r)}: v_ref={meta.get("test_v_ref") or "기본"} odom={d:.3f} m '
              f'tape={tape} ratio={ratio}')
    mean, half, n = mean_ci95(ratios)
    if mean:
        print(f'줄자/odom 비율 평균 {mean:.4f} ± {half or 0:.4f} (n={n}) → 실제 계수 ≈ 현재 계수 / 비율')


if __name__ == '__main__':
    main()
