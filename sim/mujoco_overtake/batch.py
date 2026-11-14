#!/usr/bin/env python3
"""헤드리스 일괄 실행 → CSV + 요약 표.

    python sim/mujoco_overtake/batch.py belief [--n 200] [--methods cd]   # belief 2종 × 방법, Y 회차 (재실험)
    python sim/mujoco_overtake/batch.py yaw [--yaws 0 10 20 30]           # 선행차 양보 yaw별 tracker 추적 유지
    python sim/mujoco_overtake/batch.py matrix [--n 5]                    # 방법 4 × 선행차 Y/NY
출력: sim_logs/mujoco_overtake/<이름>.csv (git에 올리지 않는다)
"""
import argparse
import csv
import os
import random

from sim import REPO, Sim, load_cfg

CAUSES = ('belief_NY', 'T_wait_a', 'T_max', 'track_lost', 'collision', 'other')
BINS = [(0.6, 0.7), (0.7, 0.8), (0.8, 0.9), (0.9, 1.0), (1.0, 1.1), (1.1, 1.2)]


def save(name, rows):
    out = os.path.join(REPO, 'sim_logs', 'mujoco_overtake')
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, name + '.csv')
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('\n저장: %s (%d회)' % (os.path.relpath(path, REPO), len(rows)))


def cause_of(r):
    return 'collision' if r['cause'].startswith('collision') else r['cause']


def belief(args):
    rows = []
    print('| belief | 방법 | 성공 | ' + ' | '.join(CAUSES) + ' | ' + ' | '.join('d_y %.1f~%.1f' % b for b in BINS) + ' |')
    print('|---' * (3 + len(CAUSES) + len(BINS)) + '|')
    for model in ('original', 'survival'):
        for m in args.methods:
            rng = random.Random(0)          # 모든 조합이 같은 d_y 표본을 쓴다
            fails, ok, bins = dict.fromkeys(CAUSES, 0), 0, [[0, 0] for _ in BINS]
            for k in range(args.n):
                d_y = rng.uniform(0.6, 1.2)
                r = Sim(m, load_cfg(**{'leader.behavior': 'Y', 'leader.d_y': d_y, 'track.W': args.W}),
                        seed=k, belief_model=model).run()
                rows.append(r)
                ok += r['success']
                if not r['success']:
                    fails[cause_of(r)] += 1
                for b, (lo, hi) in zip(bins, BINS):
                    if lo <= d_y < hi:
                        b[0] += r['success']
                        b[1] += 1
            print('| %s | %s | %d/%d | ' % (model, m, ok, args.n) + ' | '.join(str(fails[c]) for c in CAUSES) +
                  ' | ' + ' | '.join('%d/%d' % tuple(b) for b in bins) + ' |')
    save('belief', rows)


def yaw(args):
    rows = []
    print('| 양보 yaw [deg] | d_y | 추적 필요 scan | 끊긴 scan | 최장 끊김 [s] | 클러스터 폭 최대 [m] | 결과 | 원인 |')
    print('|---|---|---|---|---|---|---|---|')
    for deg in args.yaws:
        for d_y in args.d_ys:
            r = Sim(args.methods[0], load_cfg(**{'leader.behavior': 'Y', 'leader.d_y': d_y, 'track.W': args.W,
                                                 'leader.yield_yaw_deg': float(deg)})).run()
            r['yield_yaw_deg'] = deg
            rows.append(r)
            print('| %g | %.1f | %d | %d | %.1f | %s | %s | %s |' % (
                deg, d_y, r['track_need'], r['track_lost'], r['track_lost_longest_s'], r['width_max'],
                r['final_state'], r['cause'] or '-'))
    save('yaw', rows)


def matrix(args):
    rows = []
    print('| 방법 | 선행차 | 성공(DONE) | PASS 진입 | 충돌 | h_min 평균 | 최소 간격 | T_id 평균 | T_pass 평균 |')
    print('|---|---|---|---|---|---|---|---|---|')
    mean = lambda xs: sum(xs) / len(xs) if xs else float('nan')   # noqa: E731
    for m in 'abcd':
        for beh in ('Y', 'NY'):
            rs = [Sim(m, load_cfg(**{'leader.behavior': beh, 'track.W': args.W}), seed=k).run() for k in range(args.n)]
            rows += rs
            print('| %s | %s | %d/%d | %d | %d | %.3f | %.3f | %.2f | %.2f |' % (
                m, beh, sum(r['success'] for r in rs), len(rs), sum(r['pass_entered'] for r in rs),
                sum(bool(r['collision']) for r in rs), mean([r['h_min'] for r in rs]), min(r['gap_min'] for r in rs),
                mean([r['T_id'] for r in rs if r['T_id'] is not None]),
                mean([r['T_pass'] for r in rs if r['T_pass'] is not None])))
    save('matrix', rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('exp', choices=['belief', 'yaw', 'matrix'])
    ap.add_argument('--n', type=int, default=None)
    ap.add_argument('--methods', default=None)
    ap.add_argument('--W', type=float, default=1.0)
    ap.add_argument('--yaws', type=float, nargs='*', default=[0, 10, 20, 30])
    ap.add_argument('--d-ys', type=float, nargs='*', default=[0.8, 1.0, 1.2])
    args = ap.parse_args()
    args.n = args.n or {'belief': 200, 'matrix': 5, 'yaw': 1}[args.exp]
    args.methods = args.methods or {'belief': 'cd', 'yaw': 'd', 'matrix': 'abcd'}[args.exp]
    {'belief': belief, 'yaw': yaw, 'matrix': matrix}[args.exp](args)


if __name__ == '__main__':
    main()
