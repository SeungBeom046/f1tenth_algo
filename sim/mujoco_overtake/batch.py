#!/usr/bin/env python3
"""헤드리스 일괄 실행 → CSV + 요약 표.

    python sim/mujoco_overtake/batch.py belief [--n 200] [--methods cd]   # belief 2종 × 방법, Y 회차 (재실험)
    python sim/mujoco_overtake/batch.py yaw [--yaws 0 10 20 30]           # 선행차 양보 yaw별 tracker 추적 유지
    python sim/mujoco_overtake/batch.py matrix [--n 5]                    # 방법 4 × 선행차 Y/NY
    python sim/mujoco_overtake/batch.py aeb [--n 20]                      # 방법 c·d × NY × AEB 조건 (N_AEB)
    python sim/mujoco_overtake/batch.py treact [--n 200]                  # t_react별 survival c·d Y 성공률
    python sim/mujoco_overtake/batch.py yr [--n 50]                       # 양보 후 복귀(YR), 복귀 시간별
출력: sim_logs/mujoco_overtake/<이름>.csv (git에 올리지 않는다)

--aeb off|zero|true (aeb 모드 외 기본 off): 판단 로직만 보려면 AEB를 끈다. zero = 현재 실차(odom 없음, 속도 0),
true = 4단계 이후(실제 속도). AEB를 켜면 양보 차선 벽에서 AEB가 서는 알려진 문제가 결과를 덮는다 (docs/PROGRESS.md).
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


AEB = {'off': {'aeb.enabled': False}, 'zero': {'aeb.speed_source': 'zero'}, 'true': {'aeb.speed_source': 'true'},
       'true_band015': {'aeb.speed_source': 'true', 'aeb.path_margin': 0.0, 'aeb.bumper_emergency_distance': 0.10}}


def cfg(args, **over):
    return load_cfg(**dict(AEB[args.aeb], **{'track.W': args.W}, **over))


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else float('nan')


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
                r = Sim(m, cfg(args, **{'leader.behavior': 'Y', 'leader.d_y': d_y}),
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
            r = Sim(args.methods[0], cfg(args, **{'leader.behavior': 'Y', 'leader.d_y': d_y,
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
    for m in 'abcd':
        for beh in ('Y', 'NY'):
            rs = [Sim(m, cfg(args, **{'leader.behavior': beh}), seed=k).run() for k in range(args.n)]
            rows += rs
            print('| %s | %s | %d/%d | %d | %d | %.3f | %.3f | %.2f | %.2f |' % (
                m, beh, sum(r['success'] for r in rs), len(rs), sum(r['pass_entered'] for r in rs),
                sum(bool(r['collision']) for r in rs), mean([r['h_min'] for r in rs]), min(r['gap_min'] for r in rs),
                mean([r['T_id'] for r in rs if r['T_id'] is not None]),
                mean([r['T_pass'] for r in rs if r['T_pass'] is not None])))
    save('matrix', rows)


def aeb(args):
    rows = []
    print('| AEB 조건 | 방법 | 충돌 | 최소 간격 [m] | ego N_AEB 합 (회차 평균) | AEB 걸린 회차 | 선행차 N_AEB 합 | PASS 진입 |')
    print('|---|---|---|---|---|---|---|---|')
    for cond in ('off', 'zero', 'true', 'true_band015'):
        args.aeb = cond
        for m in args.methods:
            rs = [Sim(m, cfg(args, **{'leader.behavior': 'NY'}), seed=k).run() for k in range(args.n)]
            for r in rs:
                r['aeb_cond'] = cond
            rows += rs
            n = sum(r['N_AEB'] for r in rs)
            print('| %s | %s | %d/%d | %.3f | %d (%.2f) | %d | %d | %d |' % (
                cond, m, sum(bool(r['collision']) for r in rs), len(rs), min(r['gap_min'] for r in rs),
                n, n / len(rs), sum(r['N_AEB'] > 0 for r in rs), sum(r['N_AEB_leader'] for r in rs),
                sum(r['pass_entered'] for r in rs)))
    save('aeb', rows)


def treact(args):
    rows = []
    vals = (0.0, 0.5, 1.0, 1.3, 1.6)
    print('AEB: %s, W = %.2f, Y 회차, 같은 d_y 표본 (U[0.6, 1.2])' % (args.aeb, args.W))
    print('| 방법 | ' + ' | '.join('t_react %.1f' % v for v in vals) + ' |')
    print('|---' * (1 + len(vals)) + '|')
    for m in args.methods:
        cells = []
        for v in vals:
            rng = random.Random(0)
            rs = []
            for k in range(args.n):
                r = Sim(m, cfg(args, **{'leader.behavior': 'Y', 'leader.d_y': rng.uniform(0.6, 1.2)}),
                        seed=k, t_react=v).run()
                r['t_react'] = v
                rs.append(r)
            rows += rs
            ok = sum(r['success'] for r in rs)
            cells.append('%d/%d (NY 오판 %d, T_id %.1f s)' % (ok, len(rs), sum(r['cause'] == 'belief_NY' for r in rs),
                                                           mean([r['T_id'] for r in rs if r['success']])))
        print('| %s | %s |' % (m, ' | '.join(cells)))
    save('treact', rows)


def yr(args):
    rows = []
    print('AEB: %s, W = %.2f, 선행차 YR (양보 후 return_after초 뒤 중앙 복귀), d_y ~ U[0.8, 1.2]' % (args.aeb, args.W))
    print('| 방법 | return_after [s] | DONE | 추월 중단(FOLLOW) | 충돌 | 기타 | PASS 진입 | 최소 간격 [m] |')
    print('|---|---|---|---|---|---|---|---|')
    for m in args.methods:
        for ra in (0.5, 1.0, 1.5, 2.0, 3.0):
            rng = random.Random(0)
            rs = []
            for k in range(args.n):
                r = Sim(m, cfg(args, **{'leader.behavior': 'YR', 'leader.return_after': ra,
                                        'leader.d_y': rng.uniform(0.8, 1.2)}), seed=k).run()
                r['return_after'] = ra
                rs.append(r)
            rows += rs
            aborted = sum(r['n_abort'] > 0 and not r['collision'] for r in rs)
            done = sum(r['success'] for r in rs)
            coll = sum(bool(r['collision']) for r in rs)
            print('| %s | %.1f | %d | %d | %d | %d | %d | %.3f |' % (
                m, ra, done, aborted, coll, len(rs) - done - aborted - coll, sum(r['pass_entered'] for r in rs),
                min(r['gap_min'] for r in rs)))
    save('yr', rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('exp', choices=['belief', 'yaw', 'matrix', 'aeb', 'treact', 'yr'])
    ap.add_argument('--aeb', default='off', choices=list(AEB))
    ap.add_argument('--n', type=int, default=None)
    ap.add_argument('--methods', default=None)
    ap.add_argument('--W', type=float, default=1.0)
    ap.add_argument('--yaws', type=float, nargs='*', default=[0, 10, 20, 30])
    ap.add_argument('--d-ys', type=float, nargs='*', default=[0.8, 1.0, 1.2])
    args = ap.parse_args()
    args.n = args.n or {'belief': 200, 'matrix': 5, 'yaw': 1, 'aeb': 20, 'treact': 200, 'yr': 50}[args.exp]
    args.methods = args.methods or {'belief': 'cd', 'yaw': 'd', 'matrix': 'abcd'}.get(args.exp, 'cd')
    {'belief': belief, 'yaw': yaw, 'matrix': matrix, 'aeb': aeb, 'treact': treact, 'yr': yr}[args.exp](args)


if __name__ == '__main__':
    main()
