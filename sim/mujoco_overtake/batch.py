#!/usr/bin/env python3
"""헤드리스 일괄 실행 → CSV + 요약 표.

    python sim/mujoco_overtake/batch.py belief [--n 200] [--methods cd]   # belief 2종 × 방법, Y 회차 (재실험)
    python sim/mujoco_overtake/batch.py yaw [--yaws 0 10 20 30]           # 선행차 양보 yaw별 tracker 추적 유지
    python sim/mujoco_overtake/batch.py matrix [--n 5]                    # 방법 4 × 선행차 Y/NY
    python sim/mujoco_overtake/batch.py aeb [--n 20]                      # W × AEB 조건 × 방법 c·d × Y/NY (교착, N_AEB)
    python sim/mujoco_overtake/batch.py yrw --aeb geometric               # 양보 후 복귀 × W × 선행차 W 고정 on/off
    python sim/mujoco_overtake/batch.py treact_ny                         # t_react별 NY 쪽 비용
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


AEB = {'off': {'aeb.enabled': False},
       'legacy_zero_wide': {'aeb.model': 'legacy', 'aeb.speed_source': 'zero', 'aeb.path_margin': 0.15,
                            'aeb.ttc_threshold': 0.55},                           # 현재 실차 그대로
       'geometric': {},                                                           # scenario.yaml 기본 (±0.16, TTC 0.35)
       'geometric_straight': {'aeb.curved_band': False},                          # B1 전 (직선 띠)
       'geometric_ttc055': {'aeb.ttc_threshold': 0.55},
       'geometric_unknown': {'aeb.speed_source': 'unknown'},                      # 속도 없음 → 고정 0.25 m
       'legacy_zero': {'aeb.model': 'legacy', 'aeb.speed_source': 'zero', 'aeb.path_margin': 0.15, 'aeb.ttc_threshold': 0.55},
       'legacy_true': {'aeb.model': 'legacy', 'aeb.speed_source': 'measured', 'aeb.path_margin': 0.15, 'aeb.ttc_threshold': 0.55}}


def cfg(args, **over):
    return load_cfg(**dict(AEB[args.aeb], **{'track.W': args.W}, **over))


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else float('nan')


def mn(xs):
    xs = [x for x in xs if x is not None]
    return min(xs) if xs else float('nan')


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
                sum(bool(r['collision']) for r in rs), mean([r['h_min'] for r in rs]), mn([r['gap_min'] for r in rs]),
                mean([r['T_id'] for r in rs if r['T_id'] is not None]),
                mean([r['T_pass'] for r in rs if r['T_pass'] is not None])))
    save('matrix', rows)


def aeb(args):
    """AEB 조건 × W × 방법 × 선행차 Y/NY."""
    rows = []
    print('| W [m] | AEB | 방법 | 선행차 | DONE | 교착 | 차량 충돌 | 차-벽 충돌 | ego N_AEB/회 | 선행차 N_AEB/회 | 급정지 전 h_min | 정지 간격 ≥ m |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|')
    for W in args.Ws:
        args.W = W
        for cond in args.conds:
            args.aeb = cond
            for m in args.methods:
                for beh in ('Y', 'NY'):
                    rs = [Sim(m, cfg(args, **{'leader.behavior': beh}), seed=k).run() for k in range(args.n)]
                    for r in rs:
                        r['aeb_cond'] = cond
                    rows += rs
                    nb = [r for r in rs if r['brake_fired']]
                    print('| %.1f | %s | %s | %s | %d | %d | %d | %d | %.2f | %.2f | %.3f | %s |' % (
                        W, cond, m, beh, sum(r['success'] for r in rs), sum(r['deadlock'] for r in rs),
                        sum(r['collision'] == 'cars' for r in rs),
                        sum(r['collision'] in ('ego_wall', 'leader_wall') for r in rs),
                        mean([r['N_AEB'] for r in rs]), mean([r['N_AEB_leader'] for r in rs]),
                        mn([r['h_min'] for r in rs]),
                        ('%d/%d' % (sum(r['stop_gap_ok'] for r in nb), len(nb))) if nb else '-'))
    save('aeb', rows)


def yrw(args):
    """양보 후 복귀(YR) × W × 선행차 W 고정 on/off. AEB는 --aeb (기본 geometric)."""
    rows = []
    print('AEB: %s, d_y ~ U[0.8, 1.2]. "겹친 뒤 복귀" = 후행차가 이미 옆에 온 뒤 복귀 (시뮬 전용 시나리오)' % args.aeb)
    print('| W [m] | 선행차 W 고정 | 방법 | 복귀 [s] | DONE | 추월 중단 | 판정 NY | 교착 | 차량 충돌 | 선행차-벽 | ego-벽 | 겹친 뒤 복귀 회차 |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|')
    for W in args.Ws:
        args.W = W
        for fix in (False, True):
            for m in args.methods:
                for ra in args.returns:
                    rng = random.Random(0)
                    rs = [Sim(m, cfg(args, **{'leader.behavior': 'YR', 'leader.return_after': ra, 'leader.fix_W': fix,
                                              'leader.d_y': rng.uniform(0.8, 1.2)}), seed=k).run()
                          for k in range(args.n)]
                    for r in rs:
                        r.update(return_after=ra, fix_W=fix)
                    rows += rs
                    print('| %.1f | %s | %s | %.1f | %d | %d | %d | %d | %d | %d | %d | %d |' % (
                        W, 'on' if fix else 'off', m, ra, sum(r['success'] for r in rs),
                        sum(r['n_abort'] > 0 and not r['collision'] for r in rs),
                        sum(r['decided'] == 'NY' for r in rs), sum(r['deadlock'] for r in rs),
                        sum(r['collision'] == 'cars' for r in rs), sum(r['collision'] == 'leader_wall' for r in rs),
                        sum(r['collision'] == 'ego_wall' for r in rs), sum(r['return_after_overlap'] for r in rs)))
    save('yrw', rows)


def treact_ny(args):
    """t_react별 NY 쪽 비용: NY 확정 시간, 최소 간격, 급정지."""
    rows = []
    vals = (0.0, 0.5, 1.0, 1.3, 1.6)
    print('AEB: %s, W = %.2f, NY 회차 (급정지 이벤트 켜짐)' % (args.aeb, args.W))
    print('| 방법 | t_react [s] | NY 확정 | T_id 평균 [s] | T_id 최대 [s] | 최소 간격 [m] | 급정지 발생 | 정지 간격 ≥ m | 급정지 전 h_min | 차량 충돌 | PASS 진입 | ego N_AEB/회 |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|')
    for m in args.methods:
        for v in vals:
            rs = [Sim(m, cfg(args, **{'leader.behavior': 'NY'}), seed=k, t_react=v).run() for k in range(args.n)]
            for r in rs:
                r['t_react'] = v
            rows += rs
            ids = [r['T_id'] for r in rs if r['decided'] == 'NY']
            nb = [r for r in rs if r['brake_fired']]
            print('| %s | %.1f | %d/%d | %.2f | %.2f | %.3f | %d | %d/%d | %.3f | %d | %d | %.2f |' % (
                m, v, len(ids), len(rs), mean(ids), max(ids) if ids else float('nan'),
                mn([r['gap_min'] for r in rs]), len(nb), sum(r['stop_gap_ok'] for r in nb), len(nb),
                mn([r['h_min'] for r in rs]), sum(r['collision'] == 'cars' for r in rs),
                sum(r['pass_entered'] for r in rs), mean([r['N_AEB'] for r in rs])))
    save('treact_ny', rows)


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
                mn([r['gap_min'] for r in rs])))
    save('yr', rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('exp', choices=['belief', 'yaw', 'matrix', 'aeb', 'treact', 'yr', 'yrw', 'treact_ny'])
    ap.add_argument('--Ws', type=float, nargs='*', default=[1.0, 1.1, 1.2])
    ap.add_argument('--conds', nargs='*', default=['geometric', 'geometric_ttc055', 'geometric_unknown'])
    ap.add_argument('--returns', type=float, nargs='*', default=[1.0, 2.0, 3.0])
    ap.add_argument('--aeb', default='off', choices=list(AEB))
    ap.add_argument('--n', type=int, default=None)
    ap.add_argument('--methods', default=None)
    ap.add_argument('--W', type=float, default=1.0)
    ap.add_argument('--yaws', type=float, nargs='*', default=[0, 10, 20, 30])
    ap.add_argument('--d-ys', type=float, nargs='*', default=[0.8, 1.0, 1.2])
    args = ap.parse_args()
    args.n = args.n or {'belief': 200, 'matrix': 5, 'yaw': 1, 'aeb': 20, 'treact': 200, 'yr': 50, 'yrw': 20,
                        'treact_ny': 200}[args.exp]
    args.methods = args.methods or {'belief': 'cd', 'yaw': 'd', 'matrix': 'abcd'}.get(args.exp, 'cd')
    {'belief': belief, 'yaw': yaw, 'matrix': matrix, 'aeb': aeb, 'treact': treact, 'yr': yr, 'yrw': yrw,
     'treact_ny': treact_ny}[args.exp](args)


if __name__ == '__main__':
    main()
