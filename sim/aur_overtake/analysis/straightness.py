#!/usr/bin/env python3
"""직진성·통로 폭 강건성 측정: W × 선행차(Y/NY) × seed, 라이다 잡음·서보 중심 오차를 넣고 돌린다.

    python sim/aur_overtake/analysis/straightness.py --Ws 1.0 2.0 3.0 --n 4 --noise 0.02 --offset 0.02
    python sim/aur_overtake/analysis/straightness.py --set heading_gain=1.0 wall_follow_gain=0.4

직진 구간 지표 (참값 기준):
  leader: 출발 1 s 뒤 ~ 양보 시작(Y) 또는 급정지(NY) 전. ego: 출발 1 s 뒤 ~ PASS 진입 전 (APPROACH·PROBE·FOLLOW).
  y_rms: 목표 횡위치(중앙 0) 대비 RMS [m], yaw_std [deg], steer_tv: scan당 평균 |Δ조향| [rad] (떨림)
"""
import argparse
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from sim import Sim, load_cfg  # noqa: E402


def run_one(job):
    W, behavior, seed, noise, offset, params, method = job
    cfg = load_cfg(**{'track.W': W, 'leader.behavior': behavior, 'lidar.range_noise_std': noise,
                      'vehicle.steer_offset': offset, 'sim.max_time': 30.0})
    sim = Sim(method, cfg, seed=seed, **params)
    trace = []

    def hook(s):
        if not s.log or (trace and trace[-1][0] == s.log[-1]['t']):
            return
        r = s.log[-1]
        trace.append((r['t'], r['state'], s.leader.kin.y, math.degrees(s.leader.kin.yaw), s.leader.cmd[1],
                      s.ego.kin.y, math.degrees(s.ego.kin.yaw), s.ego.cmd[1],
                      s.logic.yield_triggered or s.logic.t_armed is not None))
    res = sim.run(hook=hook)

    def stats(rows, yi, wi, si):
        if len(rows) < 5:
            return None, None, None
        ys = [r[yi] for r in rows]
        yaws = [r[wi] for r in rows]
        st = [r[si] for r in rows]
        mean_yaw = sum(yaws) / len(yaws)
        return (math.sqrt(sum(y * y for y in ys) / len(ys)),
                math.sqrt(sum((w - mean_yaw) ** 2 for w in yaws) / len(yaws)),
                sum(abs(a - b) for a, b in zip(st[1:], st)) / (len(st) - 1))
    lead_rows = [r for r in trace if r[0] >= 1.0 and not r[8]]
    ego_rows = [r for r in trace if r[0] >= 1.0 and r[1] in ('APPROACH', 'PROBE', 'FOLLOW')
                and not any(q[1] in ('PASS', 'RETREAT') for q in trace if q[0] <= r[0])]
    out = dict(W=W, behavior=behavior, seed=seed, success=res['success'], final=res['final_state'],
               cause=res['cause'], collision=res['collision'], deadlock=res['deadlock'], N_AEB=res['N_AEB'],
               N_AEB_leader=res['N_AEB_leader'], pass_entered=res['pass_entered'], T_pass=res['T_pass'],
               n_abort=res['n_abort'])
    out['lead_y_rms'], out['lead_yaw_std'], out['lead_steer_tv'] = stats(lead_rows, 2, 3, 4)
    out['ego_y_rms'], out['ego_yaw_std'], out['ego_steer_tv'] = stats(ego_rows, 5, 6, 7)
    return out


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else float('nan')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--Ws', type=float, nargs='*', default=[1.0, 2.0, 3.0])
    ap.add_argument('--n', type=int, default=4, help='W·선행차마다 seed 수')
    ap.add_argument('--noise', type=float, default=0.02, help='라이다 거리 잡음 표준편차 [m]')
    ap.add_argument('--offset', type=float, default=0.02, help='서보 중심 오차 [rad] (seed 홀짝으로 ± 번갈아)')
    ap.add_argument('--method', default='d')
    ap.add_argument('--set', nargs='*', default=[], metavar='K=V', help='AurParams 덮어쓰기')
    ap.add_argument('--jobs', type=int, default=os.cpu_count())
    args = ap.parse_args(argv)
    params = {k: yaml.safe_load(v) for k, v in (kv.split('=', 1) for kv in args.set)}
    jobs = [(W, b, s, args.noise, args.offset * (1 if s % 2 == 0 else -1), params, args.method)
            for W in args.Ws for b in ('Y', 'NY') for s in range(args.n)]
    with ProcessPoolExecutor(args.jobs) as ex:
        rows = list(ex.map(run_one, jobs))
    print('설정:', params or '(기본)', '| noise', args.noise, '| offset ±', args.offset, '| method', args.method)
    print('| W | 선행차 | 성공 | 충돌 | 교착 | ego AEB/회 | 선행차 AEB/회 | NY 추월시작 | 추월중단 | T_pass | '
          'leader y_rms | leader yaw_std | leader 조향떨림 | ego y_rms | ego yaw_std | ego 조향떨림 |')
    print('|' + '---|' * 16)
    for W in args.Ws:
        for b in ('Y', 'NY'):
            g = [r for r in rows if r['W'] == W and r['behavior'] == b]
            n = len(g)
            ok = sum(r['success'] for r in g) if b == 'Y' else sum(r['final'] == 'FOLLOW' and not r['collision'] for r in g)
            print('| %.1f | %s | %d/%d | %d | %d | %.2f | %.2f | %d | %d | %s | %.3f | %.2f | %.4f | %.3f | %.2f | %.4f |' % (
                W, b, ok, n, sum(bool(r['collision']) for r in g), sum(r['deadlock'] for r in g),
                mean(r['N_AEB'] for r in g), mean(r['N_AEB_leader'] for r in g),
                sum(r['pass_entered'] for r in g) if b == 'NY' else 0, sum(r['n_abort'] for r in g),
                '%.1f' % mean(r['T_pass'] for r in g) if b == 'Y' else '-',
                mean(r['lead_y_rms'] for r in g), mean(r['lead_yaw_std'] for r in g), mean(r['lead_steer_tv'] for r in g),
                mean(r['ego_y_rms'] for r in g), mean(r['ego_yaw_std'] for r in g), mean(r['ego_steer_tv'] for r in g)))
    bad = [r for r in rows if r['collision'] or r['deadlock'] or (r['behavior'] == 'Y' and not r['success'])]
    for r in bad:
        print('  실패:', {k: r[k] for k in ('W', 'behavior', 'seed', 'final', 'cause', 'collision', 'deadlock', 'n_abort')})


if __name__ == '__main__':
    main()
