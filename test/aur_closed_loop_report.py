"""노이즈 없는 폐루프(참값 관측) 재실험: belief 2종 × 방법, 실패 원인과 d_y 구간별 성공률.

실행: python test/aur_closed_loop_report.py [--n 200] [--methods cd]
"""
import argparse
import os
import random
import sys
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from _aur_synth import run                      # noqa: E402
from my_algo.aur.params import AurParams        # noqa: E402

CAUSES = ('belief_NY', 'T_wait_a', 'T_max', 'track_lost', 'other')
BINS = [(0.6, 0.7), (0.7, 0.8), (0.8, 0.9), (0.9, 1.0), (1.0, 1.1), (1.1, 1.2)]


def classify(fsm, log):
    if log[-1]['state'] == 'DONE':
        return None
    if fsm.follow_reason == 'belief_NY':
        return 'belief_NY'
    if fsm.follow_reason == 'T_max':
        # 방법 a는 T_wait_a가 지나도 횡 여유가 없어 기다리다 T_max에 걸린다
        return 'T_wait_a' if fsm.method == 'a' else 'T_max'
    if any(not r['tracked'] for r in log if r['state'] in ('APPROACH', 'PROBE')):
        return 'track_lost'
    return 'other'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--methods', default='cd')
    args = ap.parse_args()
    base = AurParams()
    print('| belief | 방법 | 성공 | ' + ' | '.join(CAUSES) + ' | ' +
          ' | '.join('d_y %.1f~%.1f' % b for b in BINS) + ' |')
    print('|---' * (3 + len(CAUSES) + len(BINS)) + '|')
    for model in ('original', 'survival'):
        p = replace(base, belief_model=model)
        for m in args.methods:
            rng = random.Random(0)      # 모든 조합이 같은 d_y 200개를 쓴다
            fails = dict.fromkeys(CAUSES, 0)
            ok = 0
            bins = [[0, 0] for _ in BINS]
            for _ in range(args.n):
                d_y = rng.uniform(p.d_y_min, p.d_y_max)
                fsm, _, log = run(p, m, 'Y', d_y=d_y)
                cause = classify(fsm, log)
                ok += cause is None
                if cause:
                    fails[cause] += 1
                for k, (lo, hi) in enumerate(BINS):
                    if lo <= d_y < hi or (k == len(BINS) - 1 and d_y == hi):
                        bins[k][0] += cause is None
                        bins[k][1] += 1
            print('| %s | %s | %d/%d | ' % (model, m, ok, args.n) +
                  ' | '.join(str(fails[c]) for c in CAUSES) + ' | ' +
                  ' | '.join('%d/%d' % tuple(b) for b in bins) + ' |')

    print('\n급정지(방법 d, NY) 최소 h와 최종 간격, 제어 주기별:')
    for hz in (10, 20, 50, 200):
        fsm, lead, log = run(base, 'd', 'NY', dt=1.0 / hz)
        post = [r for r in log if lead.t_brake is not None and r['t'] >= lead.t_brake]
        pre = [r for r in log if r['state'] != 'WAIT' and (lead.t_brake is None or r['t'] < lead.t_brake)]
        print('  %3d Hz: 급정지 전 h_min %+.3f, 급정지 후 h_min %+.3f, 최종 간격 %.3f m' % (
            hz, min(r['h'] for r in pre), min(r['h'] for r in post), log[-1]['d']))


if __name__ == '__main__':
    main()
