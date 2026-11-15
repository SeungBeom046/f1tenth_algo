#!/usr/bin/env python3
"""선행차 횡위치 추정 오차와 '가짜 복귀 상승' 측정 (B2 사전 측정, 2026-11-15).

    python sim/mujoco_overtake/analysis/yb_bias.py collect runs.json --no-abort     # Y 160 + YR 160회, 약 4분
    python sim/mujoco_overtake/analysis/yb_bias.py report runs.json

collect: 방법 c·d × W 1.0·1.2 × AEB on/off. Y는 seed 20개, YR(복귀 1.0·2.0 s)은 seed 10개.
  --no-abort: 이 스크립트 안에서만 PASS 중단을 끈다 (중단되면 그 뒤 데이터가 없어지므로). 저장소 코드는 그대로.
  scan마다 ego.track의 y_b(중심), y_l(왼쪽 모서리, 있으면)과 참값을 기록한다. Sim.log는 건드리지 않는다
  (baseline.py 해시가 그대로 남도록).
report: Y 회차의 추정 − 참값 분포, 가짜 복귀 상승 (추정 − 그때까지 추정 최솟값, 선행차는 돌아오지 않으므로 전부 가짜),
  YR 회차의 실제 복귀 변위, 임계 θ·연속 시간 T 후보별 오판·감지·겹침 여유. y_B와 y_L을 나란히.
"""
import argparse
import json
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from sim import Sim, corners, load_cfg       # noqa: E402
from my_algo.aur import ego_fsm               # noqa: E402

DT = 0.1
TH = (0.08, 0.10, 0.12, 0.14, 0.16, 0.20)


def disable_abort():
    def no_abort(self, obs, W):
        self._y_b_most_yielded = min(self._y_b_most_yielded, obs.y_b)
        if obs.d <= 0.0:
            self._overlapped = True
        return False
    ego_fsm.EgoFSM._check_pass_abort = no_abort


def run_one(method, over, seed):
    sim = Sim(method, load_cfg(**over), seed=seed)
    extra = []
    orig = sim._log_frame

    def log_frame(*a):
        orig(*a)
        trk = sim.ego.track
        lead = corners(*sim.leader.kin.pose(), sim.cfg)
        extra.append((getattr(trk, 'y_l', None), max(y for _, y in lead)))
    sim._log_frame = log_frame
    r = sim.run()
    series = [(x['t'], x['state'], x['gap'], x['valid'], x['y_b'], x['y_b_true'], yl, ylt)
              for x, (yl, ylt) in zip(sim.log, extra)]
    t_pass = next((x['t'] for x in sim.log if x['state'] == 'PASS'), None)
    return dict(final=r['final_state'], coll=r['collision'], t_pass=t_pass, t_yield=sim.logic.t_yield, series=series)


def collect(args):
    if args.no_abort:
        disable_abort()
    rows = []
    for beh in ('Y', 'YR'):
        for m in 'cd':
            for W in (1.0, 1.2):
                for aeb in (True, False):
                    for ra in ((None,) if beh == 'Y' else (1.0, 2.0)):
                        for k in range(args.n_y if beh == 'Y' else args.n_yr):
                            over = {'leader.behavior': beh, 'track.W': W, 'aeb.enabled': aeb}
                            if ra is not None:
                                over['leader.return_after'] = ra
                            rows.append(dict(beh=beh, method=m, W=W, aeb=aeb, ra=ra, seed=k, **run_one(m, over, k)))
    with open(args.out, 'w') as f:
        json.dump(rows, f)
    print('%d회 저장: %s' % (len(rows), args.out))


# ------------------------------------------------------------ report
def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * (len(xs) - 1) + 0.5))] if xs else float('nan')


def pass_part(r):
    """PASS 진입부터 앞뒤로 겹치기 전(참 간격 > 0)까지. (t, valid, y_b, y_b_true, y_l, y_l_true)"""
    out = []
    for t, state, gap, valid, yb, ybt, yl, ylt in r['series']:
        if r['t_pass'] is not None and t >= r['t_pass'] and state == 'PASS':
            if gap <= 0:
                break
            out.append((t, valid, yb, ybt, yl, ylt))
    return out


def rise_series(part, which):
    """(t, 추정 − 지금까지 추정 최솟값, 추정 − 참값). which = 'b' | 'l'."""
    i = 2 if which == 'b' else 4
    m, out = None, []
    for row in part:
        est, true = row[i], row[i + 1]
        if row[1] and est is not None:
            m = est if m is None else min(m, est)
            out.append((row[0], est - m, est - true))
    return out


def first_fire(s, th, need):
    cur = 0
    for t, x, _ in s:
        cur = cur + 1 if x > th else 0
        if cur >= need:
            return t
    return None


def report(args):
    rows = json.load(open(args.runs))
    have_l = any(row[6] is not None for r in rows for row in r['series'])
    kinds = ('b', 'l') if have_l else ('b',)
    name = {'b': 'y_B 중심', 'l': 'y_L 왼쪽 모서리'}
    ys = [r for r in rows if r['beh'] == 'Y' and r['t_pass'] is not None]
    print('## Y 회차: PASS 진입 ~ 겹치기 전 (%d회 PASS 진입 / %d)' % (len(ys), sum(r['beh'] == 'Y' for r in rows)))
    print('| 추정량 | 방법 | W | 추정 − 참값: 최소 / 5% / 95% / 최대 [m] | 가짜 상승 최대 / 95% [m] |')
    print('|---|---|---|---|---|')
    for k in kinds:
        for m in 'cd':
            for W in (1.0, 1.2):
                sel = [r for r in ys if r['method'] == m and r['W'] == W]
                errs, rises = [], []
                for r in sel:
                    s = rise_series(pass_part(r), k)
                    errs += [e for _, _, e in s]
                    rises.append(max([x for _, x, _ in s] or [0.0]))
                print('| %s | %s | %.1f | %.3f / %.3f / %.3f / %.3f | %.3f / %.3f |' % (
                    name[k], m, W, min(errs), pct(errs, .05), pct(errs, .95), max(errs), max(rises), pct(rises, .95)))
    print('\n가짜 상승이 θ를 넘는 회차 수와 연속 최장 [s] (Y, 선행차는 돌아오지 않음):')
    print('| 추정량 | ' + ' | '.join('> %.2f' % t for t in TH) + ' |')
    print('|---' * (1 + len(TH)) + '|')
    for k in kinds:
        cells = []
        for t in TH:
            durs = []
            for r in ys:
                best = cur = 0
                for _, x, _ in rise_series(pass_part(r), k):
                    cur = cur + 1 if x > t else 0
                    best = max(best, cur)
                durs.append(best * DT)
            cells.append('%d회, %.1f s' % (sum(d > 0 for d in durs), max(durs)))
        print('| %s | %s |' % (name[k], ' | '.join(cells)))

    yr = []
    for r in rows:
        if r['beh'] != 'YR' or r['t_pass'] is None or r['t_yield'] is None:
            continue
        t_ret = r['t_yield'] + r['ra']
        part = pass_part(r)
        if part and part[0][0] <= t_ret <= part[-1][0]:
            yr.append((r, part, t_ret))
    print('\n## YR 회차: 복귀 시작(t_yield + return_after)이 PASS 중·겹치기 전인 %d회' % len(yr))
    print('참 변위 (복귀 시작 대비, 중앙값) [m]:')
    taus = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
    print('| 참값 | W | ' + ' | '.join('+%.1f s' % t for t in taus) + ' |')
    print('|---|---|' + '---|' * len(taus))
    for k in kinds:
        for W in (1.0, 1.2):
            cells = []
            for tau in taus:
                vals = []
                for r, part, t_ret in yr:
                    if r['W'] != W:
                        continue
                    i = 3 if k == 'b' else 5
                    tt = {round(row[0], 2): row[i] for row in part}
                    t0 = round(min(tt, key=lambda t: abs(t - t_ret)), 2)
                    tk = round(t0 + tau, 2)
                    if tk in tt:
                        vals.append(tt[tk] - tt[t0])
                cells.append('%+.3f' % st.median(vals) if vals else '-')
            print('| %s | %.1f | %s |' % (name[k], W, ' | '.join(cells)))
    print('\n임계 θ·연속 시간 T 후보: Y 오판 / YR 겹치기 전 감지 / 복귀 시작→감지 중앙값 / 감지→겹침 여유 중앙값·최소')
    print('| 추정량 | θ [m] | T [s] | Y 오판 | YR 감지 | 감지 지연 [s] | 겹침 여유 [s] |')
    print('|---|---|---|---|---|---|---|')
    for k in kinds:
        for th in (0.08, 0.12, 0.16, 0.20):
            for T in (0.0, 0.3):
                need = int(round(T / DT)) + 1
                fy = sum(first_fire(rise_series(pass_part(r), k), th, need) is not None for r in ys)
                lags, margins = [], []
                for r, part, t_ret in yr:
                    t = first_fire(rise_series(part, k), th, need)
                    if t is not None and t >= t_ret - 0.05:
                        lags.append(t - t_ret)
                        margins.append(part[-1][0] - t)
                print('| %s | %.2f | %.1f | %d/%d | %d/%d | %s | %s |' % (
                    name[k], th, T, fy, len(ys), len(lags), len(yr),
                    '%.2f' % st.median(lags) if lags else '-',
                    '%.2f / %.2f' % (st.median(margins), min(margins)) if margins else '-'))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('collect')
    c.add_argument('out')
    c.add_argument('--no-abort', action='store_true')
    c.add_argument('--n-y', type=int, default=20)
    c.add_argument('--n-yr', type=int, default=10)
    r = sub.add_parser('report')
    r.add_argument('runs')
    args = ap.parse_args()
    collect(args) if args.cmd == 'collect' else report(args)


if __name__ == '__main__':
    main()
