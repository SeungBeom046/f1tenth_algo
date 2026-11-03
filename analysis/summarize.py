#!/usr/bin/env python3
"""모든 회차 metrics.json → 시나리오·방법별 평균과 95% CI 표, 폐기 수 (~/exp_logs/analysis/).

    python3 analysis/summarize.py ~/exp_logs/20261110 [~/exp_logs/20261111 ...]

규칙 (2.11절):
    - 시나리오별로 분리해 보고한다. S2/S3/S4 평균은 내지 않는다 (S2는 정당한 감속, S3·S4는 과잉 보수성).
    - 명령 기준(dT_cmd)과 실측 기준(dT_meas)을 따로 낸다. 논문 주장의 핵심 근거는 명령 기준.
    - 폐기 회차(v_ref 미도달, 조이스틱 끊김, 수동 폐기)는 통계에서 빼고 폐기 수와 사유를 함께 낸다.
    - S1은 d_e별로 나눈다. 안전 지표: 무충돌 비율, 최소 TTC, 최소 거리.
    - S2–S4에서 AEB가 발동한 회차는 'AEB 오작동'으로 방법별로 따로 세고, 효율 통계에서 빼며 뺀 수를 함께 낸다.
"""

import argparse
import collections
import csv
import json
import os
import sys

from my_algo.metrics import mean_ci95
from my_algo.run_paths import METHODS, log_root

EFFICIENCY = ['dT_cmd', 'dT_meas', 'window_loss_pct_cmd', 'min_speed_cmd', 'min_speed_meas',
              'decel_length_cmd', 'decel_length_meas', 'jerk_rms']
SAFETY = ['min_distance', 'min_ttc', 'min_ego_ttc']
RQ4 = ['boundary_y_std_mean', 'phantom_count_changes', 'phantom_on_off_toggles']


def load(dirs):
    runs = []
    for d in dirs:
        for name in sorted(os.listdir(d)):
            p = os.path.join(d, name, 'analysis', 'metrics.json')
            if os.path.exists(p):
                with open(p, encoding='utf-8') as f:
                    runs.append(json.load(f))
    return runs


def condition(r):
    return (r['scenario'], 'na' if r['d_e'] is None else f'{r["d_e"]:.1f}')


def summarize(runs):
    groups = collections.defaultdict(list)
    for r in runs:
        groups[condition(r) + (r['method'],)].append(r)
    rows = []
    for (scen, de, method), rs in sorted(groups.items(),
                                         key=lambda kv: (kv[0][0], kv[0][1], METHODS.index(kv[0][2]))):
        kept = [r for r in rs if not r['discard']]
        reasons = collections.Counter(r['discard_reason'] for r in rs if r['discard'])
        malfunction = [r for r in kept if r.get('aeb_malfunction')]
        eff = [r for r in kept if not r.get('aeb_malfunction')]
        row = {'scenario': scen, 'd_e': de, 'method': method, 'n_total': len(rs), 'n_used': len(eff),
               'n_discarded': len(rs) - len(kept),
               'n_aeb_malfunction': len(malfunction),
               'discard_reasons': ';'.join(f'{k}:{v}' for k, v in reasons.items())}
        for k in EFFICIENCY + RQ4:
            mean, half, n = mean_ci95([r.get(k) for r in eff])
            row[k] = mean
            row[f'{k}_ci95'] = half
            row[f'{k}_n'] = n
        for k in (SAFETY if scen == 'S1' else []):
            mean, half, n = mean_ci95([r.get(k) for r in kept])
            row[k] = mean
            row[f'{k}_ci95'] = half
            row[f'{k}_n'] = n
        if scen == 'S1':
            col = [r.get('collision') for r in kept if r.get('collision') is not None]
            row['collision_free_ratio'] = (col.count(False) / len(col)) if col else None
            row['n_collisions'] = col.count(True)
            impacts = [r['impact_speed'] for r in kept if r.get('impact_speed') is not None]
            row['impact_speed_mean'] = (sum(impacts) / len(impacts)) if impacts else None
        rows.append(row)
    return rows


def fmt(v, half=None):
    if v is None:
        return '–'
    return f'{v:.3f}' + (f' ± {half:.3f}' if half is not None else '')


def write(rows, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in ('scenario', 'd_e', 'method'), k))
    with open(os.path.join(out_dir, 'summary.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    lines = ['# 회차 요약 (평균 ± 95% CI, 폐기 회차 제외)', '',
             'ΔT 단위 s. 명령 기준(cmd) = min(v_ref, v_cap), 실측 기준(meas) = odom 속도.', '']
    for scen in ('S1', 'S2', 'S3', 'S4'):
        sel = [r for r in rows if r['scenario'] == scen]
        if not sel:
            continue
        lines += [f'## {scen}', '']
        if scen == 'S1':
            lines += ['| d_e | 방법 | 사용/전체 | 무충돌 비율 | 최소 TTC [s] | 최소 거리 [m] | ΔT cmd | ΔT meas | 폐기 사유 |',
                      '|---|---|---|---|---|---|---|---|---|']
            for r in sel:
                lines.append(f'| {r["d_e"]} | {r["method"]} | {r["n_used"]}/{r["n_total"]} | '
                             f'{fmt(r["collision_free_ratio"])} | {fmt(r["min_ttc"], r["min_ttc_ci95"])} | '
                             f'{fmt(r["min_distance"], r["min_distance_ci95"])} | '
                             f'{fmt(r["dT_cmd"], r["dT_cmd_ci95"])} | {fmt(r["dT_meas"], r["dT_meas_ci95"])} | '
                             f'{r["discard_reasons"] or "–"} |')
        else:
            lines += ['| 방법 | 사용/전체 | AEB 오작동(제외) | ΔT cmd | ΔT meas | 최저 속도 cmd | 감속 구간 cmd [m] | jerk RMS | 폐기 사유 |',
                      '|---|---|---|---|---|---|---|---|---|']
            for r in sel:
                lines.append(f'| {r["method"]} | {r["n_used"]}/{r["n_total"]} | {r["n_aeb_malfunction"]} | '
                             f'{fmt(r["dT_cmd"], r["dT_cmd_ci95"])} | {fmt(r["dT_meas"], r["dT_meas_ci95"])} | '
                             f'{fmt(r["min_speed_cmd"], r["min_speed_cmd_ci95"])} | '
                             f'{fmt(r["decel_length_cmd"], r["decel_length_cmd_ci95"])} | '
                             f'{fmt(r["jerk_rms"], r["jerk_rms_ci95"])} | {r["discard_reasons"] or "–"} |')
        lines.append('')
    total = sum(r['n_discarded'] for r in rows)
    lines.append(f'폐기 회차 총 {total}개 (v_ref 미도달 포함).')
    lines += ['', '## AEB 오작동 (S2–S4, 돌출 없는데 AEB 발동) — 효율 통계에서 제외', '',
              '| 방법 | S2 | S3 | S4 | 합계 |', '|---|---|---|---|---|']
    for m in METHODS:
        counts = [sum(r['n_aeb_malfunction'] for r in rows if r['method'] == m and r['scenario'] == s)
                  for s in ('S2', 'S3', 'S4')]
        lines.append(f'| {m} | ' + ' | '.join(map(str, counts)) + f' | {sum(counts)} |')
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('session_dirs', nargs='+')
    ap.add_argument('--out', default=os.path.join(log_root(), 'analysis'))
    args = ap.parse_args()
    runs = load(args.session_dirs)
    if not runs:
        sys.exit('metrics.json이 없다. 먼저 analysis/extract_run.py --all 실행')
    write(summarize(runs), args.out)


if __name__ == '__main__':
    main()
