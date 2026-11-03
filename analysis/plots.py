#!/usr/bin/env python3
"""그림 5 (위치별 속도), 그림 6 (안전-효율 평면), RQ4 (경계 흔들림 vs N).

    python3 analysis/plots.py ~/exp_logs/20261110 [...] --out ~/exp_logs/analysis

색: 방법 4개 = 범주형 고정 순서 (B0 파랑, B1 주황, B2 청록, P 노랑). 시나리오·필터와 무관하게 방법마다 같은 색.
    밝은 배경 대비가 3:1 미만인 색(청록·노랑)이 있어 모든 계열에 직접 라벨과 마커 모양을 함께 쓰고,
    같은 수치는 summary.md 표로 제공한다 (색만으로 구분하지 않음).
"""

import argparse
import collections
import csv
import json
import os

import numpy as np

from my_algo.run_paths import METHODS, log_root

COLORS = {'B0': '#2a78d6', 'B1': '#eb6834', 'B2': '#1baf7a', 'P': '#eda100'}
MARKERS = {'B0': 'o', 'B1': 's', 'B2': '^', 'P': 'D'}
LINESTYLES = {'B0': '-', 'B1': '--', 'B2': '-.', 'P': '-'}
LABEL_OFFSET = {'B1': (-22, -4)}  # 최저점이 가까운 B1·P 라벨이 겹치지 않게 B1은 왼쪽
INK, MUTED, GRID = '#0b0b0b', '#52514e', '#e4e3df'


def style(ax):
    ax.set_facecolor('#fcfcfb')
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def load_runs(dirs):
    runs = []
    for d in dirs:
        for name in sorted(os.listdir(d)):
            adir = os.path.join(d, name, 'analysis')
            if not os.path.exists(os.path.join(adir, 'metrics.json')):
                continue
            with open(os.path.join(adir, 'metrics.json'), encoding='utf-8') as f:
                m = json.load(f)
            m['run_dir'] = os.path.join(d, name)
            if m.get('static'):            # Exp1 정지 회차: RQ4 그림에만 쓴다
                runs.append(m)
                continue
            if m['discard'] or m.get('aeb_malfunction'):   # 폐기·AEB 오작동 회차는 효율 그림에서 제외
                continue
            with open(os.path.join(adir, 'speed_vs_s.csv'), encoding='utf-8') as f:
                rows = list(csv.DictReader(f))
            m['s'] = np.array([float(r['s']) for r in rows])
            m['v_raw'] = np.array([float(r['v_raw']) for r in rows])
            m['v_meas'] = np.array([float(r['v_meas']) for r in rows])
            m['run_dir'] = os.path.join(d, name)
            runs.append(m)
    return runs


def fig5(runs, out, plt):
    """S2, S3: 방법별 위치별 명령 속도(v_raw) 평균, 실측은 점선 없이 옅은 범위로."""
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), sharey=True)
    for ax, scen in zip(axes, ('S2', 'S3')):
        style(ax)
        for method in METHODS:
            sel = [r for r in runs if r['scenario'] == scen and r['method'] == method]
            if not sel:
                continue
            n = min(len(r['s']) for r in sel)
            s = sel[0]['s'][:n]
            v = np.nanmean(np.vstack([r['v_raw'][:n] for r in sel]), axis=0)
            vm = np.vstack([r['v_meas'][:n] for r in sel])
            ax.fill_between(s, np.nanmin(vm, axis=0), np.nanmax(vm, axis=0),
                            color=COLORS[method], alpha=0.10, linewidth=0)
            ax.plot(s, v, color=COLORS[method], linewidth=2, linestyle=LINESTYLES[method],
                    label=f'{method} (n={len(sel)})')
            # 최저점은 가속이 끝난 뒤(처음 0.99 v_ref에 닿은 이후)에서 찾는다
            reached = np.flatnonzero(v >= 0.99 * np.nanmax(v))
            start = int(reached[0]) if len(reached) else 0
            i = start + int(np.nanargmin(v[start:]))
            if v[i] < 0.97 * np.nanmax(v):   # 감속이 거의 없으면 범례로 충분
                ax.plot(s[i], v[i], marker=MARKERS[method], color=COLORS[method], markersize=8,
                        markeredgecolor='#fcfcfb', markeredgewidth=2)
                ax.annotate(method, (s[i], v[i]), textcoords='offset points',
                            xytext=LABEL_OFFSET.get(method, (8, -4)), color=INK, fontsize=9)
        ax.set_title(f'{scen}: 위치별 명령 속도 (선) · 실측 범위 (띠)', color=INK, fontsize=11, loc='left')
        ax.set_xlabel('주행 거리 s [m]', color=MUTED)
    axes[0].set_ylabel('속도 [m/s]', color=MUTED)
    axes[1].legend(frameon=False, labelcolor=INK, loc='lower right')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig5_speed_profile.png'), dpi=200)
    fig.savefig(os.path.join(out, 'fig5_speed_profile.pdf'))


def fig6(runs, out, plt):
    """x: S3+S4 명령 기준 ΔT (불필요 감속), y: S1 무충돌 비율. S2 손실은 오른쪽 패널에 따로."""
    agg = collections.defaultdict(dict)
    for method in METHODS:
        def mean(scen, key):
            vals = [r[key] for r in runs if r['scenario'] == scen and r['method'] == method
                    and r.get(key) is not None]
            return float(np.mean(vals)) if vals else None
        s3, s4, s2 = mean('S3', 'dT_cmd'), mean('S4', 'dT_cmd'), mean('S2', 'dT_cmd')
        col = [r['collision'] for r in runs if r['scenario'] == 'S1' and r['method'] == method
               and r.get('collision') is not None]
        agg[method] = {'x': None if s3 is None or s4 is None else s3 + s4, 's2': s2,
                       'y': col.count(False) / len(col) if col else None, 'n1': len(col)}
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10, 4), gridspec_kw={'width_ratios': [3, 1.2]})
    style(ax)
    style(ax2)
    for method, a in agg.items():
        if a['x'] is None or a['y'] is None:
            continue
        ax.plot(a['x'], a['y'], marker=MARKERS[method], color=COLORS[method], markersize=10,
                markeredgecolor='#fcfcfb', markeredgewidth=2, linestyle='none')
        ax.annotate(f'{method}', (a['x'], a['y']), textcoords='offset points', xytext=(8, 4),
                    color=INK, fontsize=10)
    ax.set_xlabel('불필요 감속 손실 ΔT(S3) + ΔT(S4), 명령 기준 [s]', color=MUTED)
    ax.set_ylabel('S1 무충돌 비율', color=MUTED)
    ax.set_ylim(-0.05, 1.05)
    ax.set_title('안전-효율 평면 (왼쪽 위가 좋음)', color=INK, fontsize=11, loc='left')
    names = [m for m in METHODS if agg[m]['s2'] is not None]
    ax2.barh(names, [agg[m]['s2'] for m in names], color=[COLORS[m] for m in names], height=0.6)
    for i, m in enumerate(names):
        ax2.annotate(f'{agg[m]["s2"]:.2f}', (agg[m]['s2'], i), textcoords='offset points',
                     xytext=(4, -3), color=INK, fontsize=9)
    ax2.invert_yaxis()
    ax2.set_xlabel('ΔT(S2) [s]', color=MUTED)
    ax2.set_title('S2: 정당한 감속 비용', color=INK, fontsize=11, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig6_safety_efficiency.png'), dpi=200)
    fig.savefig(os.path.join(out, 'fig6_safety_efficiency.pdf'))
    with open(os.path.join(out, 'fig6_points.json'), 'w', encoding='utf-8') as f:
        json.dump(agg, f, indent=2)


def fig_rq4(runs, out, plt):
    """회차별 경계 흔들림(표준편차)을 그 회차의 n_frames 값별로. n_frames는 config_snapshot/params에서."""
    pts = collections.defaultdict(list)
    for r in runs:
        if r.get('boundary_y_std_mean') is None:
            continue
        n = None
        params = os.path.join(r['run_dir'], 'params.yaml')
        if os.path.exists(params):
            for line in open(params, encoding='utf-8'):
                if line.strip().startswith('n_frames:'):
                    n = int(line.split(':')[1])
                    break
        if n is None:
            import yaml
            with open(os.path.join(r['run_dir'], 'config_snapshot', 'common.yaml')) as f:
                n = yaml.safe_load(f)['/**']['ros__parameters']['n_frames']
        pts[n].append(r['boundary_y_std_mean'])
    if not pts:
        return
    fig, ax = plt.subplots(figsize=(5, 3.5))
    style(ax)
    ns = sorted(pts)
    means = [np.mean(pts[n]) for n in ns]
    for n in ns:
        ax.plot([n] * len(pts[n]), np.array(pts[n]) * 100, 'o', color=COLORS['B0'], alpha=0.35,
                markersize=8, markeredgewidth=0)
    ax.plot(ns, np.array(means) * 100, '-', color=COLORS['B0'], linewidth=2)
    ax.set_xlabel('누적 프레임 수 N', color=MUTED)
    ax.set_ylabel('가림 경계 위치 표준편차 [cm]', color=MUTED)
    ax.set_title('RQ4: 경계 흔들림', color=INK, fontsize=11, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_rq4_boundary.png'), dpi=200)


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    available = {f.name for f in font_manager.fontManager.ttflist}
    korean = [f for f in ('NanumGothic', 'Noto Sans CJK KR', 'AppleGothic', 'Malgun Gothic')
              if f in available]
    plt.rcParams['font.family'] = korean[:1] + ['DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    if not korean:
        print('경고: 한글 글꼴 없음 (sudo apt install fonts-nanum). 라벨이 깨질 수 있음')
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('session_dirs', nargs='+')
    ap.add_argument('--out', default=os.path.join(log_root(), 'analysis'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    runs = load_runs(args.session_dirs)
    driving = [r for r in runs if not r.get('static')]
    fig5(driving, args.out, plt)
    fig6(driving, args.out, plt)
    fig_rq4(runs, args.out, plt)
    print(f'그림 저장: {args.out}')


if __name__ == '__main__':
    main()
