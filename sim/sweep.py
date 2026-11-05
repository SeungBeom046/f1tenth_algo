#!/usr/bin/env python3
"""시뮬 일괄 실행: 시나리오 × 방법 × d_e × 시드 → 요약 표, 그림 5(위치별 속도)·그림 6(안전-효율 평면) 시뮬판.

    sim/.venv/bin/python sim/sweep.py --seeds 0 1 2                       # 전체 (S1 3시점 + S2–S4, 4방법)
    sim/.venv/bin/python sim/sweep.py --scenarios S3 --methods P --w-min 0.25 0.35 0.45 --seeds 0 1 --name wmin
    sim/.venv/bin/python sim/sweep.py --scenarios S2 S3 --methods B2 P --v-imp 0.867 1.0 1.2 --name vimp

    sim/.venv/bin/python sim/sweep.py --analyze sim_logs/20261105/sweep_main                # 다시 돌리지 않고 표·그림만

결과: sim_logs/<날짜>/sweep_<name>/<회차>/ 와 sweep_<name>/analysis/ (summary.md/csv, fig5, fig6).
표·그림은 analysis/summarize.py, analysis/plots.py를 그대로 쓴다 (실차와 같은 코드).
민감도 인자(--w-min 등)에 값을 여러 개 주면 각 값이 별도 회차가 된다 (회차 이름 꼬리표에 값).
"""
import argparse
import datetime
import glob
import itertools
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TUNE = ('v_imp', 'w_min', 'w_max', 'n_frames', 'known_on', 'known_off', 'b1_k', 'b1_roi_length', 'b1_roi_half_width',
        'row_face_tol')


def one(job):
    sys.path[:0] = [HERE, ROOT]
    import runner
    argv, out_dir = job
    try:
        m = runner.main(argv + ['--out-dir', out_dir])
        return out_dir, None, m
    except Exception as e:  # 한 회차 실패가 전체를 멈추지 않게
        return out_dir, f'{type(e).__name__}: {e}', None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--scenarios', nargs='+', default=['S1', 'S2', 'S3', 'S4'])
    ap.add_argument('--methods', nargs='+', default=['B0', 'B1', 'B2', 'P'])
    ap.add_argument('--d-e', nargs='+', type=float, default=[1.0, 1.5, 2.0])
    ap.add_argument('--seeds', nargs='+', type=int, default=[0])
    ap.add_argument('--steering', choices=['node', 'simple'], default='node')
    ap.add_argument('--name', default='main')
    ap.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument('--log-root', default=os.path.join(ROOT, 'sim_logs'))
    ap.add_argument('--analyze', metavar='SWEEP_DIR', default=None,
                    help='실행 없이 이 sweep 폴더의 요약 표·그림 5·6만 다시 만든다')
    for k in TUNE:
        ap.add_argument('--' + k.replace('_', '-'), dest=k, nargs='+', default=None)
    args = ap.parse_args()
    if args.analyze:
        print(f'요약·그림: {report(os.path.abspath(args.analyze))}')
        return
    root = os.path.join(args.log_root, datetime.date.today().strftime('%Y%m%d'), f'sweep_{args.name}')
    tune_axes = [(k, v) for k in TUNE if getattr(args, k) for v in getattr(args, k)]
    tune_sets = [()] if not tune_axes else [((k, v),) for k, v in tune_axes]
    jobs = []
    for scen, method, seed, tune in itertools.product(args.scenarios, args.methods, args.seeds, tune_sets):
        for d_e in (args.d_e if scen == 'S1' else [None]):
            argv = ['--method', method, '--scenario', scen, '--seed', str(seed), '--steering', args.steering]
            tag = ''
            if d_e is not None:
                argv += ['--d-e', str(d_e)]
            for k, v in tune:
                argv += ['--' + k.replace('_', '-'), str(v)]
                tag += f'_{k}{v}'
            de = 'na' if d_e is None else f'{d_e:.1f}'
            jobs.append((argv, os.path.join(root, f'{scen}_{method}_de{de}_seed{seed:02d}{tag}')))
    print(f'{len(jobs)}회, workers={args.workers} → {root}')
    ctx = mp.get_context('spawn')
    failed = []
    with ctx.Pool(args.workers) as pool:
        for i, (out_dir, err, m) in enumerate(pool.imap_unordered(one, jobs), 1):
            status = f'오류 {err}' if err else (f"dT_cmd={m.get('dT_cmd')} collision={m.get('collision')}"
                                               f" d_e_actual={m.get('d_e_actual')}")
            print(f'[{i}/{len(jobs)}] {os.path.basename(out_dir)} {status}')
            if err:
                failed.append((out_dir, err))
    out = report(root)
    print(f'요약·그림: {out}' + (f'  (실패 {len(failed)}회)' if failed else ''))
    for d, e in failed:
        print(f'  실패 {d}: {e}')


def report(root):
    """sweep 폴더 → analysis/summary.md·csv, fig5, fig6 (실차와 같은 summarize·plots 코드)."""
    sys.path.insert(0, os.path.join(ROOT, 'analysis'))
    sys.path.insert(0, ROOT)
    import plots
    import summarize
    out = os.path.join(root, 'analysis')
    rows = summarize.summarize(summarize.load([root]))
    summarize.write(rows, out)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    names = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams['font.family'] = [f for f in ('AppleGothic', 'NanumGothic') if f in names][:1] + ['DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    runs = [r for r in plots.load_runs([root]) if not r.get('static')]
    plots.fig5(runs, out, plt)
    plots.fig6(runs, out, plt)
    import report as run_report                       # 회차별 report.html 목록 + 판정 요약 (index.html)
    # 회차별 리포트는 회차가 끝날 때 만들어지지만, 같은 조건 비교는 모든 회차가 끝난 뒤에야 완전하므로 다시 만든다
    for d in sorted(glob.glob(os.path.join(root, '*', 'analysis', 'metrics.json'))):
        run_report.render(os.path.dirname(os.path.dirname(d)))
    run_report.index(root)
    return out


if __name__ == '__main__':
    main()
