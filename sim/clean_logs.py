#!/usr/bin/env python3
"""sim_logs 정리. 기본은 미리보기(지울 목록·용량만). --yes일 때만 실행하고, 기본은 macOS 휴지통(~/.Trash)으로 옮긴다.

    python sim/clean_logs.py --du                                  # sim_logs 전체·폴더별 용량
    python sim/clean_logs.py --before 20261101                     # 미리보기: 11월 1일 전 날짜 폴더의 회차
    python sim/clean_logs.py --restarts --yes                      # 뷰어 재시작 회차(_r1, _r2 ...)를 휴지통으로
    python sim/clean_logs.py --tag dev --keep-latest 3 --yes       # 꼬리표 dev 회차 중 최근 3개만 남김
    python sim/clean_logs.py --videos-only --before 20261105 --yes # mp4만 지움
    python sim/clean_logs.py --keep sim_logs/20261105/sweep_main   # KEEP 표시 (절대 안 지움)

선택 조건은 함께 쓰면 모두 만족하는 회차만 고른다. 조건이 하나도 없으면 아무것도 고르지 않는다.
--before, --tag, --restarts, --incomplete, --keep-latest N, --videos-only.
보호: 회차 폴더나 그 위 폴더(sweep, 날짜)에 KEEP 파일이 있으면 절대 지우지 않는다.
sweep 폴더 안 회차가 모두 골라지면 sweep 폴더 전체(analysis, index.html 포함)를 지운다.
sim_logs는 git에 올라가지 않으므로 지워도 코드에 영향이 없다.
"""
import argparse
import json
import os
import re
import shutil
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATE = re.compile(r'^\d{8}$')
RESTART = re.compile(r'_r\d+$')


def size_of(path):
    if os.path.isfile(path):
        return os.path.getsize(path)
    total = 0
    for dp, _, fns in os.walk(path):
        for fn in fns:
            try:
                total += os.path.getsize(os.path.join(dp, fn))
            except OSError:
                pass
    return total


def human(n):
    for unit in ('B', 'KB', 'MB', 'GB'):
        if n < 1024 or unit == 'GB':
            return f'{n:.0f} {unit}' if unit == 'B' else f'{n:.1f} {unit}'
        n /= 1024.0


def protected(path, root):
    """path 또는 root까지의 상위 폴더에 KEEP 파일이 있으면 True."""
    p = os.path.abspath(path)
    root = os.path.abspath(root)
    while True:
        if os.path.exists(os.path.join(p, 'KEEP')):
            return True
        if p == root or os.path.dirname(p) == p:
            return False
        p = os.path.dirname(p)


def find_runs(root):
    """meta.json이 있는 회차 폴더 목록 [(경로, 날짜, sweep 폴더 또는 None)]."""
    out = []
    for date in sorted(os.listdir(root)) if os.path.isdir(root) else []:
        dpath = os.path.join(root, date)
        if not (DATE.match(date) and os.path.isdir(dpath)):
            continue
        for name in sorted(os.listdir(dpath)):
            p = os.path.join(dpath, name)
            if os.path.exists(os.path.join(p, 'meta.json')):
                out.append((p, date, None))
            elif name.startswith('sweep_') and os.path.isdir(p):
                for rn in sorted(os.listdir(p)):
                    rp = os.path.join(p, rn)
                    if os.path.exists(os.path.join(rp, 'meta.json')):
                        out.append((rp, date, p))
    return out


def incomplete(run):
    """창을 중간에 닫았거나 재시작으로 끊긴 회차: 최대 시간에 닿지 않았고, 정지해서 끝나지도 않았다."""
    try:
        with open(os.path.join(run, 'meta.json'), encoding='utf-8') as f:
            meta = json.load(f)
        sim = meta['config']['sim']['sim']
        z = np.load(os.path.join(run, 'log.npz'))
        t_end = float(z['t'][-1])
        if t_end >= float(sim['max_time']) - 2 * float(sim['dt']):
            return False
        reason = str(z['stop_reason'][-1])
        return not (reason in ('stop_line', 'aeb', 'joy_timeout') and float(z['v'][-1]) < 0.02)
    except (OSError, KeyError, ValueError, IndexError):
        return True                  # 읽을 수 없는 회차도 불완전으로 본다


def tag_of(name):
    """회차 이름 <S>_<M>_de<..>_seed<NN>[_<tag>][_rN] 에서 tag (sweep의 [EXP-TUNE] 꼬리표 포함)."""
    m = re.match(r'^[^_]+_[^_]+_de[^_]+_seed\d+(?:_(?!r\d+$)(.*?))?(?:_r\d+)?$', name)
    return (m.group(1) or '') if m else ''


def select(args):
    root = args.root
    runs = find_runs(root)
    picked = []
    for path, date, sweep in runs:
        name = os.path.basename(path)
        if args.before and not date < args.before:
            continue
        if args.tag is not None and tag_of(name) != args.tag:
            continue
        if args.restarts and not RESTART.search(name):
            continue
        if args.incomplete and not incomplete(path):
            continue
        picked.append((path, date, sweep))
    skipped = [p for p in picked if protected(p[0], root)]      # KEEP 보호를 먼저 빼고
    picked = [p for p in picked if not protected(p[0], root)]
    if args.keep_latest is not None:                            # 남은 후보 중 최근 N개를 남긴다
        picked.sort(key=lambda r: os.path.getmtime(os.path.join(r[0], 'meta.json')), reverse=True)
        picked = picked[args.keep_latest:]
    if args.videos_only:
        targets = [os.path.join(p, 'topdown.mp4') for p, _, _ in picked if os.path.exists(os.path.join(p, 'topdown.mp4'))]
        return sorted(targets), skipped
    targets = set(p for p, _, _ in picked)
    # sweep 안 회차가 모두 골라졌으면 sweep 폴더 전체
    by_sweep = {}
    for p, _, sw in runs:
        if sw:
            by_sweep.setdefault(sw, []).append(p)
    for sw, members in by_sweep.items():
        if all(m in targets for m in members) and not protected(sw, root):
            targets -= set(members)
            targets.add(sw)
    return sorted(targets), skipped


def trash(path):
    dest_dir = os.path.expanduser('~/.Trash')
    os.makedirs(dest_dir, exist_ok=True)
    base = os.path.basename(path.rstrip('/'))
    parent = os.path.basename(os.path.dirname(path.rstrip('/')))
    dest = os.path.join(dest_dir, f'sim_logs_{parent}_{base}')
    if os.path.exists(dest):
        dest += time.strftime('_%H%M%S')
    shutil.move(path, dest)
    return dest


def du(root):
    total = size_of(root)
    print(f'{root}: {human(total)}')
    for date in sorted(os.listdir(root)):
        dpath = os.path.join(root, date)
        if not os.path.isdir(dpath):
            continue
        print(f'  {date}/  {human(size_of(dpath))}' + ('  [KEEP]' if protected(dpath, root) else ''))
        if DATE.match(date):
            for name in sorted(os.listdir(dpath)):
                p = os.path.join(dpath, name)
                if os.path.isdir(p):
                    n = sum(1 for r in os.listdir(p) if os.path.exists(os.path.join(p, r, 'meta.json'))) \
                        if name.startswith('sweep_') else None
                    print(f'    {name}/  {human(size_of(p))}' + (f'  ({n}회)' if n is not None else '')
                          + ('  [KEEP]' if protected(p, root) else ''))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--root', default=os.path.join(ROOT, 'sim_logs'))
    ap.add_argument('--before', metavar='YYYYMMDD', help='이 날짜 전 날짜 폴더의 회차')
    ap.add_argument('--tag', help='회차 이름 꼬리표 (예: dev, w_min0.25)')
    ap.add_argument('--restarts', action='store_true', help='뷰어 재시작 회차 (_r1, _r2 ...)')
    ap.add_argument('--incomplete', action='store_true', help='중간에 끊긴 회차 (창을 닫음, 재시작)')
    ap.add_argument('--keep-latest', type=int, metavar='N', help='고른 회차 중 최근 N개는 남김')
    ap.add_argument('--videos-only', action='store_true', help='고른 회차의 topdown.mp4만')
    ap.add_argument('--yes', action='store_true', help='실제로 실행 (없으면 미리보기)')
    ap.add_argument('--permanent', action='store_true', help='휴지통 대신 영구 삭제')
    ap.add_argument('--keep', metavar='FOLDER', help='FOLDER에 KEEP 파일을 만들어 보호')
    ap.add_argument('--du', action='store_true', help='용량 보기')
    args = ap.parse_args(argv)
    if args.before and not DATE.match(args.before):
        ap.error('--before는 YYYYMMDD')
    if args.keep:
        open(os.path.join(args.keep, 'KEEP'), 'a').close()
        print(f'보호: {os.path.join(args.keep, "KEEP")}')
        return 0
    if args.du:
        du(args.root)
        return 0
    if not any([args.before, args.tag is not None, args.restarts, args.incomplete, args.keep_latest is not None,
                args.videos_only]):
        print('선택 조건이 없다 (--before, --tag, --restarts, --incomplete, --keep-latest, --videos-only). '
              '전체를 지우려면 조건을 명시할 것.', file=sys.stderr)
        return 1
    targets, skipped = select(args)
    total = 0
    for t in targets:
        n = size_of(t)
        total += n
        print(f'  {human(n):>9}  {os.path.relpath(t, ROOT)}')
    for p, _, _ in skipped:
        print(f'  {"보호":>9}  {os.path.relpath(p, ROOT)} (KEEP)')
    how = '영구 삭제' if args.permanent else '휴지통(~/.Trash)으로 이동'
    print(f'{len(targets)}개, {human(total)} → {how}' + ('' if args.yes else '  [미리보기: 실제로 하려면 --yes]'))
    if not args.yes:
        return 0
    for t in targets:
        if args.permanent:
            shutil.rmtree(t) if os.path.isdir(t) else os.remove(t)
        else:
            trash(t)
    print('완료')
    return 0


if __name__ == '__main__':
    sys.exit(main())
