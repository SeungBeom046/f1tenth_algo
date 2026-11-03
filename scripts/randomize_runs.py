#!/usr/bin/env python3
"""회차 실행 순서 무작위화. 블록마다 방법 순서를 섞고 결과를 session.json에 기록한다.

블록 = 한 조건(시나리오, d_e)에서 네 방법을 한 번씩 도는 묶음. 블록 b의 회차 번호 = b.
S1은 블록 안에서 (d_e, 방법) 12개를 함께 섞는다 (트랙 배치가 같고 트리거 거리만 다르므로).
B0의 S1 d_e=1.0은 충돌이 예상되어 5회로 줄인다 (계획서 1.10).
시나리오 순서는 트랙 재배치를 줄이려고 고정(S1 → S2 → S3 → S4)하고 그 안에서만 섞는다.

    python3 randomize_runs.py --seed 20261110            # 순서 출력 + session.json 기록
    python3 randomize_runs.py --seed 1 --blocks 2 --dry-run
"""

import argparse
import json
import os
import random

from my_algo.run_paths import METHODS, session_dir

D_E = (1.0, 1.5, 2.0)


def plan(seed, blocks, b0_s1_de1_runs=5, scenarios=('S1', 'S2', 'S3', 'S4')):
    rng = random.Random(seed)
    order = []
    for scenario in scenarios:
        for b in range(1, blocks + 1):
            if scenario == 'S1':
                cells = [(m, d) for d in D_E for m in METHODS
                         if not (m == 'B0' and d == 1.0 and b > b0_s1_de1_runs)]
            else:
                cells = [(m, None) for m in METHODS]
            rng.shuffle(cells)
            for m, d in cells:
                order.append({'scenario': scenario, 'method': m, 'd_e': d, 'run_id': b, 'block': b})
    return order


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--blocks', type=int, default=10)
    ap.add_argument('--b0-s1-de1-runs', type=int, default=5)
    ap.add_argument('--scenarios', nargs='+', default=['S1', 'S2', 'S3', 'S4'])
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    order = plan(args.seed, args.blocks, args.b0_s1_de1_runs, tuple(args.scenarios))
    for i, r in enumerate(order, 1):
        de = 'na' if r['d_e'] is None else r['d_e']
        de_arg = '' if r['d_e'] is None else f' d_e:={r["d_e"]}'
        print(f'{i:3d}  {r["scenario"]} {r["method"]:2s} d_e={de!s:4s} run={r["run_id"]:2d}  '
              f'bringup: method:={r["method"]} scenario:={r["scenario"]}{de_arg} run_id:={r["run_id"]}  '
              f'record: {r["method"]} {r["scenario"]} {de} {r["run_id"]}')
    print(f'총 {len(order)}회')
    if args.dry_run:
        return
    path = os.path.join(session_dir(), 'session.json')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    session = {}
    if os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            session = json.load(f)
    session['run_order'] = {'seed': args.seed, 'blocks': args.blocks, 'runs': order}
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(session, f, ensure_ascii=False, indent=2)
    print(f'기록: {path}')


if __name__ == '__main__':
    main()
