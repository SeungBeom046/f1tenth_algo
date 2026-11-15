#!/usr/bin/env python3
"""동작 기준선: 고정 시드 회차 묶음을 돌려 결과와 스텝 로그 해시를 CSV로 저장하고, 두 CSV를 비교한다.

    python sim/mujoco_overtake/baseline.py save <out.csv>          # 약 3분
    python sim/mujoco_overtake/baseline.py compare <a.csv> <b.csv>  # 다르면 종료 코드 1, 바뀐 회차를 표로

리팩터링 전후로 save → compare 해서 동작이 바뀌지 않았는지 확인한다 (log_hash = 모든 scan의 로그 전체).
묶음: belief(original·survival × c·d × Y·NY × 20), yaw(0/10/20/30° × d_y 0.8/1.0/1.2),
matrix(W 1.0 × a·b·c·d × Y·NY × AEB 기하/없음 × 5).
"""
import csv
import hashlib
import json
import random
import sys

from sim import Sim, load_cfg


def cases():
    for model in ('original', 'survival'):
        for m in 'cd':
            for beh in ('Y', 'NY'):
                rng = random.Random(0)
                for k in range(20):
                    yield ('belief', m, k, {'leader.behavior': beh, 'leader.d_y': rng.uniform(0.6, 1.2),
                                            'aeb.enabled': False}, {'belief_model': model})
    for deg in (0, 10, 20, 30):
        for d_y in (0.8, 1.0, 1.2):
            yield ('yaw', 'd', 0, {'leader.behavior': 'Y', 'leader.d_y': d_y, 'leader.yield_yaw_deg': float(deg),
                                   'aeb.enabled': False}, {})
    for aeb in (True, False):
        for m in 'abcd':
            for beh in ('Y', 'NY'):
                for k in range(5):
                    yield ('matrix', m, k, {'leader.behavior': beh, 'track.W': 1.0, 'aeb.enabled': aeb}, {})


def save(path):
    rows = []
    for group, m, seed, over, params in cases():
        sim = Sim(m, load_cfg(**over), seed=seed, **params)
        r = sim.run()
        r['log_hash'] = hashlib.sha1(json.dumps(sim.log, sort_keys=True, default=repr).encode()).hexdigest()[:16]
        rows.append(dict({'group': group, 'over': json.dumps(over, sort_keys=True),
                          'params': json.dumps(params, sort_keys=True)}, **r))
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('%d회 저장: %s' % (len(rows), path))


def compare(a, b):
    ra, rb = list(csv.DictReader(open(a))), list(csv.DictReader(open(b)))
    if len(ra) != len(rb):
        print('회차 수가 다르다: %d vs %d' % (len(ra), len(rb)))
        return 1
    diff = 0
    cols = ('final_state', 'cause', 'N_AEB', 'deadlock', 'n_abort', 'h_min', 'stop_gap_ok', 'T_pass')
    print('| # | 묶음 | 방법 | 조건 | ' + ' | '.join(cols) + ' | 그 밖에 바뀐 열 |')
    print('|---' * (5 + len(cols)) + '|')
    for i, (x, y) in enumerate(zip(ra, rb)):
        keys = sorted(set(x) | set(y))
        d = [k for k in keys if x.get(k) != y.get(k)]
        if d:
            diff += 1
            cond = ' '.join('%s=%s' % (k.split('.')[-1], v) for k, v in json.loads(x['over']).items()
                            if k != 'aeb.enabled' or v is False) + (' ' + x['params'] if x['params'] != '{}' else '')
            cell = lambda k: (x.get(k, '') if x.get(k) == y.get(k) else '%s → %s' % (x.get(k, ''), y.get(k, '')))  # noqa: E731
            other = [k for k in d if k not in cols and k != 'log_hash']
            print('| %d | %s | %s | %s | %s | %s |' % (i, x['group'], x['method'], cond,
                                                     ' | '.join(cell(k) for k in cols), ', '.join(other)))
    print('\n%d회 중 %d회 다름 (로그 해시 기준)' % (len(ra), diff))
    return 1 if diff else 0


if __name__ == '__main__':
    if sys.argv[1] == 'save':
        save(sys.argv[2])
    else:
        sys.exit(compare(sys.argv[2], sys.argv[3]))
