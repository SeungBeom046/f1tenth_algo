#!/usr/bin/env python3
"""offline_replay.sh 결과 비교: 같은 관측(녹화된 궤적)에서 방법별 명령 속도와 명령 기준 손실.

    python3 analysis/offline_compare.py <run_dir>

한계 (개루프): 에고 궤적·점군은 녹화된 회차 그대로다. 재생한 방법이 실제로 달렸다면 위치와 관측이
달라졌을 것이므로, 결과는 "같은 관측에 배치 규칙만 바꿨을 때 명령이 얼마나 다른가"로만 해석한다.
실차 효율 비교는 각 방법의 실제 회차(Exp2)로 한다.
"""
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import read_bag, stamp  # noqa: E402

from my_algo import metrics as M  # noqa: E402


def main():
    run_dir = sys.argv[1]
    with open(os.path.join(run_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    snap = os.path.join(run_dir, 'config_snapshot')
    with open(os.path.join(snap, 'common.yaml'), encoding='utf-8') as f:
        v_ref = float(yaml.safe_load(f)['/**']['ros__parameters']['v_ref'])
    with open(os.path.join(snap, f'scenario_{meta["scenario"]}.yaml'), encoding='utf-8') as f:
        scen = yaml.safe_load(f)['/**']['ros__parameters']
    orig = read_bag(os.path.join(run_dir, 'bag'), [meta.get('odom_topic', '/vesc/odom'),
                                                   '/autonomous_mode', '/exp/v_raw'])
    t_auto = next(t for t, m in orig['/autonomous_mode'] if m.data)
    odom = [(stamp(m), m) for _, m in orig[meta.get('odom_topic', '/vesc/odom')] if stamp(m) >= t_auto]
    t_o = np.array([t for t, _ in odom])
    s = M.travel_distance([m.pose.pose.position.x for _, m in odom],
                          [m.pose.pose.position.y for _, m in odom])
    x0, x1 = float(scen['test_x_start']), float(scen['test_x_end'])
    results = {}
    sources = {f'{meta["method"]} (recorded)': orig.get('/exp/v_raw', [])}
    replay_root = os.path.join(run_dir, 'replay')
    for method in sorted(os.listdir(replay_root)) if os.path.isdir(replay_root) else []:
        bag_dir = os.path.join(replay_root, method)
        if os.path.exists(os.path.join(bag_dir, 'metadata.yaml')):
            sources[f'{method} (replay)'] = read_bag(bag_dir, ['/exp/v_raw']).get('/exp/v_raw', [])
    for name, msgs in sources.items():
        msgs = [(t, m) for t, m in msgs if t >= t_auto]
        if not msgs:
            continue
        ss, vv = M.series_on_s(t_o, s, np.array([t for t, _ in msgs]), np.array([m.data for _, m in msgs]))
        dt, floored = M.loss_delta_t(ss, vv, v_ref, x0, x1)
        results[name] = {'dT_cmd': dt, 'floored': floored, 'min_speed_cmd': M.min_speed(ss, vv, x0, x1)}
    out = os.path.join(run_dir, 'analysis', 'offline_compare.json')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as f:
        json.dump({'note': '개루프 재생: 녹화된 궤적 기준 명령 비교', 'results': results}, f, indent=2,
                  ensure_ascii=False)
    for k, v in results.items():
        print(f'{k:16s} dT_cmd={v["dT_cmd"]} min_speed_cmd={v["min_speed_cmd"]}')


if __name__ == '__main__':
    main()
