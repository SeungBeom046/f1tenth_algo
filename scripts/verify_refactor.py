#!/usr/bin/env python3
"""리팩터링 전후 기존 주행 모드(drive) 명령 동일성 검증 (rosbag 재생 기반).

두 하위 명령:
    record  재생 중 planner/최종 명령을 CSV로 기록 (scripts/replay_drive.sh가 호출)
    compare 두 CSV(기준선 baseline_cmd.csv vs 리팩터링 후)를 비교하고 수치 보고

비교 방식:
    planner (/reactive/lidar_command): scan마다 1개. sim time이 가장 가까운 쌍(±match_tol)을
        맞춰 rpm, steering, mode가 정확히 같아야 한다 (부동소수 허용오차 abs_tol).
    final (/commands/motor/speed, /commands/servo/position): 50 Hz 타이머라 재생마다 위상이 다르다.
        sim time 격자 위에서 각 run의 직전 값(zero-order hold)을 비교한다. 값이 바뀌는 순간
        ±transition_guard 안쪽은 위상 차이로 생기므로 제외하고 따로 센다.
    입력 메시지 유실(BEST_EFFORT)로 짝이 안 맞는 planner 샘플은 '미매칭'으로 따로 보고한다.

사용:
    python3 verify_refactor.py compare baseline_cmd.csv after_cmd.csv
종료 코드: 0 = 일치, 1 = 차이 있음.
"""

import argparse
import bisect
import csv
import json
import math
import sys

STREAMS = ('planner', 'motor', 'servo', 'aeb')


def record(args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float64, String

    rclpy.init()
    node = Node('verify_refactor_recorder',
                parameter_overrides=[rclpy.parameter.Parameter(
                    'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
    out = open(args.output, 'w', newline='', encoding='utf-8')
    writer = csv.writer(out)
    writer.writerow(['stream', 't', 'rpm', 'steering', 'mode', 'value'])
    state = {'last_msg_wall': None, 'count': 0}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def stamp():
        import time
        state['last_msg_wall'] = time.monotonic()
        state['count'] += 1

    def on_planner(msg):
        stamp()
        d = json.loads(msg.data)
        writer.writerow(['planner', f'{now():.6f}', d.get('rpm'), d.get('steering_rad'),
                         d.get('mode'), ''])

    def scalar(stream):
        def cb(msg):
            stamp()
            writer.writerow([stream, f'{now():.6f}', '', '', '', msg.data])
        return cb

    node.create_subscription(String, args.planner_topic, on_planner, 100)
    node.create_subscription(Float64, args.motor_topic, scalar('motor'), 100)
    node.create_subscription(Float64, args.servo_topic, scalar('servo'), 100)
    node.create_subscription(Bool, args.aeb_topic, scalar('aeb'), 100)

    import time
    start = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            last = state['last_msg_wall']
            if last is not None and time.monotonic() - last > args.idle_timeout:
                break
            if last is None and time.monotonic() - start > args.start_timeout:
                print('시작 시간 초과: 명령 토픽이 하나도 안 들어옴', file=sys.stderr)
                break
    except KeyboardInterrupt:
        pass
    out.close()
    print(f'{state["count"]}개 메시지 기록: {args.output}')
    node.destroy_node()
    rclpy.shutdown()


def load(path):
    data = {s: [] for s in STREAMS}
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            t = float(row['t'])
            if row['stream'] == 'planner':
                data['planner'].append((t, float(row['rpm']), float(row['steering']), row['mode']))
            else:
                v = row['value']
                v = 1.0 if v == 'True' else 0.0 if v == 'False' else float(v)
                data[row['stream']].append((t, v))
    for s in STREAMS:
        data[s].sort()
    return data


def compare_planner(a, b, match_tol, abs_tol):
    tb = [x[0] for x in b]
    used, matched, mismatched, unmatched = set(), 0, [], 0
    max_rpm = max_steer = 0.0
    for t, rpm, steer, mode in a:
        i = bisect.bisect_left(tb, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(b) and j not in used and abs(tb[j] - t) <= match_tol:
                if best is None or abs(tb[j] - t) < abs(tb[best] - t):
                    best = j
        if best is None:
            unmatched += 1
            continue
        used.add(best)
        matched += 1
        _, rpm_b, steer_b, mode_b = b[best]
        dr, ds = abs(rpm - rpm_b), abs(steer - steer_b)
        max_rpm, max_steer = max(max_rpm, dr), max(max_steer, ds)
        if dr > abs_tol * max(1.0, abs(rpm)) or ds > abs_tol or mode != mode_b:
            mismatched.append((t, (rpm, steer, mode), (rpm_b, steer_b, mode_b)))
    return {'a': len(a), 'b': len(b), 'matched': matched, 'unmatched_a': unmatched,
            'unmatched_b': len(b) - len(used), 'mismatched': mismatched,
            'max_abs_rpm': max_rpm, 'max_abs_steer': max_steer}


def hold(series, t):
    ts = [x[0] for x in series]
    i = bisect.bisect_right(ts, t) - 1
    return series[i][1] if i >= 0 else None


def compare_scalar(a, b, dt, guard, abs_tol):
    if not a or not b:
        return {'samples': 0, 'mismatch': 0, 'guarded': 0, 'max_abs': None}
    t0, t1 = max(a[0][0], b[0][0]), min(a[-1][0], b[-1][0])
    changes = sorted({x[0] for i, x in enumerate(a) if i and x[1] != a[i - 1][1]}
                     | {x[0] for i, x in enumerate(b) if i and x[1] != b[i - 1][1]})
    n = mismatch = guarded = 0
    max_abs = 0.0
    t = t0
    while t <= t1:
        va, vb = hold(a, t), hold(b, t)
        if va is not None and vb is not None:
            n += 1
            diff = abs(va - vb)
            if diff > abs_tol * max(1.0, abs(va)):
                k = bisect.bisect_left(changes, t)
                near = any(0 <= j < len(changes) and abs(changes[j] - t) <= guard
                           for j in (k - 1, k))
                if near:
                    guarded += 1
                else:
                    mismatch += 1
                    max_abs = max(max_abs, diff)
        t += dt
    return {'samples': n, 'mismatch': mismatch, 'guarded': guarded, 'max_abs': max_abs}


def compare(args):
    a, b = load(args.baseline), load(args.candidate)
    ok = True
    p = compare_planner(a['planner'], b['planner'], args.match_tol, args.abs_tol)
    print('== planner (/reactive/lidar_command) ==')
    print(f'  기준선 {p["a"]}개, 비교 {p["b"]}개, 매칭 {p["matched"]}, '
          f'미매칭 기준선 {p["unmatched_a"]} / 비교 {p["unmatched_b"]}')
    print(f'  불일치 {len(p["mismatched"])}  |  최대 차이 rpm {p["max_abs_rpm"]:.6g}, '
          f'steering {p["max_abs_steer"]:.6g} rad')
    for t, va, vb in p['mismatched'][:10]:
        print(f'    t={t:.3f}  기준선 {va}  비교 {vb}')
    if p['mismatched']:
        ok = False
    if p['matched'] and (p['unmatched_a'] + p['unmatched_b']) / max(1, p['a']) > args.max_unmatched:
        print(f'  경고: 미매칭 비율 > {args.max_unmatched:.0%} -> 재생 속도(--rate)를 낮춰 다시 실행')
        ok = False
    for stream in ('motor', 'servo', 'aeb'):
        r = compare_scalar(a[stream], b[stream], args.grid_dt, args.transition_guard, args.abs_tol)
        print(f'== {stream} ==  격자 {r["samples"]}점, 불일치 {r["mismatch"]} '
              f'(전환 직후 제외 {r["guarded"]}), 최대 차이 {r["max_abs"]}')
        if r['samples'] == 0 or r['mismatch']:
            ok = False
    print('결과:', '일치' if ok else '차이 있음 -> 원인 확인 전 리팩터링 중단')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('--output', required=True)
    r.add_argument('--planner-topic', default='/reactive/lidar_command')
    r.add_argument('--motor-topic', default='/commands/motor/speed')
    r.add_argument('--servo-topic', default='/commands/servo/position')
    r.add_argument('--aeb-topic', default='/aeb/active')
    r.add_argument('--idle-timeout', type=float, default=5.0, help='마지막 메시지 후 종료까지 [s, wall]')
    r.add_argument('--start-timeout', type=float, default=60.0)
    c = sub.add_parser('compare')
    c.add_argument('baseline')
    c.add_argument('candidate')
    c.add_argument('--match-tol', type=float, default=0.02, help='planner 짝 맞춤 sim time 허용 [s]')
    c.add_argument('--abs-tol', type=float, default=1e-9)
    c.add_argument('--grid-dt', type=float, default=0.01)
    c.add_argument('--transition-guard', type=float, default=0.025, help='50 Hz 한 주기 + 여유 [s]')
    c.add_argument('--max-unmatched', type=float, default=0.02)
    args = ap.parse_args()
    if args.cmd == 'record':
        record(args)
        return 0
    return compare(args)


if __name__ == '__main__':
    sys.exit(main())
