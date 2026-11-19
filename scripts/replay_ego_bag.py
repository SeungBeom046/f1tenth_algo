#!/usr/bin/env python3
"""실차 ego bag을 EgoRunner로 다시 돌려 기록된 /exp/state와 비교한다 (ROS 없이, rosbag2 sqlite3).

    python3 scripts/replay_ego_bag.py ~/runs/<기록>/bag --method a --set stationary_pass=true W=1.0 t_d=0.25

/scan + 기록된 자기 속도(/exp/state v) + /livox/imu z 평균(scan 주기, --gyro-bias 뺌)을 넣는다.
기록과 같은 판단이면 '일치'로 센다. --set으로 파라미터를 바꾸면 결정이 달라진 첫 시점부터는 실제 차가 그대로 움직였으므로
개루프 재생이며, 그 뒤 결과는 참고만 한다. 출력: 상태·이유·추적·추월 보류 이유가 바뀌는 scan.
"""
import argparse
import bisect
import json
import os
import sqlite3
import struct
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from my_algo.aur.ego_runner import EgoRunner     # noqa: E402
from my_algo.aur.params import load_params       # noqa: E402

import sqlite3, struct, json
from types import SimpleNamespace


class _R:
    def __init__(self, b):
        self.b, self.o = b, 4          # 4바이트 encapsulation 헤더 뒤가 정렬 기준 0

    def al(self, n):
        r = (self.o - 4) % n
        if r:
            self.o += n - r

    def u32(self):
        self.al(4); v = struct.unpack_from('<I', self.b, self.o)[0]; self.o += 4; return v

    def i32(self):
        self.al(4); v = struct.unpack_from('<i', self.b, self.o)[0]; self.o += 4; return v

    def f32(self):
        self.al(4); v = struct.unpack_from('<f', self.b, self.o)[0]; self.o += 4; return v

    def f64(self):
        self.al(8); v = struct.unpack_from('<d', self.b, self.o)[0]; self.o += 8; return v

    def string(self):
        n = self.u32(); s = self.b[self.o:self.o + n - 1].decode('utf-8', 'replace'); self.o += n; return s

    def f32seq(self):
        n = self.u32(); self.al(4); v = struct.unpack_from('<%df' % n, self.b, self.o); self.o += 4 * n; return list(v)


def header(r):
    return r.i32() + r.u32() * 1e-9, r.string()


def laserscan(b):
    r = _R(b); stamp, frame = header(r)
    vals = [r.f32() for _ in range(7)]
    ranges = r.f32seq()
    return SimpleNamespace(stamp=stamp, frame_id=frame, angle_min=vals[0], angle_max=vals[1], angle_increment=vals[2],
                           range_min=vals[5], range_max=vals[6], ranges=ranges)


def imu_z(b):
    r = _R(b); header(r)
    [r.f64() for _ in range(4 + 9)]
    return r.f64(), r.f64(), r.f64()


def string(b):
    return _R(b).string()


def read(path, topic):
    db = sqlite3.connect('file:%s?mode=ro' % path, uri=True)
    tid = dict(db.execute('select name, id from topics').fetchall())[topic]
    return [(ts * 1e-9, data) for ts, data in db.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid,))]


def value(text):
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return {'true': True, 'false': False, 'null': None}.get(text.lower(), text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='rosbag2 폴더 또는 .db3')
    ap.add_argument('--method', default='a', choices=list('abcd'))
    ap.add_argument('--set', nargs='*', default=[], metavar='K=V', help='AurParams 덮어쓰기 (실차 launch 인자와 같게)')
    ap.add_argument('--gyro-bias', type=float, default=0.0, help='뺄 자이로 바이어스 [rad/s] (실차 로그의 IMU 바이어스 보정 값)')
    a = ap.parse_args()
    path = a.bag if a.bag.endswith('.db3') else os.path.join(a.bag, sorted(f for f in os.listdir(a.bag) if f.endswith('.db3'))[0])
    over = dict(kv.split('=', 1) for kv in a.set)
    p = load_params(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'exp_common.yaml'),
                    **{k: value(v) for k, v in over.items()})
    scans = [(t, laserscan(b)) for t, b in read(path, '/scan')]
    imu = [(t, imu_z(b)[2]) for t, b in read(path, '/livox/imu')]
    states = [json.loads(string(b)) for _, b in read(path, '/exp/state')]
    runner = EgoRunner(p, a.method)
    st_t, it = [s['t'] for s in states], [t for t, _ in imu]
    prev_t, prev_key, n, same = None, None, 0, 0
    t0 = states[0]['t'] if states else 0.0
    for ts, scan in scans:
        k = bisect.bisect_left(st_t, ts - 0.05)
        if k >= len(states) or abs(states[k]['t'] - ts) > 0.08:
            prev_t = ts
            continue
        log = states[k]
        lo, hi = bisect.bisect_left(it, prev_t if prev_t else ts - 0.1), bisect.bisect_right(it, ts)
        zs = [z for _, z in imu[lo:hi]]
        _, _, _, state, reason = runner.step(log['t'], scan, log['v'], sum(zs) / len(zs) - a.gyro_bias if zs else None)
        prev_t = ts
        n += 1
        same += state == log.get('state') and reason == log.get('reason')
        trk, block = runner.track, runner.fsm.pass_block
        key = (state, reason, runner.tracker.status, (block or '').split(' (')[0])
        if key != prev_key:
            print('%6.2f s  v %.2f | %-8s %-22s %-13s d %-5s | 기록 %-8s %-22s | 보류: %s' % (
                log['t'] - t0, log['v'], state, reason, runner.tracker.status,
                '-' if trk is None or trk.d is None else '%.2f' % trk.d, log.get('state'), log.get('reason'), block or '-'))
            prev_key = key
    print('scan %d개, 기록과 상태·이유 일치 %d (%.0f%%)' % (n, same, 100.0 * same / max(1, n)))


if __name__ == '__main__':
    main()
