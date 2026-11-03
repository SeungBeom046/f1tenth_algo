#!/usr/bin/env python3
"""ROS 없이 리팩터링 전후 drive 모드 노드 출력을 비교한다 (보조 검증).

rosbag 재생 검증(replay_drive.sh + verify_refactor.py)이 1순위다. 이 스크립트는 ROS가 없는
개발 PC에서 리팩터링 중에 빠르게 확인하는 용도다.

방법:
    - 가짜 rclpy/메시지 모듈(아래 STUBS)로 lidar_reactive_drive, autonomous_drive 노드를 그대로 실행
    - 기준 git 리비전과 작업 트리 각각을 별도 프로세스에서 같은 합성 입력으로 돌린다
      (좌측 벽, 우측 주차 박스, 전방 접근 장애물, inf/nan 빔, 속도 변화, AUTO 토글, planner 누락)
    - 파라미터는 각 버전의 launch/lidar_drive.launch.py에 적힌 값을 그대로 쓴다
    - 발행된 모든 메시지(토픽, 순서, 값)를 비교한다

한계: 합성 입력이라 실제 점군 분포와 다르다. 실시간 타이밍/메시지 유실은 재현하지 않는다.

사용:
    python3 scripts/verify_refactor_offline.py --baseline baseline-pre-refactor
"""

import argparse
import ast
import json
import os
import subprocess
import sys
import tempfile

PKG_FILES = ['my_algo/__init__.py', 'my_algo/vehicle_utils.py', 'my_algo/aeb.py',
             'my_algo/lidar_reactive_drive.py', 'my_algo/autonomous_drive.py',
             'launch/lidar_drive.launch.py']

STUBS = {
    'rclpy/__init__.py': '''
from . import node, qos, parameter
def init(args=None): pass
def shutdown(): pass
def ok(): return True
''',
    'rclpy/node.py': '''
CLOCK = {'ns': 0}
OVERRIDES = {}
BUS = []
class Duration:
    def __init__(self, ns): self.nanoseconds = ns
class Time:
    def __init__(self, ns): self.nanoseconds = ns
    def __sub__(self, other): return Duration(self.nanoseconds - other.nanoseconds)
class _Clock:
    def now(self): return Time(CLOCK['ns'])
class _Param:
    def __init__(self, v): self.value = v
class _Logger:
    def info(self, *a, **k): pass
    warn = warning = error = debug = info
class _Pub:
    def __init__(self, topic): self.topic = topic
    def publish(self, msg): BUS.append((self.topic, msg))
class Node:
    def __init__(self, name, **kw):
        self._params, self.subs, self.timers = {}, {}, []
    def declare_parameter(self, name, value=None, *a, **k):
        self._params[name] = OVERRIDES.get(name, value)
        return _Param(self._params[name])
    def get_parameter(self, name): return _Param(self._params[name])
    def set_parameters(self, ps): pass
    def create_subscription(self, t, topic, cb, qos): self.subs[topic] = cb
    def create_publisher(self, t, topic, qos): return _Pub(topic)
    def create_timer(self, period, cb): self.timers.append((period, cb))
    def get_clock(self): return _Clock()
    def get_logger(self): return _Logger()
    def destroy_node(self): pass
''',
    'rclpy/qos.py': '''
class QoSProfile:
    def __init__(self, **kw): pass
class QoSHistoryPolicy: KEEP_LAST = 1
class QoSReliabilityPolicy: BEST_EFFORT = 2; RELIABLE = 1
qos_profile_sensor_data = None
''',
    'rclpy/parameter.py': '''
class Parameter:
    class Type: BOOL = 1
    def __init__(self, *a): pass
''',
    'sensor_msgs/__init__.py': '',
    'sensor_msgs/msg.py': '''
class LaserScan:
    def __init__(self):
        self.angle_min = 0.0; self.angle_max = 0.0; self.angle_increment = 0.0
        self.range_min = 0.0; self.range_max = 0.0; self.ranges = []
''',
    'std_msgs/__init__.py': '',
    'std_msgs/msg.py': '''
class _D:
    def __init__(self, data=None): self.data = data
class Bool(_D): pass
class Float64(_D): pass
class String(_D): pass
''',
    'nav_msgs/__init__.py': '',
    'nav_msgs/msg.py': '''
class _V:
    def __init__(self): self.x = 0.0; self.y = 0.0; self.z = 0.0
class _T:
    def __init__(self): self.linear = _V(); self.angular = _V()
class _TC:
    def __init__(self): self.twist = _T()
class Odometry:
    def __init__(self): self.twist = _TC()
''',
}

RUNNER = r'''
import json, math, os, random, sys, contextlib
sys.path[:0] = [sys.argv[1], sys.argv[2]]
import rclpy.node as rn
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

params = json.loads(open(sys.argv[3]).read())
seed = int(sys.argv[4])

def make(cls, overrides):
    rn.OVERRIDES = overrides
    return cls()

with open(os.devnull, 'w') as devnull, contextlib.redirect_stdout(devnull):
    import my_algo.lidar_reactive_drive as L
    import my_algo.autonomous_drive as A
    planner = make(L.LidarReactiveDriveNode, params['lidar_reactive_drive'])
    drive = make(A.AutonomousDriveNode, params['autonomous_drive'])

rng = random.Random(seed)
N_BEAMS = 360
INC = 2 * math.pi / N_BEAMS

def ray_segment(a, sx, sy, ex, ey):
    dx, dy = math.cos(a), math.sin(a)
    rx, ry = ex - sx, ey - sy
    den = dx * ry - dy * rx
    if abs(den) < 1e-12: return None
    t = (sx * ry - sy * rx) / den
    u = (sx * dy - sy * dx) / den
    return t if t > 0 and 0 <= u <= 1 else None

def scan_for(segs, step):
    m = LaserScan()
    m.angle_min = -math.pi; m.angle_increment = INC; m.angle_max = math.pi - INC
    m.range_min = 0.05; m.range_max = 20.0
    out = []
    for i in range(N_BEAMS):
        a = m.angle_min + i * INC
        best = float('inf')
        for s in segs:
            t = ray_segment(a, *s)
            if t is not None and t < best: best = t
        if best != float('inf'): best += rng.gauss(0, 0.005)
        p = rng.random()
        if p < 0.02: best = float('inf')
        elif p < 0.025: best = float('nan')
        elif p < 0.027: best = 0.01
        out.append(best)
    m.ranges = out
    return m

def world(x_ego, lane_y):
    segs = []
    wl = 0.7 + rng.uniform(-0.02, 0.02)
    segs.append((-3 - 0, wl - lane_y, 12, wl - lane_y))
    for k in range(6):
        x0 = 1.5 + k * 1.0 - x_ego
        gap = 0.45 if seed % 2 == 0 else 0.15
        y0 = -0.45 - lane_y
        segs += [(x0, y0, x0 + 0.55 - gap * 0.5, y0), (x0, y0 - 0.3, x0, y0)]
    return segs

def emit(node_topic_cb, msg):
    node_topic_cb(msg)

outputs = []
def drain():
    for topic, msg in rn.BUS:
        d = msg.data
        outputs.append([topic, d if not isinstance(d, float) else repr(d)])
    rn.BUS.clear()

x, v, lane_y = 0.0, 0.0, 0.0
obstacle_x = 6.0 + rng.uniform(0, 3)
obstacle_y = rng.uniform(-0.6, 0.3)
for step in range(220):
    target = 2.4 if step < 120 else rng.choice([0.0, 0.9, 2.0])
    v = max(0.0, min(2.6, v + max(-0.3, min(0.3, target - v)) + rng.gauss(0, 0.05)))
    if step == 140 + seed: v = -0.2
    x += v * 0.1
    lane_y += rng.gauss(0, 0.01)
    segs = world(x, lane_y)
    ox = obstacle_x - x
    if seed % 3 != 2:
        segs.append((ox, obstacle_y, ox, obstacle_y + 0.3))
    odom = Odometry(); odom.twist.twist.linear.x = v; odom.twist.twist.linear.y = rng.gauss(0, 0.01)
    planner.subs['/vesc/odom'](odom); drive.subs['/vesc/odom'](odom)
    if step in (60 + seed, 61 + seed):
        b = Bool(data=(step == 61 + seed))
        planner.subs['/autonomous_mode'](b); drive.subs['/autonomous_mode'](b)
    if step == 90:
        j = Bool(data=True); planner.subs['/joy_active'](j); drive.subs['/joy_active'](j)
    if step == 93:
        j = Bool(data=False); planner.subs['/joy_active'](j); drive.subs['/joy_active'](j)
    skip_planner = 100 <= step < 115 and seed % 2 == 1
    scan = scan_for(segs, step)
    with open(os.devnull, 'w') as devnull, contextlib.redirect_stdout(devnull):
        if not skip_planner:
            planner.subs['/scan'](scan)
            for topic, msg in list(rn.BUS):
                if topic == '/reactive/lidar_command':
                    drive.subs['/reactive/lidar_command'](msg)
        drive.subs['/scan'](scan)
        drain()
        for k in range(5):
            rn.CLOCK['ns'] += 20_000_000
            for period, cb in drive.timers:
                ticks = round(period / 0.02)
                if (rn.CLOCK['ns'] // 20_000_000) % ticks == 0:
                    cb()
            drain()
open(sys.argv[5], 'w').write(json.dumps(outputs))
'''


def launch_params(launch_src):
    """lidar_drive.launch.py에서 노드별 parameters 첫 dict 리터럴을 꺼낸다."""
    tree = ast.parse(launch_src)
    found = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'Node':
            kw = {k.arg: k.value for k in node.keywords}
            exe = kw.get('executable')
            if not isinstance(exe, ast.Constant) or 'parameters' not in kw:
                continue
            first = kw['parameters'].elts[0]
            found[exe.value] = ast.literal_eval(first)
    return found


def materialize(root, files):
    for rel, content in files.items():
        path = os.path.join(root, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(content)


def version_files(repo, rev):
    out = {}
    for rel in PKG_FILES:
        if rev is None:
            with open(os.path.join(repo, rel), encoding='utf-8') as f:
                out[rel] = f.read()
        else:
            out[rel] = subprocess.check_output(['git', '-C', repo, 'show', f'{rev}:{rel}'],
                                               text=True)
    return out


def run(stub_dir, pkg_dir, params_path, seed):
    runner = os.path.join(stub_dir, 'runner.py')
    out_path = f'{params_path}.{seed}.out.json'
    res = subprocess.run([sys.executable, '-I', runner, stub_dir, pkg_dir, params_path, str(seed),
                          out_path], capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(res.stderr)
    with open(out_path, encoding='utf-8') as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--baseline', default='baseline-pre-refactor')
    ap.add_argument('--seeds', type=int, default=6)
    args = ap.parse_args()
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    with tempfile.TemporaryDirectory() as tmp:
        stub_dir = os.path.join(tmp, 'stubs')
        materialize(stub_dir, dict(STUBS, **{'runner.py': RUNNER}))
        versions = {}
        for label, rev in (('baseline', args.baseline), ('current', None)):
            files = version_files(repo, rev)
            pkg = os.path.join(tmp, label)
            materialize(pkg, files)
            params = launch_params(files['launch/lidar_drive.launch.py'])
            ppath = os.path.join(tmp, f'{label}_params.json')
            with open(ppath, 'w') as f:
                json.dump(params, f)
            versions[label] = (pkg, ppath, params)

        ok = True
        if versions['baseline'][2] != versions['current'][2]:
            ok = False
            print('launch 파라미터 다름:')
            for exe in sorted(set(versions['baseline'][2]) | set(versions['current'][2])):
                a = versions['baseline'][2].get(exe)
                b = versions['current'][2].get(exe)
                if a != b:
                    print(f'  {exe}: {a} -> {b}')
        else:
            print('launch 파라미터: 동일')

        total = 0
        for seed in range(args.seeds):
            a = run(stub_dir, versions['baseline'][0], versions['baseline'][1], seed)
            b = run(stub_dir, versions['current'][0], versions['current'][1], seed)
            total += len(a)
            topics = {}
            for t, _ in a:
                topics[t] = topics.get(t, 0) + 1
            modes = {}
            for t, d in a:
                if t == '/reactive/lidar_command':
                    m = json.loads(d)['mode']
                    modes[m] = modes.get(m, 0) + 1
            aeb_on = sum(1 for t, d in a if t == '/aeb/active' and d is True)
            if a == b:
                print(f'seed {seed}: 일치 ({len(a)}개 메시지, planner 모드 {modes}, AEB 작동 {aeb_on})')
                continue
            ok = False
            n = min(len(a), len(b))
            first = next((i for i in range(n) if a[i] != b[i]), n)
            print(f'seed {seed}: 불일치 (기준선 {len(a)}개, 현재 {len(b)}개, 첫 차이 #{first})')
            print(f'  기준선 {a[first] if first < len(a) else None}')
            print(f'  현재   {b[first] if first < len(b) else None}')
        print(f'총 비교 메시지 {total}개 -> {"일치" if ok else "차이 있음"}')
        return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
