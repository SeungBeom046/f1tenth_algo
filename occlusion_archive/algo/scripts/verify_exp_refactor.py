#!/usr/bin/env python3
"""exp 노드 순수 모듈 분리(동작 보존 리팩터) 전후 비교. 가짜 rclpy(sim/rosstub.py)로 같은 입력 → 같은 출력.

    sim/.venv/bin/python scripts/verify_exp_refactor.py --baseline <분리 전 커밋>

대상: exp_drive_mux(우선순위·래치), trigger_node(트리거 거리), occupancy_grid_node(프레임 처리·잘라내기),
phantom_node(격자 복원, B0/B1/B2/P). 기준 커밋의 노드 소스를 git에서 읽어 별도 모듈로 실행하고,
작업 트리의 노드와 발행 메시지(토픽, 순서, 값)를 비교한다. 처리 시간(ms), 벽시계 시각(wall_time) 같은 비결정 값만 뺀다.
"""
import argparse
import json
import math
import os
import random
import subprocess
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'sim'))
sys.path.insert(0, os.path.join(ROOT, 'test'))
import rosstub  # noqa: E402

rn = rosstub.install()
import numpy as np  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Joy, PointCloud2, PointField  # noqa: E402
from std_msgs.msg import Bool, Float64, String  # noqa: E402

from my_algo.vehicle_utils import ReactiveDriveCommand, command_to_json  # noqa: E402


def load_old(rev, rel, name):
    src = subprocess.check_output(['git', '-C', ROOT, 'show', f'{rev}:{rel}'], text=True)
    mod = types.ModuleType(name)
    mod.__file__ = os.path.join(ROOT, rel)
    exec(compile(src, rel, 'exec'), mod.__dict__)
    return mod


def norm(v):
    """메시지를 비교 가능한 값으로 (객체 → dict 재귀, float는 repr)."""
    if isinstance(v, float):
        return repr(v)
    if isinstance(v, (int, str, bool)) or v is None:
        return v
    if isinstance(v, (list, tuple)):
        return [norm(x) for x in v]
    if isinstance(v, np.ndarray):
        return norm(v.tolist())
    if hasattr(v, '__dict__'):
        return {k: norm(x) for k, x in vars(v).items()}
    return repr(v)


def drain(drop_ms=False):
    out = []
    for topic, msg in rosstub.take(rn):
        if drop_ms and topic == '/exp/grid_debug':
            d = json.loads(msg.data)
            d.pop('ms', None)
            out.append([topic, d])
        elif topic == '/exp/abort':      # 벽시계 시각은 실행마다 달라 제외 (ros_time·사유·거리는 비교)
            d = json.loads(msg.data)
            d.pop('wall_time', None)
            out.append([topic, d])
        else:
            out.append([topic, norm(msg)])
    return out


# ---------------------------------------------------------------- mux
def run_mux(module, seed, overrides):
    rng = random.Random(seed)
    rosstub.set_time(rn, 0.0)
    rn.BUS.clear()
    node = rosstub.make_node(rn, module.ExpDriveMux, overrides)
    out, t, x = [], 0.0, 0.0
    for k in range(3000):
        t += 0.02
        rosstub.set_time(rn, t)
        r = rng.random()
        if r < 0.01:
            node.subs['/autonomous_mode'](Bool(rng.random() < 0.8))
        if rng.random() < 0.005:
            node.subs['/joy_active'](Bool(rng.random() < 0.3))
        if rng.random() < 0.01:
            node.subs['/exp/preflight_ok'](Bool(rng.random() < 0.85))
        if rng.random() < 0.01:
            node.subs['/aeb/active'](Bool(rng.random() < 0.2))
        if not (300 < k < 330) and rng.random() < 0.9:
            node.subs['/joy'](Joy())
        if not (600 < k < 640) and rng.random() < 0.95:
            node.subs['/exp/v_cmd'](Float64(rng.uniform(0.0, 2.2)))
        if not (900 < k < 930) and rng.random() < 0.95:
            node.subs['/exp/steering_command'](String(command_to_json(
                ReactiveDriveCommand(steering_rad=rng.uniform(-1, 1), mode='WALL_FOLLOW'))))
        if rng.random() < 0.002:
            node.subs['/exp/steering_command'](String('not json'))
        x += rng.uniform(0, 0.05)
        o = Odometry()
        o.pose.pose.position.x = x
        node.subs['/vesc/odom'](o)
        node.tick()
        out.extend(drain())
    return out


# ---------------------------------------------------------------- trigger
def run_trigger(module, seed):
    rng = random.Random(seed)
    rosstub.set_time(rn, 0.0)
    rn.BUS.clear()
    node = rosstub.make_node(rn, module.TriggerNode, {'trigger_enabled': True, 'd_e': rng.choice([1.0, 1.5, 2.0]),
                                                     'collision_x': rng.uniform(4, 6), 'run_id': '7'})
    out, x, t = [], 0.0, 0.0
    node.subs['/autonomous_mode'](Bool(True))
    for _ in range(800):
        t += 0.01
        rosstub.set_time(rn, t)
        x += rng.uniform(0, 0.03)
        o = Odometry()
        o.pose.pose.position.x = x
        o.header.stamp.sec, o.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
        node.subs['/vesc/odom'](o)
        if rng.random() < 0.01:
            node.info()
        out.extend(drain())
    return out, module.trigger_distance(5.0, 1.5, 2.0, 0.05, 0.2)


# ---------------------------------------------------------------- occupancy + phantom
def pose_matrix(x, y, yaw, z=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    m[:3, 3] = [x, y, z]
    return m


def make_cloud(xyz, t):
    msg = PointCloud2()
    msg.header.frame_id = 'livox_frame'
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
    msg.fields = [PointField('x', 0, 7), PointField('y', 4, 7), PointField('z', 8, 7)]
    msg.point_step = 12
    msg.width = len(xyz)
    msg.row_step = 12 * len(xyz)
    msg.data = np.asarray(xyz, dtype='<f4').tobytes()
    return msg


def scenario_frames(seed, n=25):
    import _track
    rng = np.random.default_rng(seed)
    segs = _track.segments(gap=[0.45, 0.15][seed % 2], right_wall=(seed % 3 == 2))
    T_base_lidar = pose_matrix(0.13, -0.005, 0.0, 0.24)
    frames = []
    for k in range(n):
        ex, ey, eyaw = 0.08 * k, 0.01 * math.sin(k / 3), 0.02 * math.sin(k / 5)
        T_odom_base = pose_matrix(ex, ey, eyaw)
        T_odom_lidar = T_odom_base @ T_base_lidar
        origin = T_odom_lidar[:2, 3]
        hits = _track.scan(tuple(origin), segs, step_deg=1.0)
        # 높이: 대역 안·밖(바닥, 박스 위 넘어간 점)을 섞는다. 라이다 z = base z − 0.24
        z_base = rng.choice([0.0, 0.1, 0.2, 0.28, 0.4, 0.6], size=len(hits))
        inv = np.linalg.inv(T_odom_lidar)
        hom = np.column_stack([hits, z_base, np.ones(len(hits))])
        xyz = (hom @ inv.T)[:, :3]
        xyz[:, 2] = z_base - 0.24
        frames.append((0.1 * (k + 1), xyz, T_odom_lidar, T_base_lidar, T_odom_base))
    return frames


def run_grid(module, frames, overrides):
    rosstub.set_time(rn, 0.0)
    rn.BUS.clear()
    state = {}
    module.lookup_matrix = lambda buf, target, source, stamp, timeout_sec=0.05: (
        state['odom_lidar'] if (target, source) == ('odom', 'livox_frame') else
        state['base_lidar'] if (target, source) == ('base_link', 'livox_frame') else
        state['odom_base'] if (target, source) == ('odom', 'base_link') else None)
    node = rosstub.make_node(rn, module.OccupancyGridNode, overrides)
    out, grids = [], []
    for t, xyz, T_ol, T_bl, T_ob in frames:
        rosstub.set_time(rn, t)
        state.update(odom_lidar=T_ol, base_lidar=T_bl, odom_base=T_ob)
        node.callback(make_cloud(xyz, t))
        pub = rosstub.take(rn)
        grids.extend((m, T_ob) for topic, m in pub if topic == '/exp/grid')
        rn.BUS.extend(pub)
        out.extend(drain(drop_ms=True))
    return out, grids


def run_phantom(module, grids, method):
    rosstub.set_time(rn, 0.0)
    rn.BUS.clear()
    # 이후 의도적으로 바꾼 동작(2026-11-05 결정)은 기준 커밋과 같게 맞춰 리팩터만 비교한다:
    # B1 ROI 반폭 1.0 고정, phantom 유지 끔. P의 '앞 끝보다 앞쪽 열만'·w_max는 끌 수 없어 P는 비교하지 않는다
    node = rosstub.make_node(rn, module.PhantomNode, {'method': method, 'b1_roi_half_width': 1.0,
                                                      'phantom_hold': False})
    out = []
    for msg, T_ob in grids:
        pose = (T_ob[0, 3], T_ob[1, 3], math.atan2(T_ob[1, 0], T_ob[0, 0]))
        module.lookup_pose2d = lambda *a, **k: pose
        node.callback(msg)
        out.extend(drain())
    return out


def compare(name, a, b):
    same = a == b
    print(f'{name:38s} {"일치" if same else "불일치"} ({len(a)}개 메시지)')
    if not same:
        n = min(len(a), len(b))
        i = next((i for i in range(n) if a[i] != b[i]), n)
        print(f'   첫 차이 #{i}\n   기준선 {a[i] if i < len(a) else None}\n   현재   {b[i] if i < len(b) else None}')
    return same


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--baseline', required=True)
    args = ap.parse_args()
    import my_algo.exp_drive_mux as new_mux
    import my_algo.occupancy_grid_node as new_grid
    import my_algo.phantom_node as new_ph
    import my_algo.trigger_node as new_trig
    old_mux = load_old(args.baseline, 'my_algo/exp_drive_mux.py', 'old_exp_drive_mux')
    old_trig = load_old(args.baseline, 'my_algo/trigger_node.py', 'old_trigger_node')
    old_grid = load_old(args.baseline, 'my_algo/occupancy_grid_node.py', 'old_occupancy_grid_node')
    old_ph = load_old(args.baseline, 'my_algo/phantom_node.py', 'old_phantom_node')
    ok = True
    total = 0
    for seed in range(6):
        for ov in ({}, {'require_preflight': False}, {'require_joy': False, 'aeb_latch': False}):
            a, b = run_mux(old_mux, seed, ov), run_mux(new_mux, seed, ov)
            ok &= compare(f'mux seed{seed} {ov or "기본"}', a, b)
            total += len(a)
        (a, da), (b, db) = run_trigger(old_trig, seed), run_trigger(new_trig, seed)
        ok &= compare(f'trigger seed{seed}', a, b) and da == db
        total += len(a)
    for seed in range(3):
        frames = scenario_frames(seed)
        for ov in ({}, {'debug_oob': True, 'n_frames': 5, 'known_on': 3}):
            (a, ga), (b, gb) = run_grid(old_grid, frames, ov), run_grid(new_grid, frames, ov)
            ok &= compare(f'occupancy seed{seed} {ov or "기본"}', a, b)
            total += len(a)
            for method in ('B0', 'B1', 'B2'):     # P는 의도적 동작 변경 (위 주석). 37b8a1f 시점 P 포함 일치 기록
                pa, pb = run_phantom(old_ph, gb, method), run_phantom(new_ph, gb, method)
                ok &= compare(f'  phantom {method} seed{seed}', pa, pb)
                total += len(pa)
    print(f'총 비교 메시지 {total}개 -> {"일치" if ok else "차이 있음"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
