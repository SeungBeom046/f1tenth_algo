"""합성 회차 bag(Foxy 메시지, rosbags로 기록) → analysis/extract_run.py 전체 경로 테스트.

rosbags 0.9.23과 pyyaml이 있을 때만 돈다 (pip install -r analysis/requirements.txt).
"""
import importlib.util
import json
import math
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
HAVE = importlib.util.find_spec('rosbags') is not None and importlib.util.find_spec('yaml') is not None


def build_run(run_dir):
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_FOXY)
    T = ts.types

    def stamp(t):
        return T['builtin_interfaces/msg/Time'](sec=int(t), nanosec=int(round((t % 1) * 1e9)))

    def header(t, frame=''):
        return T['std_msgs/msg/Header'](stamp=stamp(t), frame_id=frame)

    v3 = lambda x=0.0, y=0.0, z=0.0: T['geometry_msgs/msg/Vector3'](x=x, y=y, z=z)  # noqa: E731
    pt = lambda x=0.0, y=0.0, z=0.0: T['geometry_msgs/msg/Point'](x=x, y=y, z=z)  # noqa: E731
    quat = T['geometry_msgs/msg/Quaternion'](x=0.0, y=0.0, z=0.0, w=1.0)
    cov = np.zeros(36)

    os.makedirs(os.path.join(run_dir, 'config_snapshot'))
    for name in ('common.yaml', 'scenario_S2.yaml', 'vehicle_geometry.yaml'):
        shutil.copy(os.path.join(ROOT, 'config', name), os.path.join(run_dir, 'config_snapshot'))
    with open(os.path.join(run_dir, 'meta.json'), 'w') as f:
        json.dump({'method': 'P', 'scenario': 'S2', 'd_e': None, 'run_id': 1, 'discard': False,
                   'odom_topic': '/vesc/odom'}, f)

    def speed(s):  # 4.0–5.0 m 구간만 1.0 m/s, 나머지 2.0 (가속 구간 무시)
        return 1.0 if 4.0 <= s < 5.0 else 2.0

    with Writer(os.path.join(run_dir, 'bag')) as w:
        c = {k: w.add_connection(k, t, typestore=ts) for k, t in [
            ('/autonomous_mode', 'std_msgs/msg/Bool'), ('/vesc/odom', 'nav_msgs/msg/Odometry'),
            ('/exp/v_raw', 'std_msgs/msg/Float64'), ('/exp/final_cmd', 'std_msgs/msg/Float64'),
            ('/tf_static', 'tf2_msgs/msg/TFMessage'), ('/scan', 'sensor_msgs/msg/LaserScan'),
            ('/imu/linear_accel_base', 'geometry_msgs/msg/Vector3Stamped'),
            ('/aeb/active', 'std_msgs/msg/Bool'), ('/exp/min_speed_flag', 'std_msgs/msg/String'),
            ('/exp/boundary_frame', 'nav_msgs/msg/GridCells'), ('/exp/phantom_debug', 'std_msgs/msg/String')]}

        def put(topic, t, msg):
            w.write(c[topic], int(t * 1e9), ts.serialize_cdr(msg, c[topic].msgtype))

        tf = T['geometry_msgs/msg/TransformStamped'](
            header=header(0.0, 'base_link'), child_frame_id='livox_frame',
            transform=T['geometry_msgs/msg/Transform'](translation=v3(0.13, -0.005, 0.24), rotation=quat))
        put('/tf_static', 0.0, T['tf2_msgs/msg/TFMessage'](transforms=[tf]))
        put('/autonomous_mode', 0.4, T['std_msgs/msg/Bool'](data=False))
        put('/autonomous_mode', 0.5, T['std_msgs/msg/Bool'](data=True))
        t, s, dt = 0.5, 0.0, 0.005
        while s < 8.0:
            v = speed(s)
            odom = T['nav_msgs/msg/Odometry'](
                header=header(t, 'odom'), child_frame_id='base_link',
                pose=T['geometry_msgs/msg/PoseWithCovariance'](
                    pose=T['geometry_msgs/msg/Pose'](position=pt(s, 0.0, 0.0), orientation=quat), covariance=cov),
                twist=T['geometry_msgs/msg/TwistWithCovariance'](
                    twist=T['geometry_msgs/msg/Twist'](linear=v3(v), angular=v3()), covariance=cov))
            if round(t / dt) % 4 == 0:
                put('/vesc/odom', t, odom)
                put('/exp/v_raw', t, T['std_msgs/msg/Float64'](data=v))
                put('/exp/final_cmd', t, T['std_msgs/msg/Float64'](data=v))
            put('/imu/linear_accel_base', t, T['geometry_msgs/msg/Vector3Stamped'](
                header=header(t, 'base_link'), vector=v3(0.0)))
            if round(t / dt) % 20 == 0:
                # 차선 안 정면 장애물: 차 앞 끝에서 (6.5 - s) - 0.41 m
                n = 360
                ranges = np.full(n, np.inf, dtype=np.float32)
                ranges[n // 2] = max(0.1, 6.5 - s - 0.13)
                put('/scan', t, T['sensor_msgs/msg/LaserScan'](
                    header=header(t, 'livox_frame'), angle_min=-math.pi, angle_max=math.pi,
                    angle_increment=2 * math.pi / n, time_increment=0.0, scan_time=0.1,
                    range_min=0.05, range_max=20.0, ranges=ranges, intensities=np.zeros(n, np.float32)))
                put('/aeb/active', t, T['std_msgs/msg/Bool'](data=False))
                put('/exp/min_speed_flag', t, T['std_msgs/msg/String'](data=''))
                put('/exp/boundary_frame', t, T['nav_msgs/msg/GridCells'](
                    header=header(t, 'odom'), cell_width=0.05, cell_height=0.05,
                    cells=[pt(x, -0.5, 0.0) for x in np.arange(3.0, 4.0, 0.1)]))
                put('/exp/phantom_debug', t, T['std_msgs/msg/String'](data=json.dumps({'n_phantoms': 2})))
            s += v * dt
            t += dt


@unittest.skipUnless(HAVE, 'rosbags 0.9.23 / pyyaml 없음 (analysis/requirements.txt)')
class AnalysisPipelineTest(unittest.TestCase):
    def test_extract_run_from_foxy_bag(self):
        sys.path.insert(0, os.path.join(ROOT, 'analysis'))
        import extract_run
        with tempfile.TemporaryDirectory() as d:
            run = os.path.join(d, 'S2_P_dena_run01')
            build_run(run)
            m = extract_run.extract(run)
            # 시험 구간 2.5–7.0 m 중 1 m를 1 m/s로: ΔT = 1/1 - 1/2 = 0.5 s
            self.assertAlmostEqual(m['dT_cmd'], 0.5, delta=0.03)
            self.assertAlmostEqual(m['dT_meas'], 0.5, delta=0.03)
            self.assertAlmostEqual(m['min_speed_cmd'], 1.0, places=6)
            self.assertAlmostEqual(m['decel_length_cmd'], 1.0, delta=0.03)
            self.assertTrue(m['vref_reached'])
            self.assertFalse(m['aeb_malfunction'])
            self.assertEqual(m['boundary_frames'] > 0, True)
            # 장애물은 x=6.5에 정지. 차가 8 m 전에 지나가므로 마지막엔 거리 0 근처 → 충돌로 잡혀야 한다
            self.assertTrue(m['collision'])
            self.assertTrue(os.path.exists(os.path.join(run, 'analysis', 'speed_vs_s.csv')))


if __name__ == '__main__':
    unittest.main()
