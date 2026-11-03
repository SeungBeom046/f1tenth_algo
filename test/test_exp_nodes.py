"""exp 노드 로직 테스트 (가짜 rclpy). 실제 ROS 환경에서는 건너뛴다."""
import contextlib
import io
import math
import unittest

import _rosstub  # noqa: F401  가짜 rclpy를 먼저 설치

if _rosstub.ACTIVE:
    import rclpy.node as rn
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Joy, LaserScan
    from std_msgs.msg import Bool, Float64, String

    import my_algo.exp_drive_mux as mux_mod
    import my_algo.popout_vehicle_node as popout_mod
    from std_msgs.msg import Header
    import my_algo.exp_steering_node as steer_mod
    import my_algo.speed_layer_node as speed_mod
    from my_algo.geometry import VehicleGeometry
    from my_algo.speed_policy import V_MIN
    from my_algo.vehicle_utils import command_to_json, ReactiveDriveCommand, ERPM_GAIN

GEOM = None if not _rosstub.ACTIVE else VehicleGeometry(
    0.33, 0.41, 0.09, 0.27, ((0.41, 0.135), (-0.09, 0.135), (-0.09, -0.135), (0.41, -0.135)), 0.28)


def make(cls, **overrides):
    rn.OVERRIDES = overrides
    rn.BUS.clear()
    with contextlib.redirect_stdout(io.StringIO()):
        return cls()


def advance(sec):
    rn.CLOCK['ns'] += int(sec * 1e9)


def published(topic):
    return [m.data for t, m in rn.BUS if t == topic]


def wall_scan(left_y, right_y=None, front_x=None):
    """좌측 벽(y=left_y), 선택: 오른쪽 벽, 전방 장애물. 1도 간격."""
    m = LaserScan()
    n = 360
    m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / n
    m.range_min, m.range_max = 0.05, 20.0
    out = []
    for i in range(n):
        a = m.angle_min + i * m.angle_increment
        r = float('inf')
        s = math.sin(a)
        if s > 1e-3:
            r = min(r, left_y / s)
        if right_y is not None and s < -1e-3:
            r = min(r, right_y / s)
        if front_x is not None and math.cos(a) > 1e-3 and abs(front_x * math.tan(a)) < 0.2:
            r = min(r, front_x / math.cos(a))
        out.append(r if r < 20 else float('inf'))
    m.ranges = out
    return m


def odom(v, x=0.0):
    o = Odometry()
    o.twist.twist.linear.x = v
    o.pose.pose.position.x = x
    return o


@unittest.skipUnless(_rosstub.ACTIVE, '실제 ROS 환경: 가짜 rclpy 테스트 건너뜀')
class ExpSteeringTest(unittest.TestCase):
    def setUp(self):
        steer_mod.load_geometry = lambda path: GEOM

    def run_seq(self, speeds, right_y=None, front_x=None, front_from=0, cls=None):
        if cls is None:
            node = make(steer_mod.ExpSteeringNode, vehicle_geometry_file='x',
                        target_wall_distance=0.55, wall_follow_gain=0.30)
        else:
            node = make(cls)
        out = []
        for k, v in enumerate(speeds):
            advance(0.1)
            node.subs['/vesc/odom'](odom(v))
            rn.BUS.clear()
            fx = front_x if k >= front_from else None
            with contextlib.redirect_stdout(io.StringIO()):
                node.subs['/scan'](wall_scan(0.80 + 0.15 * math.sin(k / 3), right_y, fx))
            out.append(published('/reactive/lidar_command')[-1])
        return out

    def steering(self, cmds):
        import json
        return [json.loads(c)['steering_rad'] for c in cmds]

    def test_steering_independent_of_measured_speed(self):
        slow = self.steering(self.run_seq([0.5] * 30))
        fast = self.steering(self.run_seq([2.0] * 30))
        self.assertGreater(max(abs(x) for x in fast), 0.05)   # 조향이 실제로 나와야 의미가 있다
        self.assertEqual(slow, fast)

    def test_right_side_ignored_and_mode_fixed(self):
        import json
        no_right = self.run_seq([1.0] * 20)
        with_right = self.run_seq([1.0] * 20, right_y=-0.40)
        self.assertGreater(max(abs(x) for x in self.steering(no_right)), 0.05)
        self.assertEqual(self.steering(no_right), self.steering(with_right))
        self.assertTrue(all(json.loads(c)['mode'] in ('WALL_FOLLOW', 'STOP') for c in with_right))

    def test_stop_holds_previous_steering(self):
        import json
        cmds = self.run_seq([2.0] * 20, front_x=0.6, front_from=12)
        modes = [json.loads(c)['mode'] for c in cmds]
        steer = self.steering(cmds)
        self.assertEqual(modes[12:], ['STOP'] * 8)
        self.assertNotEqual(steer[11], 0.0)
        self.assertTrue(all(x == steer[11] for x in steer[12:]))

    def test_no_obstacle_avoidance_mode(self):
        import json
        from my_algo.lidar_reactive_drive import LidarReactiveDriveNode
        args = dict(speeds=[0.5] * 20, right_y=-0.40, front_x=1.2, front_from=5)
        parent = [json.loads(c)['mode'] for c in self.run_seq(cls=LidarReactiveDriveNode, **args)]
        exp = [json.loads(c)['mode'] for c in self.run_seq(**args)]
        self.assertIn('OBSTACLE_AVOIDANCE', parent)   # 같은 입력에서 drive planner는 회피로 전환
        self.assertNotIn('OBSTACLE_AVOIDANCE', exp)


@unittest.skipUnless(_rosstub.ACTIVE, '실제 ROS 환경: 가짜 rclpy 테스트 건너뜀')
class ExpDriveMuxTest(unittest.TestCase):
    def setUp(self):
        self.m = make(mux_mod.ExpDriveMux, stop_distance=5.0)
        self.m.subs['/exp/preflight_ok'](Bool(True))

    def feed(self, v=1.5, steer=0.1, joy=True):
        self.m.subs['/exp/v_cmd'](Float64(v))
        self.m.subs['/exp/steering_command'](
            String(command_to_json(ReactiveDriveCommand(steering_rad=steer, mode='WALL_FOLLOW'))))
        if joy:
            self.m.subs['/joy'](Joy())

    def tick(self):
        rn.BUS.clear()
        self.m.tick()
        return published('/commands/motor/speed'), published('/exp/stop_reason')

    def test_not_auto_publishes_nothing(self):
        self.feed()
        motor, _ = self.tick()
        self.assertEqual(motor, [])

    def test_normal_and_joy_priority(self):
        self.m.subs['/autonomous_mode'](Bool(True))
        self.feed(v=1.5)
        motor, reason = self.tick()
        self.assertAlmostEqual(motor[0], 1.5 * ERPM_GAIN)
        self.assertEqual(reason, [''])
        self.m.subs['/joy_active'](Bool(True))          # B 버튼: joy_controller가 잡음
        motor, reason = self.tick()
        self.assertEqual(motor, [0.0])                    # 전환 순간 한 번 0
        motor, _ = self.tick()
        self.assertEqual(motor, [])                       # 이후 아무것도 안 보냄

    def test_deadman_latches_and_marks_discard(self):
        self.m.subs['/autonomous_mode'](Bool(True))
        self.feed()
        self.tick()
        advance(0.6)
        self.feed(joy=False)
        motor, reason = self.tick()
        self.assertEqual((motor, reason), ([0.0], ['joy_timeout']))
        self.feed(joy=True)
        motor, reason = self.tick()
        self.assertEqual((motor, reason), ([0.0], ['joy_timeout']))   # 회차 끝까지 유지
        self.assertIn('joy_timeout', mux_mod.DISCARD_REASONS)

    def test_aeb_latch_over_speed(self):
        self.m.subs['/autonomous_mode'](Bool(True))
        self.feed(v=2.0)
        self.m.subs['/aeb/active'](Bool(True))
        self.assertEqual(self.tick(), ([0.0], ['aeb']))
        self.m.subs['/aeb/active'](Bool(False))
        self.feed(v=2.0)
        self.assertEqual(self.tick(), ([0.0], ['aeb']))

    def test_stop_line(self):
        self.m.subs['/autonomous_mode'](Bool(True))
        self.m.subs['/vesc/odom'](odom(1.0, x=0.0))
        self.m.subs['/vesc/odom'](odom(1.0, x=5.1))
        self.feed()
        self.assertEqual(self.tick(), ([0.0], ['stop_line']))

    def test_preflight_refuses_start_then_discards_mid_run(self):
        m = make(mux_mod.ExpDriveMux, stop_distance=5.0)
        m.subs['/autonomous_mode'](Bool(True))
        self.m = m
        self.feed()
        self.assertEqual(self.tick(), ([0.0], ['preflight_failed']))   # 점검 전: 출발 거부
        m.subs['/exp/preflight_ok'](Bool(True))
        self.feed()
        self.assertEqual(self.tick()[1], [''])                          # 통과 후 주행
        m.subs['/exp/preflight_ok'](Bool(False))                       # 주행 중 다른 발행자 등장
        self.feed()
        self.assertEqual(self.tick(), ([0.0], ['foreign_publisher']))
        self.assertIn('foreign_publisher', mux_mod.DISCARD_REASONS)

    def test_stale_speed_stops(self):
        self.m.subs['/autonomous_mode'](Bool(True))
        self.feed()
        advance(0.31)
        self.m.subs['/joy'](Joy())
        self.assertEqual(self.tick(), ([0.0], ['stale_speed']))


@unittest.skipUnless(_rosstub.ACTIVE, '실제 ROS 환경: 가짜 rclpy 테스트 건너뜀')
class PopoutTest(unittest.TestCase):
    def test_trigger_run_stop_reset(self):
        import tempfile
        n = make(popout_mod.PopoutVehicleNode, stop_distance=0.5, log_dir=tempfile.mkdtemp(),
                 lane_boundary_distance=0.2)
        n.subs['odom'](odom(0.0, x=0.0))       # 출발 전 정지 위치
        n.subs['/trigger'](Header())
        x = 0.0
        for _ in range(100):
            advance(0.02)
            x += 0.8 * 0.02
            n.subs['odom'](odom(0.8, x=x))
            rn.BUS.clear()
            n.tick()
            if n.state == 'stopped':
                break
        self.assertEqual(n.state, 'stopped')
        self.assertEqual(n.event['stop_reason'], 'distance')
        self.assertAlmostEqual(n.event['reach_lane_s'], 0.25, delta=0.021)  # odom 한 주기 이내
        self.assertAlmostEqual(n.event['speed_mean'], 0.8, places=6)
        n.subs['/trigger'](Header())          # 정지 상태에서는 다시 출발하지 않는다
        self.assertEqual(n.state, 'stopped')
        n.subs['/popout/reset'](Bool(True))
        self.assertEqual(n.state, 'idle')


@unittest.skipUnless(_rosstub.ACTIVE, '실제 ROS 환경: 가짜 rclpy 테스트 건너뜀')
class SpeedLayerTest(unittest.TestCase):
    def tick(self, node):
        rn.BUS.clear()
        node.tick()
        return (published('/exp/v_raw')[0], published('/exp/v_cmd')[0],
                published('/exp/min_speed_flag')[0])

    def test_b2_vcap_and_stale(self):
        n = make(speed_mod.SpeedLayerNode, method='B2', v_imp=V_MIN)
        n.subs['/exp/phantom_d'](Float64(math.inf))
        self.assertEqual(self.tick(n), (2.0, 2.0, ''))
        n.subs['/exp/phantom_d'](Float64(0.3))
        raw, cmd, flag = self.tick(n)
        self.assertLess(raw, V_MIN)
        self.assertEqual((cmd, flag), (V_MIN, 'clamped_to_vmin'))
        advance(0.31)
        self.assertEqual(self.tick(n)[1:], (0.0, 'stale_perception'))

    def test_b0_ignores_phantom(self):
        n = make(speed_mod.SpeedLayerNode, method='B0')
        n.subs['/exp/phantom_d'](Float64(0.3))
        self.assertEqual(self.tick(n), (2.0, 2.0, ''))


if __name__ == '__main__':
    unittest.main()
