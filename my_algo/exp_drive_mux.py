"""exp 모드 최종 명령 노드. 우선순위대로 정지 조건을 검사하고 VESC 명령을 낸다.

역할 (우선순위 높은 순):
    1. 비상 정지: /autonomous_mode=false 또는 /joy_active=true → 명령 발행 중단 (joy_controller가 0을 냄)
       autonomous_drive와 같은 규칙. 조이스틱 B 버튼이 속도 계층·AEB보다 우선한다.
    1-1. 발행자 점검: /exp/preflight_ok가 true가 아니면 출발 거부 (0 명령, 'preflight_failed').
       주행을 시작한 뒤 위반이 생기면 정지 + 회차 폐기 ('foreign_publisher') [EXP-FIXED]
    2. 조이스틱 끊김 감시(데드맨): /joy가 joy_timeout 이상 안 오면 0 명령, 회차 폐기 기록 [EXP-FIXED]
    3. 소프트웨어 정지선: AUTO 시작 후 odom 주행 거리 > stop_distance 이면 0 명령 (래치) [EXP-FIXED]
    4. AEB: /aeb/active=true → 0 명령 (aeb_latch=true면 회차 끝까지 유지) [EXP-FIXED]
    5. 입력 끊김: 조향 명령이나 속도 명령이 input_timeout보다 오래되면 0 명령
    6. 정상: 속도 = /exp/v_cmd, 조향 = /exp/steering_command
파이프라인 위치: [speed_layer_node, exp_aeb_node, exp_steering_node, joy_controller] → (exp_drive_mux) → [VESC]
구독: /exp/v_cmd (Float64), /exp/steering_command (String JSON), /aeb/active (Bool),
      /exp/preflight_ok (Bool, transient_local),
      /autonomous_mode (Bool), /joy_active (Bool), /joy (sensor_msgs/Joy), odom_topic (Odometry)
발행 (50 Hz):
    /commands/motor/speed (Float64 ERPM), /commands/servo/position (Float64)
    /exp/final_cmd (Float64 m/s, 실제로 보낸 속도), /exp/stop_reason (String), /exp/abort (String JSON, 폐기 사유)
파라미터 (config/common.yaml): joy_timeout, require_joy, aeb_latch, input_timeout, max_steering,
          require_preflight
          (config/scenario_*.yaml): stop_distance
          run_dir: 폐기 사유를 <run_dir>/abort.json에 기록 (record_run.sh가 meta.json에 합침)
변환: ERPM = vehicle_utils.speed_to_erpm (ERPM_GAIN 단일 출처), sanitize_rpm 최소 속도 규칙 그대로.
"""

import json
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64, String

from my_algo.ros_util import publish_float, publish_string
from my_algo.vehicle_utils import (
    command_from_json, sanitize_rpm, speed_to_erpm, steering_to_servo,
)


DISCARD_REASONS = ('joy_timeout', 'foreign_publisher')


class ExpDriveMux(Node):
    def __init__(self):
        super().__init__('exp_drive_mux')
        defaults = [('odom_topic', '/vesc/odom'), ('joy_topic', '/joy'),
                    ('motor_topic', '/commands/motor/speed'), ('servo_topic', '/commands/servo/position'),
                    ('rate_hz', 50.0), ('joy_timeout', 0.5), ('require_joy', True),
                    ('aeb_latch', True), ('input_timeout', 0.3), ('max_steering', 0.78),
                    ('stop_distance', 7.5), ('start_autonomous', False), ('run_dir', ''),
                    ('require_preflight', True)]
        for k, v in defaults:
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.joy_timeout = float(gp('joy_timeout'))
        self.require_joy = bool(gp('require_joy'))
        self.aeb_latch = bool(gp('aeb_latch'))
        self.input_timeout = float(gp('input_timeout'))
        self.max_steering = float(gp('max_steering'))
        self.stop_distance = float(gp('stop_distance'))
        self.run_dir = gp('run_dir')
        self.require_preflight = bool(gp('require_preflight'))
        self.preflight_ok = False
        self.has_driven = False

        self.auto_mode = bool(gp('start_autonomous'))
        self.joy_active = False
        self.last_joy = None
        self.aeb = False
        self.v_cmd, self.v_time = 0.0, None
        self.steer, self.steer_time = 0.0, None
        self.odom_xy, self.travel = None, 0.0
        self.latched = ''     # 회차 끝까지 유지되는 정지 사유
        self.was_running = False

        self.create_subscription(Float64, '/exp/v_cmd', self.on_v, 10)
        self.create_subscription(String, '/exp/steering_command', self.on_steer, 10)
        self.create_subscription(Bool, '/aeb/active', self.on_aeb, 10)
        self.create_subscription(Bool, '/autonomous_mode', self.on_auto, 10)
        self.create_subscription(Bool, '/joy_active', self.on_joy_active, 10)
        self.create_subscription(Joy, gp('joy_topic'), self.on_joy, 10)
        self.create_subscription(Bool, '/exp/preflight_ok', self.on_preflight,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 10)
        self.motor_pub = self.create_publisher(Float64, gp('motor_topic'), 10)
        self.servo_pub = self.create_publisher(Float64, gp('servo_topic'), 10)
        self.final_pub = self.create_publisher(Float64, '/exp/final_cmd', 10)
        self.reason_pub = self.create_publisher(String, '/exp/stop_reason', 10)
        self.abort_pub = self.create_publisher(String, '/exp/abort', 10)
        self.create_timer(1.0 / float(gp('rate_hz')), self.tick)
        self.get_logger().info(
            f'exp mux: joy_timeout={self.joy_timeout} require_joy={self.require_joy} '
            f'aeb_latch={self.aeb_latch} stop_distance={self.stop_distance}')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_v(self, msg):
        self.v_cmd, self.v_time = msg.data, self.now_s()

    def on_steer(self, msg):
        cmd = command_from_json(msg.data)
        if cmd.is_valid():
            self.steer, self.steer_time = cmd.steering_rad, self.now_s()

    def on_aeb(self, msg):
        self.aeb = bool(msg.data)

    def on_auto(self, msg):
        if self.auto_mode and not msg.data:
            self.send(0.0, 0.0, 'auto_off')
        if not self.auto_mode and msg.data:
            self.travel = 0.0   # 정지선 거리는 AUTO 시작부터 센다
            self.odom_xy = None
        self.auto_mode = bool(msg.data)

    def on_joy_active(self, msg):
        self.joy_active = bool(msg.data)

    def on_preflight(self, msg):
        self.preflight_ok = bool(msg.data)

    def on_joy(self, msg):
        self.last_joy = self.now_s()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.odom_xy is not None and self.auto_mode:
            self.travel += math.hypot(p.x - self.odom_xy[0], p.y - self.odom_xy[1])
        self.odom_xy = (p.x, p.y)

    def abort(self, reason):
        """회차 폐기 사유 기록. 같은 사유는 한 번만 쓴다."""
        if self.latched == reason:
            return
        self.latched = reason
        record = {'reason': reason, 'ros_time': self.now_s(), 'wall_time': time.time(),
                  'travel_m': round(self.travel, 3)}
        publish_string(self.abort_pub, json.dumps(record))
        self.get_logger().warn(f'회차 정지: {reason}')
        if not self.run_dir:
            return
        try:
            os.makedirs(self.run_dir, exist_ok=True)
            with open(os.path.join(self.run_dir, 'mux_events.jsonl'), 'a', encoding='utf-8') as f:
                f.write(json.dumps(record) + '\n')
            # 조이스틱 끊김은 실험 조건 위반이므로 폐기 회차로 표시한다. AEB·정지선은 정상 결과.
            if reason in DISCARD_REASONS:
                with open(os.path.join(self.run_dir, 'abort.json'), 'w', encoding='utf-8') as f:
                    json.dump(dict(record, discard=True), f)
        except OSError as exc:
            self.get_logger().error(f'이벤트 기록 실패: {exc}')

    def tick(self):
        now = self.now_s()
        # 1. 비상 정지: 사람이 잡고 있으면 아무것도 보내지 않는다 (joy_controller가 0을 보냄)
        if self.joy_active or not self.auto_mode:
            if self.was_running:
                self.send(0.0, 0.0, 'manual_or_auto_off')
                self.was_running = False
            return
        self.was_running = True
        # 1-1. 발행자 점검
        if self.require_preflight and not self.preflight_ok:
            if self.has_driven:
                self.abort('foreign_publisher')
            elif not self.latched:
                self.send(0.0, 0.0, 'preflight_failed')
                return
        # 2. 데드맨
        if self.require_joy and (self.last_joy is None or now - self.last_joy > self.joy_timeout):
            self.abort('joy_timeout')
        # 3. 정지선
        if self.travel > self.stop_distance and not self.latched:
            self.abort('stop_line')
        # 4. AEB
        if self.aeb and self.aeb_latch and not self.latched:
            self.abort('aeb')
        if self.latched:
            self.send(0.0, self.steer, self.latched)
            return
        if self.aeb:
            self.send(0.0, self.steer, 'aeb')
            return
        # 5. 입력 끊김
        if self.v_time is None or now - self.v_time > self.input_timeout:
            self.send(0.0, self.steer, 'stale_speed')
            return
        if self.steer_time is None or now - self.steer_time > self.input_timeout:
            self.send(0.0, 0.0, 'stale_steering')
            return
        # 6. 정상
        self.has_driven = True
        self.send(self.v_cmd, self.steer, '')

    def send(self, speed, steering, reason):
        steering = max(-self.max_steering, min(self.max_steering, steering))
        rpm = sanitize_rpm(speed_to_erpm(max(0.0, speed)))
        publish_float(self.motor_pub, rpm)
        publish_float(self.servo_pub, steering_to_servo(steering))
        publish_float(self.final_pub, rpm / speed_to_erpm(1.0))
        publish_string(self.reason_pub, reason)


def main(args=None):
    rclpy.init(args=args)
    node = ExpDriveMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0, 'shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
