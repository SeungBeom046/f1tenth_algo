"""돌출 차량 #2 노드 (#2 Jetson에서 실행). /trigger를 받으면 0.8 m/s 직진 후 정지한다.

역할: 트리거 수신 → 속도 v_popout, 조향 0 → 지정 거리 또는 지정 시간 후 정지, 최대 시간 넘으면 무조건 정지.
    트리거 수신 시각과 출발(첫 명령) 시각을 기록한다.
파이프라인 위치: [에고 trigger_node] → /trigger → (popout_vehicle_node) → [#2 VESC]
구독: /trigger (std_msgs/Header, 에고와 같은 ROS_DOMAIN_ID), odom_topic (#2 odom, 거리 정지용),
      /popout/reset (std_msgs/Bool, true면 정지 상태 → 대기. 회차마다 #2를 출발 위치에 둔 뒤 발행)
발행 (50 Hz, 동작 중): motor_topic (Float64 ERPM), servo_topic (Float64), /popout/status (String JSON)

주의 (같은 ROS_DOMAIN_ID): 에고와 #2의 VESC 토픽 이름이 같으면 서로의 명령이 섞인다.
    기본 토픽은 상대 이름(commands/motor/speed 등)이라 launch에서 namespace(예: car2)를 주면
    /car2/commands/... 로 분리된다. #2의 VESC 드라이버도 같은 namespace로 띄우고,
    #2의 vesc_to_odom은 publish_tf=false로 둔다 (/tf는 namespace가 적용되지 않아 에고 TF와 섞임).
파라미터 ([EXP-FIXED], config/popout.yaml):
    v_popout 0.8, stop_distance, stop_time, max_time, erpm_gain(#2 실측), servo_center(#2)
    #2 명령에는 에고의 최소 RPM 규칙(sanitize_rpm)을 적용하지 않는다 (0.8 m/s = 3691 ERPM < 4000).
    lane_boundary_distance: 출발 위치 → 차선 경계까지 거리. 이 거리에 닿은 시각을 기록 (Exp0 진입 시간)
Exp0 #2 단독 시험: ros2 topic pub --once /trigger std_msgs/msg/Header "{frame_id: exp0}" 로 출발시키고
    /popout/status 또는 popout_events.jsonl의 reach_lane_s, speed_mean, speed_std를 기록한다.
"""

import json
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Header, String

from my_algo.ros_util import publish_float, publish_string
from my_algo.vehicle_utils import ERPM_GAIN, SERVO_CENTER


class PopoutVehicleNode(Node):
    def __init__(self):
        super().__init__('popout_vehicle_node')
        defaults = [('trigger_topic', '/trigger'), ('odom_topic', 'odom'),
                    ('motor_topic', 'commands/motor/speed'), ('servo_topic', 'commands/servo/position'),
                    ('v_popout', 0.8), ('stop_distance', 1.0), ('stop_time', 0.0), ('max_time', 3.0),
                    ('erpm_gain', ERPM_GAIN), ('servo_center', SERVO_CENTER),
                    ('lane_boundary_distance', 0.3),
                    ('rate_hz', 50.0), ('log_dir', '~/exp_logs/popout')]
        for k, v in defaults:
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.v = float(gp('v_popout'))
        self.stop_distance = float(gp('stop_distance'))
        self.stop_time = float(gp('stop_time'))
        self.max_time = float(gp('max_time'))
        self.erpm_gain = float(gp('erpm_gain'))
        self.servo_center = float(gp('servo_center'))
        self.log_dir = os.path.expanduser(gp('log_dir'))
        self.lane_boundary_distance = float(gp('lane_boundary_distance'))
        self.speeds = []
        self.state = 'idle'
        self.start = None
        self.travel = 0.0
        self.last_xy = None
        self.event = {}
        self.create_subscription(Header, gp('trigger_topic'), self.on_trigger, 10)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 50)
        self.create_subscription(Bool, '/popout/reset', self.on_reset, 10)
        self.motor_pub = self.create_publisher(Float64, gp('motor_topic'), 10)
        self.servo_pub = self.create_publisher(Float64, gp('servo_topic'), 10)
        self.status_pub = self.create_publisher(String, '/popout/status', 10)
        self.create_timer(1.0 / float(gp('rate_hz')), self.tick)
        self.get_logger().info(
            f'#2 대기: v={self.v} m/s ({self.v * self.erpm_gain:.0f} ERPM), '
            f'정지 거리 {self.stop_distance} m / 시간 {self.stop_time} s / 최대 {self.max_time} s, '
            f'motor_topic={self.motor_pub.topic_name}')

    def ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_trigger(self, msg):
        if self.state != 'idle':
            return
        recv = self.ros_now()
        sent = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.event = {'run_id': msg.frame_id, 'trigger_stamp': sent, 'recv_time': recv,
                      'recv_wall_time': time.time(), 'latency_s': recv - sent}
        self.state = 'running'
        self.start = recv
        # 트리거 전 정지 위치(last_xy)를 출발점으로 둔다. None으로 지우면 첫 odom 구간 이동을 놓친다.
        self.travel = 0.0
        self.speeds = []
        self.get_logger().info(f'트리거 수신: 지연 {recv - sent:+.4f} s (두 Jetson 시간 동기 전제)')

    def on_reset(self, msg):
        if msg.data and self.state == 'stopped':
            self.state = 'idle'
            self.event = {}
            self.get_logger().info('#2 대기 상태로 복귀 (출발 위치 확인할 것)')
        elif msg.data and self.state == 'running':
            self.get_logger().warn('주행 중에는 reset을 무시한다')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.state == 'running' and self.last_xy is not None:
            self.travel += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
            self.speeds.append(math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y))
            if 'reach_lane_s' not in self.event and self.travel >= self.lane_boundary_distance:
                self.event['reach_lane_s'] = self.ros_now() - self.start
        self.last_xy = (p.x, p.y)

    def tick(self):
        if self.state != 'running':
            if self.state == 'stopped':
                self.command(0.0)
            return
        elapsed = self.ros_now() - self.start
        reason = ''
        if elapsed >= self.max_time:
            reason = 'max_time'
        elif self.stop_distance > 0.0 and self.travel >= self.stop_distance:
            reason = 'distance'
        elif self.stop_time > 0.0 and elapsed >= self.stop_time:
            reason = 'time'
        if reason:
            self.finish(reason, elapsed)
            return
        if 'start_time' not in self.event:
            self.event['start_time'] = self.ros_now()
        self.command(self.v)

    def command(self, v):
        publish_float(self.motor_pub, v * self.erpm_gain)
        publish_float(self.servo_pub, self.servo_center)

    def finish(self, reason, elapsed):
        self.command(0.0)
        self.state = 'stopped'
        # 정상 속도 구간(최대값의 50% 이상) 통계: 출발 가속 구간을 빼고 속도 편차를 본다
        cruise = [v for v in self.speeds if self.speeds and v >= 0.5 * max(self.speeds)]
        mean = sum(cruise) / len(cruise) if cruise else None
        std = (math.sqrt(sum((v - mean) ** 2 for v in cruise) / len(cruise)) if cruise else None)
        self.event.update(stop_reason=reason, elapsed_s=elapsed, travel_m=self.travel,
                          v_command=self.v, speed_mean=mean, speed_std=std)
        publish_string(self.status_pub, json.dumps(self.event))
        self.get_logger().info(f'#2 정지: {self.event}')
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            with open(os.path.join(self.log_dir, 'popout_events.jsonl'), 'a', encoding='utf-8') as f:
                f.write(json.dumps(self.event) + '\n')
        except OSError as exc:
            self.get_logger().error(f'기록 실패: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = PopoutVehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.command(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
