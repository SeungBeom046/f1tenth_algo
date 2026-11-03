"""에고 트리거 노드 (exp 모드). 에고 주행 거리가 트리거 거리를 지나면 /trigger를 한 번 발행한다.

역할: 돌출 차량 #2 출발 신호. 발행 시각을 기록해 Exp0에서 지연을 측정한다.
트리거 거리 (2.10절):
    d_e 정의: #2의 앞부분이 차선 경계에 닿는 순간, 에고 차체 앞 끝과 충돌 지점 사이 거리
    trigger_distance = collision_x - d_e - v_ref * (trigger_latency + popout_enter_time)
    collision_x: AUTO 시작 시 에고 차체 앞 끝에서 충돌 지점(#2 진입선과 에고 경로의 교점)까지 거리 [m]
    주행 거리는 AUTO 시작부터 odom 위치 변화를 누적한 값 (앞 끝 이동 거리와 같다)
파이프라인 위치: [odom] → (trigger_node) → /trigger → [#2 popout_vehicle_node]
구독: odom_topic (nav_msgs/Odometry), /autonomous_mode (std_msgs/Bool)
발행: /trigger (std_msgs/Header, 1회, stamp=발행 시각, frame_id=run_id)
      /trigger/info (std_msgs/String JSON, 1 Hz: 계산된 trigger_distance, 현재 거리, 발행 여부)
파라미터:
    d_e [EXP-VAR] (launch 인자), collision_x (config/scenario_S1.yaml)
    trigger_latency, popout_enter_time [EXP0-MEASURE] (config/common.yaml)
    v_ref [EXP-FIXED] (config/common.yaml)
    trigger_enabled: 돌출 없는 시나리오(S2-S4)에서는 false
"""

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Header, String

from my_algo.ros_util import publish_string


def trigger_distance(collision_x, d_e, v_ref, trigger_latency, popout_enter_time):
    return collision_x - d_e - v_ref * (trigger_latency + popout_enter_time)


class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')
        defaults = [('odom_topic', '/vesc/odom'), ('trigger_enabled', False), ('run_id', ''),
                    ('d_e', 1.5), ('collision_x', 6.0), ('v_ref', 2.0),
                    ('trigger_latency', 0.05), ('popout_enter_time', 0.2)]
        for k, v in defaults:
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.enabled = bool(gp('trigger_enabled'))
        self.run_id = str(gp('run_id'))
        self.distance = trigger_distance(float(gp('collision_x')), float(gp('d_e')),
                                         float(gp('v_ref')), float(gp('trigger_latency')),
                                         float(gp('popout_enter_time')))
        self.auto = False
        self.travel = 0.0
        self.last_xy = None
        self.fired = False
        self.fired_info = None
        self.pub = self.create_publisher(Header, '/trigger', 10)
        self.info_pub = self.create_publisher(String, '/trigger/info', 10)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 50)
        self.create_subscription(Bool, '/autonomous_mode', self.on_auto, 10)
        self.create_timer(1.0, self.info)
        if self.enabled and self.distance <= 0.0:
            self.get_logger().error(f'trigger_distance={self.distance:.3f} <= 0: 시나리오 값 확인')
        self.get_logger().info(f'trigger enabled={self.enabled} distance={self.distance:.3f} m')

    def on_auto(self, msg):
        if msg.data and not self.auto:
            self.travel, self.last_xy = 0.0, None
        self.auto = bool(msg.data)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.auto and self.last_xy is not None:
            self.travel += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)
        if self.enabled and self.auto and not self.fired and self.travel >= self.distance:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.run_id
            self.pub.publish(header)
            self.fired = True
            self.fired_info = {'stamp': header.stamp.sec + header.stamp.nanosec * 1e-9,
                               'odom_stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                               'travel_m': round(self.travel, 4)}
            self.get_logger().info(f'/trigger 발행: {self.fired_info}')

    def info(self):
        publish_string(self.info_pub, json.dumps({
            'enabled': self.enabled, 'trigger_distance': round(self.distance, 4),
            'travel_m': round(self.travel, 4), 'fired': self.fired, 'fired_info': self.fired_info}))


def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
