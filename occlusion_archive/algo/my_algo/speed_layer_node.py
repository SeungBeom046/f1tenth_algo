"""속도 계층 노드 (exp 모드). 네 방법이 같은 v_cap 식과 같은 최소 속도 정책을 쓴다.

역할: phantom 거리 d(B2·P) 또는 rho(B1)로 목표 속도를 정하고 최소 속도 정책을 적용한다.
파이프라인 위치: [phantom_node] → (speed_layer_node) → [exp_drive_mux] (AEB override는 mux에서)
구독: /exp/phantom_d, /exp/rho (std_msgs/Float64, 10 Hz)
발행 (50 Hz):
    /exp/v_cap           Float64  방법별 속도 상한 (B0: inf, B1: v_ref(1-k rho), B2·P: v_cap(d))
    /exp/v_raw           Float64  min(v_ref, v_cap) — 명령 기준 손실 계산용 (정책 전)
    /exp/v_cmd           Float64  최소 속도 정책 후 목표 속도
    /exp/min_speed_flag  String   '' / 'clamped_to_vmin' / 'stopped_below_vmin' / 'stale_perception'
파라미터 (config/common.yaml):
    v_ref, t_r, a_b, s_m        [EXP-FIXED] (t_r, a_b는 [EXP0-MEASURE])
    v_imp, b1_k                 [EXP-TUNE] (v_imp 하한 = V_MIN)
    min_speed_policy, restart_margin   [EXP-FIXED]
    perception_timeout          인지 입력이 이보다 오래되면 0 명령 (안전)
    method                      [EXP-VAR] config/method_*.yaml
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from my_algo.phantom import METHODS
from my_algo.ros_util import publish_float, publish_string
from my_algo.speed_policy import (
    V_MIN, MinSpeedPolicy, b1_speed, check_v_imp, command_speed, v_cap,
)


class SpeedLayerNode(Node):
    def __init__(self):
        super().__init__('speed_layer_node')
        defaults = [('method', 'B0'), ('v_ref', 2.0), ('t_r', 0.15), ('a_b', 3.0), ('s_m', 0.2),
                    ('v_imp', V_MIN), ('b1_k', 1.0), ('min_speed_policy', 'hold'),
                    ('restart_margin', 0.1), ('perception_timeout', 0.3), ('rate_hz', 50.0)]
        for k, v in defaults:
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.method = gp('method')
        if self.method not in METHODS:
            raise ValueError(f'method must be one of {METHODS}')
        self.v_ref = float(gp('v_ref'))
        self.cap_params = {k: float(gp(k)) for k in ('t_r', 'a_b', 's_m', 'v_imp')}
        self.k = float(gp('b1_k'))
        self.timeout = float(gp('perception_timeout'))
        self.policy = MinSpeedPolicy(gp('min_speed_policy'), V_MIN, float(gp('restart_margin')))
        warning = check_v_imp(self.cap_params['v_imp'])
        if warning:
            self.get_logger().warn(warning)

        self.d = math.inf
        self.rho = 0.0
        self.last_input = None
        self.create_subscription(Float64, '/exp/phantom_d', self.on_d, 10)
        self.create_subscription(Float64, '/exp/rho', self.on_rho, 10)
        self.cap_pub = self.create_publisher(Float64, '/exp/v_cap', 10)
        self.raw_pub = self.create_publisher(Float64, '/exp/v_raw', 10)
        self.cmd_pub = self.create_publisher(Float64, '/exp/v_cmd', 10)
        self.flag_pub = self.create_publisher(String, '/exp/min_speed_flag', 10)
        self.create_timer(1.0 / float(gp('rate_hz')), self.tick)
        self.get_logger().info(
            f'method={self.method} v_ref={self.v_ref} {self.cap_params} k={self.k} '
            f'policy={self.policy.mode} V_MIN={V_MIN:.3f}')

    def on_d(self, msg):
        self.d = msg.data
        self.last_input = self.get_clock().now()

    def on_rho(self, msg):
        self.rho = msg.data

    def limit(self):
        if self.method == 'B0':
            return math.inf
        if self.method == 'B1':
            return b1_speed(self.rho, self.v_ref, self.k)
        return v_cap(self.d, **self.cap_params)

    def tick(self):
        stale = (self.last_input is None or
                 (self.get_clock().now() - self.last_input).nanoseconds * 1e-9 > self.timeout)
        limit = self.limit()
        if stale:
            raw, cmd, flag = min(self.v_ref, limit), 0.0, 'stale_perception'
        else:
            raw, cmd, flag = command_speed(self.v_ref, limit, self.policy)
        publish_float(self.cap_pub, limit)
        publish_float(self.raw_pub, raw)
        publish_float(self.cmd_pub, cmd)
        publish_string(self.flag_pub, flag)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedLayerNode()
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
