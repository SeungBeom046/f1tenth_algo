"""phantom 배치 노드 (exp 모드). method 파라미터로 B0/B1/B2/P 전략 하나를 고른다.

역할: 공통 격자에서 방법별 규칙으로 phantom을 두고 최근접 거리 d와 미관측 비율 rho를 낸다.
파이프라인 위치: [occupancy_grid_node] → (phantom_node) → [speed_layer_node]
구독: /exp/grid (nav_msgs/OccupancyGrid, 10 Hz, odom), TF odom<-base_link (격자 stamp)
발행:
    /exp/phantom_d      std_msgs/Float64 (10 Hz) 차체 앞 끝 기준 최근접 phantom 거리 [m], 없으면 inf
    /exp/rho            std_msgs/Float64 (10 Hz) B1 ROI 미관측 비율 (모든 방법에서 기록)
    /exp/phantoms       visualization_msgs/MarkerArray (10 Hz, odom)
    /exp/phantom_debug  std_msgs/String JSON (P의 주차열·틈 목록, phantom 수)
파라미터: method ([EXP-VAR], config/method_*.yaml), ROI·w_min 등은 config/common.yaml,
    차체 앞 끝은 config/vehicle_geometry.yaml (rear_axle_to_front)
실험 관련: 회차 중 method 변경 금지 (노드 재시작). 전략 외 처리는 방법과 무관하게 같다.
"""

import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from my_algo.geometry import load_geometry
from my_algo.perception import decode_grid
from my_algo.phantom import (
    METHODS, PhantomHold, PhantomParams, b1_roi_half_width_auto, make_strategy, nearest_d, to_odom,
)
from my_algo.ros_util import default_geometry_path, lookup_pose2d, publish_float, publish_string


class PhantomNode(Node):
    def __init__(self):
        super().__init__('phantom_node')
        defaults = [('method', 'B0'), ('vehicle_geometry_file', ''),
                    ('odom_frame', 'odom'), ('base_frame', 'base_link'),
                    ('roi_x_max', 6.0), ('roi_half_y', 1.5),
                    ('b1_roi_length', 3.0), ('b1_roi_half_width', -1.0),
                    ('lane_width', 0.7), ('b1_object_speed', 0.8), ('t_r', 0.15), ('v_ref', 2.0), ('a_b', 3.0),
                    ('w_min', 0.35), ('w_max', 0.50), ('row_y_min', 0.30), ('row_y_max', 1.20),
                    ('row_link_dist', 0.10), ('row_face_tol', 0.10), ('row_end_margin', 0.5),
                    ('p_lateral_filter', 0.0), ('phantom_hold', True), ('phantom_hold_margin', 0.10)]
        for k, v in defaults:
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        method = gp('method')
        if method not in METHODS:
            raise ValueError(f'method must be one of {METHODS}')
        geom = load_geometry(gp('vehicle_geometry_file') or default_geometry_path())
        b1_half = float(gp('b1_roi_half_width'))
        if b1_half <= 0.0:      # 자동: 차선 반폭 + 물체 속도 (t_r + v_ref / a_b) (config/common.yaml 근거)
            b1_half = b1_roi_half_width_auto(gp('lane_width'), gp('b1_object_speed'), gp('t_r'), gp('v_ref'),
                                             gp('a_b'))
        self.params = PhantomParams(
            front=geom.rear_axle_to_front, b1_roi_half_width=b1_half,
            **{k: float(gp(k)) for k in ('roi_x_max', 'roi_half_y', 'b1_roi_length',
                                         'w_min', 'w_max', 'row_y_min', 'row_y_max',
                                         'row_link_dist', 'row_face_tol', 'row_end_margin',
                                         'p_lateral_filter')})
        self.hold_enabled = bool(gp('phantom_hold'))
        self.hold_margin = float(gp('phantom_hold_margin'))
        self.hold = None        # 첫 격자에서 칸 크기로 만든다
        self.strategy = make_strategy(method, self.params)
        self.odom_frame, self.base_frame = gp('odom_frame'), gp('base_frame')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.d_pub = self.create_publisher(Float64, '/exp/phantom_d', 10)
        self.rho_pub = self.create_publisher(Float64, '/exp/rho', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/exp/phantoms', 5)
        self.debug_pub = self.create_publisher(String, '/exp/phantom_debug', 5)
        self.create_subscription(OccupancyGrid, '/exp/grid', self.callback, 5)
        self.get_logger().info(f'phantom 전략 {method}, 앞 끝 {geom.rear_axle_to_front} m, '
                               f'B1 ROI 반폭 {b1_half:.4f} m, w {self.params.w_min}–{self.params.w_max} m, '
                               f'유지 {self.hold_enabled} (여유 {self.hold_margin} m)')

    def callback(self, msg):
        pose = lookup_pose2d(self.buffer, self.odom_frame, self.base_frame,
                             Time.from_msg(msg.header.stamp))
        if pose is None:
            self.get_logger().warn('TF odom<-base_link 없음', throttle_duration_sec=2.0)
            return
        info = msg.info
        grid, spec = decode_grid(msg.data, info.width, info.height, info.origin.position.x,
                                 info.origin.position.y, info.resolution)
        res = self.strategy.compute(grid, spec, pose)
        if self.hold_enabled:
            if self.hold is None:
                self.hold = PhantomHold(self.params.front, self.hold_margin, spec.resolution)
            res.phantoms_base = self.hold.update(res.phantoms_base, pose)
            res.d = nearest_d(res.phantoms_base, self.params.front)
        publish_float(self.d_pub, res.d)
        publish_float(self.rho_pub, res.rho)
        self.publish_markers(msg.header.stamp, to_odom(res.phantoms_base, pose)
                             if len(res.phantoms_base) else np.empty((0, 2)))
        publish_string(self.debug_pub, json.dumps({
            'method': res.method, 'n_phantoms': int(len(res.phantoms_base)),
            'd': None if math.isinf(res.d) else round(res.d, 4), 'rho': round(res.rho, 4),
            'rows': [[s, round(a, 3), round(b, 3), round(y, 3)] for s, a, b, y in res.rows],
            'gaps': [[s, round(x, 3), round(w, 3)] for s, x, w in res.gaps]}))

    def publish_markers(self, stamp, xy_odom):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (x, y) in enumerate(xy_odom):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.odom_frame
            m.ns = 'phantom'
            m.id = i
            m.type = Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.10
            m.scale.z = 0.30
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.0, 0.8
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PhantomNode()
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
