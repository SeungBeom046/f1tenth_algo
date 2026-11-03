"""점군 deskew 노드 (exp 모드, launch 인자 deskew:=true일 때만 실행).

역할: 프레임 내 이동으로 생긴 점군 왜곡을 odom TF 보간으로 보정한다.
파이프라인 위치: [Livox 드라이버] → (deskew_node) → [occupancy_grid_node]
구독: cloud_topic (sensor_msgs/PointCloud2, 10 Hz, livox_frame), TF odom<-livox_frame
발행: output_topic (sensor_msgs/PointCloud2, 10 Hz, livox_frame, 필드 그대로, x/y/z만 보정)
파라미터 (config/common.yaml):
    time_field   점별 시각 필드 이름. livox_ros_driver2 포맷에 따라 다름 → check_vehicle.sh로 확정
    time_scale   필드 값 → 초 변환 계수 (예: ns면 1e-9)
    time_mode    'absolute' (필드가 절대 시각) | 'offset' (header.stamp 기준 오프셋)
실험 관련: deskew on/off는 모든 방법 동일 [EXP-FIXED]. 점별 시각 필드가 없으면 노드가 경고 후 원본을 그대로 넘긴다.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

from my_algo.deskew import deskew_points
from my_algo.ros_util import lookup_pose2d


class DeskewNode(Node):
    def __init__(self):
        super().__init__('deskew_node')
        for k, v in [('cloud_topic', '/livox/lidar'), ('output_topic', '/exp/cloud_deskewed'),
                     ('odom_frame', 'odom'), ('time_field', 'timestamp'),
                     ('time_scale', 1e-9), ('time_mode', 'absolute')]:
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.odom_frame = gp('odom_frame')
        self.time_field = gp('time_field')
        self.time_scale = float(gp('time_scale'))
        self.time_mode = gp('time_mode')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PointCloud2, gp('output_topic'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, gp('cloud_topic'), self.callback,
                                 qos_profile_sensor_data)
        self.warned = False

    def callback(self, msg):
        names = [f.name for f in msg.fields]
        if self.time_field not in names:
            if not self.warned:
                self.get_logger().warn(
                    f'점별 시각 필드 {self.time_field!r} 없음 (필드: {names}). deskew 없이 통과')
                self.warned = True
            self.pub.publish(msg)
            return
        pts = point_cloud2.read_points(msg, skip_nans=True)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = pts[self.time_field].astype(np.float64) * self.time_scale
        if self.time_mode == 'offset':
            t = stamp + t
        if len(t) == 0:
            return
        t0, t1 = float(t.min()), float(t.max())
        pose0 = lookup_pose2d(self.buffer, self.odom_frame, msg.header.frame_id,
                              Time(nanoseconds=int(t0 * 1e9)))
        pose1 = lookup_pose2d(self.buffer, self.odom_frame, msg.header.frame_id,
                              Time(nanoseconds=int(t1 * 1e9)))
        if pose0 is None or pose1 is None:
            self.get_logger().warn('deskew용 TF 없음: 원본 통과', throttle_duration_sec=2.0)
            self.pub.publish(msg)
            return
        xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        out = deskew_points(xyz, t, t0, pose0, t1, pose1, t_ref=stamp)
        pts = pts.copy()
        pts['x'], pts['y'], pts['z'] = out[:, 0], out[:, 1], out[:, 2]
        cloud = point_cloud2.create_cloud(msg.header, msg.fields, pts)
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = DeskewNode()
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
