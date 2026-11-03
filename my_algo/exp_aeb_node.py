"""exp 모드 AEB 노드. drive 모드와 같은 AEBSafetyModel 클래스, 같은 임계값을 쓴다.

역할: 전방 진행 띠 안 최근접 장애물 거리로 긴급 정지 여부를 판단해 발행한다.
    판단 근거는 drive 모드와 같다 (에고 속도 기반 TTC·제동거리, 장애물 상대속도는 쓰지 않음).
    drive 모드와 다른 점은 기준점뿐이다: 라이다→앞 끝 거리와 차폭을 vehicle_geometry.yaml에서 읽는다.
파이프라인 위치: [pointcloud_to_laserscan /scan] → (exp_aeb_node) → [exp_drive_mux] (최우선 override)
구독: scan_topic (sensor_msgs/LaserScan, 10 Hz, livox_frame), odom_topic (nav_msgs/Odometry)
발행 (50 Hz): /aeb/active (std_msgs/Bool), /aeb/status (std_msgs/String, 판단 근거 문자열)
파라미터:
    aeb_ttc_threshold, aeb_path_margin, aeb_bumper_emergency_distance,
    aeb_bumper_overlap_tolerance, aeb_comfortable_decel, aeb_stopping_margin
        [EXP-FIXED] config/common.yaml. 값은 drive 모드 launch(lidar_drive.launch.py)와 같다.
    vehicle_geometry_file: lidar_to_front_edge, body_width/2 [EXP-FIXED]
실험 관련: 모든 방법 동일. 방법별 AEB 파라미터 금지.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from my_algo.aeb import AEBSafetyModel
from my_algo.geometry import load_geometry
from my_algo.ros_util import default_geometry_path, publish_string
from my_algo.vehicle_utils import ReactiveDriveCommand


class ExpAebNode(Node):
    def __init__(self):
        super().__init__('exp_aeb_node')
        defaults = [('scan_topic', '/scan'), ('odom_topic', '/vesc/odom'),
                    ('vehicle_geometry_file', ''), ('rate_hz', 50.0),
                    ('aeb_ttc_threshold', 0.55), ('aeb_path_margin', 0.15),
                    ('aeb_bumper_emergency_distance', 0.25), ('aeb_bumper_overlap_tolerance', 0.10),
                    ('aeb_comfortable_decel', 2.8), ('aeb_stopping_margin', 0.20)]
        for k, v in defaults:
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        geom = load_geometry(gp('vehicle_geometry_file') or default_geometry_path())
        self.model = AEBSafetyModel(
            ttc_threshold=float(gp('aeb_ttc_threshold')),
            stopping_margin=float(gp('aeb_stopping_margin')),
            bumper_emergency_distance=float(gp('aeb_bumper_emergency_distance')),
            bumper_overlap_tolerance=float(gp('aeb_bumper_overlap_tolerance')),
            lidar_to_bumper_dist=geom.lidar_to_front_edge,
            vehicle_half_width=geom.half_width,
            path_margin=float(gp('aeb_path_margin')),
            comfortable_decel=float(gp('aeb_comfortable_decel')),
        )
        # AEBSafetyModel은 planner 명령의 유효성도 검사한다. exp 모드에서는 속도 계층이 따로 있으므로
        # 항상 유효한 빈 명령을 넘긴다 (scan 누락 시 정지 동작은 그대로 유지된다).
        self.dummy_command = ReactiveDriveCommand(mode='EXP')
        self.scan = None
        self.speed = 0.0
        self.create_subscription(LaserScan, gp('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 10)
        self.active_pub = self.create_publisher(Bool, '/aeb/active', 10)
        self.status_pub = self.create_publisher(String, '/aeb/status', 10)
        self.create_timer(1.0 / float(gp('rate_hz')), self.tick)
        self.get_logger().info(
            f'exp AEB: lidar_to_front={geom.lidar_to_front_edge} half_width={geom.half_width} '
            f'ttc={gp("aeb_ttc_threshold")}')

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.speed = v if math.isfinite(v) else 0.0

    def tick(self):
        active = self.model.check_emergency_braking(self.scan, self.speed, self.dummy_command)
        msg = Bool()
        msg.data = bool(active)
        self.active_pub.publish(msg)
        publish_string(self.status_pub, self.model.last_reason)


def main(args=None):
    rclpy.init(args=args)
    node = ExpAebNode()
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
