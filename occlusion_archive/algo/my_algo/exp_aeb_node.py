"""exp 모드 AEB 노드. drive 모드와 같은 AEBSafetyModel 클래스, 같은 임계값을 쓴다.

역할: 전방 진행 띠 안 최근접 장애물 거리로 긴급 정지 여부를 판단해 발행한다.
    판단 근거는 drive 모드와 같다 (에고 속도 기반 TTC·제동거리, 장애물 상대속도는 쓰지 않음).
    drive 모드와 다른 점은 기준점뿐이다: 라이다→앞 끝 거리와 차폭을 vehicle_geometry.yaml에서 읽는다.
파이프라인 위치: [pointcloud_to_laserscan /scan] → (exp_aeb_node) → [exp_drive_mux] (최우선 override)
구독: scan_topic (sensor_msgs/LaserScan, 10 Hz, livox_frame), odom_topic (nav_msgs/Odometry)
발행 (50 Hz): /aeb/active (std_msgs/Bool), /aeb/status (std_msgs/String, 판단 근거 문자열)
      /exp/aeb_event (std_msgs/String JSON, 발동 순간마다): 시각, 위치(AUTO 후 주행 거리, odom 좌표),
          속도, 판단 근거, 원인 점(가장 가까운 점: 앞 끝 기준 x, y, range, angle) → <run_dir>/aeb_events.jsonl
          S2–S4에서 발동하면 'AEB 오작동'으로 집계된다 (run_meta.py finish, analysis/summarize.py)
파라미터:
    aeb_ttc_threshold, aeb_path_margin, aeb_bumper_emergency_distance,
    aeb_bumper_overlap_tolerance, aeb_comfortable_decel, aeb_stopping_margin
        [EXP-FIXED] config/common.yaml. 값은 drive 모드 launch(lidar_drive.launch.py)와 같다.
    vehicle_geometry_file: lidar_to_front_edge, body_width/2 [EXP-FIXED]
실험 관련: 모든 방법 동일. 방법별 AEB 파라미터 금지.
"""

import json
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from my_algo.aeb import AEBSafetyModel
from my_algo.aeb_event import closest_in_corridor
from my_algo.geometry import load_geometry
from my_algo.ros_util import default_geometry_path, publish_string
from my_algo.vehicle_utils import ReactiveDriveCommand


class ExpAebNode(Node):
    def __init__(self):
        super().__init__('exp_aeb_node')
        defaults = [('scan_topic', '/scan'), ('odom_topic', '/vesc/odom'),
                    ('vehicle_geometry_file', ''), ('rate_hz', 50.0), ('run_dir', ''),
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
        self.run_dir = gp('run_dir')
        self.was_active = False
        self.auto = False
        self.travel, self.xy = 0.0, None
        self.create_subscription(Bool, '/autonomous_mode', self.on_auto, 10)
        self.event_pub = self.create_publisher(String, '/exp/aeb_event', 10)
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

    def on_auto(self, msg):
        if msg.data and not self.auto:
            self.travel, self.xy = 0.0, None
        self.auto = bool(msg.data)

    def on_odom(self, msg):
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.speed = v if math.isfinite(v) else 0.0
        p = msg.pose.pose.position
        if self.auto and self.xy is not None:
            self.travel += math.hypot(p.x - self.xy[0], p.y - self.xy[1])
        self.xy = (p.x, p.y)

    def record_event(self):
        """발동 순간(꺼짐 → 켜짐) 기록. AEB 판단과 별개인 기록 전용 계산."""
        m, s = self.model, self.scan
        cause = None if s is None else closest_in_corridor(
            s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max,
            m.lidar_to_bumper_dist, m.vehicle_half_width, m.path_margin, m.bumper_overlap_tolerance)
        event = {'ros_time': self.get_clock().now().nanoseconds * 1e-9, 'wall_time': time.time(),
                 'auto': self.auto, 'travel_m': round(self.travel, 4),
                 'odom_xy': None if self.xy is None else [round(v, 4) for v in self.xy],
                 'speed': round(self.speed, 4), 'reason': m.last_reason, 'cause_point': cause}
        publish_string(self.event_pub, json.dumps(event))
        self.get_logger().warn(f'AEB 발동: {event}')
        if self.run_dir:
            try:
                os.makedirs(self.run_dir, exist_ok=True)
                with open(os.path.join(self.run_dir, 'aeb_events.jsonl'), 'a', encoding='utf-8') as f:
                    f.write(json.dumps(event) + '\n')
            except OSError as exc:
                self.get_logger().error(f'aeb_events 기록 실패: {exc}')

    def tick(self):
        active = self.model.check_emergency_braking(self.scan, self.speed, self.dummy_command)
        if active and not self.was_active:
            self.record_event()
        self.was_active = active
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
