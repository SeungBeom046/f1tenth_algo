"""실차 F1TENTH용 자율주행 통합 노드.

입력 토픽:
    /autonomous_mode (std_msgs/Bool)
    /joy_active (std_msgs/Bool)
    /reactive/lidar_command (std_msgs/String JSON ReactiveDriveCommand)
    /reactive/camera_command (std_msgs/String JSON ReactiveDriveCommand)
    /scan (sensor_msgs/LaserScan)
    /vesc/odom (nav_msgs/Odometry)

출력 토픽:
    /commands/motor/speed (std_msgs/Float64)
    /commands/servo/position (std_msgs/Float64)

흐름:
    LiDAR 또는 camera planner 출력 선택 -> AEB 안전 검사 -> RPM/조향 제한
    -> 최종 VESC 명령 발행.

주요 튜닝 값:
    drive_source:
        현재 실차 주행은 "lidar", 추후 카메라 주행은 "camera"로 선택한다.
    command_timeout_sec:
        planner command 최대 허용 나이. planner가 느리면 키우고,
        planner 장애 시 더 빨리 fail-safe 정지하려면 줄인다.
    aeb_ttc_threshold:
        키우면 AEB가 더 일찍 정지하고, AEB가 너무 민감하면 줄인다.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String

from my_algo.aeb import AEBSafetyModel
from my_algo.vehicle_utils import (
    ReactiveDriveCommand,
    command_from_json,
    print_event_line,
    print_status_line,
    sanitize_rpm,
    steering_to_servo,
)


class AutonomousDriveNode(Node):
    """reactive drive, AEB, hardware command publish를 통합한다."""

    def __init__(self):
        super().__init__('autonomous_drive')

        # 주행 알고리즘 선택
        # 라이다: 현재 실차 기본값, 카메라: 추후 카메라 주행 알고리즘 연결.
        self.declare_parameter('drive_source', 'lidar')

        # 계획 명령 만료 시간 [s]
        # 증가: 느린 계획기 허용, 감소: 계획기 멈춤 시 더 빨리 정지.
        self.declare_parameter('command_timeout_sec', 1.00)

        # AEB TTC 임계값 [s]
        # 증가: 더 일찍 정지, 감소: 민감도 완화.
        self.declare_parameter('aeb_ttc_threshold', 0.55)

        # AEB 검사에 사용하는 차량 반폭 [m]
        # 실제 차량 폭 0.30 m 기준 반폭 0.15 m. 폭을 키우면 AEB가 더 보수적이다.
        self.declare_parameter('aeb_vehicle_half_width', 0.15)

        # 앞범퍼 기준 AEB 즉시 정지 거리 [m]
        # 보이는 장애물은 planner가 피하고, AEB는 이 거리 안 임박 충돌만 막는다.
        self.declare_parameter('aeb_bumper_emergency_distance', 0.25)

        # 라이다-범퍼 보정 후 살짝 음수가 되는 근접점 허용치 [m]
        self.declare_parameter('aeb_bumper_overlap_tolerance', 0.10)

        # AEB 전방 검사 띠의 좌우 추가 여유거리 [m]
        # 검사 반폭 = 차량 반폭 + 이 값. 키우면 옆 장애물에도 더 민감해진다.
        self.declare_parameter('aeb_path_margin', 0.15)

        # 라이다-앞범퍼 수평거리 [m]
        # 증가: 앞범퍼 기준 충돌 거리를 더 보수적으로 계산.
        self.declare_parameter('lidar_to_bumper_dist', 0.30)

        # 자율주행 최대 조향 [rad]
        # 증가: 더 급회전 가능, 감소: 조향 안정성 증가.
        self.declare_parameter('max_steering', 0.78)

        self.drive_source = self.get_parameter('drive_source').value
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value)
        self.max_steering = float(self.get_parameter('max_steering').value)

        self.aeb = AEBSafetyModel(
            ttc_threshold=float(self.get_parameter('aeb_ttc_threshold').value),
            lidar_to_bumper_dist=float(
                self.get_parameter('lidar_to_bumper_dist').value),
            vehicle_half_width=float(
                self.get_parameter('aeb_vehicle_half_width').value),
            path_margin=float(self.get_parameter('aeb_path_margin').value),
            bumper_emergency_distance=float(
                self.get_parameter('aeb_bumper_emergency_distance').value),
            bumper_overlap_tolerance=float(
                self.get_parameter('aeb_bumper_overlap_tolerance').value),
        )

        # 초기 상태 안정화
        self.auto_mode = False
        self.joy_active = False
        self.current_speed = 0.0
        self.latest_scan = None
        self.commands = {
            'lidar': {'command': None, 'time': None},
            'camera': {'command': None, 'time': None},
        }
        self.last_status = self.get_clock().now()

        # Topic 통신 설정 (안정화/프로토콜)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1, # 최신 데이터 1개만 유지
        )

        self.auto_sub = self.create_subscription(
            Bool, '/autonomous_mode', self.auto_mode_callback, 10) # 10은 depth(큐 크기), 최근 10개까지 버퍼에 저장하겠단 의미.
        self.joy_sub = self.create_subscription(
            Bool, '/joy_active', self.joy_active_callback, 10)
        self.lidar_command_sub = self.create_subscription(
            String, '/reactive/lidar_command', self.lidar_command_callback, 10)
        self.camera_command_sub = self.create_subscription(
            String, '/reactive/camera_command', self.camera_command_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/vesc/odom', self.odom_callback, 10)

        self.motor_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(
            Float64, '/commands/servo/position', 10)
        self.aeb_active_pub = self.create_publisher(
            Bool, '/aeb/active', 10)
        self.aeb_status_pub = self.create_publisher(
            String, '/aeb/status', 10)
        self.control_timer = self.create_timer(0.02, self.control_timer_callback) # 제어 루프 0.02초마다 갱신 (50Hz)

        self.get_logger().info(
            f'Autonomous drive ready | mode={self.drive_source}'
        )

    def auto_mode_callback(self, msg):
        """자율주행 명령 출력을 시작하거나 정지한다."""
        self.auto_mode = msg.data
        if not self.auto_mode:
            self.publish_drive_command(0.0, 0.0, center_steering=True)
            self.publish_aeb_state(False, 'auto_off')
            print_event_line('[Drive] AUTO off: stop')

    def joy_active_callback(self, msg):
        """수동 입력 중에는 조이스틱 명령이 VESC를 제어하게 둔다."""
        self.joy_active = msg.data

    def lidar_command_callback(self, msg):
        """최신 LiDAR planner 출력을 저장한다."""
        self.store_reactive_command('lidar', msg.data)

    def camera_command_callback(self, msg):
        """최신 camera planner 출력을 저장한다."""
        self.store_reactive_command('camera', msg.data)

    def store_reactive_command(self, source, payload):
        """planner command를 파싱하고 수신 시간을 기록한다."""
        command = command_from_json(payload)
        self.commands[source] = {
            'command': command,
            'time': self.get_clock().now(),
        }

    def scan_callback(self, msg):
        """AEB가 사용할 최신 LaserScan을 저장한다."""
        self.latest_scan = msg

    def odom_callback(self, msg):
        """AEB가 사용할 현재 속도 추정값을 저장한다."""
        # 평면 속도 크기 공식: speed = sqrt(vx^2 + vy^2).
        # odom의 x/y 선속도 성분을 합쳐 AEB TTC 계산용 현재 속도로 사용한다.
        self.current_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )

    def control_timer_callback(self):
        """최종 drive command를 50 Hz로 발행한다."""
        if self.joy_active or not self.auto_mode:
            return

        command = self.select_reactive_command()
        if command is None:
            self.publish_drive_command(0.0, 0.0, center_steering=True)
            self.publish_aeb_state(False, 'missing_command')
            self.print_drive_status('FAILSAFE', 0.0, 0.0, 'missing_command')
            return

        emergency = self.aeb.check_emergency_braking(
            self.latest_scan,
            self.current_speed,
            command,
        )
        if emergency:
            self.publish_drive_command(0.0, command.steering_rad)
            self.publish_aeb_state(True, self.aeb.last_reason)
            self.print_drive_status('AEB', 0.0, command.steering_rad, self.aeb.last_reason)
            return

        rpm = sanitize_rpm(command.rpm)
        steering = max(-self.max_steering, min(command.steering_rad, self.max_steering))
        self.publish_drive_command(rpm, steering)
        self.publish_aeb_state(False, self.aeb.last_reason)
        self.print_drive_status(command.mode, rpm, steering, command.reason)

    def select_reactive_command(self):
        """설정된 drive source에 맞는 최신 reactive command를 반환한다."""
        source = 'camera' if self.drive_source == 'camera' else 'lidar'
        entry = self.commands.get(source)
        if not entry or entry['command'] is None or entry['time'] is None:
            return None
        # 계획 명령 나이 = 현재 시간 - 마지막 명령 수신 시간.
        # ROS 시간 차이는 ns 단위라 1e9로 나눠 초 단위로 변환한다.
        age = (self.get_clock().now() - entry['time']).nanoseconds / 1e9
        if age > self.command_timeout_sec:
            return None
        if not entry['command'].is_valid():
            return None
        return entry['command']

    def publish_drive_command(self, rpm, steering_rad, center_steering=False):
        """제한을 적용한 motor RPM과 servo position을 VESC 토픽으로 발행한다."""
        motor_msg = Float64()
        motor_msg.data = sanitize_rpm(rpm)
        self.motor_pub.publish(motor_msg)

        servo_msg = Float64()
        # center_steering이면 조향을 강제로 0 rad로 보내고,
        # 아니면 planner/AEB가 넘긴 조향각을 VESC 서보 위치로 변환한다.
        servo_msg.data = steering_to_servo(0.0 if center_steering else steering_rad)
        self.servo_pub.publish(servo_msg)

    def publish_aeb_state(self, active, status):
        """카메라 overlay와 로그가 볼 수 있도록 AEB 상태를 발행한다."""
        active_msg = Bool()
        active_msg.data = bool(active)
        self.aeb_active_pub.publish(active_msg)

        status_msg = String()
        status_msg.data = str(status)
        self.aeb_status_pub.publish(status_msg)

    def print_drive_status(self, mode, rpm, steering, reason):
        """최종 명령 상태를 약 2 Hz로 출력한다."""
        elapsed = (self.get_clock().now() - self.last_status).nanoseconds / 1e9
        if elapsed < 0.5:
            return
        print_status_line(
            '[Drive] '
            f'mode={mode:18s} | rpm={rpm:7.0f} | '
            f'steer={steering:5.2f} | {reason}'
        )
        self.last_status = self.get_clock().now()


def main(args=None):
    """자율주행 통합 노드를 실행한다."""
    rclpy.init(args=args)
    node = AutonomousDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_drive_command(0.0, 0.0, center_steering=True)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
