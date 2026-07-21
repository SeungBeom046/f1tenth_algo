"""Joystick controller node for F1TENTH real car."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool

from my_algo.vesc_utils import (
    ERPM_GAIN,
    MIN_DRIVE_ERPM,
    MIN_DRIVE_SPEED_MS,
    apply_min_drive_speed,
    print_event_line,
    print_status_line,
    speed_to_erpm,
)


class JoyControllerNode(Node):
    """
    로지텍 조이스틱 제어 노드
    
    [버튼 매핑 - 로지텍 F710 기준]
    왼쪽 스틱 상하/L3 방향 입력 → 전진/후진 제어
    LT 축(있는 경우) → 눌렀을 때만 속도 리미터
    오른쪽 스틱 좌우 → 조향 제어
    LB 버튼 (버튼 4) → 자율주행 모드 토글
    B 버튼 (버튼 1)  → 긴급 정지
    
    [안전 설계]
    - LB: 자율주행 모드 ON/OFF 토글
    - 수동 조작 중에는 /joy_active=True를 발행해서 AEB와 자율주행이 양보
    - B: 긴급 정지 (자율주행 + 조이스틱 둘 다 정지)
    """

    def __init__(self):
        super().__init__('joy_controller_node')

        # ============ 튜닝 파라미터 ============
        self.max_speed = 2.0        # 조이스틱 최대 속도 (m/s)
        self.ERPM_GAIN = ERPM_GAIN
        self.speed_deadband = 0.05   # 스틱 중립 노이즈는 정지로 처리
        self.steer_deadband = 0.03
        self.max_steer = 0.78
        self.SERVO_CENTER = 0.5
        self.SERVO_GAIN = 0.60
        self.SERVO_MIN = 0.03
        self.SERVO_MAX = 0.97
        # ======================================

        # 상태 변수
        self.autonomous_mode = False  # 자율주행 모드 여부
        self.emergency_stop = False   # 긴급 정지 상태
        self.prev_lb = False          # LB 버튼 이전 상태 (토글용)

        # 조이스틱 구독
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        # VESC 제어 발행
        self.speed_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(
            Float64, '/commands/servo/position', 10)

        # 자율주행 모드 상태 발행 (wall_follow_real이 구독)
        self.auto_mode_pub = self.create_publisher(
            Bool, '/autonomous_mode', 10)

        # 조이스틱 제어 중인지 발행 (자율주행 노드가 양보할지 판단)
        self.joy_active_pub = self.create_publisher(
            Bool, '/joy_active', 10)

        self.get_logger().info('Joy Controller Node 시작!')
        self.get_logger().info(
            '조작법:\n'
            '  LB: 자율주행 모드 토글\n'
            '  왼쪽 스틱 상하/L3: 전진/후진\n'
            '  LT 축: 눌렀을 때만 속도 리미터(지원되는 패드에서)\n'
            '  오른쪽 스틱 좌우: 조향\n'
            '  B: 긴급 정지\n'
            f'  최소 구동: {MIN_DRIVE_SPEED_MS:.2f}m/s '
            f'({MIN_DRIVE_ERPM:.0f} ERPM)'
        )

    def get_axis(self, msg, idx, default=0.0):
        return msg.axes[idx] if idx < len(msg.axes) else default

    def get_button(self, msg, idx):
        return bool(msg.buttons[idx]) if idx < len(msg.buttons) else False

    def get_left_trigger_scale(self, msg):
        """
        Logitech-style trigger axis is usually +1 released, -1 fully pressed.
        If released or missing, keep full scale so left-stick drive still works.
        """
        if len(msg.axes) <= 2:
            return 1.0
        raw = self.get_axis(msg, 2, 1.0)
        pressed_scale = max(0.0, min(1.0, (1.0 - raw) * 0.5))
        if pressed_scale < 0.05:
            return 1.0
        return pressed_scale

    def joy_callback(self, msg):
        """
        조이스틱 입력 처리
        
        [로지텍 F710 축/버튼 인덱스]
        axes[0]: 왼쪽 스틱 좌우 (-1=오른쪽, 1=왼쪽)
        axes[1]: 왼쪽 스틱 상하/L3 (-1=아래/후진, 1=위/전진)
        axes[2]: LT 트리거(+1=해제, -1=완전 입력, 패드 설정에 따라 다름)
        axes[3]: 오른쪽 스틱 좌우 (-1=오른쪽, 1=왼쪽)
        axes[4]: 오른쪽 스틱 상하 (-1=아래, 1=위)
        buttons[4]: LB (자율주행 토글)
        buttons[1]: B (긴급 정지)
        """

        # 버튼 읽기
        lb = self.get_button(msg, 4)
        emergency = self.get_button(msg, 1)

        # 긴급 정지
        if emergency:
            self.emergency_stop = True
            self.autonomous_mode = False
            self.stop()
            print_event_line('긴급 정지!')
            self._publish_mode(joy_active=True, auto_mode=False)
            return

        # 긴급 정지 해제 (B 안 누른 상태)
        self.emergency_stop = False

        # LB로 자율주행 모드 토글 (엣지 감지)
        if lb and not self.prev_lb:
            self.autonomous_mode = not self.autonomous_mode
            mode_str = '자율주행' if self.autonomous_mode else '수동'
            print_event_line(f'모드 전환: {mode_str}')
        self.prev_lb = lb

        # 자율주행 모드에서는 gap_follow_real이 제어
        if self.autonomous_mode:
            self._publish_mode(joy_active=False, auto_mode=True)
            return

        # 수동 조이스틱 제어
        drive_axis = self.get_axis(msg, 1)    # 왼쪽 스틱/L3 상하
        steer_axis = self.get_axis(msg, 3)    # 오른쪽 스틱 좌우
        trigger_scale = self.get_left_trigger_scale(msg)

        requested_speed_ms = drive_axis * trigger_scale * self.max_speed
        speed_ms = apply_min_drive_speed(
            requested_speed_ms,
            deadband=self.speed_deadband,
        )
        erpm = speed_to_erpm(speed_ms)

        if abs(steer_axis) < self.steer_deadband:
            steer_axis = 0.0
        steering_rad = steer_axis * self.max_steer
        servo_pos = self.SERVO_CENTER - steering_rad * self.SERVO_GAIN
        servo_pos = max(self.SERVO_MIN, min(self.SERVO_MAX, servo_pos))

        active_manual = (
            abs(speed_ms) > 0.0
            or abs(steering_rad) > 0.0
        )

        if active_manual:
            speed_msg = Float64()
            speed_msg.data = erpm
            self.speed_pub.publish(speed_msg)

            servo_msg = Float64()
            servo_msg.data = servo_pos
            self.servo_pub.publish(servo_msg)

            self._publish_mode(joy_active=True, auto_mode=False)

            print_status_line(
                '[JOY] '
                f'speed={speed_ms:5.2f} m/s | '
                f'erpm={erpm:7.0f} | '
                f'steer={steering_rad:6.2f} rad | '
                f'servo={servo_pos:5.3f} | '
                f'lt={trigger_scale:4.2f}'
            )
        else:
            self.stop()
            self._publish_mode(joy_active=False, auto_mode=False)

    def _publish_mode(self, joy_active, auto_mode):
        """모드 상태 발행"""
        joy_msg = Bool()
        joy_msg.data = joy_active
        self.joy_active_pub.publish(joy_msg)

        auto_msg = Bool()
        auto_msg.data = auto_mode
        self.auto_mode_pub.publish(auto_msg)

    def stop(self):
        """정지 명령"""
        speed_msg = Float64()
        speed_msg.data = 0.0
        self.speed_pub.publish(speed_msg)

        servo_msg = Float64()
        servo_msg.data = self.SERVO_CENTER
        self.servo_pub.publish(servo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
