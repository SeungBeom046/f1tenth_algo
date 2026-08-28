"""실차 F1TENTH 조이스틱 제어 노드."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool

from my_algo.vehicle_utils import (
    DRIVE_SPEED_SCALE,
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

        self.declare_parameter('drive_axis_index', 1)
        self.declare_parameter('steer_axis_indices', [3, 0])
        self.declare_parameter('auto_toggle_button_indices', [4, 6])
        self.declare_parameter('emergency_button_indices', [1])
        self.declare_parameter('invert_drive_axis', False)
        self.declare_parameter('invert_steer_axis', False)

        # ============ 튜닝 파라미터 ============
        self.max_speed = 2.0 * DRIVE_SPEED_SCALE
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
        self.toggle_debounce_sec = 0.70
        self.last_toggle_time = None
        self.last_mode_publish_time = self.get_clock().now()
        self.last_published_joy_active = None
        self.last_published_auto_mode = None
        self.mode_heartbeat_sec = 0.10
        self.last_manual_command_time = self.get_clock().now()
        self.manual_command_timeout_sec = 0.30
        self.manual_active = False
        self.last_manual_erpm = 0.0
        self.last_manual_servo = self.SERVO_CENTER
        self.prev_buttons = []

        self.drive_axis_index = int(self.get_parameter('drive_axis_index').value)
        self.steer_axis_indices = [
            int(index) for index in self.get_parameter('steer_axis_indices').value
        ]
        self.auto_toggle_button_indices = [
            int(index) for index in self.get_parameter('auto_toggle_button_indices').value
        ]
        self.emergency_button_indices = [
            int(index) for index in self.get_parameter('emergency_button_indices').value
        ]
        self.invert_drive_axis = bool(self.get_parameter('invert_drive_axis').value)
        self.invert_steer_axis = bool(self.get_parameter('invert_steer_axis').value)

        # 조이스틱 구독
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        # VESC 제어 발행
        self.speed_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(
            Float64, '/commands/servo/position', 10)

        # 자율주행 모드 상태 발행 (autonomous_drive/lidar_reactive_drive가 구독)
        self.auto_mode_pub = self.create_publisher(
            Bool, '/autonomous_mode', 10)

        # 조이스틱 제어 중인지 발행 (자율주행 노드가 양보할지 판단)
        self.joy_active_pub = self.create_publisher(
            Bool, '/joy_active', 10)
        self.mode_timer = self.create_timer(0.05, self.mode_timer_callback)

        self.get_logger().info('Joy Controller Node 시작!')
        self.get_logger().info(
            '조작법:\n'
            f'  AUTO toggle buttons: {self.auto_toggle_button_indices}\n'
            f'  Emergency buttons: {self.emergency_button_indices}\n'
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

    def get_any_button(self, msg, indices):
        return any(self.get_button(msg, index) for index in indices)

    def button_edge(self, msg, indices):
        for index in indices:
            now_pressed = self.get_button(msg, index)
            was_pressed = bool(self.prev_buttons[index]) if index < len(self.prev_buttons) else False
            if now_pressed and not was_pressed:
                return True
        return False

    def log_button_edges(self, msg):
        pressed = [
            index for index, value in enumerate(msg.buttons)
            if value and not (index < len(self.prev_buttons) and self.prev_buttons[index])
        ]
        if pressed:
            self.get_logger().info(f'Joy button pressed indices: {pressed}')

    def get_left_trigger_scale(self, msg):
        """
        Logitech 계열 트리거 축은 보통 +1이 해제, -1이 완전 입력이다.
        해제 상태이거나 축이 없으면 왼쪽 스틱 주행이 가능하도록 최대 scale을 유지한다.
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

        self.log_button_edges(msg)
        auto_toggle_edge = self.button_edge(msg, self.auto_toggle_button_indices)
        emergency = self.get_any_button(msg, self.emergency_button_indices)

        # 긴급 정지
        if emergency:
            self.emergency_stop = True
            self.autonomous_mode = False
            self.manual_active = True
            self.stop()
            print_event_line('긴급 정지!')
            self._publish_mode(joy_active=True, auto_mode=False)
            return

        # 긴급 정지 해제 (B 안 누른 상태)
        self.emergency_stop = False

        # LB로 자율주행 모드 토글. 무선 패드/버튼 바운스를 막기 위해
        # 짧은 시간 안의 재입력은 같은 눌림으로 간주한다.
        now = self.get_clock().now()
        since_last_toggle = (
            self.toggle_debounce_sec
            if self.last_toggle_time is None
            else (now - self.last_toggle_time).nanoseconds / 1e9
        )
        if auto_toggle_edge and since_last_toggle >= self.toggle_debounce_sec:
            self.autonomous_mode = not self.autonomous_mode
            self.last_toggle_time = now
            mode_str = '자율주행' if self.autonomous_mode else '수동'
            print_event_line(f'모드 전환: {mode_str}')
        self.prev_buttons = list(msg.buttons)

        # 자율주행 모드에서는 autonomous_drive가 VESC 명령을 제어
        if self.autonomous_mode:
            self.manual_active = False
            self._publish_mode(joy_active=False, auto_mode=True)
            return

        # 수동 조이스틱 제어
        drive_axis = self.get_axis(msg, self.drive_axis_index)
        if self.invert_drive_axis:
            drive_axis *= -1.0
        steer_axis = self.select_steer_axis(msg)
        if self.invert_steer_axis:
            steer_axis *= -1.0
        trigger_scale = self.get_left_trigger_scale(msg)

        # 수동 속도 공식: 스틱 입력(-1~1) * LT scale(0~1) * 최대속도[m/s].
        requested_speed_ms = drive_axis * trigger_scale * self.max_speed
        speed_ms = apply_min_drive_speed(
            requested_speed_ms,
            deadband=self.speed_deadband,
        )
        # VESC 모터 명령은 m/s가 아니라 ERPM이므로 공통 변환식을 적용한다.
        erpm = speed_to_erpm(speed_ms)

        if abs(steer_axis) < self.steer_deadband:
            steer_axis = 0.0
        # 수동 조향각 공식: 스틱 입력(-1~1) * 최대 조향각[rad].
        steering_rad = steer_axis * self.max_steer
        # 서보 위치 공식: servo = center - steering_rad * gain.
        servo_pos = self.SERVO_CENTER - steering_rad * self.SERVO_GAIN
        servo_pos = max(self.SERVO_MIN, min(self.SERVO_MAX, servo_pos))

        active_manual = (
            abs(speed_ms) > 0.0
            or abs(steering_rad) > 0.0
        )

        if active_manual:
            self.manual_active = True
            self.last_manual_command_time = self.get_clock().now()
            self.publish_manual_command(erpm, servo_pos)

            self._publish_mode(joy_active=True, auto_mode=False)

            print_status_line(
                '[JOY] '
                f'speed={speed_ms:5.2f} m/s | '
                f'erpm={erpm:7.0f} | '
                f'steer={steering_rad:6.2f} rad | '
                f'servo={servo_pos:5.3f}'
            )
        else:
            self.manual_active = False
            self.last_manual_erpm = 0.0
            self.last_manual_servo = self.SERVO_CENTER
            self.stop()
            self._publish_mode(joy_active=False, auto_mode=False)

    def select_steer_axis(self, msg):
        """F710 모드 차이를 흡수하기 위해 오른쪽/왼쪽 stick X 중 실제 입력된 축을 고른다."""
        candidates = [
            self.get_axis(msg, index)
            for index in self.steer_axis_indices
        ]
        active = [value for value in candidates if abs(value) >= self.steer_deadband]
        if active:
            return max(active, key=abs)
        return candidates[0] if candidates else 0.0

    def publish_manual_command(self, erpm, servo_pos):
        """수동 VESC 명령을 발행하고 마지막 값을 저장한다."""
        self.last_manual_erpm = erpm
        self.last_manual_servo = servo_pos

        speed_msg = Float64()
        speed_msg.data = erpm
        self.speed_pub.publish(speed_msg)

        servo_msg = Float64()
        servo_msg.data = servo_pos
        self.servo_pub.publish(servo_msg)

    def mode_timer_callback(self):
        """joy 메시지가 잠깐 끊겨도 mode/joy_active 상태를 계속 발행한다."""
        if self.autonomous_mode:
            self._publish_mode(joy_active=False, auto_mode=True)
            return

        elapsed = (
            self.get_clock().now() - self.last_manual_command_time
        ).nanoseconds / 1e9
        active = self.manual_active and elapsed <= self.manual_command_timeout_sec
        self._publish_mode(joy_active=active, auto_mode=False)
        if active:
            self.publish_manual_command(self.last_manual_erpm, self.last_manual_servo)

    def _publish_mode(self, joy_active, auto_mode):
        """모드 상태 발행"""
        now = self.get_clock().now()
        elapsed = (now - self.last_mode_publish_time).nanoseconds / 1e9
        unchanged = (
            self.last_published_joy_active == joy_active
            and self.last_published_auto_mode == auto_mode
        )
        if unchanged and elapsed < self.mode_heartbeat_sec:
            return

        joy_msg = Bool()
        joy_msg.data = joy_active
        self.joy_active_pub.publish(joy_msg)

        auto_msg = Bool()
        auto_msg.data = auto_mode
        self.auto_mode_pub.publish(auto_msg)
        self.last_published_joy_active = joy_active
        self.last_published_auto_mode = auto_mode
        self.last_mode_publish_time = now

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
