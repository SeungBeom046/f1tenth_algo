"""Joystick controller node for F1TENTH real car."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
import math


class JoyControllerNode(Node):
    """
    로지텍 조이스틱 제어 노드
    
    [버튼 매핑 - 로지텍 F710 기준]
    왼쪽 스틱 상하  → 속도 제어
    오른쪽 스틱 좌우 → 조향 제어
    LB 버튼 (버튼 4) → 데드맨 스위치 (이거 누르고 있어야 차 움직임)
    RB 버튼 (버튼 5) → 자율주행 모드 토글
    B 버튼 (버튼 1)  → 긴급 정지
    
    [안전 설계]
    - 데드맨 스위치(LB): 이걸 누르고 있어야만 조이스틱으로 차 제어 가능
      (손 놓으면 자동 정지)
    - RB: 자율주행 모드 ON/OFF
    - B: 긴급 정지 (자율주행 + 조이스틱 둘 다 정지)
    """

    def __init__(self):
        super().__init__('joy_controller_node')

        # ============ 튜닝 파라미터 ============
        self.max_speed = 2.0        # 조이스틱 최대 속도 (m/s)
        self.ERPM_GAIN = 4614.0     # wall_follow_real.py와 동일하게
        self.SERVO_CENTER = 0.5
        self.SERVO_GAIN = 0.4
        # ======================================

        # 상태 변수
        self.autonomous_mode = False  # 자율주행 모드 여부
        self.deadman_pressed = False  # 데드맨 스위치 상태
        self.emergency_stop = False   # 긴급 정지 상태
        self.prev_rb = False          # RB 버튼 이전 상태 (토글용)

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
            '  LB 누른 채로: 조이스틱 제어 활성화\n'
            '  왼쪽 스틱 상하: 속도\n'
            '  오른쪽 스틱 좌우: 조향\n'
            '  RB: 자율주행 모드 토글\n'
            '  B: 긴급 정지'
        )

    def joy_callback(self, msg):
        """
        조이스틱 입력 처리
        
        [로지텍 F710 축/버튼 인덱스]
        axes[0]: 왼쪽 스틱 좌우 (-1=오른쪽, 1=왼쪽)
        axes[1]: 왼쪽 스틱 상하 (-1=아래, 1=위)
        axes[3]: 오른쪽 스틱 좌우 (-1=오른쪽, 1=왼쪽)
        axes[4]: 오른쪽 스틱 상하 (-1=아래, 1=위)
        buttons[4]: LB (데드맨 스위치)
        buttons[5]: RB (자율주행 토글)
        buttons[1]: B (긴급 정지)
        """

        # 버튼 읽기
        deadman = bool(msg.buttons[4])   # LB
        rb = bool(msg.buttons[5])         # RB
        emergency = bool(msg.buttons[1])  # B

        # 긴급 정지
        if emergency:
            self.emergency_stop = True
            self.autonomous_mode = False
            self.stop()
            print('🚨 긴급 정지!', flush=True)
            self._publish_mode(joy_active=True, auto_mode=False)
            return

        # 긴급 정지 해제 (B 안 누른 상태)
        self.emergency_stop = False

        # RB로 자율주행 모드 토글 (엣지 감지)
        if rb and not self.prev_rb:
            self.autonomous_mode = not self.autonomous_mode
            mode_str = '자율주행' if self.autonomous_mode else '수동'
            print(f'🔄 모드 전환: {mode_str}', flush=True)
        self.prev_rb = rb

        # 데드맨 스위치 상태 업데이트
        self.deadman_pressed = deadman

        # 자율주행 모드이고 조이스틱 입력 없으면 → 자율주행에 양보
        if self.autonomous_mode and not deadman:
            self._publish_mode(joy_active=False, auto_mode=True)
            return

        # 조이스틱 제어 (데드맨 눌린 상태에서만)
        if deadman:
            # 스틱 값 읽기
            speed_axis = msg.axes[1]    # 왼쪽 스틱 상하
            steer_axis = msg.axes[3]    # 오른쪽 스틱 좌우

            # 속도 변환 (m/s → ERPM)
            speed_ms = speed_axis * self.max_speed
            erpm = speed_ms * self.ERPM_GAIN

            # 조향 변환 (라디안 → 서보 위치)
            steering_rad = steer_axis * self.max_steer if hasattr(
                self, 'max_steer') else steer_axis * 0.42
            servo_pos = self.SERVO_CENTER - steering_rad * self.SERVO_GAIN
            servo_pos = max(0.0, min(1.0, servo_pos))

            # VESC에 명령 발행
            speed_msg = Float64()
            speed_msg.data = erpm
            self.speed_pub.publish(speed_msg)

            servo_msg = Float64()
            servo_msg.data = servo_pos
            self.servo_pub.publish(servo_msg)

            self._publish_mode(joy_active=True, auto_mode=False)

            print(
                f'🕹️ 조이스틱 | '
                f'speed: {speed_ms:.2f}m/s | '
                f'ERPM: {erpm:.0f} | '
                f'servo: {servo_pos:.3f}',
                flush=True
            )
        else:
            # 데드맨 안 눌린 상태 + 수동 모드 → 정지
            if not self.autonomous_mode:
                self.stop()
            self._publish_mode(
                joy_active=False,
                auto_mode=self.autonomous_mode
            )

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