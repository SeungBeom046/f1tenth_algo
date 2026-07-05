import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math


class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        # ============ PD 튜닝 파라미터 ============
        self.kp = 3.0          # P 게인: 오차에 비례해서 조향(크게 하면 반응 빠르지만 진동 발생)
        self.kd = 0.1        # D 게인: 오차 변화율에 비례해서 조향(크게 하면 진동 억제되지만 반응 느려짐)
        self.target_dist = 1.0 # 양쪽 벽까지 유지할 목표 거리 (m) *트랙 폭의 절반 정도로 설정하면 중앙 유지
        self.lookahead = 0.5   # 미래 위치 예측 거리 (m) *크게 하면 코너 예측 빠르지만 불안정
        # ======================================

        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('Wall Follow Node 시작! (양쪽 벽 감지)')

    def get_range(self, scan_msg, angle_deg):
        """
        특정 각도(도)의 LiDAR 거리값 반환
        
        LiDAR는 ranges[] 배열로 모든 방향의 거리를 줌.
        원하는 각도 → 배열 인덱스로 변환하는 공식:
        
        index = (원하는 각도 - 시작 각도) / 각도 간격
        
        예) angle_min = -2.35rad, angle_increment = 0.004rad 일 때
            -90도(-1.57rad)의 인덱스 = (-1.57 - (-2.35)) / 0.004 = 195
        """
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - scan_msg.angle_min)
                    / scan_msg.angle_increment)
        # 배열 범위 초과 방지
        index = max(0, min(index, len(scan_msg.ranges) - 1))
        r = scan_msg.ranges[index]
        # NaN(측정 불가), inf(범위 초과) 처리
        if math.isnan(r) or math.isinf(r):
            return scan_msg.range_max
        return r

    def get_wall_distance(self, scan_msg, side='right'):
        """
        한쪽 벽까지의 수직 거리 계산
        
        두 개의 LiDAR 빔(a, b)과 사이 각도(theta)를 이용해
        벽까지의 수직 거리와 벽과의 각도(alpha)를 삼각함수로 계산.
        
        빔 a, b와 theta로 삼각형을 이루고: 
        alpha = atan2(a*cos(θ) - b, a*sin(θ))
        수직 거리 D = b * cos(alpha)
        
        lookahead 거리 추가 (앞을 미리 내다보기):
        future_D = D + lookahead * sin(alpha)
        
        이유: 현재 위치만 보면 코너에서 늦게 반응함.
             조금 앞의 예측 거리를 더해서 미리 핸들을 꺾게 함.
        """
        theta = 50  # 두 빔 사이 각도 (도), 클수록 더 정확하지만 노이즈에 민감

        if side == 'right':
            # 오른쪽: -90도 기준 (오른쪽이 음수 방향)
            a = self.get_range(scan_msg, -90 + theta)  # -40도
            b = self.get_range(scan_msg, -90)           # -90도
        else:
            # 왼쪽: +90도 기준 (왼쪽이 양수 방향)
            a = self.get_range(scan_msg, 90 - theta)   # +40도
            b = self.get_range(scan_msg, 90)            # +90도

        theta_rad = math.radians(theta)

        # 벽과 차체 사이의 각도 alpha 계산
        alpha = math.atan2(
            a * math.cos(theta_rad) - b,  # 수평 성분 차이
            a * math.sin(theta_rad)        # 수직 성분
        )

        # 현재 벽까지 수직 거리
        dist = b * math.cos(alpha)

        # 미래 예측 거리 (lookahead 적용)
        future_dist = dist + self.lookahead * math.sin(alpha)
        return future_dist

    def scan_callback(self, scan_msg):
        """
        LiDAR 데이터가 들어올 때마다 실행되는 콜백 함수
        
        [전체 제어 흐름]
        1. 양쪽 벽까지 거리 측정
        2. 오차 계산 (중앙 유지 기준)
        3. PD 제어로 조향각 계산
        4. 속도 결정 (조향각에 반비례)
        5. 드라이브 명령 발행
        """

        # 1. 양쪽 벽까지 거리 측정
        right_dist = self.get_wall_distance(scan_msg, side='right')
        left_dist  = self.get_wall_distance(scan_msg, side='left')

        # 2. 오차 계산
        # [핵심 아이디어]
        # 오른쪽이 멀고 왼쪽이 가까우면 → 오른쪽으로 핸들 꺾어야 함 (error > 0)
        # 오른쪽이 가깝고 왼쪽이 멀면 → 왼쪽으로 핸들 꺾어야 함 (error < 0)
        # 
        # 방법 A: 트랙 중앙 유지 (양쪽 거리 차이)
        # error = right_dist - left_dist
        # → 둘이 같으면 0 (완벽한 중앙), 오른쪽이 멀면 양수
        #
        # 방법 B: 목표 거리 기준 (지금 사용)
        # 오른쪽/왼쪽 중 더 가까운 쪽을 위험으로 판단해서 제어
        if right_dist < left_dist:
            # 오른쪽이 더 가까움 → 오른쪽 벽 기준으로 제어
            # error > 0이면 오른쪽으로 너무 붙은 것 → 왼쪽으로 꺾어야 함
            error = right_dist - self.target_dist
        else:
            # 왼쪽이 더 가까움 → 왼쪽 벽 기준으로 제어
            # error < 0이면 왼쪽으로 너무 붙은 것 → 오른쪽으로 꺾어야 함
            error = -(left_dist - self.target_dist)

        # 3. 시간 간격(dt) 계산
        # D 항 계산에 필요 (오차 변화율 = 오차 변화량 / 시간)
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        dt = max(dt, 1e-3)  # 0으로 나누기 방지 (최소 1ms)

        # 4. PD 제어
        # [P 항] 현재 오차에 비례한 조향
        # error가 크면 많이 꺾고, 작으면 조금 꺾음
        p_term = self.kp * error

        # [D 항] 오차 변화율에 비례한 조향
        # 오차가 빠르게 줄어들면 (derivative < 0) 조향을 줄여서 오버슈트 방지
        # 오차가 빠르게 커지면 (derivative > 0) 조향을 늘려서 빠르게 대응
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        steering = p_term + d_term

        # 5. 조향각 제한
        # 너무 급격한 조향 방지 (-0.4 ~ 0.4 라디안 ≈ -23 ~ 23도)
        steering = max(min(steering, 0.4), -0.4)

        # 6. 속도 결정
        # [핵심 아이디어] 핸들을 많이 꺾을수록 속도를 줄임
        # 이유: 고속 코너링 시 원심력으로 트랙 이탈 위험
        abs_steer = abs(steering)
        if abs_steer > 0.2:
            speed = 0.5   # 급코너: 천천히
        elif abs_steer > 0.1:
            speed = 0.8   # 완만한 코너: 중간
        else:
            speed = 1.2   # 직선: 빠르게

        # 7. 드라이브 메시지 발행
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = speed
        self.publisher.publish(drive_msg)

        # 8. 디버깅 로그
        print(
            f'R: {right_dist:.2f}m | L: {left_dist:.2f}m | '
            f'error: {error:.2f} | '
            f'steer: {math.degrees(steering):.1f}deg | '
            f'speed: {speed:.1f}m/s', flush=True
        )

        self.prev_error = error
        self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()