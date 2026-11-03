"""Mid-360 내장 IMU 정규화 재발행 노드 (exp·Exp0 공통, 분석 전용).

역할: 가속도 g → m/s², 자이로 바이어스 차감, 중력(roll/pitch 포함)을 뺀 base_link 종방향 가속도 발행.
    제어 루프에는 쓰지 않는다 (종방향 jerk, Exp0 a_b 측정용).
파이프라인 위치: [Livox 드라이버 /livox/imu] → (imu_republisher) → [rosbag 기록 → analysis/extract_run.py]
구독: input_topic (sensor_msgs/Imu, 200 Hz, livox_frame — 토픽 이름 TBD(세션 1)), /imu/recalibrate (Bool)
      TF base_link <- livox_imu (회전만, tf_static.launch.py)
발행:
    /imu                     sensor_msgs/Imu (200 Hz, livox_imu) m/s², rad/s, 바이어스 차감, 축 회전 없음
    /imu/linear_accel_base   geometry_msgs/Vector3Stamped (200 Hz, base_link) 중력 제거 선형 가속도, x = 종방향
    /imu/status              std_msgs/String JSON (1 Hz) 수신율, 보정 여부, |g_ref|, TF 확보
파라미터 (config/imu.yaml): accel_unit (기본 g), gyro_unit, gyro_bias [EXP0-MEASURE], gravity_ref,
    calibration_samples, track_attitude
실험 관련: 정지 보정은 회차마다 정지 상태에서 (/imu/recalibrate). 제동 중 pitch 1° = 0.17 m/s² 오차라
    자이로로 자세 변화를 추적한다 (my_algo/imu_normalize.py).
"""

import json
import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from my_algo.imu_normalize import G0, ImuNormalizer, norm, quat_to_matrix


class ImuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')
        p = self.declare_parameter
        p('input_topic', '/livox/imu')
        p('output_topic', '/imu')
        p('linear_accel_topic', '/imu/linear_accel_base')
        p('status_topic', '/imu/status')
        p('recalibrate_topic', '/imu/recalibrate')
        p('imu_frame', 'livox_imu')
        p('base_frame', 'base_link')
        p('accel_unit', 'g')
        p('gyro_unit', 'rad_s')
        p('gyro_bias', [0.0, 0.0, 0.0])
        p('gravity_ref', [0.0])  # [0.0] = 시작 시 정지 샘플로 자동 추정
        p('calibration_samples', 400)
        p('rest_accel_tolerance', 0.5)
        p('rest_gyro_tolerance', 0.05)
        p('track_attitude', True)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        gravity_ref = list(gp('gravity_ref'))
        self.engine = ImuNormalizer(
            accel_unit=gp('accel_unit'),
            gyro_unit=gp('gyro_unit'),
            gyro_bias=list(gp('gyro_bias')),
            gravity_ref=gravity_ref if len(gravity_ref) == 3 else None,
            calibration_samples=gp('calibration_samples'),
            rest_accel_tolerance=gp('rest_accel_tolerance'),
            rest_gyro_tolerance=gp('rest_gyro_tolerance'),
            track_attitude=gp('track_attitude'),
        )
        self.imu_frame = gp('imu_frame')
        self.base_frame = gp('base_frame')
        self.rotation_base_from_imu = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.imu_pub = self.create_publisher(Imu, gp('output_topic'), 50)
        self.lin_pub = self.create_publisher(Vector3Stamped, gp('linear_accel_topic'), 50)
        self.status_pub = self.create_publisher(String, gp('status_topic'), 10)
        self.create_subscription(Imu, gp('input_topic'), self.imu_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, gp('recalibrate_topic'), self.recalibrate_callback, 10)
        self.create_timer(1.0, self.status_timer)
        self.samples_in = 0
        self.get_logger().info(
            f'IMU republisher: {gp("input_topic")} -> {gp("output_topic")} '
            f'(accel_unit={gp("accel_unit")}). 차를 정지 상태로 두세요.')

    def recalibrate_callback(self, msg):
        if msg.data:
            self.engine.reset()
            self.get_logger().info('IMU 정지 재보정 시작: 차를 완전히 멈춰 두세요.')

    def lookup_rotation(self):
        if self.rotation_base_from_imu is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.imu_frame, Time(), timeout=Duration(seconds=0.0))
        except TransformException:
            return False
        r = tf.transform.rotation
        self.rotation_base_from_imu = quat_to_matrix((r.w, r.x, r.y, r.z))
        return True

    def imu_callback(self, msg):
        self.samples_in += 1
        a, g = msg.linear_acceleration, msg.angular_velocity
        accel, gyro = self.engine.to_si((a.x, a.y, a.z), (g.x, g.y, g.z))
        if not all(math.isfinite(x) for x in (*accel, *gyro)):
            return

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.imu_frame
        out.orientation_covariance[0] = -1.0  # 자세 미제공
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = gyro
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = accel
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.imu_pub.publish(out)

        if not self.engine.ready:
            if self.engine.feed_calibration(accel, gyro):
                mag = norm(self.engine.gravity_ref)
                self.get_logger().info(
                    f'IMU 정지 보정 완료: |g_ref|={mag:.3f} m/s^2 '
                    f'g_ref={tuple(round(x, 3) for x in self.engine.gravity_ref)}')
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.engine.update_attitude(stamp, gyro)
        if not self.lookup_rotation():
            return
        lin = ImuNormalizer.to_base(
            self.rotation_base_from_imu, self.engine.linear_accel_imu(accel))
        vec = Vector3Stamped()
        vec.header.stamp = msg.header.stamp
        vec.header.frame_id = self.base_frame
        vec.vector.x, vec.vector.y, vec.vector.z = lin
        self.lin_pub.publish(vec)

    def status_timer(self):
        g_ref = self.engine.gravity_ref
        status = {
            'samples_per_s': self.samples_in,
            'calibrated': self.engine.ready,
            'g_ref_norm': None if g_ref is None else round(norm(g_ref), 4),
            'tf_ok': self.rotation_base_from_imu is not None,
        }
        self.samples_in = 0
        if g_ref is not None and abs(norm(g_ref) - G0) > 0.3:
            self.get_logger().warn(
                f'|g_ref|={norm(g_ref):.3f}: 9.81과 다름. accel_unit 또는 정지 상태 확인')
        if self.engine.ready and not status['tf_ok']:
            self.get_logger().warn(
                f'TF {self.base_frame} <- {self.imu_frame} 없음: linear_accel 미발행',
                throttle_duration_sec=5.0)
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
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
