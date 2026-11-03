#!/usr/bin/env python3
"""Mid-360 내장 IMU 정지 캘리브레이션

구하는 값
  - 가속도 단위 자동 판별 (g / m/s^2)
  - 장착 roll, pitch  -> tf_static.launch.py 의 LIDAR_ROLL / LIDAR_PITCH
  - 자이로 바이어스   -> /imu 재발행 노드에서 빼줄 값
  - 노이즈 표준편차, 실제 발행 주기

사용법 (평평한 바닥, 차량 완전 정지, 라이다 켜고 5분 이상 예열 후)
  1) python3 imu_static_calib.py 10          # 차를 전방으로 둔 상태
     python3 imu_static_calib.py 10 --topic /livox/imu   # 토픽 지정 (기본 /livox/imu)
  2) 차를 제자리에서 180도 돌려 놓고 한 번 더 실행
  3) 두 결과의 roll/pitch 를 평균 -> 바닥 기울기는 상쇄되고 장착 기울기만 남음
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

G = 9.80665
_ap = argparse.ArgumentParser()
_ap.add_argument('duration', nargs='?', type=float, default=10.0, help='수집 시간 [s]')
_ap.add_argument('--topic', default='/livox/imu', help='IMU 토픽 (check_vehicle.sh로 확인)')
_args = _ap.parse_args()
TOPIC = _args.topic
DURATION = _args.duration
GYRO_MOTION_STD = 0.02  # rad/s, 이보다 흔들리면 정지 상태가 아니라고 판단


class Collector(Node):
    def __init__(self):
        super().__init__('imu_static_calib')
        self.acc, self.gyr = [], []
        self.create_subscription(Imu, TOPIC, self.cb, qos_profile_sensor_data)

    def cb(self, m):
        a, w = m.linear_acceleration, m.angular_velocity
        self.acc.append((a.x, a.y, a.z))
        self.gyr.append((w.x, w.y, w.z))


def main():
    rclpy.init()
    node = Collector()
    print(f'{TOPIC} 에서 {DURATION:.0f}초 수집 중... 차를 건드리지 마세요')
    t0 = time.time()
    while time.time() - t0 < DURATION:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if len(node.acc) < 50:
        sys.exit(f'메시지 {len(node.acc)}개만 수신됨. 토픽 이름과 IMU 출력 활성화 여부 확인')

    acc = np.array(node.acc)
    gyr = np.array(node.gyr)

    # 단위 판별: 정지 시 가속도 크기가 ~1 이면 g, ~9.8 이면 m/s^2
    norm = np.linalg.norm(acc, axis=1).mean()
    unit = 'g' if norm < 3.0 else 'm/s^2'
    acc_g = acc if unit == 'g' else acc / G
    ax, ay, az = acc_g.mean(axis=0)

    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    gyro_bias = gyr.mean(axis=0)
    gyro_std = gyr.std(axis=0)
    acc_std = acc_g.std(axis=0) * G

    print('\n==== 결과 ====')
    print(f'수신 {len(acc)}개, 실제 주기 ≈ {len(acc) / DURATION:.0f} Hz (스펙 200 Hz)')
    print(f'가속도 단위: {unit}  (|a| 평균 = {norm:.3f})')
    print(f'중력 크기 검산: {np.linalg.norm([ax, ay, az]):.4f} g  (1.00 근처여야 정상)')
    print(f'roll  = {roll:+.5f} rad ({math.degrees(roll):+.2f} deg)')
    print(f'pitch = {pitch:+.5f} rad ({math.degrees(pitch):+.2f} deg)')
    print(f'gyro bias [rad/s] = {gyro_bias[0]:+.5f}, {gyro_bias[1]:+.5f}, {gyro_bias[2]:+.5f}')
    print(f'gyro std  [rad/s] = {gyro_std[0]:.5f}, {gyro_std[1]:.5f}, {gyro_std[2]:.5f}')
    print(f'acc std   [m/s^2] = {acc_std[0]:.4f}, {acc_std[1]:.4f}, {acc_std[2]:.4f}')

    if gyro_std.max() > GYRO_MOTION_STD:
        print('\n경고: 자이로 흔들림이 큼. 차가 움직였거나 진동원이 있음 -> 다시 측정')

    leak = G * math.sin(abs(pitch))
    print(f'\n참고: pitch 를 보정 안 하면 종방향 가속도에 {leak:.3f} m/s^2 오차 '
          f'(a_b 3 m/s^2 기준 {leak / 3 * 100:.1f}%)')


if __name__ == '__main__':
    main()
