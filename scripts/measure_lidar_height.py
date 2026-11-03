#!/usr/bin/env python3
"""평평한 바닥에서 라이다 원점 높이와 장착 roll/pitch를 측정한다. [EXP0-MEASURE]

사용 (Jetson, 라이다 드라이버 실행 중, 차 주변 0.4~4 m 바닥이 비어 있어야 함):
    source install/setup.bash
    python3 scripts/measure_lidar_height.py --topic /livox/lidar --frames 10

출력값을 tf_static.launch.py의 livox_frame z / roll / pitch에 넣는다.
base_link z=0이 바닥이므로 livox_frame z(base_link 기준) = height_m.
Mid-360은 비반복 스캔이라 여러 프레임을 모아 바닥 점 밀도를 확보한다.
"""

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from my_algo.floor_plane import measure


def cloud_to_xyz(msg):
    pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if isinstance(pts, np.ndarray) and pts.dtype.names:
        return np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
    return np.array([tuple(p) for p in pts], dtype=np.float64).reshape(-1, 3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--topic', required=True, help='PointCloud2 토픽 (check_vehicle.sh로 확인)')
    ap.add_argument('--frames', type=int, default=10)
    ap.add_argument('--range-min', type=float, default=0.4, help='차체 점 제외 [m]')
    ap.add_argument('--range-max', type=float, default=4.0)
    ap.add_argument('--z-max', type=float, default=-0.05,
                    help='라이다 프레임 z 상한. 바닥은 라이다보다 아래 [m]')
    ap.add_argument('--threshold', type=float, default=0.01, help='RANSAC inlier 거리 [m]')
    ap.add_argument('--timeout', type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('measure_lidar_height')
    clouds, frame_ids = [], set()

    def callback(msg):
        if len(clouds) < args.frames:
            clouds.append(cloud_to_xyz(msg))
            frame_ids.add(msg.header.frame_id)

    node.create_subscription(PointCloud2, args.topic, callback, qos_profile_sensor_data)
    start = node.get_clock().now()
    while rclpy.ok() and len(clouds) < args.frames:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - start).nanoseconds / 1e9 > args.timeout:
            print(f'시간 초과: {args.topic}에서 {len(clouds)}/{args.frames} 프레임만 수신',
                  file=sys.stderr)
            break
    node.destroy_node()
    rclpy.shutdown()
    if not clouds:
        return 1

    pts = np.vstack(clouds)
    r = np.linalg.norm(pts[:, :2], axis=1)
    roi = pts[(r >= args.range_min) & (r <= args.range_max) & (pts[:, 2] <= args.z_max)]
    result = measure(roi, threshold=args.threshold)

    print(f'point cloud frame_id : {sorted(frame_ids)}')
    print(f'frames / ROI points  : {len(clouds)} / {result["points"]} '
          f'(floor inliers {result["inliers"]})')
    print(f'height_m (livox z)   : {result["height_m"]:.4f}')
    print(f'roll                 : {math.degrees(result["roll_rad"]):+.3f} deg '
          f'({result["roll_rad"]:+.5f} rad)')
    print(f'pitch                : {math.degrees(result["pitch_rad"]):+.3f} deg '
          f'({result["pitch_rad"]:+.5f} rad)')
    print(f'보정 후 바닥 z 평균   : {result["floor_z_mean_m"]:+.4f} m '
          f'(std {result["floor_z_std_m"]:.4f}, -height와 같아야 함)')
    if result['inliers'] < 0.5 * result['points']:
        print('경고: inlier가 ROI의 절반 미만. 주변 장애물을 치우거나 --range-max를 줄이세요.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
