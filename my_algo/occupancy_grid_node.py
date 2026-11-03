"""점유격자·가림 경계 노드 (exp 모드, 네 방법 공통).

역할: 높이 대역 안 점만 레이캐스팅해 odom 고정 격자에 누적하고, 에고 중심으로 잘라 발행한다.
파이프라인 위치: [Livox 드라이버 또는 deskew_node] → (occupancy_grid_node) → [phantom_node]
구독:
    cloud_topic   sensor_msgs/PointCloud2 (10 Hz, livox_frame)
    TF            odom<-livox_frame (프레임 stamp), base_link<-livox_frame (높이 대역용)
발행:
    /exp/grid            nav_msgs/OccupancyGrid (10 Hz, odom, 에고 중심 crop, -1/0/100)
    /exp/boundary        nav_msgs/GridCells (10 Hz, odom) 누적 격자의 가림 경계
    /exp/boundary_frame  nav_msgs/GridCells (10 Hz, odom) 한 프레임만의 가림 경계 (RQ4 흔들림 분석)
    /exp/grid_debug      std_msgs/String JSON (대역 안/밖 점 수, 처리 시간, 대역 밖 free 비율)
파라미터 (config/common.yaml):
    grid_x_min/x_max/y_min/y_max, grid_resolution, max_range   [EXP-FIXED] 격자 범위·해상도
    band_z_min, band_z_max                                     [EXP-FIXED] 높이 대역 (상한 < 박스 높이)
    n_frames, known_on, known_off                              [EXP-TUNE] 누적·히스테리시스
    crop_half_x, crop_half_y, debug_oob
실험 관련: 모든 방법이 이 노드 하나의 출력을 쓴다. 방법별 격자 파라미터는 금지.
"""

import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from my_algo.occupancy import (
    GridSpec, OccupancyAccumulator, crop, height_band_mask, occlusion_boundary,
    out_of_band_free_ratio, raycast, single_frame_grid,
)
from my_algo.pc2 import read_xyz
from my_algo.ros_util import lookup_matrix, publish_string


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')
        defaults = [
            ('cloud_topic', '/livox/lidar'), ('odom_frame', 'odom'), ('base_frame', 'base_link'),
            ('grid_x_min', -3.0), ('grid_x_max', 15.0), ('grid_y_min', -3.0), ('grid_y_max', 3.0),
            ('grid_resolution', 0.05), ('max_range', 8.0),
            ('band_z_min', 0.05), ('band_z_max', 0.30),
            ('n_frames', 3), ('known_on', 2), ('known_off', 1),
            ('crop_half_x', 8.0), ('crop_half_y', 2.5), ('debug_oob', False),
        ]
        for k, v in defaults:
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.spec = GridSpec.from_bounds(gp('grid_x_min'), gp('grid_x_max'),
                                         gp('grid_y_min'), gp('grid_y_max'), gp('grid_resolution'))
        self.acc = OccupancyAccumulator(self.spec, gp('n_frames'), gp('known_on'), gp('known_off'))
        self.band = (float(gp('band_z_min')), float(gp('band_z_max')))
        self.max_range = float(gp('max_range'))
        self.odom_frame, self.base_frame = gp('odom_frame'), gp('base_frame')
        self.crop_half = (float(gp('crop_half_x')), float(gp('crop_half_y')))
        self.debug_oob = bool(gp('debug_oob'))

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/exp/grid', 5)
        self.boundary_pub = self.create_publisher(GridCells, '/exp/boundary', 5)
        self.frame_boundary_pub = self.create_publisher(GridCells, '/exp/boundary_frame', 5)
        self.debug_pub = self.create_publisher(String, '/exp/grid_debug', 5)
        self.create_subscription(PointCloud2, gp('cloud_topic'), self.callback,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'격자 {self.spec.width}x{self.spec.height} @ {self.spec.resolution} m, '
            f'높이 대역 {self.band} m (base_link z), N={gp("n_frames")}')

    def callback(self, msg):
        t_start = time.perf_counter()
        stamp = Time.from_msg(msg.header.stamp)
        to_odom = lookup_matrix(self.buffer, self.odom_frame, msg.header.frame_id, stamp)
        to_base = lookup_matrix(self.buffer, self.base_frame, msg.header.frame_id, Time())
        if to_odom is None or to_base is None:
            self.get_logger().warn('TF 없음 (odom/base_link <- lidar): 프레임 건너뜀',
                                   throttle_duration_sec=2.0)
            return
        xyz = read_xyz(msg)
        if len(xyz) == 0:
            return
        hom = np.column_stack([xyz, np.ones(len(xyz))])
        z_base = hom @ to_base[2, :]
        xy_odom = (hom @ to_odom[:2, :].T)
        origin = to_odom[:2, 3]
        in_band = height_band_mask(z_base, *self.band)

        free, occ = raycast(self.spec, origin, xy_odom[in_band], self.max_range)
        self.acc.add_frame(free, occ)
        grid = self.acc.grid()
        boundary = occlusion_boundary(grid)
        frame_boundary = occlusion_boundary(single_frame_grid(self.spec, free, occ))

        debug = {'points': int(len(xyz)), 'in_band': int(in_band.sum()),
                 'free_cells': int(len(free)), 'occ_cells': int(len(occ))}
        if self.debug_oob:
            ratio, n = out_of_band_free_ratio(self.spec, origin, free, grid, xy_odom[~in_band])
            debug.update(oob_would_free_ratio=round(ratio, 4), oob_would_free_cells=n)

        base_in_odom = lookup_matrix(self.buffer, self.odom_frame, self.base_frame, stamp)
        center = base_in_odom[:2, 3] if base_in_odom is not None else origin
        self.publish_grid(msg.header.stamp, grid, center)
        self.publish_cells(self.boundary_pub, msg.header.stamp, boundary)
        self.publish_cells(self.frame_boundary_pub, msg.header.stamp, frame_boundary)
        debug['ms'] = round((time.perf_counter() - t_start) * 1e3, 1)
        publish_string(self.debug_pub, json.dumps(debug))

    def publish_grid(self, stamp, grid, center):
        sub, (ox, oy) = crop(grid, self.spec, center, *self.crop_half)
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.info.resolution = self.spec.resolution
        msg.info.width = sub.shape[1]
        msg.info.height = sub.shape[0]
        msg.info.origin.position.x = float(ox)
        msg.info.origin.position.y = float(oy)
        msg.info.origin.orientation.w = 1.0
        msg.data = sub.reshape(-1).astype(np.int8).tolist()
        self.grid_pub.publish(msg)

    def publish_cells(self, pub, stamp, mask):
        msg = GridCells()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.cell_width = msg.cell_height = self.spec.resolution
        centers = self.spec.cell_centers(np.flatnonzero(mask.reshape(-1)))
        msg.cells = [Point(x=float(x), y=float(y), z=0.0) for x, y in centers]
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
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
