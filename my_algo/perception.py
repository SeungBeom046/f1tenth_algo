"""점유격자 노드의 프레임 처리와 phantom 노드의 격자 복원 (ROS 비의존).

occupancy_grid_node, phantom_node, 시뮬이 같은 함수를 쓴다. 계산 자체는 occupancy.py, phantom.py에 있고
여기는 좌표 변환·대역 분리·누적·잘라내기를 한 프레임 단위로 묶는다.
"""
from dataclasses import dataclass

import numpy as np

from my_algo.occupancy import (
    GridSpec, crop, height_band_mask, occlusion_boundary, out_of_band_free_ratio, raycast,
    single_frame_grid,
)


@dataclass
class FrameResult:
    grid: np.ndarray            # 누적 격자 (odom 고정 창)
    boundary: np.ndarray        # 누적 격자의 가림 경계 마스크
    frame_boundary: np.ndarray  # 이번 프레임만의 가림 경계 (RQ4)
    origin: np.ndarray          # 라이다 원점 odom xy
    debug: dict


def process_frame(acc, spec, xyz_lidar, to_odom, to_base, band, max_range, debug_oob=False):
    """라이다 프레임 점 (N,3) + odom<-lidar, base<-lidar 4x4 → 누적 격자·경계.

    높이 대역은 base_link z 기준, 레이캐스팅은 odom xy에서 대역 안 점만 쓴다 (occupancy.py 설명).
    acc(OccupancyAccumulator)는 갱신된다.
    """
    hom = np.column_stack([xyz_lidar, np.ones(len(xyz_lidar))])
    z_base = hom @ to_base[2, :]
    xy_odom = (hom @ to_odom[:2, :].T)
    origin = to_odom[:2, 3]
    in_band = height_band_mask(z_base, *band)

    free, occ = raycast(spec, origin, xy_odom[in_band], max_range)
    acc.add_frame(free, occ)
    grid = acc.grid()
    boundary = occlusion_boundary(grid)
    frame_boundary = occlusion_boundary(single_frame_grid(spec, free, occ))

    debug = {'points': int(len(xyz_lidar)), 'in_band': int(in_band.sum()),
             'free_cells': int(len(free)), 'occ_cells': int(len(occ))}
    if debug_oob:
        ratio, n = out_of_band_free_ratio(spec, origin, free, grid, xy_odom[~in_band])
        debug.update(oob_would_free_ratio=round(ratio, 4), oob_would_free_cells=n)
    return FrameResult(grid, boundary, frame_boundary, origin, debug)


def cropped(grid, spec, center_xy, half_x, half_y):
    """발행용 에고 중심 잘라내기와 그 격자의 GridSpec (phantom 노드가 메시지에서 복원하는 것과 같다)."""
    sub, (ox, oy) = crop(grid, spec, center_xy, half_x, half_y)
    return sub, GridSpec(ox, oy, spec.resolution, sub.shape[1], sub.shape[0])


def decode_grid(data, width, height, origin_x, origin_y, resolution):
    """OccupancyGrid 메시지 값 → (int8 격자, GridSpec)."""
    spec = GridSpec(origin_x, origin_y, resolution, width, height)
    return np.asarray(data, dtype=np.int8).reshape(height, width), spec
