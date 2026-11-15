"""2D 점유격자와 가림 경계 (ROS 비의존, numpy). 네 방법이 모두 이 격자 하나를 쓴다.

파이프라인 위치: (deskew된) 점군 → (이 모듈) → phantom 배치
    1. 높이 대역 분리: base_link z가 [z_min, z_max] 안인 점만 레이캐스팅에 쓴다.
       위쪽 빔이 박스(0.35 m) 위를 넘어 뒤 바닥/벽에 닿은 점으로 free를 칠하면 박스 뒤 가림이
       사라지기 때문이다. 대역 밖 점은 occupied와 free 어디에도 쓰지 않는다. [EXP-FIXED]
    2. 레이캐스팅: 라이다 원점 → 대역 안 끝점. 지나간 칸 free, 끝점 칸 occupied.
    3. N 프레임 누적 + 히스테리시스: 최근 N 프레임 중 관측된 프레임 수로 known/unknown 결정.
    4. 가림 경계: unknown 칸 중 free와 4-이웃으로 맞닿은 칸.

격자는 odom 프레임의 고정 창(파라미터)에 저장한다. 발행은 에고 중심으로 잘라서 한다.
칸 값: UNKNOWN=-1, FREE=0, OCCUPIED=100 (nav_msgs/OccupancyGrid와 같은 규약).
"""

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


@dataclass(frozen=True)
class GridSpec:
    """odom 프레임 고정 격자. 칸 (row, col) = (y, x) 인덱스, 원점은 칸 (0, 0)의 모서리."""
    origin_x: float
    origin_y: float
    resolution: float
    width: int   # x 방향 칸 수
    height: int  # y 방향 칸 수

    @classmethod
    def from_bounds(cls, x_min, x_max, y_min, y_max, resolution):
        return cls(x_min, y_min, resolution,
                   int(math.ceil((x_max - x_min) / resolution)),
                   int(math.ceil((y_max - y_min) / resolution)))

    @property
    def size(self):
        return self.width * self.height

    def to_cell(self, xy):
        """(N,2) odom 좌표 → (col, row) 정수 배열, 범위 밖 마스크."""
        col = np.floor((xy[:, 0] - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((xy[:, 1] - self.origin_y) / self.resolution).astype(np.int64)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        return col, row, inside

    def cell_centers(self, flat_idx):
        row, col = np.divmod(flat_idx, self.width)
        return np.column_stack([self.origin_x + (col + 0.5) * self.resolution,
                                self.origin_y + (row + 0.5) * self.resolution])


def height_band_mask(z_base, z_min, z_max):
    """base_link z 기준 높이 대역 안 점 마스크. [EXP-FIXED] 대역 상한은 박스 높이보다 낮아야 한다."""
    return (z_base >= z_min) & (z_base <= z_max)


def raycast(spec, origin_xy, endpoints_xy, max_range=None):
    """라이다 원점에서 끝점들로 광선을 그어 이번 프레임의 (free 칸, occupied 칸) flat 인덱스를 낸다.

    같은 칸에 떨어지는 끝점은 하나로 합쳐 계산량을 줄인다. 같은 프레임에서 occupied인 칸은
    다른 광선이 스쳐 지나가도 free로 만들지 않는다.
    """
    if len(endpoints_xy) == 0:
        return np.empty(0, np.int64), np.empty(0, np.int64)
    ends = np.asarray(endpoints_xy, dtype=np.float64)
    if max_range is not None:
        rng = np.hypot(ends[:, 0] - origin_xy[0], ends[:, 1] - origin_xy[1])
        ends = ends[rng <= max_range]
    col, row, inside = spec.to_cell(ends)
    occ, first = np.unique(row[inside] * spec.width + col[inside], return_index=True)

    # 같은 칸 끝점은 하나만 광선을 쏜다. 칸 중심이 아니라 실제 끝점 좌표를 쓴다
    # (중심으로 옮기면 면 바로 뒤 칸이 free로 칠해져 가림 경계가 한 칸 밀린다).
    ends_in = ends[inside][first]
    ends_out = ends[~inside]
    targets = np.vstack([ends_in, ends_out]) if len(ends_out) else ends_in
    if len(targets) == 0:
        return np.empty(0, np.int64), occ
    d = targets - np.asarray(origin_xy, dtype=np.float64)
    length = np.hypot(d[:, 0], d[:, 1])
    step = 0.5 * spec.resolution
    n = np.maximum(1, np.ceil(length / step).astype(np.int64))
    total = int(n.sum())
    ray_id = np.repeat(np.arange(len(targets)), n)
    starts = np.cumsum(n) - n
    k = np.arange(total) - np.repeat(starts, n)
    # 끝점 칸 바로 앞까지만 free: t = k / n (k = 0..n-1)
    t = k / n[ray_id]
    pts = np.asarray(origin_xy) + d[ray_id] * t[:, None]
    c, r, ok = spec.to_cell(pts)
    free = np.unique(r[ok] * spec.width + c[ok])
    free = np.setdiff1d(free, occ, assume_unique=True)
    return free, occ


class OccupancyAccumulator:
    """최근 N 프레임 관측을 누적하고 히스테리시스로 known/unknown을 정한다.

    known_on  [EXP-TUNE]: unknown 칸이 known이 되려면 최근 N 프레임 중 이 수 이상 관측
    known_off [EXP-TUNE]: known 칸이 unknown으로 돌아가는 기준 (관측 수 < known_off). 1 <= off <= on <= N
    occ_min_hits: occupied로 볼 최소 관측 수
    """

    def __init__(self, spec, n_frames=3, known_on=2, known_off=1, occ_min_hits=1):
        if not 1 <= known_off <= known_on <= n_frames:
            raise ValueError('1 <= known_off <= known_on <= n_frames 이어야 한다')
        self.spec = spec
        self.n_frames = n_frames
        self.known_on = known_on
        self.known_off = known_off
        self.occ_min_hits = occ_min_hits
        self.frames = deque(maxlen=n_frames)
        self.known = np.zeros(spec.size, dtype=bool)

    def reset(self):
        self.frames.clear()
        self.known[:] = False

    def add_frame(self, free_idx, occ_idx):
        free = np.zeros(self.spec.size, dtype=bool)
        occ = np.zeros(self.spec.size, dtype=bool)
        free[free_idx] = True
        occ[occ_idx] = True
        self.frames.append((free, occ))

    def grid(self):
        """누적 격자 (height, width) int8."""
        free_c = np.zeros(self.spec.size, dtype=np.int16)
        occ_c = np.zeros(self.spec.size, dtype=np.int16)
        for free, occ in self.frames:
            free_c += free
            occ_c += occ
        observed = free_c + occ_c
        self.known = np.where(self.known, observed >= self.known_off, observed >= self.known_on)
        out = np.full(self.spec.size, UNKNOWN, dtype=np.int8)
        out[self.known] = FREE
        out[self.known & (occ_c >= self.occ_min_hits)] = OCCUPIED
        return out.reshape(self.spec.height, self.spec.width)


def single_frame_grid(spec, free_idx, occ_idx):
    """누적 없이 한 프레임만으로 만든 격자 (RQ4 프레임별 경계 분석용)."""
    out = np.full(spec.size, UNKNOWN, dtype=np.int8)
    out[free_idx] = FREE
    out[occ_idx] = OCCUPIED
    return out.reshape(spec.height, spec.width)


def occlusion_boundary(grid):
    """가림 경계 마스크: UNKNOWN 칸 중 4-이웃에 FREE가 있는 칸."""
    free = grid == FREE
    near = np.zeros_like(free)
    near[1:, :] |= free[:-1, :]
    near[:-1, :] |= free[1:, :]
    near[:, 1:] |= free[:, :-1]
    near[:, :-1] |= free[:, 1:]
    return (grid == UNKNOWN) & near


def out_of_band_free_ratio(spec, origin_xy, band_free_idx, grid, oob_endpoints_xy):
    """디버그: 대역 밖 점까지 레이캐스팅에 썼다면 free가 됐을 칸 중 지금 unknown인 칸의 비율.

    분모는 대역 밖 점 광선이 지나는 칸 수. 이 값이 크면 높이 대역 필터가 가림을 지키고 있다는 뜻.
    """
    if len(oob_endpoints_xy) == 0:
        return 0.0, 0
    oob_free, _ = raycast(spec, origin_xy, oob_endpoints_xy)
    if len(oob_free) == 0:
        return 0.0, 0
    flat = grid.reshape(-1)
    would_change = np.count_nonzero(flat[oob_free] == UNKNOWN)
    return would_change / len(oob_free), int(would_change)


def crop(grid, spec, center_xy, half_x, half_y):
    """에고 중심 창으로 자른 격자와 그 원점 (odom). 발행용."""
    c0 = max(0, int((center_xy[0] - half_x - spec.origin_x) / spec.resolution))
    c1 = min(spec.width, int(math.ceil((center_xy[0] + half_x - spec.origin_x) / spec.resolution)))
    r0 = max(0, int((center_xy[1] - half_y - spec.origin_y) / spec.resolution))
    r1 = min(spec.height, int(math.ceil((center_xy[1] + half_y - spec.origin_y) / spec.resolution)))
    sub = grid[r0:r1, c0:c1]
    return sub, (spec.origin_x + c0 * spec.resolution, spec.origin_y + r0 * spec.resolution)
