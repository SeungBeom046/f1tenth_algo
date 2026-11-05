"""phantom 배치 전략 (ROS 비의존). 네 방법의 차이는 이 파일의 전략 클래스에만 있다.

파이프라인 위치: 점유격자·가림 경계 → (이 모듈) → 속도 계층(speed_policy)
공통 입력: 누적 격자(odom), 에고 자세(odom), 차체 앞 끝 거리. 공통 출력: PhantomResult.

d 정의: 차체 앞 끝에서 phantom의 에고 경로상 진입 지점까지의 종방향 거리.
    에고 경로는 현재 진행 방향(base_link +x) 직선으로 보고, 진입 지점의 x = phantom의 base_link x.
    d = x - rear_axle_to_front. d < 0 (앞 끝보다 뒤)인 phantom은 무시한다.

전략:
    B0  공백 무시. phantom 없음.
    B1  전방 ROI 미관측 칸 비율 rho만 계산 (속도는 speed_policy.b1_speed).
    B2  ROI 안 모든 가림 경계 칸에 phantom.
    P   측면 주차열(앞 끝보다 앞쪽 occupied 군집) 사이 틈 폭 w_min–w_max인 곳의 입구와 주차열 양 끝에만 phantom.
        w_max보다 넓은 빈 공간은 열 끝으로 본다.

공통 유지 규칙 (PhantomHold, B2·P 같음): 한 번 생긴 phantom은 odom에 고정해 앞 끝이 x + 여유를 지날 때까지 유지.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from my_algo.occupancy import OCCUPIED, UNKNOWN, occlusion_boundary

METHODS = ('B0', 'B1', 'B2', 'P')


def b1_roi_half_width_auto(lane_width, object_speed, t_r, v_ref, a_b):
    """B1 ROI 반폭 = 차선 반폭 + 물체 속도 · (t_r + v_ref / a_b).

    에고가 v_ref에서 반응(t_r)해 멈출 때까지(v_ref / a_b) 걸리는 시간 동안 차선 밖 물체가 옆으로 올 수 있는 거리를
    차선 반폭에 더한다 = 에고가 정지하기 전에 물체가 차선에 도달할 수 있는 범위. 값은 config/common.yaml.
    """
    return 0.5 * float(lane_width) + float(object_speed) * (float(t_r) + float(v_ref) / float(a_b))


@dataclass
class PhantomParams:
    """모든 방법이 같은 값을 쓰는 ROI와 방법별 파라미터. 값은 config/common.yaml."""
    front: float                 # rear_axle_to_front (vehicle_geometry.yaml) [EXP-FIXED]
    roi_x_max: float = 6.0       # phantom 탐색 앞쪽 한계, base_link x [m] [EXP-FIXED]
    roi_half_y: float = 1.5      # phantom 탐색 좌우 한계 |y| [m] [EXP-FIXED]
    b1_roi_length: float = 3.0   # B1 ROI: 앞 끝부터 길이 [m] [EXP-TUNE]
    b1_roi_half_width: float = b1_roi_half_width_auto(0.7, 0.8, 0.15, 2.0, 3.0)  # [EXP-TUNE] 근거: common.yaml
    w_min: float = 0.35          # P 최소 틈 폭 [m] [EXP-TUNE]
    w_max: float = 0.50          # P 최대 틈 폭 [m] [EXP-TUNE]. 넘으면 열 끝
    row_y_min: float = 0.30      # P 주차열로 볼 occupied 칸의 |y| 하한 (차선 안 물체 제외) [m]
    row_y_max: float = 1.20      # P 주차열 |y| 상한 [m]
    row_link_dist: float = 0.10  # P 같은 군집으로 묶는 x 간격 상한 [m]
    row_end_margin: float = 0.5  # P ROI 앞쪽 경계에서 이 거리 안에서 끝나는 열은 시야 밖으로 이어진 것으로 봄 [m]
    row_face_tol: float = 0.10   # [EXP-TUNE] P 주차열 차선 쪽 면에서 이 깊이 안의 칸만 열로 묶음 [m].
                                 #   틈 안에 숨은 물체(면보다 더 들어가 있음)가 두 박스를 잇지 않게 한다
    p_lateral_filter: float = 0.0  # P 선택 필터: |y| > 이 값인 phantom 제외 (0 = 끔)


@dataclass
class PhantomResult:
    method: str
    phantoms_base: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    d: float = math.inf          # 최근접 phantom 거리 (없으면 inf)
    rho: float = 0.0             # B1 ROI 미관측 비율 (모든 방법에서 계산해 기록)
    rows: list = field(default_factory=list)  # P 디버그: (side, x_start, x_end, y_inner)
    gaps: list = field(default_factory=list)  # P 디버그: (side, x_mid, width)


def to_base(xy_odom, pose):
    """odom 좌표 (N,2) → base_link 좌표. pose = (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = xy_odom[:, 0] - x, xy_odom[:, 1] - y
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


def to_odom(xy_base, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * xy_base[:, 0] - s * xy_base[:, 1],
                            y + s * xy_base[:, 0] + c * xy_base[:, 1]])


def cells_base(mask, spec, pose):
    idx = np.flatnonzero(mask.reshape(-1))
    if len(idx) == 0:
        return np.empty((0, 2))
    return to_base(spec.cell_centers(idx), pose)


def nearest_d(phantoms_base, front):
    if len(phantoms_base) == 0:
        return math.inf
    ahead = phantoms_base[:, 0] - front
    ahead = ahead[ahead >= 0.0]
    return float(ahead.min()) if len(ahead) else math.inf


def roi_rho(grid, spec, pose, params):
    """B1 ROI(앞 끝부터 b1_roi_length, 반폭 b1_roi_half_width) 안 UNKNOWN 칸 비율."""
    allc = cells_base(np.ones_like(grid, dtype=bool), spec, pose)
    flat = grid.reshape(-1)
    inside = ((allc[:, 0] >= params.front) & (allc[:, 0] <= params.front + params.b1_roi_length)
              & (np.abs(allc[:, 1]) <= params.b1_roi_half_width))
    n = int(inside.sum())
    return float(np.count_nonzero(flat[inside] == UNKNOWN)) / n if n else 0.0


def in_roi(xy, params):
    return (xy[:, 0] <= params.roi_x_max) & (np.abs(xy[:, 1]) <= params.roi_half_y)


class Strategy:
    name = ''

    def __init__(self, params):
        self.params = params

    def compute(self, grid, spec, pose):
        res = PhantomResult(self.name)
        res.rho = roi_rho(grid, spec, pose, self.params)
        self.place(res, grid, spec, pose)
        res.d = nearest_d(res.phantoms_base, self.params.front)
        return res

    def place(self, res, grid, spec, pose):
        raise NotImplementedError


class B0(Strategy):
    name = 'B0'

    def place(self, res, grid, spec, pose):
        pass


class B1(Strategy):
    name = 'B1'

    def place(self, res, grid, spec, pose):
        pass  # rho만 쓴다


class B2(Strategy):
    name = 'B2'

    def place(self, res, grid, spec, pose):
        xy = cells_base(occlusion_boundary(grid), spec, pose)
        res.phantoms_base = xy[in_roi(xy, self.params)]


class P(Strategy):
    name = 'P'

    def place(self, res, grid, spec, pose):
        prm = self.params
        occ = cells_base(grid == OCCUPIED, spec, pose)
        # 주차열은 차체 앞 끝보다 앞쪽 칸으로만 만든다 (출발점 뒤 벽 등 뒤쪽 구조물을 열로 잡지 않게)
        occ = occ[in_roi(occ, prm) & (occ[:, 0] > prm.front)]
        phantoms = []
        half_cell = 0.5 * spec.resolution
        for side, sign in (('left', 1.0), ('right', -1.0)):
            side_y = sign * occ[:, 1]
            band = occ[(side_y >= prm.row_y_min) & (side_y <= prm.row_y_max)]
            if len(band) == 0:
                continue
            # 열의 차선 쪽 면 = 측면 occupied 칸 |y|의 하위 10% 분위. 그 면에서 row_face_tol 안쪽만 쓴다.
            face = float(np.quantile(sign * band[:, 1], 0.1))
            band = band[sign * band[:, 1] <= face + prm.row_face_tol]
            clusters = self.clusters(band, prm.row_link_dist)
            for x0, x1, y_in in clusters:
                res.rows.append((side, x0, x1, y_in))
            for (a0, a1, ay), (b0, b1, by) in zip(clusters, clusters[1:]):
                width = (b0 - half_cell) - (a1 + half_cell)
                res.gaps.append((side, 0.5 * (a1 + b0), width))
                if width > prm.w_max:
                    # 틈이 아니라 열이 끊긴 것: 앞 열 끝과 뒤 열 시작에 phantom (열 끝 규칙과 같음)
                    phantoms.append((a1 + half_cell, ay))
                    phantoms.append((b0 - half_cell, by))
                elif width >= prm.w_min:
                    phantoms.append((0.5 * (a1 + b0), 0.5 * (ay + by)))
            # 주차열 양 끝. ROI 경계에 닿은 끝은 열이 시야 밖으로 이어지는 것이라 끝으로 보지 않는다.
            # 시작 끝도 같다: 첫 칸이 앞 끝(열을 자른 곳) 바로 앞이면 열이 차 옆으로 이어지는 것이라 끝으로 보지 않는다.
            first, last = clusters[0], clusters[-1]
            if first[0] - half_cell > prm.front + prm.row_link_dist:
                phantoms.append((first[0] - half_cell, first[2]))
            if last[1] < prm.roi_x_max - prm.row_end_margin:
                phantoms.append((last[1] + half_cell, last[2]))
        xy = np.array(phantoms, dtype=float).reshape(-1, 2)
        if prm.p_lateral_filter > 0.0 and len(xy):
            xy = xy[np.abs(xy[:, 1]) <= prm.p_lateral_filter]
        res.phantoms_base = xy

    @staticmethod
    def clusters(band, link):
        """x 방향으로 정렬해 간격 > link인 곳에서 끊는다. (x_start, x_end, 차선 쪽 y) 목록."""
        order = np.argsort(band[:, 0])
        xs, ys = band[order, 0], band[order, 1]
        cuts = np.flatnonzero(np.diff(xs) > link) + 1
        out = []
        for seg_x, seg_y in zip(np.split(xs, cuts), np.split(ys, cuts)):
            inner = seg_y[np.argmin(np.abs(seg_y))]  # 차선에 가장 가까운 면
            out.append((float(seg_x[0]), float(seg_x[-1]), float(inner)))
        return out


STRATEGIES = {'B0': B0, 'B1': B1, 'B2': B2, 'P': P}


class PhantomHold:
    """phantom 유지 (모든 방법 공통 규칙, config/common.yaml phantom_hold·phantom_hold_margin).

    매 프레임 새 phantom을 odom 좌표로 바꿔 기억에 더하고(격자 칸 단위로 중복 제거), 차체 앞 끝이
    phantom x + margin을 지난 것(base_link x < front - margin)만 지운다. 반환은 기억 전체의 base_link 좌표.
    """

    def __init__(self, front, margin, cell):
        self.front, self.margin, self.cell = float(front), float(margin), float(cell)
        self.mem = {}

    def update(self, phantoms_base, pose):
        if len(phantoms_base):
            for x, y in to_odom(np.asarray(phantoms_base, float), pose):
                self.mem[(int(round(x / self.cell)), int(round(y / self.cell)))] = (float(x), float(y))
        if not self.mem:
            return np.empty((0, 2))
        keys = list(self.mem)
        base = to_base(np.array([self.mem[k] for k in keys]), pose)
        keep = base[:, 0] >= self.front - self.margin
        for k, ok in zip(keys, keep):
            if not ok:
                del self.mem[k]
        return base[keep]


def make_strategy(method, params):
    if method not in STRATEGIES:
        raise ValueError(f'method must be one of {METHODS}, got {method!r}')
    return STRATEGIES[method](params)
