"""바닥 평면 RANSAC 적합과 라이다 장착 높이/roll/pitch 계산 (ROS 비의존).

scripts/measure_lidar_height.py가 사용한다.

각도 규약: tf2 static_transform_publisher와 같은 R = Rz(yaw) Ry(pitch) Rx(roll),
R은 livox_frame 벡터를 base_link 축으로 돌리는 회전. 바닥이 평평하고 base_link z축이
바닥 법선과 같다고 가정하면, 라이다 프레임에서 본 바닥 법선 n으로부터
    n = (-sin p, sin r cos p, cos r cos p)
    pitch = -asin(n_x),  roll = atan2(n_y, n_z)
yaw는 바닥만으로는 알 수 없다 (벽 평행 검사로 따로 확인).
"""

import math

import numpy as np


def fit_plane_ransac(points, threshold=0.01, iterations=500, seed=0):
    """points (N,3)에서 평면을 찾는다. 반환: (unit normal, d, inlier mask), n.p + d = 0."""
    rng = np.random.default_rng(seed)
    n_pts = len(points)
    if n_pts < 3:
        raise ValueError('평면 적합에 점이 3개 이상 필요')
    best_mask, best_count = None, -1
    for _ in range(iterations):
        p0, p1, p2 = points[rng.choice(n_pts, 3, replace=False)]
        normal = np.cross(p1 - p0, p2 - p0)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        d = -normal.dot(p0)
        mask = np.abs(points @ normal + d) < threshold
        count = int(mask.sum())
        if count > best_count:
            best_mask, best_count = mask, count
    # inlier 전체로 최소제곱 재적합 (SVD)
    inliers = points[best_mask]
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid)
    normal = vt[-1]
    d = -normal.dot(centroid)
    # 라이다 원점이 바닥 위에 있으므로 원점에서의 부호 거리 d가 양수가 되도록 법선을 위로 향하게 한다.
    if d < 0:
        normal, d = -normal, -d
    mask = np.abs(points @ normal + d) < threshold
    return normal, d, mask


def mount_from_floor(normal):
    """라이다 프레임 바닥 법선 -> (roll, pitch) [rad]."""
    n = normal / np.linalg.norm(normal)
    pitch = -math.asin(max(-1.0, min(1.0, float(n[0]))))
    roll = math.atan2(float(n[1]), float(n[2]))
    return roll, pitch


def rotation_rp(roll, pitch):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return ry @ rx


def measure(points, threshold=0.01, iterations=500):
    """반환 dict: height(라이다 원점의 바닥 위 높이), roll, pitch, 보정 후 바닥점 z 평균/표준편차."""
    normal, d, mask = fit_plane_ransac(points, threshold, iterations)
    roll, pitch = mount_from_floor(normal)
    floor = points[mask] @ rotation_rp(roll, pitch).T
    return {
        'height_m': float(d),
        'roll_rad': roll,
        'pitch_rad': pitch,
        'floor_z_mean_m': float(floor[:, 2].mean()),
        'floor_z_std_m': float(floor[:, 2].std()),
        'inliers': int(mask.sum()),
        'points': int(len(points)),
    }
