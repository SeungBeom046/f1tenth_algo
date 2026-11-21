"""[1 인지] D455 깊이 카메라 → 라이다 /scan과 같은 각도 격자의 장애물 거리 + YOLO 상자 → 앞차 위치. rclpy 없음.

지도·학습 없이 기하만 쓴다 (실차 노드 my_algo/camera_fusion.py, 시뮬 sim/aur_overtake, 테스트가 같이 쓴다).
- 좌표: optical(x 오른쪽, y 아래, z 앞) → body(x 앞, y 왼쪽, z 위, 카메라 원점) → level(바닥과 나란, 카메라 원점)
  → 라이다 수평 프레임(livox_level, /scan 원점) = level + 장착 위치 (mount_x, mount_y), yaw 보정.
- 기울기: 바닥 평면을 RANSAC으로 맞춰 pitch(+ = 앞이 숙여짐)·roll·카메라 높이를 잰다. 숙인 각도를 손으로 바꿔도
  시작할 때 다시 잰다 (실차 장착 15~20°, 2026-11-21).
- 장애물: 바닥 위 [min_height, max_height] 점만 남긴다 (라이다 /scan 높이 띠와 같은 생각). 각도 bin마다 최소 수평 거리,
  bin 안 점이 min_points 미만이면 버린다 (바닥 깊이 잡음 한두 점이 장애물로 보이지 않게).
- 왜 카메라인가: 라이다(바닥에서 0.24 m)는 높이 0.25 m 검은 앞차의 위 모서리만 스쳐 폭 0.02~0.11 m로 보였다
  (실차 test_01·overtake_02, 2026-11-19). 카메라는 더 낮고 아래를 봐서 뒷면 전체·바퀴·낮은 가드레일(덕트)이 보인다.
"""
import math
from collections import namedtuple

import numpy as np

Calib = namedtuple('Calib', 'pitch roll height')        # [rad], [rad], 바닥 → 카메라 [m]


def deproject(depth, K, stride=4, scale=0.001):
    """깊이 영상 → optical 점 (N,3)과 화소 (u, v). depth: (H,W) uint16 [mm] 또는 float [m] (scale 1.0).
    K = (fx, fy, cx, cy). stride마다 한 화소 (640×480, 4 → 160×120 = 19200점)."""
    fx, fy, cx, cy = K
    d = np.asarray(depth)[::stride, ::stride]
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = d.astype(np.float64) * scale
    ok = np.isfinite(z) & (z > 0.0)
    z, u, v = z[ok], u[ok], v[ok]
    return np.column_stack([(u - cx) / fx * z, (v - cy) / fy * z, z]), u, v


def to_body(opt):
    """optical → body (x 앞, y 왼쪽, z 위)."""
    return np.column_stack([opt[:, 2], -opt[:, 0], -opt[:, 1]])


def rotation(pitch, roll):
    """body → level. pitch + = 앞이 숙여짐 (sim rot_rpy, 라이다 lidar_pitch와 같은 부호)."""
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return Rx @ Ry


def fit_floor(body, rng=None, iters=80, tol=0.015, max_points=3000, max_tilt_deg=45.0):
    """body 점에서 바닥 평면 → (Calib, 인라이어 비율). 못 찾으면 None.
    평면 법선 n (body) = (−sinθ·cosφ, sinφ, cosθ·cosφ), 바닥 점은 n·p + h = 0."""
    rng = np.random.default_rng(0) if rng is None else rng
    pts = body[body[:, 2] < -0.02]          # 카메라보다 아래 점만 (바닥 후보)
    if len(pts) < 50:
        return None
    if len(pts) > max_points:
        pts = pts[rng.choice(len(pts), max_points, replace=False)]
    tri = pts[rng.integers(0, len(pts), (iters, 3))]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    norm = np.linalg.norm(n, axis=1)
    good = norm > 1e-9
    n = n[good] / norm[good, None]
    n *= np.where(n[:, 2] < 0.0, -1.0, 1.0)[:, None]
    h = -np.einsum('ij,ij->i', n, tri[good, 0])
    good = (n[:, 2] > math.cos(math.radians(max_tilt_deg))) & (h > 0.02)
    if not good.any():
        return None
    n, h = n[good], h[good]
    counts = (np.abs(pts @ n.T + h) < tol).sum(axis=0)
    k = int(np.argmax(counts))
    inl = pts[np.abs(pts @ n[k] + h[k]) < tol]
    if len(inl) < 30:
        return None
    c = inl.mean(axis=0)
    nn = np.linalg.svd(inl - c, full_matrices=False)[2][-1]
    nn = -nn if nn[2] < 0.0 else nn
    calib = Calib(math.atan2(-nn[0], nn[2]), math.asin(max(-1.0, min(1.0, nn[1]))), float(-nn @ c))
    return calib, len(inl) / float(len(pts))


def to_lidar(body, calib, mount):
    """body 점 → (X, Y, 바닥 위 높이). mount = (x, y, yaw): 라이다 수평 프레임에서 카메라 위치 [m]·yaw [rad]."""
    q = body @ rotation(calib.pitch, calib.roll).T
    c, s = math.cos(mount[2]), math.sin(mount[2])
    return (c * q[:, 0] - s * q[:, 1] + mount[0], s * q[:, 0] + c * q[:, 1] + mount[1], q[:, 2] + calib.height)


def obstacle_ranges(body, calib, mount, angle_min, angle_increment, n_bins,
                    min_height=0.05, max_height=0.5, range_min=0.3, range_max=3.0, min_points=3):
    """장애물 점 → /scan과 같은 격자의 거리 배열 (없는 bin = inf). 반환: (ranges, 장애물 점 수)."""
    X, Y, H = to_lidar(body, calib, mount)
    r = np.hypot(X, Y)
    a = np.arctan2(Y, X)
    sel = (H >= min_height) & (H <= max_height) & (r >= range_min) & (r <= range_max) & (a >= angle_min)
    idx = ((a[sel] - angle_min) / angle_increment).astype(int)
    keep = idx < n_bins
    idx, rs = idx[keep], r[sel][keep]
    ranges = np.full(n_bins, np.inf)
    np.minimum.at(ranges, idx, rs)
    ranges[np.bincount(idx, minlength=n_bins) < min_points] = np.inf
    return ranges, int(len(idx))


def fuse_ranges(lidar, l_min, l_inc, cam, c_min, c_inc):
    """라이다 scan에 카메라 거리를 bin마다 최솟값으로 넣는다 (각도 격자가 달라도 된다). 반환: np 배열."""
    out = np.array(lidar, dtype=np.float64)
    cam = np.asarray(cam, dtype=np.float64)
    k = np.nonzero(np.isfinite(cam))[0]
    idx = np.floor((c_min + (k + 0.5) * c_inc - l_min) / l_inc).astype(int)
    ok = (idx >= 0) & (idx < len(out))
    np.minimum.at(out, idx[ok], cam[k[ok]])
    return out


def lead_from_box(body, u, v, calib, mount, box, min_height=0.05, max_height=0.5, range_min=0.3,
                  depth_window=0.55, shrink=0.1, min_points=8):
    """YOLO 상자 (u1, v1, u2, v2) [화소] 안 장애물 점 → 앞차 뒷면 {'r': 수평 거리, 'a_lo', 'a_hi': 방위 [rad]}.
    상자 가장자리 shrink 비율은 버리고(옆 레일), 가장 가까운 면에서 depth_window 안 점만 차로 본다(뒤 배경). 없으면 None."""
    u1, v1, u2, v2 = box
    du, dv = shrink * (u2 - u1), shrink * (v2 - v1)
    X, Y, H = to_lidar(body, calib, mount)
    r = np.hypot(X, Y)
    sel = ((u >= u1 + du) & (u <= u2 - du) & (v >= v1 + dv) & (v <= v2 - dv)
           & (H >= min_height) & (H <= max_height) & (r >= range_min))
    if sel.sum() < min_points:
        return None
    r, a = r[sel], np.arctan2(Y[sel], X[sel])
    r0 = float(np.percentile(r, 5))
    near = r <= r0 + depth_window
    if near.sum() < min_points:
        return None
    return {'r': r0, 'a_lo': float(a[near].min()), 'a_hi': float(a[near].max())}
