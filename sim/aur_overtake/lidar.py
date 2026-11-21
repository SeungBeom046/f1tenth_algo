# 출처: sim/mujoco 브랜치의 sim/lidar.py (47121b9). 내용은 그대로 가져왔다.
"""Livox Mid-360 근사 라이다 (MuJoCo mj_multiRay).

- 장착: base_link → livox_frame (launch/tf_static.launch.py 값, simconfig.mount()).
- 광선: 수평 360°, 수직 vfov_min~vfov_max. pattern=nonrepetitive이면 프레임마다 방위·고도를 무작위
  (시드 고정, 고도는 구면 균등), grid이면 고정 격자.
- 반환: livox_frame 기준 (N, 3) 점 (occupancy 노드가 받는 형식). 맞지 않은 광선·최소 거리 미만은 버린다.
- 2D scan: pointcloud_to_laserscan과 같은 규칙 (target_frame=livox_frame, min/max_height 안 점,
  각도 bin마다 최소 수평 거리, 범위 밖·없는 bin은 inf).
"""
import math

import mujoco
import numpy as np

from world import SENSE_GROUP

# 판정용 geom 그룹만 맞힌다 (도로·가드레일·바퀴 같은 보기용 모양은 scan에 안 들어간다)
RAY_GROUPS = np.array([1 if g == SENSE_GROUP else 0 for g in range(6)], np.uint8)


def rot_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Lidar:
    def __init__(self, model, lidar_cfg, mount, exclude_body, seed=0):
        self.m = model
        self.c = lidar_cfg
        self.offset = np.array(mount['lidar_xyz'], float)
        self.R_mount = rot_rpy(*mount['lidar_rpy'])
        self.exclude = exclude_body
        self.rng = np.random.default_rng(seed)
        self.n = int(lidar_cfg['rays_per_frame'])
        lo, hi = math.radians(lidar_cfg['vfov_min_deg']), math.radians(lidar_cfg['vfov_max_deg'])
        self.el_range = (lo, hi)
        if lidar_cfg['pattern'] == 'grid':
            n_az = int(lidar_cfg['grid_azimuth'])
            n_el = max(1, self.n // n_az)
            az, el = np.meshgrid(np.linspace(-math.pi, math.pi, n_az, endpoint=False),
                                 np.linspace(lo, hi, n_el))
            self.fixed = (az.ravel(), el.ravel())
            self.n = len(self.fixed[0])
        else:
            self.fixed = None
        self.geomid = np.zeros(self.n, np.int32)
        self.dist = np.zeros(self.n)

    def directions(self):
        if self.fixed is not None:
            az, el = self.fixed
        else:
            az = self.rng.uniform(-math.pi, math.pi, self.n)
            s0, s1 = math.sin(self.el_range[0]), math.sin(self.el_range[1])
            el = np.arcsin(self.rng.uniform(s0, s1, self.n))      # 구면 균등
        ce = np.cos(el)
        return np.column_stack([ce * np.cos(az), ce * np.sin(az), np.sin(el)])   # livox_frame

    def scan(self, data, base_pose):
        """base_pose = (x, y, yaw) 월드. 반환: livox_frame 점 (N,3), 월드 점 (N,3), 라이다 월드 원점."""
        x, y, yaw = base_pose
        R_base = rot_rpy(0.0, 0.0, yaw)
        origin = np.array([x, y, 0.0]) + R_base @ self.offset
        R = R_base @ self.R_mount
        d_lidar = self.directions()
        d_world = d_lidar @ R.T
        mujoco.mj_multiRay(self.m, data, origin, np.ascontiguousarray(d_world).ravel(), RAY_GROUPS, 1,
                           self.exclude, self.geomid, self.dist, None, self.n, float(self.c['range_max']))
        dist = self.dist.copy()
        hit = (dist >= self.c['range_min']) & (dist <= self.c['range_max']) & (self.geomid >= 0)
        self.last_geom = self.geomid[hit].copy()     # 맞힌 geom (검은 앞차 라이다 반사 모사에 쓴다)
        if self.c['range_noise_std'] > 0:
            dist[hit] += self.rng.normal(0.0, self.c['range_noise_std'], int(hit.sum()))
        pts_lidar = d_lidar[hit] * dist[hit, None]
        pts_world = origin + pts_lidar @ R.T
        return pts_lidar, pts_world, origin


class DepthCamera:
    """D455 깊이 카메라 근사: 화소 격자 광선 → 깊이 영상(optical z) [m], 0 = 깊이 없음.

    실차 노드는 640×480을 stride 4로 쓰므로 시뮬은 처음부터 160×120 (같은 화소 수). 잡음 σ = depth_noise·z².
    장착 mount_xyz = 뒤축 기준, pitch_deg + = 아래를 봄. 지연은 모사하지 않는다 (scan과 같은 시각).
    """

    def __init__(self, model, cam_cfg, exclude_body, seed=0):
        from my_algo.aur import camera
        self.m, self.c, self.exclude = model, cam_cfg, exclude_body
        W, H = int(cam_cfg['width']), int(cam_cfg['height'])
        fx = 0.5 * W / math.tan(math.radians(cam_cfg['hfov_deg']) / 2)
        fy = 0.5 * H / math.tan(math.radians(cam_cfg['vfov_deg']) / 2)
        self.K = (fx, fy, 0.5 * W - 0.5, 0.5 * H - 0.5)
        self.shape = (H, W)
        v, u = np.mgrid[0:H, 0:W]
        d_opt = np.stack([(u - self.K[2]) / fx, (v - self.K[3]) / fy, np.ones((H, W))], -1).reshape(-1, 3)
        self.norm = np.linalg.norm(d_opt, axis=1)
        self.R = camera.rotation(math.radians(cam_cfg['pitch_deg']), 0.0)
        self.d_level = (camera.to_body(d_opt) / self.norm[:, None]) @ self.R.T
        self.offset = np.array(cam_cfg['mount_xyz'], float)
        self.n = len(self.d_level)
        self.geomid = np.zeros(self.n, np.int32)
        self.dist = np.zeros(self.n)
        self.rng = np.random.default_rng(seed)

    def origin(self, base_pose):
        x, y, yaw = base_pose
        return np.array([x, y, 0.0]) + rot_rpy(0.0, 0.0, yaw) @ self.offset

    def depth(self, data, base_pose, drop_geom=-1, drop_p=0.0):
        """깊이 영상 (H, W). drop_geom을 맞힌 화소는 drop_p 확률로 깊이 없음 (검은 차체에서 스테레오 실패)."""
        R_yaw = rot_rpy(0.0, 0.0, base_pose[2])
        d_world = self.d_level @ R_yaw.T
        mujoco.mj_multiRay(self.m, data, self.origin(base_pose), np.ascontiguousarray(d_world).ravel(), RAY_GROUPS, 1,
                           self.exclude, self.geomid, self.dist, None, self.n, float(self.c['max_depth']))
        z = self.dist / self.norm
        ok = (self.geomid >= 0) & (self.dist > 0) & (z <= self.c['max_depth'])
        if drop_p > 0:
            ok &= ~((self.geomid == drop_geom) & (self.rng.random(self.n) < drop_p))
        z = np.where(ok, z, 0.0)
        if self.c['depth_noise'] > 0:
            z = np.where(ok, z + self.rng.normal(0.0, 1.0, self.n) * self.c['depth_noise'] * z ** 2, 0.0)
        return z.reshape(self.shape)

    def project_box(self, base_pose, corners_world):
        """월드 꼭짓점들 → 화소 상자 (u1, v1, u2, v2), 카메라 앞이 아니거나 화면 밖이면 None (시뮬 YOLO 상자)."""
        q = (corners_world - self.origin(base_pose)) @ rot_rpy(0.0, 0.0, base_pose[2])   # level
        q = q @ self.R                                                                      # body
        if (q[:, 0] < 0.1).any():
            return None
        u = self.K[2] - q[:, 1] / q[:, 0] * self.K[0]
        v = self.K[3] - q[:, 2] / q[:, 0] * self.K[1]
        H, W = self.shape
        u1, u2, v1, v2 = max(u.min(), 0), min(u.max(), W - 1), max(v.min(), 0), min(v.max(), H - 1)
        return (u1, v1, u2, v2) if u2 - u1 > 2 and v2 - v1 > 2 else None


def cloud_to_scan(xyz, p2l):
    """pointcloud_to_laserscan 규칙 (target_frame = 점군 프레임). 반환 dict: angle_min, angle_increment,
    range_min, range_max, ranges (np.float32)."""
    a_min, a_max, inc = p2l['angle_min'], p2l['angle_max'], p2l['angle_increment']
    n = int(math.ceil((a_max - a_min) / inc))
    ranges = np.full(n, np.inf, dtype=np.float64)
    z = xyz[:, 2]
    sel = (z >= p2l['min_height']) & (z <= p2l['max_height'])
    r = np.hypot(xyz[sel, 0], xyz[sel, 1])
    a = np.arctan2(xyz[sel, 1], xyz[sel, 0])
    ok = (r >= p2l['range_min']) & (r <= p2l['range_max']) & (a >= a_min) & (a <= a_max)
    idx = ((a[ok] - a_min) / inc).astype(int)
    idx = np.clip(idx, 0, n - 1)
    np.minimum.at(ranges, idx, r[ok])
    if not p2l.get('use_inf', True):
        ranges[np.isinf(ranges)] = p2l['range_max'] + p2l.get('inf_epsilon', 1.0)
    return {'angle_min': a_min, 'angle_max': a_max, 'angle_increment': inc,
            'range_min': p2l['range_min'], 'range_max': p2l['range_max'], 'ranges': ranges.astype(np.float32)}
