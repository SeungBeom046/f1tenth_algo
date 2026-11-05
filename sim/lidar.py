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
        mujoco.mj_multiRay(self.m, data, origin, np.ascontiguousarray(d_world).ravel(), None, 1,
                           self.exclude, self.geomid, self.dist, None, self.n, float(self.c['range_max']))
        dist = self.dist.copy()
        hit = (dist >= self.c['range_min']) & (dist <= self.c['range_max']) & (self.geomid >= 0)
        if self.c['range_noise_std'] > 0:
            dist[hit] += self.rng.normal(0.0, self.c['range_noise_std'], int(hit.sum()))
        pts_lidar = d_lidar[hit] * dist[hit, None]
        pts_world = origin + pts_lidar @ R.T
        return pts_lidar, pts_world, origin


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
