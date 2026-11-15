"""시뮬 단위 테스트 (sim/.venv에서 실행): sim/.venv/bin/python -m unittest discover -s sim/tests"""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROOT = os.path.dirname(SIM)
sys.path[:0] = [SIM, ROOT]

import mujoco  # noqa: E402

import lidar as lidar_mod  # noqa: E402
import simconfig  # noqa: E402
import vehicle  # noqa: E402
import world as world_mod  # noqa: E402
from my_algo.geometry import load_geometry  # noqa: E402
from my_algo.mux_logic import to_vesc  # noqa: E402
from my_algo.occupancy import UNKNOWN, GridSpec, OccupancyAccumulator  # noqa: E402
from my_algo.perception import process_frame  # noqa: E402
from my_algo.speed_policy import V_MIN  # noqa: E402

MOUNT = {'lidar_xyz': (0.13, -0.005, 0.24), 'lidar_rpy': (0.0, 0.0, 0.0)}
LCFG = {'rays_per_frame': 7200, 'pattern': 'grid', 'grid_azimuth': 720, 'vfov_min_deg': -7.0,
        'vfov_max_deg': 52.0, 'range_min': 0.1, 'range_max': 40.0, 'range_noise_std': 0.0}


def model(xml_body):
    xml = f'''<mujoco><worldbody><geom type="plane" size="20 20 .1"/>{xml_body}
      <body name="ego" mocap="true" pos="0 0 0"><geom type="box" pos="0.16 0 0.1" size="0.25 0.135 0.1" group="1"/></body>
      </worldbody></mujoco>'''
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    return m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'ego')


class LidarTest(unittest.TestCase):
    def test_box_distance_matches_analytic(self):
        # 벽 앞면 x = 3.0 (두께 0.1, 높이 1). 라이다 원점 x = 0.13 → 수평 광선 거리 2.87
        m, d, ego = model('<geom type="box" pos="3.05 0 0.5" size="0.05 2 0.5"/>')
        lid = lidar_mod.Lidar(m, LCFG, MOUNT, ego)
        pts, world_pts, origin = lid.scan(d, (0.0, 0.0, 0.0))
        self.assertTrue(np.allclose(origin, [0.13, -0.005, 0.24]))
        el = np.degrees(np.arctan2(pts[:, 2], np.hypot(pts[:, 0], pts[:, 1])))
        az = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        front = (np.abs(az) < 0.3) & (np.abs(el) < 2.0)
        self.assertTrue(front.any())
        r = np.linalg.norm(pts[front], axis=1)
        expected = 2.87 / np.cos(np.radians(el[front])) / np.cos(np.radians(az[front]))
        self.assertLess(np.abs(r - expected).max(), 1e-6)
        self.assertLess(np.abs(world_pts[front, 0] - 3.0).max(), 1e-6)

    def test_scan_matches_pointcloud_to_laserscan_rule(self):
        p2l = {'min_height': -0.1, 'max_height': 0.5, 'angle_min': -3.14159, 'angle_max': 3.14159,
               'angle_increment': 0.00436, 'range_min': 0.05, 'range_max': 20.0, 'use_inf': True}
        xyz = np.array([[2.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 0.0, 0.6], [0.0, 1.0, 0.0]])
        s = lidar_mod.cloud_to_scan(xyz, p2l)
        i0 = int((0.0 - p2l['angle_min']) / p2l['angle_increment'])
        self.assertAlmostEqual(float(s['ranges'][i0]), 1.0, places=5)   # 같은 bin은 최소, 높이 밖(0.6) 제외
        self.assertTrue(np.isinf(s['ranges']).any())


class OcclusionBandTest(unittest.TestCase):
    def test_beams_over_box_do_not_clear_occlusion(self):
        """박스(0.35 m) 위로 넘어간 빔이 박스 뒤 칸을 free로 만들지 않는다 (높이 대역 0.05–0.30)."""
        cfg = simconfig.RunConfig('P', 'S2')
        w, eh = world_mod.build(cfg)
        geom = load_geometry(cfg.geometry_path)
        m = mujoco.MjModel.from_xml_string(w.xml.replace('{EGO_GEOM}', world_mod.ego_geom_xml(geom.footprint, eh)))
        d = mujoco.MjData(m)
        ego = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'ego')
        d.mocap_pos[m.body_mocapid[ego]] = [-0.41, 0, 0]
        mujoco.mj_kinematics(m, d)
        lid = lidar_mod.Lidar(m, dict(LCFG, rays_per_frame=40000, grid_azimuth=1440), MOUNT, ego)
        pts, world_pts, _ = lid.scan(d, (-0.41, 0.0, 0.0))
        p1 = w.layout.boxes[0]
        behind = (world_pts[:, 0] > p1[0] + 0.1) & (world_pts[:, 0] < p1[1] - 0.1) & (world_pts[:, 1] < -0.75)
        self.assertTrue((behind & (world_pts[:, 2] > 0.30)).any(), '박스 뒤에 닿은 대역 밖 점이 있어야 의미 있는 테스트')
        spec = GridSpec.from_bounds(-3, 15, -3, 3, 0.05)
        T_base_lidar = np.eye(4)
        T_base_lidar[:3, 3] = MOUNT['lidar_xyz']
        T_odom_lidar = T_base_lidar.copy()
        T_odom_lidar[0, 3] -= 0.41
        res = process_frame(OccupancyAccumulator(spec, 1, 1, 1), spec, pts, T_odom_lidar, T_base_lidar,
                            (0.05, 0.30), 8.0, debug_oob=True)
        cx = 0.5 * (p1[0] + p1[1])
        col = int((cx - spec.origin_x) / spec.resolution)
        row = int((-1.0 - spec.origin_y) / spec.resolution)
        self.assertEqual(res.grid[row, col], UNKNOWN)
        self.assertGreater(res.debug['oob_would_free_cells'], 0)


class VehicleTest(unittest.TestCase):
    def test_stopping_distance(self):
        t_r, a_b, v0 = 0.15, 3.0, 2.0
        car = vehicle.Kinematic(0, 0, 0, 0.33, t_r, 1e-4, 10.0, a_b, 100.0)
        car.v, car.rpm_now = v0, v0 * car.erpm_gain
        dt = 0.0005
        car.command(0.0, 0.0, car.servo_center)
        t = 0.0
        while car.v > 0 or t < t_r:
            t += dt
            car.step(t, dt)
        self.assertAlmostEqual(car.x, v0 * t_r + v0 ** 2 / (2 * a_b), delta=0.005)

    def test_min_erpm_applied(self):
        rpm, servo, final = to_vesc(0.3, 0.0, 0.78)
        car = vehicle.Kinematic(0, 0, 0, 0.33, 0.0, 1e-4, 10.0, 3.0, 100.0)
        car.command(0.0, rpm, servo)
        for k in range(200):
            car.step((k + 1) * 0.01, 0.01)
        self.assertEqual(rpm, 4000.0)
        self.assertAlmostEqual(car.v, V_MIN, places=6)


if __name__ == '__main__':
    unittest.main()
