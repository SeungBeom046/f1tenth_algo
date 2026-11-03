import math
import unittest

import numpy as np

from my_algo.floor_plane import measure, rotation_rp


class FloorPlaneTest(unittest.TestCase):
    def test_recovers_height_roll_pitch(self):
        rng = np.random.default_rng(1)
        roll, pitch, height = math.radians(1.5), math.radians(-2.0), 0.24
        # base_link 축(바닥 z=-height 기준)의 바닥점 + 장애물점
        floor = np.column_stack([rng.uniform(0.5, 4, 3000), rng.uniform(-1.5, 1.5, 3000),
                                 np.full(3000, -height) + rng.normal(0, 0.003, 3000)])
        box = np.column_stack([rng.uniform(1, 1.3, 500), rng.uniform(-0.8, -0.5, 500),
                               rng.uniform(-height, 0.35 - height, 500)])
        pts_base = np.vstack([floor, box])
        # 라이다 프레임으로 변환: p_lidar = R^T p_base
        pts_lidar = pts_base @ rotation_rp(roll, pitch)
        r = measure(pts_lidar)
        self.assertAlmostEqual(r['height_m'], height, delta=0.002)
        self.assertAlmostEqual(r['roll_rad'], roll, delta=math.radians(0.1))
        self.assertAlmostEqual(r['pitch_rad'], pitch, delta=math.radians(0.1))
        self.assertAlmostEqual(r['floor_z_mean_m'], -height, delta=0.002)


if __name__ == '__main__':
    unittest.main()
