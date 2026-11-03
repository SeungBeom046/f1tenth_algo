import math
import unittest

import numpy as np

from my_algo.deskew import deskew_points


class DeskewTest(unittest.TestCase):
    def test_straight_motion_restores_wall(self):
        # 차가 0.1 s 동안 +x로 0.2 m 이동하며 x=2.0 (odom) 벽면 점들을 잰다.
        t = np.linspace(0.0, 0.1, 11)
        lidar_x = 2.0 * t                       # 2.0 m/s
        xyz = np.column_stack([2.0 - lidar_x, np.linspace(-0.5, 0.5, 11), np.zeros(11)])
        out = deskew_points(xyz, t, 0.0, (0.0, 0.0, 0.0), 0.1, (0.2, 0.0, 0.0), t_ref=0.1)
        # 기준 시각(끝)의 라이다는 x=0.2에 있으므로 벽은 모두 x=1.8로 보여야 한다
        self.assertTrue(np.allclose(out[:, 0], 1.8))
        self.assertTrue(np.allclose(out[:, 1], xyz[:, 1]))

    def test_rotation(self):
        t = np.array([0.0, 0.1])
        # 끝 시각 기준 yaw 0.1 rad 회전. 처음 잰 점 (1,0)은 기준 프레임에서 (cos, -sin) 방향
        xyz = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        out = deskew_points(xyz, t, 0.0, (0, 0, 0.0), 0.1, (0, 0, 0.1), t_ref=0.1)
        self.assertAlmostEqual(out[0, 0], math.cos(0.1))
        self.assertAlmostEqual(out[0, 1], -math.sin(0.1))
        self.assertTrue(np.allclose(out[1], [1.0, 0.0, 0.0]))

    def test_no_motion_identity(self):
        xyz = np.random.default_rng(0).normal(size=(50, 3))
        t = np.linspace(0, 0.1, 50)
        out = deskew_points(xyz, t, 0.0, (1.0, 2.0, 0.3), 0.1, (1.0, 2.0, 0.3), 0.1)
        self.assertTrue(np.allclose(out, xyz))


if __name__ == '__main__':
    unittest.main()
