import unittest

import numpy as np

from my_algo.braking import braking_metrics


class BrakingTest(unittest.TestCase):
    def test_constant_decel_with_delay(self):
        # 2.0 m/s, t=1.0에 0 명령, 0.12 s 뒤부터 3.0 m/s^2로 감속
        t = np.arange(0, 3, 0.005)
        onset = 1.12
        v = np.where(t < onset, 2.0, np.maximum(0.0, 2.0 - 3.0 * (t - onset)))
        s = np.concatenate([[0], np.cumsum(v[:-1] * 0.005)])
        a = np.where((t >= onset) & (v > 0), -3.0, 0.0)
        r = braking_metrics(1.0, t, v, s, t, a)
        self.assertAlmostEqual(r['a_b'], 3.0, delta=0.05)
        self.assertAlmostEqual(r['t_r'], 0.12, delta=0.03)
        # 정지거리 = 2.0*0.12 + 2^2/(2*3)
        self.assertAlmostEqual(r['stop_distance'], 0.24 + 4 / 6, delta=0.03)


if __name__ == '__main__':
    unittest.main()
