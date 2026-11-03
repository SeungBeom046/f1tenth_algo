import math
import unittest

import numpy as np

from my_algo import metrics as M

FP = ((0.41, 0.135), (-0.09, 0.135), (-0.09, -0.135), (0.41, -0.135))


class LossTest(unittest.TestCase):
    def test_constant_vref_zero_loss(self):
        s = np.linspace(0, 8, 801)
        dt, floored = M.loss_delta_t(s, np.full_like(s, 2.0), 2.0, 2.5, 7.0)
        self.assertAlmostEqual(dt, 0.0, places=9)
        self.assertFalse(floored)

    def test_half_speed_section(self):
        # 1 m 구간을 1 m/s로 가면 2 m/s 대비 0.5 s 손실
        s = np.linspace(0, 8, 8001)
        v = np.where((s >= 4.0) & (s < 5.0), 1.0, 2.0)
        dt, _ = M.loss_delta_t(s, v, 2.0, 2.5, 7.0)
        self.assertAlmostEqual(dt, 0.5, delta=0.01)
        self.assertAlmostEqual(M.decel_length(s, v, 2.0, 2.5, 7.0), 1.0, delta=0.02)
        self.assertEqual(M.min_speed(s, v, 2.5, 7.0), 1.0)
        self.assertAlmostEqual(M.window_loss_pct(s, v, 2.0, 4.0, 5.0), 100 * 0.5 / 1.5, delta=1)

    def test_not_completed(self):
        s = np.linspace(0, 5, 100)
        self.assertEqual(M.loss_delta_t(s, np.full_like(s, 2.0), 2.0, 2.5, 7.0), (None, False))

    def test_series_on_s_uses_min_while_stopped(self):
        t_s = np.array([0, 1, 2, 3.0])
        s = np.array([0, 1, 1, 2.0])           # 1~2 s 정지
        su, vu = M.series_on_s(t_s, s, np.array([0.5, 1.2, 1.8, 2.5]), np.array([1, 0.3, 0.0, 1]))
        self.assertEqual(vu[np.argmin(np.abs(su - 1.0))], 0.0)

    def test_vref_reached(self):
        s = np.linspace(0, 3, 301)
        self.assertTrue(M.vref_reached(s, np.minimum(2.0, s), 2.5, 2.0))
        self.assertFalse(M.vref_reached(s, np.minimum(1.8, s), 2.5, 2.0))


class JerkTest(unittest.TestCase):
    def test_lowpass_removes_high_frequency(self):
        fs = 200.0
        t = np.arange(0, 4, 1 / fs)
        slow = np.sin(2 * math.pi * 0.5 * t)
        noisy = slow + 0.5 * np.sin(2 * math.pi * 40 * t)
        f = M.butter2_lowpass(noisy, fs, 5.0)
        self.assertLess(np.abs(f - slow)[100:-100].max(), 0.05)

    def test_jerk_of_ramp(self):
        t = np.arange(0, 2, 0.005)
        a = 3.0 * t                            # jerk 3 m/s^3
        self.assertAlmostEqual(M.jerk_rms(t, a, 0.5, 1.5), 3.0, delta=0.05)


class SafetyTest(unittest.TestCase):
    def test_footprint_distance(self):
        self.assertAlmostEqual(M.footprint_distance(np.array([[1.41, 0.0], [2, 2]]), FP), 1.0)
        self.assertTrue(math.isinf(M.footprint_distance(np.empty((0, 2)), FP)))

    def test_collision_and_ttc(self):
        t = np.linspace(0, 1, 101)
        dist = 1.0 - 1.0 * t                   # 1 m/s로 접근해 t=1에서 접촉
        r = M.safety_metrics(t, dist, np.full_like(t, 2.0))
        self.assertTrue(r['collision'])
        self.assertAlmostEqual(r['impact_speed'], 2.0)
        self.assertAlmostEqual(r['min_ttc'], 0.0, delta=0.02)
        r2 = M.safety_metrics(t, 1.0 - 0.5 * t, np.full_like(t, 1.0))
        self.assertFalse(r2['collision'])
        self.assertAlmostEqual(r2['min_distance'], 0.5)
        self.assertAlmostEqual(r2['min_ttc'], 1.0, delta=0.02)   # 0.5 m / 0.5 m/s

    def test_ci(self):
        mean, half, n = M.mean_ci95([1.0, 2.0, 3.0, None])
        self.assertEqual((mean, n), (2.0, 3))
        self.assertAlmostEqual(half, 4.303 * 1.0 / math.sqrt(3), places=3)


if __name__ == '__main__':
    unittest.main()
