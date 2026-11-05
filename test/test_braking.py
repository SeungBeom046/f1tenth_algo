import unittest

import numpy as np

from my_algo.braking import braking_metrics, equivalent_delay, tracking_lag


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
        self.assertAlmostEqual(r['t_r_equiv'], 0.12, delta=0.02)

    def test_equivalent_delay(self):
        self.assertAlmostEqual(equivalent_delay(2.0, 2.0 * 0.25 + 4 / 6, 3.0), 0.25, places=9)
        self.assertIsNone(equivalent_delay(2.0, None, 3.0))

    def lagged(self, t_d, tau, a_b, cmd):
        """순수 지연 t_d + 1차 지연 tau + 감속 한계 a_b 계 (sim/vehicle.Kinematic과 같은 식)."""
        dt = 0.001
        t = np.arange(0, 4, dt)
        u = np.interp(t - t_d, t, cmd(t), left=cmd(t)[0])
        v = np.empty_like(t)
        v[0] = u[0]
        for k in range(1, len(t)):
            dv = (u[k] - v[k - 1]) * dt / tau
            v[k] = v[k - 1] + max(-a_b * dt, min(2.5 * dt, dv))
        return t, cmd(t), v

    def test_tracking_lag_equals_dead_time_plus_tau(self):
        # 기울기 -1.5 m/s^2 명령 (a_b 3.0보다 완만): 등가 지연 = t_d + tau (과도 응답 포함해 모델로 맞춤)
        ramp = lambda t: np.clip(2.0 - 1.5 * (t - 1.0), 0.9, 2.0)  # noqa: E731
        for t_d, tau in ((0.15, 0.10), (0.05, 0.0), (0.10, 0.20)):
            t, u, v = self.lagged(t_d, max(tau, 1e-3), 3.0, ramp)
            r = tracking_lag(t, u, t, v, 3.0)
            self.assertGreater(r['n'], 50)
            self.assertAlmostEqual(r['lag'], t_d + tau, delta=0.006, msg=(t_d, tau, r))

    def test_step_braking_hides_tau(self):
        # 계단 0 명령: 감속이 a_b로 바로 포화 → 등가 지연은 t_d + tau보다 훨씬 작다 (그래서 추종 시험이 필요)
        step = lambda t: np.where(t < 1.0, 2.0, 0.0)  # noqa: E731
        t, u, v = self.lagged(0.15, 0.10, 3.0, step)
        s = np.concatenate([[0], np.cumsum(v[:-1] * 0.001)])
        k0 = int(1.0 / 0.001)
        stop = s[np.flatnonzero((t > 1.0) & (v < 0.01))[0]] - s[k0]
        self.assertLess(equivalent_delay(2.0, stop, 3.0), 0.19)


if __name__ == '__main__':
    unittest.main()
