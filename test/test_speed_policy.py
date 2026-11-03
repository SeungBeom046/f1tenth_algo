import math
import unittest

from my_algo.speed_policy import (
    V_MIN, MinSpeedPolicy, b1_speed, check_v_imp, command_speed, v_cap,
)

# 계획서 가정값 (Exp0 실측 전)
P = dict(t_r=0.15, a_b=3.0, s_m=0.2, v_imp=0.5)


class VCapTest(unittest.TestCase):
    def test_minimum_at_s_m_is_about_022(self):
        self.assertAlmostEqual(v_cap(P['s_m'], **P), 0.2227, places=4)

    def test_constant_inside_s_m(self):
        # d <= s_m에서는 v_cap(s_m)으로 고정 (v_imp로 튀지 않음)
        for d in (0.0, 0.05, 0.1999):
            self.assertEqual(v_cap(d, **P), v_cap(P['s_m'], **P))
        self.assertNotAlmostEqual(v_cap(0.1, **P), P['v_imp'], places=2)

    def test_continuous_at_s_m(self):
        eps = 1e-9
        self.assertAlmostEqual(v_cap(P['s_m'] - eps, **P), v_cap(P['s_m'] + eps, **P), places=6)

    def test_monotonic(self):
        values = [v_cap(0.2 + 0.01 * i, **P) for i in range(300)]
        self.assertTrue(all(b > a for a, b in zip(values, values[1:])))

    def test_satisfies_stopping_condition(self):
        # v t_r + (v^2 - v_imp^2)/(2 a_b) = d - s_m
        for d in (0.3, 0.8, 1.125, 3.0):
            v = v_cap(d, **P)
            lhs = v * P['t_r'] + (v * v - P['v_imp'] ** 2) / (2 * P['a_b'])
            self.assertAlmostEqual(lhs, d - P['s_m'], places=9)

    def test_v_ref_upper_bound(self):
        policy = MinSpeedPolicy()
        for d in (0.5, 1.125, 5.0, None):
            raw, cmd, _ = command_speed(2.0, v_cap(d, **P), policy)
            self.assertLessEqual(raw, 2.0)
            self.assertLessEqual(cmd, 2.0)
        self.assertAlmostEqual(v_cap(1.125, **P), 2.0, places=3)

    def test_no_phantom_means_no_limit(self):
        self.assertTrue(math.isinf(v_cap(None, **P)))


class MinSpeedPolicyTest(unittest.TestCase):
    def test_v_min_from_vehicle_utils(self):
        self.assertAlmostEqual(V_MIN, 4000 / 4614)

    def test_hold_clamps_and_flags(self):
        p = MinSpeedPolicy('hold')
        self.assertEqual(p.apply(0.3), (V_MIN, 'clamped_to_vmin'))
        self.assertEqual(p.apply(0.0), (V_MIN, 'clamped_to_vmin'))
        self.assertEqual(p.apply(1.5), (1.5, ''))

    def test_hold_never_stops_so_no_deadlock(self):
        p = MinSpeedPolicy('hold')
        for d in (0.0, 0.2, 0.3, 0.41):
            cmd, _ = p.apply(v_cap(d, **P))
            self.assertGreater(cmd, 0.0)

    def test_hysteresis(self):
        p = MinSpeedPolicy('hysteresis', restart_margin=0.1)
        self.assertEqual(p.apply(1.0), (1.0, ''))
        self.assertEqual(p.apply(0.5), (0.0, 'stopped_below_vmin'))
        self.assertEqual(p.apply(V_MIN + 0.05), (0.0, 'stopped_below_vmin'))
        self.assertEqual(p.apply(V_MIN + 0.11)[1], '')

    def test_v_imp_lower_bound_warning(self):
        self.assertIsNotNone(check_v_imp(0.5))
        self.assertIsNone(check_v_imp(V_MIN))

    def test_v_imp_equal_v_min_keeps_impact_bound(self):
        # v_imp = v_min이면 정책 적용 후 명령 속도가 v_imp를 넘는 경우는 v_cap > v_imp인 거리뿐
        params = dict(P, v_imp=V_MIN)
        policy = MinSpeedPolicy('hold')
        for i in range(200):
            d = 0.01 * i
            vc = v_cap(d, **params)
            _, cmd, flag = command_speed(2.0, vc, policy)
            if flag:
                self.assertAlmostEqual(cmd, params['v_imp'])

    def test_b1_same_policy(self):
        p = MinSpeedPolicy('hold')
        self.assertEqual(b1_speed(0.0, 2.0, 1.0), 2.0)
        self.assertEqual(b1_speed(1.2, 2.0, 1.0), 0.0)
        self.assertEqual(command_speed(2.0, b1_speed(0.9, 2.0, 1.0), p)[1:], (V_MIN, 'clamped_to_vmin'))


if __name__ == '__main__':
    unittest.main()
