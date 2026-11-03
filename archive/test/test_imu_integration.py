import math
import unittest
from my_algo.imu_integration import ImuIntegration


class IntegrationTest(unittest.TestCase):
    def engine(self):
        engine = ImuIntegration(10, 0.11)
        for _ in range(10):
            engine.calibrate((0., 0., 9.80665), (0., 0., 0.01))
        self.assertTrue(engine.ready)
        return engine

    def test_stationary_and_gyro_bias(self):
        e = self.engine()
        for i in range(101):
            e.update(i/10, (0., 0., 9.80665), (0., 0., 0.01))
        self.assertAlmostEqual(e.distance, 0.)
        self.assertAlmostEqual(e.speed, 0.)

    def test_constant_acceleration(self):
        e = self.engine()
        for i in range(11):
            e.update(i/10, (1., 0., 9.80665), (0., 0., 0.01))
        self.assertAlmostEqual(e.speed, 1.)
        self.assertAlmostEqual(e.distance, 0.5)

    def test_rotation_removes_gravity(self):
        e = self.engine()
        for i in range(11):
            angle = i*0.01
            e.update(i/10, (0., 9.80665*math.sin(angle), 9.80665*math.cos(angle)), (0.1, 0., 0.01))
        self.assertLess(e.speed, 1e-10)

    def test_gap_duplicate_and_nan(self):
        e = self.engine()
        e.update(1., (0., 0., 9.80665), (0., 0., 0.01))
        self.assertFalse(e.update(1., (0., 0., 9.80665), (0., 0., 0.01)))
        self.assertFalse(e.update(2., (0., 0., 9.80665), (0., 0., 0.01)))
        self.assertFalse(e.update(2.01, (math.nan, 0., 9.), (0., 0., 0.)))
        self.assertEqual(e.gaps, 2)
        self.assertEqual(e.distance, 0.)

    def test_reject_bad_units_and_moving_calibration(self):
        e = ImuIntegration(2)
        self.assertFalse(e.calibrate((0., 0., 1.), (0., 0., 0.)))
        self.assertFalse(e.calibrate((0., 0., 9.80665), (0., 0., 1.)))
        self.assertFalse(e.ready)
