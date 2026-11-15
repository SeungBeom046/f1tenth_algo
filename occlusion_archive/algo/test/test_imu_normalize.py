import math
import unittest

from my_algo.imu_normalize import G0, ImuNormalizer, quat_to_matrix


def calibrated(**kwargs):
    n = ImuNormalizer(calibration_samples=5, **kwargs)
    for _ in range(5):
        a, g = n.to_si((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
        n.feed_calibration(a, g)
    return n


class ImuNormalizeTest(unittest.TestCase):
    def test_g_unit_converted_to_mps2(self):
        n = ImuNormalizer(accel_unit='g')
        a, _ = n.to_si((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(a[2], G0)

    def test_gyro_bias_subtracted(self):
        n = ImuNormalizer(gyro_bias=(0.01, -0.02, 0.03))
        _, g = n.to_si((0, 0, 1), (0.01, -0.02, 0.03))
        self.assertTrue(all(abs(x) < 1e-12 for x in g))

    def test_wrong_unit_never_calibrates(self):
        # 실제로 g 단위인데 mps2로 설정하면 |a|=1 이라 정지 보정이 거부되어야 한다.
        n = ImuNormalizer(accel_unit='mps2', calibration_samples=3)
        for _ in range(10):
            a, g = n.to_si((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
            n.feed_calibration(a, g)
        self.assertFalse(n.ready)

    def test_rest_gives_zero_linear_accel(self):
        n = calibrated()
        a, _ = n.to_si((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
        self.assertTrue(all(abs(x) < 1e-9 for x in n.linear_accel_imu(a)))

    def test_tilted_mount_forward_accel(self):
        # 2도 pitch로 장착된 IMU: 정지 g_ref에 기울기가 들어가므로 전진 가속만 남아야 한다.
        p = math.radians(2.0)
        n = ImuNormalizer(calibration_samples=5)
        rest = (-G0 * math.sin(p), 0.0, G0 * math.cos(p))
        for _ in range(5):
            n.feed_calibration(rest, (0.0, 0.0, 0.0))
        # 차체 +x로 2 m/s^2 가속 -> IMU 프레임으로 투영
        accel = (rest[0] + 2.0 * math.cos(p), 0.0, rest[2] + 2.0 * math.sin(p))
        lin = n.linear_accel_imu(accel)
        self.assertAlmostEqual(math.hypot(lin[0], lin[2]), 2.0, places=9)

    def test_pitch_change_tracked_by_gyro(self):
        # 정지 보정 후 제동으로 1도 pitch 변화: 자이로 추적이 없으면 0.17 m/s^2 오차.
        n = calibrated(accel_unit='g')
        pitch = math.radians(1.0)
        rate = pitch / 0.1
        n.update_attitude(0.0, (0.0, rate, 0.0))
        n.update_attitude(0.1, (0.0, rate, 0.0))
        # 오른손 법칙으로 y축 +pitch 후 정지 상태 비력이 IMU 프레임에서 보이는 값
        accel = (-G0 * math.sin(pitch), 0.0, G0 * math.cos(pitch))
        lin = n.linear_accel_imu(accel)
        self.assertLess(max(abs(x) for x in lin), 1e-6)

    def test_rotation_to_base(self):
        # yaw 90도: IMU x -> base y
        q = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
        v = ImuNormalizer.to_base(quat_to_matrix(q), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(v[0], 0.0)
        self.assertAlmostEqual(v[1], 1.0)


if __name__ == '__main__':
    unittest.main()
