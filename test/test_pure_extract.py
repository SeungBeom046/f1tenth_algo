"""exp 노드에서 분리한 순수 함수: trigger_math, mux_logic, perception."""
import unittest

import numpy as np

import _track
from my_algo.mux_logic import MuxConfig, MuxInputs, MuxState, tick, to_vesc
from my_algo.occupancy import OCCUPIED, UNKNOWN, GridSpec, OccupancyAccumulator
from my_algo.perception import cropped, decode_grid, process_frame
from my_algo.speed_policy import V_MIN
from my_algo.trigger_math import trigger_distance
from my_algo.vehicle_utils import ERPM_GAIN, MIN_DRIVING_RPM


def inputs(**kw):
    base = dict(now=1.0, auto_mode=True, joy_active=False, preflight_ok=True, last_joy=0.9, aeb=False,
                v_cmd=1.5, v_time=0.95, steer=0.1, steer_time=0.95, travel=1.0)
    base.update(kw)
    return MuxInputs(**base)


class TriggerMathTest(unittest.TestCase):
    def test_formula(self):
        self.assertAlmostEqual(trigger_distance(4.775, 1.5, 2.0, 0.05, 0.2), 4.775 - 1.5 - 0.5)


class MuxLogicTest(unittest.TestCase):
    def test_normal(self):
        st = MuxState()
        out = tick(MuxConfig(), st, inputs())
        self.assertEqual((out.send, out.speed, out.steering, out.reason), (True, 1.5, 0.1, ''))
        self.assertTrue(st.has_driven)

    def test_speed_step_for_exp0_i(self):
        cfg = MuxConfig(step_x=2.5, step_v=1.2)
        before = tick(cfg, MuxState(), inputs(v_cmd=2.0, travel=2.4))
        after = tick(cfg, MuxState(), inputs(v_cmd=2.0, travel=2.6))
        self.assertEqual((before.speed, before.reason), (2.0, ''))
        self.assertEqual((after.speed, after.reason), (1.2, 'speed_step'))
        # 기본(끔)은 영향 없음, 정지선이 계단보다 우선
        self.assertEqual(tick(MuxConfig(), MuxState(), inputs(v_cmd=2.0, travel=2.6)).speed, 2.0)
        self.assertEqual(tick(MuxConfig(stop_distance=2.0, step_x=1.0, step_v=1.2), MuxState(),
                              inputs(v_cmd=2.0, travel=2.6)).reason, 'stop_line')

    def test_manual_sends_once_then_silent(self):
        st = MuxState()
        tick(MuxConfig(), st, inputs())
        self.assertEqual(tick(MuxConfig(), st, inputs(joy_active=True)).reason, 'manual_or_auto_off')
        self.assertFalse(tick(MuxConfig(), st, inputs(joy_active=True)).send)

    def test_priorities_and_latch(self):
        cfg, st = MuxConfig(stop_distance=5.0), MuxState()
        out = tick(cfg, st, inputs(aeb=True))
        self.assertEqual((out.reason, out.aborts), ('aeb', ['aeb']))
        out = tick(cfg, st, inputs(aeb=False))
        self.assertEqual((out.speed, out.reason, out.aborts), (0.0, 'aeb', []))      # 래치 유지
        out = tick(cfg, st, inputs(last_joy=0.2))                                   # joy_timeout은 덮어씀
        self.assertEqual((out.reason, out.aborts), ('joy_timeout', ['joy_timeout']))

    def test_stop_line_and_stale(self):
        self.assertEqual(tick(MuxConfig(stop_distance=0.5), MuxState(), inputs()).reason, 'stop_line')
        self.assertEqual(tick(MuxConfig(), MuxState(), inputs(v_time=0.5)).reason, 'stale_speed')
        out = tick(MuxConfig(), MuxState(), inputs(steer_time=0.5))
        self.assertEqual((out.reason, out.steering), ('stale_steering', 0.0))

    def test_preflight(self):
        st = MuxState()
        self.assertEqual(tick(MuxConfig(), st, inputs(preflight_ok=False)).reason, 'preflight_failed')
        tick(MuxConfig(), st, inputs())
        self.assertEqual(tick(MuxConfig(), st, inputs(preflight_ok=False)).reason, 'foreign_publisher')

    def test_to_vesc_min_rpm(self):
        rpm, servo, final = to_vesc(0.3, 2.0, 0.78)
        self.assertEqual(rpm, MIN_DRIVING_RPM)                  # 0.3 m/s → 최소 4000 ERPM
        self.assertAlmostEqual(final, V_MIN)
        self.assertAlmostEqual(servo, 0.5 - 0.78 * 0.6)          # 조향 제한 후 서보
        self.assertEqual(to_vesc(0.0, 0.0, 0.78)[0], 0.0)
        self.assertAlmostEqual(to_vesc(2.0, 0.0, 0.78)[0], 2.0 * ERPM_GAIN)


class PerceptionTest(unittest.TestCase):
    def test_process_frame_band_and_crop(self):
        spec = GridSpec.from_bounds(-2, 10, -2, 2, 0.05)
        acc = OccupancyAccumulator(spec, 1, 1, 1)
        T_base_lidar = np.eye(4)
        T_base_lidar[:3, 3] = [0.13, 0.0, 0.24]
        hits = _track.scan((0.13, 0.0), _track.segments(gap=0.45))
        # 모두 대역 안(base z 0.2): 라이다 z = -0.04
        xyz = np.column_stack([hits[:, 0] - 0.13, hits[:, 1], np.full(len(hits), -0.04)])
        res = process_frame(acc, spec, xyz, T_base_lidar, T_base_lidar, (0.05, 0.30), 8.0, debug_oob=True)
        self.assertEqual(res.debug['in_band'], len(hits))
        self.assertTrue((res.grid == OCCUPIED).any() and res.boundary.any())
        # 같은 점을 대역 밖(base z 0.5)으로 올리면 레이캐스팅에 안 쓰여 전부 UNKNOWN
        acc2 = OccupancyAccumulator(spec, 1, 1, 1)
        xyz[:, 2] = 0.26
        res2 = process_frame(acc2, spec, xyz, T_base_lidar, T_base_lidar, (0.05, 0.30), 8.0)
        self.assertEqual(res2.debug['in_band'], 0)
        self.assertTrue((res2.grid == UNKNOWN).all())
        sub, sub_spec = cropped(res.grid, spec, (0.0, 0.0), 3.0, 1.0)
        back, back_spec = decode_grid(sub.reshape(-1).tolist(), sub_spec.width, sub_spec.height,
                                      sub_spec.origin_x, sub_spec.origin_y, sub_spec.resolution)
        self.assertTrue(np.array_equal(back, sub))
        self.assertEqual(back_spec, sub_spec)


if __name__ == '__main__':
    unittest.main()
