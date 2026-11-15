import math
import unittest

import numpy as np

import _track
from my_algo.geometry import VehicleGeometry, point_polygon_distance
from my_algo.occupancy import (
    FREE, OCCUPIED, UNKNOWN, GridSpec, OccupancyAccumulator, height_band_mask,
    occlusion_boundary, out_of_band_free_ratio, raycast, single_frame_grid,
)
from my_algo.phantom import PhantomHold, PhantomParams, b1_roi_half_width_auto, make_strategy, nearest_d
from my_algo.speed_policy import v_cap

SPEC = GridSpec.from_bounds(-2.0, 10.0, -2.0, 2.0, 0.05)
LIDAR = (0.13, 0.0)          # base_link 원점 = 뒤축, 에고 시작 위치
POSE = (0.0, 0.0, 0.0)
FRONT = 0.41


def build_grid(segs, frames=3, origin=LIDAR):
    acc = OccupancyAccumulator(SPEC, n_frames=frames, known_on=1, known_off=1)
    hits = _track.scan(origin, segs)
    for _ in range(frames):
        acc.add_frame(*raycast(SPEC, origin, hits))
    return acc.grid()


def cell_value(grid, x, y):
    col = int((x - SPEC.origin_x) / SPEC.resolution)
    row = int((y - SPEC.origin_y) / SPEC.resolution)
    return grid[row, col]


class OccupancyTest(unittest.TestCase):
    def test_height_band(self):
        z = np.array([0.0, 0.05, 0.2, 0.30, 0.31, 0.6])
        self.assertEqual(height_band_mask(z, 0.05, 0.30).tolist(),
                         [False, True, True, True, False, False])

    def test_raycast_free_and_occupied(self):
        free, occ = raycast(SPEC, (0.0, 0.0), np.array([[2.0, 0.0]]))
        g = single_frame_grid(SPEC, free, occ)
        self.assertEqual(cell_value(g, 1.0, 0.01), FREE)
        self.assertEqual(cell_value(g, 2.01, 0.01), OCCUPIED)
        self.assertEqual(cell_value(g, 3.0, 0.01), UNKNOWN)

    def test_box_hides_space_behind(self):
        grid = build_grid(_track.segments(gap=0.45))
        # 라이다에서 볼 때 박스 P1 뒤 (광선이 P1을 지나야 닿는 곳)는 보이지 않아야 한다
        self.assertEqual(cell_value(grid, 4.2, -1.0), UNKNOWN)
        face = [cell_value(grid, x, y) for x in np.arange(3.0, 3.55, 0.05) for y in (-0.44, -0.46)]
        self.assertGreater(face.count(OCCUPIED), 3)                  # P1 차선 쪽 면
        self.assertEqual(cell_value(grid, 3.3, -0.6), UNKNOWN)       # P1 내부
        self.assertEqual(cell_value(grid, 3.3, 0.0), FREE)
        self.assertTrue(occlusion_boundary(grid).any())

    def test_hysteresis(self):
        acc = OccupancyAccumulator(SPEC, n_frames=3, known_on=2, known_off=1)
        f, o = raycast(SPEC, (0.0, 0.0), np.array([[1.0, 0.0]]))
        acc.add_frame(f, o)
        self.assertEqual(cell_value(acc.grid(), 0.5, 0.01), UNKNOWN)   # 1회 관측: 아직 unknown
        acc.add_frame(f, o)
        self.assertEqual(cell_value(acc.grid(), 0.5, 0.01), FREE)      # 2회: known
        acc.add_frame(np.empty(0, np.int64), np.empty(0, np.int64))
        acc.add_frame(np.empty(0, np.int64), np.empty(0, np.int64))
        self.assertEqual(cell_value(acc.grid(), 0.5, 0.01), FREE)      # 관측 1회 남음: off 기준 유지
        acc.add_frame(np.empty(0, np.int64), np.empty(0, np.int64))
        self.assertEqual(cell_value(acc.grid(), 0.5, 0.01), UNKNOWN)

    def test_out_of_band_points_would_clear_occlusion(self):
        segs = _track.segments(gap=0.45)
        grid = build_grid(segs)
        # 박스 위를 넘어 뒤 바닥에 떨어진 점을 흉내: 박스 뒤 y=-1.2 지점
        oob = np.array([[4.2, -1.0], [5.0, -1.1]])
        band_free, _ = raycast(SPEC, LIDAR, _track.scan(LIDAR, segs))
        ratio, n = out_of_band_free_ratio(SPEC, LIDAR, band_free, grid, oob)
        self.assertGreater(n, 0)
        self.assertGreater(ratio, 0.0)


class PhantomTest(unittest.TestCase):
    def params(self, **kw):
        return PhantomParams(front=FRONT, **kw)

    def run_method(self, method, segs, **kw):
        grid = build_grid(segs)
        return make_strategy(method, self.params(**kw)).compute(grid, SPEC, POSE)

    def test_b0_no_phantom(self):
        r = self.run_method('B0', _track.segments())
        self.assertTrue(math.isinf(r.d))
        self.assertEqual(len(r.phantoms_base), 0)

    def test_p_gap_entrance_when_passable(self):
        r = self.run_method('P', _track.segments(gap=0.45))
        # 비스듬한 면의 점 간격 때문에 w_min보다 좁은 가짜 틈은 생길 수 있다. 통과 가능한 틈은 2개.
        right_gaps = [g for g in r.gaps if g[0] == 'right' and g[2] >= 0.35]
        self.assertEqual(len(right_gaps), 2)
        # 가까운 틈(약 3.6 m)은 폭이 정확해야 한다. 먼 틈은 점 간격만큼 넓게 잡힐 수 있다.
        self.assertAlmostEqual(right_gaps[0][2], 0.45, delta=0.06)
        xs = sorted(r.phantoms_base[:, 0])
        # P1-P2 틈 입구 x = 3.55 + 0.225 = 3.775
        self.assertTrue(any(abs(x - 3.775) < 0.06 for x in xs))
        self.assertAlmostEqual(r.d, 3.0 - FRONT, delta=0.06)  # 가장 가까운 것은 P1 시작 끝

    def test_p_no_gap_phantom_when_too_narrow(self):
        r = self.run_method('P', _track.segments(gap=0.15))
        xs = r.phantoms_base[:, 0]
        for gx in (3.55 + 0.075, 4.25 + 0.075):
            self.assertFalse(any(abs(x - gx) < 0.06 for x in xs))
        # 주차열 양 끝 phantom은 남는다
        self.assertTrue(any(abs(x - 3.0) < 0.06 for x in xs))

    def test_p_hidden_car_does_not_bridge_gap(self):
        # 틈 안 0.15 m 들어간 곳에 숨은 차: 열로 묶이지 않아 틈 입구 phantom이 유지되어야 한다
        r = self.run_method('P', _track.segments(gap=0.45, hidden_car=0.15))
        self.assertTrue(any(abs(x - 3.775) < 0.06 for x in r.phantoms_base[:, 0]))

    def test_b2_more_phantoms_than_p(self):
        segs = _track.segments(gap=0.15)
        b2 = self.run_method('B2', segs)
        p = self.run_method('P', segs)
        self.assertGreater(len(b2.phantoms_base), len(p.phantoms_base))
        self.assertLessEqual(b2.d, p.d + 1e-9)

    def test_b1_rho_higher_with_continuous_wall(self):
        boxes = self.run_method('B1', _track.segments(gap=0.45), b1_roi_half_width=1.2)
        wall = self.run_method('B1', _track.segments(right_wall=True), b1_roi_half_width=1.2)
        self.assertGreater(wall.rho, boxes.rho)
        self.assertTrue(math.isinf(wall.d))

    def test_phantom_behind_front_ignored(self):
        r = self.run_method('P', _track.segments(first_box_x=0.0))
        self.assertGreaterEqual(r.d, 0.0)

    def test_same_grid_for_all_methods(self):
        grid = build_grid(_track.segments())
        before = grid.copy()
        for m in ('B0', 'B1', 'B2', 'P'):
            make_strategy(m, self.params()).compute(grid, SPEC, POSE)
        self.assertTrue(np.array_equal(grid, before))  # 전략은 격자를 바꾸지 않는다


class PhantomRuleTest(unittest.TestCase):
    """phantom 유지(공통), P 앞쪽 열만, P w_max, B1 ROI 반폭 식."""

    def params(self, **kw):
        return PhantomParams(front=FRONT, **kw)

    def frame(self, method, segs, ego_x):
        grid = build_grid(segs, origin=(LIDAR[0] + ego_x, LIDAR[1]))
        pose = (ego_x, 0.0, 0.0)
        return make_strategy(method, self.params()).compute(grid, SPEC, pose), pose

    def test_hold_keeps_phantom_when_car_fills_gap(self):
        cap = dict(t_r=0.25, a_b=3.0, s_m=0.2, v_imp=0.867)
        for method in ('P', 'B2'):
            hold = PhantomHold(FRONT, 0.10, SPEC.resolution)
            # 1) #2가 틈 안 깊이(면에서 0.30 m) 숨어 있음: 틈 입구 phantom이 있다
            r1, pose1 = self.frame(method, _track.segments(gap=0.45, hidden_car=0.30), 0.0)
            held1 = hold.update(r1.phantoms_base, pose1)
            d1 = nearest_d(held1, FRONT)
            # 2) 에고가 0.2 m 나아가고 #2 앞 끝이 박스 면까지 나와 틈을 채움
            r2, pose2 = self.frame(method, _track.segments(gap=0.45, hidden_car=0.0), 0.2)
            gap_x = 3.0 + 0.55 + 0.225 - 0.2          # P1–P2 틈 입구 (base_link, 에고 0.2 m 전진 후)
            if method == 'P':
                # 유지 규칙이 없으면 틈이 메워져 입구 phantom이 사라진다 (지키려는 경우가 실제로 생김)
                self.assertFalse(any(abs(x - gap_x) < 0.06 for x in r2.phantoms_base[:, 0]))
                self.assertTrue(any(abs(x - (gap_x + 0.2)) < 0.06 for x in r1.phantoms_base[:, 0]))
            held2 = hold.update(r2.phantoms_base, pose2)
            self.assertTrue(any(abs(x - gap_x) < 0.06 for x in held2[:, 0]), method)
            d2 = nearest_d(held2, FRONT)
            self.assertLessEqual(d2, d1 - 0.2 + 1e-6, method)                  # odom 고정: 전진한 만큼 줄어든다
            self.assertLessEqual(v_cap(d2, **cap), v_cap(d1, **cap) + 1e-9, method)   # v_cap이 풀리지 않음

    def test_hold_drops_after_front_passes_margin(self):
        hold = PhantomHold(FRONT, 0.10, SPEC.resolution)
        hold.update(np.array([[2.0, -0.4]]), (0.0, 0.0, 0.0))                 # odom x = 2.0
        self.assertEqual(len(hold.update(np.empty((0, 2)), (2.0 - FRONT + 0.05, 0.0, 0.0))), 1)  # 앞 끝 2.05
        self.assertEqual(len(hold.update(np.empty((0, 2)), (2.0 - FRONT + 0.11, 0.0, 0.0))), 0)  # 앞 끝 2.11

    def test_p_ignores_structures_behind_front(self):
        # 출발점 뒤 오른쪽에 박스(뒤 벽 같은 구조물). 예전에는 그 박스 ~ P1 사이를 틈으로 보고 중간에 phantom을 뒀다
        segs = _track.segments(gap=0.45)
        y0, y1 = _track.BOX_INNER_Y, _track.BOX_INNER_Y - _track.BOX_DEPTH
        segs += [(-1.0, y0, -0.4, y0), (-1.0, y1, -0.4, y1), (-0.4, y0, -0.4, y1)]
        r, _ = self.frame('P', segs, 0.0)
        right = r.phantoms_base[r.phantoms_base[:, 1] < 0]
        self.assertFalse(any(0.5 < x < 2.9 for x in right[:, 0]))
        self.assertAlmostEqual(r.d, 3.0 - FRONT, delta=0.06)           # 가장 가까운 것은 P1 시작 끝

    def test_p_wide_space_is_row_end_not_gap(self):
        r, _ = self.frame('P', _track.segments(gap=1.2, n_boxes=2), 0.0)
        xs = r.phantoms_base[r.phantoms_base[:, 1] < 0][:, 0]
        a1, b0 = 3.0 + 0.55, 3.0 + 0.55 + 1.2
        self.assertFalse(any(abs(x - 0.5 * (a1 + b0)) < 0.1 for x in xs))     # 중간(틈) phantom 없음
        self.assertTrue(any(abs(x - a1) < 0.06 for x in xs))                  # 앞 열 끝
        self.assertTrue(any(abs(x - b0) < 0.08 for x in xs))                  # 뒤 열 시작
        # 0.45 틈은 그대로 틈 (w_max 0.50 이하)
        r, _ = self.frame('P', _track.segments(gap=0.45), 0.0)
        self.assertTrue(any(abs(x - 3.775) < 0.06 for x in r.phantoms_base[:, 0]))

    def test_b1_roi_half_width_formula(self):
        self.assertAlmostEqual(b1_roi_half_width_auto(0.7, 0.8, 0.15, 2.0, 3.0), 0.35 + 0.8 * (0.15 + 2.0 / 3.0))
        self.assertAlmostEqual(b1_roi_half_width_auto(0.7, 0.8, 0.25, 2.0, 3.0), 0.35 + 0.8 * (0.25 + 2.0 / 3.0))


class GeometryTest(unittest.TestCase):
    def test_footprint_distance(self):
        fp = ((0.41, 0.135), (-0.09, 0.135), (-0.09, -0.135), (0.41, -0.135))
        self.assertEqual(point_polygon_distance(0.0, 0.0, fp), 0.0)
        self.assertAlmostEqual(point_polygon_distance(1.41, 0.0, fp), 1.0)
        self.assertAlmostEqual(point_polygon_distance(0.0, 0.335, fp), 0.2)
        g = VehicleGeometry(0.33, 0.41, 0.09, 0.27, fp, 0.28)
        self.assertAlmostEqual(g.half_width, 0.135)


if __name__ == '__main__':
    unittest.main()
