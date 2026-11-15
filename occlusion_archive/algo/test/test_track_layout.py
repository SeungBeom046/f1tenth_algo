import os
import unittest

from my_algo.track_layout import check, from_params, layout
from test_config_consistency import load_params

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


class TrackLayoutTest(unittest.TestCase):
    def test_s1_numbers(self):
        lay = layout(4.0, 0.55, 0.45, 3)
        self.assertAlmostEqual(lay.row_x_end, 6.55)
        self.assertAlmostEqual(lay.collision_x, 4.775)
        self.assertEqual(len(lay.boxes), 3)
        self.assertAlmostEqual(lay.boxes[1][0], 5.0)

    def test_s3_and_empty(self):
        self.assertAlmostEqual(layout(4.0, 0.55, 0.15, 3).row_x_end, 5.95)
        e = layout(0.0, 0.55, 0.0, 0)
        self.assertEqual((e.row_x_end, e.collision_x, e.boxes), (0.0, None, ()))

    def test_check_warns(self):
        p = dict(row_x_start=4.0, box_len=0.55, gap_width=0.45, n_boxes=3, row_x_end=6.3, collision_x=5.0)
        self.assertEqual(len(check(p)), 2)

    def test_repo_scenarios_match_layout(self):
        for s in ('S1', 'S2', 'S3', 'S4', 'E0'):
            p = load_params(os.path.join(ROOT, 'config', f'scenario_{s}.yaml'))['/**']
            self.assertEqual(check(p), [], s)
            self.assertLess(from_params(p).row_x_end, float(p['stop_distance']), s)


if __name__ == '__main__':
    unittest.main()
