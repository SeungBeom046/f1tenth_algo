import math
import random
import re
import unittest

from my_algo.aeb import AEBSafetyModel
from my_algo.aeb_event import closest_in_corridor
from my_algo.vehicle_utils import ReactiveDriveCommand


class Scan:
    pass


class AebEventTest(unittest.TestCase):
    def test_same_closest_as_aeb_model(self):
        rng = random.Random(0)
        model = AEBSafetyModel(lidar_to_bumper_dist=0.28, vehicle_half_width=0.135, path_margin=0.15)
        for _ in range(200):
            s = Scan()
            s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / 720
            s.range_min, s.range_max = 0.05, 20.0
            s.ranges = [rng.choice([float('inf'), float('nan'), rng.uniform(0.0, 6.0)]) for _ in range(720)]
            model.check_emergency_braking(s, 2.0, ReactiveDriveCommand())
            pt = closest_in_corridor(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                     s.range_max, 0.28, 0.135, 0.15, 0.10)
            m = re.search(r'closest=([0-9.]+)m', model.last_reason)
            if pt is None:
                self.assertEqual(model.last_reason, 'clear')
            else:
                self.assertAlmostEqual(pt['x_front'], float(m.group(1)), places=2)


if __name__ == '__main__':
    unittest.main()
