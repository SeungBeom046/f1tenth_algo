import unittest

import numpy as np

from my_algo.pc2 import read_cloud, read_xyz, write_cloud


class Field:
    def __init__(self, name, offset, datatype, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class Cloud:
    def __init__(self):
        self.header = None
        self.fields = []
        self.is_bigendian = False
        self.point_step = self.row_step = self.width = 0
        self.height = 1
        self.is_dense = True
        self.data = b''


def livox_like(points):
    """livox_ros_driver2 PointCloud2(xfer_format 0)과 비슷한 배치: xyz f32, intensity f32, tag/line u8, timestamp f64."""
    dt = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
                   'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
                   'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26})
    arr = np.zeros(len(points), dt)
    for i, (x, y, z, t) in enumerate(points):
        arr[i] = (x, y, z, 1.0, 0, 0, t)
    c = Cloud()
    c.fields = [Field('x', 0, 7), Field('y', 4, 7), Field('z', 8, 7), Field('intensity', 12, 7),
                Field('tag', 16, 2), Field('line', 17, 2), Field('timestamp', 18, 8)]
    c.point_step, c.width = 26, len(points)
    c.row_step = 26 * len(points)
    c.data = arr.tobytes()
    return c


class Pc2Test(unittest.TestCase):
    def test_read_fields_and_skip_nan(self):
        c = livox_like([(1, 2, 3, 1e18), (float('nan'), 0, 0, 2e18), (4, 5, 6, 3e18)])
        pts = read_cloud(c)
        self.assertEqual(len(pts), 2)
        self.assertEqual(pts['timestamp'][1], 3e18)
        self.assertTrue(np.allclose(read_xyz(c), [[1, 2, 3], [4, 5, 6]]))

    def test_write_roundtrip(self):
        c = livox_like([(1, 2, 3, 10.0), (4, 5, 6, 20.0)])
        pts = read_cloud(c).copy()
        pts['x'] += 1.0
        out = write_cloud(c, pts)
        self.assertEqual(out.width, 2)
        back = read_cloud(out)
        self.assertTrue(np.allclose(back['x'], [2, 5]))
        self.assertTrue(np.allclose(back['timestamp'], [10, 20]))


if __name__ == '__main__':
    unittest.main()
